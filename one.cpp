#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit/planning_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>

using rclcpp::Node;

struct ObjSize { double x=0.05, y=0.05, z=0.02; }; // meters by default
static constexpr double kMMtoM = 1.0/1000.0;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Node>("pick_lift_obj1");

  // ---------------- Params ----------------
  node->declare_parameter<std::string>("base_frame", "base");     // or base_link
  node->declare_parameter<std::string>("obj_frame",  "obj_1");
  node->declare_parameter<std::string>("ee_link",    "");         // optional override
  node->declare_parameter<std::string>("z_reference","top");      // top|center|bottom

  node->declare_parameter<double>("approach_dist",   0.10);       // m
  node->declare_parameter<double>("table_clear",     0.01);       // m
  node->declare_parameter<double>("lift_dist",       0.15);       // m

  node->declare_parameter<bool>("wait_for_size",     true);
  node->declare_parameter<double>("size_timeout_s",  2.0);

  const std::string base  = node->get_parameter("base_frame").as_string();
  const std::string obj   = node->get_parameter("obj_frame").as_string();
  const std::string zref  = node->get_parameter("z_reference").as_string();
  const double approach_dist = node->get_parameter("approach_dist").as_double();
  const double table_clear   = node->get_parameter("table_clear").as_double();
  const double lift_dist     = node->get_parameter("lift_dist").as_double();
  const bool   wait_for_size = node->get_parameter("wait_for_size").as_bool();
  const double size_timeout  = node->get_parameter("size_timeout_s").as_double();

  // ---------------- TF ----------------
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // ---------------- Size topic ----------------
  ObjSize obj_size; bool got_size=false;
  auto sub = node->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/obj_1/size_xyz", 10,
    [&](const std_msgs::msg::Float32MultiArray::SharedPtr m){
      if (m->data.size() >= 3) {
        obj_size.x = m->data[0]; obj_size.y = m->data[1]; obj_size.z = m->data[2];
        got_size = true;
      }
    });

  // Optionally wait a bit for size
  rclcpp::Time t0 = node->get_clock()->now();
  while (rclcpp::ok() && wait_for_size && !got_size &&
         (node->get_clock()->now() - t0).seconds() < size_timeout)
    rclcpp::spin_some(node);

  // Heuristic: if sizes look like millimeters, auto-convert to meters and warn.
  if (obj_size.x > 1.0 || obj_size.y > 1.0 || obj_size.z > 1.0) {
    RCLCPP_WARN(node->get_logger(),
      "Object size looks too large (%.3f,%.3f,%.3f). Assuming millimeters and converting to meters.",
      obj_size.x, obj_size.y, obj_size.z);
    obj_size.x *= kMMtoM; obj_size.y *= kMMtoM; obj_size.z *= kMMtoM;
  }

  // ---------------- MoveIt setup ----------------
  moveit::planning_interface::MoveGroupInterface arm(node, "arm");
  moveit::planning_interface::MoveGroupInterface gripper(node, "gripper");
  arm.setPoseReferenceFrame(base);
  arm.setPlanningTime(10.0);
  arm.setNumPlanningAttempts(10);
  arm.setGoalPositionTolerance(0.005);           // 5 mm
  arm.setGoalOrientationTolerance(10.0 * M_PI/180.0); // 10 deg

  std::string ee_link = node->get_parameter("ee_link").as_string();
  if (!ee_link.empty()) {
    arm.setEndEffectorLink(ee_link);
  } else {
    ee_link = arm.getEndEffectorLink();
  }

  moveit::planning_interface::PlanningSceneInterface psi;

  auto now = [&]{ return node->get_clock()->now(); };

  // ---------------- TF wait ----------------
  while (rclcpp::ok() &&
         !tf_buffer->canTransform(base, obj, tf2::TimePointZero, tf2::durationFromSec(0.1)))
    rclcpp::spin_some(node);

  geometry_msgs::msg::TransformStamped t_bo;
  try {
    t_bo = tf_buffer->lookupTransform(base, obj, tf2::TimePointZero);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(node->get_logger(), "TF lookup failed: %s", ex.what());
    rclcpp::shutdown(); return 1;
  }

  // ---------------- Compute heights ----------------
  double obj_top_z;
  if (zref == "center")      obj_top_z = t_bo.transform.translation.z + obj_size.z*0.5;
  else if (zref == "bottom") obj_top_z = t_bo.transform.translation.z + obj_size.z;
  else                       obj_top_z = t_bo.transform.translation.z; // "top"

  // Z‑down orientation facing object (robust for many grippers)
  double yaw = std::atan2(t_bo.transform.translation.y, t_bo.transform.translation.x);
  tf2::Quaternion q_pick_tf; q_pick_tf.setRPY(M_PI, 0.0, yaw);
  geometry_msgs::msg::Quaternion q_pick = tf2::toMsg(q_pick_tf);

  auto mkPose = [&](double x,double y,double z, const geometry_msgs::msg::Quaternion &q){
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = base; ps.header.stamp = now();
    ps.pose.position.x=x; ps.pose.position.y=y; ps.pose.position.z=z;
    ps.pose.orientation = q; return ps;
  };

  auto pre   = mkPose(t_bo.transform.translation.x,
                      t_bo.transform.translation.y,
                      obj_top_z + approach_dist, q_pick);
  auto grasp = mkPose(t_bo.transform.translation.x,
                      t_bo.transform.translation.y,
                      obj_top_z + table_clear,   q_pick);
  auto lift  = grasp; lift.pose.position.z += lift_dist;

  RCLCPP_INFO(node->get_logger(),
    "size(m)=%.3f,%.3f,%.3f  zref=%s  top_z=%.3f  pre_z=%.3f  grasp_z=%.3f  lift_z=%.3f  ee=%s",
    obj_size.x, obj_size.y, obj_size.z, zref.c_str(),
    obj_top_z, pre.pose.position.z, grasp.pose.position.z, lift.pose.position.z, ee_link.c_str());

  // ---------------- Safety: dry‑run IK checks ----------------
  {
    // Try planning to pre‑grasp (pose goal)
    arm.setPoseTarget(pre);
    moveit::planning_interface::MoveGroupInterface::Plan plan_pre;
    if (arm.plan(plan_pre) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "Pre‑grasp IK/plan failed. Adjust approach_dist or orientation.");
      rclcpp::shutdown(); return 2;
    }
  }

  // ---------------- Open gripper BEFORE approach (optional) ----------------
  gripper.setNamedTarget("open");
  if (gripper.move() != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(node->get_logger(), "Gripper open may have failed; continuing.");
  }

  // ---------------- Execute to pre‑grasp ----------------
  {
    arm.setPoseTarget(pre);
    moveit::planning_interface::MoveGroupInterface::Plan plan_pre;
    if (arm.plan(plan_pre) != moveit::planning_interface::MoveItErrorCode::SUCCESS ||
        arm.execute(plan_pre) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "Move to pre‑grasp failed.");
      rclcpp::shutdown(); return 3;
    }
  }

  // ---------------- Cartesian descend (pre -> grasp) ----------------
  {
    std::vector<geometry_msgs::msg::Pose> wps;
    auto p = pre.pose; p.position.z = grasp.pose.position.z; // straight down
    wps.push_back(p);

    moveit_msgs::msg::RobotTrajectory traj;
    const double eef_step = 0.005; // 5 mm
    const double jump_thr = 0.0;
    double ratio = arm.computeCartesianPath(wps, eef_step, jump_thr, traj);
    if (ratio < 0.99) {
      RCLCPP_ERROR(node->get_logger(), "Cartesian descend failed (ratio=%.2f). Increase approach_dist or relax tolerances.", ratio);
      rclcpp::shutdown(); return 4;
    }
    moveit::planning_interface::MoveGroupInterface::Plan cart;
    cart.trajectory_ = traj;
    if (arm.execute(cart) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "Execute descend failed.");
      rclcpp::shutdown(); return 5;
    }
  }

  // ---------------- Close gripper (or vacuum on) ----------------
  gripper.setNamedTarget("close");
  if (gripper.move() != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Gripper close failed.");
    rclcpp::shutdown(); return 6;
  }

  // ---------------- Attach collision object to EE ----------------
  {
    moveit_msgs::msg::CollisionObject co;
    co.header.frame_id = base;
    co.id = "obj_1";

    shape_msgs::msg::SolidPrimitive prim; prim.type = prim.BOX;
    prim.dimensions = {obj_size.x, obj_size.y, obj_size.z};

    geometry_msgs::msg::Pose p; p.orientation.w = 1.0;
    p.position.x = t_bo.transform.translation.x;
    p.position.y = t_bo.transform.translation.y;
    // Place the box center at the correct Z given zref:
    if (zref == "center")      p.position.z = t_bo.transform.translation.z;
    else if (zref == "bottom") p.position.z = t_bo.transform.translation.z + obj_size.z*0.5;
    else /* top */             p.position.z = t_bo.transform.translation.z - obj_size.z*0.5;

    co.primitives.push_back(prim);
    co.primitive_poses.push_back(p);
    co.operation = co.ADD;

    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObject(co);

    moveit_msgs::msg::AttachedCollisionObject aco;
    aco.link_name = ee_link;
    aco.object = co;
    psi.applyAttachedCollisionObject(aco);
  }

  // ---------------- Lift (Cartesian up) ----------------
  {
    std::vector<geometry_msgs::msg::Pose> wps;
    auto p = grasp.pose; p.position.z += lift_dist;
    wps.push_back(p);

    moveit_msgs::msg::RobotTrajectory traj;
    const double eef_step = 0.005, jump_thr = 0.0;
    double ratio = arm.computeCartesianPath(wps, eef_step, jump_thr, traj);
    if (ratio < 0.99) {
      RCLCPP_ERROR(node->get_logger(), "Cartesian lift failed (ratio=%.2f). Increase lift_dist.", ratio);
      rclcpp::shutdown(); return 7;
    }
    moveit::planning_interface::MoveGroupInterface::Plan cart;
    cart.trajectory_ = traj;
    if (arm.execute(cart) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "Execute lift failed.");
      rclcpp::shutdown(); return 8;
    }
  }

  RCLCPP_INFO(node->get_logger(), "Pick & lift complete ✅");
  rclcpp::shutdown();
  return 0;
}
