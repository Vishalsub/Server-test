#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/planning_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>

using rclcpp::Node;

struct ObjSize { double x=0.05, y=0.05, z=0.02; }; // default if topic not received

bool movePose(moveit::planning_interface::MoveGroupInterface &mg,
              const geometry_msgs::msg::PoseStamped &ps)
{
  mg.setPoseTarget(ps);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (mg.plan(plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) return false;
  return mg.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Node>("pick_place_obj1");

  // --- TF ---
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  const std::string base = "base";          // or base_link
  const std::string obj  = "obj_1";

  // --- Get size from topic ---
  ObjSize obj_size;
  auto sub = node->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/obj_1/size_xyz", 10,
      [&](const std_msgs::msg::Float32MultiArray::SharedPtr m){
        if (m->data.size() >= 3) { obj_size.x=m->data[0]; obj_size.y=m->data[1]; obj_size.z=m->data[2]; }
      });

  // --- MoveIt groups ---
  moveit::planning_interface::MoveGroupInterface arm(node, "arm");
  moveit::planning_interface::MoveGroupInterface gripper(node, "gripper");
  arm.setPoseReferenceFrame(base);
  arm.setPlanningTime(5.0);
  std::string ee_link = arm.getEndEffectorLink();

  // --- helpers ---
  auto now = [&]{ return node->get_clock()->now(); };

  // Wait for TF
  while (rclcpp::ok() && !tf_buffer->canTransform(base, obj, tf2::TimePointZero, tf2::durationFromSec(0.1)))
    rclcpp::spin_some(node);

  // Lookup base->obj
  auto t_bo = tf_buffer->lookupTransform(base, obj, tf2::TimePointZero);

  // Compute key heights
  const double table_clear = 0.01;          // safety clearance above top surface
  const double approach_dist = 0.10;        // approach from above (10 cm)
  double obj_top_z = t_bo.transform.translation.z + obj_size.z * 0.5;

  // Build poses
  auto mkPose = [&](double x,double y,double z, const geometry_msgs::msg::Quaternion &q){
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = base; ps.header.stamp = now();
    ps.pose.position.x=x; ps.pose.position.y=y; ps.pose.position.z=z;
    ps.pose.orientation = q;
    return ps;
  };

  // Use object's orientation for top-down pick (adjust if your gripper needs Z-down, etc.)
  geometry_msgs::msg::Quaternion q_pick = t_bo.transform.rotation;

  // 1) Open gripper
  gripper.setNamedTarget("open");
  gripper.move();

  // 2) Pre-grasp (above object)
  auto pre = mkPose(t_bo.transform.translation.x,
                    t_bo.transform.translation.y,
                    obj_top_z + approach_dist,
                    q_pick);
  if (!movePose(arm, pre)) { RCLCPP_ERROR(node->get_logger(),"pre-grasp failed"); return 1; }

  // 3) Descend to grasp (just above top surface)
  auto grasp = mkPose(t_bo.transform.translation.x,
                      t_bo.transform.translation.y,
                      obj_top_z + table_clear,
                      q_pick);
  if (!movePose(arm, grasp)) { RCLCPP_ERROR(node->get_logger(),"grasp pose failed"); return 1; }

  // 4) Close gripper (or enable vacuum)
  gripper.setNamedTarget("close");
  gripper.move();

  // 4b) Attach object to EE (so planner knows it's in hand)
  moveit::planning_interface::PlanningSceneInterface psi;
  moveit_msgs::msg::CollisionObject co;
  co.header.frame_id = base;
  co.id = "obj_1";
  shape_msgs::msg::SolidPrimitive prim;
  prim.type = prim.BOX;
  prim.dimensions = {obj_size.x, obj_size.y, obj_size.z};
  geometry_msgs::msg::Pose p; p.orientation.w = 1.0;
  p.position.x = t_bo.transform.translation.x;
  p.position.y = t_bo.transform.translation.y;
  p.position.z = t_bo.transform.translation.z;
  co.primitives.push_back(prim);
  co.primitive_poses.push_back(p);
  co.operation = co.ADD;
  psi.applyCollisionObject(co);

  moveit_msgs::msg::AttachedCollisionObject aco;
  aco.link_name = ee_link;
  aco.object = co;
  psi.applyAttachedCollisionObject(aco);

  // 5) Lift
  auto lift = grasp; lift.pose.position.z += 0.15;
  if (!movePose(arm, lift)) { RCLCPP_ERROR(node->get_logger(),"lift failed"); return 1; }

  // 6) Place target (vertical)
  // Choose a place frame or XY; here we re-use obj XY but you can use a TF "place_target".
  // Rotate to make the object vertical: rotate EE by +90deg about Y (example).
  tf2::Quaternion q_v; q_v.setRPY(0.0, M_PI/2.0, 0.0);
  geometry_msgs::msg::Quaternion q_place; q_place = tf2::toMsg(q_v);

  // pick a place position (change as needed)
  double place_x = 0.50, place_y = 0.0, ground_z = 0.0;
  double place_top = ground_z + obj_size.z;      // when vertical, top is 1*height above ground
  auto place_pre = mkPose(place_x, place_y, place_top + 0.20, q_place);
  auto place     = mkPose(place_x, place_y, place_top + table_clear, q_place);

  if (!movePose(arm, place_pre)) { RCLCPP_ERROR(node->get_logger(),"pre-place failed"); return 1; }
  if (!movePose(arm, place))    { RCLCPP_ERROR(node->get_logger(),"place descend failed"); return 1; }

  // 7) Release & detach
  gripper.setNamedTarget("open");
  gripper.move();

  // detach + remove collision
  aco.object.operation = aco.object.REMOVE;
  psi.applyAttachedCollisionObject(aco);
  co.operation = co.REMOVE;
  psi.applyCollisionObject(co);

  // Retreat
  auto retreat = place; retreat.pose.position.z += 0.20;
  movePose(arm, retreat);

  RCLCPP_INFO(node->get_logger(), "Pick & place done.");
  rclcpp::shutdown();
  return 0;
}
