#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/planning_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

using rclcpp::Node;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Node>("scene_add_obj1");

  // --- Parameters ---
  node->declare_parameter<std::string>("planning_frame", "world");  // MoveIt planning frame (often "world" or "base")
  node->declare_parameter<std::string>("obj_frame",      "obj_1");  // TF of the object from Isaac
  node->declare_parameter<std::string>("z_reference",    "top");    // one of: top|center|bottom
  node->declare_parameter<bool>("use_obj_orientation",   true);     // use TF orientation; set false to make it axis-aligned
  node->declare_parameter<std::string>("object_id",      "obj_1");  // CollisionObject id

  // Box size in meters (set these to match your USD after scaling)
  node->declare_parameter<double>("size_x", 0.05);
  node->declare_parameter<double>("size_y", 0.05);
  node->declare_parameter<double>("size_z", 0.02);

  // Read params
  const auto planning_frame     = node->get_parameter("planning_frame").as_string();
  const auto obj_frame          = node->get_parameter("obj_frame").as_string();
  const auto z_reference        = node->get_parameter("z_reference").as_string();
  const bool use_obj_orientation= node->get_parameter("use_obj_orientation").as_bool();
  const auto object_id          = node->get_parameter("object_id").as_string();
  double sx = node->get_parameter("size_x").as_double();
  double sy = node->get_parameter("size_y").as_double();
  double sz = node->get_parameter("size_z").as_double();

  // Clamp sizes to sane bounds (1 mm.. 2 m)
  auto clamp = [](double &v){ v = std::max(0.001, std::min(2.0, v)); };
  clamp(sx); clamp(sy); clamp(sz);

  // --- TF setup ---
  auto buffer   = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto listener = std::make_shared<tf2_ros::TransformListener>(*buffer);

  // Wait for TF planning_frame -> obj_frame
  while (rclcpp::ok() &&
         !buffer->canTransform(planning_frame, obj_frame, tf2::TimePointZero, tf2::durationFromSec(0.1)))
    rclcpp::spin_some(node);

  geometry_msgs::msg::TransformStamped T_w_o;
  try {
    T_w_o = buffer->lookupTransform(planning_frame, obj_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(node->get_logger(), "TF lookup failed (%s -> %s): %s",
                 planning_frame.c_str(), obj_frame.c_str(), ex.what());
    rclcpp::shutdown();
    return 1;
  }

  // --- Build CollisionObject (BOX), pose at object CENTER ---
  moveit_msgs::msg::CollisionObject co;
  co.header.frame_id = planning_frame;
  co.id = object_id;

  shape_msgs::msg::SolidPrimitive prim;
  prim.type = prim.BOX;
  prim.dimensions = {sx, sy, sz};

  geometry_msgs::msg::Pose pose_center;
  // Position: start from TF translation…
  pose_center.position.x = T_w_o.transform.translation.x;
  pose_center.position.y = T_w_o.transform.translation.y;
  pose_center.position.z = T_w_o.transform.translation.z;

  // …adjust Z so the collision box is centered (if TF is top/bottom)
  if (z_reference == "top")       pose_center.position.z -= sz * 0.5;
  else if (z_reference == "bottom") pose_center.position.z += sz * 0.5;
  // if "center": no change

  // Orientation: either use TF orientation, or align with world axes
  if (use_obj_orientation) {
    pose_center.orientation = T_w_o.transform.rotation;
  } else {
    pose_center.orientation.w = 1.0; // identity (axis-aligned)
  }

  co.primitives.push_back(prim);
  co.primitive_poses.push_back(pose_center);
  co.operation = co.ADD;

  // --- Apply to Planning Scene ---
  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(co);

  RCLCPP_INFO(node->get_logger(),
              "Added '%s' box to scene: size(%.3f, %.3f, %.3f) m, z_ref=%s, frame=%s, from TF=%s",
              object_id.c_str(), sx, sy, sz, z_reference.c_str(),
              planning_frame.c_str(), obj_frame.c_str());

  // Keep node alive briefly so RViz/MoveGroup receive it
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  rclcpp::shutdown();
  return 0;
}
