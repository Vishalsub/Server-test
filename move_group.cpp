#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/planning_interface/move_group_interface.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("go_to_obj1");

  // ---- TF setup ----
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  const std::string base_frame = "base";     // or "base_link"
  const std::string obj_frame  = "obj_1";
  const std::string world_frame= "world";    // RViz fixed frame

  // Wait for TF to be available (up to 5s)
  const rclcpp::Duration timeout = rclcpp::Duration::from_seconds(5.0);
  while (rclcpp::ok() &&
         !tf_buffer->canTransform(base_frame, obj_frame, tf2::TimePointZero, tf2::durationFromSec(0.1))) {
    rclcpp::spin_some(node);
  }

  geometry_msgs::msg::TransformStamped t_bo;
  try {
    t_bo = tf_buffer->lookupTransform(base_frame, obj_frame, tf2::TimePointZero, timeout.to_chrono<std::chrono::seconds>());
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(node->get_logger(), "TF lookup failed: %s", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  // ---- Build target pose (add small Z approach) ----
  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = base_frame;
  target.header.stamp = node->now();
  target.pose.position.x = t_bo.transform.translation.x;
  target.pose.position.y = t_bo.transform.translation.y;
  target.pose.position.z = t_bo.transform.translation.z + 0.05;  // 5 cm above

  // Use obj_1 orientation directly (adjust to your gripper as needed)
  target.pose.orientation = t_bo.transform.rotation;

  // ---- MoveIt setup ----
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  // Name must match your MoveIt group (e.g., "arm")
  moveit::planning_interface::MoveGroupInterface move_group(node, "arm");
  move_group.setPoseReferenceFrame(base_frame);
  move_group.setPlanningTime(5.0);

  move_group.setPoseTarget(target);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool ok = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (!ok) {
    RCLCPP_ERROR(node->get_logger(), "Planning failed.");
    rclcpp::shutdown();
    return 2;
  }

  auto res = move_group.execute(plan);
  if (res != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Execution failed.");
  } else {
    RCLCPP_INFO(node->get_logger(), "Executed plan to obj_1.");
  }

  rclcpp::shutdown();
  return 0;
}
