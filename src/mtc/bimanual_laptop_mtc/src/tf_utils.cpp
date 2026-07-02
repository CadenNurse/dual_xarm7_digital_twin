#include "bimanual_laptop_mtc/tf_utils.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace bimanual_laptop_mtc
{

geometry_msgs::msg::PoseStamped lookupPoseOrNominal(
  const rclcpp::Node::SharedPtr& node,
  tf2_ros::Buffer& tf_buffer,
  const std::string& world_frame,
  const std::string& object_frame)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = node->now();
  pose.header.frame_id = world_frame;

  try
  {
    const auto tf = tf_buffer.lookupTransform(
      world_frame, object_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));

    pose.pose.position.x = tf.transform.translation.x;
    pose.pose.position.y = tf.transform.translation.y;
    pose.pose.position.z = tf.transform.translation.z;
    pose.pose.orientation = tf.transform.rotation;
    return pose;
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN(
      node->get_logger(),
      "Falling back to nominal laptop pose because TF lookup failed: %s",
      ex.what());

    pose.pose.position.x = 0.45;
    pose.pose.position.y = 0.00;
    pose.pose.position.z = 0.02;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    return pose;
  }
}

}  // namespace bimanual_laptop_mtc