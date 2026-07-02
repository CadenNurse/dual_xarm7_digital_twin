#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <string>

namespace bimanual_laptop_mtc
{

geometry_msgs::msg::PoseStamped lookupPoseOrNominal(
  const rclcpp::Node::SharedPtr& node,
  tf2_ros::Buffer& tf_buffer,
  const std::string& world_frame,
  const std::string& laptop_frame_hint);

}  // namespace bimanual_laptop_mtc