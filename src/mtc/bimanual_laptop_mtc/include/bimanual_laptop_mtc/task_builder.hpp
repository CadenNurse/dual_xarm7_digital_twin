#pragma once

#include "bimanual_laptop_mtc/laptop_geometry.hpp"
#include <memory>
#include <moveit/task_constructor/task.h>
#include <rclcpp/rclcpp.hpp>

namespace bimanual_laptop_mtc
{

struct TaskBuilderConfig
{
  std::string arm_group;
  std::string eef_frame;
  std::string world_frame;

  double lid_angle_start_rad;
  double lid_angle_goal_rad;
  double pregrasp_offset_z;
  double step_translation_m;
  int step_count;
  double retreat_distance_m;

  double table_size_x;
  double table_size_y;
  double table_size_z;
  double table_pose_z;
};

moveit::task_constructor::Task buildCloseLaptopSingleArmTask(
  const rclcpp::Node::SharedPtr& node,
  const geometry_msgs::msg::PoseStamped& laptop_base_pose,
  const LaptopGeometry& geom,
  const TaskBuilderConfig& cfg);

}  // namespace bimanual_laptop_mtc