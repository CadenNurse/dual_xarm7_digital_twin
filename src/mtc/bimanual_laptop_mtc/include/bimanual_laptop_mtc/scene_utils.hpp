#pragma once

#include "bimanual_laptop_mtc/laptop_geometry.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <string>
#include <vector>

namespace bimanual_laptop_mtc
{

moveit_msgs::msg::CollisionObject makeBoxObject(
  const std::string& id,
  const std::string& frame_id,
  double x, double y, double z,
  const geometry_msgs::msg::Pose& pose);

std::vector<moveit_msgs::msg::CollisionObject> makeLaptopScene(
  const std::string& world_frame,
  const geometry_msgs::msg::PoseStamped& base_pose,
  const LaptopGeometry& geom,
  double table_size_x,
  double table_size_y,
  double table_size_z,
  double table_pose_z);

void applyScene(
  moveit::planning_interface::PlanningSceneInterface& psi,
  const std::vector<moveit_msgs::msg::CollisionObject>& objects);

}  // namespace bimanual_laptop_mtc