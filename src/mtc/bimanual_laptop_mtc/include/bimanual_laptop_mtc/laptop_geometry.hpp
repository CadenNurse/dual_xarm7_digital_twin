#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>

namespace bimanual_laptop_mtc
{

struct LaptopGeometry
{
  double width{0.320};
  double depth{0.220};
  double thickness_base{0.018};
  double thickness_lid{0.008};
  double hinge_offset_x{-0.140};
};

struct LaptopKinematicState
{
  geometry_msgs::msg::PoseStamped base_pose;
  double lid_angle_rad{0.0};
};

geometry_msgs::msg::PoseStamped makeLidEdgePregraspPose(
  const LaptopGeometry& geom,
  const geometry_msgs::msg::PoseStamped& base_pose,
  double lid_angle_rad,
  double offset_z);

}  // namespace bimanual_laptop_mtc