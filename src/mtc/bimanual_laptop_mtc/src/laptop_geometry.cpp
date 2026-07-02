#include "bimanual_laptop_mtc/laptop_geometry.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace bimanual_laptop_mtc
{

geometry_msgs::msg::PoseStamped makeLidEdgePregraspPose(
  const LaptopGeometry& geom,
  const geometry_msgs::msg::PoseStamped& base_pose,
  double lid_angle_rad,
  double offset_z)
{
  geometry_msgs::msg::PoseStamped pose = base_pose;

  const double hinge_x = geom.hinge_offset_x;
  const double edge_x = hinge_x + geom.depth * std::cos(lid_angle_rad);
  const double edge_z = geom.thickness_base + geom.depth * std::sin(lid_angle_rad);

  pose.pose.position.x += edge_x;
  pose.pose.position.z += edge_z + offset_z;

  tf2::Quaternion q;
  q.setRPY(M_PI, 0.0, -M_PI_2);
  pose.pose.orientation = tf2::toMsg(q);
  return pose;
}

}  // namespace bimanual_laptop_mtc