#include "bimanual_laptop_mtc/scene_utils.hpp"

#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace bimanual_laptop_mtc
{

moveit_msgs::msg::CollisionObject makeBoxObject(
  const std::string& id,
  const std::string& frame_id,
  double x, double y, double z,
  const geometry_msgs::msg::Pose& pose)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.id = id;
  obj.header.frame_id = frame_id;

  shape_msgs::msg::SolidPrimitive prim;
  prim.type = prim.BOX;
  prim.dimensions = {x, y, z};

  obj.primitives.push_back(prim);
  obj.primitive_poses.push_back(pose);
  obj.operation = obj.ADD;
  return obj;
}

std::vector<moveit_msgs::msg::CollisionObject> makeLaptopScene(
  const std::string& world_frame,
  const geometry_msgs::msg::PoseStamped& base_pose,
  const LaptopGeometry& geom,
  double table_size_x,
  double table_size_y,
  double table_size_z,
  double table_pose_z)
{
  std::vector<moveit_msgs::msg::CollisionObject> objects;

  geometry_msgs::msg::Pose table_pose;
  table_pose.orientation.w = 1.0;
  table_pose.position.z = table_pose_z;
  objects.push_back(makeBoxObject(
    "table", world_frame, table_size_x, table_size_y, table_size_z, table_pose));

  geometry_msgs::msg::Pose base_box = base_pose.pose;
  base_box.position.z += geom.thickness_base * 0.5;
  objects.push_back(makeBoxObject(
    "laptop_base", world_frame, geom.depth, geom.width, geom.thickness_base, base_box));

  geometry_msgs::msg::Pose lid_box = base_pose.pose;
  lid_box.position.x += geom.hinge_offset_x + geom.depth * 0.35;
  lid_box.position.z += geom.thickness_base + 0.07;
  tf2::Quaternion q;
  q.setRPY(0.0, -1.22, 0.0);
  lid_box.orientation = tf2::toMsg(q);

  objects.push_back(makeBoxObject(
    "laptop_lid_proxy", world_frame, geom.depth, geom.width, geom.thickness_lid, lid_box));

  return objects;
}

void applyScene(
  moveit::planning_interface::PlanningSceneInterface& psi,
  const std::vector<moveit_msgs::msg::CollisionObject>& objects)
{
  psi.applyCollisionObjects(objects);
}

}  // namespace bimanual_laptop_mtc