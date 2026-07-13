#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class CloseLaptopNode : public rclcpp::Node {
public:
  CloseLaptopNode() : Node("close_laptop") {
    world_frame_ = declare_parameter<std::string>("world_frame", "workspace_origin");
    base_tag_frame_ = declare_parameter<std::string>("base_tag_frame", "tag_laptop_base");
    object_id_ = declare_parameter<std::string>("object_id", "laptop_base");

    base_length_ = declare_parameter<double>("base_length", 0.305);
    base_width_ = declare_parameter<double>("base_width", 0.217);
    base_thickness_ = declare_parameter<double>("base_thickness", 0.015);

    base_tag_to_object_xyz_ = declare_parameter<std::vector<double>>(
      "base_tag_to_object_xyz", std::vector<double>{0.0, 0.0, 0.0});
    base_tag_to_object_rpy_ = declare_parameter<std::vector<double>>(
      "base_tag_to_object_rpy", std::vector<double>{0.0, 0.0, 0.0});

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&CloseLaptopNode::spawnLaptopBase, this));

    RCLCPP_INFO(get_logger(), "close_laptop node started");
  }

private:
  geometry_msgs::msg::Pose makePoseFromXYZRPY(
    const std::vector<double>& xyz,
    const std::vector<double>& rpy) const {
    geometry_msgs::msg::Pose pose;
    pose.position.x = xyz.at(0);
    pose.position.y = xyz.at(1);
    pose.position.z = xyz.at(2);
    tf2::Quaternion q;
    q.setRPY(rpy.at(0), rpy.at(1), rpy.at(2));
    pose.orientation = tf2::toMsg(q);
    return pose;
  }

  geometry_msgs::msg::Pose multiplyPoses(
    const geometry_msgs::msg::Pose& a,
    const geometry_msgs::msg::Pose& b) const {
    tf2::Transform ta, tb, tc;
    tf2::fromMsg(a, ta);
    tf2::fromMsg(b, tb);
    tc = ta * tb;

    geometry_msgs::msg::Pose out;
    tf2::toMsg(tc, out);
    return out;
  }

  void spawnLaptopBase() {
    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_->lookupTransform(world_frame_, base_tag_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for base tag TF %s <- %s: %s",
        world_frame_.c_str(), base_tag_frame_.c_str(), ex.what());
      return;
    }

    geometry_msgs::msg::Pose world_to_tag;
    world_to_tag.position.x = tf_msg.transform.translation.x;
    world_to_tag.position.y = tf_msg.transform.translation.y;
    world_to_tag.position.z = tf_msg.transform.translation.z;
    world_to_tag.orientation = tf_msg.transform.rotation;

    geometry_msgs::msg::Pose tag_to_object = makePoseFromXYZRPY(
      base_tag_to_object_xyz_, base_tag_to_object_rpy_);
    geometry_msgs::msg::Pose world_to_object = multiplyPoses(world_to_tag, tag_to_object);

    moveit_msgs::msg::CollisionObject obj;
    obj.id = object_id_;
    obj.header.frame_id = world_frame_;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {base_length_, base_width_, base_thickness_};

    geometry_msgs::msg::Pose primitive_pose = world_to_object;
    primitive_pose.position.z += base_thickness_ / 2.0;

    obj.primitives.push_back(primitive);
    obj.primitive_poses.push_back(primitive_pose);
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;

    planning_scene_interface_.applyCollisionObject(obj);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Spawned/updated laptop base collision object '%s' in frame '%s'",
      object_id_.c_str(), world_frame_.c_str());
  }

  std::string world_frame_;
  std::string base_tag_frame_;
  std::string object_id_;

  double base_length_;
  double base_width_;
  double base_thickness_;

  std::vector<double> base_tag_to_object_xyz_;
  std::vector<double> base_tag_to_object_rpy_;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CloseLaptopNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
