#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

class PushBoxNode : public rclcpp::Node
{
public:
  PushBoxNode()
  : Node("push_box_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter<std::string>("planning_group", "L_xarm7");
    declare_parameter<std::string>("planning_frame", "world");
    declare_parameter<std::string>("object_frame", "object_nominal");
    declare_parameter<std::string>("ee_link", "");

    declare_parameter<std::vector<double>>(
      "initial_joint_positions",
      std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    declare_parameter<double>("box_height", 0.12);
    declare_parameter<double>("box_width", 0.07);
    declare_parameter<double>("box_thickness", 0.02);

    declare_parameter<double>("approach_standoff", 0.05);
    declare_parameter<double>("push_distance", 0.04);
    declare_parameter<double>("cartesian_eef_step", 0.005);
    declare_parameter<double>("cartesian_fraction_min", 0.90);

    planning_group_ = get_parameter("planning_group").as_string();
    planning_frame_ = get_parameter("planning_frame").as_string();
    object_frame_ = get_parameter("object_frame").as_string();
    ee_link_ = get_parameter("ee_link").as_string();

    initial_joint_positions_ =
      get_parameter("initial_joint_positions").as_double_array();

    box_height_ = get_parameter("box_height").as_double();
    box_width_ = get_parameter("box_width").as_double();
    box_thickness_ = get_parameter("box_thickness").as_double();

    approach_standoff_ = get_parameter("approach_standoff").as_double();
    push_distance_ = get_parameter("push_distance").as_double();
    cartesian_eef_step_ = get_parameter("cartesian_eef_step").as_double();
    cartesian_fraction_min_ = get_parameter("cartesian_fraction_min").as_double();
  }

  void run()
  {
    if (initial_joint_positions_.size() != 7) {
      RCLCPP_ERROR(get_logger(),
                   "initial_joint_positions must contain exactly 7 values, got %zu",
                   initial_joint_positions_.size());
      return;
    }

    auto move_group =
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), planning_group_);

    move_group->setPlanningTime(10.0);
    move_group->setNumPlanningAttempts(10);
    move_group->setMaxVelocityScalingFactor(0.2);
    move_group->setMaxAccelerationScalingFactor(0.2);

    if (!ee_link_.empty()) {
      move_group->setEndEffectorLink(ee_link_);
    }

    move_group->setStartStateToCurrentState();

    bool joint_target_ok = move_group->setJointValueTarget(initial_joint_positions_);
    if (!joint_target_ok) {
      RCLCPP_ERROR(get_logger(),
                   "initial_joint_positions were rejected or out of bounds");
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan seed_plan;
    auto seed_ok = move_group->plan(seed_plan);

    if (seed_ok != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to plan to initial joint pose");
      return;
    }

    RCLCPP_INFO(get_logger(), "Executing initial joint pose");
    if (move_group->execute(seed_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to execute initial joint pose");
      return;
    }

    auto object_tf = waitForTransform();
    addCollisionBox(object_tf);

    rclcpp::sleep_for(500ms);
    move_group->setStartStateToCurrentState();

    geometry_msgs::msg::Pose pre_push_pose;
    geometry_msgs::msg::Pose contact_pose;
    geometry_msgs::msg::Pose post_push_pose;
    buildPushPoses(object_tf, pre_push_pose, contact_pose, post_push_pose);

    geometry_msgs::msg::PoseStamped pre_push_target;
    pre_push_target.header.frame_id = planning_frame_;
    pre_push_target.header.stamp = now();
    pre_push_target.pose = pre_push_pose;

    move_group->setStartStateToCurrentState();
    move_group->setPoseTarget(pre_push_target);

    moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
    auto approach_ok = move_group->plan(approach_plan);

    if (approach_ok != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to plan to pre-push pose");
      return;
    }

    RCLCPP_INFO(get_logger(), "Executing approach to pre-push pose");
    if (move_group->execute(approach_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to execute approach plan");
      return;
    }

    rclcpp::sleep_for(250ms);
    move_group->setStartStateToCurrentState();

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(pre_push_pose);
    waypoints.push_back(contact_pose);
    waypoints.push_back(post_push_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group->computeCartesianPath(
      waypoints, cartesian_eef_step_, 0.0, trajectory);

    RCLCPP_INFO(get_logger(), "Cartesian path fraction: %.3f", fraction);

    if (fraction < cartesian_fraction_min_) {
      RCLCPP_ERROR(get_logger(),
                   "Cartesian push path fraction too low: %.3f < %.3f",
                   fraction, cartesian_fraction_min_);
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan push_plan;
    push_plan.trajectory = trajectory;

    RCLCPP_INFO(get_logger(), "Executing push trajectory");
    if (move_group->execute(push_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to execute push trajectory");
      return;
    }

    RCLCPP_INFO(get_logger(), "Push completed");
  }

private:
  geometry_msgs::msg::TransformStamped waitForTransform()
  {
    for (int i = 0; i < 100; ++i) {
      try {
        return tf_buffer_.lookupTransform(
          planning_frame_, object_frame_, tf2::TimePointZero, tf2::durationFromSec(0.2));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Waiting for transform %s -> %s: %s",
          planning_frame_.c_str(), object_frame_.c_str(), ex.what());
        rclcpp::sleep_for(100ms);
      }
    }
    throw std::runtime_error("Timed out waiting for object transform");
  }

  void addCollisionBox(const geometry_msgs::msg::TransformStamped & object_tf)
  {
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = planning_frame_;
    obj.id = "push_box_object";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {box_thickness_, box_width_, box_height_};

    geometry_msgs::msg::Pose pose;
    pose.position.x = object_tf.transform.translation.x;
    pose.position.y = object_tf.transform.translation.y;
    pose.position.z = object_tf.transform.translation.z - (box_height_ / 2.0);
    // pose.orientation = object_tf.transform.rotation;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, M_PI_2);
    q.normalize();
    pose.orientation = tf2::toMsg(q);

    // pose.orientation.x = 0.0;
    // pose.orientation.y = 0.0;
    // pose.orientation.z = 0.0;
    // pose.orientation.w = 1.0;

    obj.primitives.push_back(primitive);
    obj.primitive_poses.push_back(pose);
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;

    planning_scene_interface.applyCollisionObject(obj);

    RCLCPP_INFO(get_logger(),
                "Added collision box at (%.3f, %.3f, %.3f)",
                pose.position.x, pose.position.y, pose.position.z);
  }

  void buildPushPoses(const geometry_msgs::msg::TransformStamped & object_tf,
                      geometry_msgs::msg::Pose & pre_push_pose,
                      geometry_msgs::msg::Pose & contact_pose,
                      geometry_msgs::msg::Pose & post_push_pose)
  {
    tf2::Transform T_world_object;
    tf2::fromMsg(object_tf.transform, T_world_object);

    tf2::Quaternion push_orientation;
    push_orientation.setRPY(0.0, M_PI_2, 0.0);

    tf2::Transform T_object_tool;
    T_object_tool.setRotation(push_orientation);

    const double contact_x = -box_thickness_ / 2.0;
    const double contact_y = 0.0;
    const double contact_z = -box_height_ / 2.0;

    const double pre_x = contact_x - approach_standoff_;
    const double post_x = contact_x + push_distance_;

    T_object_tool.setOrigin(tf2::Vector3(pre_x, contact_y, contact_z));
    tf2::Transform T_world_pre = T_world_object * T_object_tool;

    T_object_tool.setOrigin(tf2::Vector3(contact_x, contact_y, contact_z));
    tf2::Transform T_world_contact = T_world_object * T_object_tool;

    T_object_tool.setOrigin(tf2::Vector3(post_x, contact_y, contact_z));
    tf2::Transform T_world_post = T_world_object * T_object_tool;

    tf2::toMsg(T_world_pre, pre_push_pose);
    tf2::toMsg(T_world_contact, contact_pose);
    tf2::toMsg(T_world_post, post_push_pose);

    RCLCPP_INFO(get_logger(),
                "Pre-push pose: (%.3f, %.3f, %.3f)",
                pre_push_pose.position.x, pre_push_pose.position.y, pre_push_pose.position.z);
    RCLCPP_INFO(get_logger(),
                "Contact pose: (%.3f, %.3f, %.3f)",
                contact_pose.position.x, contact_pose.position.y, contact_pose.position.z);
    RCLCPP_INFO(get_logger(),
                "Post-push pose: (%.3f, %.3f, %.3f)",
                post_push_pose.position.x, post_push_pose.position.y, post_push_pose.position.z);
  }

  std::vector<double> initial_joint_positions_;

  std::string planning_group_;
  std::string planning_frame_;
  std::string object_frame_;
  std::string ee_link_;

  double box_height_;
  double box_width_;
  double box_thickness_;
  double approach_standoff_;
  double push_distance_;
  double cartesian_eef_step_;
  double cartesian_fraction_min_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PushBoxNode>();

  try {
    node->run();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(node->get_logger(), "Unhandled exception: %s", ex.what());
  }

  rclcpp::shutdown();
  return 0;
}