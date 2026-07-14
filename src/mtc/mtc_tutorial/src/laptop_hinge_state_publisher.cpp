#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class LaptopHingeStatePublisher : public rclcpp::Node
{
public:
  LaptopHingeStatePublisher()
  : rclcpp::Node("laptop_hinge_state_publisher")
  {
    world_frame_ = declare_parameter<std::string>("world_frame", "workspace_origin");
    laptop_root_frame_ = declare_parameter<std::string>("laptop_root_frame", "laptop_base_link");
    base_tag_frame_ = declare_parameter<std::string>("base_tag_frame", "tag_laptop_base");
    lid_inner_tag_frame_ = declare_parameter<std::string>("lid_inner_tag_frame", "tag_laptop_lid_inner");
    lid_outer_tag_frame_ = declare_parameter<std::string>("lid_outer_tag_frame", "tag_laptop_lid_outer");
    hinge_joint_name_ = declare_parameter<std::string>("hinge_joint_name", "hinge_joint");
    hinge_axis_ = declare_parameter<std::string>("hinge_axis", "x");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);
    hinge_lower_ = declare_parameter<double>("hinge_lower", 0.0);
    hinge_upper_ = declare_parameter<double>("hinge_upper", 3.14159265359);

    base_tag_to_root_xyz_ = declare_parameter<std::vector<double>>(
      "base_tag_to_root_xyz", std::vector<double>{0.0, 0.0, 0.0});
    base_tag_to_root_rpy_ = declare_parameter<std::vector<double>>(
      "base_tag_to_root_rpy", std::vector<double>{0.0, 0.0, 0.0});
    lid_inner_tag_to_lid_xyz_ = declare_parameter<std::vector<double>>(
      "lid_inner_tag_to_lid_xyz", std::vector<double>{0.0, 0.0, 0.0});
    lid_inner_tag_to_lid_rpy_ = declare_parameter<std::vector<double>>(
      "lid_inner_tag_to_lid_rpy", std::vector<double>{0.0, 0.0, 0.0});
    lid_outer_tag_to_lid_xyz_ = declare_parameter<std::vector<double>>(
      "lid_outer_tag_to_lid_xyz", std::vector<double>{0.0, 0.0, 0.0});
    lid_outer_tag_to_lid_rpy_ = declare_parameter<std::vector<double>>(
      "lid_outer_tag_to_lid_rpy", std::vector<double>{0.0, 0.0, 0.0});

    validateVectorParam(base_tag_to_root_xyz_, "base_tag_to_root_xyz");
    validateVectorParam(base_tag_to_root_rpy_, "base_tag_to_root_rpy");
    validateVectorParam(lid_inner_tag_to_lid_xyz_, "lid_inner_tag_to_lid_xyz");
    validateVectorParam(lid_inner_tag_to_lid_rpy_, "lid_inner_tag_to_lid_rpy");
    validateVectorParam(lid_outer_tag_to_lid_xyz_, "lid_outer_tag_to_lid_xyz");
    validateVectorParam(lid_outer_tag_to_lid_rpy_, "lid_outer_tag_to_lid_rpy");

    if (hinge_axis_ != "x" && hinge_axis_ != "y" && hinge_axis_ != "z") {
      throw std::runtime_error("hinge_axis must be one of: x, y, z");
    }

    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/laptop/joint_states", 10);

    RCLCPP_INFO(get_logger(), "Laptop hinge state publisher constructed");
    RCLCPP_INFO(get_logger(), "world_frame: %s", world_frame_.c_str());
    RCLCPP_INFO(get_logger(), "laptop_root_frame: %s", laptop_root_frame_.c_str());
    RCLCPP_INFO(get_logger(), "base_tag_frame: %s", base_tag_frame_.c_str());
    RCLCPP_INFO(get_logger(), "lid_inner_tag_frame: %s", lid_inner_tag_frame_.c_str());
    RCLCPP_INFO(get_logger(), "lid_outer_tag_frame: %s", lid_outer_tag_frame_.c_str());
  }

  void initialize()
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_buffer_->setUsingDedicatedThread(true);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, shared_from_this(), true);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&LaptopHingeStatePublisher::update, this));

    RCLCPP_INFO(get_logger(), "Laptop hinge state publisher initialized");
  }

private:
  static void validateVectorParam(const std::vector<double>& vec, const std::string& name)
  {
    if (vec.size() != 3) {
      throw std::runtime_error(name + " must contain exactly 3 values");
    }
  }

  static tf2::Transform transformMsgToTf(const geometry_msgs::msg::Transform& msg)
  {
    tf2::Transform tf;
    tf2::fromMsg(msg, tf);
    return tf;
  }

  static geometry_msgs::msg::Transform tfToTransformMsg(const tf2::Transform& tf)
  {
    return tf2::toMsg(tf);
  }

  static tf2::Transform makeTransform(const std::vector<double>& xyz, const std::vector<double>& rpy)
  {
    tf2::Quaternion q;
    q.setRPY(rpy[0], rpy[1], rpy[2]);
    tf2::Transform t;
    t.setOrigin(tf2::Vector3(xyz[0], xyz[1], xyz[2]));
    t.setRotation(q);
    return t;
  }

  bool lookupTransform(
    const std::string& target_frame,
    const std::string& source_frame,
    geometry_msgs::msg::TransformStamped& out_tf)
  {
    try {
      out_tf = tf_buffer_->lookupTransform(
        target_frame, source_frame, tf2::TimePointZero, tf2::durationFromSec(0.05));
      return true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed TF lookup %s <- %s: %s",
        target_frame.c_str(), source_frame.c_str(), ex.what());
      return false;
    }
  }

  double extractHingeAngle(const tf2::Transform& base_tag_T_lid_tag) const
  {
    const tf2::Matrix3x3 R(base_tag_T_lid_tag.getRotation());

    // Tag -> RViz mapping:
    // tag x = rviz z
    // tag y = rviz -x
    // tag z = rviz -y
    //
    // Hinge is about rviz x, so in tag coordinates:
    // rviz x = tag -y
    const tf2::Vector3 hinge_axis_tag(0.0, -1.0, 0.0);
    const tf2::Vector3 ref_tag(1.0, 0.0, 0.0);  // rviz z

    tf2::Vector3 base_ref = ref_tag;
    tf2::Vector3 lid_ref = R * ref_tag;

    base_ref -= hinge_axis_tag * base_ref.dot(hinge_axis_tag);
    lid_ref  -= hinge_axis_tag * lid_ref.dot(hinge_axis_tag);

    if (base_ref.length2() < 1e-12 || lid_ref.length2() < 1e-12) {
      return 0.0;
    }

    base_ref.normalize();
    lid_ref.normalize();

    const double sin_angle = hinge_axis_tag.dot(lid_ref.cross(base_ref));
    const double cos_angle = base_ref.dot(lid_ref);
    double angle = std::atan2(sin_angle, cos_angle);

    if (angle < 0.0) {
      angle += 2.0 * M_PI;
    }

    angle = M_PI - angle;

    if (angle < 0.0) {
      angle += 2.0 * M_PI;
    }
    if (angle >= 2.0 * M_PI) {
      angle -= 2.0 * M_PI;
    }

    RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "hinge_axis=(%.3f %.3f %.3f) base_ref=(%.3f %.3f %.3f) lid_ref=(%.3f %.3f %.3f) sin=%.3f cos=%.3f angle=%.3f",
    hinge_axis_tag.x(), hinge_axis_tag.y(), hinge_axis_tag.z(),
    base_ref.x(), base_ref.y(), base_ref.z(),
    lid_ref.x(), lid_ref.y(), lid_ref.z(),
    sin_angle, cos_angle, angle);

    return std::clamp(angle, hinge_lower_, hinge_upper_);
  }

  void publishHingeJoint(
    const geometry_msgs::msg::TransformStamped& world_T_base_tag_msg,
    const geometry_msgs::msg::TransformStamped& world_T_lid_tag_msg,
    bool using_inner_tag)
  {
    const tf2::Transform world_T_base_tag = transformMsgToTf(world_T_base_tag_msg.transform);
    const tf2::Transform world_T_lid_tag = transformMsgToTf(world_T_lid_tag_msg.transform);

    const tf2::Transform base_tag_T_world = world_T_base_tag.inverse();
    const tf2::Transform base_tag_T_lid_tag = base_tag_T_world * world_T_lid_tag;

    const double hinge_angle = extractHingeAngle(base_tag_T_lid_tag);

    geometry_msgs::msg::TransformStamped dbg_tf;
    dbg_tf.header.stamp = now();
    dbg_tf.header.frame_id = base_tag_frame_;
    dbg_tf.child_frame_id = using_inner_tag ? "real_lid_tag_from_inner" : "real_lid_tag_from_outer";
    dbg_tf.transform = tfToTransformMsg(base_tag_T_lid_tag);
    tf_broadcaster_->sendTransform(dbg_tf);

    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = now();
    joint_state_msg.name = {hinge_joint_name_};
    joint_state_msg.position = {hinge_angle};
    joint_pub_->publish(joint_state_msg);

    RCLCPP_INFO(
      get_logger(),
      "RAW hinge_angle=%.6f clamp=[%.6f, %.6f]",
      hinge_angle, hinge_lower_, hinge_upper_);

    RCLCPP_INFO(
      get_logger(),
      "Publishing joint_state position=%.6f",
      joint_state_msg.position[0]);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Using %s lid tag, hinge_joint=%.3f rad (%.1f deg)",
      using_inner_tag ? "INNER" : "OUTER",
      hinge_angle, hinge_angle * 180.0 / M_PI);
  }

  void publishLaptopRoot(const geometry_msgs::msg::TransformStamped& world_T_base_tag_msg)
  {
    const tf2::Transform world_T_base_tag = transformMsgToTf(world_T_base_tag_msg.transform);
    const tf2::Transform base_tag_T_root = makeTransform(base_tag_to_root_xyz_, base_tag_to_root_rpy_);
    const tf2::Transform world_T_root = world_T_base_tag * base_tag_T_root;

    geometry_msgs::msg::TransformStamped root_tf;
    root_tf.header.stamp = now();
    root_tf.header.frame_id = world_frame_;
    root_tf.child_frame_id = laptop_root_frame_;
    root_tf.transform = tfToTransformMsg(world_T_root);
    tf_broadcaster_->sendTransform(root_tf);
  }

  void update()
  {
    geometry_msgs::msg::TransformStamped world_T_base_tag_msg;
    if (!lookupTransform(world_frame_, base_tag_frame_, world_T_base_tag_msg)) {
      return;
    }

    publishLaptopRoot(world_T_base_tag_msg);

    geometry_msgs::msg::TransformStamped world_T_lid_tag_msg;

    bool using_inner_tag =
      lookupTransform(world_frame_, lid_inner_tag_frame_, world_T_lid_tag_msg);

    if (using_inner_tag) {
      publishHingeJoint(world_T_base_tag_msg, world_T_lid_tag_msg, true);
      return;
    }

    bool using_outer_tag =
      lookupTransform(world_frame_, lid_outer_tag_frame_, world_T_lid_tag_msg);

    if (using_outer_tag) {
      publishHingeJoint(world_T_base_tag_msg, world_T_lid_tag_msg, false);
      return;
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "No lid tag visible: neither %s nor %s",
      lid_inner_tag_frame_.c_str(), lid_outer_tag_frame_.c_str());
  }

  std::string world_frame_;
  std::string laptop_root_frame_;
  std::string base_tag_frame_;
  std::string lid_inner_tag_frame_;
  std::string lid_outer_tag_frame_;
  std::string hinge_joint_name_;
  std::string hinge_axis_;

  double publish_rate_hz_;
  double hinge_lower_;
  double hinge_upper_;

  std::vector<double> base_tag_to_root_xyz_;
  std::vector<double> base_tag_to_root_rpy_;
  std::vector<double> lid_inner_tag_to_lid_xyz_;
  std::vector<double> lid_inner_tag_to_lid_rpy_;
  std::vector<double> lid_outer_tag_to_lid_xyz_;
  std::vector<double> lid_outer_tag_to_lid_rpy_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LaptopHingeStatePublisher>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}