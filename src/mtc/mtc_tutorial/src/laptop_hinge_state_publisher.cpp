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
    laptop_root_frame_ = declare_parameter<std::string>("laptop_root_frame", "laptop_world");
    base_tag_frame_ = declare_parameter<std::string>("base_tag_frame", "tag_laptop_base");
    lid_inner_tag_frame_ = declare_parameter<std::string>("lid_inner_tag_frame", "tag_laptop_lid_inner");
    lid_outer_tag_frame_ = declare_parameter<std::string>("lid_outer_tag_frame", "tag_laptop_lid_outer");
    hinge_joint_name_ = declare_parameter<std::string>("hinge_joint_name", "hinge_joint");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
    hinge_lower_ = declare_parameter<double>("hinge_lower", 0.0);
    hinge_upper_ = declare_parameter<double>("hinge_upper", 3.1416);

    base_tag_to_root_xyz_ = declare_parameter<std::vector<double>>(
      "base_tag_to_root_xyz", std::vector<double>{0.0, 0.0, 0.0});
    base_tag_to_root_rpy_ = declare_parameter<std::vector<double>>(
      "base_tag_to_root_rpy", std::vector<double>{0.0, 0.0, 0.0});

    validateVectorParam(base_tag_to_root_xyz_, "base_tag_to_root_xyz");
    validateVectorParam(base_tag_to_root_rpy_, "base_tag_to_root_rpy");

    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/laptop/joint_states", 10);

    RCLCPP_INFO(get_logger(), "Laptop hinge state publisher constructed");
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

  double unwrapToNearest(double angle, double reference) const
  {
    while (angle - reference > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle - reference < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  double extractHingeAngle(const tf2::Transform& base_tag_T_lid_tag, bool using_inner_tag) const
  {
    const tf2::Matrix3x3 R(base_tag_T_lid_tag.getRotation());

    const tf2::Vector3 hinge_axis_tag(0.0, -1.0, 0.0);
    const tf2::Vector3 ref_tag(1.0, 0.0, 0.0);

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

    if (!using_inner_tag) {
      angle -= M_PI;
    }

    while (angle < 0.0) {
      angle += 2.0 * M_PI;
    }
    while (angle >= 2.0 * M_PI) {
      angle -= 2.0 * M_PI;
    }

    return angle;
  }

  void publishLaptopRoot(const geometry_msgs::msg::TransformStamped& world_T_base_tag_msg)
  {
    const tf2::Transform world_T_base_tag = transformMsgToTf(world_T_base_tag_msg.transform);
    const tf2::Transform base_tag_T_root = makeTransform(base_tag_to_root_xyz_, base_tag_to_root_rpy_);
    const tf2::Transform world_T_root = world_T_base_tag * base_tag_T_root;

    geometry_msgs::msg::TransformStamped root_tf;
    root_tf.header.stamp = world_T_base_tag_msg.header.stamp;
    root_tf.header.frame_id = world_frame_;
    root_tf.child_frame_id = laptop_root_frame_;
    root_tf.transform = tfToTransformMsg(world_T_root);

    tf_broadcaster_->sendTransform(root_tf);
  }

  void publishAngle(const geometry_msgs::msg::TransformStamped& world_T_base_tag_msg,
                    const geometry_msgs::msg::TransformStamped& world_T_lid_tag_msg,
                    bool using_inner_tag)
  {
    const tf2::Transform world_T_base_tag = transformMsgToTf(world_T_base_tag_msg.transform);
    const tf2::Transform world_T_lid_tag = transformMsgToTf(world_T_lid_tag_msg.transform);

    const tf2::Transform base_tag_T_lid_tag = world_T_base_tag.inverse() * world_T_lid_tag;

    double hinge_angle = extractHingeAngle(base_tag_T_lid_tag, using_inner_tag);

    if (has_last_hinge_angle_) {
      hinge_angle = unwrapToNearest(hinge_angle, last_hinge_angle_);
    }

    hinge_angle = std::clamp(hinge_angle, hinge_lower_, hinge_upper_);
    last_hinge_angle_ = hinge_angle;
    has_last_hinge_angle_ = true;

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name = {hinge_joint_name_};
    msg.position = {hinge_angle};
    joint_pub_->publish(msg);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Using %s tag, hinge_joint=%.3f rad (%.1f deg)",
      using_inner_tag ? "INNER" : "OUTER",
      hinge_angle, hinge_angle * 180.0 / M_PI);
  }

  void update()
  {
    geometry_msgs::msg::TransformStamped world_T_base_tag_msg;
    geometry_msgs::msg::TransformStamped world_T_lid_tag_msg;

    const bool have_base = lookupTransform(world_frame_, base_tag_frame_, world_T_base_tag_msg);
    if (have_base) {
      last_base_tag_msg_ = world_T_base_tag_msg;
      has_last_base_tag_ = true;
      publishLaptopRoot(world_T_base_tag_msg);
    } else if (has_last_base_tag_) {
      publishLaptopRoot(last_base_tag_msg_);
    }

    const bool have_inner = lookupTransform(world_frame_, lid_inner_tag_frame_, world_T_lid_tag_msg);
    if (have_base && have_inner) {
      publishAngle(world_T_base_tag_msg, world_T_lid_tag_msg, true);
      return;
    }
    if (!have_base && has_last_base_tag_ && have_inner) {
      publishAngle(last_base_tag_msg_, world_T_lid_tag_msg, true);
      return;
    }

    const bool have_outer = lookupTransform(world_frame_, lid_outer_tag_frame_, world_T_lid_tag_msg);
    if (have_base && have_outer) {
      publishAngle(world_T_base_tag_msg, world_T_lid_tag_msg, false);
      return;
    }
    if (!have_base && has_last_base_tag_ && have_outer) {
      publishAngle(last_base_tag_msg_, world_T_lid_tag_msg, false);
      return;
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "No usable tag combination visible");
  }

  std::string world_frame_;
  std::string laptop_root_frame_;
  std::string base_tag_frame_;
  std::string lid_inner_tag_frame_;
  std::string lid_outer_tag_frame_;
  std::string hinge_joint_name_;

  std::vector<double> base_tag_to_root_xyz_;
  std::vector<double> base_tag_to_root_rpy_;

  double publish_rate_hz_;
  double hinge_lower_;
  double hinge_upper_;

  double last_hinge_angle_ = 0.0;
  bool has_last_hinge_angle_ = false;

  geometry_msgs::msg::TransformStamped last_base_tag_msg_;
  bool has_last_base_tag_ = false;

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