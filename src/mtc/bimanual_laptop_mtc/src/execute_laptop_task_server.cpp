#include "bimanual_laptop_mtc/task_builder.hpp"
#include "bimanual_laptop_mtc/tf_utils.hpp"

#include "bimanual_laptop_msgs/action/execute_laptop_task.hpp"

#include <moveit/task_constructor/task.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <memory>
#include <string>
#include <thread>

namespace mtc = moveit::task_constructor;
using ExecuteLaptopTask = bimanual_laptop_msgs::action::ExecuteLaptopTask;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("execute_laptop_task_server");

namespace
{
double deg2rad(double deg) { return deg * M_PI / 180.0; }
double rad2deg(double rad) { return rad * 180.0 / M_PI; }
}  // namespace

class ExecuteLaptopTaskServer : public rclcpp::Node
{
public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteLaptopTask>;

  ExecuteLaptopTaskServer()
  : Node("execute_laptop_task_server"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    declareParameters();

    action_server_ = rclcpp_action::create_server<ExecuteLaptopTask>(
      this,
      "execute_laptop_task",
      std::bind(&ExecuteLaptopTaskServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ExecuteLaptopTaskServer::handleCancel, this, std::placeholders::_1),
      std::bind(&ExecuteLaptopTaskServer::handleAccepted, this, std::placeholders::_1));
  }

private:
  rclcpp_action::Server<ExecuteLaptopTask>::SharedPtr action_server_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  void declareParameters()
  {
    this->declare_parameter("world_frame", "world");
    this->declare_parameter("object_frame_hint", "object_nominal");
    this->declare_parameter("arm_group", "right_xarm7");
    this->declare_parameter("eef_frame", "right_gripper_tcp");

    this->declare_parameter("laptop_width", 0.320);
    this->declare_parameter("laptop_depth", 0.220);
    this->declare_parameter("laptop_thickness_base", 0.018);
    this->declare_parameter("laptop_thickness_lid", 0.008);
    this->declare_parameter("laptop_hinge_offset_x", -0.140);

    this->declare_parameter("lid_angle_start_deg", 70.0);
    this->declare_parameter("lid_angle_goal_default_deg", 15.0);
    this->declare_parameter("pregrasp_offset_z", 0.060);
    this->declare_parameter("step_translation_m", 0.008);
    this->declare_parameter("step_count", 6);
    this->declare_parameter("retreat_distance_m", 0.060);

    this->declare_parameter("table_size_x", 1.20);
    this->declare_parameter("table_size_y", 0.80);
    this->declare_parameter("table_size_z", 0.04);
    this->declare_parameter("table_pose_z", -0.02);
  }

  bimanual_laptop_mtc::LaptopGeometry loadGeometry() const
  {
    bimanual_laptop_mtc::LaptopGeometry geom;
    geom.width = this->get_parameter("laptop_width").as_double();
    geom.depth = this->get_parameter("laptop_depth").as_double();
    geom.thickness_base = this->get_parameter("laptop_thickness_base").as_double();
    geom.thickness_lid = this->get_parameter("laptop_thickness_lid").as_double();
    geom.hinge_offset_x = this->get_parameter("laptop_hinge_offset_x").as_double();
    return geom;
  }

  bimanual_laptop_mtc::TaskBuilderConfig loadConfig(
    const std::shared_ptr<const ExecuteLaptopTask::Goal>& goal) const
  {
    bimanual_laptop_mtc::TaskBuilderConfig cfg;
    cfg.world_frame = goal->world_frame.empty()
      ? this->get_parameter("world_frame").as_string()
      : goal->world_frame;

    cfg.arm_group = this->get_parameter("arm_group").as_string();
    cfg.eef_frame = this->get_parameter("eef_frame").as_string();
    cfg.lid_angle_start_rad = deg2rad(this->get_parameter("lid_angle_start_deg").as_double());

    const double goal_deg = goal->lid_angle_goal_deg > 0.0
      ? goal->lid_angle_goal_deg
      : this->get_parameter("lid_angle_goal_default_deg").as_double();
    cfg.lid_angle_goal_rad = deg2rad(goal_deg);

    cfg.pregrasp_offset_z = this->get_parameter("pregrasp_offset_z").as_double();
    cfg.step_translation_m = this->get_parameter("step_translation_m").as_double();
    cfg.step_count = this->get_parameter("step_count").as_int();
    cfg.retreat_distance_m = this->get_parameter("retreat_distance_m").as_double();

    cfg.table_size_x = this->get_parameter("table_size_x").as_double();
    cfg.table_size_y = this->get_parameter("table_size_y").as_double();
    cfg.table_size_z = this->get_parameter("table_size_z").as_double();
    cfg.table_pose_z = this->get_parameter("table_pose_z").as_double();

    return cfg;
  }

  std::string resolveObjectFrameHint(
    const std::shared_ptr<const ExecuteLaptopTask::Goal>& goal) const
  {
    return goal->object_frame_hint.empty()
      ? this->get_parameter("object_frame_hint").as_string()
      : goal->object_frame_hint;
  }

  void publishFeedback(
    const std::shared_ptr<GoalHandle>& goal_handle,
    const std::string& state,
    double estimated_angle_deg)
  {
    auto feedback = std::make_shared<ExecuteLaptopTask::Feedback>();
    feedback->current_state = state;
    feedback->estimated_lid_angle_deg = estimated_angle_deg;
    goal_handle->publish_feedback(feedback);
  }

  void abortGoal(
    const std::shared_ptr<GoalHandle>& goal_handle,
    const std::string& message,
    double final_angle_deg)
  {
    auto result = std::make_shared<ExecuteLaptopTask::Result>();
    result->success = false;
    result->message = message;
    result->final_lid_angle_deg = final_angle_deg;
    goal_handle->abort(result);
  }

  void succeedGoal(
    const std::shared_ptr<GoalHandle>& goal_handle,
    const std::string& message,
    double final_angle_deg)
  {
    auto result = std::make_shared<ExecuteLaptopTask::Result>();
    result->success = true;
    result->message = message;
    result->final_lid_angle_deg = final_angle_deg;
    goal_handle->succeed(result);
  }

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const ExecuteLaptopTask::Goal> goal)
  {
    if (goal->task_name != "close" && goal->task_name != "close_single")
      RCLCPP_WARN(LOGGER, "Only close/close_single are currently implemented");

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_INFO(LOGGER, "Cancel request received");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread{std::bind(&ExecuteLaptopTaskServer::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    const auto geom = loadGeometry();
    const auto cfg = loadConfig(goal);
    const auto object_frame_hint = resolveObjectFrameHint(goal);
    const double start_angle_deg = rad2deg(cfg.lid_angle_start_rad);
    const double goal_angle_deg = rad2deg(cfg.lid_angle_goal_rad);

    RCLCPP_INFO(LOGGER, "Received task '%s' with world_frame='%s' and object_frame_hint='%s'",
                goal->task_name.c_str(), cfg.world_frame.c_str(), object_frame_hint.c_str());

    auto base_pose = bimanual_laptop_mtc::lookupPoseOrNominal(
      shared_from_this(), tf_buffer_, cfg.world_frame, object_frame_hint);

    publishFeedback(goal_handle, "building_task", start_angle_deg);

    mtc::Task task;
    try
    {
      RCLCPP_INFO(LOGGER, "Building task");
      task = bimanual_laptop_mtc::buildCloseLaptopSingleArmTask(
        shared_from_this(), base_pose, geom, cfg);
    }
    catch (const std::exception& ex)
    {
      abortGoal(goal_handle, std::string("Task construction failed: ") + ex.what(), start_angle_deg);
      return;
    }

    try
    {
      RCLCPP_INFO(LOGGER, "Initializing task");
      task.init();
    }
    catch (const std::exception& ex)
    {
      abortGoal(goal_handle, std::string("Task init failed: ") + ex.what(), start_angle_deg);
      return;
    }

    publishFeedback(goal_handle, "planning", start_angle_deg);

    RCLCPP_INFO(LOGGER, "Planning task");
    const bool planned = static_cast<bool>(task.plan(3));
    if (!planned || task.solutions().empty())
    {
      abortGoal(goal_handle, "Task planning failed", start_angle_deg);
      return;
    }

    publishFeedback(goal_handle, "executing", goal_angle_deg);

    try
    {
      RCLCPP_INFO(LOGGER, "Executing task");
      task.execute(*task.solutions().front());
    }
    catch (const std::exception& ex)
    {
      abortGoal(goal_handle, std::string("Task execution failed: ") + ex.what(), start_angle_deg);
      return;
    }

    RCLCPP_INFO(LOGGER, "Task completed successfully");
    succeedGoal(goal_handle, "close_laptop_single_arm completed", goal_angle_deg);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExecuteLaptopTaskServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}