#include "bimanual_laptop_mtc/task_builder.hpp"
#include "bimanual_laptop_mtc/tf_utils.hpp"

#include "bimanual_laptop_msgs/action/execute_laptop_task.hpp"

#include <moveit/task_constructor/task.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <thread>
#include <cmath>

namespace mtc = moveit::task_constructor;

using ExecuteLaptopTask = bimanual_laptop_msgs::action::ExecuteLaptopTask;

class ExecuteLaptopTaskServer : public rclcpp::Node
{
public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteLaptopTask>;

  ExecuteLaptopTaskServer()
  : Node("execute_laptop_task_server"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    this->declare_parameter("world_frame", "workspace_world");
    this->declare_parameter("laptop_frame_hint", "laptop_nominal");
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

    action_server_ = rclcpp_action::create_server<ExecuteLaptopTask>(
      this,
      "execute_laptop_task",
      std::bind(&ExecuteLaptopTaskServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ExecuteLaptopTaskServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&ExecuteLaptopTaskServer::handle_accepted, this, std::placeholders::_1));
  }

private:
  rclcpp_action::Server<ExecuteLaptopTask>::SharedPtr action_server_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const ExecuteLaptopTask::Goal> goal)
  {
    if (goal->task_name != "close" && goal->task_name != "close_single")
    {
      RCLCPP_WARN(this->get_logger(), "Only close/close_single are implemented in starter package");
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    auto goal = goal_handle->get_goal();

    bimanual_laptop_mtc::LaptopGeometry geom;
    geom.width = this->get_parameter("laptop_width").as_double();
    geom.depth = this->get_parameter("laptop_depth").as_double();
    geom.thickness_base = this->get_parameter("laptop_thickness_base").as_double();
    geom.thickness_lid = this->get_parameter("laptop_thickness_lid").as_double();
    geom.hinge_offset_x = this->get_parameter("laptop_hinge_offset_x").as_double();

    bimanual_laptop_mtc::TaskBuilderConfig cfg;
    cfg.world_frame = goal->world_frame.empty()
      ? this->get_parameter("world_frame").as_string()
      : goal->world_frame;
    cfg.arm_group = this->get_parameter("arm_group").as_string();
    cfg.eef_frame = this->get_parameter("eef_frame").as_string();
    cfg.lid_angle_start_rad = this->get_parameter("lid_angle_start_deg").as_double() * M_PI / 180.0;
    cfg.lid_angle_goal_rad = (
      goal->lid_angle_goal_deg > 0.0 ? goal->lid_angle_goal_deg
                                     : this->get_parameter("lid_angle_goal_default_deg").as_double()
    ) * M_PI / 180.0;
    cfg.pregrasp_offset_z = this->get_parameter("pregrasp_offset_z").as_double();
    cfg.step_translation_m = this->get_parameter("step_translation_m").as_double();
    cfg.step_count = this->get_parameter("step_count").as_int();
    cfg.retreat_distance_m = this->get_parameter("retreat_distance_m").as_double();
    cfg.table_size_x = this->get_parameter("table_size_x").as_double();
    cfg.table_size_y = this->get_parameter("table_size_y").as_double();
    cfg.table_size_z = this->get_parameter("table_size_z").as_double();
    cfg.table_pose_z = this->get_parameter("table_pose_z").as_double();

    const std::string laptop_frame_hint = goal->laptop_frame_hint.empty()
      ? this->get_parameter("laptop_frame_hint").as_string()
      : goal->laptop_frame_hint;

    auto base_pose = bimanual_laptop_mtc::lookupPoseOrNominal(
      shared_from_this(), tf_buffer_, cfg.world_frame, laptop_frame_hint);

    auto feedback = std::make_shared<ExecuteLaptopTask::Feedback>();
    feedback->current_state = "building_task";
    feedback->estimated_lid_angle_deg = cfg.lid_angle_start_rad * 180.0 / M_PI;
    goal_handle->publish_feedback(feedback);

    mtc::Task task;
    try
    {
      task = bimanual_laptop_mtc::buildCloseLaptopSingleArmTask(
        shared_from_this(), base_pose, geom, cfg);
    }
    catch (const std::exception& ex)
    {
      auto result = std::make_shared<ExecuteLaptopTask::Result>();
      result->success = false;
      result->message = std::string("Task construction failed: ") + ex.what();
      result->final_lid_angle_deg = cfg.lid_angle_start_rad * 180.0 / M_PI;
      goal_handle->abort(result);
      return;
    }

    feedback->current_state = "planning";
    goal_handle->publish_feedback(feedback);

    RCLCPP_INFO(this->get_logger(), "1: got goal");
    RCLCPP_INFO(this->get_logger(), "2: base pose resolved");
    RCLCPP_INFO(this->get_logger(), "3: building task");
    RCLCPP_INFO(this->get_logger(), "4: task built");
    RCLCPP_INFO(this->get_logger(), "5: calling init");
    RCLCPP_INFO(this->get_logger(), "6: init returned");
    RCLCPP_INFO(this->get_logger(), "7: calling plan");
    RCLCPP_INFO(this->get_logger(), "8: plan returned");
    RCLCPP_INFO(this->get_logger(), "9: calling execute");
    RCLCPP_INFO(this->get_logger(), "10: execute returned");

    try
    {
      task.init();
    }
    catch (const std::exception& ex)
    {
      auto result = std::make_shared<ExecuteLaptopTask::Result>();
      result->success = false;
      result->message = std::string("Task init failed: ") + ex.what();
      result->final_lid_angle_deg = cfg.lid_angle_start_rad * 180.0 / M_PI;
      goal_handle->abort(result);
      return;
    }

    const auto plan_result = task.plan(3);

    if (!plan_result || task.solutions().empty())
    {
      auto result = std::make_shared<ExecuteLaptopTask::Result>();
      result->success = false;
      result->message = "Task planning failed";
      result->final_lid_angle_deg = cfg.lid_angle_start_rad * 180.0 / M_PI;
      goal_handle->abort(result);
      return;
    }

    feedback->current_state = "executing";
    feedback->estimated_lid_angle_deg = cfg.lid_angle_goal_rad * 180.0 / M_PI;
    goal_handle->publish_feedback(feedback);

    RCLCPP_INFO(this->get_logger(), "1: got goal");
    RCLCPP_INFO(this->get_logger(), "2: base pose resolved");
    RCLCPP_INFO(this->get_logger(), "3: building task");
    RCLCPP_INFO(this->get_logger(), "4: task built");
    RCLCPP_INFO(this->get_logger(), "5: calling init");
    RCLCPP_INFO(this->get_logger(), "6: init returned");
    RCLCPP_INFO(this->get_logger(), "7: calling plan");
    RCLCPP_INFO(this->get_logger(), "8: plan returned");
    RCLCPP_INFO(this->get_logger(), "9: calling execute");
    RCLCPP_INFO(this->get_logger(), "10: execute returned");

    try
    {
      task.execute(*task.solutions().front());
    }
    catch (const std::exception& ex)
    {
      auto result = std::make_shared<ExecuteLaptopTask::Result>();
      result->success = false;
      result->message = std::string("Task execution failed: ") + ex.what();
      result->final_lid_angle_deg = cfg.lid_angle_start_rad * 180.0 / M_PI;
      goal_handle->abort(result);
      return;
    }

    auto result = std::make_shared<ExecuteLaptopTask::Result>();
    result->success = true;
    result->message = "close_laptop_single_arm completed";
    result->final_lid_angle_deg = cfg.lid_angle_goal_rad * 180.0 / M_PI;
    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(), "1: got goal");
    RCLCPP_INFO(this->get_logger(), "2: base pose resolved");
    RCLCPP_INFO(this->get_logger(), "3: building task");
    RCLCPP_INFO(this->get_logger(), "4: task built");
    RCLCPP_INFO(this->get_logger(), "5: calling init");
    RCLCPP_INFO(this->get_logger(), "6: init returned");
    RCLCPP_INFO(this->get_logger(), "7: calling plan");
    RCLCPP_INFO(this->get_logger(), "8: plan returned");
    RCLCPP_INFO(this->get_logger(), "9: calling execute");
    RCLCPP_INFO(this->get_logger(), "10: execute returned");
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