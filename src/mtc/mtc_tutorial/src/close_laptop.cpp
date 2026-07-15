#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/stages/fixed_cartesian_poses.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>
#include <sensor_msgs/msg/joint_state.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <limits>
#include <algorithm>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("close_laptop_mtc");
namespace mtc = moveit::task_constructor;

namespace
{
constexpr char kNodeName[] = "close_laptop_mtc_node";
constexpr char kExecuteServiceName[] = "/execute_close_laptop";

constexpr char kWorldFrame[] = "workspace_origin";
constexpr char kLaptopBaseTagFrame[] = "tag_laptop_base";
constexpr char kLaptopScreenTagFrame[] = "tag_laptop_lid_inner";

constexpr char kArmGroup[] = "L_xarm7";
constexpr char kHandGroup[] = "L_xarm_gripper";
constexpr char kHandFrame[] = "L_link_tcp";
constexpr char kHomePose[] = "prepare_L";

constexpr std::size_t kMaxPlanSolutions = 35;

constexpr double klateralY = 0.0;
constexpr double kverticleZ = 0.08;
constexpr double knormalX = -0.04;

constexpr double kTargetAngleRad = 0.30;
constexpr double kHingeToContactM = 0.14;
constexpr double kMaxPushDistanceM = 0.20;

constexpr double kStartupDelaySec = 2.0;
constexpr double kWaitForAngleTimeoutSec = 5.0;
}  // namespace

struct LaptopState
{
  bool have_angle{ false };
  double lid_angle_rad{ 0.0 };
};

class MTCTaskNode
{
public:
  explicit MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void doTask();

private:
  mtc::Task createTask();
  void resetPlanState();
  bool getLaptopTargetPose(geometry_msgs::msg::PoseStamped& target_pose, double& push_distance) const;

  void executeCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr hinge_state_sub_;
  rclcpp::TimerBase::SharedPtr startup_timer_;

  mutable std::mutex data_mutex_;
  LaptopState laptop_state_;

  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_service_;
  std::mutex task_mutex_;
  bool plan_ready_{ false };
  const mtc::SolutionBase* selected_solution_{ nullptr };
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>(kNodeName, options) }
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, true);

  hinge_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/laptop/joint_states",
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg)
      {
        if (msg->position.empty())
        {
          RCLCPP_WARN_THROTTLE(
              LOGGER, *node_->get_clock(), 2000,
              "/laptop/joint_states received with empty position array");
          return;
        }

        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          laptop_state_.lid_angle_rad = msg->position[0];
          laptop_state_.have_angle = true;
        }

        RCLCPP_INFO_THROTTLE(
            LOGGER, *node_->get_clock(), 3000,
            "hinge_joint angle = %.6f rad", msg->position[0]);
      });

  startup_timer_ = node_->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(kStartupDelaySec)),
      [this]()
      {
        if (startup_timer_)
          startup_timer_->cancel();
        this->doTask();
      });

  execute_service_ = node_->create_service<std_srvs::srv::Trigger>(
      kExecuteServiceName,
      std::bind(&MTCTaskNode::executeCallback, this, std::placeholders::_1, std::placeholders::_2));
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::resetPlanState()
{
  plan_ready_ = false;
  selected_solution_ = nullptr;
}

bool MTCTaskNode::getLaptopTargetPose(geometry_msgs::msg::PoseStamped& target_pose,
                                      double& push_distance) const
{
  if (!tf_buffer_)
  {
    RCLCPP_ERROR(LOGGER, "TF buffer is not initialized");
    return false;
  }

  if (!tf_buffer_->canTransform(
          kWorldFrame, kLaptopScreenTagFrame, tf2::TimePointZero, tf2::durationFromSec(2.0)))
  {
    RCLCPP_ERROR(LOGGER, "Cannot transform '%s' -> '%s'", kWorldFrame, kLaptopScreenTagFrame);
    return false;
  }

  if (!tf_buffer_->canTransform(
          kWorldFrame, kLaptopBaseTagFrame, tf2::TimePointZero, tf2::durationFromSec(2.0)))
  {
    RCLCPP_ERROR(LOGGER, "Cannot transform '%s' -> '%s'", kWorldFrame, kLaptopBaseTagFrame);
    return false;
  }

  geometry_msgs::msg::TransformStamped tf_screen, tf_base;
  try
  {
    tf_screen = tf_buffer_->lookupTransform(
        kWorldFrame, kLaptopScreenTagFrame, tf2::TimePointZero, tf2::durationFromSec(2.0));
    tf_base = tf_buffer_->lookupTransform(
        kWorldFrame, kLaptopBaseTagFrame, tf2::TimePointZero, tf2::durationFromSec(2.0));
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_ERROR(LOGGER, "Laptop TF lookup failed: %s", ex.what());
    return false;
  }

  LaptopState snapshot;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    snapshot = laptop_state_;
  }

  if (!snapshot.have_angle)
  {
    RCLCPP_ERROR(LOGGER, "No lid angle received yet on /laptop/joint_states");
    return false;
  }

  const Eigen::Isometry3d T_screen = tf2::transformToEigen(tf_screen.transform); // pose of screen tag in world frame
  const Eigen::Isometry3d T_base = tf2::transformToEigen(tf_base.transform); // pose of base tag in world frame

  const Eigen::Vector3d tag_x_world = T_screen.rotation().col(0); // tag local x axis as seen in rviz
  const Eigen::Vector3d tag_y_world = T_screen.rotation().col(1); // tag local y axis as seen in rviz
  const Eigen::Vector3d tag_z_world = T_screen.rotation().col(2); // tag local z axis as seen in rviz

  Eigen::Isometry3d T_target = T_screen; // offset pose
  T_target.translation() +=
      tag_x_world * knormalX + // shift by _x
      tag_y_world * klateralY + // shift by _y
      tag_z_world * kverticleZ; // shift by _z

  // tool orientation
  Eigen::Vector3d tool_z = tag_x_world.normalized(); // tool z-axis should point towards tag +x-axis
  Eigen::Vector3d tool_y = tag_y_world.normalized(); // tool y-axis should point towards tag +y-axis
  Eigen::Vector3d tool_x = tool_y.cross(tool_z).normalized(); // tool x-axis should point towards tag +z-axis
  tool_y = tool_z.cross(tool_x).normalized(); // recompute y to ensure mutually orthogonal
  

  // // if pushing along tool x axis
  // tool_x = tag_x_world.normalized();
  // tool_y = tag_y_world.normalized();
  // tool_z = tool_x.cross(tool_y).normalized();
  // tool_y = tool_z.cross(tool_x).normalized();

  Eigen::Matrix3d R_target; // replace rotation parts of T_target with tool frame 
  R_target.col(0) = tool_x;
  R_target.col(1) = tool_y;
  R_target.col(2) = tool_z;

  Eigen::Matrix3d R_flip =
    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  Eigen::Matrix3d R_tilt =
    Eigen::AngleAxisd(-30.0 * M_PI / 180.0, Eigen::Vector3d::UnitY()).toRotationMatrix();

  T_target.linear() = R_target * R_flip * R_tilt;

  target_pose.header.frame_id = kWorldFrame; // Eigen to ROS PoseStamped
  target_pose.header.stamp = node_->now();
  target_pose.pose = tf2::toMsg(T_target);

  // calc to find out how much more it must move before "close enough"
  const double remaining_angle = std::max(0.0, snapshot.lid_angle_rad - kTargetAngleRad); 
  push_distance = std::clamp(kHingeToContactM * remaining_angle, 0.0, kMaxPushDistanceM);

  RCLCPP_INFO(
      LOGGER,
      "tag_x_world=[%.3f, %.3f, %.3f] tag_y_world=[%.3f, %.3f, %.3f] tag_z_world=[%.3f, %.3f, %.3f]",
      tag_x_world.x(), tag_x_world.y(), tag_x_world.z(),
      tag_y_world.x(), tag_y_world.y(), tag_y_world.z(),
      tag_z_world.x(), tag_z_world.y(), tag_z_world.z());

  RCLCPP_INFO(
      LOGGER,
      "Laptop target pose computed. lid_angle=%.4f rad, remaining=%.4f rad, push_distance=%.4f m",
      snapshot.lid_angle_rad, remaining_angle, push_distance);

  (void)T_base;
  return true;
}

void MTCTaskNode::doTask()
{
  std::lock_guard<std::mutex> lock(task_mutex_);

  const auto timeout = node_->now() + rclcpp::Duration::from_seconds(kWaitForAngleTimeoutSec);
  while (rclcpp::ok())
  {
    {
      std::lock_guard<std::mutex> data_lock(data_mutex_);
      if (laptop_state_.have_angle)
        break;
    }

    if (node_->now() > timeout)
    {
      RCLCPP_ERROR(LOGGER, "Timed out waiting for first /laptop/joint_states message");
      return;
    }

    rclcpp::sleep_for(std::chrono::milliseconds(20));
  }

  resetPlanState();

  try
  {
    task_ = createTask();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(LOGGER, "Failed to build task: %s", e.what());
    return;
  }

  try
  {
    task_.init();
  }
  catch (const mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(kMaxPlanSolutions))
  {
    RCLCPP_ERROR(LOGGER, "Task planning failed");
    return;
  }

  const auto& sols = task_.solutions();
  if (sols.empty())
  {
    RCLCPP_ERROR(LOGGER, "Planning reported success, but no solutions were stored");
    return;
  }

  selected_solution_ = sols.front().get();
  if (!selected_solution_)
  {
    RCLCPP_ERROR(LOGGER, "Selected solution is null");
    return;
  }

  task_.introspection().publishSolution(*selected_solution_);
  plan_ready_ = true;

  RCLCPP_INFO(
      LOGGER,
      "Plan ready: %zu solution(s) found, selected first solution with mtc_cost=%.6f",
      sols.size(),
      selected_solution_->cost());
  RCLCPP_INFO(LOGGER, "Execute with:");
  RCLCPP_INFO(LOGGER, "ros2 service call %s std_srvs/srv/Trigger", kExecuteServiceName);
}

void MTCTaskNode::executeCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  std::lock_guard<std::mutex> lock(task_mutex_);

  if (!plan_ready_)
  {
    response->success = false;
    response->message = "No plan is ready yet";
    RCLCPP_WARN(LOGGER, "Execute requested, but no plan is ready");
    return;
  }

  if (task_.solutions().empty())
  {
    response->success = false;
    response->message = "Task has no stored solutions";
    RCLCPP_WARN(LOGGER, "Execute requested, but task_.solutions() is empty");
    resetPlanState();
    return;
  }

  if (!selected_solution_)
  {
    response->success = false;
    response->message = "No selected solution is available";
    RCLCPP_WARN(LOGGER, "Execute requested, but selected_solution_ is null");
    resetPlanState();
    return;
  }

  RCLCPP_INFO(
      LOGGER,
      "Execute service called. Executing selected solution with cost %.6f",
      selected_solution_->cost());

  const auto result = task_.execute(*selected_solution_);

  if (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    response->success = true;
    response->message = "Execution succeeded";
    RCLCPP_INFO(LOGGER, "Task execution succeeded");
    return;
  }

  response->success = false;
  response->message = "Task execution failed";
  RCLCPP_ERROR(LOGGER, "Task execution failed with code %d", result.val);
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("close laptop by push");
  task.loadRobotModel(node_);

  task.setProperty("group", std::string(kArmGroup));
  task.setProperty("eef", std::string(kHandGroup));
  task.setProperty("ik_frame", std::string(kHandFrame));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();

  sampling_planner->setPlannerId("ompl", "RRTConnect");
  sampling_planner->setProperty("max_velocity_scaling_factor", 0.20);
  sampling_planner->setProperty("max_acceleration_scaling_factor", 0.20);

  cartesian_planner->setMaxVelocityScalingFactor(0.10);
  cartesian_planner->setMaxAccelerationScalingFactor(0.10);
  cartesian_planner->setStepSize(0.002);

  auto current_state_stage = std::make_unique<mtc::stages::CurrentState>("current");
  mtc::Stage* current_state_ptr = current_state_stage.get();
  task.add(std::move(current_state_stage));

  geometry_msgs::msg::PoseStamped preclose_pose;
  double push_distance = 0.0;
  if (!getLaptopTargetPose(preclose_pose, push_distance))
  {
    throw std::runtime_error("Failed to compute laptop target pose from AprilTag/angle data");
  }

  // {
  //   auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  //   stage->setGroup(kHandGroup);
  //   stage->setGoal("open");
  //   task.add(std::move(stage));
  // }

  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "move near laptop",
        mtc::stages::Connect::GroupPlannerVector{ { kArmGroup, sampling_planner } });
    stage->setTimeout(15.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage));
  }

  {
    auto fixed = std::make_unique<mtc::stages::FixedCartesianPoses>("pre-close approach pose");
    fixed->properties().set("marker_ns", "preclose_pose");
    fixed->setMonitoredStage(current_state_ptr);
    fixed->addPose(preclose_pose);

    auto ik = std::make_unique<mtc::stages::ComputeIK>("pre-close IK", std::move(fixed));
    ik->setMaxIKSolutions(16);
    ik->setMinSolutionDistance(0.05);
    ik->setIKFrame(kHandFrame);
    ik->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });
    ik->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
    task.add(std::move(ik));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("closing lid push", cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setIKFrame(kHandFrame);
    stage->properties().set("marker_ns", "push_close");

    const double min_dist = std::max(0.0, push_distance * 0.8);
    stage->setMinMaxDistance(min_dist, push_distance);

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = kHandFrame;
    vec.vector.z = 1.0;
    stage->setDirection(vec);

    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal(kHomePose);
    task.add(std::move(stage));
  }

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(mtc_task_node->getNodeBaseInterface());

  RCLCPP_INFO(LOGGER, "Node is alive and waiting for startup timer / service calls.");
  RCLCPP_INFO(LOGGER, "Execute from another terminal with:");
  RCLCPP_INFO(LOGGER, "ros2 service call /execute_close_laptop std_srvs/srv/Trigger");

  executor.spin();

  rclcpp::shutdown();
  return 0;
}