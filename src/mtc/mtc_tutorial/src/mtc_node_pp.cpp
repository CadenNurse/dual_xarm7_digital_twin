#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

namespace
{
constexpr char kNodeName[] = "mtc_node";
constexpr char kExecuteServiceName[] = "/execute_task";

constexpr char kWorldFrame[] = "workspace_origin";
constexpr char kTagFrame[] = "tag_21";
constexpr char kObjectId[] = "object";

constexpr char kArmGroup[] = "L_xarm7";
constexpr char kHandGroup[] = "L_xarm_gripper";
constexpr char kHandFrame[] = "L_link_tcp";
constexpr char kHomePose[] = "prepare_L";

constexpr double kVelocityScaling = 0.25;
constexpr double kAccelerationScaling = 0.25;
constexpr double kCartesianStepSize = 0.002;
constexpr std::size_t kMaxPlanSolutions = 35;
}  // namespace

class MTCTaskNode
{
public:
  explicit MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void setupPlanningScene();
  void doTask();

private:
  mtc::Task createTask();

  void executeCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  bool lookupObjectTransform(geometry_msgs::msg::TransformStamped& tf_tag) const;
  void resetPlanState();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_service_;

  mtc::Task task_;
  const mtc::SolutionBase* selected_solution_{ nullptr };
  bool plan_ready_{ false };
  std::mutex task_mutex_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_(std::make_shared<rclcpp::Node>(kNodeName, options))
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, true);

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
  selected_solution_ = nullptr;
  plan_ready_ = false;
}

bool MTCTaskNode::lookupObjectTransform(geometry_msgs::msg::TransformStamped& tf_tag) const
{
  if (!tf_buffer_)
  {
    RCLCPP_ERROR(LOGGER, "TF buffer is not initialized");
    return false;
  }

  if (!tf_buffer_->canTransform(
          kWorldFrame, kTagFrame, tf2::TimePointZero, tf2::durationFromSec(10.0)))
  {
    RCLCPP_ERROR(LOGGER, "Cannot transform from '%s' to '%s'", kWorldFrame, kTagFrame);
    return false;
  }

  try
  {
    tf_tag = tf_buffer_->lookupTransform(
        kWorldFrame, kTagFrame, tf2::TimePointZero, tf2::durationFromSec(10.0));
    return true;
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_ERROR(LOGGER, "Failed to look up AprilTag transform: %s", ex.what());
    return false;
  }
}

void MTCTaskNode::setupPlanningScene()
{
  geometry_msgs::msg::TransformStamped tf_tag;
  if (!lookupObjectTransform(tf_tag))
    return;

  moveit_msgs::msg::CollisionObject object;
  object.id = kObjectId;
  object.header.frame_id = kWorldFrame;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.12;
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.07;
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.025;

  const Eigen::Isometry3d world_T_tag = tf2::transformToEigen(tf_tag.transform);

  Eigen::Isometry3d tag_T_box = Eigen::Isometry3d::Identity();
  tag_T_box.translation().x() = -0.05;

  const Eigen::Isometry3d world_T_box = world_T_tag * tag_T_box;
  const geometry_msgs::msg::Pose box_pose = tf2::toMsg(world_T_box);

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(box_pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);

  rclcpp::sleep_for(std::chrono::milliseconds(1000));

  RCLCPP_INFO(
      LOGGER,
      "Applied collision object '%s' at [x=%.3f, y=%.3f, z=%.3f] in frame '%s'",
      kObjectId,
      box_pose.position.x,
      box_pose.position.y,
      box_pose.position.z,
      kWorldFrame);
}

void MTCTaskNode::doTask()
{
  std::lock_guard<std::mutex> lock(task_mutex_);

  resetPlanState();
  task_ = createTask();

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

  const auto& solutions = task_.solutions();
  if (solutions.empty())
  {
    RCLCPP_ERROR(LOGGER, "Planning succeeded, but no solutions were stored");
    return;
  }

  selected_solution_ = solutions.front().get();
  if (!selected_solution_)
  {
    RCLCPP_ERROR(LOGGER, "Selected solution is null");
    return;
  }

  task_.introspection().publishSolution(*selected_solution_);
  plan_ready_ = true;

  RCLCPP_INFO(
      LOGGER,
      "Plan ready: %zu solution(s) found, selected first solution with cost %.6f",
      solutions.size(),
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
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  task.setProperty("group", kArmGroup);
  task.setProperty("eef", kHandGroup);
  task.setProperty("ik_frame", kHandFrame);

  mtc::Stage* current_state_ptr = nullptr;
  mtc::Stage* attach_object_stage = nullptr;

  auto current_state = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = current_state.get();
  task.add(std::move(current_state));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();

  sampling_planner->setPlannerId("ompl", "RRTConnect");
  sampling_planner->setProperty("max_velocity_scaling_factor", kVelocityScaling);
  sampling_planner->setProperty("max_acceleration_scaling_factor", kAccelerationScaling);

  cartesian_planner->setMaxVelocityScalingFactor(kVelocityScaling);
  cartesian_planner->setMaxAccelerationScalingFactor(kAccelerationScaling);
  cartesian_planner->setStepSize(kCartesianStepSize);

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    stage->setGroup(kHandGroup);
    stage->setGoal("open");
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "move to pick",
        mtc::stages::Connect::GroupPlannerVector{{kArmGroup, sampling_planner}});
    stage->setTimeout(20.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage));
  }

  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject(kObjectId);
      stage->setMonitoredStage(current_state_ptr);
      stage->setAngleDelta(M_PI);

      Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
      grasp_frame_transform.translation() = Eigen::Vector3d(0.0, 0.0, 0.10);
      grasp_frame_transform.linear() =
          (Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitZ()))
              .toRotationMatrix();

      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setIKFrame(grasp_frame_transform, kHandFrame);
      wrapper->setMaxIKSolutions(40);
      wrapper->setMinSolutionDistance(0.05);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});

      grasp->insert(std::move(wrapper));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("insert grasp", cartesian_planner);
      stage->properties().set("marker_ns", "insert_grasp");
      stage->properties().set("link", kHandFrame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.06, 0.08);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = kObjectId;
      vec.vector.x = -1.0;
      stage->setDirection(vec);

      grasp->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions(
          kObjectId,
          task.getRobotModel()
              ->getJointModelGroup(kHandGroup)
              ->getLinkModelNamesWithCollisionGeometry(),
          true);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(kHandGroup);
      stage->setGoal("close");
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(kObjectId, kHandFrame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.18, 0.20);
      stage->setIKFrame(kHandFrame);
      stage->properties().set("marker_ns", "lift_object");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = kWorldFrame;
      vec.vector.x = 1.0;
      stage->setDirection(vec);

      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }

  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{{kArmGroup, sampling_planner}});
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage));
  }

  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
    place->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    {
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject(kObjectId);
      stage->setTimeout(3.0);

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = kWorldFrame;
      target_pose_msg.pose.position.x = 0.062;
      target_pose_msg.pose.position.y = 0.15;
      target_pose_msg.pose.position.z = 0.15;
      target_pose_msg.pose.orientation.w = 1.0;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage);

      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(40);
      wrapper->setMinSolutionDistance(0.05);
      wrapper->setIKFrame("object");
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});

      place->insert(std::move(wrapper));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::MoveTo>("open hand place", interpolation_planner);
      stage->setGroup(kHandGroup);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions(
          kObjectId,
          task.getRobotModel()
              ->getJointModelGroup(kHandGroup)
              ->getLinkModelNamesWithCollisionGeometry(),
          false);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject(kObjectId, kHandFrame);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.08, 0.08);
      stage->setIKFrame(kHandFrame);
      stage->properties().set("marker_ns", "retreat");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = kObjectId;
      vec.vector.x = 1.0;
      stage->setDirection(vec);

      place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
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

  rclcpp::sleep_for(std::chrono::seconds(1));

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  RCLCPP_INFO(LOGGER, "Node is alive and waiting for service calls.");
  RCLCPP_INFO(LOGGER, "Execute from another terminal with:");
  RCLCPP_INFO(LOGGER, "ros2 service call %s std_srvs/srv/Trigger", kExecuteServiceName);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}