#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <mutex>
#include <limits>
#include <memory>
#include <string>

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

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void doTask();
  void setupPlanningScene();

private:
  mtc::Task createTask();
  const mtc::SolutionBase* selectBestSolution() const;
  void logAllSolutionCosts() const;

  void executeCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_service_;
  std::mutex task_mutex_;
  bool plan_ready_{ false };
  const mtc::SolutionBase* selected_solution_{ nullptr };
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
  execute_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "/execute_task",
      std::bind(&MTCTaskNode::executeCallback, this, std::placeholders::_1, std::placeholders::_2));
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

const mtc::SolutionBase* MTCTaskNode::selectBestSolution() const
{
  const mtc::SolutionBase* best = nullptr;
  double best_cost = std::numeric_limits<double>::infinity();

  for (const auto& sol : task_.solutions())
  {
    if (!sol)
      continue;

    const double cost = sol->cost();
    if (!best || cost < best_cost)
    {
      best = sol.get();
      best_cost = cost;
    }
  }

  return best;
}

void MTCTaskNode::logAllSolutionCosts() const
{
  std::size_t i = 0;
  for (const auto& sol : task_.solutions())
  {
    if (!sol)
    {
      RCLCPP_WARN(LOGGER, "Solution[%zu] is null", i);
    }
    else
    {
      RCLCPP_INFO(LOGGER, "Solution[%zu] cost = %.6f", i, sol->cost());
    }
    ++i;
  }
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
    return;
  }

  if (!selected_solution_)
  {
    response->success = false;
    response->message = "No selected solution is available";
    RCLCPP_WARN(LOGGER, "Execute requested, but selected_solution_ is null");
    return;
  }

  RCLCPP_INFO(LOGGER,
              "Execute service called. Executing selected solution with cost %.6f",
              selected_solution_->cost());

  auto result = task_.execute(*selected_solution_);

  if (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    response->success = true;
    response->message = "Execution succeeded";
    RCLCPP_INFO(LOGGER, "Task execution succeeded");
  }
  else
  {
    response->success = false;
    response->message = "Task execution failed";
    RCLCPP_ERROR(LOGGER, "Task execution failed with code %d", result.val);
  }
}

void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.1, 0.02 };

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.25;
  pose.position.y = 0.35;
  pose.position.z = 0.05;
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

void MTCTaskNode::doTask()
{
  std::lock_guard<std::mutex> lock(task_mutex_);

  task_ = createTask();
  plan_ready_ = false;
  selected_solution_ = nullptr;

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(20))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }

  if (task_.solutions().empty())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Planning reported success, but no solutions were stored");
    return;
  }

  logAllSolutionCosts();

  selected_solution_ = selectBestSolution();
  if (!selected_solution_)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed to select a valid solution");
    return;
  }

  task_.introspection().publishSolution(*selected_solution_);
  plan_ready_ = true;

  RCLCPP_INFO(LOGGER, "Plan ready and published to RViz");
  RCLCPP_INFO(LOGGER, "Stored %zu solutions, selected best cost %.6f",
              task_.solutions().size(), selected_solution_->cost());
  RCLCPP_INFO(LOGGER, "Execute with:");
  RCLCPP_INFO(LOGGER, "ros2 service call /execute_task std_srvs/srv/Trigger");
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "L_xarm7";
  const auto& hand_group_name = "L_xarm_gripper";
  const auto& hand_frame = "L_link_tcp";

  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;
  mtc::Stage* attach_object_stage = nullptr;
#pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();

  const std::string pipeline_name = "ompl";
  const std::string connect_planner_id = "RRTConnectkConfigDefault";
  sampling_planner->setPlannerId(pipeline_name, connect_planner_id);

  sampling_planner->setProperty("max_velocity_scaling_factor", 0.05);
  sampling_planner->setProperty("max_acceleration_scaling_factor", 0.05);

  cartesian_planner->setMaxVelocityScalingFactor(0.05);
  cartesian_planner->setMaxAccelerationScalingFactor(0.05);
  cartesian_planner->setStepSize(0.002);
  cartesian_planner->setJumpThreshold(0.0); // added for collision awareness

  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  // stage_open_hand->setCostTerm(0.5);
  task.add(std::move(stage_open_hand));

  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_pick->setTimeout(20.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  // stage_move_to_pick->setCostTerm(1.0);
  task.add(std::move(stage_move_to_pick));

  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.08, 0.15);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);

      // Penalize Cartesian approach more because this is where execution trouble has happened
      // stage->setCostTerm(4.0);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject("object");
      stage->setAngleDelta(M_PI / 18);
      stage->setMonitoredStage(current_state_ptr);

      Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
      Eigen::Quaterniond q =
          Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = 0.16;

      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(40);
      wrapper->setMinSolutionDistance(0.05);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });

      // Slightly prefer simpler IK choices
      // wrapper->setCostTerm(1.5);
      grasp->insert(std::move(wrapper));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions(
          "object",
          task.getRobotModel()
              ->getJointModelGroup(hand_group_name)
              ->getLinkModelNamesWithCollisionGeometry(),
          true);
      // stage->setCostTerm(0.1);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("close");
      // stage->setCostTerm(0.5);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", hand_frame);
      attach_object_stage = stage.get();
      // stage->setCostTerm(0.1);
      grasp->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.12, 0.15);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);

      // Penalize Cartesian lift too, but less than approach
      // stage->setCostTerm(3.0);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
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

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  executor.spin();
  rclcpp::shutdown();
  return 0;
}