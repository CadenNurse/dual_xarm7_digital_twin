#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit/task_constructor/storage.h>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <cmath>
#include <mutex>
#include <limits>
#include <memory>
#include <string>
#include <sstream>
#include <typeinfo>
#include <Eigen/Geometry>
#include <vector>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

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
static constexpr const char* TCP_LINK = "L_link_tcp";
static constexpr double ORIENTATION_WEIGHT = 0.15; // lower to reduce distace travelled, increase to reduce total rotations 

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void doTask();
  void setupPlanningScene();

private:
  mtc::Task createTask();
  const mtc::SolutionBase* selectBestSolution();
  void logAllSolutionCosts() const;

  struct TcpPathMetric
  {
    double translation{ 0.0 };
    double rotation{ 0.0 };
  };

  TcpPathMetric computeTcpPathMetric(const mtc::SolutionBase& solution, const std::string& link_name) const;
  TcpPathMetric computeTcpPathMetricRecursive(const mtc::SolutionBase& solution, const std::string& link_name) const;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

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
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, true);
  execute_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "/execute_task",
      std::bind(&MTCTaskNode::executeCallback, this, std::placeholders::_1, std::placeholders::_2));
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

const mtc::SolutionBase* MTCTaskNode::selectBestSolution()
{
  const mtc::SolutionBase* best = nullptr;
  double best_score = std::numeric_limits<double>::infinity();
  double chosen_translation = 0.0;
  double chosen_rotation = 0.0;
  double chosen_mtc_cost = 0.0;

  for (const auto& sol : task_.solutions())
  {
    if (!sol)
      continue;

    const TcpPathMetric metric = computeTcpPathMetric(*sol, TCP_LINK);
    const double mtc_cost = sol->cost();
    const double score = metric.translation + ORIENTATION_WEIGHT * metric.rotation + 0.001 * mtc_cost;

    RCLCPP_INFO(
        LOGGER,
        "Candidate solution: mtc_cost=%.6f tcp_translation=%.6f tcp_rotation=%.6f combined_score=%.6f",
        mtc_cost, metric.translation, metric.rotation, score);

    if (!best || score < best_score)
    {
      best = sol.get();
      best_score = score;
      chosen_translation = metric.translation;
      chosen_rotation = metric.rotation;
      chosen_mtc_cost = mtc_cost;
    }
  }

  if (best)
  {
    RCLCPP_INFO(
        LOGGER,
        "Selected solution: mtc_cost=%.6f tcp_translation=%.6f tcp_rotation=%.6f combined_score=%.6f",
        chosen_mtc_cost, chosen_translation, chosen_rotation, best_score);
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
      const TcpPathMetric metric = computeTcpPathMetric(*sol, TCP_LINK);
      const double combined_score =
          metric.translation + ORIENTATION_WEIGHT * metric.rotation + 0.001 * sol->cost();

      RCLCPP_INFO(
          LOGGER,
          "Solution[%zu] mtc_cost=%.6f tcp_translation=%.6f tcp_rotation=%.6f combined_score=%.6f",
          i, sol->cost(), metric.translation, metric.rotation, combined_score);
    }
    ++i;
  }
}

MTCTaskNode::TcpPathMetric MTCTaskNode::computeTcpPathMetric(
    const mtc::SolutionBase& solution, const std::string& link_name) const
{
  return computeTcpPathMetricRecursive(solution, link_name);
}

MTCTaskNode::TcpPathMetric MTCTaskNode::computeTcpPathMetricRecursive(
    const mtc::SolutionBase& solution, const std::string& link_name) const
{
  TcpPathMetric total;

  if (const auto* sub = dynamic_cast<const mtc::SubTrajectory*>(&solution))
  {
    auto traj = sub->trajectory(); // robot_trajectory::RobotTrajectoryConstPtr traj = sub->trajectory();
    if (traj)
    {
      const std::size_t n = traj->getWayPointCount();
      if (n >= 2)
      {
        for (std::size_t i = 1; i < n; ++i)
        {
          const moveit::core::RobotState& prev = traj->getWayPoint(i - 1);
          const moveit::core::RobotState& curr = traj->getWayPoint(i);

          const Eigen::Isometry3d& T_prev = prev.getGlobalLinkTransform(link_name);
          const Eigen::Isometry3d& T_curr = curr.getGlobalLinkTransform(link_name);

          total.translation += (T_curr.translation() - T_prev.translation()).norm();

          Eigen::Quaterniond q_prev(T_prev.rotation());
          Eigen::Quaterniond q_curr(T_curr.rotation());
          q_prev.normalize();
          q_curr.normalize();
          total.rotation += q_prev.angularDistance(q_curr);
        }
      }
    }
  }

  if (const auto* seq = dynamic_cast<const mtc::SolutionSequence*>(&solution))
  {
    for (const auto& child : seq->solutions())
    {
      if (child)
      {
        const TcpPathMetric child_metric = computeTcpPathMetricRecursive(*child, link_name);
        total.translation += child_metric.translation;
        total.rotation += child_metric.rotation;
      }
    }
  }

  return total;
}

// code to clamp the velocity if it is under 0.001
static void clampFinalWaypoint(robot_trajectory::RobotTrajectory& traj,
                               const std::string& label,
                               double eps = 1e-3)
{
  const std::size_t n = traj.getWayPointCount();
  if (n == 0)
  {
    RCLCPP_WARN(LOGGER, "[clamp] %s: trajectory has 0 waypoints", label.c_str());
    return;
  }

  moveit::core::RobotState& last = *traj.getWayPointPtr(n - 1);
  const moveit::core::JointModelGroup* jmg = traj.getGroup();
  if (!jmg)
  {
    RCLCPP_WARN(LOGGER, "[clamp] %s: trajectory has no JointModelGroup", label.c_str());
    return;
  }

  std::vector<double> v;
  last.copyJointGroupVelocities(jmg, v);

  std::ostringstream before;
  before << "[clamp] " << label << " BEFORE:";
  for (double x : v) before << " " << x;
  RCLCPP_INFO(LOGGER, "%s", before.str().c_str());

  bool changed = false;
  for (double& x : v)
  {
    if (std::abs(x) < eps)
    {
      x = 0.0;
      changed = true;
    }
  }

  if (changed)
    last.setJointGroupVelocities(jmg, v);

  std::vector<double> verify;
  last.copyJointGroupVelocities(jmg, verify);

  std::ostringstream after;
  after << "[clamp] " << label << " AFTER:";
  for (double x : verify) after << " " << x;
  RCLCPP_INFO(LOGGER, "%s", after.str().c_str());
}

static void clampSequenceRecursive(const mtc::SolutionBase& solution, int depth = 0)
{
  std::string indent(depth * 2, ' ');
  RCLCPP_INFO(LOGGER,
              "[clamp] %snode type='%s' cost=%.6f",
              indent.c_str(), typeid(solution).name(), solution.cost());

  if (const auto* sub = dynamic_cast<const mtc::SubTrajectory*>(&solution))
  {
    auto traj = std::const_pointer_cast<robot_trajectory::RobotTrajectory>(sub->trajectory());
    if (traj)
      clampFinalWaypoint(*traj, indent + "SubTrajectory");
    else
      RCLCPP_WARN(LOGGER, "[clamp] %sSubTrajectory has null trajectory", indent.c_str());
  }

  if (const auto* seq = dynamic_cast<const mtc::SolutionSequence*>(&solution))
  {
    std::size_t i = 0;
    for (const auto& child : seq->solutions())
    {
      if (child)
      {
        RCLCPP_INFO(LOGGER, "[clamp] %ssequence child[%zu]", indent.c_str(), i);
        clampSequenceRecursive(*child, depth + 1);
      }
      ++i;
    }
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

  clampSequenceRecursive(*selected_solution_);

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
  const std::string world_frame = "workspace_origin";
  const std::string tag_frame = "tag_21";
  const std::string object_id = "object";

  if (!tf_buffer_) {
    RCLCPP_ERROR(LOGGER, "TF buffer is not initialized");
    return;
  }

  if (!tf_buffer_->canTransform(world_frame, tag_frame, tf2::TimePointZero, tf2::durationFromSec(10.0))) {
    RCLCPP_ERROR(LOGGER, "Cannot transform from '%s' to '%s'", world_frame.c_str(), tag_frame.c_str());
    return;
  }

  geometry_msgs::msg::TransformStamped tf_tag;
  try {
    tf_tag = tf_buffer_->lookupTransform(
        world_frame, tag_frame, tf2::TimePointZero, tf2::durationFromSec(10.0));
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(LOGGER, "Failed to look up AprilTag transform: %s", ex.what());
    return;
  }

  moveit_msgs::msg::CollisionObject object;
  object.id = object_id;
  object.header.frame_id = world_frame;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.12;  // height
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.07;  // length
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.025; // thickness

  geometry_msgs::msg::Pose pose;

  Eigen::Isometry3d T = tf2::transformToEigen(tf_tag.transform);

  Eigen::Isometry3d T_tag_to_box = Eigen::Isometry3d::Identity();
  T_tag_to_box.translation().x() = -0.05;

  Eigen::Isometry3d T_box = T * T_tag_to_box;
  pose = tf2::toMsg(T_box);

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
  rclcpp::sleep_for(std::chrono::milliseconds(1000));

  RCLCPP_INFO(LOGGER,
              "Applied AprilTag box collision object at [%.3f, %.3f, %.3f]",
              pose.position.z, pose.position.y, pose.position.x);
  RCLCPP_INFO(LOGGER, "tag z = %.3f", tf_tag.transform.translation.x);
  RCLCPP_INFO(LOGGER, "box z = %.3f", T.translation().x());
  
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

  if (!task_.plan(35))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }

  if (task_.solutions().empty())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Planning reported success, but no solutions were stored");
    return;
  }

  const auto& sols = task_.solutions();
  selected_solution_ = sols.front().get();

  if (!selected_solution_)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Selected solution is null");
    return;
  }

  RCLCPP_INFO(
      LOGGER,
      "Stored %zu solutions, selected first solution with mtc_cost=%.6f",
      sols.size(),
      selected_solution_->cost());

  task_.introspection().publishSolution(*selected_solution_);
  plan_ready_ = true;

  RCLCPP_INFO(LOGGER, "Plan ready and published to RViz");
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
  const std::string connect_planner_id = "RRTConnect";
  sampling_planner->setPlannerId(pipeline_name, connect_planner_id);

  sampling_planner->setProperty("max_velocity_scaling_factor", 0.25);
  sampling_planner->setProperty("max_acceleration_scaling_factor", 0.25);

  cartesian_planner->setMaxVelocityScalingFactor(0.25);
  cartesian_planner->setMaxAccelerationScalingFactor(0.25);
  cartesian_planner->setStepSize(0.002);

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("open");
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "move to pick",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
    stage->setTimeout(20.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage));
  }

  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    // 1) Top-down pregrasp: TCP aligned with object's +X axis (object X is vertical),
    //    hovering 10 cm above the object. No yaw sampling for now.
    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject("object");
      stage->setMonitoredStage(current_state_ptr);
      stage->setAngleDelta(M_PI);  // effectively two canonical pose. could try multiple as long as it picks thin side

      Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();

      // Hover 10 cm above object along object +X (your object's vertical axis)
      grasp_frame_transform.translation() = Eigen::Vector3d(0.0, 0.0, 0.10);

      // Top-down orientation:
      // Assumes the TCP approach axis should align with object -X so the gripper faces downward.
      // If your tool points the opposite way, flip the sign or use AngleAxisd(M_PI, UnitY()).
      grasp_frame_transform.linear() =
         (Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()) *   // rotate 90 deg about object vertical
          Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitZ())).toRotationMatrix();

      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->setMaxIKSolutions(40);
      wrapper->setMinSolutionDistance(0.05);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });

      grasp->insert(std::move(wrapper));
    }

    // 2) Insert downward into the grasp by 3-5 cm relative to the object vertical axis
    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("insert grasp", cartesian_planner);
      stage->properties().set("marker_ns", "insert_grasp");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.06, 0.08);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "object";
      vec.vector.x = -1.0;   // move downward along object vertical axis
      stage->setDirection(vec);

      grasp->insert(std::move(stage));
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
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("close");
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    // 3) Lift until TCP reaches workspace x = 0.20 m (workspace x is vertical)
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.18, 0.20);   // lift 25 cm
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "workspace_origin";
      vec.vector.x = 1.0;   // x is vertical in your workspace
      stage->setDirection(vec);

      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }

  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{
            {arm_group_name, sampling_planner}});
    // { hand_group_name, interpolation_planner } });
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }

  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    {
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("object");
      stage->setTimeout(3.0);

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "workspace_origin";
      target_pose_msg.pose.position.x = 0.062;
      target_pose_msg.pose.position.y = 0.15;
      target_pose_msg.pose.position.z = 0.15;
      target_pose_msg.pose.orientation.w = 1.0;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage);

      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      // wrapper->setTimeout(5.0);
      wrapper->setMaxIKSolutions(40);
      wrapper->setMinSolutionDistance(0.05);
      wrapper->setIKFrame("object");
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });

      place->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand place", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions(
          "object",
          task.getRobotModel()
              ->getJointModelGroup(hand_group_name)
              ->getLinkModelNamesWithCollisionGeometry(),
          false);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("object", hand_frame);
      place->insert(std::move(stage));
    }

    // 4) Retreat 8 cm upward relative to the placed object's vertical axis
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.08, 0.08);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "object";
      vec.vector.x = 1.0;   // retreat upward along object vertical axis
      stage->setDirection(vec);

      place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("prepare_L");
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
  RCLCPP_INFO(LOGGER, "ros2 service call /execute_task std_srvs/srv/Trigger");

  executor.spin();

  rclcpp::shutdown();
  return 0;
}