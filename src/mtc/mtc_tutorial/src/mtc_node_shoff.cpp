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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("serial_bimanual_handover");
namespace mtc = moveit::task_constructor;

namespace
{
constexpr char kNodeName[] = "serial_bimanual_handover_node";
constexpr char kExecuteServiceName[] = "/execute_task";

constexpr char kWorldFrame[] = "workspace_origin";
constexpr char kTagFrame[] = "tag_21";
constexpr char kObjectId[] = "object";

constexpr char kLeftArmGroup[] = "L_xarm7";
constexpr char kLeftHandGroup[] = "L_xarm_gripper";
constexpr char kLeftTcpLink[] = "L_link_tcp";

constexpr char kRightArmGroup[] = "R_xarm7";
constexpr char kRightHandGroup[] = "R_xarm_gripper";
constexpr char kRightTcpLink[] = "R_link_tcp";

constexpr char kLeftHomePose[] = "prepare_L";
constexpr char kRightHomePose[] = "prepare_R";

constexpr std::size_t kMaxPlanSolutions = 60;

geometry_msgs::msg::Quaternion quatFromRPY(double roll, double pitch, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return tf2::toMsg(q);
}

}  // namespace

class MTCTaskNode
{
public:
  explicit MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void doTask();
  void setupPlanningScene();

private:
  mtc::Task createTask();
  void resetPlanState();
  bool lookupObjectTransform(geometry_msgs::msg::TransformStamped& tf_tag) const;

  void executeCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

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

  const Eigen::Isometry3d T_world_tag = tf2::transformToEigen(tf_tag.transform);

  Eigen::Isometry3d T_tag_box = Eigen::Isometry3d::Identity();
  T_tag_box.translation().x() = -0.05;

  const Eigen::Isometry3d T_world_box = T_world_tag * T_tag_box;
  const geometry_msgs::msg::Pose pose = tf2::toMsg(T_world_box);

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
  rclcpp::sleep_for(std::chrono::milliseconds(1000));

  RCLCPP_INFO(
      LOGGER,
      "Applied box collision object at [x=%.3f, y=%.3f, z=%.3f]",
      pose.position.x,
      pose.position.y,
      pose.position.z);
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
  task.stages()->setName("dual arm handover task");
  task.loadRobotModel(node_);

  const std::string left_arm_group_name = kLeftArmGroup;
  const std::string left_hand_group_name = kLeftHandGroup;
  const std::string left_hand_frame = kLeftTcpLink;

  const std::string right_arm_group_name = kRightArmGroup;
  const std::string right_hand_group_name = kRightHandGroup;
  const std::string right_hand_frame = kRightTcpLink;

  mtc::Stage* current_state_ptr = nullptr;
  mtc::Stage* left_attach_stage = nullptr;
  mtc::Stage* left_handover_pose_stage = nullptr;
  mtc::Stage* right_attach_stage = nullptr;
  mtc::Stage* left_prepare_stage = nullptr; // uncomment with the rest

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();

  sampling_planner->setPlannerId("ompl", "RRTConnect");
  sampling_planner->setProperty("max_velocity_scaling_factor", 0.20);
  sampling_planner->setProperty("max_acceleration_scaling_factor", 0.20);

  cartesian_planner->setMaxVelocityScalingFactor(0.20);
  cartesian_planner->setMaxAccelerationScalingFactor(0.20);
  cartesian_planner->setStepSize(0.005);
  cartesian_planner->setMinFraction(0.90);

  moveit::core::CartesianPrecision cartesian_precision;
  cartesian_precision.translational = 0.008;
  cartesian_precision.rotational = 0.08;
  cartesian_planner->setPrecision(cartesian_precision);

  auto configureLeftContainer = [&](mtc::ContainerBase& container) {
    container.properties().set("group", left_arm_group_name);
    container.properties().set("eef", left_hand_group_name);
    container.properties().set("ik_frame", left_hand_frame);
  };

  auto configureRightContainer = [&](mtc::ContainerBase& container) {
    container.properties().set("group", right_arm_group_name);
    container.properties().set("eef", right_hand_group_name);
    container.properties().set("ik_frame", right_hand_frame);
  };

  {
    auto stage = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage.get();
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open left hand", interpolation_planner);
    stage->setGroup(left_hand_group_name);
    stage->setGoal("open");
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open right hand", interpolation_planner);
    stage->setGroup(right_hand_group_name);
    stage->setGoal("open");
    task.add(std::move(stage));
  }

  { // MIGHT BE REDUNDENT
    auto stage = std::make_unique<mtc::stages::MoveTo>("move left to prepare_L", interpolation_planner);
    stage->setGroup(left_arm_group_name);
    stage->setGoal(kLeftHomePose);
    left_prepare_stage = stage.get();
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "move left to pick",
        mtc::stages::Connect::GroupPlannerVector{{left_arm_group_name, sampling_planner}});
    stage->setTimeout(30.0);
    task.add(std::move(stage));
  }

  {
    auto left_pick = std::make_unique<mtc::SerialContainer>("left pick and lift");
    configureLeftContainer(*left_pick);

    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate left top grasp");
      stage->setEndEffector(left_hand_group_name);
      stage->properties().set("eef", left_hand_group_name);
      stage->properties().set("group", left_arm_group_name);
      stage->properties().set("ik_frame", left_hand_frame);
      stage->properties().set("marker_ns", "left_top_grasp");
      stage->setPreGraspPose("open");
      stage->setObject(kObjectId);
      stage->setMonitoredStage(left_prepare_stage ? left_prepare_stage : current_state_ptr);
      stage->setAngleDelta(M_PI);

      Eigen::Isometry3d grasp_tf = Eigen::Isometry3d::Identity();
      grasp_tf.linear() =
          (Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitZ()))
           .toRotationMatrix();
      grasp_tf.translation() = Eigen::Vector3d(0.0, 0.0, 0.08);

      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("left top grasp IK", std::move(stage));
      wrapper->setGroup(left_arm_group_name);
      wrapper->setEndEffector(left_hand_group_name);
      wrapper->setIKFrame(grasp_tf, left_hand_frame);
      wrapper->setMaxIKSolutions(40);
      wrapper->setMinSolutionDistance(0.05);
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});

      left_pick->insert(std::move(wrapper));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("left descend into grasp", cartesian_planner);
      stage->setGroup(left_arm_group_name);
      stage->setIKFrame(left_hand_frame);
      stage->setMinMaxDistance(0.04, 0.06);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = kObjectId;
      vec.vector.x = -1.0;
      stage->setDirection(vec);

      left_pick->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision left hand/object");
      stage->allowCollisions(
          kObjectId,
          task.getRobotModel()
              ->getJointModelGroup(left_hand_group_name)
              ->getLinkModelNamesWithCollisionGeometry(),
          true);
      left_pick->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close left hand", interpolation_planner);
      stage->setGroup(left_hand_group_name);
      stage->setGoal("close");
      left_pick->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("attach object to left tcp");
      stage->attachObject(kObjectId, left_hand_frame);
      left_attach_stage = stage.get();
      left_pick->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("lift object with left", cartesian_planner);
      stage->setGroup(left_arm_group_name);
      stage->setIKFrame(left_hand_frame);
      stage->setMinMaxDistance(0.12, 0.18);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = kWorldFrame;
      vec.vector.x = 1.0;
      stage->setDirection(vec);

      left_pick->insert(std::move(stage));
    }

    task.add(std::move(left_pick));
  }

  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "move left to handover pose",
        mtc::stages::Connect::GroupPlannerVector{{left_arm_group_name, sampling_planner}});
    stage->setTimeout(30.0);
    task.add(std::move(stage));
  }

  {
    auto left_move_handover =
        std::make_unique<mtc::SerialContainer>("left move object to handover pose");
    configureLeftContainer(*left_move_handover);

    auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate left handover pose");
    stage->properties().set("eef", left_hand_group_name);
    stage->properties().set("group", left_arm_group_name);
    stage->properties().set("ik_frame", left_hand_frame);
    stage->properties().set("marker_ns", "left_handover_pose");
    stage->setObject(kObjectId);
    stage->setMonitoredStage(left_attach_stage);
    stage->setTimeout(5.0);

    geometry_msgs::msg::PoseStamped handover_pose;
    handover_pose.header.frame_id = kWorldFrame;
    handover_pose.pose.position.x = 0.40;
    handover_pose.pose.position.y = 0.40;
    handover_pose.pose.position.z = 0.35;
    handover_pose.pose.orientation = quatFromRPY(M_PI_2, 0.0, 0.0); // 90 deg about x (z as they are flipped)
    stage->setPose(handover_pose);

    auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("left handover pose IK", std::move(stage));
    wrapper->setGroup(left_arm_group_name);
    wrapper->setEndEffector(left_hand_group_name);
    wrapper->setIKFrame(left_hand_frame);
    wrapper->setMaxIKSolutions(40);
    wrapper->setMinSolutionDistance(0.05);
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});

    left_handover_pose_stage = wrapper.get();
    left_move_handover->insert(std::move(wrapper));

    task.add(std::move(left_move_handover));
  }

  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "bring right arm to handover",
        mtc::stages::Connect::GroupPlannerVector{
            {left_arm_group_name, sampling_planner},
            {right_arm_group_name, sampling_planner}});
    stage->setTimeout(30.0);
    task.add(std::move(stage));
  }

  {
    auto right_handover = std::make_unique<mtc::SerialContainer>("right grasp from side");
    configureRightContainer(*right_handover);

    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate right side grasp");
      stage->setEndEffector(right_hand_group_name);
      stage->properties().set("eef", right_hand_group_name);
      stage->properties().set("group", right_arm_group_name);
      stage->properties().set("ik_frame", right_hand_frame);
      stage->properties().set("marker_ns", "right_side_grasp");
      stage->setPreGraspPose("open");
      stage->setObject(kObjectId);
      stage->setMonitoredStage(left_handover_pose_stage);
      stage->setAngleDelta(M_PI);

      Eigen::Isometry3d grasp_tf = Eigen::Isometry3d::Identity(); // use M_PI_2 UnitX, M_PI_2 UnitZ for upside-down side grasp
      grasp_tf.linear() =
          (Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()))
           .toRotationMatrix();
      grasp_tf.translation() = Eigen::Vector3d(0.0, 0.0, 0.12);

      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("right side grasp IK", std::move(stage));
      wrapper->setGroup(right_arm_group_name);
      wrapper->setEndEffector(right_hand_group_name);
      wrapper->setIKFrame(grasp_tf, right_hand_frame);
      wrapper->setMaxIKSolutions(40);
      wrapper->setMinSolutionDistance(0.05);
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});

      right_handover->insert(std::move(wrapper));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("right insert side grasp", cartesian_planner);
      stage->setGroup(right_arm_group_name);
      stage->setIKFrame(right_hand_frame);
      stage->setMinMaxDistance(0.06, 0.10);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = kObjectId;
      vec.vector.x = 1.0;
      stage->setDirection(vec);

      right_handover->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision right hand/object");
      stage->allowCollisions(
          kObjectId,
          task.getRobotModel()
              ->getJointModelGroup(right_hand_group_name)
              ->getLinkModelNamesWithCollisionGeometry(),
          true);
      right_handover->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision left hand/right hand");
      stage->allowCollisions(
          task.getRobotModel()
              ->getJointModelGroup(left_hand_group_name)
              ->getLinkModelNamesWithCollisionGeometry(),
          task.getRobotModel()
              ->getJointModelGroup(right_hand_group_name)
              ->getLinkModelNamesWithCollisionGeometry(),
          true);
      right_handover->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close right hand", interpolation_planner);
      stage->setGroup(right_hand_group_name);
      stage->setGoal("close");
      right_handover->insert(std::move(stage));
    }

    task.add(std::move(right_handover));
  }

  {
    auto transfer = std::make_unique<mtc::SerialContainer>("handover transfer left to right");

    {
      auto stage =
          std::make_unique<mtc::stages::MoveTo>("open left hand release", interpolation_planner);
      stage->setGroup(left_hand_group_name);
      stage->setGoal("open");
      transfer->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("detach object from left tcp");
      stage->detachObject(kObjectId, left_hand_frame);
      transfer->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object to right tcp");
      stage->attachObject(kObjectId, right_hand_frame);
      right_attach_stage = stage.get();
      transfer->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("left retreat after handover", cartesian_planner);
      stage->setGroup(left_arm_group_name);
      stage->setIKFrame(left_hand_frame);
      stage->setMinMaxDistance(0.08, 0.12);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = kWorldFrame;
      vec.vector.y = -1.0;
      stage->setDirection(vec);

      transfer->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("right retreat after handover", cartesian_planner);
      stage->setGroup(right_arm_group_name);
      stage->setIKFrame(right_hand_frame);
      stage->setMinMaxDistance(0.08, 0.12);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = kWorldFrame;
      vec.vector.y = 1.0;
      stage->setDirection(vec);

      transfer->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision left hand/object");
      stage->allowCollisions(
          kObjectId,
          task.getRobotModel()
              ->getJointModelGroup(left_hand_group_name)
              ->getLinkModelNamesWithCollisionGeometry(),
          false);
      transfer->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision left hand/right hand");
      stage->allowCollisions(
          task.getRobotModel()
              ->getJointModelGroup(left_hand_group_name)
              ->getLinkModelNamesWithCollisionGeometry(),
          task.getRobotModel()
              ->getJointModelGroup(right_hand_group_name)
              ->getLinkModelNamesWithCollisionGeometry(),
          false);
      transfer->insert(std::move(stage));
    }

    task.add(std::move(transfer));
  }

  {
    auto stage =
        std::make_unique<mtc::stages::MoveTo>("return left to prepare_L", interpolation_planner);
    stage->setGroup(left_arm_group_name);
    stage->setGoal(kLeftHomePose);
    task.add(std::move(stage));
  }

  //  PROBLMEATIC CONNECT BIG TIME CAUSING ME PAIN
  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "move right to place",
        mtc::stages::Connect::GroupPlannerVector{{left_arm_group_name, sampling_planner},
                                                 {right_arm_group_name, sampling_planner}}); // could remove if causing issues. sending L_arm back to handover at the moment
    stage->setTimeout(30.0);
    task.add(std::move(stage));
  }

  {
    auto right_place = std::make_unique<mtc::SerialContainer>("right place object");
    configureRightContainer(*right_place);

    {
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate right place pose");
      stage->properties().set("eef", right_hand_group_name);
      stage->properties().set("group", right_arm_group_name);
      stage->properties().set("ik_frame", right_hand_frame);
      stage->properties().set("marker_ns", "right_place_pose");
      stage->setObject(kObjectId);
      stage->setMonitoredStage(right_attach_stage);
      stage->setTimeout(5.0);


      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = kWorldFrame;
      target_pose_msg.pose.position.x = 0.05;
      target_pose_msg.pose.position.y = 0.60;
      target_pose_msg.pose.position.z = 0.15;
      target_pose_msg.pose.orientation = quatFromRPY(-M_PI_2, 0.0, 0.0);
      stage->setPose(target_pose_msg);

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("right place IK", std::move(stage));
      wrapper->setGroup(right_arm_group_name);
      wrapper->setEndEffector(right_hand_group_name);
      wrapper->setIKFrame(right_hand_frame);
      wrapper->setMaxIKSolutions(40);
      wrapper->setMinSolutionDistance(0.05);
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});

      right_place->insert(std::move(wrapper));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::MoveTo>("open right hand place", interpolation_planner);
      stage->setGroup(right_hand_group_name);
      stage->setGoal("open");
      right_place->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("detach object from right tcp");
      stage->detachObject(kObjectId, right_hand_frame);
      right_place->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision right hand/object");
      stage->allowCollisions(
          kObjectId,
          task.getRobotModel()
              ->getJointModelGroup(right_hand_group_name)
              ->getLinkModelNamesWithCollisionGeometry(),
          false);
      right_place->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("right retreat after place", cartesian_planner);
      stage->setGroup(right_arm_group_name);
      stage->setIKFrame(right_hand_frame);
      stage->setMinMaxDistance(0.10, 0.15);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = kWorldFrame;
      vec.vector.x = 1.0;
      stage->setDirection(vec);

      right_place->insert(std::move(stage));
    }

    task.add(std::move(right_place));
  }

  {
    auto stage =
        std::make_unique<mtc::stages::MoveTo>("return left to prepare_L", interpolation_planner);
    stage->setGroup(left_arm_group_name);
    stage->setGoal(kLeftHomePose);
    task.add(std::move(stage));
  }

  {
    auto stage =
        std::make_unique<mtc::stages::MoveTo>("return right to prepare_R", interpolation_planner);
    stage->setGroup(right_arm_group_name);
    stage->setGoal(kRightHomePose);
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