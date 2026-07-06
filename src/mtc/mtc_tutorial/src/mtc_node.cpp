#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
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

// main class for the MTC tutorial node functionality. This class is responsible for setting up the planning scene, creating the MTC task, and executing it
class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

// initialize the MTCTaskNode with specified node options. The consturctor of the MTCTaskNode class
MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}
// getter function to get node base interface. Used by executor later
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

// setup the planning scene by adding a collision object (a cylinder) to the world frame. This function is called before executing the task
void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.1, 0.02 }; // dimensions: height, radius

  // position the cylinder in the world frame
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.25;
  pose.position.y = 0.35;
  pose.position.z = 0.05;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

// interfaces with the MTC object to create and execute a task. The task is composed of a series of stages, including moving the robot to an initial state, opening the hand, and performing a Cartesian path to push an object. The function handles exceptions and logs errors if any stage fails
void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

    if (!task_.plan(5)) {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
    }

    task_.introspection().publishSolution(*task_.solutions().front());

    RCLCPP_INFO(LOGGER, "Plan ready. Press Enter to execute...");
    std::string dummy;
    std::getline(std::cin >> std::ws, dummy);

    auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task"); // set task name to "demo task" 
  task.loadRobotModel(node_); // load the robot model from the parameter server using the node's parameters

  // define the names of useful frames
  const auto& arm_group_name = "L_xarm7";
  const auto& hand_group_name = "L_xarm_gripper";
  const auto& hand_frame = "L_link_tcp";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator. creates a pointer to a stage to be reused or other scenerios
#pragma GCC diagnostic pop

  // generator stage for the current state of the robot
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current)); // save a pointer to it in current_state_ptr for later use

  // solver options
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_); // PipelinePlanner (defaults to OMPL)
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>(); // JointInterpolationPlanner (simple motions, nothing complex)

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>(); // CartesianPath (for Cartesian motions, e.g., pushing an object)
  cartesian_planner->setMaxVelocityScalingFactor(0.05); // use the cartesian planner which requries the following properties to be set
  cartesian_planner->setMaxAccelerationScalingFactor(0.05);
  cartesian_planner->setStepSize(.01);

  // propogator stage for moving the hand to a named pose (e.g., "open" or "closed")
  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner); // simple movement so use the interpolation planner
  stage_open_hand->setGroup(hand_group_name); // set the group to the hand group
  stage_open_hand->setGoal("open"); // set the goal to the "open" named pose defined in the SRDF
  task.add(std::move(stage_open_hand)); // add the stage to the task

  // connector stage for moving the arm to a named pose (e.g., "pick" or "place")
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick", // initialize the stage with a name
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } }); // select the arm group and the sampling planner for this stage
  stage_move_to_pick->setTimeout(5.0); // set a timeout of 5 seconds for this stage
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT); // set properties to be initialized from the parent stage (the task)
  task.add(std::move(stage_move_to_pick)); // add the stage to the task

  // create a pointer to a MTC stage object that will be used later to save a stage
  mtc::Stage* attach_object_stage =
    nullptr;  // Forward attach_object_stage to place pose generator


  { 
    // create a serial container stage for the grasping sequence. this stage will contain the stages for moving to the grasp pose, closing the hand, and attaching the object to the robot
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" }); // use exposeTo to declare the task properties from the parent task in the new serial container
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, // initialize the properties of the grasp stage from the parent task
                                          { "eef", "group", "ik_frame" });

    { 
      // create a propogator stage for approaching the object
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner); // use the cartesian planner with MoveRelative, receiving soltuion from neighbor stages and moves it to other stages
      stage->properties().set("marker_ns", "approach_object"); // set properties
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.15); // set min and max distance for the approach motion

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec; // create a vector to define the direction of the approach motion
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage)); // add to the grasp serial container
    }

    {
      // Sample grasp pose (generator stage) - computes results regardless of stages before and after it
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose"); // conenct to 
      stage->properties().configureInitFrom(mtc::Stage::PARENT); // define properties to be initialized from the parent stage (the grasp serial container)
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject("object");
      stage->setAngleDelta(M_PI / 12); // property of GenerateGraspPose stage, used to determine the number of poses to generate - attempting many different orientations
      stage->setMonitoredStage(current_state_ptr);  // Hook into current state (used to forwward information aboout object pose and shape to the IK solver)

      //define the frame with a PoseStamped message from geometry_msgs, or define the transform using Eigen transformation matrix and the relavent link name
      Eigen::Isometry3d grasp_frame_transform;
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = 0.05; // offset from the object

      // Compute IK
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage)); // name it grasp pose IK and hand it the grasp pose generator stage
      wrapper->setMaxIKSolutions(8); // set limiit on the number of IK solutions to compute for each grasp pose
      wrapper->setMinSolutionDistance(1.0); // set the minimum distance between IK solutions to 1.0 (to avoid similar solutions)
      wrapper->setIKFrame(grasp_frame_transform, hand_frame); // configure addtional properties
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper)); // add the ComputeIK stage to the grasp serial container
    }

    {
      // allow collisions between the hand and the object during the grasping sequence. 
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)"); // ModifyPlanningScene stage to allow collisions 
      stage->allowCollisions("object", // specify the object to allow collisions with (or disable collisions)
                            task.getRobotModel()
                                ->getJointModelGroup(hand_group_name)
                                ->getLinkModelNamesWithCollisionGeometry(), // get all the names of links with collision geometry in the hand group
                            true);
      grasp->insert(std::move(stage));
    }

    {
      // close the hand
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner); // use MoveTo similar to the open hand stage
      stage->setGroup(hand_group_name);
      stage->setGoal("close");
      grasp->insert(std::move(stage));
    }

    {
      // attach object to hand
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object"); // use ModifyPlanningScene stage to attach the object to the hand
      stage->attachObject("object", hand_frame);
      attach_object_stage = stage.get(); // save a pointer to the attach object stage for later use
      grasp->insert(std::move(stage));
    }

    {
      // lift the object after grasping it. This stage uses the Cartesian planner to move the hand (and attached object) upwards
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner); // similar to the approach object stage
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp)); // add the serial container and all of its substages to the task
   }

   {
    // connect stage to bring together the pick and place tasks
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                  { hand_group_name, interpolation_planner } });
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }
  
  {
    // form a serial container to place the stages in
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });

    { //generates the poses to place the object and computes the IK for them
      // Sample place pose
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("object");

      geometry_msgs::msg::PoseStamped target_pose_msg; // determine where to place the object using PoseStamped
      target_pose_msg.header.frame_id = "object";
      target_pose_msg.pose.position.y = 0.5;
      target_pose_msg.pose.orientation.w = 1.0;
      stage->setPose(target_pose_msg); // pass target pose to the stage 
      stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

      // Compute IK
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage)); // compute and hand-off to generator stage
      wrapper->setMaxIKSolutions(2); // same logic as the pick stages...
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame("object");
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    {
      // open the hand using a MoveTo stage and joint interpolation planner
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }

    {
      // re-enable collison with the object using a ModifyPlanningScene stage almost same as enabling collision
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions("object",
                            task.getRobotModel()
                                ->getJointModelGroup(hand_group_name)
                                ->getLinkModelNamesWithCollisionGeometry(),
                            false); // false instead of true forbids collision
      place->insert(std::move(stage));
    }

    {
      // detach the object using same concept as above
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("object", hand_frame);
      place->insert(std::move(stage));
    }

    {
      // retreat from object using a MoveRelative stage, similar to approach and lift object stages
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      // Set retreat direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.x = -0.5;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }
      // finish serial container and add to task
    task.add(std::move(place));
  }

  {
    // return home using a MoveTo stage ang pass it the pose of "prepare_L" for the left arm
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("prepare_L");
    task.add(std::move(stage));
  }

  return task;
}

// the following lines create a node using the define class and calls the methods to setup and execute the task
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

    mtc_task_node->setupPlanningScene();
    mtc_task_node->doTask();

    executor.cancel();
    spin_thread->join();
    rclcpp::shutdown();
    return 0;
}