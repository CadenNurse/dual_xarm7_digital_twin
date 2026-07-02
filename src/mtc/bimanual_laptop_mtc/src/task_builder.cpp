#include "bimanual_laptop_mtc/task_builder.hpp"
#include "bimanual_laptop_mtc/scene_utils.hpp"
#include "bimanual_laptop_mtc/laptop_geometry.hpp"

#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <geometry_msgs/msg/vector3_stamped.hpp>
// #include <tf2_eigen/tf2_eigen.hpp>

namespace mtc = moveit::task_constructor;
namespace stages = mtc::stages;
namespace solvers = mtc::solvers;

namespace bimanual_laptop_mtc
{

moveit::task_constructor::Task buildCloseLaptopSingleArmTask(
  const rclcpp::Node::SharedPtr& node,
  const geometry_msgs::msg::PoseStamped& laptop_base_pose,
  const LaptopGeometry& geom,
  const TaskBuilderConfig& cfg)
{
  mtc::Task task;
  task.stages()->setName("close_laptop_single_arm");
  task.loadRobotModel(node);

  auto pipeline = std::make_shared<solvers::PipelinePlanner>(node);
  auto cartesian = std::make_shared<solvers::CartesianPath>();
  cartesian->setMaxVelocityScalingFactor(0.1);
  cartesian->setMaxAccelerationScalingFactor(0.1);
  cartesian->setStepSize(0.005);

  task.add(std::make_unique<stages::CurrentState>("current"));

  auto scene_stage = std::make_unique<stages::ModifyPlanningScene>("add scene");
  const auto objects = makeLaptopScene(
    cfg.world_frame, laptop_base_pose, geom,
    cfg.table_size_x, cfg.table_size_y, cfg.table_size_z, cfg.table_pose_z);
  for (const auto& obj : objects) {
    scene_stage->addObject(obj);
  }
  task.add(std::move(scene_stage));

  auto move_to_pregrasp = std::make_unique<stages::MoveTo>("move to pregrasp", pipeline);
  move_to_pregrasp->setGroup(cfg.arm_group);

  // const auto pregrasp_pose_msg =
  // makeLidEdgePregraspPose(geom, laptop_base_pose, cfg.lid_angle_start_rad, cfg.pregrasp_offset_z);

  // Eigen::Isometry3d pregrasp_pose_eigen = Eigen::Isometry3d::Identity();
  // tf2::fromMsg(pregrasp_pose_msg.pose, pregrasp_pose_eigen);

  // move_to_pregrasp->setIKFrame(pregrasp_pose_eigen, cfg.eef_frame);
  // task.add(std::move(move_to_pregrasp));

  const auto pregrasp_pose_msg =
  makeLidEdgePregraspPose(geom, laptop_base_pose, cfg.lid_angle_start_rad, cfg.pregrasp_offset_z);

  move_to_pregrasp->setGroup(cfg.arm_group);
  move_to_pregrasp->setIKFrame(cfg.eef_frame);
  move_to_pregrasp->setGoal(pregrasp_pose_msg);
  task.add(std::move(move_to_pregrasp));

  for (int i = 0; i < cfg.step_count; ++i)
  {
    auto step = std::make_unique<stages::MoveRelative>("close step " + std::to_string(i), cartesian);
    step->properties().set("group", cfg.arm_group);
    step->properties().set("ik_frame", cfg.eef_frame);

    geometry_msgs::msg::Vector3Stamped direction;
    direction.header.frame_id = cfg.world_frame;
    direction.vector.x = -cfg.step_translation_m;
    direction.vector.y = 0.0;
    direction.vector.z = -0.25 * cfg.step_translation_m;

    step->setDirection(direction);
    task.add(std::move(step));
  }

  auto retreat = std::make_unique<stages::MoveRelative>("retreat", cartesian);
  retreat->properties().set("group", cfg.arm_group);
  retreat->properties().set("ik_frame", cfg.eef_frame);

  geometry_msgs::msg::Vector3Stamped retreat_dir;
  retreat_dir.header.frame_id = cfg.world_frame;
  retreat_dir.vector.x = 0.0;
  retreat_dir.vector.y = 0.0;
  retreat_dir.vector.z = cfg.retreat_distance_m;
  retreat->setDirection(retreat_dir);
  task.add(std::move(retreat));

  return task;
}

}  // namespace bimanual_laptop_mtc