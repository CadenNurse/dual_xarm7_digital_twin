/********************************************************************************
Copyright (c) 2015, TRACLabs, Inc.
All rights reserved.
********************************************************************************/

#include <algorithm>
#include <cassert>
#include <limits>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_kdl/tf2_kdl.hpp>

#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <trac_ik/trac_ik.hpp>
#include <trac_ik/trac_ik_kinematics_plugin.hpp>

namespace trac_ik_kinematics_plugin
{

static const rclcpp::Logger LOGGER =
    rclcpp::get_logger("trac_ik_kinematics_plugin");

bool TRAC_IKKinematicsPlugin::initialize(const rclcpp::Node::SharedPtr& node,
                                         const moveit::core::RobotModel& robot_model,
                                         const std::string& group_name,
                                         const std::string& base_name,
                                         const std::vector<std::string>& tip_frames,
                                         double search_discretization)
{
  storeValues(robot_model, group_name, base_name, tip_frames, search_discretization);
  node_ = node;

  if (tip_frames.empty())
  {
    RCLCPP_ERROR(LOGGER, "No tip frame specified");
    return false;
  }

  const std::string& tip_name = tip_frames[0];

  const auto urdf_model = robot_model.getURDF();
  if (!urdf_model)
  {
    RCLCPP_FATAL(LOGGER, "Robot model does not contain a URDF");
    return false;
  }

  RCLCPP_DEBUG(LOGGER, "Reading joints and links from URDF");

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(*urdf_model, tree))
  {
    RCLCPP_FATAL(LOGGER, "Failed to extract kdl tree from robot model");
    return false;
  }

  if (!tree.getChain(base_name, tip_name, chain))
  {
    RCLCPP_FATAL(LOGGER, "Couldn't find chain %s to %s", base_name.c_str(), tip_name.c_str());
    return false;
  }

  num_joints_ = chain.getNrOfJoints();
  const std::vector<KDL::Segment> chain_segs = chain.segments;

  joint_min.resize(num_joints_);
  joint_max.resize(num_joints_);
  link_names_.clear();
  joint_names_.clear();

  unsigned int joint_num = 0;
  for (unsigned int i = 0; i < chain_segs.size(); ++i)
  {
    link_names_.push_back(chain_segs[i].getName());

    urdf::JointConstSharedPtr joint = urdf_model->getJoint(chain_segs[i].getJoint().getName());
    if (!joint)
      continue;

    if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
    {
      ++joint_num;
      assert(joint_num <= num_joints_);

      float lower = 0.0;
      float upper = 0.0;
      bool has_limits = false;
      joint_names_.push_back(joint->name);

      if (joint->type != urdf::Joint::CONTINUOUS)
      {
        if (joint->safety)
        {
          lower = std::max(joint->limits->lower, joint->safety->soft_lower_limit);
          upper = std::min(joint->limits->upper, joint->safety->soft_upper_limit);
        }
        else
        {
          lower = joint->limits->lower;
          upper = joint->limits->upper;
        }
        has_limits = true;
      }

      if (has_limits)
      {
        joint_min(joint_num - 1) = lower;
        joint_max(joint_num - 1) = upper;
      }
      else
      {
        joint_min(joint_num - 1) = std::numeric_limits<float>::lowest();
        joint_max(joint_num - 1) = std::numeric_limits<float>::max();
      }

      RCLCPP_INFO_STREAM(LOGGER,
                         "IK Using joint " << chain_segs[i].getName() << " "
                                           << joint_min(joint_num - 1) << " "
                                           << joint_max(joint_num - 1));
    }
  }

  RCLCPP_INFO_STREAM(LOGGER, "Looking for param: " << group_name << ".position_only_ik");
  node_->declare_parameter(group_name + ".position_only_ik", false);
  node_->get_parameter(group_name + ".position_only_ik", position_ik_);

  RCLCPP_INFO_STREAM(LOGGER, "Looking for param: " << group_name << ".solve_type");
  node_->declare_parameter(group_name + ".solve_type", std::string("Speed"));
  node_->get_parameter(group_name + ".solve_type", solve_type);

  RCLCPP_INFO_STREAM(LOGGER, "Using solve type " << solve_type);

  active_ = true;
  return true;
}

int TRAC_IKKinematicsPlugin::getKDLSegmentIndex(const std::string& name) const
{
  int i = 0;
  while (i < static_cast<int>(chain.getNrOfSegments()))
  {
    if (chain.getSegment(i).getName() == name)
      return i + 1;
    ++i;
  }
  return -1;
}

bool TRAC_IKKinematicsPlugin::getPositionFK(const std::vector<std::string>& link_names,
                                            const std::vector<double>& joint_angles,
                                            std::vector<geometry_msgs::msg::Pose>& poses) const
{
  if (!active_)
  {
    RCLCPP_ERROR(LOGGER, "kinematics not active");
    return false;
  }

  poses.resize(link_names.size());
  if (joint_angles.size() != num_joints_)
  {
    RCLCPP_ERROR(LOGGER, "Joint angles vector must have size: %u", num_joints_);
    return false;
  }

  KDL::Frame p_out;
  KDL::JntArray jnt_pos_in(num_joints_);
  for (unsigned int i = 0; i < num_joints_; ++i)
    jnt_pos_in(i) = joint_angles[i];

  KDL::ChainFkSolverPos_recursive fk_solver(chain);

  bool valid = true;
  for (unsigned int i = 0; i < poses.size(); ++i)
  {
    const int segment_index = getKDLSegmentIndex(link_names[i]);
    RCLCPP_DEBUG(LOGGER, "End effector index: %d", segment_index);

    if (fk_solver.JntToCart(jnt_pos_in, p_out, segment_index) >= 0)
    {
      poses[i] = tf2::toMsg(p_out);
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "Could not compute FK for %s", link_names[i].c_str());
      valid = false;
    }
  }

  return valid;
}

bool TRAC_IKKinematicsPlugin::getPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                            const std::vector<double>& ik_seed_state,
                                            std::vector<double>& solution,
                                            moveit_msgs::msg::MoveItErrorCodes& error_code,
                                            const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          default_timeout_,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool TRAC_IKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                               const std::vector<double>& ik_seed_state,
                                               double timeout,
                                               std::vector<double>& solution,
                                               moveit_msgs::msg::MoveItErrorCodes& error_code,
                                               const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool TRAC_IKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                               const std::vector<double>& ik_seed_state,
                                               double timeout,
                                               const std::vector<double>& consistency_limits,
                                               std::vector<double>& solution,
                                               moveit_msgs::msg::MoveItErrorCodes& error_code,
                                               const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = 0;
  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool TRAC_IKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                               const std::vector<double>& ik_seed_state,
                                               double timeout,
                                               std::vector<double>& solution,
                                               const IKCallbackFn& solution_callback,
                                               moveit_msgs::msg::MoveItErrorCodes& error_code,
                                               const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool TRAC_IKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                               const std::vector<double>& ik_seed_state,
                                               double timeout,
                                               const std::vector<double>& consistency_limits,
                                               std::vector<double>& solution,
                                               const IKCallbackFn& solution_callback,
                                               moveit_msgs::msg::MoveItErrorCodes& error_code,
                                               const kinematics::KinematicsQueryOptions& options) const
{
  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool TRAC_IKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                               const std::vector<double>& ik_seed_state,
                                               double timeout,
                                               std::vector<double>& solution,
                                               const IKCallbackFn& solution_callback,
                                               moveit_msgs::msg::MoveItErrorCodes& error_code,
                                               const std::vector<double>& /*consistency_limits*/,
                                               const kinematics::KinematicsQueryOptions& /*options*/) const
{
  RCLCPP_DEBUG(LOGGER, "getPositionIK");

  if (!active_)
  {
    RCLCPP_ERROR(LOGGER, "kinematics not active");
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;
  }

  if (ik_seed_state.size() != num_joints_)
  {
    RCLCPP_ERROR_STREAM(LOGGER,
                        "Seed state must have size " << num_joints_
                                                     << " instead of size " << ik_seed_state.size());
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;
  }

  KDL::Frame frame;
  tf2::fromMsg(ik_pose, frame);

  KDL::JntArray in(num_joints_), out(num_joints_);
  for (unsigned int z = 0; z < num_joints_; ++z)
    in(z) = ik_seed_state[z];

  KDL::Twist bounds = KDL::Twist::Zero();
  if (position_ik_)
  {
    bounds.rot.x(std::numeric_limits<float>::max());
    bounds.rot.y(std::numeric_limits<float>::max());
    bounds.rot.z(std::numeric_limits<float>::max());
  }

  const double epsilon = 1e-5;
  TRAC_IK::SolveType solvetype;

  if (solve_type == "Manipulation1")
    solvetype = TRAC_IK::Manip1;
  else if (solve_type == "Manipulation2")
    solvetype = TRAC_IK::Manip2;
  else if (solve_type == "Distance")
    solvetype = TRAC_IK::Distance;
  else
  {
    if (solve_type != "Speed")
      RCLCPP_WARN_STREAM(LOGGER, solve_type << " is not a valid solve_type; setting to default: Speed");
    solvetype = TRAC_IK::Speed;
  }

  TRAC_IK::TRAC_IK ik_solver(chain, joint_min, joint_max, timeout, epsilon, solvetype);
  const int rc = ik_solver.CartToJnt(in, frame, out, bounds);

  solution.resize(num_joints_);

  if (rc >= 0)
  {
    for (unsigned int z = 0; z < num_joints_; ++z)
      solution[z] = out(z);

    if (solution_callback)
    {
      solution_callback(ik_pose, solution, error_code);
      if (error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
      {
        RCLCPP_DEBUG(LOGGER, "Solution passes callback");
        return true;
      }
      else
      {
        RCLCPP_DEBUG_STREAM(LOGGER, "Solution has error code " << error_code.val);
        return false;
      }
    }
    return true;
  }

  error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
  return false;
}

}  // namespace trac_ik_kinematics_plugin

PLUGINLIB_EXPORT_CLASS(trac_ik_kinematics_plugin::TRAC_IKKinematicsPlugin,
                       kinematics::KinematicsBase)