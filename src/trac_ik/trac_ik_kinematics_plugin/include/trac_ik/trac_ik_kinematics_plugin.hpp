/********************************************************************************
Copyright (c) 2015, TRACLabs, Inc.
All rights reserved.
********************************************************************************/

#ifndef TRAC_IK_KINEMATICS_PLUGIN_
#define TRAC_IK_KINEMATICS_PLUGIN_

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <moveit/kinematics_base/kinematics_base.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/rclcpp.hpp>

namespace trac_ik_kinematics_plugin
{

class TRAC_IKKinematicsPlugin : public kinematics::KinematicsBase
{
public:
  TRAC_IKKinematicsPlugin()
    : num_joints_(0), active_(false), position_ik_(false)
  {
  }

  ~TRAC_IKKinematicsPlugin() override = default;

  const std::vector<std::string>& getJointNames() const override
  {
    return joint_names_;
  }

  const std::vector<std::string>& getLinkNames() const override
  {
    return link_names_;
  }

  bool initialize(const rclcpp::Node::SharedPtr& node,
                  const moveit::core::RobotModel& robot_model,
                  const std::string& group_name,
                  const std::string& base_frame,
                  const std::vector<std::string>& tip_frames,
                  double search_discretization) override;

  bool getPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                     const std::vector<double>& ik_seed_state,
                     std::vector<double>& solution,
                     moveit_msgs::msg::MoveItErrorCodes& error_code,
                     const kinematics::KinematicsQueryOptions& options =
                         kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                        const std::vector<double>& ik_seed_state,
                        double timeout,
                        std::vector<double>& solution,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options =
                            kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                        const std::vector<double>& ik_seed_state,
                        double timeout,
                        const std::vector<double>& consistency_limits,
                        std::vector<double>& solution,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options =
                            kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                        const std::vector<double>& ik_seed_state,
                        double timeout,
                        std::vector<double>& solution,
                        const IKCallbackFn& solution_callback,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options =
                            kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                        const std::vector<double>& ik_seed_state,
                        double timeout,
                        const std::vector<double>& consistency_limits,
                        std::vector<double>& solution,
                        const IKCallbackFn& solution_callback,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options =
                            kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                        const std::vector<double>& ik_seed_state,
                        double timeout,
                        std::vector<double>& solution,
                        const IKCallbackFn& solution_callback,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const std::vector<double>& consistency_limits,
                        const kinematics::KinematicsQueryOptions& options) const;

  bool getPositionFK(const std::vector<std::string>& link_names,
                     const std::vector<double>& joint_angles,
                     std::vector<geometry_msgs::msg::Pose>& poses) const override;

private:
  int getKDLSegmentIndex(const std::string& name) const;

  std::vector<std::string> joint_names_;
  std::vector<std::string> link_names_;

  unsigned int num_joints_;
  bool active_;
  KDL::Chain chain;
  bool position_ik_;
  KDL::JntArray joint_min, joint_max;
  std::string solve_type;
};

}  // namespace trac_ik_kinematics_plugin

#endif