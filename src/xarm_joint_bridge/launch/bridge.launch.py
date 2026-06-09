"""
launch/bridge.launch.py
=======================
Launches the xarm_joint_bridge node with all parameters configurable from
the command line.

Example (default — xarm_ros2 namespaced driver):
  ros2 launch xarm_joint_bridge bridge.launch.py

Example (single /joint_states topic per arm, no joint renaming):
  ros2 launch xarm_joint_bridge bridge.launch.py \
      left_in_topic:=/xarm1/joint_states \
      right_in_topic:=/xarm2/joint_states \
      rename_joints:=False

Example (arms share one topic, already differentiated by joint name):
  ros2 launch xarm_joint_bridge bridge.launch.py \
      left_in_topic:=/xarm_left/joint_states \
      right_in_topic:=/xarm_right/joint_states \
      left_prefix:=L_ \
      right_prefix:=R_
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ------------------------------------------------------------------ args
    args = [
        DeclareLaunchArgument("left_in_topic",   default_value="/xarm_left/joint_states",
                              description="Topic the left real robot publishes to"),
        DeclareLaunchArgument("right_in_topic",  default_value="/xarm_right/joint_states",
                              description="Topic the right real robot publishes to"),
        DeclareLaunchArgument("left_out_topic",  default_value="/L/joint_states",
                              description="Topic Isaac Sim left prim subscribes to"),
        DeclareLaunchArgument("right_out_topic", default_value="/R/joint_states",
                              description="Topic Isaac Sim right prim subscribes to"),
        DeclareLaunchArgument("left_prefix",     default_value="L_",
                              description="Prefix added to joint names for the left arm"),
        DeclareLaunchArgument("right_prefix",    default_value="R_",
                              description="Prefix added to joint names for the right arm"),
        DeclareLaunchArgument("rename_joints",   default_value="True",
                              description="Rename joints with arm prefix (True/False)"),
        DeclareLaunchArgument("qos_depth",       default_value="10",
                              description="ROS2 QoS history depth"),
    ]

    # ------------------------------------------------------------------ node
    bridge_node = Node(
        package="xarm_joint_bridge",
        executable="joint_bridge_node",
        name="xarm_joint_bridge",
        output="screen",
        parameters=[{
            "left_in_topic":   LaunchConfiguration("left_in_topic"),
            "right_in_topic":  LaunchConfiguration("right_in_topic"),
            "left_out_topic":  LaunchConfiguration("left_out_topic"),
            "right_out_topic": LaunchConfiguration("right_out_topic"),
            "left_prefix":     LaunchConfiguration("left_prefix"),
            "right_prefix":    LaunchConfiguration("right_prefix"),
            "rename_joints":   LaunchConfiguration("rename_joints"),
            "qos_depth":       LaunchConfiguration("qos_depth"),
        }],
    )

    return LaunchDescription(args + [bridge_node])
