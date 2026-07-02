#!/usr/bin/env python3

# if you change any of these parameters, make sure you delete the display in .rviz file and 
# reintroduce it in rviz

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    xarm_moveit_config_dir = get_package_share_directory('xarm_moveit_config')
    orbbec_camera_dir = get_package_share_directory('orbbec_camera')
    apriltag_ros_dir = get_package_share_directory('apriltag_ros')

    robot1_ip = '192.168.1.211'
    robot2_ip = '192.168.1.221'

    dual_xarm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                xarm_moveit_config_dir,
                'launch',
                'demo',
                'demo_dual_test1.launch.py' # change to the launch file you want to use for the dual xarm demo
            )
        ),
        launch_arguments={
            'dof_1': '7',
            'dof_2': '7',
            'robot_type_1': 'xarm',
            'robot_type_2': 'xarm',
            'prefix_1': 'L_',
            'prefix_2': 'R_',
            'robot_ip_1': robot1_ip,
            'robot_ip_2': robot2_ip,
            'add_gripper_1': 'true',
            'add_gripper_2': 'true',
            'add_vacuum_gripper_1': 'false',
            'add_vacuum_gripper_2': 'false',
            'add_bio_gripper_1': 'false',
            'add_bio_gripper_2': 'false',
            'hw_ns': 'xarm',
            'limited': 'false',
            'effort_control': 'false',
            'velocity_control': 'false',
            'no_gui_ctrl': 'false',
        }.items(),
    )

    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                xarm_moveit_config_dir,
                'launch',
                'multi_camera.launch.py'
            )
        )
    )

    wall_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(xarm_moveit_config_dir, 'launch', 'wall.launch.py')
        )
    )

    apriltag_ros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(apriltag_ros_dir, 'launch', 'apriltag_femto_std.launch.py')
        )
    )

    return LaunchDescription([
        dual_xarm,
        cameras,
        wall_node,
        apriltag_ros,
    ])