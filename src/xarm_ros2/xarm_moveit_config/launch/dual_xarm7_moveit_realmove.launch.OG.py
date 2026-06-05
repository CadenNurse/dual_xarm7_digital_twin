#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    xarm_moveit_config_dir = get_package_share_directory('xarm_moveit_config')

    robot1_ip = '192.168.1.211'
    robot2_ip = '192.168.1.221'

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    xarm_moveit_config_dir,
                    'launch',
                    'demo',
                    'demo_dual_realmove.launch.py'
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
    ])