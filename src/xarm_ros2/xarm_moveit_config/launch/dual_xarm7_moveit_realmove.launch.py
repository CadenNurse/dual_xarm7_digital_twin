#!/usr/bin/env python3

# if you change any of these parameters, make sure you delete the display in .rviz file and 
# reintroduce it in rviz

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
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
                'demo_dual_realmove_new.launch.py'
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

    right_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orbbec_camera_dir, 'launch', 'gemini2.launch.py')
        ),
        launch_arguments={
            #'serial_number': 'AY3794301A0',
            'usb_port': '4-4.2',
            'camera_name': 'R_camera',
            'device_num': '3',
            'enable_depth': 'true',
            'depth_fps': '15',
            'enable_color': 'true',
            'color_fps': '15',
            'enable_ir': 'false',
            'enable_point_cloud': 'false',
            'enable_colored_point_cloud': 'true',
            'depth_registration': 'true',
            'publish_tf': 'false',
                # QoS for all streams
            'color_qos':            'SENSOR_DATA',
            'depth_qos':            'SENSOR_DATA',
            'ir_qos':               'SENSOR_DATA',
            'point_cloud_qos':      'SENSOR_DATA',
            'color_camera_info_qos': 'SENSOR_DATA',
            'depth_camera_info_qos': 'SENSOR_DATA',
            'enable_accel': 'false',
            'enable_gyro': 'false',
            'enable_publish_extrinsic': 'false',
        }.items(),
    )

    left_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orbbec_camera_dir, 'launch', 'gemini2.launch.py')
        ),
        launch_arguments={
            #'serial_number': 'AY3794301C4',
            'usb_port': '4-4.3',
            'camera_name': 'L_camera',
            'device_num': '3',
            'enable_depth': 'true',
            'depth_fps': '15',
            'enable_color': 'true',
            'color_fps': '15',
            'enable_ir': 'false',
            'enable_point_cloud': 'false',
            'enable_colored_point_cloud': 'true',
            'depth_registration': 'true',
            'publish_tf': 'false',
                # QoS for all streams
            'color_qos':            'SENSOR_DATA',
            'depth_qos':            'SENSOR_DATA',
            'ir_qos':               'SENSOR_DATA',
            'point_cloud_qos':      'SENSOR_DATA',
            'color_camera_info_qos': 'SENSOR_DATA',
            'depth_camera_info_qos': 'SENSOR_DATA',
            'enable_accel': 'false',
            'enable_gyro': 'false',
            'enable_publish_extrinsic': 'false',
        }.items(),
    )

    global_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orbbec_camera_dir, 'launch', 'femto_bolt.launch.py')
        ),
        launch_arguments={
            #'serial_number': 'CL8855301G8',
            'usb_port': '2-1',
            'camera_name': 'G_camera',
            'device_num': '3',
            'enable_depth': 'true',
            'depth_width': '640',
            'depth_height': '576',
            'depth_format': 'Y16',
            'depth_fps': '15',
            'enable_color': 'true',
            'color_width': '1920',
            'color_height': '1080',
            'color_format': 'MJPG',
            'color_fps': '15',
            'enable_ir': 'false',
            'enable_point_cloud': 'false',
            'enable_colored_point_cloud': 'true',
            'depth_registration': 'true',
            'publish_tf': 'false',
            'enable_frame_sync': 'true',
                # QoS for all streams
            'color_qos':            'SENSOR_DATA',
            'depth_qos':            'SENSOR_DATA',
            'ir_qos':               'SENSOR_DATA',
            'point_cloud_qos':      'SENSOR_DATA',
            'color_camera_info_qos': 'SENSOR_DATA',
            'depth_camera_info_qos': 'SENSOR_DATA',
            'enable_accel': 'false',
            'enable_gyro': 'false',
            'enable_publish_extrinsic': 'false',
        }.items(),
    )

    wall_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(xarm_moveit_config_dir, 'launch', 'wall.launch.py')
        )
    )

    apriltag_node = Node(
        package='apriltag_ros',
        executable='tag_detector',
        name='apriltag',
        output='screen',
        remappings=[
            ('image', '/G_camera/color/image_raw'),
            ('camera_info', '/G_camera/color/camera_info'),
        ],
        parameters=[
            os.path.join(apriltag_ros_dir, 'cfg', 'femto_tags.yaml')
        ],
    )

    return LaunchDescription([
        dual_xarm,
        TimerAction(period=2.0, actions=[right_camera]),
        TimerAction(period=3.0, actions=[wall_node]),
        TimerAction(period=4.0, actions=[left_camera]),
        TimerAction(period=6.0, actions=[global_camera]),
        TimerAction(period=8.0, actions=[apriltag_node]),
    ])