from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Include launch files
    package_dir = get_package_share_directory('orbbec_camera')
    launch_file_dir = os.path.join(package_dir, 'launch')


    right_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini2.launch.py')
        ),
        launch_arguments={
            #'serial_number': 'AY3794301A0',
            'camera_name': 'R_camera',
            'usb_port': '6-2',  # replace your usb port here
            #'device_num': '1',
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_colored_point_cloud': 'false', # for testing purposes these are off
            'depth_registration': 'false',
                # QoS for all streams
            'color_qos':            'SENSOR_DATA',
            'depth_qos':            'SENSOR_DATA',
            'color_camera_info_qos': 'SENSOR_DATA',
            'depth_camera_info_qos': 'SENSOR_DATA',
                # disable unneeded topics        
            'enable_accel': 'false',
            'enable_gyro': 'false',
            'enable_publish_extrinsic': 'false',
            'publish_tf': 'false',
            'enable_ir': 'false',
            'enable_point_cloud': 'false',
        }.items()
    )

    left_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini2.launch.py')
        ),
        launch_arguments={
            #'serial_number': 'AY3794301C4',
            'camera_name': 'L_camera',
            'usb_port': '6-1.1',  # replace your usb port here
            'device_num': '3',
            'enable_colored_point_cloud': 'true',
            'depth_registration': 'true',
                # QoS for all streams
            'color_qos':            'SENSOR_DATA',
            'depth_qos':            'SENSOR_DATA',
            'color_camera_info_qos': 'SENSOR_DATA',
            'depth_camera_info_qos': 'SENSOR_DATA',
                # disable unneeded topics        
            'enable_accel': 'false',
            'enable_gyro': 'false',
            'enable_publish_extrinsic': 'false',
            'publish_tf': 'false',
            'enable_ir': 'false',
            'enable_point_cloud': 'false',
        }.items()
    )

    # If you need more cameras, just add more launch_include here, and change the usb_port and device_num
    
    global_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'femto_bolt.launch.py')
        ),
        launch_arguments={
            #'serial_number': 'CL8855301G8',
            'usb_port': '2-1',  # replace your usb port here
            'camera_name': 'G_camera',
            'device_num': '3',
            'enable_colored_point_cloud': 'true',
            'depth_registration': 'true',
            'enable_depth': 'true',
            # 'depth_width': '640',
            # 'depth_height': '576',
            # 'depth_format': 'Y16',
            # 'depth_fps': '15',
            'enable_color': 'true',
            # 'color_width': '1920',
            # 'color_height': '1080',
            # 'color_format': 'MJPG',
            # 'color_fps': '15',
            #'enable_frame_sync': 'true',
                # QoS for all streams
            'color_qos':            'SENSOR_DATA',
            'depth_qos':            'SENSOR_DATA',
            'color_camera_info_qos': 'SENSOR_DATA',
            'depth_camera_info_qos': 'SENSOR_DATA',
                # disable unneeded topics
            'enable_accel': 'false',
            'enable_gyro': 'false',
            'enable_publish_extrinsic': 'false',
            'enable_point_cloud': 'false',
            'enable_ir': 'false',
            'publish_tf': 'false',
        }.items(),
    )


    # Launch description
    ld = LaunchDescription([
        GroupAction([right_camera]),
        #GroupAction([left_camera]),
        #GroupAction([global_camera]),
    ])

    return ld