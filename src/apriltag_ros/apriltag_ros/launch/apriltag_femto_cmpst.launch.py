import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    apriltag_ros_dir = get_package_share_directory('apriltag_ros')
    config = os.path.join(apriltag_ros_dir, 'cfg', 'femto_tags.yaml')

    composable_node = ComposableNode(
        name='apriltag',
        package='apriltag_ros',
        plugin='AprilTagNode',
        parameters=[config],
        remappings=[
            ('/image', '/G_camera/color/image_raw'),
            ('/camera_info', '/G_camera/color/camera_info'),
        ],
    )

    container = ComposableNodeContainer(
        name='apriltag_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[composable_node],
        output='screen',
    )

    return LaunchDescription([container])

# launch using:
# ros2 launch your_package_name apriltag_femto_cmpst.launch.py