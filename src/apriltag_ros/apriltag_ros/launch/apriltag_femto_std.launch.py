import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    apriltag_ros_dir = get_package_share_directory('apriltag_ros')

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

    laptop_node = Node(
        package='apriltag_ros',
        executable='tag_detector',
        name='apriltag', # both named same thing
        output='screen',
        remappings=[
            ('image', '/G_camera/color/image_raw'),
            ('camera_info', '/G_camera/color/camera_info'),
        ],
        parameters=[
            os.path.join(apriltag_ros_dir, 'cfg', 'laptop_tags.yaml')
        ],
    )

    workspace_origin_node = Node(
        package='apriltag_ros',
        executable='workspace_origin_broadcaster.py',
        name='workspace_origin_broadcaster',
        output='screen',
        parameters=[{
            'detections_topic': '/apriltag_detections',
            'camera_frame_fallback': 'G_camera_color_optical_frame',
            'origin_tag_id': 4,
            'origin_frame': 'workspace_origin',
            'tag_frame_prefix': 'tag_',
            'publish_tag_frame': True,
        }],
    )

    return LaunchDescription([
        apriltag_node,
        workspace_origin_node,
        laptop_node,
    ])

# launch using:
# ros2 launch apriltag_ros apriltag_femto_std.launch.py