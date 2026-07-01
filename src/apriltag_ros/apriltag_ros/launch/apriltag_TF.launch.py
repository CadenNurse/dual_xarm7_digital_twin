from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='workspace_to_tag4',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'workspace_origin', 'tag_4'],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='workspace_to_tag5',
            arguments=['W', '0', '0', '0', '0', '0', '1', 'workspace_origin', 'tag_5'],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='workspace_to_tag2',
            arguments=['0', 'D', '0', '0', '0', '0', '1', 'workspace_origin', 'tag_2'],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='workspace_to_tag3',
            arguments=['W', 'D', '0', '0', '0', '0', '1', 'workspace_origin', 'tag_3'],
            output='screen',
        ),
    ])