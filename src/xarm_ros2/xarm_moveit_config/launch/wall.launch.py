from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xarm_moveit_config',
            executable='add_wall.py',
            name='add_wall',
            output='screen'
        ),
        Node(
            package='xarm_moveit_config',
            executable='add_posts.py',
            name='add_posts',
            output='screen'
        )
    ])