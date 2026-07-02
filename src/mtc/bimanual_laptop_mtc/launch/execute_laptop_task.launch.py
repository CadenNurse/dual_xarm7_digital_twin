from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("bimanual_laptop_mtc")
    params = os.path.join(pkg_share, "config", "task_params.yaml")

    return LaunchDescription([
        Node(
            package="bimanual_laptop_mtc",
            executable="execute_laptop_task_server",
            name="execute_laptop_task_server",
            output="screen",
            parameters=[
                params, 
                moveit_config.to_dict(),
            ],
        )
    ])