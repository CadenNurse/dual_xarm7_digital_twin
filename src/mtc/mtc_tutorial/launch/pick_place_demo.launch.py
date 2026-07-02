from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import xacro


def load_yaml(package_name, relative_path):
    pkg_share = get_package_share_directory(package_name)
    abs_path = os.path.join(pkg_share, relative_path)
    with open(abs_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    pkg_share = get_package_share_directory("xarm_moveit_config")
    cfg = os.path.join(pkg_share, "dual_config", "config")
    # xarm_joint_limits = os.path.join(cfg, "xarm_gripper_joint_limits.yaml")

    robot_description = {
        "robot_description": xacro.process_file(
            os.path.join(cfg, "dual_xarm_device.urdf.xacro")
        ).toxml()
    }
    
    robot_description_planning = {
        "robot_description_planning": load_yaml(
            "xarm_moveit_config",
            "dual_config/config/xarm_gripper_joint_limits.yaml",
        )
    }

    robot_description_semantic = {
        "robot_description_semantic": xacro.process_file(
            os.path.join(cfg, "dual_xarm.srdf.xacro"),
            mappings={
                "add_gripper_1": "true",
                "add_gripper_2": "true",
            },
        ).toxml()
    }

    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            "xarm_moveit_config",
            "dual_config/config/kinematics.yaml",
        )
    }

    moveit_controllers = {
        "moveit_simple_controller_manager": load_yaml(
            "xarm_moveit_config",
            "dual_config/config/moveit_controllers.yaml",
        )
    }

    pick_place_demo = Node(
        package="mtc_tutorial",
        executable="mtc_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            moveit_controllers,
            robot_description_planning,
        ],
    )

    return LaunchDescription([pick_place_demo])