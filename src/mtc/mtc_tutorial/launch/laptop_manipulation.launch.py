import os
import yaml
import xacro

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def merge_dict(dst, src):
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            merge_dict(dst[k], v)
        else:
            dst[k] = v


def load_yaml(package_name, relative_path):
    pkg_share = get_package_share_directory(package_name)
    abs_path = os.path.join(pkg_share, relative_path)
    with open(abs_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    xarm_moveit_share = get_package_share_directory("xarm_moveit_config")
    xarm_description_dir = get_package_share_directory("xarm_description")
    cfg = os.path.join(xarm_moveit_share, "dual_config", "config")

    moveit_params = {
        "robot_description": xacro.process_file(
            os.path.join(cfg, "dual_xarm_device.urdf.xacro")
        ).toxml(),
        "robot_description_semantic": xacro.process_file(
            os.path.join(cfg, "dual_xarm.srdf.xacro"),
            mappings={
                "add_gripper_1": "true",
                "add_gripper_2": "true",
            },
        ).toxml(),
        "robot_description_kinematics": load_yaml(
            "xarm_moveit_config", "dual_config/config/kinematics.yaml"
        ),
    }

    xarm_joint_limits = load_yaml(
        "xarm_moveit_config", "dual_config/config/xarm7_joint_limits.yaml"
    )
    gripper_joint_limits = load_yaml(
        "xarm_moveit_config", "dual_config/config/xarm_gripper_joint_limits.yaml"
    )
    ompl_main = load_yaml(
        "xarm_moveit_config", "dual_config/config/ompl_main.yaml"
    )
    moveit_ctrls = load_yaml(
        "xarm_moveit_config", "dual_config/config/moveit_controllers.yaml"
    )

    moveit_params.setdefault("robot_description_planning", {})
    moveit_params["robot_description_planning"].setdefault("joint_limits", {})
    merge_dict(
        moveit_params["robot_description_planning"]["joint_limits"],
        xarm_joint_limits.get("joint_limits", {})
    )
    merge_dict(
        moveit_params["robot_description_planning"]["joint_limits"],
        gripper_joint_limits.get("joint_limits", {})
    )

    merge_dict(moveit_params, moveit_ctrls)

    moveit_params["planning_pipelines"] = ["ompl"]
    moveit_params.setdefault("ompl", {})
    moveit_params["ompl"]["planning_plugin"] = "ompl_interface/OMPLPlanner"
    moveit_params["ompl"]["planning_plugins"] = ["ompl_interface/OMPLPlanner"]
    moveit_params["ompl"]["request_adapters"] = [
        "default_planning_request_adapters/ResolveConstraintFrames",
        "default_planning_request_adapters/ValidateWorkspaceBounds",
        "default_planning_request_adapters/CheckStartStateBounds",
        "default_planning_request_adapters/CheckStartStateCollision",
    ]
    moveit_params["ompl"]["response_adapters"] = [
        "default_planning_response_adapters/AddTimeOptimalParameterization",
        "default_planning_response_adapters/ValidateSolution",
        "default_planning_response_adapters/DisplayMotionPath",
    ]
    moveit_params["ompl"]["start_state_max_bounds_error"] = 0.1
    merge_dict(moveit_params["ompl"], ompl_main)

    laptop_xacro_path = os.path.join(
        xarm_description_dir, "urdf", "other", "thinkpad_x13_gen1.urdf.xacro"
    )
    laptop_robot_description = xacro.process_file(laptop_xacro_path).toxml()

    close_laptop_node = Node(
        package="mtc_tutorial",
        executable="close_laptop",
        output="screen",
        arguments=["--ros-args", "--log-level", "info"],
        parameters=[{
            "robot_description": moveit_params["robot_description"],
            "robot_description_semantic": moveit_params["robot_description_semantic"],
            "robot_description_kinematics": moveit_params["robot_description_kinematics"],
            "robot_description_planning": moveit_params["robot_description_planning"],
            "planning_pipelines": moveit_params["planning_pipelines"],
            "ompl": moveit_params["ompl"],
        }],
    )

    laptop_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": laptop_robot_description,
        }],
        remappings=[
            ("joint_states", "/laptop/joint_states"),
        ],
    )

    laptop_hinge_state_publisher_node = Node(
        package="mtc_tutorial",
        executable="laptop_hinge_state_publisher",
        namespace="laptop",
        name="laptop_hinge_state_publisher",
        output="screen",
        parameters=[{
            "world_frame": "workspace_origin",
            "laptop_root_frame": "laptop_world",
            "base_tag_frame": "tag_laptop_base",
            "lid_inner_tag_frame": "tag_laptop_lid_inner",
            "lid_outer_tag_frame": "tag_laptop_lid_outer",
            "hinge_joint_name": "hinge_joint",
            "hinge_lower": 0.0,
            "hinge_upper": 3.1416,
            "publish_rate_hz": 10.0,
            "base_tag_to_root_xyz": [0.0, 0.0, 0.0],
            "base_tag_to_root_rpy": [1.5708, 3.1416, 1.5708],
        }],
    )

    return LaunchDescription([
        laptop_state_publisher,
        laptop_hinge_state_publisher_node,
        # close_laptop_node,
    ])