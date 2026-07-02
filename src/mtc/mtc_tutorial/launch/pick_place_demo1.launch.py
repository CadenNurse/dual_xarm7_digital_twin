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
    pkg_share = get_package_share_directory("xarm_moveit_config")
    cfg = os.path.join(pkg_share, "dual_config", "config")

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
    xarm_ompl = load_yaml(
        "xarm_moveit_config", "dual_config/config/xarm7_ompl_planning.yaml"
    )
    gripper_ompl = load_yaml(
        "xarm_moveit_config", "dual_config/config/gripper_ompl_planning.yaml"
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

    moveit_params.setdefault("planning_pipelines", {})
    moveit_params["planning_pipelines"] = ["ompl"]

    moveit_params.setdefault("ompl", {})
    moveit_params["ompl"]["planning_plugin"] = "ompl_interface/OMPLPlanner"
    moveit_params["ompl"]["planning_plugins"] = ["ompl_interface/OMPLPlanner"]
    moveit_params["ompl"]["request_adapters"] = [
        # "default_planning_request_adapters/AddRuckigTrajectorySmoothing",
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
    merge_dict(moveit_params["ompl"], xarm_ompl)
    merge_dict(moveit_params["ompl"], gripper_ompl)

    pick_place_demo = Node(
        package="mtc_tutorial",
        executable="mtc_node_cntrl",
        output="screen",
        parameters=[moveit_params],
    )

    return LaunchDescription([pick_place_demo])