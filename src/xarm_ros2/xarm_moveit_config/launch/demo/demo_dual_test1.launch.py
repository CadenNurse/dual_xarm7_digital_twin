import os
import yaml

from launch import LaunchDescription
from launch.actions import OpaqueFunction, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from uf_ros_lib.moveit_configs_builder import DualMoveItConfigsBuilder


def merge_dict(dst, src):
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            merge_dict(dst[k], v)
        else:
            dst[k] = v


def load_local_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):
    robot_ip_1 = LaunchConfiguration('robot_ip_1')
    robot_ip_2 = LaunchConfiguration('robot_ip_2')
    dof = LaunchConfiguration('dof', default=7)
    dof_1 = LaunchConfiguration('dof_1', default=dof)
    dof_2 = LaunchConfiguration('dof_2', default=dof)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    robot_type_1 = LaunchConfiguration('robot_type_1', default=robot_type)
    robot_type_2 = LaunchConfiguration('robot_type_2', default=robot_type)
    prefix_1 = LaunchConfiguration('prefix_1', default='L_')
    prefix_2 = LaunchConfiguration('prefix_2', default='R_')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_gripper_1 = LaunchConfiguration('add_gripper_1', default=add_gripper)
    add_gripper_2 = LaunchConfiguration('add_gripper_2', default=add_gripper)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_vacuum_gripper_1 = LaunchConfiguration('add_vacuum_gripper_1', default=add_vacuum_gripper)
    add_vacuum_gripper_2 = LaunchConfiguration('add_vacuum_gripper_2', default=add_vacuum_gripper)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    add_bio_gripper_1 = LaunchConfiguration('add_bio_gripper_1', default=add_bio_gripper)
    add_bio_gripper_2 = LaunchConfiguration('add_bio_gripper_2', default=add_bio_gripper)

    xarm_type_1 = '{}{}'.format(
        robot_type_1.perform(context),
        dof_1.perform(context) if robot_type_1.perform(context) in ('xarm', 'lite') else ''
    )
    xarm_type_2 = '{}{}'.format(
        robot_type_2.perform(context),
        dof_2.perform(context) if robot_type_2.perform(context) in ('xarm', 'lite') else ''
    )

    xarm_moveit_config_dir = get_package_share_directory('xarm_moveit_config')
    dual_cfg = os.path.join(xarm_moveit_config_dir, 'dual_config', 'config')

    dual_arm_ros2_controllers_path = os.path.join(
        xarm_moveit_config_dir, 'config', 'dual_arm', 'ros2_controllers.yaml'
    )

    moveit_config = DualMoveItConfigsBuilder(
        context=context,
        controllers_name='controllers',
        robot_ip_1=robot_ip_1,
        robot_ip_2=robot_ip_2,
        dof_1=dof_1,
        dof_2=dof_2,
        robot_type_1=robot_type_1,
        robot_type_2=robot_type_2,
        prefix_1=prefix_1,
        prefix_2=prefix_2,
        hw_ns=hw_ns,
        add_gripper_1=add_gripper_1,
        add_gripper_2=add_gripper_2,
        add_vacuum_gripper_1=add_vacuum_gripper_1,
        add_vacuum_gripper_2=add_vacuum_gripper_2,
        add_bio_gripper_1=add_bio_gripper_1,
        add_bio_gripper_2=add_bio_gripper_2,
        ros2_control_plugin='uf_robot_hardware/UFRobotSystemHardware',
        ros2_control_params=dual_arm_ros2_controllers_path,
    ).to_moveit_configs()

    moveit_params = moveit_config.to_dict() # create parameter file

    # joint limits
    xarm_joint_limits = load_local_yaml(os.path.join(dual_cfg, 'xarm7_joint_limits.yaml'))
    gripper_joint_limits = load_local_yaml(os.path.join(dual_cfg, 'xarm_gripper_joint_limits.yaml'))

    # ompl planning
    ompl_main = load_local_yaml(os.path.join(dual_cfg, 'ompl_main.yaml'))

    # kinematics and controllers
    kinematics = load_local_yaml(os.path.join(dual_cfg, 'kinematics.yaml'))
    moveit_ctrls = load_local_yaml(os.path.join(dual_cfg, 'moveit_controllers.yaml'))

    moveit_params["robot_description_kinematics"] = kinematics

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
    moveit_params["ompl"]["request_adapters"] = [
        "default_planning_request_adapters/ResolveConstraintFrames",
        "default_planning_request_adapters/ValidateWorkspaceBounds", # remove to reduce noise in terminal
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


    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_params,
            {"capabilities": "move_group/ExecuteTaskSolutionCapability"},
        ],
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare('xarm_moveit_config'),
        'rviz',
        'dual_moveit_camera.rviz'
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[moveit_params],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description],
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            moveit_config.robot_description,
            dual_arm_ros2_controllers_path,
        ],
        output='screen',
    )

    # set_left_collision_sensitivity = TimerAction(
    #     period=3.0,
    #     actions=[
    #         ExecuteProcess(
    #             cmd=[
    #                 'ros2', 'service', 'call',
    #                 '/L_xarm/set_collision_sensitivity',
    #                 'xarm_msgs/srv/SetInt16',
    #                 '{data: 3}'
    #             ],
    #             output='screen'
    #         )
    #     ]
    # )

    # set_right_collision_sensitivity = TimerAction(
    #     period=3.5,
    #     actions=[
    #         ExecuteProcess(
    #             cmd=[
    #                 'ros2', 'service', 'call',
    #                 '/R_xarm/set_collision_sensitivity',
    #                 'xarm_msgs/srv/SetInt16',
    #                 '{data: 3}'
    #             ],
    #             output='screen'
    #         )
    #     ]
    # )

    # # set_left_collision_rebound = TimerAction(
    # #     period=4.0,
    # #     actions=[
    # #         ExecuteProcess(
    # #             cmd=[
    # #                 'ros2', 'service', 'call',
    # #                 '/L_xarm/set_collision_rebound',
    # #                 'xarm_msgs/srv/SetInt16',
    # #                 '{data: 0}'
    # #             ],
    # #             output='screen'
    # #         )
    # #     ]
    # # )

    controllers = [
        'joint_state_broadcaster',
        '{}{}_traj_controller'.format(prefix_1.perform(context), xarm_type_1),
        '{}{}_traj_controller'.format(prefix_2.perform(context), xarm_type_2),
        # 'L_xarm_gripper',
        # 'R_xarm_gripper',
    ]

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'source_list': [
                '{}{}/joint_states'.format(prefix_1.perform(context), hw_ns.perform(context)),
                '{}{}/joint_states'.format(prefix_2.perform(context), hw_ns.perform(context))
            ],
        }],
    )

    controller_nodes = []
    for controller in controllers:
        controller_nodes.append(Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[
                controller,
                '--controller-manager', '/controller_manager',
            ],
        ))

    return [
        robot_state_publisher,
        joint_state_publisher_node,
        move_group_node,
        ros2_control_node,
        rviz_node,
        # set_left_collision_sensitivity,
        # set_right_collision_sensitivity,
    ] + controller_nodes


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])