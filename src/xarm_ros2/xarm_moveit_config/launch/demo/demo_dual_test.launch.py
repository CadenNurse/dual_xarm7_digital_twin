import os

from launch import LaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from uf_ros_lib.moveit_configs_builder import DualMoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    robot_ip_1 = LaunchConfiguration('robot_ip_1')
    robot_ip_2 = LaunchConfiguration('robot_ip_2')

    dof = LaunchConfiguration('dof', default='7')
    dof_1 = LaunchConfiguration('dof_1', default=dof)
    dof_2 = LaunchConfiguration('dof_2', default=dof)

    robot_type = LaunchConfiguration('robot_type', default='xarm')
    robot_type_1 = LaunchConfiguration('robot_type_1', default=robot_type)
    robot_type_2 = LaunchConfiguration('robot_type_2', default=robot_type)

    prefix_1 = LaunchConfiguration('prefix_1', default='L_')
    prefix_2 = LaunchConfiguration('prefix_2', default='R_')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')

    add_gripper = LaunchConfiguration('add_gripper', default='true')
    add_gripper_1 = LaunchConfiguration('add_gripper_1', default=add_gripper)
    add_gripper_2 = LaunchConfiguration('add_gripper_2', default=add_gripper)

    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default='false')
    add_vacuum_gripper_1 = LaunchConfiguration('add_vacuum_gripper_1', default=add_vacuum_gripper)
    add_vacuum_gripper_2 = LaunchConfiguration('add_vacuum_gripper_2', default=add_vacuum_gripper)

    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default='false')
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

    dual_arm_ros2_controllers_path = os.path.join(
        xarm_moveit_config_dir,
        'config',
        'dual_arm',
        'ros2_controllers.yaml'
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

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description],
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[dual_arm_ros2_controllers_path],
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
    )

    left_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            '{}{}_traj_controller'.format(prefix_1.perform(context), xarm_type_1),
            '--controller-manager', '/controller_manager',
        ],
    )

    right_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            '{}{}_traj_controller'.format(prefix_2.perform(context), xarm_type_2),
            '--controller-manager', '/controller_manager',
        ],
    )

    left_gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            '{}{}_gripper_controller'.format(prefix_1.perform(context), xarm_type_1),
            '--controller-manager', '/controller_manager',
        ],
    )

    right_gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            '{}{}_gripper_controller'.format(prefix_2.perform(context), xarm_type_2),
            '--controller-manager', '/controller_manager',
        ],
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict()],
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
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    delay_left_arm_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[left_arm_controller_spawner],
        )
    )

    delay_right_arm_after_left_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=left_arm_controller_spawner,
            on_exit=[right_arm_controller_spawner],
        )
    )

    delay_left_gripper_after_right_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=right_arm_controller_spawner,
            on_exit=[left_gripper_controller_spawner],
        )
    )

    delay_right_gripper_after_left_gripper = RegisterEventHandler(
        OnProcessExit(
            target_action=left_gripper_controller_spawner,
            on_exit=[right_gripper_controller_spawner],
        )
    )

    delay_moveit_after_right_gripper = RegisterEventHandler(
        OnProcessExit(
            target_action=right_gripper_controller_spawner,
            on_exit=[move_group_node, rviz_node],
        )
    )

    return [
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        delay_left_arm_after_jsb,
        delay_right_arm_after_left_arm,
        delay_left_gripper_after_right_arm,
        delay_right_gripper_after_left_gripper,
        delay_moveit_after_right_gripper,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])