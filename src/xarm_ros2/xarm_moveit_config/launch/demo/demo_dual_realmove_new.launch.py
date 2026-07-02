import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
# from uf_ros_lib.uf_robot_utils import generate_dual_ros2_control_params_temp_file
from uf_ros_lib.moveit_configs_builder import DualMoveItConfigsBuilder
import yaml 
# Import yaml for the trajectory override
import pprint
#for pprint for debugging

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

    xarm_type_1 = '{}{}'.format(robot_type_1.perform(context), dof_1.perform(context) if robot_type_1.perform(context) in ('xarm', 'lite') else '')
    xarm_type_2 = '{}{}'.format(robot_type_2.perform(context), dof_2.perform(context) if robot_type_2.perform(context) in ('xarm', 'lite') else '')
    
    xarm_moveit_config_dir = get_package_share_directory('xarm_moveit_config')


    dual_arm_ros2_controllers_path = os.path.join(
        xarm_moveit_config_dir,
        'config',
        'dual_arm',
        'ros2_controllers.yaml'
    )

    dual_arm_moveit_controllers_path = os.path.join(
        xarm_moveit_config_dir,
        'config',
        'dual_arm',
        'moveit_controllers.yaml'
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
        ros2_control_params=dual_arm_ros2_controllers_path, # added dual_arm_ in front of ros2_...
    ).to_moveit_configs()

    # load personallized controllers overtop of generated ones in moveit_config.to_dict()
    # with open(dual_arm_moveit_controllers_path, "r") as f:
    #     moveit_controllers = yaml.safe_load(f)

    # Start the actual move_group node/action server
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),

            # moveit_controllers,
        ],
    )

    # Launch RViz
    rviz_config = PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'rviz', 'dual_moveit_camera.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        #name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description],
    )

    # DEBUGGING
    # print("USING ROS2 CONTROLLERS FILE:", dual_arm_ros2_controllers_path)
    # print("USING MOVEIT CONTROLLERS FILE:", dual_arm_moveit_controllers_path)
    # with open(dual_arm_moveit_controllers_path, "r") as f:
    #     print(f.read())
    # pp = pprint.PrettyPrinter(depth=6)
    # pp.pprint(moveit_config.to_dict())
    # print(moveit_config.to_dict().get("moveit_simple_controller_manager", {}))

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            moveit_config.robot_description,
            dual_arm_ros2_controllers_path, # added dual_arm_ in front of ros2_...
        ],
        output='screen',
    )

    controllers = [
        'joint_state_broadcaster',
        '{}{}_traj_controller'.format(prefix_1.perform(context), xarm_type_1),
        '{}{}_traj_controller'.format(prefix_2.perform(context), xarm_type_2),
        # '{}xarm_gripper'.format(prefix_1.perform(context), xarm_type_1),
        # '{}xarm_gripper'.format(prefix_2.perform(context), xarm_type_2),
    ]    
    
    #Consider using joint_state_broadcaster instead of joint_state_publisher for better performance
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

    execute_laptop_task_server = Node(
        package='bimanual_laptop_mtc',
        executable='execute_laptop_task_server',
        name='execute_laptop_task_server',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'use_sim_time': False,
            },
        ],
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
                #'--param-file', dual_arm_ros2_controllers_path,
            ],
        ))
    
    return [
        robot_state_publisher,
        joint_state_publisher_node,
        move_group_node,
        ros2_control_node,
        rviz_node,
        execute_laptop_task_server,
    ] + controller_nodes


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])