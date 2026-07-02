from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("xarm7", package_name="xarm_moveit_config")
        # .robot_description(
        #     file_path="/home/cadennurse/xarm_ws/src/xarm_ros2/xarm_description/urdf/dual_arm_device.urdf.xacro"
        # )
        .robot_description_semantic(
            file_path="srdf/dual_xarm.srdf.xacro"
        )
        .robot_description_kinematics(
            file_path="config/xarm7/kinematics.yaml"
        )
        .joint_limits(
            file_path="config/xarm7/joint_limits.yaml"
        )
        .trajectory_execution(
            file_path="config/dual_arm/moveit_controllers.yaml"
        )
        .planning_pipelines(
            pipelines=["ompl"],
            default_planning_pipeline="ompl",
            # config_folder="config/moveit_configs"
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    return LaunchDescription([
        Node(
            package="push_box",
            executable="push_box_node",
            name="push_box_node",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {
                    "planning_group": "L_xarm7",
                    "planning_frame": "world",
                    "object_frame": "object_nominal",
                    "ee_link": "L_link_tcp",
                    "box_height": 0.12,
                    "box_width": 0.07,
                    "box_thickness": 0.02,
                    "approach_standoff": 0.05,
                    "push_distance": 0.04,
                    "cartesian_eef_step": 0.005,
                    "cartesian_fraction_min": 0.90,
                    "initial_joint_positions": [
                        -1.0472,
                        0.8378,
                        0.4712,
                        1.5359,
                        -2.4435,
                        -1.6920,
                        0.7330,
                    ],
                },
            ],
        ),
    ])

    return LaunchDescription([push_box_node])
# launch with: ros2 launch push_box push_box.launch.py