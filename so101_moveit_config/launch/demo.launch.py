"""MoveIt 2 for the SO-101: the control stack from so101_bringup, move_group and RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    moveit_config = (
        MoveItConfigsBuilder("so101", package_name="so101_moveit_config")
        .robot_description(
            file_path=os.path.join(get_package_share_directory("so101_description"), "urdf", "so101.urdf.xacro"),
            mappings={"ros2_control": "true", "use_mock_hardware": use_mock_hardware},
        )
        .robot_description_semantic(file_path="config/so101.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .to_moveit_configs()
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_mock_hardware", default_value="true",
                              description="mock_components/GenericSystem instead of the servo bus"),
        DeclareLaunchArgument("camera", default_value="false",
                              description="Start the wrist camera (image in RViz, hand-eye transform in TF)"),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare("so101_bringup"), "launch", "so101_control.launch.py"]),
            launch_arguments={"use_mock_hardware": use_mock_hardware}.items(),
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare("so101_perception"), "launch", "wrist_camera.launch.py"]),
            condition=IfCondition(LaunchConfiguration("camera")),
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            parameters=[moveit_config.to_dict(), {"publish_robot_description_semantic": True}],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", PathJoinSubstitution([FindPackageShare("so101_moveit_config"), "config", "moveit.rviz"])],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
        ),
    ])
