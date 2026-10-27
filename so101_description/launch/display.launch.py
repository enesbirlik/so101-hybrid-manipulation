"""robot_state_publisher + (optional) joint_state_publisher_gui + RViz for the SO-101."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("so101_description")
    use_wrist_camera = LaunchConfiguration("use_wrist_camera")
    gui = LaunchConfiguration("gui")

    robot_description = ParameterValue(
        Command([
            "xacro ", PathJoinSubstitution([pkg, "urdf", "so101.urdf.xacro"]),
            " use_wrist_camera:=", use_wrist_camera,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_wrist_camera", default_value="true",
                              description="Add the wrist camera mount and wrist_camera_link"),
        DeclareLaunchArgument("gui", default_value="true",
                              description="Start joint_state_publisher_gui (false: joint states come from elsewhere)"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(gui),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", PathJoinSubstitution([pkg, "rviz", "display.rviz"])],
        ),
    ])
