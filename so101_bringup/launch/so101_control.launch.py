"""robot_state_publisher + controller_manager (mock or real hardware) + the SO-101 controllers."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    robot_description = ParameterValue(
        Command([
            "xacro ", PathJoinSubstitution([FindPackageShare("so101_description"), "urdf", "so101.urdf.xacro"]),
            " ros2_control:=true use_mock_hardware:=", use_mock_hardware,
        ]),
        value_type=str,
    )
    controllers = PathJoinSubstitution([FindPackageShare("so101_bringup"), "config", "ros2_controllers.yaml"])

    def spawner(name):
        return Node(package="controller_manager", executable="spawner", arguments=[name])

    return LaunchDescription([
        DeclareLaunchArgument("use_mock_hardware", default_value="true",
                              description="mock_components/GenericSystem instead of the servo bus"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[controllers],
            remappings=[("~/robot_description", "/robot_description")],
            output="screen",
        ),
        spawner("joint_state_broadcaster"),
        spawner("arm_controller"),
        spawner("gripper_controller"),
    ])
