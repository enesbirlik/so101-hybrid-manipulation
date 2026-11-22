"""Cube pick/place action server; optionally a FAKE cube publisher and a simulated contact (mock only).

  ros2 launch so101_manipulation pick_place.launch.py                      # with the real detector running
  ros2 launch so101_manipulation pick_place.launch.py fake_cube:=true sim_contact_deg:=12.6   # mock tests
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=PathJoinSubstitution(
            [FindPackageShare("so101_manipulation"), "config", "pick_place.yaml"])),
        DeclareLaunchArgument("sim_contact_deg", default_value="nan",
                              description="MOCK ONLY: simulated cube contact angle (refused on real hardware)"),
        DeclareLaunchArgument("fake_cube", default_value="false", description="publish a FAKE cube on /cubes/detections"),
        DeclareLaunchArgument("fake_x", default_value="0.22"),
        DeclareLaunchArgument("fake_y", default_value="0.03"),
        DeclareLaunchArgument("fake_yaw_deg", default_value="20.0"),
        DeclareLaunchArgument("fake_visible_cone_deg", default_value="0.0"),
        Node(
            package="so101_manipulation", executable="pick_place", name="cube_pick_place", output="screen",
            parameters=[arg("params_file"),
                        {"sim_contact_deg": ParameterValue(arg("sim_contact_deg"), value_type=float)}],
        ),
        Node(
            package="so101_manipulation", executable="fake_cube_publisher", output="screen",
            condition=IfCondition(arg("fake_cube")),
            parameters=[{"x": ParameterValue(arg("fake_x"), value_type=float),
                         "y": ParameterValue(arg("fake_y"), value_type=float),
                         "yaw_deg": ParameterValue(arg("fake_yaw_deg"), value_type=float),
                         "visible_cone_deg": ParameterValue(arg("fake_visible_cone_deg"), value_type=float)}],
        ),
    ])
