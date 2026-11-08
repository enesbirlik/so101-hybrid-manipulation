"""The SO-101 dashboard window. Start it next to a running control stack (so101_bringup / so101_moveit_config)."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="so101_dashboard", executable="dashboard", output="screen"),
    ])
