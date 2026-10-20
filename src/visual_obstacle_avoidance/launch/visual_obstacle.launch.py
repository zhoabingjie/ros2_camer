"""Launch only the monocular visual obstacle perception node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("visual_obstacle_avoidance"),
        "config",
        "visual_obstacle.yaml",
    )
    params_file = LaunchConfiguration("params_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Visual obstacle parameter file",
            ),
            Node(
                package="visual_obstacle_avoidance",
                executable="visual_obstacle_node",
                name="visual_obstacle_node",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
