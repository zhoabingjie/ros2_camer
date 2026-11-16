"""启动单台海康 USB3 Vision 相机；正式参数统一由 YAML 读取。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("camera_driver"), "config", "camera.yaml"
    )

    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="camera_driver YAML 参数文件",
            ),
            Node(
                package="camera_driver",
                executable="camera_node",
                name="camera_node",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
