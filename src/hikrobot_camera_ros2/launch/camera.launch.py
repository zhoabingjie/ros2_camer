"""Launch the Hikrobot camera adapter (Phase 1, C++).

Only the camera node is started here. image_proc / preview / detection live in
their own launch files so each stage can be validated independently.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("hikrobot_camera_ros2"), "config", "camera.yaml"
    )

    params_file = LaunchConfiguration("params_file")
    backend = LaunchConfiguration("backend")
    frame_id = LaunchConfiguration("frame_id")
    calibration_file = LaunchConfiguration("calibration_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="YAML parameter file for camera_node",
            ),
            DeclareLaunchArgument(
                "backend",
                default_value="mock",
                description="'mock' for synthetic frames, 'mvs' for the real Hikvision camera",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="camera_optical_frame",
                description="frame_id stamped on Image and CameraInfo",
            ),
            DeclareLaunchArgument(
                "calibration_file",
                default_value="",
                description="Path to a camera_calibration YAML; empty means uncalibrated",
            ),
            Node(
                package="hikrobot_camera_ros2",
                executable="camera_node",
                name="camera_node",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "backend": backend,
                        "frame_id": frame_id,
                        "calibration_file": calibration_file,
                    },
                ],
            ),
        ]
    )
