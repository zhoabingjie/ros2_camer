"""Launch the single real Hikrobot MVS GigE camera driver."""

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
    frame_id = LaunchConfiguration("frame_id")
    calibration_file = LaunchConfiguration("calibration_file")
    require_camera = LaunchConfiguration("require_camera_at_startup")
    pixel_format = LaunchConfiguration("mvs_pixel_format")
    width = LaunchConfiguration("mvs_width")
    height = LaunchConfiguration("mvs_height")
    exposure_us = LaunchConfiguration("mvs_exposure_us")
    gain = LaunchConfiguration("mvs_gain")
    packet_size = LaunchConfiguration("mvs_packet_size")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="YAML parameter file for camera_node",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="camera_optical_frame",
                description="frame_id stamped on Image and CameraInfo",
            ),
            DeclareLaunchArgument(
                "calibration_file",
                default_value="",
                description="camera_calibration YAML; empty means uncalibrated",
            ),
            DeclareLaunchArgument(
                "require_camera_at_startup",
                default_value="true",
                description="Fail the process when the MVS camera cannot open",
            ),
            DeclareLaunchArgument(
                "mvs_pixel_format",
                default_value="",
                description="Mono8, BGR8, RGB8, or empty to keep the camera setting",
            ),
            DeclareLaunchArgument("mvs_width", default_value="0"),
            DeclareLaunchArgument("mvs_height", default_value="0"),
            DeclareLaunchArgument("mvs_exposure_us", default_value="0.0"),
            DeclareLaunchArgument("mvs_gain", default_value="0.0"),
            DeclareLaunchArgument("mvs_packet_size", default_value="0"),
            Node(
                package="hikrobot_camera_ros2",
                executable="camera_node",
                name="camera_node",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "frame_id": frame_id,
                        "calibration_file": calibration_file,
                        "require_camera_at_startup": require_camera,
                        "mvs_pixel_format": pixel_format,
                        "mvs_width": width,
                        "mvs_height": height,
                        "mvs_exposure_us": exposure_us,
                        "mvs_gain": gain,
                        "mvs_packet_size": packet_size,
                    },
                ],
            ),
        ]
    )
