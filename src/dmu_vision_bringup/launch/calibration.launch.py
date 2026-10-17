"""Interactive monocular calibration (Phase 2).

REQUIRES a real camera AND a printed calibration target. cameracalibrator opens
an OpenCV GUI window, collects samples as you move the board, and only then can
it emit a calibration. There is no headless path and no way to produce a valid
calibration without the physical target -- see extras/docs/CAMERA_CALIBRATION.md.

cameracalibrator normally refuses to start unless a <camera>/set_camera_info
service is present. This adapter deliberately does not implement that service
yet, so the launch passes --no-service-check and the resulting calibration is
collected from /tmp/calibrationdata.tar.gz instead of being pushed to a service.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_params = os.path.join(
        get_package_share_directory("hikrobot_camera_ros2"), "config", "camera.yaml"
    )

    camera_params_file = LaunchConfiguration("camera_params_file")
    backend = LaunchConfiguration("backend")
    frame_id = LaunchConfiguration("frame_id")
    pattern = LaunchConfiguration("pattern")
    size = LaunchConfiguration("size")
    square = LaunchConfiguration("square")
    camera_name = LaunchConfiguration("camera_name")

    camera_node = Node(
        package="hikrobot_camera_ros2",
        executable="camera_node",
        name="camera_node",
        output="screen",
        parameters=[
            camera_params_file,
            {"backend": backend, "frame_id": frame_id},
        ],
    )

    calibrator = ExecuteProcess(
        cmd=[
            "ros2", "run", "camera_calibration", "cameracalibrator",
            "--pattern", pattern,
            "--size", size,
            "--square", square,
            "--camera_name", camera_name,
            "--no-service-check",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_params_file", default_value=camera_params),
            DeclareLaunchArgument(
                "backend", default_value="mock",
                description="'mock' only proves the wiring; a real calibration needs backend:=mvs",
            ),
            DeclareLaunchArgument("frame_id", default_value="camera_optical_frame"),
            DeclareLaunchArgument(
                "pattern", default_value="chessboard",
                description="chessboard | circles | acircles | charuco (see cameracalibrator --help)",
            ),
            DeclareLaunchArgument(
                "size", default_value="8x6",
                description="INTERIOR corner count, e.g. 8x6 for a 9x7-square board",
            ),
            DeclareLaunchArgument(
                "square", default_value="0.025",
                description="square size in METRES (measure your printed board)",
            ),
            DeclareLaunchArgument("camera_name", default_value="cam0"),
            camera_node,
            calibrator,
        ]
    )
