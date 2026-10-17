"""Image preview pipeline (Phase 2).

    camera_node  ->  /camera/image_raw  + /camera/camera_info
                          |
                     image_proc rectify_node (namespace /camera)
                          |
                     /camera/image_rect

The viewers (rqt_image_view / image_view) are GUI applications and are NOT
started here, so the pipeline can be validated headless and on a machine without
a display. Start one separately:

    ros2 run rqt_image_view rqt_image_view        # then pick the topic
    ros2 run image_view image_view --ros-args -r image:=/camera/image_rect
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_params = os.path.join(
        get_package_share_directory("hikrobot_camera_ros2"), "config", "camera.yaml"
    )

    camera_params_file = LaunchConfiguration("camera_params_file")
    backend = LaunchConfiguration("backend")
    frame_id = LaunchConfiguration("frame_id")
    calibration_file = LaunchConfiguration("calibration_file")
    start_rectify = LaunchConfiguration("start_rectify")
    mock_pattern = LaunchConfiguration("mock_pattern")

    camera_node = Node(
        package="hikrobot_camera_ros2",
        executable="camera_node",
        name="camera_node",
        output="screen",
        parameters=[
            camera_params_file,
            {
                "backend": backend,
                "frame_id": frame_id,
                "calibration_file": calibration_file,
                "mock_pattern": mock_pattern,
            },
        ],
    )

    # IMPORTANT: image_proc::RectifyNode's INPUT topic is named `image`, NOT
    # `image_raw`. Verified two ways:
    #   1. the node's own package launch remaps ('image', 'image_mono') and
    #      ('image', 'image_color');
    #   2. running it under /camera without a remap left
    #      `ros2 topic info /camera/image_raw` reporting "Subscription count: 0",
    #      and /camera/image_rect published no frames.
    # So the camera's `image_raw` must be remapped onto the node's `image` input.
    # Output stays `image_rect`; camera_info lines up on its own.
    rectify_node = Node(
        package="image_proc",
        executable="rectify_node",
        name="RectifyNode",
        namespace="camera",
        output="screen",
        remappings=[("image", "image_raw")],
        condition=IfCondition(start_rectify),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_params_file",
                default_value=camera_params,
                description="hikrobot_camera_ros2 parameter file",
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
                description=(
                    "camera_calibration YAML. Empty means uncalibrated; image_proc then "
                    "passes the image through instead of rectifying with invented intrinsics."
                ),
            ),
            DeclareLaunchArgument(
                "mock_pattern",
                default_value="moving_bar",
                description=(
                    "mock backend pattern. Use 'gradient' for a time-invariant frame, which "
                    "makes raw-vs-rect pixel comparison meaningful in verification scripts."
                ),
            ),
            DeclareLaunchArgument(
                "start_rectify",
                default_value="true",
                description="Start image_proc rectify_node",
            ),
            camera_node,
            rectify_node,
        ]
    )
