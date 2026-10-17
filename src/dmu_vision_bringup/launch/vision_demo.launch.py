"""One-command demo: image -> detections.

    ros2 launch dmu_vision_bringup vision_demo.launch.py

Starts:
    hikrobot_camera_ros2/camera_node   mock backend replaying a still scene
    yolo_ros/yolo_node                 detection + tracking + debug image

READ THIS BEFORE USING IT
    This is a REPLAY demo, not a camera. The `mock` backend republishes a still
    image; the node keeps reporting `mock_source: True` and logs a warning, and
    /vision/status stays honest about it. It exists so the whole
    image -> detection path can be demonstrated and timed without hardware.

    For real capture, run `camera.launch.py backend:=mvs` once the Linux MVS SDK
    and the camera are available (see extras/docs/CAMERA_DRIVER.md).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("dmu_vision_bringup")
    camera_share = get_package_share_directory("hikrobot_camera_ros2")
    yolo_launch = os.path.join(
        get_package_share_directory("yolo_bringup"), "launch", "yolo.launch.py")

    default_scene = os.path.expanduser("~/vision_ws/models/demo_scene.jpg")
    scene_image = LaunchConfiguration("scene_image")
    frame_id = LaunchConfiguration("frame_id")
    fps = LaunchConfiguration("fps")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scene_image",
                default_value=default_scene,
                description=(
                    "Still image the mock backend replays. Ships as "
                    "~/vision_ws/models/demo_scene.jpg (public test photo). "
                    "Missing file -> the node fails loudly instead of showing nothing."
                ),
            ),
            DeclareLaunchArgument(
                "frame_id", default_value="camera_optical_frame"),
            DeclareLaunchArgument(
                "fps", default_value="10.0",
                description="Replay rate. Keep it <= the measured detection rate or "
                            "debug_image will not pair (see extras/docs/TROUBLESHOOTING.md #8)."),
            LogInfo(
                msg="[vision_demo] REPLAY SOURCE: the mock backend republishes a still "
                    "image. This is NOT a real camera capture."),
            Node(
                package="hikrobot_camera_ros2",
                executable="camera_node",
                name="camera_node",
                output="screen",
                parameters=[
                    os.path.join(camera_share, "config", "camera.yaml"),
                    {
                        "backend": "mock",
                        "mock_image_path": scene_image,
                        "mock_width": 640,
                        "mock_height": 480,
                        "mock_encoding": "bgr8",
                        "mock_fps": fps,
                        "mock_pattern": "gradient",
                        "frame_id": frame_id,
                    },
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(yolo_launch),
                launch_arguments={
                    "params_file": os.path.join(
                        bringup_share, "config", "yolo_mono_raw.yaml"),
                    "namespace": "yolo",
                }.items(),
            ),
        ]
    )
