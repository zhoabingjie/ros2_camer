"""Formal vision pipeline.

Default source is the real Hikvision MVS camera. Mock operation is explicit:

    ros2 launch dmu_vision_bringup vision.launch.py backend:=mock

Stable interfaces from this launch:
  /camera/image_raw        sensor_msgs/msg/Image
  /camera/camera_info      sensor_msgs/msg/CameraInfo
  /vision/detections       yolo_msgs/msg/DetectionArray
  /vision/tracks           yolo_msgs/msg/DetectionArray
  /vision/debug_image      sensor_msgs/msg/Image
  /vision/obstacles        dmu_vision_interfaces/msg/VisualObstacleArray
  /vision/status           diagnostic_msgs/msg/DiagnosticArray

The upstream `yolo_bringup yolo.launch.py` remains available and keeps its
legacy /yolo/cam0/* topic names.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_share = get_package_share_directory("hikrobot_camera_ros2")
    bringup_share = get_package_share_directory("dmu_vision_bringup")
    obstacle_share = get_package_share_directory("visual_obstacle_avoidance")

    camera_params = LaunchConfiguration("camera_params_file")
    yolo_params = LaunchConfiguration("yolo_params_file")
    backend = LaunchConfiguration("backend")
    frame_id = LaunchConfiguration("frame_id")
    calibration_file = LaunchConfiguration("calibration_file")
    yolo_input = LaunchConfiguration("yolo_input_topic")
    mock_image_path = LaunchConfiguration("mock_image_path")
    mock_encoding = LaunchConfiguration("mock_encoding")
    mock_fps = LaunchConfiguration("mock_fps")
    mock_pattern = LaunchConfiguration("mock_pattern")
    start_camera = LaunchConfiguration("start_camera")
    start_detector = LaunchConfiguration("start_detector")
    start_obstacle = LaunchConfiguration("start_obstacle")
    start_rectify = LaunchConfiguration("start_rectify")

    camera_node = Node(
        package="hikrobot_camera_ros2",
        executable="camera_node",
        name="camera_node",
        output="screen",
        condition=IfCondition(start_camera),
        on_exit=Shutdown(reason="camera driver exited"),
        parameters=[
            camera_params,
            {
                "backend": backend,
                "frame_id": frame_id,
                "calibration_file": calibration_file,
                "require_camera_at_startup": True,
                "mock_image_path": mock_image_path,
                "mock_encoding": mock_encoding,
                "mock_fps": mock_fps,
                "mock_pattern": mock_pattern,
            },
        ],
    )

    rectifier = Node(
        package="image_proc",
        executable="rectify_node",
        name="RectifyNode",
        namespace="camera",
        output="screen",
        condition=IfCondition(start_rectify),
        remappings=[("image", "image_raw")],
    )

    detector = Node(
        package="yolo_ros",
        executable="yolo_node",
        name="yolo_node",
        namespace="yolo",
        output="screen",
        condition=IfCondition(start_detector),
        on_exit=Shutdown(reason="YOLO detector exited"),
        parameters=[
            yolo_params,
            {
                "cam0.rgb_topic": yolo_input,
                "cam0.image_reliability": 2,
            },
        ],
        remappings=[
            ("/yolo/cam0/detections", "/vision/detections"),
            ("/yolo/cam0/tracking", "/vision/tracks"),
            ("/yolo/cam0/debug_image", "/vision/debug_image"),
            ("/yolo/cam0/debug_bb_markers", "/vision/debug_bb_markers"),
            ("/yolo/cam0/debug_kp_markers", "/vision/debug_kp_markers"),
        ],
    )

    obstacle = Node(
        package="visual_obstacle_avoidance",
        executable="visual_obstacle_node",
        name="visual_obstacle_node",
        output="screen",
        condition=IfCondition(start_obstacle),
        on_exit=Shutdown(reason="visual obstacle node exited"),
        parameters=[LaunchConfiguration("visual_obstacle_params_file")],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "backend",
                default_value="mvs",
                description="Real Hikvision backend by default; use 'mock' only for offline testing",
            ),
            DeclareLaunchArgument(
                "camera_params_file",
                default_value=os.path.join(camera_share, "config", "camera.yaml"),
            ),
            DeclareLaunchArgument(
                "yolo_params_file",
                default_value=os.path.join(bringup_share, "config", "yolo_mono_raw.yaml"),
                description="Use yolo_mono.yaml only after a real calibration exists",
            ),
            DeclareLaunchArgument("frame_id", default_value="camera_optical_frame"),
            DeclareLaunchArgument("calibration_file", default_value=""),
            DeclareLaunchArgument("yolo_input_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument(
                "mock_image_path",
                default_value="",
                description="Still image used only when backend:=mock is explicit",
            ),
            DeclareLaunchArgument("mock_encoding", default_value="bgr8"),
            DeclareLaunchArgument("mock_fps", default_value="10.0"),
            DeclareLaunchArgument("mock_pattern", default_value="moving_bar"),
            DeclareLaunchArgument(
                "visual_obstacle_params_file",
                default_value=os.path.join(
                    obstacle_share, "config", "visual_obstacle.yaml"),
            ),
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("start_detector", default_value="true"),
            DeclareLaunchArgument("start_obstacle", default_value="true"),
            DeclareLaunchArgument(
                "start_rectify",
                default_value="false",
                description="Requires valid calibration; otherwise image_proc emits no image_rect",
            ),
            camera_node,
            rectifier,
            detector,
            obstacle,
        ]
    )
