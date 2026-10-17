"""YOLO 2D detection / tracking (Phase 3).

Reuses the UPSTREAM launcher (`yolo_bringup/launch/yolo.launch.py`) rather than
re-implementing it: upstream keeps the whole pipeline (cameras, ordered plugins,
plugin classes, every parameter) in the YAML, and its launch only forwards
`params_file` + `namespace`. We therefore supply our own monocular YAML and let
upstream do the rest -- which is exactly what the Phase 0 audit recommended.

Output topics (verified from source, namespace `yolo`, camera `cam0`):
    /yolo/cam0/detections        yolo_msgs/msg/DetectionArray
    /yolo/cam0/tracking          yolo_msgs/msg/DetectionArray
    /yolo/cam0/debug_image       sensor_msgs/msg/Image
    /yolo/cam0/debug_bb_markers  visualization_msgs/msg/MarkerArray
    /yolo/cam0/debug_kp_markers  visualization_msgs/msg/MarkerArray

Requires a model file. `config/yolo_mono.yaml` points at
/home/asd/vision_ws/models/yolo26n.onnx; without it the node fails loudly at
configure time instead of silently producing no detections.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("dmu_vision_bringup"), "config", "yolo_mono.yaml"
    )
    upstream_launch = os.path.join(
        get_package_share_directory("yolo_bringup"), "launch", "yolo.launch.py"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Monocular YOLO parameter file",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="yolo",
                description="Node namespace. Must match the params file block key (/<namespace>/yolo_node)",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(upstream_launch),
                launch_arguments={
                    "params_file": LaunchConfiguration("params_file"),
                    "namespace": LaunchConfiguration("namespace"),
                }.items(),
            ),
        ]
    )
