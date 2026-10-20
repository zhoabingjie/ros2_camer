"""Independent image viewer. Closing it never stops the camera pipeline."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic = LaunchConfiguration("image_topic")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/image_raw",
                description="Image topic to display; use /vision/debug_image for YOLO boxes",
            ),
            Node(
                package="rqt_image_view",
                executable="rqt_image_view",
                name="vision_image_viewer",
                output="screen",
                remappings=[("image", image_topic)],
            ),
        ]
    )
