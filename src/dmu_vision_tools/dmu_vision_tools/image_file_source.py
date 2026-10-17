"""Publish a static image file as sensor_msgs/Image.

Why this exists
---------------
The detection pipeline must be verifiable **offline**, on an image whose content
is known. The project plan (section 7.4) explicitly allows "static image or
rosbag replay" as an acceptance method. This node provides exactly that, and it
is the only way to prove the detector actually detects anything: the `mock`
camera backend renders synthetic patterns that contain no COCO objects, so a
YOLO model correctly returns zero detections on it -- which proves nothing.

This is a TEST SOURCE, not a camera driver. It never pretends otherwise: the
default topic lives under /image_source and the startup log says so.

Usage
-----
    ros2 run dmu_vision_tools image_file_source --ros-args \
        -p filename:=/tmp/bus.jpg -p rate_hz:=10.0 \
        -p topic:=/camera/image_raw -p frame_id:=camera_optical_frame
"""

from __future__ import annotations

import sys
from typing import Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image

#: channel count -> ROS image encoding
ENCODING_BY_CHANNELS = {1: "mono8", 3: "bgr8", 4: "bgra8"}

#: ROS image encoding -> bytes per pixel
BYTES_PER_PIXEL = {"mono8": 1, "bgr8": 3, "bgra8": 4}


def load_image(path: str) -> Tuple[np.ndarray, str]:
    """Load an image file into (tightly packed array, ROS encoding).

    Raises ValueError with an actionable message rather than publishing garbage:
    a wrong encoding silently corrupts every downstream consumer.
    """
    data = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if data is None:
        raise ValueError(f"cannot read image file (missing or unsupported format): {path}")
    if data.dtype != np.uint8:
        raise ValueError(
            f"{path}: only 8-bit images are supported, got dtype {data.dtype}. "
            "Convert the file first (16-bit mono would need encoding 'mono16' and "
            "different byte handling).")
    channels = 1 if data.ndim == 2 else data.shape[2]
    encoding = ENCODING_BY_CHANNELS.get(channels)
    if encoding is None:
        raise ValueError(f"{path}: unsupported channel count {channels}")
    return np.ascontiguousarray(data), encoding


def build_qos(reliability: str, depth: int) -> QoSProfile:
    policy = (
        ReliabilityPolicy.BEST_EFFORT
        if reliability.strip().lower() in ("best_effort", "besteffort", "2")
        else ReliabilityPolicy.RELIABLE)
    return QoSProfile(
        depth=max(1, int(depth)),
        history=HistoryPolicy.KEEP_LAST,
        reliability=policy,
        durability=DurabilityPolicy.VOLATILE,
    )


class ImageFileSource(Node):
    """Republishes one image file at a fixed rate."""

    def __init__(self) -> None:
        super().__init__("image_file_source")
        self.declare_parameter("filename", "")
        self.declare_parameter("topic", "/image_source/image_raw")
        self.declare_parameter("frame_id", "camera_optical_frame")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("qos_reliability", "best_effort")
        self.declare_parameter("qos_depth", 5)

        filename = self.get_parameter("filename").value
        topic = self.get_parameter("topic").value
        self._frame_id = self.get_parameter("frame_id").value
        rate = float(self.get_parameter("rate_hz").value)

        try:
            self._data, self._encoding = load_image(filename)
        except ValueError as exc:
            # Fail loudly at startup: a silent no-image source would look like a
            # detection problem much further down the pipeline.
            self.get_logger().error(f"image_file_source cannot start: {exc}")
            raise

        self._step = int(self._data.shape[1]) * BYTES_PER_PIXEL[self._encoding]
        qos = build_qos(self.get_parameter("qos_reliability").value,
                        int(self.get_parameter("qos_depth").value))
        self._publisher = self.create_publisher(Image, topic, qos)
        self.get_logger().warn(
            f"TEST SOURCE: replaying the static file '{filename}' as "
            f"{self._data.shape[1]}x{self._data.shape[0]} {self._encoding} on {topic} "
            f"at {rate:.1f} Hz. This is NOT a camera.")
        self._timer = self.create_timer(1.0 / max(rate, 0.1), self._publish)

    def _publish(self) -> None:
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.height = int(self._data.shape[0])
        msg.width = int(self._data.shape[1])
        msg.encoding = self._encoding
        msg.is_bigendian = False
        msg.step = self._step
        msg.data = self._data.tobytes()
        self._publisher.publish(msg)


def main(argv=None) -> int:
    rclpy.init()
    try:
        node = ImageFileSource()
    except ValueError:
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
