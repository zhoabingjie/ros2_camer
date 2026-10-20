"""Measure detection throughput, end-to-end latency and detected content.

Makes the plan's section 11.3 performance report reproducible instead of a
one-off measurement. Run it while a detector and an image source are up.

    ros2 run dmu_vision_tools measure_detection --seconds 60

What it reports
---------------
* rate of detections and of the debug image, plus the rate of the source topic
* latency from the image header stamp to the detection message arriving
  (valid because DetectionPlugin sets `output.header = frame.header`,
  detection_plugin.cpp:250)
* mean and P95 latency, and every detected class with counts and score stats

Honesty notes
-------------
* The *image source rate* you see is what this process actually observed; a
  BEST_EFFORT subscription may drop frames when the machine is saturated, so it
  is a lower bound on what the source published.
* On a CPU-only machine the numbers are bounded by the total CPU budget shared
  by the source, the detector and every subscriber. They characterise the
  machine, not the algorithm. Do not quote them as "the pipeline's FPS".
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image
from yolo_msgs.msg import DetectionArray

RELIABLE_QOS = QoSProfile(
    depth=10,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


def rate_hz(times: List[float]) -> float:
    """Arrival rate over a list of monotonic timestamps (0.0 if undefined)."""
    if len(times) < 2:
        return 0.0
    span = times[-1] - times[0]
    return 0.0 if span <= 0 else (len(times) - 1) / span


def percentile(values: List[float], fraction: float) -> float:
    """Nearest-rank percentile: index = ceil(fraction * n) - 1.

    ``round()`` would be wrong here. For 100 samples and fraction 0.5 it rounds
    49.5 up to 50 and reports the 51st value; the nearest-rank definition wants
    the 50th. That is a systematic bias, and this number ends up in the
    performance report, so the exact definition matters.
    """
    if not values:
        raise ValueError("percentile() needs at least one value")
    ordered = sorted(values)
    rank = math.ceil(fraction * len(ordered)) - 1
    index = max(0, min(len(ordered) - 1, rank))
    return ordered[index]


class DetectionMeasurer(Node):
    def __init__(self, image_topic: str, detection_topic: str, debug_topic: str) -> None:
        super().__init__("measure_detection")
        self._image_times: List[float] = []
        self._detection_times: List[float] = []
        self._debug_times: List[float] = []
        self._latency_ms: List[float] = []
        self._class_scores: Dict[str, List[float]] = {}
        self.debug_description: Optional[str] = None

        self.create_subscription(Image, image_topic, self._on_image, qos_profile_sensor_data)
        self.create_subscription(
            DetectionArray, detection_topic, self._on_detection, qos_profile_sensor_data)
        # The debug image is published RELIABLE; subscribe the same way so a
        # saturated machine does not silently drop the very topic under test.
        self.create_subscription(Image, debug_topic, self._on_debug, RELIABLE_QOS)

    def _on_image(self, _msg: Image) -> None:
        self._image_times.append(time.monotonic())

    def _on_debug(self, msg: Image) -> None:
        self._debug_times.append(time.monotonic())
        if self.debug_description is None:
            self.debug_description = f"{msg.width}x{msg.height} {msg.encoding}"

    def _on_detection(self, msg: DetectionArray) -> None:
        self._detection_times.append(time.monotonic())
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if stamp_ns > 0:
            now_ns = self.get_clock().now().nanoseconds
            self._latency_ms.append((now_ns - stamp_ns) / 1e6)
        for detection in msg.detections:
            self._class_scores.setdefault(detection.class_name, []).append(float(detection.score))

    def render(self) -> str:
        lines = [
            f"  image source rate    : {rate_hz(self._image_times):6.2f} Hz  (n={len(self._image_times)})",
            f"  detection rate       : {rate_hz(self._detection_times):6.2f} Hz  (n={len(self._detection_times)})",
            f"  debug_image rate     : {rate_hz(self._debug_times):6.2f} Hz  (n={len(self._debug_times)})",
        ]
        if self.debug_description:
            lines.append(f"  debug_image          : {self.debug_description}")
        if self._latency_ms:
            lines.append(
                f"  latency stamp->recv  : mean {statistics.mean(self._latency_ms):6.1f} ms   "
                f"P95 {percentile(self._latency_ms, 0.95):6.1f} ms   n={len(self._latency_ms)}")
        else:
            lines.append("  latency stamp->recv  : n/a (no detection carried a header stamp)")
        if self._class_scores:
            lines.append("  detected classes:")
            for name, scores in sorted(self._class_scores.items(), key=lambda kv: -len(kv[1])):
                lines.append(
                    f"    {name:<14} n={len(scores):<5} mean={statistics.mean(scores):.3f} "
                    f"max={max(scores):.3f}")
        else:
            lines.append("  detected classes     : none")
        return "\n".join(lines)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="measure_detection",
        description="Measure detection rate, latency and content.")
    parser.add_argument("--image", default="/camera/image_raw", help="source image topic")
    parser.add_argument("--detections", default="/vision/detections", help="detection topic")
    parser.add_argument("--debug", default="/vision/debug_image", help="debug image topic")
    parser.add_argument("--seconds", type=float, default=30.0, help="measurement window")
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    rclpy.init()
    node = DetectionMeasurer(args.image, args.detections, args.debug)
    try:
        end = time.time() + max(0.5, args.seconds)
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
        print(node.render(), flush=True)
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
