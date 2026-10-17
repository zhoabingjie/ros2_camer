"""Vision subsystem health check.

Answers the question "is this pipeline actually working?" without guessing, and
-- most importantly -- surfaces QoS interoperability failures, whose symptom is a
healthy-looking topic graph that never delivers a single message.

Usage:
    ros2 run dmu_vision_tools vision_health_check \
        --image /camera/image_raw --camera-info /camera/camera_info \
        --status /vision/status --measure 5

Exit code 0 = no FAIL checks, 1 = at least one FAIL. Warnings do not fail the run.
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import List, Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from .topic_probe import (
    Durability,
    QosProfile,
    RateEstimator,
    Reliability,
    Report,
    StalenessTracker,
    Status,
    format_age,
    format_rate,
    qos_compatible,
)

# Probing with BEST_EFFORT is deliberate: it is compatible with both BEST_EFFORT
# and RELIABLE publishers, so a silent probe never masquerades as a QoS failure.
PROBE_QOS = qos_profile_sensor_data


def _reliability_of(value) -> Reliability:
    """Adapt an rclpy reliability policy, tolerating enum API differences."""
    name = getattr(value, "name", str(value)).lower()
    if "best" in name:
        return Reliability.BEST_EFFORT
    if "reliable" in name:
        return Reliability.RELIABLE
    return Reliability.SYSTEM_DEFAULT


def _durability_of(value) -> Durability:
    name = getattr(value, "name", str(value)).lower()
    if "transient" in name:
        return Durability.TRANSIENT_LOCAL
    if "volatile" in name:
        return Durability.VOLATILE
    return Durability.SYSTEM_DEFAULT


def endpoint_to_profile(info) -> QosProfile:
    """Convert an rclpy TopicEndpointInfo into the pure QosProfile type."""
    qos = info.qos_profile
    return QosProfile(
        reliability=_reliability_of(qos.reliability),
        durability=_durability_of(qos.durability),
        depth=int(getattr(qos, "depth", 0) or 0),
        node_name=f"{info.node_namespace.rstrip('/')}/{info.node_name}",
    )


class HealthCheck(Node):
    """Collects live evidence, then evaluates it into a Report."""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("vision_health_check")
        self._args = args
        self._image_rate = RateEstimator(window=30)
        self._info_rate = RateEstimator(window=30)
        self._image_stale = StalenessTracker()
        self._info_stale = StalenessTracker()
        self._image_frame_ids: List[str] = []
        self._info_frame_ids: List[str] = []
        self._calibrated: Optional[bool] = None
        self._stream_state: Optional[str] = None

        self.create_subscription(Image, args.image, self._on_image, PROBE_QOS)
        self.create_subscription(CameraInfo, args.camera_info, self._on_info, PROBE_QOS)
        if args.status:
            self.create_subscription(DiagnosticArray, args.status, self._on_status, 10)

    # -- callbacks ---------------------------------------------------------
    def _on_image(self, msg: Image) -> None:
        now = time.monotonic()
        self._image_rate.add(now)
        self._image_stale.add(now)
        self._image_frame_ids.append(msg.header.frame_id)

    def _on_info(self, msg: CameraInfo) -> None:
        now = time.monotonic()
        self._info_rate.add(now)
        self._info_stale.add(now)
        self._info_frame_ids.append(msg.header.frame_id)
        self._calibrated = any(abs(v) > 1e-9 for v in msg.k)

    def _on_status(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            for kv in status.values:
                if kv.key == "state":
                    self._stream_state = kv.value
            if self._stream_state is None:
                self._stream_state = status.message.split(":")[0].strip()

    # -- checks ------------------------------------------------------------
    def _check_topic(self, report: Report, label: str, topic: str, expected_type: str) -> None:
        names_and_types = dict(self.get_topic_names_and_types())
        types = names_and_types.get(topic)
        if types is None:
            report.add(f"topic {label}", Status.FAIL, f"{topic} does not exist")
            return
        if expected_type not in types:
            report.add(
                f"topic {label}", Status.FAIL,
                f"{topic} has type {types} but expected {expected_type}")
            return
        publishers = self.count_publishers(topic)
        if publishers == 0:
            report.add(f"topic {label}", Status.FAIL, f"{topic} exists but has no publisher")
            return
        report.add(f"topic {label}", Status.OK, f"{topic} ({expected_type}), {publishers} publisher(s)")

    def _check_qos_pairs(self, report: Report, label: str, topic: str) -> None:
        """Every publisher must satisfy every subscriber, or no data flows."""
        publishers = self.get_publishers_info_by_topic(topic)
        subscribers = self.get_subscriptions_info_by_topic(topic)
        if not publishers:
            return
        if not subscribers:
            report.add(f"qos {label}", Status.OK, "no subscribers yet; nothing to mismatch")
            return

        bad = []
        for pub_info in publishers:
            pub = endpoint_to_profile(pub_info)
            for sub_info in subscribers:
                sub = endpoint_to_profile(sub_info)
                verdict = qos_compatible(pub, sub)
                if not verdict.compatible:
                    bad.append(f"{pub.node_name} -> {sub.node_name}: {verdict.reason}")
        if bad:
            report.add(f"qos {label}", Status.FAIL, "; ".join(bad))
        else:
            report.add(
                f"qos {label}", Status.OK,
                f"{len(publishers)} publisher(s) x {len(subscribers)} subscriber(s) all compatible")

    def _check_rate(self, report: Report, label: str, rate: Optional[float], minimum: float) -> None:
        if rate is None:
            report.add(f"rate {label}", Status.FAIL, f"no messages received in the measurement window")
        elif rate < minimum:
            report.add(f"rate {label}", Status.FAIL,
                       f"{format_rate(rate)} is below the required {minimum:.2f} Hz")
        else:
            report.add(f"rate {label}", Status.OK, format_rate(rate))

    def _check_staleness(self, report: Report, label: str, age: Optional[float], maximum: float) -> None:
        if age is None:
            report.add(f"staleness {label}", Status.FAIL, "never received a message")
        elif age > maximum:
            report.add(f"staleness {label}", Status.WARN,
                       f"last message is {format_age(age)} old (> {maximum:.2f} s)")
        else:
            report.add(f"staleness {label}", Status.OK, f"last message {format_age(age)} old")

    def _check_frame_id_consistency(self, report: Report) -> None:
        image_ids = {f for f in self._image_frame_ids if f}
        info_ids = {f for f in self._info_frame_ids if f}
        if not image_ids or not info_ids:
            report.add("frame_id consistency", Status.WARN, "missing frame_id on Image and/or CameraInfo")
            return
        if image_ids != info_ids:
            report.add(
                "frame_id consistency", Status.FAIL,
                f"Image frame_id={sorted(image_ids)} but CameraInfo frame_id={sorted(info_ids)}"
                " -> tf2 lookups will resolve to the wrong frame")
            return
        if len(image_ids) > 1:
            report.add("frame_id consistency", Status.WARN,
                       f"Image frame_id changed mid-run: {sorted(image_ids)}")
            return
        report.add("frame_id consistency", Status.OK, f"both use '{next(iter(image_ids))}'")

    def _check_calibration(self, report: Report) -> None:
        if self._calibrated is None:
            report.add("calibration", Status.FAIL, "no CameraInfo received")
        elif self._calibrated:
            report.add("calibration", Status.OK, "CameraInfo carries non-zero intrinsics")
        else:
            report.add(
                "calibration", Status.WARN,
                "UNCALIBRATED: K is all zero. This is the honest marker, not a bug -- but "
                "image_proc will refuse to rectify and /camera/image_rect will be silent. "
                "See extras/docs/CAMERA_CALIBRATION.md.")

    def evaluate(self) -> Report:
        args = self._args
        report = Report()

        self._check_topic(report, "image", args.image, "sensor_msgs/msg/Image")
        self._check_topic(report, "camera_info", args.camera_info, "sensor_msgs/msg/CameraInfo")
        self._check_qos_pairs(report, "image", args.image)
        self._check_qos_pairs(report, "camera_info", args.camera_info)
        self._check_rate(report, "image", self._image_rate.rate_hz(), args.min_rate)
        self._check_rate(report, "camera_info", self._info_rate.rate_hz(), args.min_rate)
        self._check_staleness(report, "image", self._image_stale.age_s(time.monotonic()), args.max_age)
        self._check_frame_id_consistency(report)
        self._check_calibration(report)
        if args.status:
            if self._stream_state is None:
                report.add("stream status", Status.WARN, f"no diagnostics received on {args.status}")
            else:
                report.add("stream status", Status.OK, f"reported state '{self._stream_state}'")
        return report


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="vision_health_check",
        description="Check that the vision image pipeline is actually delivering data.")
    parser.add_argument("--image", default="/camera/image_raw", help="image topic (default %(default)s)")
    parser.add_argument("--camera-info", default="/camera/camera_info", help="camera_info topic (default %(default)s)")
    parser.add_argument("--status", default="/vision/status", help="diagnostics topic (default %(default)s)")
    parser.add_argument("--measure", type=float, default=5.0, help="measurement window in seconds (default %(default)s)")
    parser.add_argument("--min-rate", type=float, default=1.0, help="minimum acceptable image rate in Hz (default %(default)s)")
    parser.add_argument("--max-age", type=float, default=1.0, help="maximum acceptable staleness in seconds (default %(default)s)")
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    rclpy.init()
    node = HealthCheck(args)
    try:
        deadline = time.monotonic() + max(0.1, args.measure)
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
        report = node.evaluate()
        print(report.render(), flush=True)
        return report.exit_code
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
