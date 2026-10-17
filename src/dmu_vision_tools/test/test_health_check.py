"""Tests for the ROS-facing adapter.

Only the pure adapter is exercised here; the live end-to-end check is run
manually and recorded in extras/docs/TEST_REPORT.md.
"""

import pytest

from dmu_vision_tools.health_check import _durability_of, _reliability_of, build_parser
from dmu_vision_tools.topic_probe import Durability, Reliability


class FakeEnum:
    def __init__(self, name):
        self.name = name

    def __str__(self):
        return name_to_str(self.name)


def name_to_str(value):
    return value


@pytest.mark.parametrize(
    "name,expected",
    [
        ("BEST_EFFORT", Reliability.BEST_EFFORT),
        ("RELIABLE", Reliability.RELIABLE),
        ("SYSTEM_DEFAULT", Reliability.SYSTEM_DEFAULT),
    ],
)
def test_reliability_adapter_accepts_rclpy_style_enums(name, expected):
    assert _reliability_of(FakeEnum(name)) is expected


@pytest.mark.parametrize(
    "name,expected",
    [
        ("TRANSIENT_LOCAL", Durability.TRANSIENT_LOCAL),
        ("VOLATILE", Durability.VOLATILE),
        ("SYSTEM_DEFAULT", Durability.SYSTEM_DEFAULT),
    ],
)
def test_durability_adapter_accepts_rclpy_style_enums(name, expected):
    assert _durability_of(FakeEnum(name)) is expected


def test_reliability_adapter_falls_back_to_string_form():
    assert _reliability_of("ReliabilityPolicy.BEST_EFFORT") is Reliability.BEST_EFFORT
    assert _reliability_of("ReliabilityPolicy.RELIABLE") is Reliability.RELIABLE


def test_parser_defaults_match_the_documented_interface():
    args = build_parser().parse_args([])
    assert args.image == "/camera/image_raw"
    assert args.camera_info == "/camera/camera_info"
    assert args.status == "/vision/status"
    assert args.measure == 5.0
    assert args.min_rate == 1.0


def test_parser_accepts_overrides():
    args = build_parser().parse_args(
        ["--image", "/other", "--measure", "2.5", "--min-rate", "8", "--max-age", "0.3"])
    assert args.image == "/other"
    assert args.measure == 2.5
    assert args.min_rate == 8.0
    assert args.max_age == 0.3
