"""Tests for the measurement maths and its argument surface."""

import pytest

from dmu_vision_tools.measure_detection import build_parser, percentile, rate_hz


def test_rate_of_ten_hz():
    assert rate_hz([0.0, 0.1, 0.2, 0.3]) == pytest.approx(10.0)


def test_rate_is_zero_when_undefined():
    assert rate_hz([]) == 0.0
    assert rate_hz([1.0]) == 0.0
    assert rate_hz([1.0, 1.0]) == 0.0


def test_percentile_nearest_rank():
    values = [float(v) for v in range(1, 101)]  # 1..100
    assert percentile(values, 0.5) == 50.0
    assert percentile(values, 0.95) == 95.0
    assert percentile(values, 1.0) == 100.0


def test_percentile_handles_single_and_small_samples():
    assert percentile([7.0], 0.95) == 7.0
    assert percentile([3.0, 1.0, 2.0], 0.0) == 1.0


def test_percentile_does_not_mutate_input():
    values = [3.0, 1.0, 2.0]
    percentile(values, 0.5)
    assert values == [3.0, 1.0, 2.0]


def test_parser_defaults_match_the_verified_topic_names():
    args = build_parser().parse_args([])
    assert args.image == "/camera/image_raw"
    assert args.detections == "/yolo/cam0/detections"
    assert args.debug == "/yolo/cam0/debug_image"
    assert args.seconds == 30.0
