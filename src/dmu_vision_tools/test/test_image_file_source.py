"""Tests for the static image test source."""

import cv2
import numpy as np
import pytest

from dmu_vision_tools.image_file_source import (
    BYTES_PER_PIXEL,
    ENCODING_BY_CHANNELS,
    build_qos,
    load_image,
)
from rclpy.qos import ReliabilityPolicy


def _write(tmp_path, name, array):
    path = tmp_path / name
    assert cv2.imwrite(str(path), array), f"could not write {path}"
    return str(path)


def test_loads_a_bgr_jpeg(tmp_path):
    path = _write(tmp_path, "img.jpg", np.full((40, 30, 3), 128, np.uint8))
    data, encoding = load_image(path)
    assert encoding == "bgr8"
    assert data.shape == (40, 30, 3)
    assert data.dtype == np.uint8
    assert data.flags["C_CONTIGUOUS"]


def test_loads_a_grayscale_png(tmp_path):
    path = _write(tmp_path, "gray.png", np.full((16, 8), 200, np.uint8))
    data, encoding = load_image(path)
    assert encoding == "mono8"
    assert data.ndim == 2


def test_loads_a_bgra_png(tmp_path):
    path = _write(tmp_path, "rgba.png", np.full((8, 8, 4), 10, np.uint8))
    data, encoding = load_image(path)
    assert encoding == "bgra8"


def test_missing_file_raises_with_the_path_in_the_message(tmp_path):
    missing = str(tmp_path / "nope.jpg")
    with pytest.raises(ValueError) as excinfo:
        load_image(missing)
    assert missing in str(excinfo.value)
    assert "cannot read image file" in str(excinfo.value)


def test_16bit_image_is_rejected_rather_than_mis_encoded(tmp_path):
    path = _write(tmp_path, "deep.png", np.full((8, 8), 1000, np.uint16))
    with pytest.raises(ValueError) as excinfo:
        load_image(path)
    assert "8-bit" in str(excinfo.value)


def test_every_encoding_has_a_known_pixel_size():
    for encoding in ENCODING_BY_CHANNELS.values():
        assert encoding in BYTES_PER_PIXEL


@pytest.mark.parametrize(
    "value,expected",
    [
        ("best_effort", ReliabilityPolicy.BEST_EFFORT),
        ("BEST_EFFORT", ReliabilityPolicy.BEST_EFFORT),
        ("2", ReliabilityPolicy.BEST_EFFORT),
        ("reliable", ReliabilityPolicy.RELIABLE),
        ("1", ReliabilityPolicy.RELIABLE),
    ],
)
def test_build_qos(value, expected):
    assert build_qos(value, 5).reliability == expected


def test_build_qos_depth_is_at_least_one():
    assert build_qos("best_effort", 0).depth == 1
