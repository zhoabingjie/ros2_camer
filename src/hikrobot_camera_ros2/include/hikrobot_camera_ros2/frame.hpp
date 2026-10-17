// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0
//
// Backend-neutral frame representation.

#ifndef HIKROBOT_CAMERA_ROS2__FRAME_HPP_
#define HIKROBOT_CAMERA_ROS2__FRAME_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace hikrobot_camera_ros2
{

/// @brief Bytes per pixel for a ROS image encoding; 0 when the encoding is unknown.
std::size_t bytes_per_pixel(const std::string & encoding);

/// @brief Throw std::invalid_argument unless the geometry is possible.
///
/// Deliberately separate from Frame::is_well_formed(), which also checks the
/// payload length and therefore cannot validate geometry before data exists.
void validate_geometry(int width, int height, const std::string & encoding);

/// @brief One acquired frame, tightly packed (step == width * bytes_per_pixel).
///
/// `hardware_stamp_ns` is set only when the backend can actually read a device
/// timestamp. It stays empty rather than being filled in from the host clock,
/// so a consumer can never mistake a host stamp for a device stamp.
struct Frame
{
  int width{0};
  int height{0};
  std::string encoding;
  std::vector<uint8_t> data;
  int64_t stamp_ns{0};
  uint64_t sequence{0};
  std::optional<int64_t> hardware_stamp_ns{};

  std::size_t step() const;
  std::size_t expected_size() const;
  bool is_well_formed() const;
};

}  // namespace hikrobot_camera_ros2

#endif  // HIKROBOT_CAMERA_ROS2__FRAME_HPP_
