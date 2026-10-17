// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0

#include "hikrobot_camera_ros2/frame.hpp"

#include <array>
#include <stdexcept>
#include <string>
#include <utility>

namespace hikrobot_camera_ros2
{
namespace
{
const std::array<std::pair<const char *, std::size_t>, 6> kEncodings{{
  {"mono8", 1}, {"8UC1", 1}, {"bgr8", 3}, {"rgb8", 3}, {"mono16", 2}, {"16UC1", 2}}};
}  // namespace

std::size_t bytes_per_pixel(const std::string & encoding)
{
  for (const auto & entry : kEncodings) {
    if (encoding == entry.first) {return entry.second;}
  }
  return 0;
}

void validate_geometry(int width, int height, const std::string & encoding)
{
  if (width <= 0 || height <= 0) {
    throw std::invalid_argument(
            "frame geometry must be positive, got " + std::to_string(width) + "x" +
            std::to_string(height));
  }
  if (bytes_per_pixel(encoding) == 0) {
    throw std::invalid_argument("unsupported image encoding '" + encoding + "'");
  }
}

std::size_t Frame::step() const
{
  return static_cast<std::size_t>(width) * bytes_per_pixel(encoding);
}

std::size_t Frame::expected_size() const
{
  return step() * static_cast<std::size_t>(height);
}

bool Frame::is_well_formed() const
{
  if (width <= 0 || height <= 0) {return false;}
  if (bytes_per_pixel(encoding) == 0) {return false;}
  return data.size() == expected_size();
}

}  // namespace hikrobot_camera_ros2
