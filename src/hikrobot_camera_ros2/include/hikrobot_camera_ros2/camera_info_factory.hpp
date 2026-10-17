// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0
//
// CameraInfo construction that refuses to fabricate intrinsics.
//
// Project rules implemented here:
//  * Before calibration the node may publish CameraInfo, but it must be honestly
//    marked uncalibrated ("不得填造焦距/主点"). Zeros are the ROS convention for
//    "unknown", and image_proc treats an all-zero K as uncalibrated instead of
//    rectifying with invented numbers.
//  * A calibration whose resolution does not match the live stream is rejected,
//    not silently rescaled ("若更改分辨率、ROI、binning 或镜头，必须重新验证标定参数").

#ifndef HIKROBOT_CAMERA_ROS2__CAMERA_INFO_FACTORY_HPP_
#define HIKROBOT_CAMERA_ROS2__CAMERA_INFO_FACTORY_HPP_

#include <array>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace hikrobot_camera_ros2
{

class CalibrationError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

class CalibrationMismatchError : public CalibrationError
{
public:
  // CalibrationError is the direct base, so its constructors are the ones to
  // inherit (std::runtime_error is only an indirect base).
  using CalibrationError::CalibrationError;
};

/// @brief A loaded, validated camera calibration.
struct Calibration
{
  int image_width{0};
  int image_height{0};
  std::array<double, 9> camera_matrix{};
  std::string distortion_model;
  std::vector<double> distortion_coefficients;
  std::array<double, 9> rectification_matrix{};
  std::array<double, 12> projection_matrix{};
  std::string camera_name;
  std::optional<double> reprojection_error{};
  std::string source_path;

  /// True when K carries no usable focal length (fx or fy is zero).
  bool is_degenerate() const;

  /// Throw CalibrationMismatchError unless this calibration matches the stream.
  void check_against(int width, int height) const;
};

/// @brief Load a ROS camera_calibration YAML file.
/// @throws CalibrationError on missing, malformed or degenerate content.
Calibration load_calibration(const std::string & path);

/// @brief Build a CameraInfo. Uncalibrated output is all-zero and clearly marked.
///
/// passing nullptr for @p calibration yields the ROS "uncalibrated" marker
/// (all-zero K and P). No value is ever invented; a real calibration is used
/// only after Calibration::check_against() has passed.
sensor_msgs::msg::CameraInfo build_camera_info(
  int width, int height, const std::string & frame_id, const builtin_interfaces::msg::Time & stamp,
  const Calibration * calibration);

/// @brief True when a CameraInfo carries the all-zero uncalibrated marker.
bool is_uncalibrated(const sensor_msgs::msg::CameraInfo & info);

}  // namespace hikrobot_camera_ros2

#endif  // HIKROBOT_CAMERA_ROS2__CAMERA_INFO_FACTORY_HPP_
