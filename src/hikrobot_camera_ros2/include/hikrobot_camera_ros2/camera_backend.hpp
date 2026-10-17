// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0
//
// Acquisition backend contract.

#ifndef HIKROBOT_CAMERA_ROS2__CAMERA_BACKEND_HPP_
#define HIKROBOT_CAMERA_ROS2__CAMERA_BACKEND_HPP_

#include <optional>
#include <stdexcept>
#include <string>

#include "hikrobot_camera_ros2/frame.hpp"

namespace hikrobot_camera_ros2
{

/// @brief Thrown when a backend cannot be used in this environment.
///
/// The message must stay actionable: name the missing artifact and where it was
/// searched. Returning placeholder frames instead of throwing is forbidden, so
/// that no consumer can mistake fabricated data for a real capture.
class BackendUnavailableError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

/// @brief What a backend can be asked to do.
class CameraBackend
{
public:
  virtual ~CameraBackend() = default;

  /// @brief Registered name, e.g. "mock" or "mvs".
  virtual std::string name() const = 0;

  /// @brief True when the backend needs real hardware or a vendor SDK.
  virtual bool requires_hardware() const { return false; }

  /// @brief Acquire device handles; throw BackendUnavailableError if impossible.
  virtual void open() = 0;

  /// @brief Release device handles. Must be idempotent.
  virtual void close() = 0;

  /// @brief Next frame, or std::nullopt when none arrived within the timeout.
  ///
  /// nullopt means "no data". It never means a placeholder frame.
  virtual std::optional<Frame> grab(double timeout_s) = 0;

  /// @brief One-line backend facts for logs and diagnostics.
  virtual std::string describe() const;
};

}  // namespace hikrobot_camera_ros2

#endif  // HIKROBOT_CAMERA_ROS2__CAMERA_BACKEND_HPP_
