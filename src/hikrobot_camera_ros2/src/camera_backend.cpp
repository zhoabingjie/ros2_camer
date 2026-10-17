// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0

#include "hikrobot_camera_ros2/camera_backend.hpp"

namespace hikrobot_camera_ros2
{

std::string CameraBackend::describe() const
{
  return "backend=" + name() + " requires_hardware=" + (requires_hardware() ? "true" : "false");
}

}  // namespace hikrobot_camera_ros2
