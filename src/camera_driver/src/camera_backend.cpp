// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0

#include "camera_driver/camera_backend.hpp"

namespace camera_driver
{

std::string CameraBackend::describe() const
{
  return "backend=" + name() + " requires_hardware=" + (requires_hardware() ? "true" : "false");
}

}  // namespace camera_driver
