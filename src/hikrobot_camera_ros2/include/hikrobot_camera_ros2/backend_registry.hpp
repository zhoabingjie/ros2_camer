// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0

#ifndef HIKROBOT_CAMERA_ROS2__BACKEND_REGISTRY_HPP_
#define HIKROBOT_CAMERA_ROS2__BACKEND_REGISTRY_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hikrobot_camera_ros2/camera_backend.hpp"
#include "hikrobot_camera_ros2/mock_backend.hpp"
#include "hikrobot_camera_ros2/mvs_backend.hpp"

namespace hikrobot_camera_ros2
{

/// @brief Everything a caller may need to construct any backend.
///
/// Kept as one struct so the node has a single construction path, and so a
/// backend receives exactly its own keys rather than a permissive superset
/// (which would hide typos behind a runtime error).
struct BackendRequest
{
  std::string name;
  MockBackendOptions mock;
  MvsBackendOptions mvs;
};

/// @brief Registered backend names, sorted.
std::vector<std::string> available_backends();

/// @brief Instantiate a backend; throws BackendUnavailableError for unknown names.
std::unique_ptr<CameraBackend> create_backend(const BackendRequest & request);

}  // namespace hikrobot_camera_ros2

#endif  // HIKROBOT_CAMERA_ROS2__BACKEND_REGISTRY_HPP_
