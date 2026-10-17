// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0

#include "hikrobot_camera_ros2/backend_registry.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>

#include "hikrobot_camera_ros2/mvs_backend.hpp"

namespace hikrobot_camera_ros2
{

std::vector<std::string> available_backends()
{
  return {"mock", "mvs"};
}

std::unique_ptr<CameraBackend> create_backend(const BackendRequest & request)
{
  std::string key;
  key.reserve(request.name.size());
  for (const char c : request.name) {
    key.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  }
  while (!key.empty() && std::isspace(static_cast<unsigned char>(key.front()))) {key.erase(key.begin());}
  while (!key.empty() && std::isspace(static_cast<unsigned char>(key.back()))) {key.pop_back();}

  if (key == "mock") {
    // Only the mock's own keys are forwarded: passing a permissive superset
    // would turn a typo into a confusing runtime failure.
    return std::make_unique<MockBackend>(request.mock);
  }
  if (key == "mvs") {
    return std::make_unique<MvsBackend>(request.sdk_lib_candidates);
  }

  const auto names = available_backends();
  std::ostringstream os;
  os << "unknown camera backend '" << request.name << "'; available: ";
  for (std::size_t i = 0; i < names.size(); ++i) {
    os << names[i] << (i + 1 < names.size() ? ", " : "");
  }
  throw BackendUnavailableError(os.str());
}

}  // namespace hikrobot_camera_ros2
