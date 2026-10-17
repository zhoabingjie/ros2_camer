// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0
//
// Deterministic synthetic backend: no hardware, no vendor SDK.

#ifndef HIKROBOT_CAMERA_ROS2__MOCK_BACKEND_HPP_
#define HIKROBOT_CAMERA_ROS2__MOCK_BACKEND_HPP_

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "hikrobot_camera_ros2/camera_backend.hpp"

namespace hikrobot_camera_ros2
{

/// @brief Construction parameters for MockBackend.
///
/// Declared at namespace scope instead of nested inside MockBackend: the default
/// member initializers of a nested type are not available to a default argument
/// of a member function of the enclosing class (ISO C++ [class.mem]).
struct MockBackendOptions
{
  int width{640};
  int height{480};
  std::string encoding{"mono8"};
  double fps{10.0};
  std::string pattern{"moving_bar"};
  std::optional<int64_t> fail_after_frames{};
  double fail_duration_s{0.0};
  std::optional<int64_t> raise_after_frames{};

  /// Optional still image to replay instead of the synthetic pattern.
  ///
  /// This makes the mock able to show a REAL SCENE (e.g. a photo with objects the
  /// detector can find), which the synthetic gradient/bar patterns can never do.
  /// It is still a replay source, not a camera: the node keeps labelling itself
  /// mock_source=true. Empty means "render the synthetic pattern".
  std::string image_path;
};

class MockBackend : public CameraBackend
{
public:
  /// Injectable clock so dropout timing is deterministic in tests.
  using Clock = std::function<double()>;
  using TimeNs = std::function<int64_t()>;

  static const std::vector<std::string> & patterns();

  explicit MockBackend(
    MockBackendOptions options = MockBackendOptions{}, Clock clock = Clock{},
    TimeNs time_ns = TimeNs{});

  std::string name() const override {return "mock";}
  bool requires_hardware() const override {return false;}
  void open() override;
  void close() override;
  std::optional<Frame> grab(double timeout_s) override;
  std::string describe() const override;

private:
  std::vector<uint8_t> render_background() const;
  std::vector<uint8_t> render(uint64_t sequence) const;

  MockBackendOptions options_;
  Clock clock_;
  TimeNs time_ns_;
  std::vector<uint8_t> background_;
  std::vector<uint8_t> scene_bytes_;  ///< precomputed replay bytes when image_path is set
  uint64_t sequence_{0};
  bool opened_{false};
  std::optional<double> fail_until_{};
};

}  // namespace hikrobot_camera_ros2

#endif  // HIKROBOT_CAMERA_ROS2__MOCK_BACKEND_HPP_
