// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0
//
// Stream health and dropout detection.
//
// Contract (plan T09, "断流恢复"): when the stream stalls or the backend errors,
// the node reports a diagnostic and must NOT keep publishing stale or fabricated
// data. This monitor only observes; the node acts on the snapshot.

#ifndef HIKROBOT_CAMERA_ROS2__STREAM_HEALTH_HPP_
#define HIKROBOT_CAMERA_ROS2__STREAM_HEALTH_HPP_

#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <string>

namespace hikrobot_camera_ros2
{

enum class StreamState
{
  kNoFrames,
  kOk,
  kStalled,
  kBackendError,
};

std::string to_string(StreamState state);

struct HealthSnapshot
{
  StreamState state{StreamState::kNoFrames};
  uint64_t frames_total{0};
  uint64_t consecutive_misses{0};
  uint64_t consecutive_errors{0};
  std::optional<double> last_frame_age_s{};
  std::optional<double> measured_fps{};
  std::optional<std::string> last_error{};
  uint64_t backend_reopens{0};
};

/// @brief Deterministic, clock-injected stream watchdog. Thread-safe.
class StreamHealthMonitor
{
public:
  using Clock = std::function<double()>;

  explicit StreamHealthMonitor(
    double stall_timeout_s = 2.0, std::size_t fps_window = 20, Clock clock = Clock{});

  void on_frame(std::optional<double> now = std::nullopt);
  void on_missed();
  void on_error(const std::string & message);
  void on_reopen();

  StreamState state(std::optional<double> now = std::nullopt) const;
  HealthSnapshot snapshot(std::optional<double> now = std::nullopt) const;
  std::optional<double> measured_fps() const;

  /// @brief Whether publishing is allowed right now, plus the reason state.
  bool should_publish(std::optional<double> now, StreamState * state_out = nullptr) const;

  double stall_timeout_s() const {return stall_timeout_s_;}

private:
  double now_or(std::optional<double> now) const;

  double stall_timeout_s_;
  std::size_t fps_window_;
  Clock clock_;
  mutable std::mutex mutex_;
  std::deque<double> frame_times_;
  uint64_t frames_total_{0};
  uint64_t consecutive_misses_{0};
  uint64_t consecutive_errors_{0};
  uint64_t backend_reopens_{0};
  std::optional<double> last_frame_time_{};
  std::optional<std::string> last_error_{};
};

}  // namespace hikrobot_camera_ros2

#endif  // HIKROBOT_CAMERA_ROS2__STREAM_HEALTH_HPP_
