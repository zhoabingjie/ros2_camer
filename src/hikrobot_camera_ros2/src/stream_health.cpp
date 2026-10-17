// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0

#include "hikrobot_camera_ros2/stream_health.hpp"

#include <chrono>
#include <stdexcept>

namespace hikrobot_camera_ros2
{
namespace
{
double steady_seconds()
{
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}
}  // namespace

std::string to_string(StreamState state)
{
  switch (state) {
    case StreamState::kNoFrames: return "no_frames";
    case StreamState::kOk: return "ok";
    case StreamState::kStalled: return "stalled";
    case StreamState::kBackendError: return "backend_error";
  }
  return "unknown";
}

StreamHealthMonitor::StreamHealthMonitor(double stall_timeout_s, std::size_t fps_window, Clock clock)
: stall_timeout_s_(stall_timeout_s), fps_window_(fps_window), clock_(std::move(clock))
{
  if (!(stall_timeout_s_ > 0.0)) {throw std::invalid_argument("stall_timeout_s must be > 0");}
  if (fps_window_ < 2) {throw std::invalid_argument("fps_window must be >= 2");}
  if (!clock_) {clock_ = steady_seconds;}
}

double StreamHealthMonitor::now_or(std::optional<double> now) const
{
  return now.has_value() ? *now : clock_();
}

void StreamHealthMonitor::on_frame(std::optional<double> now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const double stamp = now_or(now);
  frame_times_.push_back(stamp);
  while (frame_times_.size() > fps_window_) {frame_times_.pop_front();}
  last_frame_time_ = stamp;
  ++frames_total_;
  consecutive_misses_ = 0;
  consecutive_errors_ = 0;
  last_error_.reset();
}

void StreamHealthMonitor::on_missed()
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++consecutive_misses_;
}

void StreamHealthMonitor::on_error(const std::string & message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++consecutive_errors_;
  last_error_ = message;
}

void StreamHealthMonitor::on_reopen()
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++backend_reopens_;
}

StreamState StreamHealthMonitor::state(std::optional<double> now) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (consecutive_errors_ > 0) {return StreamState::kBackendError;}
  if (!last_frame_time_.has_value()) {return StreamState::kNoFrames;}
  const double age = now_or(now) - *last_frame_time_;
  return age <= stall_timeout_s_ ? StreamState::kOk : StreamState::kStalled;
}

std::optional<double> StreamHealthMonitor::measured_fps() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (frame_times_.size() < 2) {return std::nullopt;}
  const double span = frame_times_.back() - frame_times_.front();
  if (!(span > 0.0)) {return std::nullopt;}
  return static_cast<double>(frame_times_.size() - 1) / span;
}

HealthSnapshot StreamHealthMonitor::snapshot(std::optional<double> now) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const double stamp = now_or(now);
  HealthSnapshot snap;
  snap.state = consecutive_errors_ > 0 ?
    StreamState::kBackendError :
    (!last_frame_time_.has_value() ? StreamState::kNoFrames :
    ((stamp - *last_frame_time_) <= stall_timeout_s_ ? StreamState::kOk : StreamState::kStalled));
  snap.frames_total = frames_total_;
  snap.consecutive_misses = consecutive_misses_;
  snap.consecutive_errors = consecutive_errors_;
  if (last_frame_time_.has_value()) {snap.last_frame_age_s = stamp - *last_frame_time_;}
  if (frame_times_.size() >= 2) {
    const double span = frame_times_.back() - frame_times_.front();
    if (span > 0.0) {
      snap.measured_fps = static_cast<double>(frame_times_.size() - 1) / span;
    }
  }
  snap.last_error = last_error_;
  snap.backend_reopens = backend_reopens_;
  return snap;
}

bool StreamHealthMonitor::should_publish(std::optional<double> now, StreamState * state_out) const
{
  const StreamState current = state(now);
  if (state_out != nullptr) {*state_out = current;}
  return current == StreamState::kOk || current == StreamState::kNoFrames;
}

}  // namespace hikrobot_camera_ros2
