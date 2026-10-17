// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <stdexcept>

#include "hikrobot_camera_ros2/stream_health.hpp"

using hikrobot_camera_ros2::HealthSnapshot;
using hikrobot_camera_ros2::StreamHealthMonitor;
using hikrobot_camera_ros2::StreamState;

namespace
{
class FakeClock
{
public:
  double operator()() const {return now;}
  void advance(double seconds) {now += seconds;}
  double now{500.0};
};
}  // namespace

TEST(StreamHealth, StartsInNoFramesAndAllowsPublishing)
{
  FakeClock clock;
  StreamHealthMonitor monitor(1.0, 20, [&clock]() {return clock();});
  EXPECT_EQ(monitor.state(), StreamState::kNoFrames);
  StreamState state = StreamState::kBackendError;
  EXPECT_TRUE(monitor.should_publish(std::nullopt, &state));
  EXPECT_EQ(state, StreamState::kNoFrames);
}

TEST(StreamHealth, FreshFramesAreOk)
{
  FakeClock clock;
  StreamHealthMonitor monitor(1.0, 20, [&clock]() {return clock();});
  monitor.on_frame();
  EXPECT_EQ(monitor.state(), StreamState::kOk);
  EXPECT_EQ(monitor.snapshot().frames_total, 1u);
}

TEST(StreamHealth, SilenceBeyondTimeoutIsStalledAndBlocksPublishing)
{
  FakeClock clock;
  StreamHealthMonitor monitor(1.0, 20, [&clock]() {return clock();});
  monitor.on_frame();
  clock.advance(1.5);
  EXPECT_EQ(monitor.state(), StreamState::kStalled);
  StreamState state = StreamState::kOk;
  EXPECT_FALSE(monitor.should_publish(std::nullopt, &state))
    << "a stalled stream must not keep publishing";
  EXPECT_EQ(state, StreamState::kStalled);
}

TEST(StreamHealth, RecoversAfterAStall)
{
  FakeClock clock;
  StreamHealthMonitor monitor(1.0, 20, [&clock]() {return clock();});
  monitor.on_frame();
  clock.advance(5.0);
  EXPECT_EQ(monitor.state(), StreamState::kStalled);
  monitor.on_frame();
  EXPECT_EQ(monitor.state(), StreamState::kOk);
}

TEST(StreamHealth, BackendErrorDominatesAndCarriesTheMessage)
{
  FakeClock clock;
  StreamHealthMonitor monitor(1.0, 20, [&clock]() {return clock();});
  monitor.on_frame();
  monitor.on_error("injected backend failure");
  const auto snapshot = monitor.snapshot();
  EXPECT_EQ(snapshot.state, StreamState::kBackendError);
  ASSERT_TRUE(snapshot.last_error.has_value());
  EXPECT_EQ(*snapshot.last_error, "injected backend failure");
  EXPECT_EQ(snapshot.consecutive_errors, 1u);

  monitor.on_frame();
  EXPECT_EQ(monitor.state(), StreamState::kOk);
  EXPECT_FALSE(monitor.snapshot().last_error.has_value());
}

TEST(StreamHealth, MissesAreCountedAndResetByAFrame)
{
  FakeClock clock;
  StreamHealthMonitor monitor(1.0, 20, [&clock]() {return clock();});
  monitor.on_frame();
  monitor.on_missed();
  monitor.on_missed();
  EXPECT_EQ(monitor.snapshot().consecutive_misses, 2u);
  monitor.on_frame();
  EXPECT_EQ(monitor.snapshot().consecutive_misses, 0u);
}

TEST(StreamHealth, MeasuredFpsOverTheWindow)
{
  FakeClock clock;
  StreamHealthMonitor monitor(10.0, 5, [&clock]() {return clock();});
  EXPECT_FALSE(monitor.measured_fps().has_value()) << "needs at least two frames";
  for (int i = 0; i < 5; ++i) {
    monitor.on_frame();
    clock.advance(0.1);
  }
  ASSERT_TRUE(monitor.measured_fps().has_value());
  EXPECT_NEAR(*monitor.measured_fps(), 10.0, 1e-6);
}

TEST(StreamHealth, ReopenCounterIsReported)
{
  FakeClock clock;
  StreamHealthMonitor monitor(1.0, 20, [&clock]() {return clock();});
  monitor.on_reopen();
  monitor.on_reopen();
  EXPECT_EQ(monitor.snapshot().backend_reopens, 2u);
}

TEST(StreamHealth, InvalidConstructionIsRejected)
{
  EXPECT_THROW(StreamHealthMonitor(0.0), std::invalid_argument);
  EXPECT_THROW(StreamHealthMonitor(-1.0), std::invalid_argument);
  EXPECT_THROW(StreamHealthMonitor(1.0, 1), std::invalid_argument);
}

TEST(StreamHealth, StateNameIsStable)
{
  EXPECT_EQ(hikrobot_camera_ros2::to_string(StreamState::kNoFrames), "no_frames");
  EXPECT_EQ(hikrobot_camera_ros2::to_string(StreamState::kOk), "ok");
  EXPECT_EQ(hikrobot_camera_ros2::to_string(StreamState::kStalled), "stalled");
  EXPECT_EQ(hikrobot_camera_ros2::to_string(StreamState::kBackendError), "backend_error");
}
