// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

#include "hikrobot_camera_ros2/backend_registry.hpp"
#include "hikrobot_camera_ros2/mock_backend.hpp"

using hikrobot_camera_ros2::BackendRequest;
using hikrobot_camera_ros2::BackendUnavailableError;
using hikrobot_camera_ros2::MockBackend;
using hikrobot_camera_ros2::MockBackendOptions;
using hikrobot_camera_ros2::create_backend;

namespace
{
class FakeClock
{
public:
  double operator()() const {return now;}
  void advance(double seconds) {now += seconds;}
  double now{1000.0};
};

MockBackendOptions smallOptions()
{
  MockBackendOptions options;
  options.width = 32;
  options.height = 24;
  options.encoding = "mono8";
  options.fps = 1000.0;
  options.pattern = "moving_bar";
  return options;
}

std::unique_ptr<MockBackend> makeBackend(
  MockBackendOptions options, const FakeClock * clock = nullptr)
{
  if (clock == nullptr) {
    return std::make_unique<MockBackend>(options, []() {return 1000.0;}, []() {return int64_t{123456789};});
  }
  const FakeClock * captured = clock;
  return std::make_unique<MockBackend>(options, [captured]() {return (*captured)();}, []() {return int64_t{123456789};});
}
}  // namespace

TEST(MockBackend, GrabYieldsWellFormedFrame)
{
  auto backend = makeBackend(smallOptions());
  backend->open();
  const auto frame = backend->grab(0.0);
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(frame->width, 32);
  EXPECT_EQ(frame->height, 24);
  EXPECT_EQ(frame->encoding, "mono8");
  EXPECT_EQ(frame->step(), 32u);
  EXPECT_EQ(frame->expected_size(), 32u * 24u);
  EXPECT_TRUE(frame->is_well_formed());
}

TEST(MockBackend, SequenceAdvancesAndContentChanges)
{
  auto backend = makeBackend(smallOptions());
  backend->open();
  const auto first = backend->grab(0.0);
  const auto second = backend->grab(0.0);
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(first->sequence, 0u);
  EXPECT_EQ(second->sequence, 1u);
  EXPECT_NE(first->data, second->data) << "moving_bar must move between frames";
}

TEST(MockBackend, SameSequenceIsDeterministicAcrossInstances)
{
  auto a = makeBackend(smallOptions());
  auto b = makeBackend(smallOptions());
  a->open();
  b->open();
  const auto fa = a->grab(0.0);
  const auto fb = b->grab(0.0);
  ASSERT_TRUE(fa.has_value());
  ASSERT_TRUE(fb.has_value());
  EXPECT_EQ(fa->data, fb->data);
}

TEST(MockBackend, HardwareStampIsNotFaked)
{
  auto backend = makeBackend(smallOptions());
  backend->open();
  const auto frame = backend->grab(0.0);
  ASSERT_TRUE(frame.has_value());
  EXPECT_FALSE(frame->hardware_stamp_ns.has_value());
}

TEST(MockBackend, AllDeclaredPatternsRender)
{
  for (const auto & pattern : MockBackend::patterns()) {
    auto options = smallOptions();
    options.pattern = pattern;
    auto backend = makeBackend(options);
    backend->open();
    const auto frame = backend->grab(0.0);
    ASSERT_TRUE(frame.has_value()) << pattern;
    EXPECT_TRUE(frame->is_well_formed()) << pattern;
  }
}

TEST(MockBackend, Bgr8Geometry)
{
  auto options = smallOptions();
  options.encoding = "bgr8";
  auto backend = makeBackend(options);
  backend->open();
  const auto frame = backend->grab(0.0);
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(frame->step(), 32u * 3u);
  EXPECT_TRUE(frame->is_well_formed());
}

TEST(MockBackend, DropoutReportsNoDataThenRecovers)
{
  FakeClock clock;
  auto options = smallOptions();
  options.fail_after_frames = 2;
  options.fail_duration_s = 10.0;
  auto backend = makeBackend(options, &clock);
  backend->open();
  EXPECT_TRUE(backend->grab(0.0).has_value());
  EXPECT_TRUE(backend->grab(0.0).has_value());
  EXPECT_FALSE(backend->grab(0.0).has_value())
    << "stream must report 'no data', not a stale frame";
  clock.advance(11.0);
  EXPECT_TRUE(backend->grab(0.0).has_value()) << "must recover after the injected outage";
}

TEST(MockBackend, InjectedErrorThrowsFromGrab)
{
  auto options = smallOptions();
  options.raise_after_frames = 1;
  auto backend = makeBackend(options);
  backend->open();
  EXPECT_TRUE(backend->grab(0.0).has_value());
  EXPECT_THROW(backend->grab(0.0), std::runtime_error);
}

TEST(MockBackend, GrabBeforeOpenIsAnError)
{
  auto backend = makeBackend(smallOptions());
  EXPECT_THROW(backend->grab(0.0), std::runtime_error);
}

TEST(MockBackend, CloseIsIdempotent)
{
  auto backend = makeBackend(smallOptions());
  backend->open();
  backend->close();
  EXPECT_NO_THROW(backend->close());
}

TEST(MockBackend, InvalidPatternAndFpsAreRejectedAtConstruction)
{
  auto bad_pattern = smallOptions();
  bad_pattern.pattern = "nope";
  EXPECT_THROW(makeBackend(bad_pattern), std::invalid_argument);

  auto bad_fps = smallOptions();
  bad_fps.fps = 0.0;
  EXPECT_THROW(makeBackend(bad_fps), std::invalid_argument);
}

TEST(MockBackend, InvalidGeometryIsRejected)
{
  auto options = smallOptions();
  options.width = 0;
  EXPECT_THROW(makeBackend(options), std::invalid_argument);
}

TEST(BackendRegistry, UnknownNameListsValidChoices)
{
  BackendRequest request;
  request.name = "does-not-exist";
  try {
    create_backend(request);
    FAIL() << "expected BackendUnavailableError";
  } catch (const BackendUnavailableError & exc) {
    const std::string message = exc.what();
    EXPECT_NE(message.find("available"), std::string::npos) << message;
    EXPECT_NE(message.find("mock"), std::string::npos) << message;
  }
}

TEST(BackendRegistry, MockIsSelectableAndIsNotAHardwareBackend)
{
  BackendRequest request;
  request.name = "mock";
  request.mock = smallOptions();
  auto backend = create_backend(request);
  ASSERT_NE(backend, nullptr);
  EXPECT_EQ(backend->name(), "mock");
  EXPECT_FALSE(backend->requires_hardware());
}

TEST(BackendRegistry, NameIsCaseInsensitiveAndTrimmed)
{
  BackendRequest request;
  request.name = "  MoCk ";
  request.mock = smallOptions();
  auto backend = create_backend(request);
  ASSERT_NE(backend, nullptr);
  EXPECT_EQ(backend->name(), "mock");
}


TEST(MockBackend, ReplaysAStillImageWhenImagePathIsSet)
{
  // A tiny PPM needs no image library to create, and cv::imread handles it.
  const std::string path =
    (std::filesystem::path(::testing::TempDir()) / "scene.ppm").string();
  {
    std::ofstream out(path, std::ios::binary);
    out << "P6\n4 3\n255\n";
    for (int i = 0; i < 4 * 3; ++i) {
      out.put(static_cast<char>(200));
      out.put(static_cast<char>(100));
      out.put(static_cast<char>(50));
    }
  }

  auto options = smallOptions();  // 32x24 mono8
  options.image_path = path;
  auto backend = makeBackend(options);
  backend->open();

  const auto first = backend->grab(0.0);
  ASSERT_TRUE(first.has_value());
  EXPECT_TRUE(first->is_well_formed());
  EXPECT_EQ(first->width, 32) << "the scene must be resized to the configured geometry";
  EXPECT_EQ(first->height, 24);
  EXPECT_EQ(first->encoding, "mono8");

  const auto second = backend->grab(0.0);
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(first->data, second->data)
    << "a replayed scene is time-invariant by design (needed for latency measurement)";
}

TEST(MockBackend, MissingSceneImageFailsLoudly)
{
  auto options = smallOptions();
  options.image_path = "/nonexistent/scene.jpg";
  auto backend = makeBackend(options);
  EXPECT_THROW(backend->open(), std::runtime_error)
    << "a missing scene must not silently fall back to the synthetic pattern";
}

TEST(MockBackend, UnsupportedSceneEncodingFailsLoudly)
{
  auto options = smallOptions();
  options.encoding = "mono16";
  options.image_path = "/nonexistent/scene.jpg";
  auto backend = makeBackend(options);
  EXPECT_THROW(backend->open(), std::runtime_error);
}
