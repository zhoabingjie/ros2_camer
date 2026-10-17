// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0
//
// Contract under test: the MVS backend must never publish anything it cannot
// verify. Depending on the machine it either reports the missing SDK, or (with
// the SDK present) reports that no camera is reachable, or captures for real.
// It must never return a fabricated frame.

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>

#include "hikrobot_camera_ros2/backend_registry.hpp"
#include "hikrobot_camera_ros2/mvs_backend.hpp"

using hikrobot_camera_ros2::BackendRequest;
using hikrobot_camera_ros2::BackendUnavailableError;
using hikrobot_camera_ros2::MvsBackend;
using hikrobot_camera_ros2::create_backend;
using hikrobot_camera_ros2::probe_sdk;

TEST(MvsBackend, ExtraCandidateThatDoesNotExistIsNotReportedAsFound)
{
  const std::string bogus = "/nonexistent/path/libMvCameraControl.so";
  const auto probe = probe_sdk({bogus});
  EXPECT_EQ(
    std::find(probe.lib_paths.begin(), probe.lib_paths.end(), bogus), probe.lib_paths.end())
    << "a nonexistent extra candidate must never be reported as found";
}

TEST(MvsBackend, ProbeReportsSearchedLocationsWhenNothingIsFound)
{
  const auto probe = probe_sdk();
  if (probe.library_found()) {
    GTEST_SKIP() << "MVS SDK is installed on this machine; the 'not found' path is not exercised";
  }
  EXPECT_FALSE(probe.headers_found());
  const std::string summary = probe.summary();
  EXPECT_NE(summary.find("NOT FOUND"), std::string::npos);
  EXPECT_NE(summary.find("/opt/MVS/lib/64"), std::string::npos)
    << "the report must name where it looked";
}

TEST(MvsBackend, OpenNeverSucceedsWithoutAWorkingCameraAndNeverInventsAFrame)
{
  MvsBackend backend;
  const auto probe = probe_sdk();

  std::string message;
  try {
    backend.open();
    // With a camera attached and the SDK present this could legitimately succeed.
    // In that case the contract to check is that grab() returns real data or
    // throws -- never a placeholder.
    backend.close();
    GTEST_SKIP() << "a camera is attached and the SDK works; capture path not exercised here";
  } catch (const std::exception & exc) {
    message = exc.what();
  }

  ASSERT_FALSE(message.empty()) << "open() must explain why it could not start";
  if (!probe.library_found()) {
    EXPECT_NE(message.find("MVS SDK not found"), std::string::npos) << message;
    EXPECT_NE(message.find("ACTION"), std::string::npos) << message;
  } else {
    // SDK present: the failure must name the actual blocker.
    EXPECT_NE(message.find("MVS"), std::string::npos) << message;
  }
  EXPECT_THROW(backend.grab(0.0), std::exception);
}

TEST(MvsBackend, DescribeReportsSdkAndImplementationState)
{
  MvsBackend backend;
  const std::string description = backend.describe();
  const auto probe = probe_sdk();

  EXPECT_NE(description.find("requires_hardware=true"), std::string::npos) << description;
  EXPECT_NE(description.find("implemented="), std::string::npos) << description;
  // The reported SDK state must match what the probe actually found.
  EXPECT_EQ(
    description.find("sdk_library_found=true") != std::string::npos,
    probe.library_found()) << description;
  EXPECT_EQ(
    description.find("sdk_headers_found=true") != std::string::npos,
    probe.headers_found()) << description;
}

TEST(MvsBackend, IsSelectableThroughTheRegistryAndNeedsHardware)
{
  BackendRequest request;
  request.name = "mvs";
  auto backend = create_backend(request);
  ASSERT_NE(backend, nullptr);
  EXPECT_EQ(backend->name(), "mvs");
  EXPECT_TRUE(backend->requires_hardware())
    << "a real camera backend must declare that it needs hardware";
}
