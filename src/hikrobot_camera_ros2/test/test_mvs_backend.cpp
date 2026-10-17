// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0
//
// Contract under test: the MVS backend has no business publishing anything it
// cannot verify. It must either refuse with an actionable message naming the
// missing artifact, or (once the Linux SDK is installed) refuse to publish until
// the wrapper is written against the real headers. It must never return a frame.

#include <gtest/gtest.h>

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

TEST(MvsBackend, ProbeReportsSearchedLocationsWhenNothingIsFound)
{
  const auto probe = probe_sdk();
  if (probe.library_found()) {
    GTEST_SKIP() << "MVS SDK is installed on this machine; the 'not found' path is not exercised";
  }
  EXPECT_FALSE(probe.headers_found());
  EXPECT_FALSE(probe.wrapper_writable());
  const std::string summary = probe.summary();
  EXPECT_NE(summary.find("NOT FOUND"), std::string::npos);
  EXPECT_NE(summary.find("/opt/MVS/lib/64"), std::string::npos)
    << "the report must name where it looked";
}

TEST(MvsBackend, ExtraCandidateThatDoesNotExistIsNotReportedAsFound)
{
  const auto probe = probe_sdk({"/nonexistent/path/libMvCameraControl.so"});
  EXPECT_FALSE(probe.library_found()) << "a missing candidate must not count as found";
}

TEST(MvsBackend, OpenRefusesAndNeverReturnsAFrame)
{
  MvsBackend backend;
  std::string message;
  try {
    backend.open();
    FAIL() << "MvsBackend::open() must not succeed before the real wrapper exists";
  } catch (const BackendUnavailableError & exc) {
    message = exc.what();
    EXPECT_NE(message.find("MVS SDK not found"), std::string::npos) << message;
    EXPECT_NE(message.find("ACTION"), std::string::npos)
      << "the error must be actionable, not just descriptive: " << message;
    EXPECT_NE(message.find("Linux"), std::string::npos) << message;
  } catch (const std::logic_error & exc) {
    // SDK present but the wrapper is deliberately unimplemented: also acceptable,
    // and it must say why rather than pretend.
    message = exc.what();
    EXPECT_NE(message.find("not implemented"), std::string::npos) << message;
  }
  EXPECT_THROW(backend.grab(0.0), std::exception);
}

TEST(MvsBackend, DescribeStatesItIsNotImplemented)
{
  MvsBackend backend;
  const std::string description = backend.describe();
  EXPECT_NE(description.find("implemented=false"), std::string::npos) << description;
  EXPECT_NE(description.find("requires_hardware=true"), std::string::npos) << description;
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
