// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "hikrobot_camera_ros2/frame.hpp"

using hikrobot_camera_ros2::Frame;
using hikrobot_camera_ros2::bytes_per_pixel;
using hikrobot_camera_ros2::validate_geometry;

TEST(Frame, BytesPerPixelForKnownEncodings)
{
  EXPECT_EQ(bytes_per_pixel("mono8"), 1u);
  EXPECT_EQ(bytes_per_pixel("8UC1"), 1u);
  EXPECT_EQ(bytes_per_pixel("bgr8"), 3u);
  EXPECT_EQ(bytes_per_pixel("rgb8"), 3u);
  EXPECT_EQ(bytes_per_pixel("mono16"), 2u);
  EXPECT_EQ(bytes_per_pixel("16UC1"), 2u);
  EXPECT_EQ(bytes_per_pixel("made_up"), 0u) << "unknown encodings must report 0, not a guess";
}

TEST(Frame, ValidateGeometryAcceptsValidAndRejectsImpossible)
{
  EXPECT_NO_THROW(validate_geometry(640, 480, "mono8"));
  EXPECT_THROW(validate_geometry(0, 480, "mono8"), std::invalid_argument);
  EXPECT_THROW(validate_geometry(640, 0, "mono8"), std::invalid_argument);
  EXPECT_THROW(validate_geometry(-4, 4, "mono8"), std::invalid_argument);
  EXPECT_THROW(validate_geometry(640, 480, "made_up"), std::invalid_argument);
}

TEST(Frame, StepAndExpectedSizeFollowEncoding)
{
  Frame frame;
  frame.width = 64;
  frame.height = 48;
  frame.encoding = "bgr8";
  EXPECT_EQ(frame.step(), 192u);
  EXPECT_EQ(frame.expected_size(), 192u * 48u);
}

TEST(Frame, WellFormedRequiresMatchingPayload)
{
  Frame frame;
  frame.width = 4;
  frame.height = 2;
  frame.encoding = "mono8";
  EXPECT_FALSE(frame.is_well_formed()) << "no payload yet";
  frame.data.assign(7, 0);
  EXPECT_FALSE(frame.is_well_formed()) << "short payload";
  frame.data.assign(frame.expected_size(), 0);
  EXPECT_TRUE(frame.is_well_formed());
}

TEST(Frame, WellFormedRejectsUnknownEncoding)
{
  Frame frame;
  frame.width = 4;
  frame.height = 2;
  frame.encoding = "made_up";
  frame.data.assign(8, 0);
  EXPECT_FALSE(frame.is_well_formed());
}

TEST(Frame, HardwareStampDefaultsToEmptyAndIsNotFaked)
{
  Frame frame;
  EXPECT_FALSE(frame.hardware_stamp_ns.has_value())
    << "a frame must not carry an invented device timestamp";
}
