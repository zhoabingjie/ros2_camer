// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "hikrobot_camera_ros2/camera_info_factory.hpp"

using hikrobot_camera_ros2::Calibration;
using hikrobot_camera_ros2::CalibrationError;
using hikrobot_camera_ros2::CalibrationMismatchError;
using hikrobot_camera_ros2::build_camera_info;
using hikrobot_camera_ros2::is_uncalibrated;
using hikrobot_camera_ros2::load_calibration;

namespace
{
builtin_interfaces::msg::Time stamp()
{
  builtin_interfaces::msg::Time value;
  value.sec = 0;
  value.nanosec = 0;
  return value;
}

std::string writeCalibration(
  const std::string & name, int width = 640, int height = 480, double fx = 500.0, double fy = 500.0,
  double cx = 320.0, double cy = 240.0)
{
  const std::string path = (std::filesystem::path(::testing::TempDir()) / name).string();
  std::ofstream out(path);
  out << "image_width: " << width << "\n"
      << "image_height: " << height << "\n"
      << "camera_name: cam0\n"
      << "camera_matrix:\n  rows: 3\n  cols: 3\n  data: [" << fx << ", 0.0, " << cx
      << ", 0.0, " << fy << ", " << cy << ", 0.0, 0.0, 1.0]\n"
      << "distortion_model: plumb_bob\n"
      << "distortion_coefficients:\n  rows: 1\n  cols: 5\n  data: [0.1, -0.2, 0.001, 0.002, 0.05]\n"
      << "rectification_matrix:\n  rows: 3\n  cols: 3\n  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]\n"
      << "projection_matrix:\n  rows: 3\n  cols: 4\n  data: [" << fx << ", 0.0, " << cx
      << ", 0.0, 0.0, " << fy << ", " << cy << ", 0.0, 0.0, 0.0, 1.0, 0.0]\n"
      << "reprojection_error: 0.31\n";
  out.close();
  return path;
}
}  // namespace

TEST(CameraInfoFactory, UncalibratedOutputIsAllZeroAndNotInvented)
{
  const auto info = build_camera_info(640, 480, "camera_optical_frame", stamp(), nullptr);
  EXPECT_EQ(info.width, 640u);
  EXPECT_EQ(info.height, 480u);
  EXPECT_EQ(info.header.frame_id, "camera_optical_frame");
  EXPECT_EQ(info.k[0], 0.0) << "no focal length may be invented";
  EXPECT_EQ(info.k[4], 0.0);
  EXPECT_EQ(info.k[2], 0.0);
  EXPECT_EQ(info.p[0], 0.0);
  EXPECT_TRUE(info.d.empty());
  EXPECT_TRUE(info.distortion_model.empty());
  EXPECT_TRUE(is_uncalibrated(info));
}

TEST(CameraInfoFactory, CalibratedOutputUsesFileValues)
{
  const auto path = writeCalibration("ok.yaml");
  const auto calibration = load_calibration(path);
  const auto info = build_camera_info(640, 480, "cam", stamp(), &calibration);
  EXPECT_DOUBLE_EQ(info.k[0], 500.0);
  EXPECT_DOUBLE_EQ(info.k[4], 500.0);
  EXPECT_DOUBLE_EQ(info.k[2], 320.0);
  EXPECT_DOUBLE_EQ(info.k[5], 240.0);
  ASSERT_EQ(info.d.size(), 5u);
  EXPECT_DOUBLE_EQ(info.d[0], 0.1);
  EXPECT_EQ(info.distortion_model, "plumb_bob");
  EXPECT_FALSE(is_uncalibrated(info));
  ASSERT_TRUE(calibration.reprojection_error.has_value());
  EXPECT_NEAR(*calibration.reprojection_error, 0.31, 1e-9);
}

TEST(CameraInfoFactory, MissingFileIsReported)
{
  const std::string missing =
    (std::filesystem::path(::testing::TempDir()) / "definitely_absent.yaml").string();
  try {
    load_calibration(missing);
    FAIL() << "expected CalibrationError";
  } catch (const CalibrationError & exc) {
    EXPECT_NE(std::string(exc.what()).find("not found"), std::string::npos) << exc.what();
  }
}

TEST(CameraInfoFactory, ZeroFocalLengthIsRejected)
{
  const auto path = writeCalibration("degenerate.yaml", 640, 480, 0.0, 0.0);
  try {
    load_calibration(path);
    FAIL() << "expected CalibrationError";
  } catch (const CalibrationError & exc) {
    EXPECT_NE(std::string(exc.what()).find("zero focal length"), std::string::npos) << exc.what();
  }
}

TEST(CameraInfoFactory, ShortMatrixIsRejected)
{
  const std::string path =
    (std::filesystem::path(::testing::TempDir()) / "short_matrix.yaml").string();
  std::ofstream out(path);
  out << "image_width: 640\nimage_height: 480\n"
      << "camera_matrix:\n  data: [1.0, 2.0]\n"
      << "distortion_coefficients:\n  data: []\n"
      << "rectification_matrix:\n  data: [1,0,0,0,1,0,0,0,1]\n"
      << "projection_matrix:\n  data: [1,0,0,0,0,1,0,0,0,0,1,0]\n";
  out.close();
  try {
    load_calibration(path);
    FAIL() << "expected CalibrationError";
  } catch (const CalibrationError & exc) {
    EXPECT_NE(std::string(exc.what()).find("camera_matrix"), std::string::npos) << exc.what();
  }
}

TEST(CameraInfoFactory, ResolutionMismatchIsRejectedNotRescaled)
{
  const auto path = writeCalibration("mismatch.yaml", 1280, 960);
  const auto calibration = load_calibration(path);
  EXPECT_THROW(calibration.check_against(640, 480), CalibrationMismatchError);
  try {
    calibration.check_against(640, 480);
  } catch (const CalibrationMismatchError & exc) {
    EXPECT_NE(std::string(exc.what()).find("re-run camera_calibration"), std::string::npos)
      << exc.what();
  }
  EXPECT_THROW(build_camera_info(640, 480, "cam", stamp(), &calibration), CalibrationMismatchError);
}

TEST(CameraInfoFactory, DistortionCoefficientCountIsNotForced)
{
  const std::string path =
    (std::filesystem::path(::testing::TempDir()) / "equidistant.yaml").string();
  std::ofstream out(path);
  out << "image_width: 640\nimage_height: 480\ndistortion_model: equidistant\n"
      << "camera_matrix:\n  data: [500,0,320,0,500,240,0,0,1]\n"
      << "distortion_coefficients:\n  data: [0.1, 0.2, 0.3, 0.4]\n"
      << "rectification_matrix:\n  data: [1,0,0,0,1,0,0,0,1]\n"
      << "projection_matrix:\n  data: [500,0,320,0,0,500,240,0,0,0,1,0]\n";
  out.close();
  const auto calibration = load_calibration(path);
  EXPECT_EQ(calibration.distortion_coefficients.size(), 4u)
    << "4-parameter models (e.g. equidistant) must survive the loader";
  EXPECT_EQ(calibration.distortion_model, "equidistant");
}
