// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "camera_driver/mvs_backend.hpp"

#ifdef HIKROBOT_MVS_AVAILABLE
#include <MvCameraControl.h>
#endif

using camera_driver::MvsBackendOptions;
using camera_driver::mvs_pixel_type_is_color;
using camera_driver::mvs_pixel_type_name;
using camera_driver::rgb8_to_bgr8;
using camera_driver::validate_mvs_options;

TEST(MvsControls, GenericDefaultsAreAutoCompatible)
{
  const MvsBackendOptions options;
  EXPECT_EQ(options.transport, "auto");
  EXPECT_EQ(options.exposure_mode, "auto");
  EXPECT_EQ(options.exposure_auto_mode, "continuous");
  EXPECT_EQ(options.gain_mode, "auto");
  EXPECT_EQ(options.gain_auto_mode, "continuous");
  EXPECT_EQ(options.output_encoding, "auto");
  EXPECT_NO_THROW(validate_mvs_options(options));
}

TEST(MvsControls, AutoManualAndZeroGainAreValid)
{
  MvsBackendOptions options;
  options.exposure_mode = "manual";
  options.exposure_time_us = 8000.0;
  options.gain_mode = "manual";
  options.gain_value = 0.0;
  EXPECT_NO_THROW(validate_mvs_options(options));

  options.exposure_mode = "auto";
  options.exposure_auto_mode = "once";
  options.gain_mode = "auto";
  options.gain_auto_mode = "once";
  EXPECT_NO_THROW(validate_mvs_options(options));

  options = MvsBackendOptions{};
  options.exposure_mode = "camera";
  options.gain_mode = "camera";
  EXPECT_NO_THROW(validate_mvs_options(options));
}

TEST(MvsControls, InvalidModesAndNonFiniteValuesAreRejected)
{
  MvsBackendOptions options;
  options.exposure_mode = "program";
  EXPECT_THROW(validate_mvs_options(options), std::invalid_argument);

  options = MvsBackendOptions{};
  options.exposure_auto_mode = "sometimes";
  EXPECT_THROW(validate_mvs_options(options), std::invalid_argument);

  options = MvsBackendOptions{};
  options.exposure_time_us = 0.0;
  EXPECT_THROW(validate_mvs_options(options), std::invalid_argument);

  options = MvsBackendOptions{};
  options.gain_value = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(validate_mvs_options(options), std::invalid_argument);
}

TEST(MvsControls, AutoEncodingAcceptsMonoAndColorModes)
{
  MvsBackendOptions options;
  options.pixel_format = "Mono8";
  EXPECT_NO_THROW(validate_mvs_options(options));

  options = MvsBackendOptions{};
  options.output_encoding = "mono8";
  EXPECT_NO_THROW(validate_mvs_options(options));

  options = MvsBackendOptions{};
  options.output_encoding = "bgr8";
  options.pixel_format = "Mono8";
  EXPECT_THROW(validate_mvs_options(options), std::invalid_argument);

  options = MvsBackendOptions{};
  options.pixel_format = "BayerRG8";
  EXPECT_NO_THROW(validate_mvs_options(options));
}

TEST(MvsControls, RgbToBgrSwapsOnlyRedAndBlue)
{
  const std::vector<uint8_t> rgb{
    1, 2, 3,
    10, 20, 30,
    100, 110, 120,
    255, 128, 0};
  const auto bgr = rgb8_to_bgr8(rgb, 2, 2);
  const std::vector<uint8_t> expected{
    3, 2, 1,
    30, 20, 10,
    120, 110, 100,
    0, 128, 255};
  EXPECT_EQ(bgr, expected);
}

TEST(MvsControls, RgbConversionRejectsWrongPayload)
{
  EXPECT_THROW(
    rgb8_to_bgr8(std::vector<uint8_t>(11, 0), 2, 2),
    std::invalid_argument);
}

#ifdef HIKROBOT_MVS_AVAILABLE
TEST(MvsControls, MvsPixelTypeClassificationMatchesSdkConstants)
{
  EXPECT_TRUE(mvs_pixel_type_is_color(PixelType_Gvsp_BGR8_Packed));
  EXPECT_TRUE(mvs_pixel_type_is_color(PixelType_Gvsp_RGB8_Packed));
  EXPECT_TRUE(mvs_pixel_type_is_color(PixelType_Gvsp_BayerRG8));
  EXPECT_TRUE(mvs_pixel_type_is_color(PixelType_Gvsp_BayerBG8));
  EXPECT_TRUE(mvs_pixel_type_is_color(PixelType_Gvsp_BayerGB8));
  EXPECT_TRUE(mvs_pixel_type_is_color(PixelType_Gvsp_BayerGR8));
  EXPECT_FALSE(mvs_pixel_type_is_color(PixelType_Gvsp_Mono8));

  EXPECT_EQ(mvs_pixel_type_name(PixelType_Gvsp_BGR8_Packed), "BGR8");
  EXPECT_EQ(mvs_pixel_type_name(PixelType_Gvsp_BayerRG8), "BayerRG8");
}
#endif
