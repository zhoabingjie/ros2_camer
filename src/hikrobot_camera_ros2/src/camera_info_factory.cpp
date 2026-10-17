// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0

#include "hikrobot_camera_ros2/camera_info_factory.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <sstream>

#include <yaml-cpp/yaml.h>

namespace hikrobot_camera_ros2
{
namespace
{
constexpr double kEpsilon = 1e-9;

std::vector<double> numbers_from(
  const YAML::Node & node, const std::string & key, const std::string & source,
  std::optional<std::size_t> expected)
{
  if (!node[key]) {throw CalibrationError(source + ": missing '" + key + "'");}
  const YAML::Node block = node[key];
  const YAML::Node data = block["data"] ? block["data"] : block;
  if (!data.IsSequence()) {throw CalibrationError(source + ": '" + key + "' must be a sequence");}
  if (expected.has_value() && data.size() != *expected) {
    std::ostringstream os;
    os << source << ": '" << key << "' must hold exactly " << *expected
       << " numbers, got " << data.size();
    throw CalibrationError(os.str());
  }
  std::vector<double> out;
  out.reserve(data.size());
  for (const auto & entry : data) {out.push_back(entry.as<double>());}
  return out;
}

template<std::size_t N>
std::array<double, N> to_array(const std::vector<double> & values)
{
  std::array<double, N> out{};
  std::copy(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(N), out.begin());
  return out;
}
}  // namespace

bool Calibration::is_degenerate() const
{
  return std::abs(camera_matrix[0]) < kEpsilon || std::abs(camera_matrix[4]) < kEpsilon;
}

void Calibration::check_against(int width, int height) const
{
  if (image_width != width || image_height != height) {
    std::ostringstream os;
    os << "calibration is for " << image_width << "x" << image_height << " but the stream is "
       << width << "x" << height
       << ": re-run camera_calibration at the current resolution (file: "
       << (source_path.empty() ? "n/a" : source_path) << ")";
    throw CalibrationMismatchError(os.str());
  }
}

Calibration load_calibration(const std::string & path)
{
  std::error_code ec;
  if (!std::filesystem::is_regular_file(path, ec)) {
    throw CalibrationError("calibration file not found: " + path);
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & exc) {
    throw CalibrationError(path + ": not valid YAML (" + exc.what() + ")");
  }
  if (!root.IsMap()) {throw CalibrationError(path + ": expected a YAML mapping at the top level");}

  if (!root["image_width"] || !root["image_height"]) {
    throw CalibrationError(path + ": image_width/image_height must be integers");
  }

  Calibration calibration;
  calibration.source_path = path;
  calibration.image_width = root["image_width"].as<int>();
  calibration.image_height = root["image_height"].as<int>();
  if (root["camera_name"]) {calibration.camera_name = root["camera_name"].as<std::string>();}
  if (root["distortion_model"]) {
    calibration.distortion_model = root["distortion_model"].as<std::string>();
  }
  calibration.camera_matrix = to_array<9>(numbers_from(root, "camera_matrix", path, 9));
  calibration.rectification_matrix = to_array<9>(numbers_from(root, "rectification_matrix", path, 9));
  calibration.projection_matrix = to_array<12>(numbers_from(root, "projection_matrix", path, 12));
  // Distortion coefficient count is model dependent (5 plumb_bob, 8 rational, 4 equidistant...),
  // so it is read as-is instead of being forced to a fixed length.
  if (root["distortion_coefficients"]) {
    calibration.distortion_coefficients =
      numbers_from(root, "distortion_coefficients", path, std::nullopt);
  }
  if (root["reprojection_error"]) {
    calibration.reprojection_error = root["reprojection_error"].as<double>();
  }

  if (calibration.image_width <= 0 || calibration.image_height <= 0) {
    throw CalibrationError(
            path + ": calibration resolution must be positive, got " +
            std::to_string(calibration.image_width) + "x" +
            std::to_string(calibration.image_height));
  }
  if (calibration.is_degenerate()) {
    std::ostringstream os;
    os << path << ": camera_matrix has zero focal length (fx=" << calibration.camera_matrix[0]
       << ", fy=" << calibration.camera_matrix[4] << "); this is not a usable calibration";
    throw CalibrationError(os.str());
  }
  return calibration;
}

sensor_msgs::msg::CameraInfo build_camera_info(
  int width, int height, const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp, const Calibration * calibration)
{
  sensor_msgs::msg::CameraInfo message;
  message.header.frame_id = frame_id;
  message.header.stamp = stamp;
  message.width = static_cast<uint32_t>(width);
  message.height = static_cast<uint32_t>(height);

  if (calibration == nullptr) {
    // ROS convention for "no intrinsics known": all-zero K and P. image_proc reads
    // this as uncalibrated and passes the image through instead of rectifying with
    // invented numbers.
    message.k.fill(0.0);
    message.r.fill(0.0);
    message.p.fill(0.0);
    message.distortion_model.clear();
    message.d.clear();
    return message;
  }

  calibration->check_against(width, height);
  message.k = calibration->camera_matrix;
  message.r = calibration->rectification_matrix;
  message.p = calibration->projection_matrix;
  message.distortion_model = calibration->distortion_model;
  message.d = calibration->distortion_coefficients;
  return message;
}

bool is_uncalibrated(const sensor_msgs::msg::CameraInfo & info)
{
  return std::all_of(info.k.begin(), info.k.end(), [](double v) {return std::abs(v) < kEpsilon;});
}

}  // namespace hikrobot_camera_ros2
