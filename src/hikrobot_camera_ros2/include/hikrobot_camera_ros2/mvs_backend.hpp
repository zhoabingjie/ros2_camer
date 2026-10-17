// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0
//
// Hikrobot MVS backend: SDK availability probe only. Not a driver. On purpose.
//
// PHASE 0 AUDIT EVIDENCE (extras/docs/ENVIRONMENT_AUDIT.md section 4):
//   * The Linux MVS SDK is not installed in this environment.
//   * No Hikvision camera is attached.
//   * The only SDK archive on hand is MVS_Win_STD_5.0.2 -- a Windows build.
//
// An MvCameraControl wrapper cannot be written from memory: symbol names, struct
// layouts, pixel-format constants and error codes must come from the real
// `MvCameraControl.h` of the Linux SDK. Guessing them yields code that looks
// complete and fails on the robot. This backend therefore detects the SDK,
// reports exactly what is missing, and refuses to acquire -- it never returns
// fabricated frames.

#ifndef HIKROBOT_CAMERA_ROS2__MVS_BACKEND_HPP_
#define HIKROBOT_CAMERA_ROS2__MVS_BACKEND_HPP_

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "hikrobot_camera_ros2/camera_backend.hpp"

namespace hikrobot_camera_ros2
{

/// @brief What the environment actually provides. Reports; never assumes.
struct MvsSdkProbe
{
  std::vector<std::string> lib_paths;
  std::vector<std::string> header_paths;
  std::vector<std::pair<std::string, std::string>> env;
  std::vector<std::string> searched_lib_dirs;
  std::vector<std::string> searched_header_paths;

  bool library_found() const {return !lib_paths.empty();}
  bool headers_found() const {return !header_paths.empty();}
  /// A wrapper needs both the shared library and the headers to be written and verified.
  bool wrapper_writable() const {return library_found() && headers_found();}
  std::string summary() const;
};

/// @brief Detect the Linux MVS SDK without loading anything.
MvsSdkProbe probe_sdk(const std::vector<std::string> & extra_lib_candidates = {});

class MvsBackend : public CameraBackend
{
public:
  explicit MvsBackend(std::vector<std::string> sdk_lib_candidates = {});

  std::string name() const override {return "mvs";}
  bool requires_hardware() const override {return true;}
  void open() override;
  void close() override;
  std::optional<Frame> grab(double timeout_s) override;
  std::string describe() const override;

  const MvsSdkProbe & probe() const;

private:
  std::vector<std::string> extra_candidates_;
  mutable std::optional<MvsSdkProbe> probe_;
};

}  // namespace hikrobot_camera_ros2

#endif  // HIKROBOT_CAMERA_ROS2__MVS_BACKEND_HPP_
