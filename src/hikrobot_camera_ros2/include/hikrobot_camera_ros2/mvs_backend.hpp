// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0
//
// Hikrobot MVS GigE backend.
// The production node has no synthetic image fallback.
//
// The vendor library is loaded with dlopen() so the package can still be built
// and report a missing SDK without a hard vendor-link dependency. All symbols
// and structure layouts used by this backend are taken from the MVS
// headers installed on the target system; no SDK API is guessed.

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

/// Release MVS SDK process-wide resources after all camera backends are closed.
/// Call this from the executable's controlled shutdown path, not from a static
/// destructor: the vendor GEV transport plugin performs its own teardown and
/// static-destructor ordering is unsafe with MVS 5.0.2.
void finalize_mvs_sdk();

/// @brief Construction parameters for the real camera.
struct MvsBackendOptions
{
  std::vector<std::string> sdk_lib_candidates;


  /// Empty = leave the device's current pixel format (we convert to mono8/bgr8).
  /// Any other value is written to the "PixelFormat" node by name, e.g. "Mono8".
  std::string pixel_format;
  /// 0 = leave the device's current value.
  int width{0};
  int height{0};
  /// <= 0 = leave the device's current value (microseconds / dB).
  double exposure_us{0.0};
  double gain{0.0};
  /// 0 = ask the SDK for the optimal GigE packet size.
  int packet_size{0};
};

class MvsBackend : public CameraBackend
{
public:
  explicit MvsBackend(MvsBackendOptions options = MvsBackendOptions{});
  ~MvsBackend() override;

  std::string name() const override {return "mvs";}
  bool requires_hardware() const override {return true;}
  void open() override;
  void close() override;
  std::optional<Frame> grab(double timeout_s) override;
  std::string describe() const override;

  const MvsSdkProbe & probe() const;

private:
  MvsBackendOptions options_;
  mutable std::optional<MvsSdkProbe> probe_;
  void * handle_{nullptr};
  std::vector<uint8_t> buffer_;
  /// Human-readable list of devices seen by the last enumeration.
  std::string device_summary_;
  std::string sdk_version_;
};

}  // namespace hikrobot_camera_ros2

#endif  // HIKROBOT_CAMERA_ROS2__MVS_BACKEND_HPP_
