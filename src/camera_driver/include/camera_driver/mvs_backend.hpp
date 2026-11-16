// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0
//
// 海康 MVS USB3 Vision 相机后端。
// The production node has no synthetic image fallback.
//
// The vendor library is loaded with dlopen() so the package can still be built
// and report a missing SDK without a hard vendor-link dependency. All symbols
// and structure layouts used by this backend are taken from the MVS
// headers installed on the target system; no SDK API is guessed.

#ifndef HIKROBOT_CAMERA_ROS2__MVS_BACKEND_HPP_
#define HIKROBOT_CAMERA_ROS2__MVS_BACKEND_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "camera_driver/camera_backend.hpp"

namespace camera_driver
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

  /// auto: 自动匹配 GigE/USB3 Vision；也可显式设为 usb 或 gige。
  std::string transport{"auto"};
  /// Empty = accept the only matching device. Required when several match.
  std::string serial_number;

  /// 空值表示由驱动根据相机能力自动选择像素格式。
  std::string pixel_format;
  /// 0 = leave the device's current value.
  int width{0};
  int height{0};

  /// 曝光模式：auto/manual。曝光时间单位是微秒。
  std::string exposure_mode{"auto"};
  /// 自动曝光子模式：continuous/once。
  std::string exposure_auto_mode{"continuous"};
  /// 手动曝光值；自动模式下只保存，不覆盖相机自动结果。
  double exposure_time_us{8000.0};

  /// 增益模式：auto/manual。
  std::string gain_mode{"auto"};
  /// 自动增益子模式：continuous/once。
  std::string gain_auto_mode{"continuous"};
  /// 手动增益值；0.0 是合法值。
  double gain_value{0.0};

  /// auto: 彩色优先 bgr8，单色回退 mono8；也可强制 bgr8 或 mono8。
  std::string output_encoding{"auto"};

  /// 0 = ask the SDK for the optimal GigE packet size.
  int packet_size{0};
};

/// @brief 运行时可读取的真实相机状态与参数范围。
struct MvsCameraInfo
{
  bool connected{false};
  std::string transport;
  std::string model;
  std::string serial_number;
  uint32_t vendor_id{0};
  uint32_t product_id{0};
  std::string pixel_format;
  std::string image_encoding;
  int width{0};
  int height{0};
  std::string exposure_mode;
  std::string exposure_auto_mode;
  double exposure_time_us{0.0};
  std::optional<double> exposure_min_us;
  std::optional<double> exposure_max_us;
  std::string gain_mode;
  std::string gain_auto_mode;
  double gain_value{0.0};
  std::optional<double> gain_min;
  std::optional<double> gain_max;
};

/// @brief 在访问硬件前校验控制参数，便于 ROS 参数回调给出明确错误。
void validate_mvs_options(const MvsBackendOptions & options);

/// @brief 将紧密排列的 rgb8 正确转换为 bgr8。数据长度或尺寸不匹配时抛异常。
std::vector<uint8_t> rgb8_to_bgr8(
  const std::vector<uint8_t> & rgb, int width, int height);

/// @brief 查询 MVS PixelType 数值的可读名称；未知值返回十六进制文本。
std::string mvs_pixel_type_name(uint32_t pixel_type);

/// @brief 该 MVS 像素格式能否产生真实彩色图像。
bool mvs_pixel_type_is_color(uint32_t pixel_type);

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

  /// @brief 查询当前曝光/增益、像素格式和真实取值范围。
  MvsCameraInfo camera_info() const;

  /// @brief 运行时应用曝光和增益设置；失败时尽量回滚到原状态。
  void apply_control_settings(const MvsBackendOptions & settings);

  /// @brief 运行时切换像素格式；需要时暂停并恢复取流。
  void apply_pixel_format(
    const std::string & pixel_format, const std::string & output_encoding);

  /// @brief 运行时调整 Width/Height；需要时暂停并恢复取流。
  void apply_geometry(int width, int height);

  const MvsSdkProbe & probe() const;

private:
  MvsBackendOptions options_;
  mutable std::optional<MvsSdkProbe> probe_;
  void * handle_{nullptr};
  std::vector<uint8_t> buffer_;
  /// Human-readable list of devices seen by the last enumeration.
  std::string device_summary_;
  std::string sdk_version_;
  bool grabbing_{false};
  MvsCameraInfo camera_info_;
};

}  // namespace camera_driver

#endif  // HIKROBOT_CAMERA_ROS2__MVS_BACKEND_HPP_
