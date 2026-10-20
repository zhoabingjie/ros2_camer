// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0
//
// Hikrobot MVS backend.
//
// Without the Linux MVS SDK this file only probes for it and reports precisely
// what is missing (Phase 0/1 behaviour). When the SDK headers are present at
// configure time (-DHIKROBOT_MVS_INCLUDE_DIR, default /opt/MVS/include) the real
// acquisition path is compiled in.
//
// The vendor library is loaded with dlopen() rather than linked, on purpose:
// `camera_node` must keep starting on machines without the SDK so the mock
// backend keeps working. A hard link dependency would break that.
//
// Every symbol below was read from the SDK's own headers
// (/opt/MVS/include/MvCameraControl.h, CameraParams.h, MvErrorDefine.h,
// PixelType.h). Nothing here is guessed.

#include "hikrobot_camera_ros2/mvs_backend.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include <dlfcn.h>

#ifdef HIKROBOT_MVS_AVAILABLE
#include <MvCameraControl.h>
#endif

namespace hikrobot_camera_ros2
{
namespace
{
const std::vector<std::string> & lib_dirs()
{
  static const std::vector<std::string> kDirs{
    "/opt/MVS/lib/64",
    "/opt/MVS/lib/32",
    "/opt/MVS/lib/aarch64",
    "/opt/MVS/lib/arm",
    "/usr/lib",
    "/usr/lib/x86_64-linux-gnu",
    "/usr/local/lib"};
  return kDirs;
}

constexpr const char * kLibPrefix = "libMvCameraControl.so";

const std::vector<std::string> & header_candidates()
{
  static const std::vector<std::string> kPaths{
    "/opt/MVS/include/MvCameraControl.h",
    "/opt/MVS/include/x86_64/MvCameraControl.h",
    "/opt/MVS/include/aarch64/MvCameraControl.h",
    "/usr/include/MvCameraControl.h",
    "/usr/local/include/MvCameraControl.h"};
  return kPaths;
}

const std::vector<std::string> & env_hints()
{
  static const std::vector<std::string> kHints{
    "MVCAM_SDK_PATH", "MVCAM_COMMON_RUNENV", "MVCAM_GENICAM_CLPROTOCOL"};
  return kHints;
}

void push_unique(std::vector<std::string> & target, const std::string & value)
{
  if (std::find(target.begin(), target.end(), value) == target.end()) {
    target.push_back(value);
  }
}

/// @brief Files in @p dir whose name starts with @p prefix. Never throws.
std::vector<std::string> scan_for_prefix(const std::string & dir, const std::string & prefix)
{
  std::vector<std::string> found;
  std::error_code ec;
  if (!std::filesystem::is_directory(dir, ec)) {return found;}
  try {
    for (const auto & entry : std::filesystem::directory_iterator(
        dir, std::filesystem::directory_options::skip_permission_denied))
    {
      const std::string name = entry.path().filename().string();
      if (name.rfind(prefix, 0) == 0) {found.push_back(entry.path().string());}
    }
  } catch (const std::filesystem::filesystem_error &) {
    // Unreadable directory: report what was found, never fabricate success.
  }
  std::sort(found.begin(), found.end());
  return found;
}

int64_t now_ns()
{
  using namespace std::chrono;
  return duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
}
}  // namespace

std::string MvsSdkProbe::summary() const
{
  std::ostringstream os;
  os << "MVS SDK probe:";
  os << "\n  shared library : ";
  if (library_found()) {
    for (const auto & p : lib_paths) {os << p << " ";}
  } else {
    os << "NOT FOUND";
  }
  os << "\n  headers        : ";
  if (headers_found()) {
    for (const auto & p : header_paths) {os << p << " ";}
  } else {
    os << "NOT FOUND";
  }
  os << "\n  env hints      : ";
  if (env.empty()) {
    os << "none set";
  } else {
    for (const auto & kv : env) {os << kv.first << "=" << kv.second << " ";}
  }
  if (!library_found()) {
    os << "\n  searched libs  : ";
    for (const auto & d : searched_lib_dirs) {os << d << "/" << kLibPrefix << "* ";}
  }
  if (!headers_found()) {
    os << "\n  searched hdrs  : ";
    for (const auto & p : searched_header_paths) {os << p << " ";}
  }
  return os.str();
}

MvsSdkProbe probe_sdk(const std::vector<std::string> & extra_lib_candidates)
{
  MvsSdkProbe probe;
  probe.searched_lib_dirs = lib_dirs();
  probe.searched_header_paths = header_candidates();

  for (const auto & dir : lib_dirs()) {
    for (const auto & path : scan_for_prefix(dir, kLibPrefix)) {push_unique(probe.lib_paths, path);}
  }
  std::error_code ec;
  for (const auto & candidate : extra_lib_candidates) {
    if (!candidate.empty() && std::filesystem::exists(candidate, ec)) {
      push_unique(probe.lib_paths, candidate);
    }
  }
  for (const auto & path : header_candidates()) {
    if (std::filesystem::is_regular_file(path, ec)) {push_unique(probe.header_paths, path);}
  }
  for (const auto & name : env_hints()) {
    const char * value = std::getenv(name.c_str());
    if (value != nullptr) {probe.env.emplace_back(name, std::string(value));}
  }
  return probe;
}

#ifdef HIKROBOT_MVS_AVAILABLE
namespace
{
/// @brief Function table resolved from libMvCameraControl.so at runtime.
struct MvsApi
{
  void * lib{nullptr};
  int (*enum_devices)(unsigned int, MV_CC_DEVICE_INFO_LIST *){nullptr};
  int (*create_handle)(void **, const MV_CC_DEVICE_INFO *){nullptr};
  int (*open_device)(void *, unsigned int, unsigned short){nullptr};
  int (*close_device)(void *){nullptr};
  int (*destroy_handle)(void *){nullptr};
  int (*start_grabbing)(void *){nullptr};
  int (*stop_grabbing)(void *){nullptr};
  int (*get_one_frame_timeout)(
    void *, unsigned char *, unsigned int, MV_FRAME_OUT_INFO_EX *, unsigned int){nullptr};
  int (*convert_pixel_type)(void *, MV_CC_PIXEL_CONVERT_PARAM_EX *){nullptr};
  int (*set_enum_value)(void *, const char *, unsigned int){nullptr};
  int (*set_int_value_ex)(void *, const char *, int64_t){nullptr};
  int (*get_int_value_ex)(void *, const char *, MVCC_INTVALUE_EX *){nullptr};
  int (*get_optimal_packet_size)(void *){nullptr};

  bool load(const std::string & path, std::string & error)
  {
    lib = ::dlopen(path.c_str(), RTLD_NOW | RTLD_GLOBAL);
    if (lib == nullptr) {
      error = std::string("dlopen(") + path + ") failed: " + ::dlerror();
      return false;
    }
    bool ok = true;
    auto need = [&](auto & fn, const char * name) {
        fn = reinterpret_cast<std::remove_reference_t<decltype(fn)>>(::dlsym(lib, name));
        if (fn == nullptr) {
          error += std::string(error.empty() ? "" : ", ") + "missing symbol " + name;
          ok = false;
        }
      };
    need(enum_devices, "MV_CC_EnumDevices");
    need(create_handle, "MV_CC_CreateHandle");
    need(open_device, "MV_CC_OpenDevice");
    need(close_device, "MV_CC_CloseDevice");
    need(destroy_handle, "MV_CC_DestroyHandle");
    need(start_grabbing, "MV_CC_StartGrabbing");
    need(stop_grabbing, "MV_CC_StopGrabbing");
    need(get_one_frame_timeout, "MV_CC_GetOneFrameTimeout");
    need(convert_pixel_type, "MV_CC_ConvertPixelTypeEx");
    need(set_enum_value, "MV_CC_SetEnumValue");
    need(set_int_value_ex, "MV_CC_SetIntValueEx");
    need(get_int_value_ex, "MV_CC_GetIntValueEx");
    need(get_optimal_packet_size, "MV_CC_GetOptimalPacketSize");
    if (!ok) {
      ::dlclose(lib);
      lib = nullptr;
    }
    return ok;
  }
};

std::string mvs_error_text(int code)
{
  std::ostringstream os;
  os << "0x" << std::hex << static_cast<unsigned int>(code);
  switch (static_cast<unsigned int>(code)) {
    case MV_OK: os << " (MV_OK)"; break;
    case MV_E_NODATA: os << " (MV_E_NODATA: timeout, no data)"; break;
    case MV_E_PARAMETER: os << " (MV_E_PARAMETER)"; break;
    case MV_E_PARAMETER_RANGE: os << " (MV_E_PARAMETER_RANGE)"; break;
    case MV_E_ACCESS_DENIED: os << " (MV_E_ACCESS_DENIED: no permission)"; break;
    default: break;
  }
  return os.str();
}

std::mutex g_api_mutex;

MvsApi * ensure_api(const std::vector<std::string> & candidates, std::string & error)
{
  static MvsApi api;
  static bool loaded = false;
  std::lock_guard<std::mutex> lock(g_api_mutex);
  if (loaded) {
    if (api.lib == nullptr) {error = "SDK previously failed to load"; return nullptr;}
    return &api;
  }
  // Prefer the unversioned symlink if present, then whatever the probe found.
  std::vector<std::string> paths = candidates;
  if (paths.empty()) {
    for (const auto & p : probe_sdk().lib_paths) {paths.push_back(p);}
  }
  std::sort(
    paths.begin(), paths.end(), [](const std::string & a, const std::string & b) {
      return a.size() < b.size();
    });
  for (const auto & path : paths) {
    std::string err;
    if (api.load(path, err)) {
      loaded = true;
      return &api;
    }
    error += (error.empty() ? "" : "; ") + err;
  }
  loaded = true;
  return nullptr;
}

void set_env_if_unset(const char * key, const std::string & value)
{
  if (std::getenv(key) == nullptr) {::setenv(key, value.c_str(), 0);}
}

std::string format_ip(unsigned int ip)
{
  std::ostringstream os;
  os << ((ip >> 24) & 0xFF) << "." << ((ip >> 16) & 0xFF) << "."
     << ((ip >> 8) & 0xFF) << "." << (ip & 0xFF);
  return os.str();
}

std::string normalise_serial(const unsigned char * raw, std::size_t max_len)
{
  std::size_t len = 0;
  while (len < max_len && raw[len] != 0) {++len;}
  return std::string(reinterpret_cast<const char *>(raw), len);
}
}  // namespace
#endif  // HIKROBOT_MVS_AVAILABLE

MvsBackend::MvsBackend(MvsBackendOptions options)
: options_(std::move(options))
{
}

MvsBackend::~MvsBackend()
{
  // open() may fail after CreateHandle/OpenDevice; the backend object is then
  // destroyed by the node without an explicit close(). Release any partial
  // state here so a retry cannot leak device handles.
  close();
}

const MvsSdkProbe & MvsBackend::probe() const
{
  if (!probe_.has_value()) {probe_ = probe_sdk(options_.sdk_lib_candidates);}
  return *probe_;
}

std::string MvsBackend::describe() const
{
  const auto & report = probe();
  std::ostringstream os;
  os << "backend=mvs requires_hardware=true implemented=";
#ifdef HIKROBOT_MVS_AVAILABLE
  os << "true";
#else
  os << "false";
#endif
  os << " sdk_library_found=" << (report.library_found() ? "true" : "false")
     << " sdk_headers_found=" << (report.headers_found() ? "true" : "false");
  if (!device_summary_.empty()) {os << " " << device_summary_;}
  return os.str();
}

void MvsBackend::open()
{
  if (handle_ != nullptr) {close();}
  const auto & report = probe();
  if (!report.library_found()) {
    throw BackendUnavailableError(
            std::string("Hikrobot MVS SDK not found. Cannot open an MV-CS050-10GM camera.\n") +
            report.summary() +
            "\nACTION: install the *Linux* MVS SDK (x86_64) and re-run.");
  }
  if (!report.headers_found()) {
    throw BackendUnavailableError(
            std::string("MVS shared library found but MvCameraControl.h is missing, so the "
                        "wrapper cannot work against the real API.\n") + report.summary());
  }

#ifdef HIKROBOT_MVS_AVAILABLE
  // The transport-layer plugins (.cti) are located through these variables.
  // Only set them when the user has not, so an explicit configuration wins.
  const std::string sdk_root = std::filesystem::path(report.header_paths.front()).parent_path().parent_path().string();
  set_env_if_unset("MVCAM_SDK_PATH", sdk_root);
  set_env_if_unset("MVCAM_COMMON_RUNENV", sdk_root + "/lib");
  set_env_if_unset("MVCAM_SOFTWARE_LIBENV", sdk_root + "/lib");
  set_env_if_unset("MVCAM_GENICAM_CLPROTOCOL", sdk_root + "/lib/CLProtocol");

  std::string load_error;
  MvsApi * api = ensure_api(options_.sdk_lib_candidates, load_error);
  if (api == nullptr) {
    throw BackendUnavailableError("failed to load libMvCameraControl: " + load_error);
  }

  MV_CC_DEVICE_INFO_LIST devices{};
  int rc = api->enum_devices(MV_GIGE_DEVICE | MV_USB_DEVICE, &devices);
  if (rc != MV_OK) {
    throw std::runtime_error("MV_CC_EnumDevices failed: " + mvs_error_text(rc));
  }
  // Record what the enumeration actually returned: model, serial and IP are
  // exactly what a human needs in order to fix the network configuration.
  {
    std::ostringstream devs;
    devs << "devices=" << devices.nDeviceNum;
    for (unsigned int i = 0; i < devices.nDeviceNum; ++i) {
      const MV_CC_DEVICE_INFO * info = devices.pDeviceInfo[i];
      if (info == nullptr) {continue;}
      if ((info->nTLayerType & MV_GIGE_DEVICE) != 0) {
        const auto & gige = info->SpecialInfo.stGigEInfo;
        devs << " [GigE model='" << normalise_serial(gige.chModelName, 32)
             << "' serial='" << normalise_serial(gige.chSerialNumber, 16)
             << "' ip=" << format_ip(gige.nCurrentIp)
             << " mask=" << format_ip(gige.nCurrentSubNetMask) << "]";
      } else {
        devs << " [non-GigE type=0x" << std::hex << info->nTLayerType << std::dec << "]";
      }
    }
    device_summary_ = devs.str();
  }

  if (devices.nDeviceNum == 0) {
    throw BackendUnavailableError(
            std::string("no Hikvision camera found by the MVS SDK (") +
            device_summary_ + "). Checklist:\n"
            "  1. camera powered? (LED on)\n"
            "  2. Ethernet cable from camera to the host/bridged adapter?\n"
            "  3. is the camera on the same L2 segment as this interface?\n"
            "  4. run the SDK's set_rp_filter.sh --mode=2 on the host for cross-subnet discovery");
  }

  const MV_CC_DEVICE_INFO * chosen = nullptr;
  unsigned int gige_count = 0;
  for (unsigned int i = 0; i < devices.nDeviceNum; ++i) {
    const MV_CC_DEVICE_INFO * info = devices.pDeviceInfo[i];
    if (info == nullptr || (info->nTLayerType & MV_GIGE_DEVICE) == 0) {continue;}
    ++gige_count;
    const std::string serial = normalise_serial(info->SpecialInfo.stGigEInfo.chSerialNumber, 16);
    if (!options_.serial_number.empty() && serial == options_.serial_number) {
      chosen = info;
    } else if (options_.serial_number.empty() && chosen == nullptr) {
      chosen = info;
    }
  }
  if (gige_count == 0) {
    throw BackendUnavailableError(
            "no GigE camera found by the MVS SDK (" + device_summary_ +
            "). Check the camera power, cable and network interface.");
  }
  if (options_.serial_number.empty() && gige_count > 1) {
    throw BackendUnavailableError(
            "multiple GigE cameras found (" + device_summary_ +
            "). Set mvs_serial_number to select exactly one camera.");
  }
  if (chosen == nullptr) {
    throw BackendUnavailableError(
            "no camera matched serial '" + options_.serial_number + "' (" +
            device_summary_ + ")");
  }

  rc = api->create_handle(&handle_, chosen);
  if (rc != MV_OK) {throw std::runtime_error("MV_CC_CreateHandle failed: " + mvs_error_text(rc));}
  rc = api->open_device(handle_, MV_ACCESS_Exclusive, 0);
  if (rc != MV_OK) {
    throw std::runtime_error(
            "MV_CC_OpenDevice failed: " + mvs_error_text(rc) +
            " (is another application, e.g. the MVS client, holding the camera?)");
  }

  // --- optional settings. A rejection is reported, not silently ignored -----
  auto warn_if = [](int code, const char * what) {
      if (code != MV_OK) {
        std::ostringstream os;
        os << "MVS setting ignored: " << what << " -> " << mvs_error_text(code);
        throw std::runtime_error(os.str());
      }
    };

  if (!options_.pixel_format.empty()) {
    // MV_CC_SetEnumValue takes the numeric enum value, so the PixelType.h
    // constants are used directly instead of a string lookup.
    if (options_.pixel_format == "Mono8") {
      warn_if(api->set_enum_value(handle_, "PixelFormat", PixelType_Gvsp_Mono8), "PixelFormat=Mono8");
    } else if (options_.pixel_format == "BGR8") {
      warn_if(api->set_enum_value(handle_, "PixelFormat", PixelType_Gvsp_BGR8_Packed), "PixelFormat=BGR8");
    } else if (options_.pixel_format == "RGB8") {
      warn_if(api->set_enum_value(handle_, "PixelFormat", PixelType_Gvsp_RGB8_Packed), "PixelFormat=RGB8");
    } else {
      throw std::runtime_error(
              "unsupported mvs_pixel_format '" + options_.pixel_format +
              "'; supported: Mono8, BGR8, RGB8");
    }
  }
  if (options_.width > 0) {
    warn_if(api->set_int_value_ex(handle_, "Width", options_.width), "Width");
  }
  if (options_.height > 0) {
    warn_if(api->set_int_value_ex(handle_, "Height", options_.height), "Height");
  }
  if (options_.exposure_us > 0.0) {
    warn_if(
      api->set_enum_value(handle_, "ExposureAuto", 0), "ExposureAuto=Off");
    warn_if(
      api->set_int_value_ex(handle_, "ExposureTime", static_cast<int64_t>(options_.exposure_us)),
      "ExposureTime");
  }
  if (options_.gain > 0.0) {
    warn_if(api->set_enum_value(handle_, "GainAuto", 0), "GainAuto=Off");
    warn_if(
      api->set_int_value_ex(handle_, "Gain", static_cast<int64_t>(options_.gain)), "Gain");
  }

  // GigE: enlarge the packet size to reduce per-packet overhead / loss.
  if ((chosen->nTLayerType & MV_GIGE_DEVICE) != 0) {
    int packet = options_.packet_size;
    if (packet <= 0) {packet = api->get_optimal_packet_size(handle_);}
    if (packet > 0) {
      (void)api->set_int_value_ex(handle_, "GevSCPSPacketSize", packet);
    }
  }

  MVCC_INTVALUE_EX payload{};
  rc = api->get_int_value_ex(handle_, "PayloadSize", &payload);
  if (rc != MV_OK || payload.nCurValue <= 0) {
    throw std::runtime_error("could not read PayloadSize: " + mvs_error_text(rc));
  }
  buffer_.assign(static_cast<std::size_t>(payload.nCurValue), 0);

  rc = api->start_grabbing(handle_);
  if (rc != MV_OK) {throw std::runtime_error("MV_CC_StartGrabbing failed: " + mvs_error_text(rc));}
  return;
#else
  throw std::logic_error(
          "The MVS SDK is installed, but this build was compiled WITHOUT its headers, so the "
          "acquisition path is not available. Rebuild with -DHIKROBOT_MVS_INCLUDE_DIR pointing "
          "at the SDK's include directory (default /opt/MVS/include).");
#endif
}

void MvsBackend::close()
{
#ifdef HIKROBOT_MVS_AVAILABLE
  std::string err;
  MvsApi * api = ensure_api(options_.sdk_lib_candidates, err);
  if (api != nullptr && handle_ != nullptr) {
    (void)api->stop_grabbing(handle_);
    (void)api->close_device(handle_);
    (void)api->destroy_handle(handle_);
  }
#endif
  handle_ = nullptr;
  buffer_.clear();
}

std::optional<Frame> MvsBackend::grab(double timeout_s)
{
#ifdef HIKROBOT_MVS_AVAILABLE
  std::string err;
  MvsApi * api = ensure_api(options_.sdk_lib_candidates, err);
  if (api == nullptr || handle_ == nullptr) {
    throw std::runtime_error("MVS backend used before open()");
  }
  if (buffer_.empty()) {throw std::runtime_error("MVS backend: no payload buffer");}

  MV_FRAME_OUT_INFO_EX info{};
  const unsigned int timeout_ms =
    static_cast<unsigned int>(std::max(1.0, timeout_s * 1000.0));
  const int rc = api->get_one_frame_timeout(
    handle_, buffer_.data(), static_cast<unsigned int>(buffer_.size()), &info, timeout_ms);
  if (static_cast<unsigned int>(rc) == MV_E_NODATA) {return std::nullopt;}  // timeout: no frame
  if (rc != MV_OK) {
    throw std::runtime_error("MV_CC_GetOneFrameTimeout failed: " + mvs_error_text(rc));
  }

  Frame frame;
  frame.width = static_cast<int>(info.nWidth);
  frame.height = static_cast<int>(info.nHeight);
  frame.sequence = info.nFrameNum;

  // nHostTimeStamp is a host-clock value. Sanity-check that it looks like a
  // nanosecond epoch (2026 is ~1.79e18 ns); otherwise fall back to our own clock
  // rather than publishing a nonsense stamp.
  frame.stamp_ns = (info.nHostTimeStamp > 1000000000000000000LL) ?
    info.nHostTimeStamp : now_ns();

  // The device timestamp's epoch is not documented, so it is deliberately left
  // empty instead of being published as if it were a wall-clock time.
  frame.hardware_stamp_ns = std::nullopt;

  const std::size_t src_len = std::min<std::size_t>(info.nFrameLen, buffer_.size());

  if (info.enPixelType == PixelType_Gvsp_Mono8) {
    frame.encoding = "mono8";
    frame.data.assign(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(src_len));
  } else if (info.enPixelType == PixelType_Gvsp_BGR8_Packed) {
    frame.encoding = "bgr8";
    frame.data.assign(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(src_len));
  } else if (info.enPixelType == PixelType_Gvsp_RGB8_Packed) {
    frame.encoding = "rgb8";
    frame.data.assign(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(src_len));
  } else {
    // Anything else (Bayer, YUV, packed mono...) is converted to mono8 by the
    // SDK so downstream nodes never see an encoding they cannot handle.
    std::vector<uint8_t> converted(static_cast<std::size_t>(frame.width) * frame.height);
    MV_CC_PIXEL_CONVERT_PARAM_EX cvt{};
    cvt.nWidth = static_cast<unsigned int>(frame.width);
    cvt.nHeight = static_cast<unsigned int>(frame.height);
    cvt.enSrcPixelType = info.enPixelType;
    cvt.pSrcData = buffer_.data();
    cvt.nSrcDataLen = static_cast<unsigned int>(src_len);
    cvt.enDstPixelType = PixelType_Gvsp_Mono8;
    cvt.pDstBuffer = converted.data();
    cvt.nDstBufferSize = static_cast<unsigned int>(converted.size());
    const int crc = api->convert_pixel_type(handle_, &cvt);
    if (crc != MV_OK) {
      throw std::runtime_error("MV_CC_ConvertPixelTypeEx failed: " + mvs_error_text(crc));
    }
    converted.resize(cvt.nDstLen);
    frame.encoding = "mono8";
    frame.data = std::move(converted);
  }

  if (!frame.is_well_formed()) {
    std::ostringstream os;
    os << "MVS produced a malformed frame: " << frame.width << "x" << frame.height << " "
       << frame.encoding << " data=" << frame.data.size()
       << " expected=" << frame.expected_size();
    throw std::runtime_error(os.str());
  }
  return frame;
#else
  (void)timeout_s;
  open();  // throws with the "built without headers" explanation
  return std::nullopt;
#endif
}

}  // namespace hikrobot_camera_ros2
