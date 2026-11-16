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
// package builds and SDK diagnostics still work without a hard vendor-link
// dependency, while real capture is enabled only when the SDK is installed.
//
// Every symbol below was read from the SDK's own headers
// (/opt/MVS/include/MvCameraControl.h, CameraParams.h, MvErrorDefine.h,
// PixelType.h). Nothing here is guessed.

#include "camera_driver/mvs_backend.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <filesystem>
#include <limits>
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

namespace camera_driver
{
namespace
{
void push_unique(std::vector<std::string> & target, const std::string & value)
{
  if (!value.empty() && std::find(target.begin(), target.end(), value) == target.end()) {
    target.push_back(value);//如果 value 非空、且 target 里还没有它，就追加到末尾
  }
}

std::vector<std::string> sdk_env_roots()//读 3 个环境变量，返回它们的值
{
  std::vector<std::string> roots;//声明一个空 vector，用来装"用户告诉我们的 SDK 根目录"
  for (const char * name : {"MVCAM_SDK_PATH", "MVS_SDK_ROOT", "HIKROBOT_MVS_ROOT"}) {
//这三个名字是海康 MVS SDK 在不同系统上使用的环境变量名
    const char * value = std::getenv(name);
    if (value != nullptr) {push_unique(roots, value);}
  }//查看这三个环境变量里哪些设置了，把它们的值收集成一个列表返回
  return roots;
}

const std::vector<std::string> & lib_dirs()//lib_dirs()：.so 动态库可能在哪些目录里
{
  static const std::vector<std::string> kDirs = []() {
      std::vector<std::string> dirs;
      auto add_root = [&dirs](const std::filesystem::path & root) {//给定一个"SDK 根目录"，把该 SDK 可能的 lib 子目录都加进 dirs
          push_unique(dirs, (root / "lib" / "64").string());
          push_unique(dirs, (root / "lib" / "x86_64").string());
          push_unique(dirs, (root / "lib" / "aarch64").string());
          push_unique(dirs, (root / "lib" / "32").string());
          push_unique(dirs, (root / "lib").string());
        };
      for (const auto & root : sdk_env_roots()) {add_root(root);}
      const char * common_runenv = std::getenv("MVCAM_COMMON_RUNENV");//是海康 SDK 的一个特殊环境变量，它通常直接指向库目录本身
      if (common_runenv != nullptr) {
        push_unique(dirs, common_runenv);//把环境变量的值直接加入
        push_unique(dirs, (std::filesystem::path(common_runenv) / "64").string());//再试一下它下面的 64 子目录
      }
//CMake 配置的路径
#ifdef HIKROBOT_MVS_CONFIGURED_INCLUDE_DIR
      const auto configured_include = std::filesystem::path(HIKROBOT_MVS_CONFIGURED_INCLUDE_DIR);
      add_root(configured_include.parent_path().parent_path());
#endif
      push_unique(dirs, "/opt/MVS/lib/64");
      push_unique(dirs, "/opt/MVS/lib/32");
      push_unique(dirs, "/opt/MVS/lib/aarch64");
      push_unique(dirs, "/opt/MVS/lib/arm");
      push_unique(dirs, "/usr/lib");
      push_unique(dirs, "/usr/lib/x86_64-linux-gnu");
      push_unique(dirs, "/usr/local/lib");
      return dirs;
    }();
  return kDirs;
}

constexpr const char * kLibPrefix = "libMvCameraControl.so";

//kLibPrefix  库文件名前缀常量，用于扫目录
const std::vector<std::string> & header_candidates()//收集所有可能的头文件路径
{
  static const std::vector<std::string> kPaths = []() {
      std::vector<std::string> paths;
      auto add_root = [&paths](const std::filesystem::path & root) {
          push_unique(paths, (root / "include" / "MvCameraControl.h").string());
          push_unique(paths, (root / "include" / "x86_64" / "MvCameraControl.h").string());
          push_unique(paths, (root / "include" / "aarch64" / "MvCameraControl.h").string());
        };
      for (const auto & root : sdk_env_roots()) {add_root(root);}
#ifdef HIKROBOT_MVS_CONFIGURED_INCLUDE_DIR
      push_unique(
        paths,
        (std::filesystem::path(HIKROBOT_MVS_CONFIGURED_INCLUDE_DIR) / "MvCameraControl.h").string());
#endif
      push_unique(paths, "/opt/MVS/include/MvCameraControl.h");
      push_unique(paths, "/opt/MVS/include/x86_64/MvCameraControl.h");
      push_unique(paths, "/opt/MVS/include/aarch64/MvCameraControl.h");
      push_unique(paths, "/usr/include/MvCameraControl.h");
      push_unique(paths, "/usr/local/include/MvCameraControl.h");
      return paths;
    }();
  return kPaths;
}

const std::vector<std::string> & env_hints()//env_hints()：哪些环境变量会影响 SDK 查找（只用于报告）
{
  //列出需要报告的环境变量名（不读值）
  static const std::vector<std::string> kHints{
    "MVCAM_SDK_PATH", "MVS_SDK_ROOT", "HIKROBOT_MVS_ROOT",
    "MVCAM_COMMON_RUNENV", "MVCAM_GENICAM_CLPROTOCOL"};
  return kHints;
}

/// @brief Files in @p dir whose name starts with @p prefix. Never throws.
std::vector<std::string> scan_for_prefix(const std::string & dir, const std::string & prefix)
{
  //扫目录找文件  dir：要扫的目录  prefix：文件名前缀
  std::vector<std::string> found;
  std::error_code ec;
  if (!std::filesystem::is_directory(dir, ec)) {return found;}
  try {
    for (const auto & entry : std::filesystem::directory_iterator(
        dir, std::filesystem::directory_options::skip_permission_denied))//遇到没权限的子目录，跳过不报错
    {
      const std::string name = entry.path().filename().string();
//entry.path()：该项的完整路径  .filename()：取文件名部分
      if (name.rfind(prefix, 0) == 0) {found.push_back(entry.path().string());}//前缀匹配
    }
  } catch (const std::filesystem::filesystem_error &) {
    // Unreadable directory: report what was found, never fabricate success.
  }
  std::sort(found.begin(), found.end());//排序
  return found;//lib_dirs()返回一堆目录-> scan_for_prefix(每个目录, "libMvCameraControl.so")->找到实际的 .so 文件路径
}

int64_t now_ns()//取当前时间纳秒
{
  using namespace std::chrono;
  return duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
}

//算图像字节数，带溢出检查
std::size_t checked_image_bytes(int width, int height, std::size_t channels)
{
  if (width <= 0 || height <= 0) {
    throw std::runtime_error(
            "MVS returned invalid image dimensions: " + std::to_string(width) + "x" +
            std::to_string(height));
  }
//检查通道数
  if (channels == 0U) {throw std::invalid_argument("image channel count must be positive");}

  const std::size_t pixels =//算像素数
    static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  if (pixels > std::numeric_limits<std::size_t>::max() / channels) {
    throw std::overflow_error("image byte count overflows size_t");
  }
  const std::size_t bytes = pixels * channels;
  if (bytes > std::numeric_limits<unsigned int>::max()) {
    throw std::overflow_error(
            "image byte count exceeds MVS API unsigned int limit: " + std::to_string(bytes));
  }//检查是否超过 MVS API 的限制
//为什么要检查 unsigned int 上限？
//因为后面 grab() 里调用 SDK 的 get_one_frame_timeout 时，缓冲区大小参数是 unsigned int：
  return bytes;
}

std::string lower_value(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return value;
}

void require_finite_positive(double value, const char * name)
{
  if (!std::isfinite(value) || !(value > 0.0)) {
    throw std::invalid_argument(std::string(name) + " must be finite and > 0");
  }
}

void require_finite(double value, const char * name)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument(std::string(name) + " must be finite");
  }
}
}  // namespace

void validate_mvs_options(const MvsBackendOptions & options)
{
  const std::string transport = lower_value(options.transport);
  if (transport != "auto" && transport != "gige" && transport != "usb") {
    throw std::invalid_argument(
            "mvs_transport must be auto, usb, or gige");
  }

  if (options.width < 0 || options.height < 0) {
    throw std::invalid_argument("mvs_width and mvs_height must be >= 0");
  }

  const std::string exposure_mode = lower_value(options.exposure_mode);
  if (exposure_mode != "auto" &&
    exposure_mode != "manual" &&
    exposure_mode != "camera")
  {
    throw std::invalid_argument(
            "exposure_mode must be 'auto', 'manual', or 'camera'");
  }
  const std::string exposure_auto_mode = lower_value(options.exposure_auto_mode);
  if (exposure_auto_mode != "continuous" && exposure_auto_mode != "once") {
    throw std::invalid_argument(
            "exposure_auto_mode must be 'continuous' or 'once'");
  }
  require_finite_positive(options.exposure_time_us, "exposure_time_us");

  const std::string gain_mode = lower_value(options.gain_mode);
  if (gain_mode != "auto" &&
    gain_mode != "manual" &&
    gain_mode != "camera")
  {
    throw std::invalid_argument(
            "gain_mode must be 'auto', 'manual', or 'camera'");
  }
  const std::string gain_auto_mode = lower_value(options.gain_auto_mode);
  if (gain_auto_mode != "continuous" && gain_auto_mode != "once") {
    throw std::invalid_argument("gain_auto_mode must be 'continuous' or 'once'");
  }
  require_finite(options.gain_value, "gain_value");

  const std::string output_encoding = lower_value(options.output_encoding);
  if (output_encoding != "auto" &&
    output_encoding != "bgr8" &&
    output_encoding != "mono8")
  {
    throw std::invalid_argument(
            "image_encoding must be 'auto', 'bgr8', or 'mono8'");
  }

  const std::string pixel_format = lower_value(options.pixel_format);
  if (!pixel_format.empty() &&
    pixel_format != "mono8" &&
    pixel_format != "bgr8" &&
    pixel_format != "rgb8" &&
    pixel_format != "bayerrg8" &&
    pixel_format != "bayerbg8" &&
    pixel_format != "bayergb8" &&
    pixel_format != "bayergr8")
  {
    throw std::invalid_argument(
            "mvs_pixel_format must be empty, BGR8, RGB8, BayerRG8, BayerBG8, "
            "BayerGB8, BayerGR8, or Mono8");
  }
  if (pixel_format == "mono8" && output_encoding == "bgr8") {
    throw std::invalid_argument(
            "mvs_pixel_format=Mono8 conflicts with explicit image_encoding=bgr8; "
            "use image_encoding=auto or mono8 for monochrome cameras");
  }
}

std::vector<uint8_t> rgb8_to_bgr8(
  const std::vector<uint8_t> & rgb, int width, int height)
{
  const std::size_t expected = checked_image_bytes(width, height, 3U);
  if (rgb.size() != expected) {
    throw std::invalid_argument(
            "rgb8_to_bgr8 size mismatch: got " + std::to_string(rgb.size()) +
            " expected " + std::to_string(expected));
  }

  std::vector<uint8_t> bgr(expected);
  for (std::size_t i = 0; i < expected; i += 3U) {
    bgr[i] = rgb[i + 2U];
    bgr[i + 1U] = rgb[i + 1U];
    bgr[i + 2U] = rgb[i];
  }
  return bgr;
}

std::string MvsSdkProbe::summary() const//记录了"找到了啥"和"搜了哪些地方"
{
  std::ostringstream os;
  os << "MVS SDK probe:";//往缓冲区写标题
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

MvsSdkProbe probe_sdk(const std::vector<std::string> & extra_lib_candidates)//执行探测
{
  //扫描所有可能的位置，收集：找到的 .so、找到的头文件、设了的环境变量，全部打包进 MvsSdkProbe 返回
  MvsSdkProbe probe;
  probe.searched_lib_dirs = lib_dirs();
  probe.searched_header_paths = header_candidates();

  std::error_code ec;
  for (const auto & candidate : extra_lib_candidates) {
    if (!candidate.empty() && std::filesystem::exists(candidate, ec)) {
      push_unique(probe.lib_paths, candidate);
    }
  }
  for (const auto & dir : lib_dirs()) {//扫描所有目录找 .so
    for (const auto & path : scan_for_prefix(dir, kLibPrefix)) {push_unique(probe.lib_paths, path);}
  }
  for (const auto & path : header_candidates()) {//检查头文件是否存在
    if (std::filesystem::is_regular_file(path, ec)) {push_unique(probe.header_paths, path);}
  }
  for (const auto & name : env_hints()) {//记录设了哪些环境变量
    const char * value = std::getenv(name.c_str());
    if (value != nullptr) {probe.env.emplace_back(name, std::string(value));}
  }
  return probe;
}

#ifdef HIKROBOT_MVS_AVAILABLE
namespace
{
/// @brief Function table resolved from libMvCameraControl.so at runtime.
//一个函数指针表：每个成员对应厂商 SDK 的一个函数，加上一个 lib 句柄。所有函数指针初始为 nullptr，load 成功后才填上真实地址
struct MvsApi
{
  void * lib{nullptr};
  int (*initialize)(){nullptr};
  int (*finalize)(){nullptr};
  unsigned int (*get_sdk_version)(){nullptr};
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
  int (*get_enum_value)(void *, const char *, MVCC_ENUMVALUE *){nullptr};
  int (*set_float_value)(void *, const char *, float){nullptr};
  int (*get_float_value)(void *, const char *, MVCC_FLOATVALUE *){nullptr};
  int (*set_int_value_ex)(void *, const char *, int64_t){nullptr};
  int (*get_int_value_ex)(void *, const char *, MVCC_INTVALUE_EX *){nullptr};
  int (*get_optimal_packet_size)(void *){nullptr};

  bool load(const std::string & path, std::string & error)
  {
    *this = MvsApi{};
//用 dlopen 打开 path 指向的 .so，加载时立即解析所有符号，并让符号对后续加载的库可见
    lib = ::dlopen(path.c_str(), RTLD_NOW | RTLD_GLOBAL);//用于运行时打开动态库 RTLD_NOW：立即解析所有符号
    if (lib == nullptr) {
      error = std::string("dlopen(") + path + ") failed: " + ::dlerror();
      return false;
    }
    bool ok = true;
//need：一个 lambda，作用是"按名字查函数地址并赋给 fn，查不到就记录错误"
//[&]：捕获所有外部变量的引用（lib、error、ok）
    auto need = [&](auto & fn, const char * name) {
        fn = reinterpret_cast<std::remove_reference_t<decltype(fn)>>(::dlsym(lib, name));
//用 dlsym 查 name 的地址（返回 void *），强转成 fn 原本的类型（去掉引用），赋给 fn
        if (fn == nullptr) {
          error += std::string(error.empty() ? "" : ", ") + "missing symbol " + name;
          ok = false;
        }
      };
//逐个查 16 个函数
    need(initialize, "MV_CC_Initialize");
    need(finalize, "MV_CC_Finalize");
    need(get_sdk_version, "MV_CC_GetSDKVersion");
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
    need(get_enum_value, "MV_CC_GetEnumValue");
    need(set_float_value, "MV_CC_SetFloatValue");
    need(get_float_value, "MV_CC_GetFloatValue");
    need(set_int_value_ex, "MV_CC_SetIntValueEx");
    need(get_int_value_ex, "MV_CC_GetIntValueEx");
    need(get_optimal_packet_size, "MV_CC_GetOptimalPacketSize");
    if (!ok) {
      ::dlclose(lib);
      *this = MvsApi{};//重置
    }
    return ok;
  }
};

std::string mvs_error_text(int code)
{
  std::ostringstream os;
  os << "0x" << std::hex << static_cast<unsigned int>(code);//以十六进制打印错误码
  switch (static_cast<unsigned int>(code)) {
    case MV_OK: os << " (MV_OK)"; break;
    case MV_E_NODATA: os << " (MV_E_NODATA: timeout, no data)"; break;
    case MV_E_PARAMETER: os << " (MV_E_PARAMETER)"; break;//参数错
    case MV_E_PARAMETER_RANGE: os << " (MV_E_PARAMETER_RANGE)"; break;
    case MV_E_ACCESS_DENIED: os << " (MV_E_ACCESS_DENIED: no permission)"; break;
    default: break;
  }
  return os.str();
}

//这四个变量是整份文件的"全局状态"，管理"SDK 到底加载了没有"
std::mutex g_api_mutex;
MvsApi g_api;
bool g_api_loaded = false;
bool g_api_initialized = false;

MvsApi * ensure_api(const std::vector<std::string> & candidates, std::string & error)
{//确保 SDK 已加载并初始化
  std::lock_guard<std::mutex> lock(g_api_mutex);
  if (g_api_loaded) {
    if (g_api.lib == nullptr) {error = "SDK previously failed to load"; return nullptr;}
    return &g_api;
  }
  // Explicit candidates come first. CMake/environment-derived probe paths keep
  // their discovery order, with /opt/MVS only as a fallback.
  std::vector<std::string> paths;//第一次加载——准备候选路径
  for (const auto & candidate : candidates) {push_unique(paths, candidate);}//先加用户显式指定的（candidates 参数）
  for (const auto & path : probe_sdk().lib_paths) {push_unique(paths, path);}//再加自动探测到的（probe_sdk().lib_paths
  for (const auto & path : paths) {//逐个尝试加载
    std::string err;
    if (!g_api.load(path, err)) {
      error += (error.empty() ? "" : "; ") + err;
      continue;
    }
    const int init_rc = g_api.initialize();//加载成功，尝试 initialize
    if (init_rc != MV_OK) {
      error += (error.empty() ? "" : "; ") +
        path + ": MV_CC_Initialize failed: " + mvs_error_text(init_rc);
      ::dlclose(g_api.lib);
      g_api = MvsApi{};
      continue;
    }
    g_api_loaded = true;
    g_api_initialized = true;
    return &g_api;
  }
  g_api_loaded = true;
  return nullptr;
}

void finalize_mvs_sdk_locked()//清理
{
  if (g_api_initialized && g_api.finalize != nullptr) {
    (void)g_api.finalize();
    g_api_initialized = false;
  }
  g_api_loaded = false;
  g_api = MvsApi{};
}

void set_env_if_unset(const char * key, const std::string & value)//设环境变量,用在open() 里
{
  //如果用户没设，根据 SDK 路径自动补上；用户设了，尊重用户的。
  if (std::getenv(key) == nullptr) {::setenv(key, value.c_str(), 0);}
}

std::string format_ip(unsigned int ip)//32 位整数转 IP 字符串,用在open()
{
  std::ostringstream os;
  os << ((ip >> 24) & 0xFF) << "." << ((ip >> 16) & 0xFF) << "."
     << ((ip >> 8) & 0xFF) << "." << (ip & 0xFF);
  return os.str();
}

std::string infer_sdk_root(const std::string & header_path)//从 header 路径反推 SDK 根目录
{
  auto include_dir = std::filesystem::path(header_path).parent_path();//取父目录
  const std::string dir_name = include_dir.filename().string();
  if (dir_name == "x86_64" || dir_name == "aarch64") {//如果父目录是架构名，再往上跳一级
    include_dir = include_dir.parent_path();
  }
  if (include_dir.filename() == "include") {return include_dir.parent_path().string();}
  return include_dir.string();
}//用在open(),用户没设环境变量时，从"找到的头文件路径"自动推 SDK 根，补上环境变量。

std::string normalise_serial(const unsigned char * raw, std::size_t max_len)
{//把厂商 SDK 的定长 unsigned char 数组转成 std::string，正确处理没有终止符的情况
  std::size_t len = 0;
  while (len < max_len && raw[len] != 0) {++len;}//手动找 \0
  return std::string(reinterpret_cast<const char *>(raw), len);
}

bool is_gige_device(const MV_CC_DEVICE_INFO * info)
{
  return info != nullptr && (info->nTLayerType & MV_GIGE_DEVICE) != 0;
}

bool is_usb_device(const MV_CC_DEVICE_INFO * info)
{
  return info != nullptr && (info->nTLayerType & MV_USB_DEVICE) != 0;
}

std::string device_serial(const MV_CC_DEVICE_INFO * info)
{
  if (is_gige_device(info)) {
    return normalise_serial(info->SpecialInfo.stGigEInfo.chSerialNumber, 16);
  }
  if (is_usb_device(info)) {
    return normalise_serial(info->SpecialInfo.stUsb3VInfo.chSerialNumber, 64);
  }
  return {};
}

bool is_bayer_pixel_type(unsigned int pixel_type)
{
  switch (pixel_type) {
    case PixelType_Gvsp_BayerGR8:
    case PixelType_Gvsp_BayerRG8:
    case PixelType_Gvsp_BayerGB8:
    case PixelType_Gvsp_BayerBG8:
    case PixelType_Gvsp_BayerRBGG8:
    case PixelType_Gvsp_BayerBRGG8:
    case PixelType_Gvsp_BayerGR10:
    case PixelType_Gvsp_BayerRG10:
    case PixelType_Gvsp_BayerGB10:
    case PixelType_Gvsp_BayerBG10:
    case PixelType_Gvsp_BayerGR12:
    case PixelType_Gvsp_BayerRG12:
    case PixelType_Gvsp_BayerGB12:
    case PixelType_Gvsp_BayerBG12:
    case PixelType_Gvsp_BayerGR10_Packed:
    case PixelType_Gvsp_BayerRG10_Packed:
    case PixelType_Gvsp_BayerGB10_Packed:
    case PixelType_Gvsp_BayerBG10_Packed:
    case PixelType_Gvsp_BayerGR12_Packed:
    case PixelType_Gvsp_BayerRG12_Packed:
    case PixelType_Gvsp_BayerGB12_Packed:
    case PixelType_Gvsp_BayerBG12_Packed:
    case PixelType_Gvsp_BayerGR16:
    case PixelType_Gvsp_BayerRG16:
    case PixelType_Gvsp_BayerGB16:
    case PixelType_Gvsp_BayerBG16:
      return true;
    default:
      return false;
  }
}

bool enum_contains(const MVCC_ENUMVALUE & value, unsigned int candidate)
{
  const unsigned int count = std::min<unsigned int>(
    value.nSupportedNum, MV_MAX_XML_SYMBOLIC_NUM);
  return std::find(value.nSupportValue, value.nSupportValue + count, candidate) !=
         value.nSupportValue + count;
}

std::string supported_enum_values(const MVCC_ENUMVALUE & value)
{
  const unsigned int count = std::min<unsigned int>(
    value.nSupportedNum, MV_MAX_XML_SYMBOLIC_NUM);
  std::ostringstream os;
  for (unsigned int i = 0; i < count; ++i) {
    if (i != 0U) {os << ",";}
    os << value.nSupportValue[i];
  }
  return os.str();
}

unsigned int exposure_auto_value(const std::string & mode)
{
  return lower_value(mode) == "once" ?
         static_cast<unsigned int>(MV_EXPOSURE_AUTO_MODE_ONCE) :
         static_cast<unsigned int>(MV_EXPOSURE_AUTO_MODE_CONTINUOUS);
}

unsigned int gain_auto_value(const std::string & mode)
{
  return lower_value(mode) == "once" ?
         static_cast<unsigned int>(MV_GAIN_MODE_ONCE) :
         static_cast<unsigned int>(MV_GAIN_MODE_CONTINUOUS);
}

void require_enum_value(
  const MVCC_ENUMVALUE & supported, unsigned int requested, const std::string & feature)
{
  if (!enum_contains(supported, requested)) {
    throw std::runtime_error(
            feature + " value " + std::to_string(requested) +
            " is not supported by this camera; supported values: " +
            supported_enum_values(supported));
  }
}

void require_float_in_range(
  const MVCC_FLOATVALUE & range, double requested, const std::string & feature)
{
  if (requested < static_cast<double>(range.fMin) ||
    requested > static_cast<double>(range.fMax))
  {
    std::ostringstream os;
    os << feature << '=' << requested << " is outside camera range ["
       << range.fMin << ", " << range.fMax << ']';
    throw std::runtime_error(os.str());
  }
}

bool float_matches(double actual, double requested)
{
  const double tolerance = std::max(1.0e-3, std::abs(requested) * 1.0e-4);
  return std::abs(actual - requested) <= tolerance;
}

unsigned int pixel_type_for_name(const std::string & name)
{
  const std::string value = lower_value(name);
  if (value == "mono8") {return PixelType_Gvsp_Mono8;}
  if (value == "bgr8") {return PixelType_Gvsp_BGR8_Packed;}
  if (value == "rgb8") {return PixelType_Gvsp_RGB8_Packed;}
  if (value == "bayerrg8") {return PixelType_Gvsp_BayerRG8;}
  if (value == "bayerbg8") {return PixelType_Gvsp_BayerBG8;}
  if (value == "bayergb8") {return PixelType_Gvsp_BayerGB8;}
  if (value == "bayergr8") {return PixelType_Gvsp_BayerGR8;}
  throw std::invalid_argument(
          "unsupported mvs_pixel_format '" + name +
          "'; use Mono8, BGR8, RGB8, BayerRG8, BayerBG8, BayerGB8, or BayerGR8");
}

std::string pixel_type_name(uint32_t pixel_type)
{
  switch (pixel_type) {
    case PixelType_Gvsp_Mono8: return "Mono8";
    case PixelType_Gvsp_BGR8_Packed: return "BGR8";
    case PixelType_Gvsp_RGB8_Packed: return "RGB8";
    case PixelType_Gvsp_BayerRG8: return "BayerRG8";
    case PixelType_Gvsp_BayerBG8: return "BayerBG8";
    case PixelType_Gvsp_BayerGB8: return "BayerGB8";
    case PixelType_Gvsp_BayerGR8: return "BayerGR8";
    default: {
        std::ostringstream os;
        os << "0x" << std::hex << pixel_type;
        return os.str();
      }
  }
}

void set_enum_checked(
  MvsApi & api, void * handle, const char * feature, unsigned int value)
{
  const int rc = api.set_enum_value(handle, feature, value);
  if (rc != MV_OK) {
    throw std::runtime_error(
            std::string("MV_CC_SetEnumValue(") + feature + ") failed: " +
            mvs_error_text(rc));
  }
}

void set_float_checked(
  MvsApi & api, void * handle, const char * feature, double value)
{
  const int rc = api.set_float_value(handle, feature, static_cast<float>(value));
  if (rc != MV_OK) {
    throw std::runtime_error(
            std::string("MV_CC_SetFloatValue(") + feature + ") failed: " +
            mvs_error_text(rc));
  }
}

MVCC_ENUMVALUE get_enum_checked(MvsApi & api, void * handle, const char * feature)
{
  MVCC_ENUMVALUE value{};
  const int rc = api.get_enum_value(handle, feature, &value);
  if (rc != MV_OK) {
    throw std::runtime_error(
            std::string("MV_CC_GetEnumValue(") + feature + ") failed: " +
            mvs_error_text(rc));
  }
  return value;
}

MVCC_FLOATVALUE get_float_checked(MvsApi & api, void * handle, const char * feature)
{
  MVCC_FLOATVALUE value{};
  const int rc = api.get_float_value(handle, feature, &value);
  if (rc != MV_OK) {
    throw std::runtime_error(
            std::string("MV_CC_GetFloatValue(") + feature + ") failed: " +
            mvs_error_text(rc));
  }
  return value;
}

bool pixel_type_is_color(unsigned int pixel_type)
{
  return pixel_type == PixelType_Gvsp_BGR8_Packed ||
         pixel_type == PixelType_Gvsp_RGB8_Packed ||
         is_bayer_pixel_type(pixel_type);
}

bool pixel_type_is_known(unsigned int pixel_type)
{
  return pixel_type == PixelType_Gvsp_Mono8 || pixel_type_is_color(pixel_type);
}

unsigned int select_pixel_format(
  MvsApi & api, void * handle, const std::string & requested,
  const std::string & output_encoding)
{
  const MVCC_ENUMVALUE supported = get_enum_checked(api, handle, "PixelFormat");
  const std::string encoding = lower_value(output_encoding);

  if (!requested.empty()) {
    const unsigned int wanted = pixel_type_for_name(requested);
    if (encoding == "bgr8" && !pixel_type_is_color(wanted)) {
      throw std::runtime_error(
              "mvs_pixel_format=" + requested +
              " cannot satisfy explicit image_encoding=bgr8");
    }
    require_enum_value(supported, wanted, "PixelFormat");
    set_enum_checked(api, handle, "PixelFormat", wanted);
  } else {
    const std::vector<unsigned int> color_priority{
      PixelType_Gvsp_BGR8_Packed,
      PixelType_Gvsp_RGB8_Packed,
      PixelType_Gvsp_BayerRG8,
      PixelType_Gvsp_BayerBG8,
      PixelType_Gvsp_BayerGB8,
      PixelType_Gvsp_BayerGR8};
    std::vector<unsigned int> priority;
    if (encoding == "mono8") {
      priority.push_back(PixelType_Gvsp_Mono8);
      priority.insert(priority.end(), color_priority.begin(), color_priority.end());
    } else {
      priority = color_priority;
      if (encoding == "auto") {
        priority.push_back(PixelType_Gvsp_Mono8);
      }
    }
    std::string set_errors;
    bool selected = false;
    for (const unsigned int candidate : priority) {
      if (!enum_contains(supported, candidate)) {continue;}
      try {
        set_enum_checked(api, handle, "PixelFormat", candidate);
        selected = true;
        break;
      } catch (const std::exception & exc) {
        set_errors += (set_errors.empty() ? "" : "; ") + std::string(exc.what());
      }
    }
    if (!selected) {
      throw std::runtime_error(
              "camera reports no usable PixelFormat for image_encoding=" +
              encoding + ". Supported enum values: " +
              supported_enum_values(supported) +
              (set_errors.empty() ? "" : "; set errors: " + set_errors));
    }
  }

  const MVCC_ENUMVALUE actual = get_enum_checked(api, handle, "PixelFormat");
  if (!pixel_type_is_known(actual.nCurValue)) {
    throw std::runtime_error(
            "PixelFormat was set to an unsupported value: " +
            pixel_type_name(actual.nCurValue));
  }
  if (encoding == "bgr8" && !pixel_type_is_color(actual.nCurValue)) {
    throw std::runtime_error(
            "PixelFormat is monochrome but image_encoding=bgr8 was explicitly required");
  }
  return actual.nCurValue;
}

std::string output_encoding_for_pixel_type(
  unsigned int pixel_type, const std::string & requested_encoding)
{
  const std::string encoding = lower_value(requested_encoding);
  if (encoding == "mono8") {return "mono8";}
  if (encoding == "bgr8") {return "bgr8";}
  return pixel_type_is_color(pixel_type) ? "bgr8" : "mono8";
}
}  // namespace
#endif  // HIKROBOT_MVS_AVAILABLE

void finalize_mvs_sdk()//外部可调用的清理入口
{
#ifdef HIKROBOT_MVS_AVAILABLE
  std::lock_guard<std::mutex> lock(g_api_mutex);
  finalize_mvs_sdk_locked();
#endif
}

MvsBackend::MvsBackend(MvsBackendOptions options)//构造函数
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

const MvsSdkProbe & MvsBackend::probe() const// 懒加载探测结果 用到才做，做完缓存
{//返回 SDK 探测结果。第一次调用时扫描并缓存，后续调用直接返回缓存（引用，不拷贝）
  if (!probe_.has_value()) {probe_ = probe_sdk(options_.sdk_lib_candidates);}
  return *probe_;
}

std::string mvs_pixel_type_name(uint32_t pixel_type)
{
#ifdef HIKROBOT_MVS_AVAILABLE
  return pixel_type_name(pixel_type);
#else
  std::ostringstream os;
  os << "0x" << std::hex << pixel_type;
  return os.str();
#endif
}

bool mvs_pixel_type_is_color(uint32_t pixel_type)
{
#ifdef HIKROBOT_MVS_AVAILABLE
  return pixel_type == PixelType_Gvsp_BGR8_Packed ||
         pixel_type == PixelType_Gvsp_RGB8_Packed ||
         is_bayer_pixel_type(pixel_type);
#else
  (void)pixel_type;
  return false;
#endif
}

std::string MvsBackend::describe() const
{
  const auto & report = probe();
  std::ostringstream os;
  os << "backend=mvs transport=" << options_.transport << " requires_hardware=true implemented=";
#ifdef HIKROBOT_MVS_AVAILABLE
  os << "true";
#else
  os << "false";
#endif
  os << " sdk_library_found=" << (report.library_found() ? "true" : "false")
     << " sdk_headers_found=" << (report.headers_found() ? "true" : "false");
  if (!sdk_version_.empty()) {os << " sdk_version=" << sdk_version_;}
  if (!camera_info_.model.empty()) {os << " model=" << camera_info_.model;}
  if (!camera_info_.serial_number.empty()) {os << " serial=" << camera_info_.serial_number;}
  if (camera_info_.vendor_id != 0U) {os << " vendor_id=" << camera_info_.vendor_id;}
  if (camera_info_.product_id != 0U) {os << " product_id=" << camera_info_.product_id;}
  if (!camera_info_.pixel_format.empty()) {os << " pixel_format=" << camera_info_.pixel_format;}
  if (!camera_info_.image_encoding.empty()) {os << " image_encoding=" << camera_info_.image_encoding;}
  if (camera_info_.width > 0 && camera_info_.height > 0) {
    os << " resolution=" << camera_info_.width << "x" << camera_info_.height;
  }
  if (!camera_info_.exposure_mode.empty()) {
    os << " exposure_mode=" << camera_info_.exposure_mode
       << " exposure_time_us=" << camera_info_.exposure_time_us;
  }
  if (!camera_info_.gain_mode.empty()) {
    os << " gain_mode=" << camera_info_.gain_mode
       << " gain_value=" << camera_info_.gain_value;
  }
  if (!device_summary_.empty()) {os << " " << device_summary_;}
  return os.str();
}

void MvsBackend::open()//打开相机
{
  if (handle_ != nullptr) {close();}//handle_：保存 SDK 设备句柄的成员变量
  validate_mvs_options(options_);
  const std::string transport = lower_value(options_.transport);
  const auto & report = probe();//probe() 负责检查 MVS SDK 的相关信息，例如 SDK 动态库和头文件是否存在
  if (!report.library_found()) {//library_found()：检查是否找到 SDK 动态库
    throw BackendUnavailableError(
            std::string("Hikrobot MVS SDK not found. Cannot open a Hikrobot camera.\n") +
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
  const std::string sdk_root = infer_sdk_root(report.header_paths.front());
  set_env_if_unset("MVCAM_SDK_PATH", sdk_root);
  set_env_if_unset("MVCAM_COMMON_RUNENV", sdk_root + "/lib");
  set_env_if_unset("MVCAM_SOFTWARE_LIBENV", sdk_root + "/lib");
  set_env_if_unset("MVCAM_GENICAM_CLPROTOCOL", sdk_root + "/lib/CLProtocol");

  std::string load_error;
  MvsApi * api = ensure_api(options_.sdk_lib_candidates, load_error);
  if (api == nullptr) {
    throw BackendUnavailableError("failed to load libMvCameraControl: " + load_error);
  }
  const unsigned int sdk_version = api->get_sdk_version();
  std::ostringstream version;
  version << ((sdk_version >> 24) & 0xFFU) << "." << ((sdk_version >> 16) & 0xFFU)
          << "." << ((sdk_version >> 8) & 0xFFU) << "." << (sdk_version & 0xFFU);
  sdk_version_ = version.str();

  MV_CC_DEVICE_INFO_LIST devices{};
  int rc = api->enum_devices(MV_GIGE_DEVICE | MV_USB_DEVICE, &devices);//enum_devices() 对应海康 MVS SDK 的设备枚举操作
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
      if (is_gige_device(info)) {
        const auto & gige = info->SpecialInfo.stGigEInfo;
        devs << " [GigE model='" << normalise_serial(gige.chModelName, 32)
             << "' serial='" << normalise_serial(gige.chSerialNumber, 16)
             << "' ip=" << format_ip(gige.nCurrentIp)
             << " mask=" << format_ip(gige.nCurrentSubNetMask) << "]";
      } else if (is_usb_device(info)) {
        const auto & usb = info->SpecialInfo.stUsb3VInfo;
        devs << " [USB3 model='" << normalise_serial(usb.chModelName, 64)
             << "' serial='" << normalise_serial(usb.chSerialNumber, 64)
             << "' vid=0x" << std::hex << usb.idVendor
             << " pid=0x" << usb.idProduct << std::dec << "]";
      }
    }
    device_summary_ = devs.str();
  }

  if (devices.nDeviceNum == 0) {
    throw BackendUnavailableError(
            std::string("no Hikvision camera found by the MVS SDK (") +
            device_summary_ + ", sdk_version=" + sdk_version_ + ", transport=" +
            transport + "). Checklist:\n"
            "  1. camera powered? (LED on)\n"
            "  2. GigE: camera cable and host/VM bridge ready?\n"
            "  3. USB3: camera USB interface passed through to this VM?\n"
            "  4. are camera and host on the same L2 segment for GigE discovery?");
  }

  const MV_CC_DEVICE_INFO * chosen = nullptr;
  std::vector<const MV_CC_DEVICE_INFO *> matching;
  for (unsigned int i = 0; i < devices.nDeviceNum; ++i) {
    const MV_CC_DEVICE_INFO * info = devices.pDeviceInfo[i];
    const bool gige = is_gige_device(info);
    const bool usb = is_usb_device(info);
    if (!gige && !usb) {continue;}
    if (transport == "gige" && !gige) {continue;}
    if (transport == "usb" && !usb) {continue;}
    if (!options_.serial_number.empty() && device_serial(info) != options_.serial_number) {continue;}
    matching.push_back(info);
  }
  if (matching.empty()) {
    throw BackendUnavailableError(
            "no MVS camera matched transport='" + transport + "'" +
            (options_.serial_number.empty() ? "" : " serial='" + options_.serial_number + "'") +
            " (" + device_summary_ + ")");
  }
  if (matching.size() != 1U) {
    throw BackendUnavailableError(
            "single-camera mode found multiple matching devices (" + device_summary_ +
            "). Set mvs_serial_number to select one.");
  }
  chosen = matching.front();
  camera_info_ = MvsCameraInfo{};
  camera_info_.connected = false;
  camera_info_.image_encoding = options_.output_encoding;
  if (is_usb_device(chosen)) {
    const auto & usb = chosen->SpecialInfo.stUsb3VInfo;
    camera_info_.transport = "USB3 Vision";
    camera_info_.model = normalise_serial(usb.chModelName, 64);
    camera_info_.serial_number = normalise_serial(usb.chSerialNumber, 64);
    camera_info_.vendor_id = usb.idVendor;
    camera_info_.product_id = usb.idProduct;
  } else {
    const auto & gige = chosen->SpecialInfo.stGigEInfo;
    camera_info_.transport = "GigE";
    camera_info_.model = normalise_serial(gige.chModelName, 32);
    camera_info_.serial_number = normalise_serial(gige.chSerialNumber, 16);
  }

  rc = api->create_handle(&handle_, chosen);
  if (rc != MV_OK) {throw std::runtime_error("MV_CC_CreateHandle failed: " + mvs_error_text(rc));}
  rc = api->open_device(handle_, MV_ACCESS_Exclusive, 0);
  if (rc != MV_OK) {
    throw std::runtime_error(
            "MV_CC_OpenDevice failed: " + mvs_error_text(rc) +
            " (is another application, e.g. the MVS client, holding the camera?)");
  }

  // 自动适配相机支持的格式：彩色优先，只有 Mono8 的相机仍可正常启动。
  const unsigned int selected_pixel_type =
    select_pixel_format(
    *api, handle_, options_.pixel_format, options_.output_encoding);
  camera_info_.pixel_format = mvs_pixel_type_name(selected_pixel_type);
  camera_info_.image_encoding =
    output_encoding_for_pixel_type(selected_pixel_type, options_.output_encoding);

  if (options_.width > 0) {
    const int width_rc = api->set_int_value_ex(handle_, "Width", options_.width);
    if (width_rc != MV_OK) {
      throw std::runtime_error("MV_CC_SetIntValueEx(Width) failed: " + mvs_error_text(width_rc));
    }
  }
  if (options_.height > 0) {
    const int height_rc = api->set_int_value_ex(handle_, "Height", options_.height);
    if (height_rc != MV_OK) {
      throw std::runtime_error("MV_CC_SetIntValueEx(Height) failed: " + mvs_error_text(height_rc));
    }
  }

  MVCC_INTVALUE_EX width_value{};
  MVCC_INTVALUE_EX height_value{};
  rc = api->get_int_value_ex(handle_, "Width", &width_value);
  if (rc != MV_OK || width_value.nCurValue <= 0) {
    throw std::runtime_error("could not read Width: " + mvs_error_text(rc));
  }
  rc = api->get_int_value_ex(handle_, "Height", &height_value);
  if (rc != MV_OK || height_value.nCurValue <= 0) {
    throw std::runtime_error("could not read Height: " + mvs_error_text(rc));
  }
  camera_info_.width = static_cast<int>(width_value.nCurValue);
  camera_info_.height = static_cast<int>(height_value.nCurValue);

  // 曝光和增益独立应用；0.0 增益也必须能够显式设置。
  apply_control_settings(options_);

  // GigE: enlarge the packet size to reduce per-packet overhead / loss.
  if ((chosen->nTLayerType & MV_GIGE_DEVICE) != 0) {
    int packet = options_.packet_size;//packet 是网口传输参数
    if (packet <= 0) {packet = api->get_optimal_packet_size(handle_);}
    if (packet > 0) {
      (void)api->set_int_value_ex(handle_, "GevSCPSPacketSize", packet);
    }
  }

//申请图像缓冲区
  MVCC_INTVALUE_EX payload{};
  rc = api->get_int_value_ex(handle_, "PayloadSize", &payload);
  if (rc != MV_OK || payload.nCurValue <= 0) {
    throw std::runtime_error("could not read PayloadSize: " + mvs_error_text(rc));
  }
  buffer_.assign(static_cast<std::size_t>(payload.nCurValue), 0);

  rc = api->start_grabbing(handle_);
  if (rc != MV_OK) {throw std::runtime_error("MV_CC_StartGrabbing failed: " + mvs_error_text(rc));}
  grabbing_ = true;
  camera_info_.connected = true;
  return;
#else
  throw std::logic_error(
          "The MVS SDK is installed, but this build was compiled WITHOUT its headers, so the "
          "acquisition path is not available. Rebuild with -DHIKROBOT_MVS_INCLUDE_DIR pointing "
          "at the SDK's include directory (default /opt/MVS/include).");
#endif
}

void MvsBackend::apply_control_settings(const MvsBackendOptions & settings)
{
  validate_mvs_options(settings);
#ifdef HIKROBOT_MVS_AVAILABLE
  if (handle_ == nullptr) {
    throw std::runtime_error("cannot set exposure/gain before the MVS camera is open");
  }

  std::string err;
  MvsApi * api = ensure_api(options_.sdk_lib_candidates, err);
  if (api == nullptr) {
    throw std::runtime_error("MVS SDK is unavailable: " + err);
  }
  auto try_get_enum = [&](const char * feature) -> std::optional<MVCC_ENUMVALUE> {
      MVCC_ENUMVALUE value{};
      if (api->get_enum_value(handle_, feature, &value) == MV_OK) {
        return value;
      }
      return std::nullopt;
    };
  auto try_get_float = [&](const char * feature) -> std::optional<MVCC_FLOATVALUE> {
      MVCC_FLOATVALUE value{};
      if (api->get_float_value(handle_, feature, &value) == MV_OK) {
        return value;
      }
      return std::nullopt;
    };

  MvsBackendOptions previous = options_;
  MvsBackendOptions next = previous;
  next.exposure_mode = lower_value(settings.exposure_mode);
  next.exposure_auto_mode = lower_value(settings.exposure_auto_mode);
  next.exposure_time_us = settings.exposure_time_us;
  next.gain_mode = lower_value(settings.gain_mode);
  next.gain_auto_mode = lower_value(settings.gain_auto_mode);
  next.gain_value = settings.gain_value;

  const auto exposure_auto_supported = try_get_enum("ExposureAuto");
  const auto gain_auto_supported = try_get_enum("GainAuto");

  auto choose_auto_mode =
    [](const std::optional<MVCC_ENUMVALUE> & supported,
      unsigned int desired, unsigned int alternate) -> std::optional<unsigned int> {
      if (!supported.has_value()) {return std::nullopt;}
      if (enum_contains(*supported, desired)) {return desired;}
      if (enum_contains(*supported, alternate)) {return alternate;}
      return std::nullopt;
    };

  std::optional<unsigned int> exposure_auto;
  if (next.exposure_mode == "auto") {
    const unsigned int desired = exposure_auto_value(next.exposure_auto_mode);
    const unsigned int alternate =
      desired == MV_EXPOSURE_AUTO_MODE_CONTINUOUS ?
      MV_EXPOSURE_AUTO_MODE_ONCE : MV_EXPOSURE_AUTO_MODE_CONTINUOUS;
    exposure_auto = choose_auto_mode(exposure_auto_supported, desired, alternate);
    if (exposure_auto.has_value()) {
      next.exposure_auto_mode =
        *exposure_auto == MV_EXPOSURE_AUTO_MODE_ONCE ? "once" : "continuous";
    } else {
      // 相机没有自动曝光能力时保持它当前设置，而不是导致启动失败。
      next.exposure_mode = "camera";
    }
  } else if (exposure_auto_supported.has_value() &&
    enum_contains(*exposure_auto_supported, MV_EXPOSURE_AUTO_MODE_OFF))
  {
    exposure_auto = MV_EXPOSURE_AUTO_MODE_OFF;
  }

  std::optional<unsigned int> gain_auto;
  if (next.gain_mode == "auto") {
    const unsigned int desired = gain_auto_value(next.gain_auto_mode);
    const unsigned int alternate =
      desired == MV_GAIN_MODE_CONTINUOUS ? MV_GAIN_MODE_ONCE : MV_GAIN_MODE_CONTINUOUS;
    gain_auto = choose_auto_mode(gain_auto_supported, desired, alternate);
    if (gain_auto.has_value()) {
      next.gain_auto_mode = *gain_auto == MV_GAIN_MODE_ONCE ? "once" : "continuous";
    } else {
      next.gain_mode = "camera";
    }
  } else if (gain_auto_supported.has_value() &&
    enum_contains(*gain_auto_supported, MV_GAIN_MODE_OFF))
  {
    gain_auto = MV_GAIN_MODE_OFF;
  }

  const auto old_exposure_auto = try_get_enum("ExposureAuto");
  const auto old_gain_auto = try_get_enum("GainAuto");
  const auto old_exposure = try_get_float("ExposureTime");
  const auto old_gain = try_get_float("Gain");

  auto rollback = [&]() {
      std::string rollback_error;
      if (old_exposure_auto.has_value()) {
        const int rc = api->set_enum_value(
          handle_, "ExposureAuto", old_exposure_auto->nCurValue);
        if (rc != MV_OK) {
          rollback_error += "ExposureAuto=" + mvs_error_text(rc);
        }
      }
      if ((!old_exposure_auto.has_value() ||
        old_exposure_auto->nCurValue == MV_EXPOSURE_AUTO_MODE_OFF) &&
        old_exposure.has_value())
      {
        const int rc = api->set_float_value(handle_, "ExposureTime", old_exposure->fCurValue);
        if (rc != MV_OK) {
          rollback_error += (rollback_error.empty() ? "" : "; ") +
            std::string("ExposureTime=") + mvs_error_text(rc);
        }
      }
      if (old_gain_auto.has_value()) {
        const int rc = api->set_enum_value(handle_, "GainAuto", old_gain_auto->nCurValue);
        if (rc != MV_OK) {
          rollback_error += (rollback_error.empty() ? "" : "; ") +
            std::string("GainAuto=") + mvs_error_text(rc);
        }
      }
      if ((!old_gain_auto.has_value() || old_gain_auto->nCurValue == MV_GAIN_MODE_OFF) &&
        old_gain.has_value())
      {
        const int rc = api->set_float_value(handle_, "Gain", old_gain->fCurValue);
        if (rc != MV_OK) {
          rollback_error += (rollback_error.empty() ? "" : "; ") +
            std::string("Gain=") + mvs_error_text(rc);
        }
      }
      return rollback_error;
    };

  std::optional<MVCC_FLOATVALUE> exposure_range;
  std::optional<MVCC_FLOATVALUE> gain_range;
  try {
    if (exposure_auto.has_value()) {
      set_enum_checked(*api, handle_, "ExposureAuto", *exposure_auto);
    }
    if (next.exposure_mode == "manual") {
      exposure_range = get_float_checked(*api, handle_, "ExposureTime");
      require_float_in_range(*exposure_range, next.exposure_time_us, "exposure_time_us");
      set_float_checked(*api, handle_, "ExposureTime", next.exposure_time_us);
    } else if (next.exposure_mode == "auto") {
      exposure_range = try_get_float("ExposureTime");
    }

    if (gain_auto.has_value()) {
      set_enum_checked(*api, handle_, "GainAuto", *gain_auto);
    }
    if (next.gain_mode == "manual") {
      gain_range = get_float_checked(*api, handle_, "Gain");
      require_float_in_range(*gain_range, next.gain_value, "gain_value");
      set_float_checked(*api, handle_, "Gain", next.gain_value);
    } else if (next.gain_mode == "auto") {
      gain_range = try_get_float("Gain");
    }
  } catch (const std::exception & exc) {
    const std::string rollback_error = rollback();
    throw std::runtime_error(
            std::string("failed to apply exposure/gain settings: ") + exc.what() +
            (rollback_error.empty() ? "" : "; rollback also failed: " + rollback_error));
  }

  // 连续自动模式必须确认已经生效；once 模式可能很快回到 Off，因此只检查设置调用成功。
  if (exposure_auto.has_value() &&
    *exposure_auto == MV_EXPOSURE_AUTO_MODE_CONTINUOUS)
  {
    const unsigned int actual_exposure_auto =
      get_enum_checked(*api, handle_, "ExposureAuto").nCurValue;
    if (actual_exposure_auto != *exposure_auto) {
      const std::string rollback_error = rollback();
      throw std::runtime_error(
              "ExposureAuto did not enter continuous mode" +
              (rollback_error.empty() ? "" : "; rollback failed: " + rollback_error));
    }
  }
  if (gain_auto.has_value() && *gain_auto == MV_GAIN_MODE_CONTINUOUS) {
    const unsigned int actual_gain_auto =
      get_enum_checked(*api, handle_, "GainAuto").nCurValue;
    if (actual_gain_auto != *gain_auto) {
      const std::string rollback_error = rollback();
      throw std::runtime_error(
              "GainAuto did not enter continuous mode" +
              (rollback_error.empty() ? "" : "; rollback failed: " + rollback_error));
    }
  }

  double actual_exposure_time = next.exposure_time_us;
  double actual_gain = next.gain_value;
  try {
    if (next.exposure_mode == "manual") {
      const MVCC_FLOATVALUE exposure_now =
        get_float_checked(*api, handle_, "ExposureTime");
      actual_exposure_time = exposure_now.fCurValue;
      if (!float_matches(actual_exposure_time, next.exposure_time_us)) {
        throw std::runtime_error(
                "ExposureTime readback mismatch: requested " +
                std::to_string(next.exposure_time_us) + ", got " +
                std::to_string(actual_exposure_time));
      }
    } else {
      const auto exposure_now = try_get_float("ExposureTime");
      if (exposure_now.has_value()) {actual_exposure_time = exposure_now->fCurValue;}
    }
    if (next.gain_mode == "manual") {
      const MVCC_FLOATVALUE gain_now = get_float_checked(*api, handle_, "Gain");
      actual_gain = gain_now.fCurValue;
      if (!float_matches(actual_gain, next.gain_value)) {
        throw std::runtime_error(
                "Gain readback mismatch: requested " + std::to_string(next.gain_value) +
                ", got " + std::to_string(actual_gain));
      }
    } else {
      const auto gain_now = try_get_float("Gain");
      if (gain_now.has_value()) {actual_gain = gain_now->fCurValue;}
    }
  } catch (const std::exception & exc) {
    const std::string rollback_error = rollback();
    throw std::runtime_error(
            std::string("failed to verify exposure/gain readback: ") + exc.what() +
            (rollback_error.empty() ? "" : "; rollback failed: " + rollback_error));
  }

  options_ = next;
  camera_info_.exposure_mode = next.exposure_mode;
  camera_info_.exposure_auto_mode = next.exposure_auto_mode;
  camera_info_.exposure_time_us = actual_exposure_time;
  camera_info_.gain_mode = next.gain_mode;
  camera_info_.gain_auto_mode = next.gain_auto_mode;
  camera_info_.gain_value = actual_gain;
  if (exposure_range.has_value()) {
    camera_info_.exposure_min_us = exposure_range->fMin;
    camera_info_.exposure_max_us = exposure_range->fMax;
  }
  if (gain_range.has_value()) {
    camera_info_.gain_min = gain_range->fMin;
    camera_info_.gain_max = gain_range->fMax;
  }
#else
  (void)settings;
  throw std::logic_error("MVS headers are unavailable; exposure/gain cannot be applied");
#endif
}

void MvsBackend::apply_pixel_format(
  const std::string & pixel_format, const std::string & output_encoding)
{
#ifdef HIKROBOT_MVS_AVAILABLE
  if (handle_ == nullptr) {
    throw std::runtime_error("cannot change PixelFormat before the MVS camera is open");
  }
  MvsBackendOptions candidate = options_;
  candidate.pixel_format = pixel_format;
  candidate.output_encoding = output_encoding;
  validate_mvs_options(candidate);

  std::string err;
  MvsApi * api = ensure_api(options_.sdk_lib_candidates, err);
  if (api == nullptr) {
    throw std::runtime_error("MVS SDK is unavailable: " + err);
  }

  const unsigned int old_pixel_type =
    get_enum_checked(*api, handle_, "PixelFormat").nCurValue;
  if (grabbing_) {
    const int stop_rc = api->stop_grabbing(handle_);
    if (stop_rc != MV_OK) {
      throw std::runtime_error("MV_CC_StopGrabbing failed: " + mvs_error_text(stop_rc));
    }
    grabbing_ = false;
  }

  auto restart_with_pixel = [&](unsigned int pixel_type) {
      const int set_rc = api->set_enum_value(handle_, "PixelFormat", pixel_type);
      if (set_rc != MV_OK) {
        return std::string("PixelFormat restore failed: ") + mvs_error_text(set_rc);
      }
      MVCC_INTVALUE_EX payload{};
      const int payload_rc = api->get_int_value_ex(handle_, "PayloadSize", &payload);
      if (payload_rc != MV_OK || payload.nCurValue <= 0) {
        return std::string("PayloadSize read failed: ") + mvs_error_text(payload_rc);
      }
      buffer_.assign(static_cast<std::size_t>(payload.nCurValue), 0);
      const int start_rc = api->start_grabbing(handle_);
      if (start_rc != MV_OK) {
        return std::string("MV_CC_StartGrabbing failed: ") + mvs_error_text(start_rc);
      }
      grabbing_ = true;
      return std::string{};
    };

  try {
    const unsigned int selected =
      select_pixel_format(
      *api, handle_, candidate.pixel_format, candidate.output_encoding);
    MVCC_INTVALUE_EX payload{};
    const int payload_rc = api->get_int_value_ex(handle_, "PayloadSize", &payload);
    if (payload_rc != MV_OK || payload.nCurValue <= 0) {
      throw std::runtime_error("PayloadSize read failed: " + mvs_error_text(payload_rc));
    }
    buffer_.assign(static_cast<std::size_t>(payload.nCurValue), 0);
    const int start_rc = api->start_grabbing(handle_);
    if (start_rc != MV_OK) {
      throw std::runtime_error("MV_CC_StartGrabbing failed: " + mvs_error_text(start_rc));
    }
    grabbing_ = true;
    options_.pixel_format = candidate.pixel_format;
    options_.output_encoding = candidate.output_encoding;
    camera_info_.pixel_format = mvs_pixel_type_name(selected);
    camera_info_.image_encoding =
      output_encoding_for_pixel_type(selected, candidate.output_encoding);
    camera_info_.connected = true;
  } catch (const std::exception & exc) {
    const std::string restart_error = restart_with_pixel(old_pixel_type);
    camera_info_.connected = grabbing_;
    throw std::runtime_error(
            std::string("failed to change PixelFormat: ") + exc.what() +
            (restart_error.empty() ? "; previous stream restored" :
            "; previous stream restore failed: " + restart_error));
  }
#else
  (void)pixel_format;
  (void)output_encoding;
  throw std::logic_error("MVS headers are unavailable; PixelFormat cannot be changed");
#endif
}

void MvsBackend::apply_geometry(int width, int height)
{
#ifdef HIKROBOT_MVS_AVAILABLE
  if (handle_ == nullptr) {
    throw std::runtime_error("cannot change Width/Height before the MVS camera is open");
  }
  if (width < 0 || height < 0) {
    throw std::invalid_argument("mvs_width and mvs_height must be >= 0");
  }
  if (width == 0 && height == 0) {
    return;
  }

  std::string err;
  MvsApi * api = ensure_api(options_.sdk_lib_candidates, err);
  if (api == nullptr) {
    throw std::runtime_error("MVS SDK is unavailable: " + err);
  }

  MVCC_INTVALUE_EX old_width{};
  MVCC_INTVALUE_EX old_height{};
  int rc = api->get_int_value_ex(handle_, "Width", &old_width);
  if (rc != MV_OK || old_width.nCurValue <= 0) {
    throw std::runtime_error("could not read old Width: " + mvs_error_text(rc));
  }
  rc = api->get_int_value_ex(handle_, "Height", &old_height);
  if (rc != MV_OK || old_height.nCurValue <= 0) {
    throw std::runtime_error("could not read old Height: " + mvs_error_text(rc));
  }

  if (grabbing_) {
    rc = api->stop_grabbing(handle_);
    if (rc != MV_OK) {
      throw std::runtime_error("MV_CC_StopGrabbing failed: " + mvs_error_text(rc));
    }
    grabbing_ = false;
  }

  auto apply_and_start = [&](int target_width, int target_height, std::string & error) {
      if (target_width > 0) {
        const int width_rc =
          api->set_int_value_ex(handle_, "Width", target_width);
        if (width_rc != MV_OK) {
          error = "MV_CC_SetIntValueEx(Width) failed: " + mvs_error_text(width_rc);
          return false;
        }
      }
      if (target_height > 0) {
        const int height_rc =
          api->set_int_value_ex(handle_, "Height", target_height);
        if (height_rc != MV_OK) {
          error = "MV_CC_SetIntValueEx(Height) failed: " + mvs_error_text(height_rc);
          return false;
        }
      }

      MVCC_INTVALUE_EX actual_width{};
      MVCC_INTVALUE_EX actual_height{};
      const int width_rc = api->get_int_value_ex(handle_, "Width", &actual_width);
      const int height_rc = api->get_int_value_ex(handle_, "Height", &actual_height);
      if (width_rc != MV_OK || actual_width.nCurValue <= 0) {
        error = "could not read Width: " + mvs_error_text(width_rc);
        return false;
      }
      if (height_rc != MV_OK || actual_height.nCurValue <= 0) {
        error = "could not read Height: " + mvs_error_text(height_rc);
        return false;
      }
      if (target_width > 0 && actual_width.nCurValue != target_width) {
        error = "Width readback mismatch: requested " + std::to_string(target_width) +
          ", got " + std::to_string(actual_width.nCurValue);
        return false;
      }
      if (target_height > 0 && actual_height.nCurValue != target_height) {
        error = "Height readback mismatch: requested " + std::to_string(target_height) +
          ", got " + std::to_string(actual_height.nCurValue);
        return false;
      }

      MVCC_INTVALUE_EX payload{};
      const int payload_rc = api->get_int_value_ex(handle_, "PayloadSize", &payload);
      if (payload_rc != MV_OK || payload.nCurValue <= 0) {
        error = "PayloadSize read failed: " + mvs_error_text(payload_rc);
        return false;
      }
      buffer_.assign(static_cast<std::size_t>(payload.nCurValue), 0);
      const int start_rc = api->start_grabbing(handle_);
      if (start_rc != MV_OK) {
        error = "MV_CC_StartGrabbing failed: " + mvs_error_text(start_rc);
        return false;
      }
      grabbing_ = true;
      camera_info_.connected = true;
      camera_info_.width = static_cast<int>(actual_width.nCurValue);
      camera_info_.height = static_cast<int>(actual_height.nCurValue);
      return true;
    };

  std::string apply_error;
  if (!apply_and_start(width, height, apply_error)) {
    std::string restore_error;
    camera_info_.connected = false;
    if (!apply_and_start(
        static_cast<int>(old_width.nCurValue),
        static_cast<int>(old_height.nCurValue), restore_error))
    {
      throw std::runtime_error(
              "failed to change Width/Height: " + apply_error +
              "; previous stream restore failed: " + restore_error);
    }
    throw std::runtime_error(
            "failed to change Width/Height: " + apply_error +
            "; previous stream restored");
  }

  options_.width = width;
  options_.height = height;
#else
  (void)width;
  (void)height;
  throw std::logic_error("MVS headers are unavailable; Width/Height cannot be changed");
#endif
}

MvsCameraInfo MvsBackend::camera_info() const
{
  MvsCameraInfo info = camera_info_;
#ifdef HIKROBOT_MVS_AVAILABLE
  if (handle_ == nullptr) {
    info.connected = false;
    return info;
  }

  std::string err;
  MvsApi * api = ensure_api(options_.sdk_lib_candidates, err);
  if (api == nullptr) {
    info.connected = false;
    return info;
  }
  info.connected = true;

  MVCC_ENUMVALUE pixel{};
  if (api->get_enum_value(handle_, "PixelFormat", &pixel) == MV_OK) {
    info.pixel_format = mvs_pixel_type_name(pixel.nCurValue);
    info.image_encoding =
      output_encoding_for_pixel_type(pixel.nCurValue, options_.output_encoding);
  }
  MVCC_INTVALUE_EX width{};
  MVCC_INTVALUE_EX height{};
  if (api->get_int_value_ex(handle_, "Width", &width) == MV_OK) {
    info.width = static_cast<int>(width.nCurValue);
  }
  if (api->get_int_value_ex(handle_, "Height", &height) == MV_OK) {
    info.height = static_cast<int>(height.nCurValue);
  }

  MVCC_ENUMVALUE exposure_auto{};
  MVCC_ENUMVALUE gain_auto{};
  MVCC_FLOATVALUE exposure{};
  MVCC_FLOATVALUE gain{};
  if (api->get_enum_value(handle_, "ExposureAuto", &exposure_auto) == MV_OK) {
    info.exposure_mode =
      exposure_auto.nCurValue == MV_EXPOSURE_AUTO_MODE_OFF ? "manual" : "auto";
    if (exposure_auto.nCurValue == MV_EXPOSURE_AUTO_MODE_ONCE) {
      info.exposure_auto_mode = "once";
    } else if (exposure_auto.nCurValue == MV_EXPOSURE_AUTO_MODE_CONTINUOUS) {
      info.exposure_auto_mode = "continuous";
    }
  }
  if (api->get_enum_value(handle_, "GainAuto", &gain_auto) == MV_OK) {
    info.gain_mode = gain_auto.nCurValue == MV_GAIN_MODE_OFF ? "manual" : "auto";
    if (gain_auto.nCurValue == MV_GAIN_MODE_ONCE) {
      info.gain_auto_mode = "once";
    } else if (gain_auto.nCurValue == MV_GAIN_MODE_CONTINUOUS) {
      info.gain_auto_mode = "continuous";
    }
  }
  if (api->get_float_value(handle_, "ExposureTime", &exposure) == MV_OK) {
    info.exposure_time_us = exposure.fCurValue;
    info.exposure_min_us = exposure.fMin;
    info.exposure_max_us = exposure.fMax;
  }
  if (api->get_float_value(handle_, "Gain", &gain) == MV_OK) {
    info.gain_value = gain.fCurValue;
    info.gain_min = gain.fMin;
    info.gain_max = gain.fMax;
  }
#endif
  return info;
}

void MvsBackend::close()
{
#ifdef HIKROBOT_MVS_AVAILABLE
  if (handle_ != nullptr) {
    std::string err;
    MvsApi * api = ensure_api(options_.sdk_lib_candidates, err);
    if (api != nullptr) {
      if (grabbing_) {
        (void)api->stop_grabbing(handle_);
      }
      (void)api->close_device(handle_);
      (void)api->destroy_handle(handle_);
    }
  }
#endif
  handle_ = nullptr;
  buffer_.clear();
  grabbing_ = false;
  camera_info_.connected = false;
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

  if (info.nFrameLen == 0U) {
    throw std::runtime_error("MVS returned a frame with nFrameLen=0");
  }
  if (info.nFrameLen > buffer_.size()) {
    throw std::runtime_error(
            "MVS returned nFrameLen=" + std::to_string(info.nFrameLen) +
            " but the payload buffer is only " + std::to_string(buffer_.size()) + " bytes");
  }

  auto convert_frame = [&](MvGvspPixelType dst_pixel_type, std::size_t channels,
      const std::string & encoding) {
      const std::size_t dst_size =
        checked_image_bytes(frame.width, frame.height, channels);
      std::vector<uint8_t> converted(dst_size);
      MV_CC_PIXEL_CONVERT_PARAM_EX cvt{};
      cvt.nWidth = static_cast<unsigned int>(frame.width);
      cvt.nHeight = static_cast<unsigned int>(frame.height);
      cvt.enSrcPixelType = info.enPixelType;
      cvt.pSrcData = buffer_.data();
      cvt.nSrcDataLen = info.nFrameLen;
      cvt.enDstPixelType = dst_pixel_type;
      cvt.pDstBuffer = converted.data();
      cvt.nDstBufferSize = static_cast<unsigned int>(converted.size());
      const int crc = api->convert_pixel_type(handle_, &cvt);
      if (crc != MV_OK) {
        throw std::runtime_error("MV_CC_ConvertPixelTypeEx failed: " + mvs_error_text(crc));
      }
      if (cvt.nDstLen != dst_size) {
        throw std::runtime_error(
                "MV_CC_ConvertPixelTypeEx returned nDstLen=" +
                std::to_string(cvt.nDstLen) + " expected=" + std::to_string(dst_size));
      }
      frame.encoding = encoding;
      frame.data = std::move(converted);
    };

  const std::string desired_output = lower_value(options_.output_encoding);
  if (info.enPixelType == PixelType_Gvsp_Mono8) {
    if (desired_output == "bgr8") {
      throw std::runtime_error(
              "camera returned Mono8 while image_encoding=bgr8 was explicitly required");
    }
    const std::size_t expected = checked_image_bytes(frame.width, frame.height, 1U);
    if (info.nFrameLen < expected) {
      throw std::runtime_error(
              "Mono8 frame is shorter than width*height: nFrameLen=" +
              std::to_string(info.nFrameLen) + " expected>=" + std::to_string(expected));
    }
    frame.encoding = "mono8";
    frame.data.assign(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(expected));
  } else if (info.enPixelType == PixelType_Gvsp_BGR8_Packed) {
    const std::size_t expected = checked_image_bytes(frame.width, frame.height, 3U);
    if (info.nFrameLen < expected) {
      throw std::runtime_error(
              "packed color frame is shorter than width*height*3: nFrameLen=" +
              std::to_string(info.nFrameLen) + " expected>=" + std::to_string(expected));
    }
    if (desired_output == "mono8") {
      convert_frame(PixelType_Gvsp_Mono8, 1U, "mono8");
    } else {
      frame.encoding = "bgr8";
      frame.data.assign(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(expected));
    }
  } else if (info.enPixelType == PixelType_Gvsp_RGB8_Packed) {
    const std::size_t expected = checked_image_bytes(frame.width, frame.height, 3U);
    if (info.nFrameLen < expected) {
      throw std::runtime_error(
              "RGB8 frame is shorter than width*height*3: nFrameLen=" +
              std::to_string(info.nFrameLen) + " expected>=" + std::to_string(expected));
    }
    const std::vector<uint8_t> rgb(
      buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(expected));
    if (desired_output == "mono8") {
      convert_frame(PixelType_Gvsp_Mono8, 1U, "mono8");
    } else {
      frame.encoding = "bgr8";
      frame.data = rgb8_to_bgr8(rgb, frame.width, frame.height);
    }
  } else if (is_bayer_pixel_type(static_cast<unsigned int>(info.enPixelType))) {
    // Bayer 转换使用当前帧的真实宽高，绝不使用固定分辨率。
    if (desired_output == "mono8") {
      convert_frame(PixelType_Gvsp_Mono8, 1U, "mono8");
    } else {
      convert_frame(PixelType_Gvsp_BGR8_Packed, 3U, "bgr8");
    }
  } else {
    if (desired_output == "bgr8") {
      convert_frame(PixelType_Gvsp_BGR8_Packed, 3U, "bgr8");
    } else if (desired_output == "mono8") {
      convert_frame(PixelType_Gvsp_Mono8, 1U, "mono8");
    } else {
      try {
        convert_frame(PixelType_Gvsp_BGR8_Packed, 3U, "bgr8");
      } catch (const std::exception &) {
        convert_frame(PixelType_Gvsp_Mono8, 1U, "mono8");
      }
    }
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

}  // namespace camera_driver
