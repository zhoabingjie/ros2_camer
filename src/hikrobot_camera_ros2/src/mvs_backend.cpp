// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0
//
// See mvs_backend.hpp for why this is a probe and not a driver.

#include "hikrobot_camera_ros2/mvs_backend.hpp"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <sstream>

namespace fs = std::filesystem;

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
  if (!fs::is_directory(dir, ec)) {return found;}
  try {
    for (const auto & entry : fs::directory_iterator(
        dir, fs::directory_options::skip_permission_denied))
    {
      const std::string name = entry.path().filename().string();
      if (name.rfind(prefix, 0) == 0) {found.push_back(entry.path().string());}
    }
  } catch (const fs::filesystem_error &) {
    // Unreadable directory: report whatever was found, never fabricate success.
  }
  std::sort(found.begin(), found.end());
  return found;
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
    if (!candidate.empty() && fs::exists(candidate, ec)) {push_unique(probe.lib_paths, candidate);}
  }

  for (const auto & path : header_candidates()) {
    if (fs::is_regular_file(path, ec)) {push_unique(probe.header_paths, path);}
  }

  for (const auto & name : env_hints()) {
    const char * value = std::getenv(name.c_str());
    if (value != nullptr) {probe.env.emplace_back(name, std::string(value));}
  }

  return probe;
}

MvsBackend::MvsBackend(std::vector<std::string> sdk_lib_candidates)
: extra_candidates_(std::move(sdk_lib_candidates))
{
}

const MvsSdkProbe & MvsBackend::probe() const
{
  if (!probe_.has_value()) {probe_ = probe_sdk(extra_candidates_);}
  return *probe_;
}

std::string MvsBackend::describe() const
{
  const auto & report = probe();
  std::ostringstream os;
  os << "backend=mvs requires_hardware=true implemented=false"
     << " sdk_library_found=" << (report.library_found() ? "true" : "false")
     << " sdk_headers_found=" << (report.headers_found() ? "true" : "false");
  return os.str();
}

void MvsBackend::open()
{
  const auto & report = probe();
  if (!report.library_found()) {
    throw BackendUnavailableError(
            std::string("Hikrobot MVS SDK not found. Cannot open an MV-CS050-10GM camera.\n") +
            report.summary() +
            "\nACTION: install the *Linux* MVS SDK (x86_64) and re-run. Only a Windows build "
            "(MVS_Win_STD_5.0.2) is currently on hand, which cannot be used on Ubuntu.");
  }
  if (!report.headers_found()) {
    throw BackendUnavailableError(
            std::string("MVS shared library found but MvCameraControl.h is missing, so the "
                        "wrapper cannot be written against the real API.\n") + report.summary());
  }
  throw std::logic_error(
          "The MVS SDK is present. The acquisition wrapper is intentionally not implemented "
          "yet: per the project rule 'do not guess device compatibility', it must be written "
          "against the actual MvCameraControl.h shipped with the Linux SDK (symbol names, "
          "struct layouts, pixel-format enums and error codes). No frame is published from "
          "this backend until that is done and verified on real hardware.");
}

void MvsBackend::close()
{
}

std::optional<Frame> MvsBackend::grab(double /*timeout_s*/)
{
  open();  // always throws; kept so the contract is explicit
  return std::nullopt;
}

}  // namespace hikrobot_camera_ros2
