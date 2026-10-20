// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0

#include "hikrobot_camera_ros2/camera_node.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <exception>
#include <sstream>
#include <string>
#include <typeinfo>
#include <utility>

#include "hikrobot_camera_ros2/backend_registry.hpp"

namespace hikrobot_camera_ros2
{
namespace
{
constexpr int64_t kNanosecondsPerSecond = 1000000000LL;

rclcpp::QoS build_qos(const std::string & reliability, int depth)
{
  rclcpp::QoS qos(static_cast<std::size_t>(std::max(1, depth)));
  qos.history(rclcpp::HistoryPolicy::KeepLast);
  if (reliability == "best_effort" || reliability == "besteffort" || reliability == "sensor") {
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  } else {
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  }
  qos.durability(rclcpp::DurabilityPolicy::Volatile);
  return qos;
}

/// Negative parameter means "disabled".
std::optional<int64_t> optional_frame_count(int64_t value)
{
  if (value < 0) {return std::nullopt;}
  return value;
}

std::string state_name(StreamState state)
{
  return to_string(state);
}

std::string format_double(double value)
{
  std::ostringstream os;
  os.precision(2);
  os << std::fixed << value;
  return os.str();
}
}  // namespace

sensor_msgs::msg::Image frame_to_image(const Frame & frame, const std::string & frame_id)
{
  sensor_msgs::msg::Image image;
  const int64_t stamp_ns = std::max<int64_t>(0, frame.stamp_ns);
  image.header.stamp.sec = static_cast<int32_t>(stamp_ns / kNanosecondsPerSecond);
  image.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % kNanosecondsPerSecond);
  image.header.frame_id = frame_id;
  image.height = static_cast<uint32_t>(frame.height);
  image.width = static_cast<uint32_t>(frame.width);
  image.encoding = frame.encoding;
  image.is_bigendian = false;
  image.step = static_cast<uint32_t>(frame.step());
  image.data = frame.data;
  return image;
}

CameraNode::CameraNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("camera_node", options)
{
  declare_parameters();
  config_ = read_config();
  health_ = std::make_unique<StreamHealthMonitor>(config_.stream_timeout_s);

  const auto qos = build_qos(config_.qos_reliability, config_.qos_depth);
  image_pub_ = create_publisher<sensor_msgs::msg::Image>(config_.image_topic, qos);
  info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(config_.camera_info_topic, qos);
  status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(config_.status_topic, 10);
  advertised_topics_ = {config_.image_topic, config_.camera_info_topic, config_.status_topic};

  load_calibration(config_.calibration_file);

  RCLCPP_INFO(
    get_logger(), "camera_node starting: backend=%s frame_id=%s image=%s info=%s calibrated=%s",
    config_.backend.c_str(), config_.frame_id.c_str(), config_.image_topic.c_str(),
    config_.camera_info_topic.c_str(), calibration_.has_value() ? "true" : "false");

  if (config_.backend == "mock") {
    if (!config_.mock.image_path.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "backend 'mock' is REPLAYING the still image '%s'. This is NOT a real "
        "camera capture and no live scene is observed.",
        config_.mock.image_path.c_str());
    } else {
      RCLCPP_WARN(
        get_logger(),
        "backend 'mock' generates synthetic frames; this is NOT a real camera capture");
    }
  }

  if (config_.require_camera_at_startup) {
    auto backend = make_backend();
    if (!backend) {
      throw std::runtime_error(
              "required camera backend '" + config_.backend +
              "' could not be opened; refusing to start with no image source. "
              "Use backend:=mock only for an explicit offline demo.");
    }
    backend_ = std::move(backend);
  }

  worker_ = std::thread(&CameraNode::acquire_loop, this);
  watchdog_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(config_.status_period_s)),
    [this]() {publish_status();});
}

CameraNode::~CameraNode()
{
  stop();
}

void CameraNode::declare_parameters()
{
  declare_parameter<std::string>("backend", "mock");
  declare_parameter<std::string>("frame_id", "camera_optical_frame");
  declare_parameter<std::string>("image_topic", "/camera/image_raw");
  declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
  declare_parameter<std::string>("status_topic", "/vision/status");
  declare_parameter<std::string>("calibration_file", "");
  declare_parameter<double>("stream_timeout_s", 2.0);
  declare_parameter<double>("grab_timeout_s", 0.2);
  declare_parameter<double>("reconnect_backoff_s", 1.0);
  declare_parameter<double>("status_period_s", 1.0);
  declare_parameter<std::string>("qos_reliability", "best_effort");
  declare_parameter<int>("qos_depth", 5);
  declare_parameter<bool>("require_camera_at_startup", true);

  declare_parameter<int>("mock_width", 640);
  declare_parameter<int>("mock_height", 480);
  declare_parameter<std::string>("mock_encoding", "mono8");
  declare_parameter<double>("mock_fps", 10.0);
  declare_parameter<std::string>("mock_pattern", "moving_bar");
  declare_parameter<int>("mock_fail_after_frames", -1);
  declare_parameter<double>("mock_fail_duration_s", 0.0);
  declare_parameter<int>("mock_raise_after_frames", -1);
  declare_parameter<std::string>("mock_image_path", "");

  declare_parameter<std::vector<std::string>>("sdk_lib_candidates", std::vector<std::string>{""});

  // --- real camera (mvs backend) ---
  declare_parameter<std::string>("mvs_serial_number", "");
  declare_parameter<std::string>("mvs_pixel_format", "");
  declare_parameter<int>("mvs_width", 0);
  declare_parameter<int>("mvs_height", 0);
  declare_parameter<double>("mvs_exposure_us", 0.0);
  declare_parameter<double>("mvs_gain", 0.0);
  declare_parameter<int>("mvs_packet_size", 0);
}

CameraNode::Config CameraNode::read_config()
{
  Config config;
  config.backend = get_parameter("backend").as_string();
  std::transform(
    config.backend.begin(), config.backend.end(), config.backend.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  config.frame_id = get_parameter("frame_id").as_string();
  config.image_topic = get_parameter("image_topic").as_string();
  config.camera_info_topic = get_parameter("camera_info_topic").as_string();
  config.status_topic = get_parameter("status_topic").as_string();
  config.calibration_file = get_parameter("calibration_file").as_string();
  config.stream_timeout_s = get_parameter("stream_timeout_s").as_double();
  config.grab_timeout_s = get_parameter("grab_timeout_s").as_double();
  config.reconnect_backoff_s = get_parameter("reconnect_backoff_s").as_double();
  config.status_period_s = get_parameter("status_period_s").as_double();
  config.qos_reliability = get_parameter("qos_reliability").as_string();
  config.qos_depth = static_cast<int>(get_parameter("qos_depth").as_int());
  config.require_camera_at_startup = get_parameter("require_camera_at_startup").as_bool();

  config.mock.width = static_cast<int>(get_parameter("mock_width").as_int());
  config.mock.height = static_cast<int>(get_parameter("mock_height").as_int());
  config.mock.encoding = get_parameter("mock_encoding").as_string();
  config.mock.fps = get_parameter("mock_fps").as_double();
  config.mock.pattern = get_parameter("mock_pattern").as_string();
  config.mock.fail_after_frames =
    optional_frame_count(get_parameter("mock_fail_after_frames").as_int());
  config.mock.fail_duration_s = get_parameter("mock_fail_duration_s").as_double();
  config.mock.raise_after_frames =
    optional_frame_count(get_parameter("mock_raise_after_frames").as_int());
  config.mock.image_path = get_parameter("mock_image_path").as_string();

  const auto candidates = get_parameter("sdk_lib_candidates").as_string_array();
  for (const auto & candidate : candidates) {
    if (!candidate.empty()) {config.mvs.sdk_lib_candidates.push_back(candidate);}
  }
  config.mvs.serial_number = get_parameter("mvs_serial_number").as_string();
  config.mvs.pixel_format = get_parameter("mvs_pixel_format").as_string();
  config.mvs.width = static_cast<int>(get_parameter("mvs_width").as_int());
  config.mvs.height = static_cast<int>(get_parameter("mvs_height").as_int());
  config.mvs.exposure_us = get_parameter("mvs_exposure_us").as_double();
  config.mvs.gain = get_parameter("mvs_gain").as_double();
  config.mvs.packet_size = static_cast<int>(get_parameter("mvs_packet_size").as_int());
  return config;
}

void CameraNode::load_calibration(const std::string & path)
{
  if (path.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "no calibration_file configured: publishing UNCALIBRATED CameraInfo (all-zero K/P). "
      "Do not use it for geometric rectification.");
    return;
  }
  try {
    calibration_ = ::hikrobot_camera_ros2::load_calibration(path);
    RCLCPP_INFO(
      get_logger(), "calibration loaded: %dx%d from %s", calibration_->image_width,
      calibration_->image_height, path.c_str());
  } catch (const std::exception & exc) {
    calibration_.reset();
    calibration_problem_ = exc.what();
    RCLCPP_ERROR(get_logger(), "calibration rejected: %s", exc.what());
  }
}

std::unique_ptr<CameraBackend> CameraNode::make_backend()
{
  BackendRequest request;
  request.name = config_.backend;
  request.mock = config_.mock;
  request.mvs = config_.mvs;
  try {
    auto backend = create_backend(request);
    backend->open();
    RCLCPP_INFO(
      get_logger(), "backend '%s' opened: %s", config_.backend.c_str(),
      backend->describe().c_str());
    return backend;
  } catch (const std::exception & exc) {
    health_->on_error(exc.what());
    RCLCPP_ERROR(
      get_logger(), "backend '%s' unavailable: %s", config_.backend.c_str(), exc.what());
    return nullptr;
  }
}

void CameraNode::close_backend()
{
  std::unique_ptr<CameraBackend> backend;
  {
    std::lock_guard<std::mutex> lock(backend_mutex_);
    backend = std::move(backend_);
  }
  if (backend) {
    try {
      backend->close();
    } catch (const std::exception & exc) {
      RCLCPP_WARN(get_logger(), "backend close failed: %s", exc.what());
    }
  }
}

void CameraNode::sleep_or_shutdown(double seconds)
{
  std::unique_lock<std::mutex> lock(wakeup_mutex_);
  wakeup_.wait_for(
    lock, std::chrono::duration<double>(seconds), [this]() {return shutdown_.load();});
}

void CameraNode::acquire_loop()
{
  while (!shutdown_.load()) {
    CameraBackend * backend = nullptr;
    {
      std::lock_guard<std::mutex> lock(backend_mutex_);
      backend = backend_.get();
    }
    if (backend == nullptr) {
      auto candidate = make_backend();
      if (!candidate) {
        sleep_or_shutdown(config_.reconnect_backoff_s);
        continue;
      }
      std::lock_guard<std::mutex> lock(backend_mutex_);
      backend_ = std::move(candidate);
      backend = backend_.get();
    }

    std::optional<Frame> frame;
    try {
      frame = backend->grab(config_.grab_timeout_s);
    } catch (const std::exception & exc) {
      health_->on_error(exc.what());
      RCLCPP_ERROR(get_logger(), "grab failed: %s; reopening backend", exc.what());
      close_backend();
      health_->on_reopen();
      sleep_or_shutdown(config_.reconnect_backoff_s);
      continue;
    }

    if (!frame.has_value()) {
      health_->on_missed();
      continue;
    }
    if (!frame->is_well_formed()) {
      std::ostringstream os;
      os << "malformed frame: " << frame->width << "x" << frame->height << " " << frame->encoding
         << " data=" << frame->data.size() << " expected=" << frame->expected_size();
      health_->on_error(os.str());
      RCLCPP_ERROR(get_logger(), "%s; not publishing", os.str().c_str());
      continue;
    }

    publish_frame(*frame);
    health_->on_frame();
  }
}

void CameraNode::publish_frame(const Frame & frame)
{
  auto image = frame_to_image(frame, config_.frame_id);
  sensor_msgs::msg::CameraInfo info;
  try {
    const Calibration * calibration = calibration_.has_value() ? &*calibration_ : nullptr;
    info = build_camera_info(
      frame.width, frame.height, config_.frame_id, image.header.stamp, calibration);
  } catch (const CalibrationMismatchError & exc) {
    // Resolution changed since calibration: never rescale silently.
    calibration_.reset();
    calibration_problem_ = exc.what();
    RCLCPP_ERROR(get_logger(), "calibration no longer valid: %s", exc.what());
    info = build_camera_info(
      frame.width, frame.height, config_.frame_id, image.header.stamp, nullptr);
  }
  image_pub_->publish(image);
  info_pub_->publish(info);
}

void CameraNode::publish_status()
{
  const auto snapshot = health_->snapshot();
  CameraBackend * backend = nullptr;
  {
    std::lock_guard<std::mutex> lock(backend_mutex_);
    backend = backend_.get();
  }
  const std::string description =
    backend != nullptr ? backend->describe() : ("backend=" + config_.backend + " (not open)");

  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  if (snapshot.state == StreamState::kBackendError) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  } else if (snapshot.state == StreamState::kStalled || snapshot.state == StreamState::kNoFrames) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }

  if (!last_state_.has_value() || *last_state_ != snapshot.state) {
    const std::string message = "stream state -> " + state_name(snapshot.state);
    if (level == diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
      RCLCPP_ERROR(get_logger(), "%s", message.c_str());
    } else if (level == diagnostic_msgs::msg::DiagnosticStatus::WARN) {
      RCLCPP_WARN(get_logger(), "%s", message.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "%s", message.c_str());
    }
    last_state_ = snapshot.state;
  }

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = get_name() + std::string(": stream");
  status.hardware_id = config_.backend;
  status.level = level;
  status.message = snapshot.last_error.has_value() ?
    (state_name(snapshot.state) + ": " + *snapshot.last_error) : state_name(snapshot.state);

  const bool is_mock = config_.backend == "mock";
  auto add = [&status](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = value;
      status.values.push_back(kv);
    };
  add("backend", config_.backend);
  add("mock_source", is_mock ? "True" : "False");
  add("frame_id", config_.frame_id);
  add("frames_total", std::to_string(snapshot.frames_total));
  add(
    "measured_fps",
    snapshot.measured_fps.has_value() ? format_double(*snapshot.measured_fps) : "n/a");
  add(
    "last_frame_age_s",
    snapshot.last_frame_age_s.has_value() ? format_double(*snapshot.last_frame_age_s) : "n/a");
  add("consecutive_misses", std::to_string(snapshot.consecutive_misses));
  add("consecutive_errors", std::to_string(snapshot.consecutive_errors));
  add("backend_reopens", std::to_string(snapshot.backend_reopens));
  add("calibrated", calibration_.has_value() ? "True" : "False");
  if (calibration_problem_.has_value()) {add("calibration_problem", *calibration_problem_);}
  add("describe", description);

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  array.status.push_back(status);
  status_pub_->publish(array);
}

void CameraNode::stop()
{
  if (shutdown_.exchange(true)) {return;}
  wakeup_.notify_all();
  if (worker_.joinable() && worker_.get_id() != std::this_thread::get_id()) {worker_.join();}
  close_backend();
}

}  // namespace hikrobot_camera_ros2
