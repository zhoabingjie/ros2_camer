// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0

#include "hikrobot_camera_ros2/camera_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <exception>
#include <sstream>
#include <string>
#include <typeinfo>
#include <utility>

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

std::string state_name(StreamState state)
{
  return to_string(state);
}

std::string format_double(double value)//format_double 函数把 double 格式化成保留 2 位小数的字符串
{
  std::ostringstream os;//声明一个字符串流对象，名字叫 os
  os.precision(2);
  os << std::fixed << value;//把 value 按固定 2 位小数写进 os
  return os.str();
}
}  // namespace

sensor_msgs::msg::Image frame_to_image(const Frame & frame, const std::string & frame_id)
{
  sensor_msgs::msg::Image image;
  const int64_t stamp_ns = std::max<int64_t>(0, frame.stamp_ns);
  image.header.stamp.sec = static_cast<int32_t>(stamp_ns / kNanosecondsPerSecond);
  image.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % kNanosecondsPerSecond);
  image.header.frame_id = frame_id;//填写图像的时间和坐标系
  image.height = static_cast<uint32_t>(frame.height);
  image.width = static_cast<uint32_t>(frame.width);
  image.encoding = frame.encoding;
  image.is_bigendian = false;
  image.step = static_cast<uint32_t>(frame.step());
  image.data = frame.data;
  return image;
}

CameraNode::CameraNode(const rclcpp::NodeOptions & options)//构造函数
: rclcpp::Node("camera_node", options)//初始化
{
  declare_parameters();//声明节点支持哪些参数
  config_ = read_config();//读取参数，整理成程序内部使用的配置对象
  health_ = std::make_unique<StreamHealthMonitor>(config_.stream_timeout_s);

  const auto qos = build_qos(config_.qos_reliability, config_.qos_depth);
  image_pub_ = create_publisher<sensor_msgs::msg::Image>(config_.image_topic, qos);
  info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(config_.camera_info_topic, qos);
  status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(config_.status_topic, 10);
  advertised_topics_ = {config_.image_topic, config_.camera_info_topic, config_.status_topic};

  load_calibration(config_.calibration_file);

  RCLCPP_INFO(
    get_logger(), "camera_node starting: backend=mvs frame_id=%s image=%s info=%s calibrated=%s",
    config_.frame_id.c_str(), config_.image_topic.c_str(),
    config_.camera_info_topic.c_str(), calibration_.has_value() ? "true" : "false");

  if (config_.require_camera_at_startup) {//启动mvs后端
    auto backend = make_backend();
    if (!backend) {
      throw std::runtime_error(
              "required MVS camera could not be opened; refusing to start with no image source. "
              "Check camera power, cable, network interface and address.");
    }
    backend_ = std::move(backend);
  }

  worker_ = std::thread(&CameraNode::acquire_loop, this);//采及线程
  watchdog_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(config_.status_period_s)),
    [this]() {publish_status();});
}

CameraNode::~CameraNode()
{
  stop();
}

//declare_parameters()：告诉 ROS 2，这个节点有哪些参数、每个参数是什么类型、默认值是什么。
//read_config()：把参数当前的值读出来，保存到 C++ 的配置对象里，供后续代码使用
void CameraNode::declare_parameters()//参数声明
{
  declare_parameter<std::string>("frame_id", "camera_optical_frame");
  declare_parameter<std::string>("image_topic", "/camera/image_raw");
  declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
  declare_parameter<std::string>("status_topic", "/vision/status");//状态
  declare_parameter<std::string>("calibration_file", "");//标定
  declare_parameter<double>("stream_timeout_s", 2.0);
  declare_parameter<double>("grab_timeout_s", 0.2);
  declare_parameter<double>("reconnect_backoff_s", 1.0);
  declare_parameter<double>("status_period_s", 1.0);
  declare_parameter<std::string>("qos_reliability", "best_effort");
  declare_parameter<int>("qos_depth", 5);
  declare_parameter<bool>("require_camera_at_startup", true);

  declare_parameter<std::vector<std::string>>("sdk_lib_candidates", std::vector<std::string>{""});

  // --- real camera (MVS GigE or USB3 Vision) ---
  declare_parameter<std::string>("mvs_transport", "auto");
  declare_parameter<std::string>("mvs_serial_number", "");
  declare_parameter<std::string>("mvs_pixel_format", "");
  declare_parameter<int>("mvs_width", 0);
  declare_parameter<int>("mvs_height", 0);
  declare_parameter<double>("mvs_exposure_us", 0.0);
  declare_parameter<double>("mvs_gain", 0.0);
  declare_parameter<int>("mvs_packet_size", 0);
}

CameraNode::Config CameraNode::read_config()//配置
{
  Config config;//类似于结构体
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

  const auto candidates = get_parameter("sdk_lib_candidates").as_string_array();//candidates保存读取到的字符串列表
  for (const auto & candidate : candidates) {//候选
    if (!candidate.empty()) {config.mvs.sdk_lib_candidates.push_back(candidate);}
  }//后面驱动会依次尝试这些路径去加载 MVS 的 SDK 动态库，哪个加载成功就用哪个
  config.mvs.transport = get_parameter("mvs_transport").as_string();
  config.mvs.serial_number = get_parameter("mvs_serial_number").as_string();
  config.mvs.pixel_format = get_parameter("mvs_pixel_format").as_string();//像素格式
  config.mvs.width = static_cast<int>(get_parameter("mvs_width").as_int());
  config.mvs.height = static_cast<int>(get_parameter("mvs_height").as_int());
  config.mvs.exposure_us = get_parameter("mvs_exposure_us").as_double();
  config.mvs.gain = get_parameter("mvs_gain").as_double();
  config.mvs.packet_size = static_cast<int>(get_parameter("mvs_packet_size").as_int());
  return config;
}

//作用：后面驱动会依次尝试这些路径去加载 MVS 的 SDK 动态库，哪个加载成功就用哪个，兼容不同电脑的 SDK 安装位置
void CameraNode::load_calibration(const std::string & path)//相机标定
{
  if (path.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "no calibration_file configured: publishing UNCALIBRATED CameraInfo (all-zero K/P). "
      "Do not use it for geometric rectification.");
    return;
  }
  try {
    const auto loaded = ::hikrobot_camera_ros2::load_calibration(path);
    {
      std::lock_guard<std::mutex> lock(calibration_mutex_);
      calibration_ = loaded;
      calibration_problem_.reset();
    }
    RCLCPP_INFO(
      get_logger(), "calibration loaded: %dx%d from %s", loaded.image_width,
      loaded.image_height, path.c_str());
  } catch (const std::exception & exc) {
    {
      std::lock_guard<std::mutex> lock(calibration_mutex_);
      calibration_.reset();
      calibration_problem_ = exc.what();
    }
    RCLCPP_ERROR(get_logger(), "calibration rejected: %s", exc.what());
  }
}
//相机
//开相机入口
std::shared_ptr<CameraBackend> CameraNode::make_backend()
{
  try {
    auto backend = std::make_shared<MvsBackend>(config_.mvs);
    backend->open();
    RCLCPP_INFO(get_logger(), "MVS backend opened: %s", backend->describe().c_str());
    return backend;
  } catch (const std::exception & exc) {
    health_->on_error(exc.what());
    RCLCPP_ERROR(get_logger(), "MVS backend unavailable: %s", exc.what());
    return nullptr;
  }
}

void CameraNode::close_backend()
{
  std::shared_ptr<CameraBackend> backend;
  {//guar自动锁   mutex互斥锁保证：同一时刻只有一个线程能持有这把锁
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

//采集循环
void CameraNode::acquire_loop()
{
  while (!shutdown_.load()) {
    std::shared_ptr<CameraBackend> backend;
    {//std::lock_guard：进入作用域时自动加锁，离开作用域时自动解锁
      std::lock_guard<std::mutex> lock(backend_mutex_);//std::mutex：互斥锁
      backend = backend_;
    }
    if (!backend) {
      auto candidate = make_backend();
      if (!candidate) {
        sleep_or_shutdown(config_.reconnect_backoff_s);//等待重连
        continue;
      }
      std::lock_guard<std::mutex> lock(backend_mutex_);
      backend_ = std::move(candidate);
      backend = backend_;
    }

    std::optional<Frame> frame;//帧
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
    if (!frame->is_well_formed()) {//检查图像数据是否完整
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

//frame 把相机帧转换成 ROS 2 消息，发布相机信息
void CameraNode::publish_frame(const Frame & frame)
{
  auto image = frame_to_image(frame, config_.frame_id);//这能让下游节点知道图像消息对应的坐标系名称
  sensor_msgs::msg::CameraInfo info;
  std::optional<Calibration> calibration_snapshot;
  {
    std::lock_guard<std::mutex> lock(calibration_mutex_);
    calibration_snapshot = calibration_;
  }
  try {//has_value()：检查有没有标定数据
    const Calibration * calibration =
      calibration_snapshot.has_value() ? &*calibration_snapshot : nullptr;
    info = build_camera_info(
      frame.width, frame.height, config_.frame_id, image.header.stamp, calibration);
  } catch (const CalibrationMismatchError & exc) {
    // Resolution changed since calibration: never rescale silently.
    {
      std::lock_guard<std::mutex> lock(calibration_mutex_);
      calibration_.reset();
      calibration_problem_ = exc.what();
    }
    RCLCPP_ERROR(get_logger(), "calibration no longer valid: %s", exc.what());
    info = build_camera_info(
      frame.width, frame.height, config_.frame_id, image.header.stamp, nullptr);
  }
  image_pub_->publish(image);
  info_pub_->publish(info);
}

//相机状态检测
void CameraNode::publish_status()
{
  const auto snapshot = health_->snapshot();//健康状态快照
  std::shared_ptr<CameraBackend> backend;
  {
    std::lock_guard<std::mutex> lock(backend_mutex_);
    backend = backend_;
  }
  const std::string description =
    backend != nullptr ? backend->describe() : "backend=mvs (not open)";

  bool calibrated = false;
  std::optional<std::string> calibration_problem;
  {
    std::lock_guard<std::mutex> lock(calibration_mutex_);
    calibrated = calibration_.has_value();
    calibration_problem = calibration_problem_;
  }

  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;//默认健康状态OK
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
  status.hardware_id = "mvs";
  status.level = level;
  status.message = snapshot.last_error.has_value() ?
    (state_name(snapshot.state) + ": " + *snapshot.last_error) : state_name(snapshot.state);

  auto add = [&status](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = value;
      status.values.push_back(kv);
    };
  add("backend", "mvs");
  add("frame_id", config_.frame_id);//坐标系名
  add("frames_total", std::to_string(snapshot.frames_total));//总共采集多少
  add(
    "measured_fps",
    snapshot.measured_fps.has_value() ? format_double(*snapshot.measured_fps) : "n/a");//实际帧率
  add(
    "last_frame_age_s",
    snapshot.last_frame_age_s.has_value() ? format_double(*snapshot.last_frame_age_s) : "n/a");
  add("consecutive_misses", std::to_string(snapshot.consecutive_misses));//丢帧多少次
  add("consecutive_errors", std::to_string(snapshot.consecutive_errors));
  add("backend_reopens", std::to_string(snapshot.backend_reopens));//自动重连多少次
  add("calibrated", calibrated ? "True" : "False");//有没有标定参数
  if (calibration_problem.has_value()) {add("calibration_problem", *calibration_problem);}
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
