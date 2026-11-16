// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0

#include "camera_driver/camera_node.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <sstream>
#include <string>
#include <typeinfo>
#include <utility>

#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace camera_driver
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

std::string lowercase(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return value;
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
  grab_timeout_s_.store(config_.grab_timeout_s);
  reconnect_backoff_s_.store(config_.reconnect_backoff_s);
  reconnect_after_misses_.store(config_.reconnect_after_misses);
  register_parameter_callback();
  health_ = std::make_unique<StreamHealthMonitor>(config_.stream_timeout_s);

  const auto qos = build_qos(config_.qos_reliability, config_.qos_depth);
  image_pub_ = create_publisher<sensor_msgs::msg::Image>(config_.image_topic, qos);
  info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(config_.camera_info_topic, qos);
  status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(config_.status_topic, 10);
  advertised_topics_ = {config_.image_topic, config_.camera_info_topic, config_.status_topic};

  RCLCPP_INFO(
    get_logger(), "camera_node starting: backend=mvs frame_id=%s image=%s info=%s",
    config_.frame_id.c_str(), config_.image_topic.c_str(),
    config_.camera_info_topic.c_str());

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
  declare_parameter<double>("stream_timeout_s", 2.0);
  declare_parameter<double>("grab_timeout_s", 0.2);
  declare_parameter<double>("reconnect_backoff_s", 1.0);
  declare_parameter<double>("status_period_s", 1.0);
  declare_parameter<std::string>("qos_reliability", "reliable");
  declare_parameter<int>("qos_depth", 5);
  declare_parameter<bool>("require_camera_at_startup", false);
  declare_parameter<int>("reconnect_after_misses", 10);

  declare_parameter<std::vector<std::string>>("sdk_lib_candidates", std::vector<std::string>{""});

  // --- 单台海康相机；auto 同时支持 GigE/USB3 Vision ---
  declare_parameter<std::string>("mvs_transport", "auto");
  declare_parameter<std::string>("mvs_serial_number", "");
  declare_parameter<std::string>("mvs_pixel_format", "");
  declare_parameter<std::string>("image_encoding", "auto");
  declare_parameter<int>("mvs_width", 0);
  declare_parameter<int>("mvs_height", 0);

  declare_parameter<std::string>("exposure_mode", "auto");
  declare_parameter<std::string>("exposure_auto_mode", "continuous");
  declare_parameter<double>("exposure_time_us", 8000.0);
  declare_parameter<std::string>("gain_mode", "auto");
  declare_parameter<std::string>("gain_auto_mode", "continuous");
  declare_parameter<double>("gain_value", 0.0);

  declare_parameter<int>("mvs_packet_size", 0);
}

CameraNode::Config CameraNode::read_config()//配置
{
  Config config;//类似于结构体
  config.frame_id = get_parameter("frame_id").as_string();
  config.image_topic = get_parameter("image_topic").as_string();
  config.camera_info_topic = get_parameter("camera_info_topic").as_string();
  config.status_topic = get_parameter("status_topic").as_string();
  config.stream_timeout_s = get_parameter("stream_timeout_s").as_double();
  config.grab_timeout_s = get_parameter("grab_timeout_s").as_double();
  config.reconnect_backoff_s = get_parameter("reconnect_backoff_s").as_double();
  config.status_period_s = get_parameter("status_period_s").as_double();
  config.qos_reliability = get_parameter("qos_reliability").as_string();
  config.qos_depth = static_cast<int>(get_parameter("qos_depth").as_int());
  config.require_camera_at_startup = get_parameter("require_camera_at_startup").as_bool();
  config.reconnect_after_misses =
    static_cast<int>(get_parameter("reconnect_after_misses").as_int());
  if (config.reconnect_after_misses < 1) {
    throw std::invalid_argument("reconnect_after_misses must be >= 1");
  }

  const auto candidates = get_parameter("sdk_lib_candidates").as_string_array();//candidates保存读取到的字符串列表
  for (const auto & candidate : candidates) {//候选
    if (!candidate.empty()) {config.mvs.sdk_lib_candidates.push_back(candidate);}
  }//后面驱动会依次尝试这些路径去加载 MVS 的 SDK 动态库，哪个加载成功就用哪个
  config.mvs.transport = get_parameter("mvs_transport").as_string();
  config.mvs.serial_number = get_parameter("mvs_serial_number").as_string();
  config.mvs.pixel_format = get_parameter("mvs_pixel_format").as_string();//像素格式
  config.mvs.output_encoding = get_parameter("image_encoding").as_string();
  config.mvs.width = static_cast<int>(get_parameter("mvs_width").as_int());
  config.mvs.height = static_cast<int>(get_parameter("mvs_height").as_int());
  config.mvs.exposure_mode = get_parameter("exposure_mode").as_string();
  config.mvs.exposure_auto_mode = get_parameter("exposure_auto_mode").as_string();
  config.mvs.exposure_time_us = get_parameter("exposure_time_us").as_double();
  config.mvs.gain_mode = get_parameter("gain_mode").as_string();
  config.mvs.gain_auto_mode = get_parameter("gain_auto_mode").as_string();
  config.mvs.gain_value = get_parameter("gain_value").as_double();
  config.mvs.packet_size = static_cast<int>(get_parameter("mvs_packet_size").as_int());
  validate_mvs_options(config.mvs);
  return config;
}

void CameraNode::register_parameter_callback()
{
  parameter_callback_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;

      MvsBackendOptions previous;
      MvsBackendOptions candidate;
      std::string frame_id_candidate;
      double stream_timeout_candidate = 0.0;
      double grab_timeout_candidate = 0.0;
      double reconnect_backoff_candidate = 0.0;
      int reconnect_after_misses_candidate = 0;
      {
        std::lock_guard<std::mutex> lock(config_mutex_);
        previous = config_.mvs;
        candidate = config_.mvs;
        frame_id_candidate = config_.frame_id;
        stream_timeout_candidate = config_.stream_timeout_s;
        grab_timeout_candidate = config_.grab_timeout_s;
        reconnect_backoff_candidate = config_.reconnect_backoff_s;
        reconnect_after_misses_candidate = config_.reconnect_after_misses;
      }

      bool recognized = false;
      bool controls_changed = false;
      bool pixel_format_changed = false;
      bool geometry_changed = false;
      try {
        for (const auto & parameter : parameters) {
          const std::string & name = parameter.get_name();
          if (name == "frame_id") {
            frame_id_candidate = parameter.as_string();
            recognized = true;
          } else if (name == "stream_timeout_s") {
            stream_timeout_candidate = parameter.as_double();
            recognized = true;
          } else if (name == "grab_timeout_s") {
            grab_timeout_candidate = parameter.as_double();
            recognized = true;
          } else if (name == "reconnect_backoff_s") {
            reconnect_backoff_candidate = parameter.as_double();
            recognized = true;
          } else if (name == "reconnect_after_misses") {
            reconnect_after_misses_candidate = static_cast<int>(parameter.as_int());
            recognized = true;
          } else if (name == "mvs_width") {
            candidate.width = static_cast<int>(parameter.as_int());
            recognized = true;
            geometry_changed = true;
          } else if (name == "mvs_height") {
            candidate.height = static_cast<int>(parameter.as_int());
            recognized = true;
            geometry_changed = true;
          } else if (name == "exposure_mode") {
            candidate.exposure_mode = lowercase(parameter.as_string());
            recognized = true;
            controls_changed = true;
          } else if (name == "exposure_auto_mode") {
            candidate.exposure_auto_mode = lowercase(parameter.as_string());
            recognized = true;
            controls_changed = true;
          } else if (name == "exposure_time_us") {
            candidate.exposure_time_us = parameter.as_double();
            recognized = true;
            controls_changed = true;
          } else if (name == "gain_mode") {
            candidate.gain_mode = lowercase(parameter.as_string());
            recognized = true;
            controls_changed = true;
          } else if (name == "gain_auto_mode") {
            candidate.gain_auto_mode = lowercase(parameter.as_string());
            recognized = true;
            controls_changed = true;
          } else if (name == "gain_value") {
            candidate.gain_value = parameter.as_double();
            recognized = true;
            controls_changed = true;
          } else if (name == "mvs_pixel_format") {
            candidate.pixel_format = parameter.as_string();
            recognized = true;
            pixel_format_changed = true;
          } else if (name == "image_encoding") {
            candidate.output_encoding = parameter.as_string();
            recognized = true;
            // 输出编码变化可能需要重新选择相机 PixelFormat 或转换目标。
            pixel_format_changed = true;
          }
        }
      } catch (const std::exception & exc) {
        result.successful = false;
        result.reason = std::string("invalid parameter type: ") + exc.what();
        return result;
      }

      if (!recognized) {
        return result;
      }
      try {
        validate_mvs_options(candidate);
        if (frame_id_candidate.empty()) {
          throw std::invalid_argument("frame_id must not be empty");
        }
        const auto require_positive = [](double value, const char * name) {
            if (!std::isfinite(value) || !(value > 0.0)) {
              throw std::invalid_argument(std::string(name) + " must be finite and > 0");
            }
          };
        require_positive(stream_timeout_candidate, "stream_timeout_s");
        require_positive(grab_timeout_candidate, "grab_timeout_s");
        require_positive(reconnect_backoff_candidate, "reconnect_backoff_s");
        if (reconnect_after_misses_candidate < 1) {
          throw std::invalid_argument("reconnect_after_misses must be >= 1");
        }
      } catch (const std::exception & exc) {
        result.successful = false;
        result.reason = exc.what();
        return result;
      }

      std::shared_ptr<CameraBackend> backend;
      {
        std::lock_guard<std::mutex> lock(backend_mutex_);
        backend = backend_;
      }
      if (backend) {
        auto mvs_backend = std::dynamic_pointer_cast<MvsBackend>(backend);
        if (!mvs_backend) {
          result.successful = false;
          result.reason = "active backend is not MvsBackend";
          return result;
        }

        std::lock_guard<std::mutex> io_lock(camera_io_mutex_);
        MvsCameraInfo before_info;
        if (geometry_changed) {
          before_info = mvs_backend->camera_info();
          if (!before_info.connected || before_info.width <= 0 || before_info.height <= 0) {
            result.successful = false;
            result.reason = "cannot change Width/Height: camera geometry is unavailable";
            return result;
          }
        }

        bool controls_applied = false;
        bool pixel_format_applied = false;
        bool geometry_applied = false;
        try {
          if (geometry_changed) {
            mvs_backend->apply_geometry(candidate.width, candidate.height);
            geometry_applied = true;
          }
          if (pixel_format_changed) {
            mvs_backend->apply_pixel_format(
              candidate.pixel_format, candidate.output_encoding);
            pixel_format_applied = true;
          }
          if (controls_changed) {
            mvs_backend->apply_control_settings(candidate);
            controls_applied = true;
          }
        } catch (const std::exception & exc) {
          std::string rollback_error;
          if (controls_applied) {
            try {
              mvs_backend->apply_control_settings(previous);
            } catch (const std::exception & rollback_exc) {
              rollback_error = rollback_exc.what();
            }
          }
          if (pixel_format_applied) {
            try {
              mvs_backend->apply_pixel_format(
                previous.pixel_format, previous.output_encoding);
            } catch (const std::exception & rollback_exc) {
              rollback_error +=
                (rollback_error.empty() ? "" : "; ") + std::string("PixelFormat: ") +
                rollback_exc.what();
            }
          }
          if (geometry_applied) {
            try {
              mvs_backend->apply_geometry(before_info.width, before_info.height);
            } catch (const std::exception & rollback_exc) {
              rollback_error +=
                (rollback_error.empty() ? "" : "; ") + std::string("Width/Height: ") +
                rollback_exc.what();
            }
          }
          result.successful = false;
          result.reason = std::string("camera rejected parameter update: ") + exc.what();
          if (!rollback_error.empty()) {
            result.reason += "; rollback failed: " + rollback_error;
          }
          return result;
        }
      }

      {
        std::lock_guard<std::mutex> lock(config_mutex_);
        config_.mvs = candidate;
        config_.frame_id = frame_id_candidate;
        config_.stream_timeout_s = stream_timeout_candidate;
        config_.grab_timeout_s = grab_timeout_candidate;
        config_.reconnect_backoff_s = reconnect_backoff_candidate;
        config_.reconnect_after_misses = reconnect_after_misses_candidate;
        ++config_generation_;
      }
      grab_timeout_s_.store(grab_timeout_candidate);
      reconnect_backoff_s_.store(reconnect_backoff_candidate);
      reconnect_after_misses_.store(reconnect_after_misses_candidate);
      health_->set_stall_timeout_s(stream_timeout_candidate);
      return result;
    });
}

//相机
//开相机入口
std::shared_ptr<CameraBackend> CameraNode::make_backend()
{
  try {
    MvsBackendOptions options;
    uint64_t generation = 0;
    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      options = config_.mvs;
      generation = config_generation_;
    }
    auto backend = std::make_shared<MvsBackend>(options);
    backend->open();
    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      if (generation != config_generation_) {
        backend->close();
        RCLCPP_INFO(get_logger(), "camera configuration changed during open; retrying");
        return nullptr;
      }
    }
    if (!last_backend_error_.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "MVS backend recovered after %llu failed reconnect attempt(s)",
        static_cast<unsigned long long>(backend_error_retries_));
      last_backend_error_.clear();
      backend_error_retries_ = 0;
    }
    RCLCPP_INFO(get_logger(), "MVS backend opened: %s", backend->describe().c_str());
    return backend;
  } catch (const std::exception & exc) {
    const std::string message = exc.what();
    health_->on_error(message);

    // 相同错误只打印第一次；后台仍按 reconnect_backoff_s 持续重试。
    if (last_backend_error_.empty()) {
      RCLCPP_ERROR(get_logger(), "MVS backend unavailable: %s", message.c_str());
      last_backend_error_ = message;
      backend_error_retries_ = 1;
    } else if (last_backend_error_ != message) {
      RCLCPP_ERROR(
        get_logger(),
        "MVS backend error changed after %llu retry attempt(s): %s",
        static_cast<unsigned long long>(backend_error_retries_),
        message.c_str());
      last_backend_error_ = message;
      backend_error_retries_ = 1;
    } else {
      ++backend_error_retries_;
    }
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
      std::lock_guard<std::mutex> io_lock(camera_io_mutex_);
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
  unsigned int consecutive_grab_misses = 0;
  while (!shutdown_.load()) {
    std::shared_ptr<CameraBackend> backend;
    {//std::lock_guard：进入作用域时自动加锁，离开作用域时自动解锁
      std::lock_guard<std::mutex> lock(backend_mutex_);//std::mutex：互斥锁
      backend = backend_;
    }
    if (!backend) {
      auto candidate = make_backend();
      if (!candidate) {
        sleep_or_shutdown(reconnect_backoff_s_.load());//等待重连
        continue;
      }
      std::lock_guard<std::mutex> lock(backend_mutex_);
      backend_ = std::move(candidate);
      backend = backend_;
    }

    std::optional<Frame> frame;//帧
    try {
      std::lock_guard<std::mutex> io_lock(camera_io_mutex_);
      frame = backend->grab(grab_timeout_s_.load());
    } catch (const std::exception & exc) {
      health_->on_error(exc.what());
      RCLCPP_ERROR(get_logger(), "grab failed: %s; reopening backend", exc.what());
      close_backend();
      health_->on_reopen();
      sleep_or_shutdown(reconnect_backoff_s_.load());
      continue;
    }

    if (!frame.has_value()) {
      health_->on_missed();
      ++consecutive_grab_misses;
      if (consecutive_grab_misses >=
        static_cast<unsigned int>(reconnect_after_misses_.load()))
      {
        RCLCPP_WARN(
          get_logger(),
          "no camera frame after %u consecutive timeouts; reopening MVS device",
          consecutive_grab_misses);
        close_backend();
        health_->on_reopen();
        consecutive_grab_misses = 0;
        sleep_or_shutdown(reconnect_backoff_s_.load());
      }
      continue;
    }
    consecutive_grab_misses = 0;
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
  std::string frame_id;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    frame_id = config_.frame_id;
  }
  auto image = frame_to_image(frame, frame_id);//这能让下游节点知道图像消息对应的坐标系名称
  sensor_msgs::msg::CameraInfo info;
  info.header = image.header;
  info.width = static_cast<uint32_t>(frame.width);
  info.height = static_cast<uint32_t>(frame.height);
  info.k.fill(0.0);
  info.r.fill(0.0);
  info.p.fill(0.0);
  info.distortion_model.clear();
  info.d.clear();
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
  MvsCameraInfo camera_info;
  bool has_mvs_info = false;
  if (backend) {
    auto mvs_backend = std::dynamic_pointer_cast<MvsBackend>(backend);
    if (mvs_backend) {
      std::lock_guard<std::mutex> io_lock(camera_io_mutex_);
      camera_info = mvs_backend->camera_info();
      has_mvs_info = true;
    }
  }

  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;//默认健康状态OK
  if (snapshot.state == StreamState::kBackendError) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  } else if (
    snapshot.state == StreamState::kStalled || snapshot.state == StreamState::kNoFrames ||
    (has_mvs_info && !camera_info.connected))
  {
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
  add("camera_connected", camera_info.connected ? "True" : "False");
  add("transport", camera_info.transport.empty() ? config_.mvs.transport : camera_info.transport);
  add("model", camera_info.model.empty() ? "unknown" : camera_info.model);
  add("serial_number", camera_info.serial_number.empty() ? "unknown" : camera_info.serial_number);
  add("vendor_id", std::to_string(camera_info.vendor_id));
  add("product_id", std::to_string(camera_info.product_id));
  add("pixel_format", camera_info.pixel_format.empty() ? "unknown" : camera_info.pixel_format);
  add("image_encoding", camera_info.image_encoding.empty() ?
    config_.mvs.output_encoding : camera_info.image_encoding);
  add("width", camera_info.width > 0 ? std::to_string(camera_info.width) : "unknown");
  add("height", camera_info.height > 0 ? std::to_string(camera_info.height) : "unknown");
  add("exposure_mode", camera_info.exposure_mode.empty() ?
    config_.mvs.exposure_mode : camera_info.exposure_mode);
  add("exposure_auto_mode", camera_info.exposure_auto_mode.empty() ?
    config_.mvs.exposure_auto_mode : camera_info.exposure_auto_mode);
  add(
    "exposure_time_us",
    format_double(camera_info.connected ?
    camera_info.exposure_time_us : config_.mvs.exposure_time_us));
  add("gain_mode", camera_info.gain_mode.empty() ? config_.mvs.gain_mode : camera_info.gain_mode);
  add("gain_auto_mode", camera_info.gain_auto_mode.empty() ?
    config_.mvs.gain_auto_mode : camera_info.gain_auto_mode);
  add(
    "gain_value",
    format_double(camera_info.connected ? camera_info.gain_value : config_.mvs.gain_value));
  add("frame_id", frame_id());//坐标系名
  add("frames_total", std::to_string(snapshot.frames_total));//总共采集多少
  add(
    "measured_fps",
    snapshot.measured_fps.has_value() ? format_double(*snapshot.measured_fps) : "n/a");//实际帧率
  add(
    "last_frame_age_s",
    snapshot.last_frame_age_s.has_value() ? format_double(*snapshot.last_frame_age_s) : "n/a");
  add("consecutive_misses", std::to_string(snapshot.consecutive_misses));//丢帧多少次
  add("consecutive_errors", std::to_string(snapshot.consecutive_errors));
  add("reconnect_count", std::to_string(snapshot.backend_reopens));//自动重连多少次
  add("backend_reopens", std::to_string(snapshot.backend_reopens));
  add("last_error", snapshot.last_error.has_value() ? *snapshot.last_error : "none");
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

}  // namespace camera_driver
