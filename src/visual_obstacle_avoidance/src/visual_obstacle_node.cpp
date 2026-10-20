// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0

#include "visual_obstacle_avoidance/visual_obstacle_node.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <exception>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <opencv2/imgproc.hpp>

#include "cv_bridge/cv_bridge.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "sensor_msgs/image_encodings.hpp"

namespace visual_obstacle_avoidance
{
namespace
{
using VisualStatus = dmu_vision_interfaces::msg::VisualObstacleArray;
using Clock = std::chrono::steady_clock;

rclcpp::QoS build_qos(const std::string & reliability, int depth)
{
  rclcpp::QoS qos(static_cast<std::size_t>(std::max(1, depth)));
  qos.history(rclcpp::HistoryPolicy::KeepLast);
  if (reliability == "best_effort" || reliability == "besteffort" ||
    reliability == "sensor")
  {
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  } else {
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  }
  qos.durability(rclcpp::DurabilityPolicy::Volatile);
  return qos;
}

std::string lower_copy(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return value;
}

double seconds_since(const Clock::time_point & start, const Clock::time_point & end)
{
  return std::chrono::duration<double>(end - start).count();
}

bool diagnostic_is_error(std::uint8_t level)
{
  return level == diagnostic_msgs::msg::DiagnosticStatus::ERROR ||
         level == diagnostic_msgs::msg::DiagnosticStatus::STALE;
}
}  // namespace

VisualObstacleNode::VisualObstacleNode()
: rclcpp::Node("visual_obstacle_node"), start_time_(Clock::now())
{
  declare_parameters();
  load_parameters();

  const auto detection_qos = build_qos(detection_reliability_, qos_depth_);
  obstacles_pub_ =
    create_publisher<dmu_vision_interfaces::msg::VisualObstacleArray>(
    obstacles_topic_, rclcpp::QoS(10).reliable());
  obstacle_debug_pub_ =
    create_publisher<sensor_msgs::msg::Image>(obstacle_debug_image_topic_, rclcpp::SensorDataQoS());
  diagnostic_pub_ =
    create_publisher<diagnostic_msgs::msg::DiagnosticArray>(status_topic_, 10);

  detections_sub_ = create_subscription<yolo_msgs::msg::DetectionArray>(
    detections_topic_, detection_qos,
    [this](yolo_msgs::msg::DetectionArray::ConstSharedPtr message) {on_detections(message);});
  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {on_camera_info(message);});
  debug_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    debug_image_topic_, rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::Image::ConstSharedPtr message) {on_debug_image(message);});
  status_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    status_topic_, 10,
    [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {on_status(message);});

  advertised_topics_ = {
    detections_topic_, camera_info_topic_, debug_image_topic_, obstacles_topic_,
    obstacle_debug_image_topic_, status_topic_};
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(status_period_s_)),
    [this]() {on_timer();});

  RCLCPP_INFO(
    get_logger(),
    "visual_obstacle_node started: detections=%s obstacles=%s debug_source=%s debug_output=%s",
    detections_topic_.c_str(), obstacles_topic_.c_str(), debug_image_topic_.c_str(),
    obstacle_debug_image_topic_.c_str());
}

void VisualObstacleNode::declare_parameters()
{
  declare_parameter<std::string>("detections_topic", "/vision/detections");
  declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
  declare_parameter<std::string>("debug_image_topic", "/vision/debug_image");
  declare_parameter<std::string>("obstacles_topic", "/vision/obstacles");
  declare_parameter<std::string>(
    "obstacle_debug_image_topic", "/vision/obstacle_debug_image");
  declare_parameter<std::string>("status_topic", "/vision/status");
  declare_parameter<std::string>("detection_reliability", "reliable");
  declare_parameter<int>("qos_depth", 10);
  declare_parameter<int>("required_consecutive_frames", 2);
  declare_parameter<double>("input_timeout_s", 1.0);
  declare_parameter<double>("diagnostic_timeout_s", 3.0);
  declare_parameter<double>("status_period_s", 1.0);
  declare_parameter<double>("min_score", 0.5);
  declare_parameter<double>("roi_x_min", 0.25);
  declare_parameter<double>("roi_x_max", 0.75);
  declare_parameter<double>("roi_y_min", 0.35);
  declare_parameter<double>("roi_y_max", 1.0);
  declare_parameter<double>("min_bbox_area_ratio", 0.005);
  declare_parameter<std::vector<std::string>>(
    "target_classes", std::vector<std::string>{
      "person", "bicycle", "car", "motorcycle", "bus", "truck"});
}

void VisualObstacleNode::load_parameters()
{
  detections_topic_ = get_parameter("detections_topic").as_string();
  camera_info_topic_ = get_parameter("camera_info_topic").as_string();
  debug_image_topic_ = get_parameter("debug_image_topic").as_string();
  obstacles_topic_ = get_parameter("obstacles_topic").as_string();
  obstacle_debug_image_topic_ = get_parameter("obstacle_debug_image_topic").as_string();
  status_topic_ = get_parameter("status_topic").as_string();
  detection_reliability_ = get_parameter("detection_reliability").as_string();
  qos_depth_ = static_cast<int>(get_parameter("qos_depth").as_int());
  required_consecutive_frames_ =
    std::max<int>(1, static_cast<int>(get_parameter("required_consecutive_frames").as_int()));
  input_timeout_s_ = get_parameter("input_timeout_s").as_double();
  diagnostic_timeout_s_ = get_parameter("diagnostic_timeout_s").as_double();
  status_period_s_ = get_parameter("status_period_s").as_double();

  filter_.min_score = get_parameter("min_score").as_double();
  filter_.roi_x_min = get_parameter("roi_x_min").as_double();
  filter_.roi_x_max = get_parameter("roi_x_max").as_double();
  filter_.roi_y_min = get_parameter("roi_y_min").as_double();
  filter_.roi_y_max = get_parameter("roi_y_max").as_double();
  filter_.min_bbox_area_ratio = get_parameter("min_bbox_area_ratio").as_double();
  const auto classes = get_parameter("target_classes").as_string_array();
  filter_.target_classes.assign(classes.begin(), classes.end());

  if (input_timeout_s_ <= 0.0 || diagnostic_timeout_s_ <= 0.0 || status_period_s_ <= 0.0) {
    throw std::invalid_argument("visual obstacle timeout and period parameters must be positive");
  }
  if (filter_.roi_x_min < 0.0 || filter_.roi_x_max > 1.0 ||
    filter_.roi_y_min < 0.0 || filter_.roi_y_max > 1.0 ||
    filter_.roi_x_min >= filter_.roi_x_max || filter_.roi_y_min >= filter_.roi_y_max)
  {
    throw std::invalid_argument("visual obstacle ROI must satisfy 0 <= min < max <= 1");
  }
}

void VisualObstacleNode::on_detections(
  const yolo_msgs::msg::DetectionArray::ConstSharedPtr message)
{
  received_detection_ = true;
  last_detection_time_ = Clock::now();
  last_source_header_ = message->header;
  obstacles_ = select_obstacles(*message, image_width_, image_height_, filter_);
  if (obstacles_.empty()) {
    consecutive_frames_ = 0;
  } else {
    ++consecutive_frames_;
  }

  publish_obstacles(true);
  publish_debug_image();
}

void VisualObstacleNode::on_camera_info(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr message)
{
  image_width_ = message->width;
  image_height_ = message->height;
}

void VisualObstacleNode::on_debug_image(
  const sensor_msgs::msg::Image::ConstSharedPtr message)
{
  latest_debug_image_ = message;
  if (image_width_ == 0U || image_height_ == 0U) {
    image_width_ = message->width;
    image_height_ = message->height;
  }
}

void VisualObstacleNode::on_status(
  const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message)
{
  const auto now = Clock::now();
  for (const auto & status : message->status) {
    const std::string name = lower_copy(status.name);
    if (name.find("visual_obstacle_node") != std::string::npos) {continue;}

    if (name.find("camera") != std::string::npos) {
      if (diagnostic_is_error(status.level)) {
        camera_error_time_ = now;
        camera_error_text_ = status.message;
      } else if (status.level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
        camera_error_time_.reset();
        camera_error_text_.clear();
      }
    }
    if (name.find("yolo") != std::string::npos || name.find("detector") != std::string::npos) {
      if (diagnostic_is_error(status.level)) {
        detector_error_time_ = now;
        detector_error_text_ = status.message;
      } else if (status.level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
        detector_error_time_.reset();
        detector_error_text_.clear();
      }
    }
  }
}

void VisualObstacleNode::on_timer()
{
  publish_diagnostics();
  publish_obstacles(false);
}

std::uint8_t VisualObstacleNode::current_status() const
{
  const auto now = Clock::now();
  if (camera_error_time_.has_value() &&
    seconds_since(*camera_error_time_, now) <= diagnostic_timeout_s_)
  {
    return VisualStatus::STATUS_CAMERA_ERROR;
  }
  if (detector_error_time_.has_value() &&
    seconds_since(*detector_error_time_, now) <= diagnostic_timeout_s_)
  {
    return VisualStatus::STATUS_DETECTOR_ERROR;
  }
  if (!received_detection_) {
    return seconds_since(start_time_, now) > input_timeout_s_ ?
           VisualStatus::STATUS_INPUT_TIMEOUT : VisualStatus::STATUS_UNKNOWN;
  }
  if (seconds_since(last_detection_time_, now) > input_timeout_s_) {
    return VisualStatus::STATUS_INPUT_TIMEOUT;
  }
  if (image_width_ == 0U || image_height_ == 0U) {return VisualStatus::STATUS_UNKNOWN;}
  if (obstacles_.empty()) {return VisualStatus::STATUS_CLEAR;}
  if (consecutive_frames_ < static_cast<std::size_t>(required_consecutive_frames_)) {
    return VisualStatus::STATUS_CANDIDATE;
  }
  return VisualStatus::STATUS_OBSTACLE;
}

std::string VisualObstacleNode::status_text(std::uint8_t status) const
{
  switch (status) {
    case VisualStatus::STATUS_UNKNOWN: return "unknown: waiting for image dimensions/data";
    case VisualStatus::STATUS_CLEAR: return "clear: fresh detections, no obstacle in ROI";
    case VisualStatus::STATUS_CANDIDATE: return "candidate: waiting for consecutive-frame confirmation";
    case VisualStatus::STATUS_OBSTACLE: return "obstacle: confirmed 2D candidate in ROI";
    case VisualStatus::STATUS_INPUT_TIMEOUT: return "input timeout: detection stream is stale";
    case VisualStatus::STATUS_CAMERA_ERROR: return "camera error: " + camera_error_text_;
    case VisualStatus::STATUS_DETECTOR_ERROR: return "detector error: " + detector_error_text_;
    default: return "unknown status";
  }
}

bool VisualObstacleNode::input_is_fresh() const
{
  return received_detection_ &&
         seconds_since(last_detection_time_, Clock::now()) <= input_timeout_s_;
}

void VisualObstacleNode::publish_obstacles(bool force)
{
  const std::uint8_t status = current_status();
  if (!force && last_published_status_.has_value() && *last_published_status_ == status) {
    return;
  }

  dmu_vision_interfaces::msg::VisualObstacleArray message;
  if (input_is_fresh()) {
    message.header = last_source_header_;
  } else {
    message.header.stamp = now();
    message.header.frame_id = last_source_header_.frame_id;
  }
  message.status = status;
  message.status_text = status_text(status);
  message.input_topic = detections_topic_;
  message.input_valid = input_is_fresh() &&
    status != VisualStatus::STATUS_CAMERA_ERROR &&
    status != VisualStatus::STATUS_DETECTOR_ERROR &&
    status != VisualStatus::STATUS_UNKNOWN;
  if (status == VisualStatus::STATUS_CANDIDATE || status == VisualStatus::STATUS_OBSTACLE) {
    message.obstacles = obstacles_;
  }
  obstacles_pub_->publish(message);

  if (!last_published_status_.has_value() || *last_published_status_ != status) {
    if (status == VisualStatus::STATUS_OBSTACLE) {
      RCLCPP_WARN(get_logger(), "%s", message.status_text.c_str());
    } else if (status == VisualStatus::STATUS_CLEAR) {
      RCLCPP_INFO(get_logger(), "%s", message.status_text.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "%s", message.status_text.c_str());
    }
  }
  last_published_status_ = status;
}

void VisualObstacleNode::publish_diagnostics()
{
  const std::uint8_t state = current_status();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = get_name() + std::string(": perception");
  status.hardware_id = "monocular_camera";
  status.message = status_text(state);

  if (state == VisualStatus::STATUS_CAMERA_ERROR ||
    state == VisualStatus::STATUS_DETECTOR_ERROR)
  {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  } else if (state == VisualStatus::STATUS_CLEAR) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }

  auto add = [&status](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = value;
      status.values.push_back(kv);
    };
  add("state", status_text(state));
  add("input_valid", input_is_fresh() ? "True" : "False");
  add("input_topic", detections_topic_);
  add("obstacle_count", std::to_string(obstacles_.size()));
  add("consecutive_frames", std::to_string(consecutive_frames_));
  add("image_width", std::to_string(image_width_));
  add("image_height", std::to_string(image_height_));

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  array.status.push_back(status);
  diagnostic_pub_->publish(array);
}

void VisualObstacleNode::publish_debug_image()
{
  if (!latest_debug_image_) {return;}

  cv_bridge::CvImagePtr converted;
  try {
    converted = cv_bridge::toCvCopy(
      latest_debug_image_, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception & exc) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "cannot draw obstacle debug image: %s", exc.what());
    return;
  }

  cv::Mat & image = converted->image;
  const double x0 = filter_.roi_x_min * static_cast<double>(image.cols);
  const double x1 = filter_.roi_x_max * static_cast<double>(image.cols);
  const double y0 = filter_.roi_y_min * static_cast<double>(image.rows);
  const double y1 = filter_.roi_y_max * static_cast<double>(image.rows);
  cv::rectangle(
    image, cv::Point(static_cast<int>(x0), static_cast<int>(y0)),
    cv::Point(static_cast<int>(x1), static_cast<int>(y1)),
    cv::Scalar(255, 180, 0), 2);

  const std::uint8_t state = current_status();
  cv::putText(
    image, status_text(state), cv::Point(10, 24), cv::FONT_HERSHEY_SIMPLEX,
    0.6, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);

  for (const auto & obstacle : obstacles_) {
    const int left = static_cast<int>(obstacle.center_x - obstacle.width / 2.0);
    const int top = static_cast<int>(obstacle.center_y - obstacle.height / 2.0);
    const int right = static_cast<int>(obstacle.center_x + obstacle.width / 2.0);
    const int bottom = static_cast<int>(obstacle.center_y + obstacle.height / 2.0);
    cv::rectangle(image, cv::Point(left, top), cv::Point(right, bottom),
      cv::Scalar(0, 0, 255), 2);
    std::ostringstream label;
    label << obstacle.class_name << ' ' << obstacle.score;
    if (!obstacle.track_id.empty()) {label << " id=" << obstacle.track_id;}
    cv::putText(
      image, label.str(), cv::Point(left, std::max(16, top - 6)),
      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
  }

  auto message = converted->toImageMsg();
  obstacle_debug_pub_->publish(*message);
}

}  // namespace visual_obstacle_avoidance
