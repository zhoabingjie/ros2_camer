// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0

#ifndef VISUAL_OBSTACLE_AVOIDANCE__VISUAL_OBSTACLE_NODE_HPP_
#define VISUAL_OBSTACLE_AVOIDANCE__VISUAL_OBSTACLE_NODE_HPP_

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "dmu_vision_interfaces/msg/obstacle2_d.hpp"
#include "dmu_vision_interfaces/msg/visual_obstacle_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "yolo_msgs/msg/detection_array.hpp"

#include "visual_obstacle_avoidance/obstacle_filter.hpp"

namespace visual_obstacle_avoidance
{

class VisualObstacleNode : public rclcpp::Node
{
public:
  VisualObstacleNode();
  ~VisualObstacleNode() override = default;

  const std::vector<std::string> & advertised_topics() const {return advertised_topics_;}

private:
  void declare_parameters();
  void load_parameters();

  void on_detections(const yolo_msgs::msg::DetectionArray::ConstSharedPtr message);
  void on_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr message);
  void on_debug_image(const sensor_msgs::msg::Image::ConstSharedPtr message);
  void on_status(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message);
  void on_timer();

  std::uint8_t current_status() const;
  std::string status_text(std::uint8_t status) const;
  bool input_is_fresh() const;
  void publish_obstacles(bool force);
  void publish_diagnostics();
  void publish_debug_image();

  ObstacleFilterOptions filter_;
  std::string detections_topic_;
  std::string camera_info_topic_;
  std::string debug_image_topic_;
  std::string obstacles_topic_;
  std::string obstacle_debug_image_topic_;
  std::string status_topic_;
  std::string detection_reliability_;
  int qos_depth_{10};
  int required_consecutive_frames_{2};
  double input_timeout_s_{1.0};
  double diagnostic_timeout_s_{3.0};
  double status_period_s_{1.0};

  std::uint32_t image_width_{0};
  std::uint32_t image_height_{0};
  bool received_detection_{false};
  std::size_t consecutive_frames_{0};
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_detection_time_;
  std_msgs::msg::Header last_source_header_;
  std::vector<dmu_vision_interfaces::msg::Obstacle2D> obstacles_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_debug_image_;
  std::optional<std::uint8_t> last_published_status_;

  std::optional<std::chrono::steady_clock::time_point> camera_error_time_;
  std::optional<std::chrono::steady_clock::time_point> detector_error_time_;
  std::string camera_error_text_;
  std::string detector_error_text_;

  rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr debug_image_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_sub_;
  rclcpp::Publisher<dmu_vision_interfaces::msg::VisualObstacleArray>::SharedPtr obstacles_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr obstacle_debug_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::string> advertised_topics_;
};

}  // namespace visual_obstacle_avoidance

#endif  // VISUAL_OBSTACLE_AVOIDANCE__VISUAL_OBSTACLE_NODE_HPP_
