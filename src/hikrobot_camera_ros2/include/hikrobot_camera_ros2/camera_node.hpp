// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0
//
// Hikrobot camera node (Phase 1).
//
// Publishes
//   <image_topic>        sensor_msgs/msg/Image
//   <camera_info_topic>  sensor_msgs/msg/CameraInfo
//   <status_topic>       diagnostic_msgs/msg/DiagnosticArray
//
// Never publishes
//   /cmd_vel or any other velocity command. The vision subsystem is
//   perception-only by project rule; the chassis is driven by the control stack.
//
// Honesty rules enforced here:
//   * No fabricated pixels: frames come from a backend, and a backend that cannot
//     run throws instead of synthesising data.
//   * No fabricated intrinsics: uncalibrated output is the ROS all-zero marker.
//   * No stale data: if the stream stalls, image publishing stops (the watchdog
//     reports it) rather than repeating the last frame.

#ifndef HIKROBOT_CAMERA_ROS2__CAMERA_NODE_HPP_
#define HIKROBOT_CAMERA_ROS2__CAMERA_NODE_HPP_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "hikrobot_camera_ros2/camera_backend.hpp"
#include "hikrobot_camera_ros2/camera_info_factory.hpp"
#include "hikrobot_camera_ros2/mock_backend.hpp"
#include "hikrobot_camera_ros2/stream_health.hpp"

namespace hikrobot_camera_ros2
{

/// @brief Convert a backend Frame into a sensor_msgs/Image. Pure function, unit-tested.
sensor_msgs::msg::Image frame_to_image(const Frame & frame, const std::string & frame_id);

/// @brief Build a DiagnosticArray snapshot for the stream watchdog.
class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CameraNode() override;

  /// @brief Stop acquisition and release the backend. Idempotent.
  void stop();

  const StreamHealthMonitor & health() const {return *health_;}
  const std::string & configured_backend() const {return config_.backend;}
  const std::string & frame_id() const {return config_.frame_id;}

  /// @brief Topics this node advertises.
  ///
  /// Exposed so a test can assert the vision subsystem never advertises a
  /// velocity command without depending on discovery timing.
  const std::vector<std::string> & advertised_topics() const {return advertised_topics_;}

private:
  struct Config
  {
    std::string backend{"mock"};
    std::string frame_id{"camera_optical_frame"};
    std::string image_topic{"/camera/image_raw"};
    std::string camera_info_topic{"/camera/camera_info"};
    std::string status_topic{"/vision/status"};
    std::string calibration_file;
    double stream_timeout_s{2.0};
    double grab_timeout_s{0.2};
    double reconnect_backoff_s{1.0};
    double status_period_s{1.0};
    std::string qos_reliability{"best_effort"};
    int qos_depth{5};
    MockBackendOptions mock;
    std::vector<std::string> sdk_lib_candidates;
  };

  void declare_parameters();
  Config read_config();
  void load_calibration(const std::string & path);
  std::unique_ptr<CameraBackend> make_backend();
  void close_backend();
  void acquire_loop();
  void publish_frame(const Frame & frame);
  void publish_status();
  void sleep_or_shutdown(double seconds);

  Config config_;
  std::unique_ptr<StreamHealthMonitor> health_;
  std::unique_ptr<CameraBackend> backend_;
  mutable std::mutex backend_mutex_;
  std::thread worker_;
  std::atomic<bool> shutdown_{false};
  std::mutex wakeup_mutex_;
  std::condition_variable wakeup_;

  std::optional<Calibration> calibration_;
  std::optional<std::string> calibration_problem_;
  std::optional<StreamState> last_state_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_;

  std::vector<std::string> advertised_topics_;
};

}  // namespace hikrobot_camera_ros2

#endif  // HIKROBOT_CAMERA_ROS2__CAMERA_NODE_HPP_
