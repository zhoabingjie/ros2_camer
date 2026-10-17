// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0
//
// Node-level tests on the mock backend, plus the safety guard.
//
// These run without any camera, so they are the offline part of Phase 1
// acceptance. Real-device acceptance (plan T01/T02) stays marked "not completed".

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "hikrobot_camera_ros2/camera_info_factory.hpp"
#include "hikrobot_camera_ros2/camera_node.hpp"

using hikrobot_camera_ros2::CameraNode;
using hikrobot_camera_ros2::frame_to_image;

namespace
{
bool spin_until(const std::function<bool()> & predicate, double timeout_s)
{
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {return true;}
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return predicate();
}

rclcpp::NodeOptions fastMockOptions()
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
    rclcpp::Parameter("mock_width", 64),
    rclcpp::Parameter("mock_height", 48),
    rclcpp::Parameter("mock_fps", 100.0),
    rclcpp::Parameter("status_period_s", 0.1),
  });
  return options;
}

/// @brief Spins a node on a background executor for the lifetime of the fixture.
class SpinFixture
{
public:
  explicit SpinFixture(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    executor_.add_node(node_);
    thread_ = std::thread([this]() {executor_.spin();});
  }

  ~SpinFixture()
  {
    executor_.cancel();
    if (thread_.joinable()) {thread_.join();}
    executor_.remove_node(node_);
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread thread_;
};

class CameraNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  }
  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {rclcpp::shutdown();}
  }
};
}  // namespace

TEST_F(CameraNodeTest, AdvertisesDocumentedTopics)
{
  auto node = std::make_shared<CameraNode>(fastMockOptions());
  const auto & topics = node->advertised_topics();
  EXPECT_NE(std::find(topics.begin(), topics.end(), "/camera/image_raw"), topics.end());
  EXPECT_NE(std::find(topics.begin(), topics.end(), "/camera/camera_info"), topics.end());
  EXPECT_NE(std::find(topics.begin(), topics.end(), "/vision/status"), topics.end());
  node->stop();
}

TEST_F(CameraNodeTest, NeverAdvertisesAVelocityCommand)
{
  // Hard project constraint: the vision subsystem must not command the chassis.
  auto node = std::make_shared<CameraNode>(fastMockOptions());
  for (const auto & topic : node->advertised_topics()) {
    EXPECT_EQ(topic.find("cmd_vel"), std::string::npos) << topic;
  }
  // Secondary, graph-based check.
  for (const auto & entry : node->get_topic_names_and_types()) {
    EXPECT_EQ(entry.first.find("cmd_vel"), std::string::npos) << entry.first;
  }
  node->stop();
}

TEST_F(CameraNodeTest, PublishesImagesAndUncalibratedCameraInfo)
{
  auto node = std::make_shared<CameraNode>(fastMockOptions());
  SpinFixture spinner(node);

  std::vector<sensor_msgs::msg::Image> images;
  std::vector<sensor_msgs::msg::CameraInfo> infos;
  auto image_sub = node->create_subscription<sensor_msgs::msg::Image>(
    "/camera/image_raw", rclcpp::SensorDataQoS(),
    [&images](sensor_msgs::msg::Image::ConstSharedPtr message) {images.push_back(*message);});
  auto info_sub = node->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/camera_info", rclcpp::SensorDataQoS(),
    [&infos](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {infos.push_back(*message);});

  ASSERT_TRUE(spin_until([&]() {return !images.empty() && !infos.empty();}, 20.0))
    << "no image/camera_info received";

  const auto & image = images.front();
  EXPECT_EQ(image.width, 64u);
  EXPECT_EQ(image.height, 48u);
  EXPECT_EQ(image.encoding, "mono8");
  EXPECT_EQ(image.step, 64u);
  EXPECT_EQ(image.data.size(), 64u * 48u);
  EXPECT_EQ(image.header.frame_id, "camera_optical_frame");
  EXPECT_FALSE(image.is_bigendian);

  const auto & info = infos.front();
  EXPECT_EQ(info.width, 64u);
  EXPECT_EQ(info.height, 48u);
  EXPECT_EQ(info.header.frame_id, image.header.frame_id);
  // No calibration file configured, so the honest all-zero marker is required.
  EXPECT_TRUE(hikrobot_camera_ros2::is_uncalibrated(info));
  EXPECT_EQ(info.k[0], 0.0);
  EXPECT_EQ(info.p[0], 0.0);
}

TEST_F(CameraNodeTest, PublishesStreamDiagnostics)
{
  auto node = std::make_shared<CameraNode>(fastMockOptions());
  SpinFixture spinner(node);

  std::vector<diagnostic_msgs::msg::DiagnosticArray> statuses;
  auto status_sub = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/vision/status", 10,
    [&statuses](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {
      statuses.push_back(*message);
    });

  ASSERT_TRUE(spin_until([&]() {return !statuses.empty();}, 20.0)) << "no diagnostics received";

  const auto & status = statuses.back().status.front();
  std::map<std::string, std::string> values;
  for (const auto & kv : status.values) {values[kv.key] = kv.value;}
  EXPECT_EQ(values["backend"], "mock");
  EXPECT_EQ(values["mock_source"], "True") << "a mock source must be labelled as such";
  EXPECT_EQ(values["calibrated"], "False");
  EXPECT_NE(values.count("frames_total"), 0u);
  EXPECT_NE(values.count("measured_fps"), 0u);
}

TEST_F(CameraNodeTest, MvsBackendRefusesWithActionableMessage)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
    rclcpp::Parameter("backend", "mvs"),
    rclcpp::Parameter("status_period_s", 0.1),
    rclcpp::Parameter("reconnect_backoff_s", 0.2),
  });
  auto node = std::make_shared<CameraNode>(options);

  ASSERT_TRUE(
    spin_until([&]() {return node->health().snapshot().consecutive_errors > 0;}, 15.0))
    << "the mvs backend must report an error instead of silently doing nothing";

  const auto snapshot = node->health().snapshot();
  ASSERT_TRUE(snapshot.last_error.has_value());
  EXPECT_NE(snapshot.last_error->find("MVS"), std::string::npos) << *snapshot.last_error;
  EXPECT_EQ(snapshot.frames_total, 0u) << "no frame may be produced by an unusable backend";
  node->stop();
}

TEST(CameraFrameConversion, MapsEveryField)
{
  hikrobot_camera_ros2::Frame frame;
  frame.width = 4;
  frame.height = 2;
  frame.encoding = "bgr8";
  frame.data.assign(4u * 2u * 3u, 0);
  frame.stamp_ns = 1234;
  frame.sequence = 0;

  const auto image = frame_to_image(frame, "some_frame");
  EXPECT_EQ(image.width, 4u);
  EXPECT_EQ(image.height, 2u);
  EXPECT_EQ(image.encoding, "bgr8");
  EXPECT_EQ(image.step, 12u);
  EXPECT_FALSE(image.is_bigendian);
  EXPECT_EQ(image.data.size(), 24u);
  EXPECT_EQ(image.header.frame_id, "some_frame");
  EXPECT_EQ(image.header.stamp.nanosec, 1234u);
  EXPECT_EQ(image.header.stamp.sec, 0);
}
