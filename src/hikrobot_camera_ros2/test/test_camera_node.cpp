// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0
//
// Node-level tests use the real MVS backend. On a machine without a camera the
// node must report or refuse the missing hardware; it never produces fake data.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"

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

rclcpp::NodeOptions retryOptions()
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
    rclcpp::Parameter("require_camera_at_startup", false),
    rclcpp::Parameter("status_period_s", 0.1),
    rclcpp::Parameter("reconnect_backoff_s", 0.2),
  });
  return options;
}

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
  auto node = std::make_shared<CameraNode>(retryOptions());
  const auto & topics = node->advertised_topics();
  EXPECT_NE(std::find(topics.begin(), topics.end(), "/camera/image_raw"), topics.end());
  EXPECT_NE(std::find(topics.begin(), topics.end(), "/camera/camera_info"), topics.end());
  EXPECT_NE(std::find(topics.begin(), topics.end(), "/vision/status"), topics.end());
  node->stop();
}

TEST_F(CameraNodeTest, NeverAdvertisesAVelocityCommand)
{
  auto node = std::make_shared<CameraNode>(retryOptions());
  for (const auto & topic : node->advertised_topics()) {
    EXPECT_EQ(topic.find("cmd_vel"), std::string::npos) << topic;
  }
  for (const auto & entry : node->get_topic_names_and_types()) {
    EXPECT_EQ(entry.first.find("cmd_vel"), std::string::npos) << entry.first;
  }
  node->stop();
}

TEST_F(CameraNodeTest, PublishesMvsStreamDiagnostics)
{
  auto node = std::make_shared<CameraNode>(retryOptions());
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
  EXPECT_EQ(values["backend"], "mvs");
  EXPECT_EQ(values["calibrated"], "False");
  EXPECT_NE(values.count("describe"), 0u);
}

TEST_F(CameraNodeTest, StartupFailsWhenRealCameraIsRequiredButUnavailable)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
    rclcpp::Parameter("require_camera_at_startup", true),
    rclcpp::Parameter("reconnect_backoff_s", 0.2),
  });

  try {
    auto node = std::make_shared<CameraNode>(options);
    node->stop();
    GTEST_SKIP() << "a camera is attached and the real startup path succeeded";
  } catch (const std::exception & exc) {
    const std::string message = exc.what();
    EXPECT_NE(message.find("required MVS camera"), std::string::npos) << message;
  }
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
