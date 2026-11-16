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

#include "camera_driver/camera_node.hpp"

using camera_driver::CameraNode;
using camera_driver::frame_to_image;

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
  EXPECT_EQ(values["camera_connected"], "False");
  EXPECT_EQ(values["image_encoding"], "auto");
  EXPECT_EQ(values["exposure_mode"], "auto");
  EXPECT_EQ(values["gain_mode"], "auto");
  EXPECT_NE(values.count("reconnect_count"), 0u);
  EXPECT_NE(values.count("last_error"), 0u);
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

TEST_F(CameraNodeTest, DynamicParametersAcceptManualZeroGainAndRejectInvalidValues)
{
  auto node = std::make_shared<CameraNode>(retryOptions());

  auto result = node->set_parameter(rclcpp::Parameter("exposure_mode", "manual"));
  EXPECT_TRUE(result.successful) << result.reason;
  result = node->set_parameter(rclcpp::Parameter("exposure_time_us", 8000.0));
  EXPECT_TRUE(result.successful) << result.reason;
  result = node->set_parameter(rclcpp::Parameter("gain_mode", "manual"));
  EXPECT_TRUE(result.successful) << result.reason;
  result = node->set_parameter(rclcpp::Parameter("gain_value", 0.0));
  EXPECT_TRUE(result.successful) << result.reason;

  result = node->set_parameter(rclcpp::Parameter("frame_id", "base_link"));
  EXPECT_TRUE(result.successful) << result.reason;
  EXPECT_EQ(node->frame_id(), "base_link");

  result = node->set_parameter(rclcpp::Parameter("stream_timeout_s", 1.5));
  EXPECT_TRUE(result.successful) << result.reason;
  EXPECT_DOUBLE_EQ(node->health().stall_timeout_s(), 1.5);

  result = node->set_parameter(rclcpp::Parameter("grab_timeout_s", 0.3));
  EXPECT_TRUE(result.successful) << result.reason;
  result = node->set_parameter(rclcpp::Parameter("reconnect_backoff_s", 0.7));
  EXPECT_TRUE(result.successful) << result.reason;
  result = node->set_parameter(rclcpp::Parameter("reconnect_after_misses", 3));
  EXPECT_TRUE(result.successful) << result.reason;
  result = node->set_parameter(rclcpp::Parameter("mvs_width", 640));
  EXPECT_TRUE(result.successful) << result.reason;
  result = node->set_parameter(rclcpp::Parameter("mvs_height", 480));
  EXPECT_TRUE(result.successful) << result.reason;

  EXPECT_EQ(node->get_parameter("grab_timeout_s").as_double(), 0.3);
  EXPECT_EQ(node->get_parameter("reconnect_backoff_s").as_double(), 0.7);
  EXPECT_EQ(node->get_parameter("reconnect_after_misses").as_int(), 3);
  EXPECT_EQ(node->get_parameter("mvs_width").as_int(), 640);
  EXPECT_EQ(node->get_parameter("mvs_height").as_int(), 480);

  result = node->set_parameter(rclcpp::Parameter("stream_timeout_s", 0.0));
  EXPECT_FALSE(result.successful);
  result = node->set_parameter(rclcpp::Parameter("mvs_width", -1));
  EXPECT_FALSE(result.successful);
  result = node->set_parameter(rclcpp::Parameter("reconnect_after_misses", 0));
  EXPECT_FALSE(result.successful);

  result = node->set_parameter(rclcpp::Parameter("exposure_mode", "invalid"));
  EXPECT_FALSE(result.successful);
  EXPECT_EQ(node->get_parameter("exposure_mode").as_string(), "manual");

  result = node->set_parameter(rclcpp::Parameter("image_encoding", "mono8"));
  EXPECT_TRUE(result.successful) << result.reason;
  EXPECT_EQ(node->get_parameter("image_encoding").as_string(), "mono8");

  result = node->set_parameter(rclcpp::Parameter("image_encoding", "invalid"));
  EXPECT_FALSE(result.successful);
  EXPECT_EQ(node->get_parameter("image_encoding").as_string(), "mono8");

  result = node->set_parameter(rclcpp::Parameter("mvs_pixel_format", "Mono8"));
  EXPECT_TRUE(result.successful) << result.reason;
  node->stop();
}

TEST(CameraFrameConversion, MapsEveryField)
{
  camera_driver::Frame frame;
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
