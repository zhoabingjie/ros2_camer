// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "visual_obstacle_avoidance/visual_obstacle_node.hpp"

TEST(VisualObstacleNode, AdvertisesStableTopicsAndNeverCommandsTheChassis)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto node = std::make_shared<visual_obstacle_avoidance::VisualObstacleNode>();
  const auto & topics = node->advertised_topics();

  EXPECT_NE(std::find(topics.begin(), topics.end(), "/vision/obstacles"), topics.end());
  EXPECT_NE(
    std::find(topics.begin(), topics.end(), "/vision/obstacle_debug_image"), topics.end());
  for (const auto & topic : topics) {
    EXPECT_EQ(topic.find("cmd_vel"), std::string::npos) << topic;
  }
  node.reset();
  if (rclcpp::ok()) {rclcpp::shutdown();}
}
