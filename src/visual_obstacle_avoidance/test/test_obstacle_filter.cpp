// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <string>

#include "visual_obstacle_avoidance/obstacle_filter.hpp"

namespace
{
yolo_msgs::msg::Detection make_detection(
  const std::string & class_name, float score,
  double cx, double cy, double width, double height)
{
  yolo_msgs::msg::Detection detection;
  detection.class_id = 0;
  detection.class_name = class_name;
  detection.score = score;
  detection.id = "7";
  detection.bbox.center.position.x = cx;
  detection.bbox.center.position.y = cy;
  detection.bbox.size.x = width;
  detection.bbox.size.y = height;
  return detection;
}
}  // namespace

TEST(ObstacleFilter, SelectsOnlyConfiguredClassesInsideRoiAndAboveThresholds)
{
  yolo_msgs::msg::DetectionArray detections;
  detections.detections.push_back(make_detection("person", 0.90, 320.0, 300.0, 100.0, 200.0));
  detections.detections.push_back(make_detection("car", 0.95, 50.0, 300.0, 100.0, 100.0));
  detections.detections.push_back(make_detection("person", 0.20, 320.0, 300.0, 100.0, 200.0));
  detections.detections.push_back(make_detection("dog", 0.99, 320.0, 300.0, 100.0, 200.0));
  detections.detections.push_back(make_detection("person", 0.90, 320.0, 300.0, 10.0, 10.0));

  visual_obstacle_avoidance::ObstacleFilterOptions options;
  options.target_classes = {"person", "car"};

  const auto result = visual_obstacle_avoidance::select_obstacles(
    detections, 640U, 480U, options);

  ASSERT_EQ(result.size(), 1U);
  EXPECT_EQ(result.front().class_name, "person");
  EXPECT_FLOAT_EQ(result.front().score, 0.90F);
  EXPECT_DOUBLE_EQ(result.front().center_x, 320.0);
  EXPECT_DOUBLE_EQ(result.front().center_y, 300.0);
  EXPECT_DOUBLE_EQ(result.front().width, 100.0);
  EXPECT_DOUBLE_EQ(result.front().height, 200.0);
  EXPECT_EQ(result.front().track_id, "7");
}

TEST(ObstacleFilter, EmptyClassListMatchesAnyClass)
{
  yolo_msgs::msg::DetectionArray detections;
  detections.detections.push_back(make_detection("dog", 0.90, 320.0, 300.0, 100.0, 200.0));

  visual_obstacle_avoidance::ObstacleFilterOptions options;
  const auto result = visual_obstacle_avoidance::select_obstacles(
    detections, 640U, 480U, options);
  ASSERT_EQ(result.size(), 1U);
}

TEST(ObstacleFilter, UnknownImageSizeProducesNoObstacle)
{
  yolo_msgs::msg::DetectionArray detections;
  detections.detections.push_back(make_detection("person", 0.90, 320.0, 300.0, 100.0, 200.0));

  const auto result = visual_obstacle_avoidance::select_obstacles(
    detections, 0U, 0U, visual_obstacle_avoidance::ObstacleFilterOptions{});
  EXPECT_TRUE(result.empty());
}
