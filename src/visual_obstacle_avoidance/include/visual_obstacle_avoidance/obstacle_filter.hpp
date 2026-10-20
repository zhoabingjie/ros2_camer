// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0

#ifndef VISUAL_OBSTACLE_AVOIDANCE__OBSTACLE_FILTER_HPP_
#define VISUAL_OBSTACLE_AVOIDANCE__OBSTACLE_FILTER_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "dmu_vision_interfaces/msg/obstacle2_d.hpp"
#include "yolo_msgs/msg/detection_array.hpp"

namespace visual_obstacle_avoidance
{

struct ObstacleFilterOptions
{
  std::vector<std::string> target_classes;
  double min_score{0.5};
  double roi_x_min{0.25};
  double roi_x_max{0.75};
  double roi_y_min{0.35};
  double roi_y_max{1.0};
  double min_bbox_area_ratio{0.005};
};

/// Select 2D detections that are relevant to the configured driving corridor.
///
/// The returned coordinates remain in pixels. This function has no depth input
/// and therefore deliberately returns no distance or 3D position.
std::vector<dmu_vision_interfaces::msg::Obstacle2D> select_obstacles(
  const yolo_msgs::msg::DetectionArray & detections,
  std::uint32_t image_width,
  std::uint32_t image_height,
  const ObstacleFilterOptions & options);

}  // namespace visual_obstacle_avoidance

#endif  // VISUAL_OBSTACLE_AVOIDANCE__OBSTACLE_FILTER_HPP_
