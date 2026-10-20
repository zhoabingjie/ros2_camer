// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0

#include "visual_obstacle_avoidance/obstacle_filter.hpp"

#include <algorithm>

namespace visual_obstacle_avoidance
{
namespace
{
bool class_is_selected(
  const std::string & name, const std::vector<std::string> & selected)
{
  return selected.empty() ||
         std::find(selected.begin(), selected.end(), name) != selected.end();
}
}  // namespace

std::vector<dmu_vision_interfaces::msg::Obstacle2D> select_obstacles(
  const yolo_msgs::msg::DetectionArray & detections,
  std::uint32_t image_width,
  std::uint32_t image_height,
  const ObstacleFilterOptions & options)
{
  std::vector<dmu_vision_interfaces::msg::Obstacle2D> selected;
  if (image_width == 0U || image_height == 0U) {return selected;}

  const double image_area =
    static_cast<double>(image_width) * static_cast<double>(image_height);

  for (const auto & detection : detections.detections) {
    if (detection.score < options.min_score) {continue;}
    if (!class_is_selected(detection.class_name, options.target_classes)) {continue;}

    const double width = detection.bbox.size.x;
    const double height = detection.bbox.size.y;
    if (width <= 0.0 || height <= 0.0) {continue;}

    const double cx = detection.bbox.center.position.x;
    const double cy = detection.bbox.center.position.y;
    if (cx < 0.0 || cy < 0.0 ||
      cx > static_cast<double>(image_width) || cy > static_cast<double>(image_height))
    {
      continue;
    }

    const double nx = cx / static_cast<double>(image_width);
    const double ny = cy / static_cast<double>(image_height);
    if (nx < options.roi_x_min || nx > options.roi_x_max ||
      ny < options.roi_y_min || ny > options.roi_y_max)
    {
      continue;
    }

    if ((width * height) / image_area < options.min_bbox_area_ratio) {continue;}

    dmu_vision_interfaces::msg::Obstacle2D obstacle;
    obstacle.class_id = detection.class_id;
    obstacle.class_name = detection.class_name;
    obstacle.score = detection.score;
    obstacle.track_id = detection.id;
    obstacle.center_x = cx;
    obstacle.center_y = cy;
    obstacle.width = width;
    obstacle.height = height;
    selected.push_back(std::move(obstacle));
  }

  return selected;
}

}  // namespace visual_obstacle_avoidance
