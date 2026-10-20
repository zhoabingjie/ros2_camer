// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0

#include <cstdio>
#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "visual_obstacle_avoidance/visual_obstacle_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<visual_obstacle_avoidance::VisualObstacleNode>();
    rclcpp::spin(node);
  } catch (const std::exception & exc) {
    std::fprintf(stderr, "visual_obstacle_node failed: %s\n", exc.what());
    if (rclcpp::ok()) {rclcpp::shutdown();}
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
