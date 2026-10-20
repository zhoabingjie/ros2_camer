// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0

#include <cstdio>
#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "hikrobot_camera_ros2/camera_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    {
      // Scoped so ~CameraNode() joins acquisition and closes the backend before
      // rclcpp::shutdown().
      auto node = std::make_shared<hikrobot_camera_ros2::CameraNode>();
      rclcpp::spin(node);
    }
  } catch (const std::exception & exc) {
    std::fprintf(stderr, "camera driver startup failed: %s\n", exc.what());
    if (rclcpp::ok()) {rclcpp::shutdown();}
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
