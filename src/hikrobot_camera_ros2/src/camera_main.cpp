// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "hikrobot_camera_ros2/camera_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  {
    // Scoped so ~CameraNode() (which joins the acquisition thread and closes the
    // backend) runs before rclcpp::shutdown().
    auto node = std::make_shared<hikrobot_camera_ros2::CameraNode>();
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
