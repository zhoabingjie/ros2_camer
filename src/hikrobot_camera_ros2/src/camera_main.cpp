// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0

#include <cstdio>
#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "hikrobot_camera_ros2/camera_node.hpp"
#include "hikrobot_camera_ros2/mvs_backend.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    {
      // Scoped so ~CameraNode() joins acquisition and closes the backend before
      // SDK finalization and rclcpp::shutdown().
      auto node = std::make_shared<hikrobot_camera_ros2::CameraNode>();
      rclcpp::spin(node);
    }
  } catch (const std::exception & exc) {
    std::fprintf(stderr, "camera driver startup failed: %s\n", exc.what());
    exit_code = 1;
  }

  // MVS 5.0.2 can crash if MV_CC_Finalize runs from a static destructor after
  // its GEV plugin has begun teardown. Finalize explicitly while the process is
  // still fully alive and every camera handle has been released.
  hikrobot_camera_ros2::finalize_mvs_sdk();
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return exit_code;
}
