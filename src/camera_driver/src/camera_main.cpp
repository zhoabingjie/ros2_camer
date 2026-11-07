// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0

//程序入口：初始化 ROS 2，创建节点
#include <cstdio>
#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "camera_driver/camera_node.hpp"
#include "camera_driver/mvs_backend.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;//退出码，0正常
  try {
    {
      // Scoped so ~CameraNode() joins acquisition and closes the backend before
      // SDK finalization and rclcpp::shutdown().
      auto node = std::make_shared<camera_driver::CameraNode>();
      rclcpp::spin(node);
    }
  } catch (const std::exception & exc) {
    std::fprintf(stderr, "camera driver startup failed: %s\n", exc.what());
    exit_code = 1;
  }

  // MVS 5.0.2 can crash if MV_CC_Finalize runs from a static destructor after
  // its GEV plugin has begun teardown. Finalize explicitly while the process is
  // still fully alive and every camera handle has been released.
  camera_driver::finalize_mvs_sdk();//释放
  if (rclcpp::ok()) {rclcpp::shutdown();}//结束
  return exit_code;
}
