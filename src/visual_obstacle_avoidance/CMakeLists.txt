cmake_minimum_required(VERSION 3.16)
project(visual_obstacle_avoidance)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(yolo_msgs REQUIRED)
find_package(dmu_vision_interfaces REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

add_library(visual_obstacle_avoidance_core
  src/obstacle_filter.cpp
  src/visual_obstacle_node.cpp
)
target_include_directories(visual_obstacle_avoidance_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(visual_obstacle_avoidance_core PUBLIC
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  diagnostic_msgs::diagnostic_msgs
  cv_bridge::cv_bridge
  yolo_msgs::yolo_msgs
  dmu_vision_interfaces::dmu_vision_interfaces
  ${OpenCV_LIBS}
)

add_executable(visual_obstacle_node src/visual_obstacle_main.cpp)
target_link_libraries(visual_obstacle_node visual_obstacle_avoidance_core)

install(TARGETS visual_obstacle_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_obstacle_filter test/test_obstacle_filter.cpp)
  target_link_libraries(test_obstacle_filter visual_obstacle_avoidance_core)
  ament_add_gtest(test_visual_obstacle_node test/test_visual_obstacle_node.cpp)
  target_link_libraries(test_visual_obstacle_node visual_obstacle_avoidance_core)
endif()

ament_package()
