cmake_minimum_required(VERSION 3.16)
project(hikrobot_camera_ros2)

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
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(Threads REQUIRED)

# The Hikrobot MVS SDK is optional: without it the package still builds and the
# mock backend works. Detection is done at configure time so a machine without
# the SDK never needs the vendor headers.
set(HIKROBOT_MVS_INCLUDE_DIR "" CACHE PATH "Hikrobot MVS SDK include dir")
if(NOT HIKROBOT_MVS_INCLUDE_DIR)
  find_path(HIKROBOT_MVS_INCLUDE_DIR_FOUND
    NAMES MvCameraControl.h
    HINTS
      ENV MVCAM_SDK_PATH
      ENV MVS_SDK_ROOT
      ENV HIKROBOT_MVS_ROOT
    PATH_SUFFIXES include include/x86_64 include/aarch64
    PATHS
      /opt/MVS
      /usr
      /usr/local
  )
  if(HIKROBOT_MVS_INCLUDE_DIR_FOUND)
    set(HIKROBOT_MVS_INCLUDE_DIR "${HIKROBOT_MVS_INCLUDE_DIR_FOUND}" CACHE PATH
      "Hikrobot MVS SDK include dir" FORCE)
  endif()
endif()

if(HIKROBOT_MVS_INCLUDE_DIR AND EXISTS "${HIKROBOT_MVS_INCLUDE_DIR}/MvCameraControl.h")
  message(STATUS "Hikrobot MVS SDK headers found: ${HIKROBOT_MVS_INCLUDE_DIR} -> real camera support ON")
  add_compile_definitions(HIKROBOT_MVS_AVAILABLE)
endif()

add_library(hikrobot_camera_ros2_core
  src/frame.cpp
  src/camera_backend.cpp
  src/stream_health.cpp
  src/backend_registry.cpp
  src/mock_backend.cpp
  src/mvs_backend.cpp
  src/camera_info_factory.cpp
  src/camera_node.cpp
)
target_include_directories(hikrobot_camera_ros2_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${HIKROBOT_MVS_INCLUDE_DIR}
)
if(HIKROBOT_MVS_INCLUDE_DIR)
  target_compile_definitions(hikrobot_camera_ros2_core PRIVATE
    HIKROBOT_MVS_CONFIGURED_INCLUDE_DIR="${HIKROBOT_MVS_INCLUDE_DIR}"
  )
else()
  message(STATUS "Hikrobot MVS SDK headers NOT found -> real camera backend stays probe-only")
endif()

target_link_libraries(hikrobot_camera_ros2_core PUBLIC
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  diagnostic_msgs::diagnostic_msgs
  yaml-cpp
  ${OpenCV_LIBS}
  ${CMAKE_DL_LIBS}
  Threads::Threads
)

add_executable(camera_node src/camera_main.cpp)
target_link_libraries(camera_node hikrobot_camera_ros2_core)

install(TARGETS camera_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_frame test/test_frame.cpp)
  target_link_libraries(test_frame hikrobot_camera_ros2_core)

  ament_add_gtest(test_stream_health test/test_stream_health.cpp)
  target_link_libraries(test_stream_health hikrobot_camera_ros2_core)

  ament_add_gtest(test_mock_backend test/test_mock_backend.cpp)
  target_link_libraries(test_mock_backend hikrobot_camera_ros2_core)

  ament_add_gtest(test_camera_info_factory test/test_camera_info_factory.cpp)
  target_link_libraries(test_camera_info_factory hikrobot_camera_ros2_core)

  ament_add_gtest(test_mvs_backend test/test_mvs_backend.cpp)
  target_link_libraries(test_mvs_backend hikrobot_camera_ros2_core)

  ament_add_gtest(test_camera_node test/test_camera_node.cpp)
  target_link_libraries(test_camera_node hikrobot_camera_ros2_core)
  set_tests_properties(test_camera_node PROPERTIES TIMEOUT 120)
endif()

ament_package()
