# ros2_camer — ROS 2 Jazzy 单目视觉子系统

适用环境：Ubuntu 24.04 + ROS 2 Jazzy + C++17。当前相机：海康 `MV-CS050-10GM`（GigE）。

## 当前状态（2026-10-20）

- Linux MVS x86_64 SDK 已安装并可加载。
- 相机驱动支持真实 MVS 后端和设备枚举，可打印型号、序列号、IP。
- 正式入口默认使用真实 MVS；相机缺失时明确报错并以非零状态退出。
- Mock 后端保留，但必须显式指定。
- 图像预览与相机驱动解耦。
- YOLO 2D 检测、跟踪和 debug image 已接入。
- 新增二维视觉障碍物接口和节点，不输出伪造距离，不控制 `/cmd_vel`。
- 完整构建：10 个包通过。
- 完整测试：`404 tests / 0 failures / 0 errors / 10 skipped`。
- Mock 端到端验证：图像约 `7 Hz`；实际输出 `bus 0.882`、`person 0.831`；`/vision/obstacles` 状态为 `OBSTACLE`。
- **真实相机取流尚未验收**：当前 VMware 没有相机用桥接网卡，MVS 实测 `devices=0`。

## 正式启动

```bash
cd ~/vision_ws
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --parallel-workers 2 \
  --cmake-args \
  -DONNXRUNTIME_ROOT=$HOME/vision_ws/thirdparty/onnxruntime-linux-x64-1.20.0 \
  -DFETCHCONTENT_SOURCE_DIR_YOLO_HFHUB=$HOME/vision_ws/thirdparty/huggingface-hub-cpp

source install/setup.bash
ros2 launch dmu_vision_bringup vision.launch.py
```

只启动真实相机：

```bash
ros2 launch hikrobot_camera_ros2 camera.launch.py backend:=mvs
```

多台 GigE 相机必须指定序列号：

```bash
ros2 launch hikrobot_camera_ros2 camera.launch.py \
  backend:=mvs mvs_serial_number:=你的相机序列号
```

## 查看图像和检测结果

另开终端：

```bash
source /opt/ros/jazzy/setup.bash
source ~/vision_ws/install/setup.bash

# 原始相机图像
ros2 launch dmu_vision_bringup image_viewer.launch.py \
  image_topic:=/camera/image_raw

# YOLO 检测框
ros2 launch dmu_vision_bringup image_viewer.launch.py \
  image_topic:=/vision/debug_image

# 障碍物 ROI 和候选框
ros2 launch dmu_vision_bringup image_viewer.launch.py \
  image_topic:=/vision/obstacle_debug_image
```

终端检查：

```bash
ros2 topic list
ros2 topic hz /camera/image_raw
ros2 topic echo --once /camera/camera_info
ros2 topic echo --once /vision/detections
ros2 topic echo --once /vision/obstacles
ros2 run dmu_vision_tools vision_health_check --measure 5
```

## 显式离线 Mock

```bash
ros2 launch dmu_vision_bringup vision.launch.py \
  backend:=mock \
  mock_image_path:=$HOME/vision_ws/models/demo_scene.jpg \
  mock_encoding:=bgr8 \
  mock_fps:=8.0 \
  start_rectify:=false
```

旧回放演示仍保留：

```bash
ros2 launch dmu_vision_bringup vision_demo.launch.py
```

## 稳定接口

| 功能 | Topic | 类型 |
|---|---|---|
| 原始图像 | `/camera/image_raw` | `sensor_msgs/msg/Image` |
| 相机参数 | `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` |
| 校正图像 | `/camera/image_rect` | `sensor_msgs/msg/Image` |
| YOLO 检测 | `/vision/detections` | `yolo_msgs/msg/DetectionArray` |
| 跟踪结果 | `/vision/tracks` | `yolo_msgs/msg/DetectionArray` |
| 检测调试图 | `/vision/debug_image` | `sensor_msgs/msg/Image` |
| 障碍物结果 | `/vision/obstacles` | `dmu_vision_interfaces/msg/VisualObstacleArray` |
| 障碍物调试图 | `/vision/obstacle_debug_image` | `sensor_msgs/msg/Image` |
| 视觉状态 | `/vision/status` | `diagnostic_msgs/msg/DiagnosticArray` |

详细说明见 [INTERFACES.md](extras/docs/INTERFACES.md)。

## 包结构

| 包 | 作用 |
|---|---|
| `hikrobot_camera_ros2` | 海康 GigE 相机驱动、Mock 后端、Image/CameraInfo/诊断发布 |
| `dmu_vision_bringup` | 正式统一启动、预览、标定和参数编排 |
| `dmu_vision_interfaces` | 团队稳定消息接口 |
| `visual_obstacle_avoidance` | 单目二维障碍物筛选和状态输出 |
| `dmu_vision_tools` | 健康检查、检测测量和稳定性工具 |
| `yolo_ros` | 上游 YOLO ONNX 推理、跟踪和 debug 插件 |
| `yolo_msgs` | 上游检测、跟踪和关键点消息 |
| `yolo_bringup` | 上游 YOLO 启动与参数入口 |
| `yolo_onnxruntime_vendor` | ONNX Runtime 构建封装 |
| `yolo_hfhub_vendor` | Hugging Face 下载依赖封装 |

## 安全边界

- 单目二维框不能提供真实米制距离。
- 不启用 `detection3d`。
- 不伪造内参、深度、距离或定位结果。
- 视觉模块不发布 `/cmd_vel`，不接管 Nav2 或底盘控制。
- 雷达、码盘、定位和 Nav2 联调仍待团队接口确认。

## 文档

- `extras/docs/ENVIRONMENT_AUDIT.md`：环境与源码审计。
- `extras/docs/CAMERA_DRIVER.md`：相机接入和 GigE 排查。
- `extras/docs/INTERFACES.md`：节点、Topic、QoS 和参数。
- `extras/docs/MIGRATION.md`：旧接口到稳定接口的迁移说明。
- `extras/docs/QUICKSTART.md`：快速启动。
- `extras/docs/TEST_REPORT.md`：真实测试记录。
- `extras/docs/TROUBLESHOOTING.md`：故障排查。
