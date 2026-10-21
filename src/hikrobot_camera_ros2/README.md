# hikrobot_camera_ros2

海康 GigE 工业相机 ROS 2 Jazzy 驱动适配包，C++17。当前只支持一台 `MV-CS050-10GM`。

## 功能

- 通过 Linux MVS SDK 枚举、打开和采集一台海康 GigE 相机。
- 将真实采集帧复制为 `sensor_msgs/msg/Image`。
- 同步发布 `sensor_msgs/msg/CameraInfo`。
- 未标定时发布 K/P 全零的 CameraInfo，不伪造内参。
- 支持取流超时诊断、设备重新打开和状态发布。
- 真实相机不存在时打印检查清单并返回非零状态。
- 不包含 Mock、合成图像或静态图片回放。

## 发布

| Topic | 类型 |
|---|---|
| `/camera/image_raw` | `sensor_msgs/msg/Image` |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/vision/status` | `diagnostic_msgs/msg/DiagnosticArray` |

## 启动

```bash
cd ~/vision_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch hikrobot_camera_ros2 camera.launch.py
```

当前驱动只支持一台 GigE 相机；枚举到 0 台或多台都会拒绝启动。启动日志会列出型号、序列号、IP 和掩码。

## 主要参数

| 参数 | 默认值 |
|---|---|
| `require_camera_at_startup` | `true` |
| `mvs_pixel_format` | 空 |
| `mvs_exposure_us` | 0 |
| `mvs_gain` | 0 |
| `mvs_packet_size` | 0 |
| `calibration_file` | 空 |
| `qos_reliability` | `best_effort` |

完整参数见 `config/camera.yaml`。

## 测试

```bash
colcon test --packages-select hikrobot_camera_ros2
colcon test-result --verbose
```

测试覆盖图像消息字段、未标定 CameraInfo、MVS 状态、SDK 探测和 `/cmd_vel` 禁止约束。真实相机采集仍需现场验收。
