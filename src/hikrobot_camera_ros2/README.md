# hikrobot_camera_ros2

海康 GigE / USB3 Vision 工业相机 ROS 2 Jazzy 驱动适配包，C++17。当前支持一台匹配相机。

## 功能

- 通过 Linux MVS SDK 枚举、打开和采集一台海康 GigE 或 USB3 Vision 相机。
- 支持 `mvs_transport=auto|gige|usb` 和 `mvs_serial_number` 选择。
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

ros2 launch hikrobot_camera_ros2 camera.launch.py mvs_transport:=auto
```

USB 相机可显式使用 `mvs_transport:=usb`。

当前驱动只支持一台相机；枚举到 0 台会失败，多台匹配设备时必须设置 `mvs_serial_number`。启动日志会列出型号、序列号、传输类型和 IP。

## 主要参数

| 参数 | 默认值 |
|---|---|
| `require_camera_at_startup` | `true` |
| `mvs_transport` | `auto` |
| `mvs_serial_number` | 空 |
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
