# hikrobot_camera_ros2

海康 GigE 工业相机 ROS 2 Jazzy 驱动适配包，C++17。

## 功能

- 通过 Linux MVS SDK 枚举、选择、打开和采集海康 GigE 相机。
- 把原始帧复制为 `sensor_msgs/msg/Image`。
- 同步发布 `sensor_msgs/msg/CameraInfo`。
- 未标定时发布 K/P 全零的未标定 CameraInfo，不伪造内参。
- 取流异常、超时、设备重新打开和状态发布。
- 保留 Mock 后端用于没有硬件时的显式离线开发。
- 正式启动时若真实相机不可用，打印检查清单并返回非零状态。

## 发布

| Topic | 类型 |
|---|---|
| `/camera/image_raw` | `sensor_msgs/msg/Image` |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/vision/status` | `diagnostic_msgs/msg/DiagnosticArray` |

## 启动

真实相机：

```bash
ros2 launch hikrobot_camera_ros2 camera.launch.py backend:=mvs
```

多相机必须指定序列号。启动日志会列出枚举到的型号、序列号、IP 和掩码。

离线 Mock：

```bash
ros2 launch hikrobot_camera_ros2 camera.launch.py backend:=mock
```

## 主要参数

| 参数 | 默认值 |
|---|---|
| `backend` | `mvs` |
| `require_camera_at_startup` | `true` |
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

测试覆盖图像消息字段、未标定 CameraInfo、诊断、Mock 后端、MVS 不可用错误和 `/cmd_vel` 禁止约束。真实相机采集仍需现场验收。
