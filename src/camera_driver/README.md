# camera_driver

海康 GigE / USB3 Vision 工业相机 ROS 2 Jazzy 驱动包，C++17。

本包只负责相机驱动、参数设置、图像发布、CameraInfo 和诊断，不包含 YOLO、视觉障碍物或其它算法。

## 1. 已确认版本

| 项目 | 实测值 |
|---|---|
| 操作系统 | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| 相机架构 | x86_64 |
| MVS 安装包/客户端 | 5.0.2 |
| MVS 运行时 SDK | 4.8.1.2 |
| MVS 主库 | `/opt/MVS/lib/64/libMvCameraControl.so.4.8.1.2` |
| USB3 Vision 传输库 | `/opt/MVS/lib/64/libMvUsb3vTL.so.4.8.0.2` |
| USB3 Vision 插件 | `/opt/MVS/lib/64/MvProducerU3V.cti` |

检查 SDK：

```bash
readlink -f /opt/MVS/lib/64/libMvCameraControl.so
```

运行时日志应显示：

```text
sdk_version=4.8.1.2
```

## 2. 已实测相机信息

| 项目 | 实测值 |
|---|---|
| 相机型号 | Hikrobot MV-CS050-10UC |
| 接入方式 | USB3 Vision |
| USB 标识 | `2bdf:0001` |
| 分辨率 | 2448 x 2048 |
| 实测编码 | `mono8` |
| 传输类型 | `usb` |

检查 USB：

```bash
lsusb
```

应看到类似：

```text
ID 2bdf:0001 Hikrobot MV-CS050-10UC
```

## 3. 功能和话题

- 枚举、打开和采集一台 GigE 或 USB3 Vision 相机。
- 支持 `mvs_transport=auto|gige|usb` 和序列号选择。
- 支持像素格式、宽高、曝光、增益和 GigE 包大小设置。
- 支持 Bayer 彩色格式转换为 ROS `bgr8`。
- 相机缺失时错误退出，不生成 Mock 或假图像。

| Topic | 类型 | 说明 |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/msg/Image` | 相机图像 |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 相机参数 |
| `/vision/status` | `diagnostic_msgs/msg/DiagnosticArray` | 相机状态 |

默认图像 QoS 是 `BEST_EFFORT`。

## 4. 编译

```bash
cd ~/vision_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select camera_driver
source install/setup.bash
```

## 5. USB 相机启动

先在 VMware 中将海康 USB 设备连接到虚拟机，并关闭 MVS 客户端。

```bash
lsusb
```

启动：

```bash
ros2 launch camera_driver camera.launch.py \
  mvs_transport:=usb
```

自动识别：

```bash
ros2 launch camera_driver camera.launch.py \
  mvs_transport:=auto
```

GigE：

```bash
ros2 launch camera_driver camera.launch.py \
  mvs_transport:=gige
```

多台匹配设备：

```bash
ros2 launch camera_driver camera.launch.py \
  mvs_transport:=usb \
  mvs_serial_number:=实际序列号
```

正常日志：

```text
MVS backend opened
stream state -> ok
```

## 6. 检查图像

```bash
ros2 topic list | grep camera
ros2 topic hz /camera/image_raw
ros2 topic echo --once /camera/camera_info
ros2 topic echo --once /camera/image_raw --field width
ros2 topic echo --once /camera/image_raw --field height
ros2 topic echo --once /camera/image_raw --field encoding
```

## 7. 可调参数

| 参数 | 默认值 | 作用 |
|---|---|---|
| `mvs_transport` | `auto` | `auto`、`gige`、`usb` |
| `mvs_serial_number` | 空 | 多台相机时选择序列号 |
| `mvs_pixel_format` | 空 | 像素格式，空表示保持相机设置 |
| `mvs_width` | `0` | 0 表示保持当前宽度 |
| `mvs_height` | `0` | 0 表示保持当前高度 |
| `mvs_exposure_us` | `0.0` | 曝光微秒；0 表示不修改 |
| `mvs_gain` | `0.0` | 增益；0 表示不修改 |
| `mvs_packet_size` | `0` | 仅 GigE；0 表示 SDK 推荐值 |
| `frame_id` | `camera_optical_frame` | 图像坐标系 |
| `calibration_file` | 空 | 标定 YAML |
| `qos_reliability` | `best_effort` | 图像 QoS |
| `qos_depth` | `5` | 发布队列深度 |
| `require_camera_at_startup` | `true` | 相机失败时退出 |
| `stream_timeout_s` | `2.0` | 断流判定时间 |
| `grab_timeout_s` | `0.2` | 单次取帧超时 |
| `reconnect_backoff_s` | `1.0` | 重连间隔 |
| `status_period_s` | `1.0` | 状态发布周期 |
| `sdk_lib_candidates` | `[""]` | 额外 MVS 动态库路径 |

完整默认值见 `config/camera.yaml`。

## 8. 像素格式和彩色

支持的 `mvs_pixel_format`：

```text
Mono8
RGB8
BGR8
BayerRG8
BayerBG8
BayerGB8
BayerGR8
```

对应输出：

- `Mono8` -> ROS `mono8`
- `RGB8` -> ROS `rgb8`
- `BGR8` -> ROS `bgr8`
- Bayer -> 驱动转换为 ROS `bgr8`

测试 RGB8：

```bash
ros2 launch camera_driver camera.launch.py \
  mvs_transport:=usb \
  mvs_pixel_format:=RGB8
```

如果相机不支持 RGB8，测试 Bayer：

```bash
ros2 launch camera_driver camera.launch.py \
  mvs_transport:=usb \
  mvs_pixel_format:=BayerRG8
```

检查编码：

```bash
ros2 topic echo --once /camera/image_raw --field encoding
```

彩色输出应为 `bgr8` 或 `rgb8`。

注意：如果相机当前设置为 Mono8，驱动不会凭空恢复彩色，必须选择相机支持的彩色像素格式。

## 9. 分辨率和 ROI

默认保持当前分辨率：

```yaml
mvs_width: 0
mvs_height: 0
```

确认默认图像正常后再设置目标宽高。下面 1280x1024 只是命令格式示例，具体值必须由 MVS 属性页确认：

```bash
ros2 launch camera_driver camera.launch.py \
  mvs_transport:=usb \
  mvs_width:=1280 \
  mvs_height:=1024
```

注意：当前驱动只设置 Width/Height，不设置 OffsetX/OffsetY。若设置失败，先在 MVS 中调整 ROI 偏移到合法值。修改分辨率后旧标定失效。

## 10. 曝光调试

默认：

```yaml
mvs_exposure_us: 0.0
```

0 表示不修改当前曝光，可能是自动曝光。

设置大于 0 时，驱动会关闭 `ExposureAuto` 并设置 `ExposureTime`：

```bash
ros2 launch camera_driver camera.launch.py \
  mvs_transport:=usb \
  mvs_exposure_us:=8000.0
```

8000 表示 8 ms。实际范围以 MVS 为准。画面暗先增加曝光；过亮先降低曝光；运动拖影时降低曝光。曝光不能接近帧周期，否则帧率会下降。

## 11. 增益调试

默认：

```yaml
mvs_gain: 0.0
```

0 表示不修改当前增益。设置大于 0 时，驱动会关闭 `GainAuto` 并设置 `Gain`：

```bash
ros2 launch camera_driver camera.launch.py \
  mvs_transport:=usb \
  mvs_gain:=0.5
```

优先调整曝光，最后再增加增益。增益越高噪声越大。若需要固定 0 dB，先在 MVS 中设置 GainAuto=Off、Gain=0，然后 ROS 保持 `mvs_gain:=0.0`。

## 12. 推荐调试顺序

1. 默认参数启动，确认相机出图。
2. 在 MVS 中确认 Width、Height、OffsetX、OffsetY 和 PixelFormat。
3. 先调分辨率。
4. 再调曝光，避免过暗、过曝和运动拖影。
5. 最后少量调增益。
6. 需要彩色时设置 RGB8 或 Bayer 格式。
7. 检查 `/camera/image_raw` 的 width、height、encoding 和实际帧率。

完整视觉流程：

```bash
ros2 launch dmu_vision_bringup vision.launch.py \
  mvs_transport:=usb
```

## 13. 常见错误

### `devices=0`

虚拟机没有枚举到相机。检查 VMware USB 透传、`lsusb`、供电和 MVS 客户端占用。

### `no MVS camera matched transport`

相机已枚举，但 `mvs_transport` 或 `mvs_serial_number` 不匹配。

### `MV_CC_OpenDevice failed`

相机可能被 MVS 客户端占用、USB 权限不足或设备状态异常。

### `MVS setting ignored: Width/Height`

分辨率或 ROI/Offset 组合不合法，先在 MVS 中修正。

### `MVS setting ignored: ExposureTime/Gain`

参数超出相机支持范围。把对应值改回 0，确认默认启动正常后再测试合法值。

### `stream state -> stalled`

之前有图，现在断流。检查 USB、带宽、曝光、线缆和相机状态。

## 14. 测试

```bash
colcon test --packages-select camera_driver
colcon test-result --verbose
```

真实图像质量、彩色输出和参数范围仍需连接相机验收。

## 15. 旧包说明

旧包 `hikrobot_camera_ros2` 保留在 `src/hikrobot_camera_ros2`，没有删除。正式相机入口使用 `camera_driver`，旧包用于兼容和对照。
