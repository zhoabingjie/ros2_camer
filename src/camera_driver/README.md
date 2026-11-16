# camera_driver
海康机器人 MVS 单相机 ROS 2 Jazzy 驱动，C++17。

本包只负责真实相机采集、曝光/增益、图像发布、CameraInfo、诊断和 USB 拔插重连.

## 1. 版本与环境

| 项目 | 已确认值 |
|---|---|
| 操作系统 | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| CPU 架构 | x86_64 |
| MVS 客户端/安装包 | 5.0.2 |
| MVS 运行时 SDK | 4.8.1.2 |
| MVS 主库 | `/opt/MVS/lib/64/libMvCameraControl.so.4.8.1.2` |
| USB3 Vision 传输库 | `/opt/MVS/lib/64/libMvUsb3vTL.so.4.8.0.2` |
| USB3 Vision 插件 | `/opt/MVS/lib/64/MvProducerU3V.cti` |

## 2. 兼容策略

- 默认 `mvs_transport: auto`，自动匹配 GigE 或 USB3 Vision 相机。
- 代码支持 `mvs_serial_number: ""`，只有一台相机时自动打开。
- 当前 `camera.yaml` 已固定为真实序列号 `DB0883068`，USB 重新枚举后仍选择原相机。
- 默认 `mvs_pixel_format: ""`，根据相机实际支持的格式自动选择。
- 默认 `image_encoding: auto`：
  - 有 BGR8/RGB8/Bayer8 时优先输出 `bgr8`
  - 只有 Mono8 时自动输出 `mono8`
- 显式设置 `image_encoding: bgr8` 才会强制彩色；单色相机无法满足时会明确失败。

运行时实际结果：

| 相机能力 | `/camera/image_raw` 编码 |
|---|---|
| BGR8 | `bgr8` |
| RGB8 | `bgr8` |
| BayerRG8/BG8/GB8/GR8 | `bgr8` |
| 只有 Mono8 | `mono8` |
| 其他可通过 MVS 转换的格式 | 尝试 `bgr8`，失败后尝试 `mono8` |

已验证设备：

| 项目 | 实测值 |
|---|---|
| 型号 | `MV-CS050-10UC` |
| 序列号 | `DB0883068` |
| USB VID:PID | `2bdf:0001` |

当前配置使用该序列号固定绑定设备；更换相机时只需修改
`config/camera.yaml` 中的 `mvs_serial_number`。

## 3. USB 身份与重新枚举

物理 USB 拔出后，旧 MVS handle 已经失效，必须关闭并重新枚举设备，无法安全复用。
驱动会自动完成这一步，不需要用户手动操作。

设备索引可能变化，但驱动不依赖索引，而是通过硬件序列号 `DB0883068` 重新匹配原相机。
因此短时间拔插后会恢复到同一台相机，并重新应用像素格式、曝光和增益。

## 4. 目录结构

```text
camera_driver/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── camera.yaml
├── launch/
│   └── camera.launch.py
├── include/camera_driver/
│   ├── camera_backend.hpp
│   ├── camera_node.hpp
│   ├── frame.hpp
│   ├── mvs_backend.hpp
│   └── stream_health.hpp
├── src/
│   ├── camera_backend.cpp
│   ├── camera_main.cpp
│   ├── camera_node.cpp
│   ├── frame.cpp
│   ├── mvs_backend.cpp
│   └── stream_health.cpp
└── test/
```

## 5. 编译与启动

```bash
cd ~/vision_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select camera_driver
source install/setup.bash
ros2 launch camera_driver camera.launch.py
```

启动前关闭可能独占相机的 MVS 客户端。

## 6. Topic

| Topic | 类型 | 说明 |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/msg/Image` | 真实相机图像 |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | CameraInfo |
| `/vision/status` | `diagnostic_msgs/msg/DiagnosticArray` | 状态诊断 |

图像和 CameraInfo 默认使用：

```text
Reliability: RELIABLE
History: KeepLast
Depth: 5
Durability: VOLATILE
```

该配置可以直接连接默认 RViz2 Image Display。

CameraInfo 始终发布图像时间戳、frame_id、宽度和高度，但不加载第三方标定文件，
`K/R/P` 保持全零，`D` 为空。这些数据不能用于去畸变、测距或视觉定位。

图像编码可能是：

```text
bgr8
mono8
```

检查：

```bash
ros2 topic echo --once /camera/image_raw --field encoding
ros2 topic echo --once /camera/image_raw --field width
ros2 topic echo --once /camera/image_raw --field height
ros2 topic hz /camera/image_raw
```

## 7. 相机参数

| 参数 | 默认值 | 运行中可改 | 说明 |
|---|---|---|---|
| `frame_id` | `camera_optical_frame` | 是 | 后续 Image/CameraInfo 使用的坐标系 |
| `mvs_transport` | `auto` | 否 | `auto`、`usb`、`gige` |
| `mvs_serial_number` | `DB0883068` | 否 | 按真实序列号固定绑定当前相机 |
| `mvs_pixel_format` | 空 | 是 | 空值自动选择；可指定 Mono8/BGR8/RGB8/Bayer8 |
| `mvs_width` | `0` | 是 | 在线修改需要内部暂停并恢复取流；0 表示保持当前值 |
| `mvs_height` | `0` | 是 | 在线修改需要内部暂停并恢复取流；0 表示保持当前值 |
| `mvs_packet_size` | `0` | 否 | 仅 GigE；0 表示使用 SDK 推荐值 |
| `image_encoding` | `auto` | 是 | `auto`、`bgr8`、`mono8` |
| `exposure_mode` | `auto` | 是 | `auto`、`manual` 或 `camera` |
| `exposure_auto_mode` | `continuous` | 是 | `continuous` 或 `once` |
| `exposure_time_us` | `8000.0` | 是 | 手动曝光，单位微秒 |
| `gain_mode` | `auto` | 是 | `auto`、`manual` 或 `camera` |
| `gain_auto_mode` | `continuous` | 是 | `continuous` 或 `once` |
| `gain_value` | `0.0` | 是 | 手动增益，0.0 合法 |
| `stream_timeout_s` | `2.0` | 是 | 无新帧判定为 stalled 的时间 |
| `grab_timeout_s` | `0.2` | 是 | 单次取帧等待时间 |
| `reconnect_backoff_s` | `1.0` | 是 | 重连间隔 |
| `require_camera_at_startup` | `false` | 否 | 无相机时是否退出 |
| `reconnect_after_misses` | `10` | 是 | 连续超时多少次后重新枚举 |
| `qos_reliability` | `reliable` | 否 | Publisher 创建后不能直接修改 |

### 支持运行时在线修改

以下参数使用 `ros2 param set` 修改后立即生效，不需要重启节点或拔插相机：

```text
frame_id

mvs_pixel_format
mvs_width
mvs_height
image_encoding

exposure_mode
exposure_auto_mode
exposure_time_us

gain_mode
gain_auto_mode
gain_value

stream_timeout_s
grab_timeout_s
reconnect_backoff_s
reconnect_after_misses
```


### 修改后必须重启节点

以下参数在节点或 Publisher 创建时确定，使用 `ros2 param set` 不会完整更新当前运行状态，


```text
image_topic
camera_info_topic
status_topic

qos_reliability
qos_depth
status_period_s

mvs_transport
mvs_serial_number
mvs_packet_size
sdk_lib_candidates

require_camera_at_startup
```

重启命令：

```bash
cd ~/vision_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select camera_driver
source install/setup.bash
ros2 launch camera_driver camera.launch.py
```

在线调试示例：

```bash
ros2 param set /camera_node frame_id camera_link
ros2 param set /camera_node mvs_width 1280
ros2 param set /camera_node mvs_height 1024

ros2 param set /camera_node grab_timeout_s 0.3
ros2 param set /camera_node stream_timeout_s 1.5
ros2 param set /camera_node reconnect_backoff_s 0.5
ros2 param set /camera_node reconnect_after_misses 5
```

## 8. 曝光和增益

```bash
ros2 param set /camera_node exposure_mode auto
ros2 param set /camera_node exposure_auto_mode continuous
ros2 param set /camera_node gain_mode auto
ros2 param set /camera_node gain_auto_mode continuous
```

手动设置：

```bash
ros2 param set /camera_node exposure_mode manual
ros2 param set /camera_node exposure_time_us 8000.0
ros2 param set /camera_node gain_mode manual
ros2 param set /camera_node gain_value 0.0
```

驱动会查询相机支持的自动模式和数值范围。相机没有自动曝光或自动增益能力时，
会保留相机当前设置，不会因为功能不存在而导致整个驱动启动失败。

也可以显式设置：

```bash
ros2 param set /camera_node exposure_mode camera
ros2 param set /camera_node gain_mode camera
```

`camera` 表示不修改相机当前曝光/增益设置。

## 9. USB 拔插重连

以下行为保持不变：

1. 运行中 USB 拔出或持续取流超时。
2. 关闭旧设备句柄并清空旧缓冲。
3. 重新执行 MVS 设备枚举。
4. USB 重新透传到虚拟机后重新打开相机。
5. 重新配置像素格式、宽度、高度、曝光和增益。
6. 恢复 `/camera/image_raw`。

相同错误只打印一次，后台仍持续重连；恢复成功后打印恢复信息。

## 10. 状态诊断

`/vision/status` 包含：

```text
camera_connected
transport
model
serial_number
pixel_format
image_encoding
width
height
exposure_mode
exposure_time_us
gain_mode
gain_value
measured_fps
reconnect_count
last_error
```


## 11. 明确失败条件

以下情况会失败，其余普通相机能力差异会尽量自动适配：

- MVS SDK 枚举不到任何相机。
- 枚举到多台相机但没有配置 `mvs_serial_number`。
- 显式设置 `image_encoding: bgr8`，但相机不支持任何彩色格式。
- 手动曝光或手动增益超出相机实际允许范围。
- MVS 客户端或其他程序独占了相机。
