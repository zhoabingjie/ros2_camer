# 快速上手

当前正式入口默认真实海康 MVS 相机；只有显式指定 `backend:=mock` 才使用离线图像。

## 1. 构建

```bash
cd ~/vision_ws
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --parallel-workers 2 \
  --cmake-args \
  -DONNXRUNTIME_ROOT=$HOME/vision_ws/thirdparty/onnxruntime-linux-x64-1.20.0 \
  -DFETCHCONTENT_SOURCE_DIR_YOLO_HFHUB=$HOME/vision_ws/thirdparty/huggingface-hub-cpp

source install/setup.bash
```

## 2. 启动真实相机 + YOLO + 障碍物

```bash
ros2 launch dmu_vision_bringup vision.launch.py
```

默认接口：

```text
/camera/image_raw
/camera/camera_info
/vision/detections
/vision/tracks
/vision/debug_image
/vision/obstacles
/vision/obstacle_debug_image
/vision/status
```

如果 MVS 找不到相机，进程会打印可操作错误并返回非零，不会退回 Mock。

## 3. 查看实时图像

另开终端：

```bash
source /opt/ros/jazzy/setup.bash
source ~/vision_ws/install/setup.bash

ros2 launch dmu_vision_bringup image_viewer.launch.py \
  image_topic:=/camera/image_raw
```

查看 YOLO 检测框：

```bash
ros2 launch dmu_vision_bringup image_viewer.launch.py \
  image_topic:=/vision/debug_image
```

查看障碍物 ROI：

```bash
ros2 launch dmu_vision_bringup image_viewer.launch.py \
  image_topic:=/vision/obstacle_debug_image
```

## 4. 检查链路

```bash
ros2 node list
ros2 topic list
ros2 topic info -v /camera/image_raw
ros2 topic info -v /camera/camera_info
ros2 topic hz /camera/image_raw
ros2 topic echo --once /camera/camera_info
ros2 topic echo --once /vision/detections
ros2 topic echo --once /vision/obstacles
ros2 run dmu_vision_tools vision_health_check --measure 5
```

## 5. 只启动相机

```bash
ros2 launch hikrobot_camera_ros2 camera.launch.py backend:=mvs
```

多相机时：

```bash
ros2 launch hikrobot_camera_ros2 camera.launch.py \
  backend:=mvs mvs_serial_number:=你的序列号
```

## 6. 显式离线 Mock

```bash
ros2 launch dmu_vision_bringup vision.launch.py \
  backend:=mock \
  mock_image_path:=$HOME/vision_ws/models/demo_scene.jpg \
  mock_encoding:=bgr8 \
  mock_fps:=8.0 \
  start_rectify:=false
```

旧回放 Demo：

```bash
ros2 launch dmu_vision_bringup vision_demo.launch.py
```

## 7. 当前硬件状态

- Linux MVS SDK 已安装。
- 当前 MVS 枚举：`devices=0`。
- 当前 VMware 没有相机桥接网卡，宿主 Realtek 以太网仍为 `Disconnected / 0 bps`。
- 因此真实相机取流、真实预览和标定仍未验收。

## 8. 常见错误

### `no Hikvision camera found`

依次检查：

1. 相机是否使用正确的 12V/24V 或 PoE 供电。
2. 网线两端是否插紧，电脑网口灯是否亮。
3. Windows 网口是否显示 `Up` 和有效链路速率。
4. VMware 是否增加第二块 Bridged 网卡，且没有改动 NAT 网卡。
5. 主机和相机是否在同一可通信网段。
6. 多相机时是否设置 `mvs_serial_number`。

### YOLO 订阅不到图像

检查：

```bash
ros2 topic info -v /camera/image_raw
ros2 topic info -v /vision/detections
```

相机图像 QoS 是 BEST_EFFORT；订阅端必须兼容。

### 未标定

`calibration_file` 为空时 `/camera/camera_info` 的 K/P 全零，这是诚实标记。此时不要使用 `/camera/image_rect`，也不要进行几何测距。
