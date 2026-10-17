# 快速上手（相机 + 图像识别）

> 当前交付范围：**相机能跑 + 能识别图像**。其余（雷达融合、视觉定位、集成验收）暂不涉及。

## 1. 一条命令看效果

```bash
cd ~/vision_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch dmu_vision_bringup vision_demo.launch.py
```

启动后应该看到这些话题：

```
/camera/image_raw                 原始图像
/camera/camera_info               相机内参（未标定时 K 全零）
/yolo/cam0/detections             2D 检测结果
/yolo/cam0/tracking               跟踪结果（带 track id）
/yolo/cam0/debug_image            带检测框的可视化图
/yolo/cam0/debug_bb_markers       RViz 标记
```

**实测输出（这台 2 vCPU 虚拟机）：**

```
person   mean_score 0.832   bus  mean_score 0.882
image source 2.97 Hz   detection 2.35 Hz   debug_image 2.75 Hz (640x480 bgr8)
latency mean 131.8 ms / P95 134.8 ms
```

### ⚠️ 这条命令用的是**回放源**，不是真相机

`vision_demo.launch.py` 让 `mock` 后端把一张静态图片（`models/demo_scene.jpg`）
按固定帧率反复发布。节点会明确告知：

```
[vision_demo] REPLAY SOURCE: the mock backend republishes a still image. This is NOT a real camera capture.
[camera_node] [WARN] backend 'mock' is REPLAYING the still image '.../demo_scene.jpg'.
                      This is NOT a real camera capture and no live scene is observed.
```

`/vision/status` 里也会持续报告 `mock_source: True`。
**它的用途是证明"图像 → 检测"这条链路通，不是证明相机能用。**

## 2. 怎么确认它真的在工作

```bash
# ① 一键体检：话题 / QoS / 帧率 / 陈旧度 / frame_id 一致性 / 标定状态
ros2 run dmu_vision_tools vision_health_check --measure 5

# ② 检测性能与内容
ros2 run dmu_vision_tools measure_detection --seconds 20

# ③ 肉眼看图
ros2 run rqt_image_view rqt_image_view          # 选 /camera/image_raw 或 /yolo/cam0/debug_image
# 或
ros2 run image_view image_view --ros-args -r image:=/yolo/cam0/debug_image
```

`vision_health_check` 的退出码 0 = 无 FAIL，1 = 有 FAIL，可直接接进脚本/CI。

## 3. 数据流

```
mock(回放静态图) / 真实相机
        │
        ▼
  hikrobot_camera_ros2:camera_node
        │  /camera/image_raw + /camera/camera_info + /vision/status
        ▼
  yolo_ros:yolo_node  (detection + tracking + debug)
        │
        ├── /yolo/cam0/detections      yolo_msgs/DetectionArray
        ├── /yolo/cam0/tracking        yolo_msgs/DetectionArray
        └── /yolo/cam0/debug_image     sensor_msgs/Image（带框）
```

> 生产环境应检测**校正后**的图像（`/camera/image_rect`），配置见
> `dmu_vision_bringup/config/yolo_mono.yaml`；演示用 `yolo_mono_raw.yaml`（走原始图），
> 因为未标定时 `image_rect` 没有数据。

## 4. 接真实相机还缺什么

**代码已经就绪**（`backend:=mvs` 分支、SDK 探测、明确的错误提示都已实现）。缺的是硬件与 SDK：

| 缺什么 | 说明 |
| --- | --- |
| **Linux 版 MVS SDK（x86_64）** | 手上只有 `MVS_Win_STD_5.0.2`（Windows 版，装不进 Ubuntu）。必须从海康机器人官网下载 **Linux** 版 |
| **相机接入** | MV-CS050-10GM 是 **GigE 网口相机**，不是 USB。需要网线接到相机网段 |
| **虚拟机网络** | 当前是 NAT，GigE 广播穿不过去。需要**再加一块桥接网卡**（不要改动现有 NAT 网卡，否则会断掉 SSH/VS Code） |

SDK 到位后：

```bash
ros2 run hikrobot_camera_ros2 camera_node --ros-args -p backend:=mvs
```

若仍报 `MVS SDK not found`，它会**逐条列出搜索过的路径**，据此确认安装位置。
SDK 存在但采集未实现时，它会明确拒绝并说明原因（按项目规则，不猜 SDK API）。

## 5. 遇到问题

先看 `extras/docs/TROUBLESHOOTING.md`。最常见的一条：

**话题都在、节点也在、日志无错，但一帧数据都没有** → 九成是 QoS 不兼容。
直接跑 `vision_health_check`，它会点名是哪两个节点：

```
[FAIL] qos image   /camera_node -> /some_subscriber: publisher offers BEST_EFFORT
                   but subscriber requests RELIABLE -> no data will ever be delivered
```
