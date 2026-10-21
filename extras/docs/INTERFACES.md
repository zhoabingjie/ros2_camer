# ROS 2 视觉接口清单

本文件以当前源码和实际 `ros2 topic list/info` 输出为准。

## 1. 节点

| 节点 | 包 | 作用 |
|---|---|---|
| `/camera_node` | `hikrobot_camera_ros2` | 真实 MVS 或显式 Mock 图像源 |
| `/yolo/yolo_node` | `yolo_ros` | YOLO 2D 检测、跟踪和 debug image |
| `/visual_obstacle_node` | `visual_obstacle_avoidance` | 二维障碍物候选筛选与状态输出 |

## 2. 相机节点

### 发布

| Topic | 类型 | QoS | 说明 |
|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/msg/Image` | BEST_EFFORT, depth 5 | 原始采集帧 |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | BEST_EFFORT, depth 5 | 未标定时 K/P 全零 |
| `/vision/status` | `diagnostic_msgs/msg/DiagnosticArray` | RELIABLE, depth 10 | 相机流状态和诊断字段 |

### 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `backend` | `mvs` | `mvs` 真实相机；`mock` 仅显式离线 |
| `require_camera_at_startup` | `true` | 后端打不开时进程返回非零 |
| `frame_id` | `camera_optical_frame` | Image 和 CameraInfo 共用 |
| `image_topic` | `/camera/image_raw` | 图像话题 |
| `camera_info_topic` | `/camera/camera_info` | 内参话题 |
| `status_topic` | `/vision/status` | 诊断话题 |
| `calibration_file` | 空 | 空表示未标定；不得伪造内参 |
| `mvs_pixel_format` | 空 | `Mono8`、`BGR8`、`RGB8` |
| `mvs_width` / `mvs_height` | 0 | 0 表示保持设备当前值 |
| `mvs_exposure_us` | 0 | 0 表示保持当前值 |
| `mvs_gain` | 0 | 0 表示保持当前值 |
| `mvs_packet_size` | 0 | 0 表示请求 SDK 推荐值 |

### 启动

```bash
ros2 launch hikrobot_camera_ros2 camera.launch.py backend:=mvs
```

## 3. YOLO 节点

正式入口把上游输出映射为稳定接口：

### 稳定发布

| Topic | 类型 | QoS |
|---|---|---|
| `/vision/detections` | `yolo_msgs/msg/DetectionArray` | RELIABLE, depth 10 |
| `/vision/tracks` | `yolo_msgs/msg/DetectionArray` | RELIABLE, depth 10 |
| `/vision/debug_image` | `sensor_msgs/msg/Image` | RELIABLE, depth 10 |
| `/vision/debug_bb_markers` | `visualization_msgs/msg/MarkerArray` | RELIABLE, depth 10 |
| `/vision/debug_kp_markers` | `visualization_msgs/msg/MarkerArray` | RELIABLE, depth 10 |

### 订阅

| Topic | 类型 | QoS |
|---|---|---|
| `/camera/image_raw` 或 `/camera/image_rect` | `sensor_msgs/msg/Image` | BEST_EFFORT |

### 关键参数

| 参数 | 当前值 |
|---|---|
| `cam0.rgb_topic` | `/camera/image_raw`（未标定） |
| `cam0.image_reliability` | `2`，即 BEST_EFFORT |
| `detection.model_path` | `/home/asd/vision_ws/models/yolo26n.onnx` |
| `detection.provider` | `cpu` |
| `detection.device` | `cpu` |
| `detection.threshold` | `0.5` |
| `detection.img_width` / `img_height` | `640` / `480` |
| `plugins` | `detection`, `tracking`, `debug` |

单目配置不包含 `detection3d`，也不包含深度话题。

## 4. 视觉障碍物节点

### 订阅

| Topic | 类型 | QoS |
|---|---|---|
| `/vision/detections` | `yolo_msgs/msg/DetectionArray` | RELIABLE, depth 10 |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | BEST_EFFORT |
| `/vision/debug_image` | `sensor_msgs/msg/Image` | BEST_EFFORT |
| `/vision/status` | `diagnostic_msgs/msg/DiagnosticArray` | RELIABLE, depth 10 |

### 发布

| Topic | 类型 | QoS |
|---|---|---|
| `/vision/obstacles` | `dmu_vision_interfaces/msg/VisualObstacleArray` | RELIABLE, depth 10 |
| `/vision/obstacle_debug_image` | `sensor_msgs/msg/Image` | BEST_EFFORT |
| `/vision/status` | `diagnostic_msgs/msg/DiagnosticArray` | RELIABLE, depth 10 |

### `VisualObstacleArray` 状态

| 值 | 名称 | 含义 |
|---|---|---|
| 0 | `UNKNOWN` | 尚未获得图像尺寸或检测数据 |
| 1 | `CLEAR` | 输入有效且 ROI 内没有目标 |
| 2 | `CANDIDATE` | 有候选，但连续帧确认不足 |
| 3 | `OBSTACLE` | ROI 内候选已确认 |
| 4 | `INPUT_TIMEOUT` | 检测流超时 |
| 5 | `CAMERA_ERROR` | 相机诊断报错 |
| 6 | `DETECTOR_ERROR` | 检测器诊断报错 |

### 关键参数

| 参数 | 默认值 |
|---|---|
| `detections_topic` | `/vision/detections` |
| `obstacles_topic` | `/vision/obstacles` |
| `target_classes` | `person`, `bicycle`, `car`, `motorcycle`, `bus`, `truck` |
| `min_score` | `0.5` |
| `roi_x_min` / `roi_x_max` | `0.25` / `0.75` |
| `roi_y_min` / `roi_y_max` | `0.35` / `1.0` |
| `min_bbox_area_ratio` | `0.005` |
| `required_consecutive_frames` | `2` |
| `input_timeout_s` | `1.0` |

## 5. 消息语义

`dmu_vision_interfaces/msg/Obstacle2D` 字段全部为图像像素坐标：

- `class_id`、`class_name`、`score`、`track_id`
- `center_x`、`center_y`
- `width`、`height`

消息中**没有**距离、深度、三维中心或速度字段。单目检测结果不能被解释为米制测距。

## 6. 旧上游接口

直接运行上游 `yolo_bringup/launch/yolo.launch.py` 时仍发布：

```text
/yolo/cam0/detections
/yolo/cam0/tracking
/yolo/cam0/debug_image
/yolo/cam0/debug_bb_markers
/yolo/cam0/debug_kp_markers
```

正式入口 `dmu_vision_bringup/vision.launch.py` 使用 `/vision/*`，两者不会互相覆盖。

## 7. 后续联调预留

以下名称仅为候选，未完成联调：

| 模块 | 候选 Topic | 类型 |
|---|---|---|
| 激光雷达 | `/scan` | `sensor_msgs/msg/LaserScan` |
| 码盘里程计 | `/wheel/odom_raw` | `nav_msgs/msg/Odometry` |
| 融合定位 | `/odometry/filtered` | 待团队确认 |
| 底盘速度 | `/cmd_vel` | 待团队确认，视觉模块不发布 |
