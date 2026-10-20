# visual_obstacle_avoidance

单目二维视觉障碍物感知包。它只判断 YOLO 检测框是否落在车辆前方关注区域，不计算距离，不发布 `/cmd_vel`。

## 输入

| Topic | 类型 | 说明 |
|---|---|---|
| `/vision/detections` | `yolo_msgs/msg/DetectionArray` | YOLO 2D 检测 |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 图像宽高 |
| `/vision/debug_image` | `sensor_msgs/msg/Image` | 可选，用于叠加障碍物可视化 |
| `/vision/status` | `diagnostic_msgs/msg/DiagnosticArray` | 读取相机/检测器异常状态 |

## 输出

| Topic | 类型 | 说明 |
|---|---|---|
| `/vision/obstacles` | `dmu_vision_interfaces/msg/VisualObstacleArray` | 状态和二维障碍物候选 |
| `/vision/obstacle_debug_image` | `sensor_msgs/msg/Image` | ROI、状态和障碍物框可视化 |
| `/vision/status` | `diagnostic_msgs/msg/DiagnosticArray` | 输入超时、相机异常、检测器异常和输出状态 |

## 状态

- `UNKNOWN`：等待图像尺寸或检测数据。
- `CLEAR`：输入有效且关注区域内无目标。
- `CANDIDATE`：有候选目标，但连续帧数不足。
- `OBSTACLE`：候选目标已通过连续帧确认。
- `INPUT_TIMEOUT`：检测输入超时。
- `CAMERA_ERROR`：相机诊断报错。
- `DETECTOR_ERROR`：YOLO/检测器诊断报错。

## 启动

```bash
ros2 launch visual_obstacle_avoidance visual_obstacle.launch.py
```

## 安全边界

- 只输出二维像素坐标，不输出米制距离或三维坐标。
- 不直接控制底盘。
- 下游若需要真实距离，必须接入经过标定的雷达、深度相机或其他测距来源。
