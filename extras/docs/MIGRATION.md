# 接口与命名迁移说明

本次改造优先保护现有代码和命令，没有进行破坏性包重命名。

## 1. Package 映射

| 旧 Package | 新 Package | 处理 |
|---|---|---|
| `hikrobot_camera_ros2` | `hikrobot_camera_ros2` | 保持不变 |
| `dmu_vision_bringup` | `dmu_vision_bringup` | 保持不变 |
| `dmu_vision_tools` | `dmu_vision_tools` | 保持不变 |
| `yolo_ros` | `yolo_ros` | 上游保持不变 |
| `yolo_msgs` | `yolo_msgs` | 上游保持不变 |
| `yolo_bringup` | `yolo_bringup` | 上游保持不变 |
| 无 | `dmu_vision_interfaces` | 新增稳定消息包 |
| 无 | `visual_obstacle_avoidance` | 新增障碍物感知包 |

## 2. 节点映射

| 旧节点 | 新节点 | 兼容方式 |
|---|---|---|
| `/camera_node` | `/camera_node` | 保持不变 |
| `/yolo/yolo_node` | `/yolo/yolo_node` | 保持不变 |
| 无 | `/visual_obstacle_node` | 新增 |

没有为了命名美观而修改已有节点名。

## 3. Topic 映射

### 相机

以下 Topic 保持不变：

```text
/camera/image_raw
/camera/camera_info
/vision/status
```

### YOLO

上游旧 Topic 仍可通过 `yolo_bringup` 原始启动方式使用：

```text
/yolo/cam0/detections
/yolo/cam0/tracking
/yolo/cam0/debug_image
/yolo/cam0/debug_bb_markers
/yolo/cam0/debug_kp_markers
```

新增正式入口提供稳定 Topic：

```text
/yolo/cam0/detections   -> /vision/detections
/yolo/cam0/tracking     -> /vision/tracks
/yolo/cam0/debug_image  -> /vision/debug_image
/yolo/cam0/debug_bb_markers -> /vision/debug_bb_markers
/yolo/cam0/debug_kp_markers -> /vision/debug_kp_markers
```

### 障碍物

新增：

```text
/vision/obstacles
/vision/obstacle_debug_image
```

## 4. 启动行为迁移

| 旧行为 | 新行为 |
|---|---|
| `camera.launch.py` 默认 Mock | 默认真实 MVS |
| `preview.launch.py` 默认 Mock | 默认真实 MVS |
| `calibration.launch.py` 默认 Mock | 默认真实 MVS |
| MVS 不可用时无限重连 | 正式入口明确报错并返回非零 |
| 必须直接运行上游 YOLO launch | 新增 `vision.launch.py` 统一入口 |
| 没有视觉障碍物节点 | 新增独立 C++ 节点 |


## 5. 下游迁移建议

新模块优先订阅：

```text
/camera/image_raw
/camera/camera_info
/vision/detections
/vision/tracks
/vision/obstacles
/vision/status
```

如果已有模块依赖 `/yolo/cam0/*`，继续使用原上游启动文件即可，不需要立即迁移。
