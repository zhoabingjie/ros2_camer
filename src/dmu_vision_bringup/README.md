# dmu_vision_bringup

视觉模块启动与参数编排包。

每个新终端都需要先加载环境：

```bash
cd ~/vision_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```它不包含视觉算法，只负责组合相机、YOLO、障碍物和预览入口。

## 正式入口

```bash
ros2 launch dmu_vision_bringup vision.launch.py
```

默认：

- `backend:=mvs`
- 启动相机、YOLO 和视觉障碍物节点
- 发布稳定 `/vision/*` 接口
- 不启动图像窗口

只启动相机：

```bash
ros2 launch dmu_vision_bringup vision.launch.py start_detector:=false start_obstacle:=false
```

不启动相机，使用外部图像源：

```bash
ros2 launch dmu_vision_bringup vision.launch.py start_camera:=false
```

## 图像预览

预览进程独立于相机：

```bash
QT_QPA_PLATFORM=xcb ros2 launch dmu_vision_bringup image_viewer.launch.py image_topic:=/camera/image_raw
QT_QPA_PLATFORM=xcb ros2 launch dmu_vision_bringup image_viewer.launch.py image_topic:=/vision/debug_image
QT_QPA_PLATFORM=xcb ros2 launch dmu_vision_bringup image_viewer.launch.py image_topic:=/vision/obstacle_debug_image
```

关闭预览窗口不会停止相机。

## 其他入口

| 文件 | 用途 |
|---|---|
| `vision.launch.py` | 正式统一入口，默认真实 MVS |
| `camera.launch.py` 所属相机包入口 | 只启动相机 |
| `preview.launch.py` | 相机 + 可选 image_proc 校正 |
| `detection.launch.py` | 旧上游 YOLO 启动，保留 `/yolo/cam0/*` |
| `calibration.launch.py` | Jazzy camera_calibration 流程 |
| `vision_demo.launch.py` | 旧 Mock 回放演示 |

## Mock 验收

```bash
ros2 launch dmu_vision_bringup vision.launch.py \
  backend:=mock \
  mock_image_path:=$HOME/vision_ws/models/demo_scene.jpg \
  mock_encoding:=bgr8 \
  mock_fps:=8.0 \
  start_rectify:=false
```

稳定接口说明见 `extras/docs/INTERFACES.md`。
