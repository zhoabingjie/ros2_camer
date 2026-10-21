# dmu_vision_bringup

视觉模块启动与参数编排包，不包含视觉算法。它负责组合相机、YOLO、障碍物和图像预览入口。

## 通用环境加载

每个新终端都要先执行：

```bash
cd ~/vision_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

确认包可以被找到：

```bash
ros2 pkg prefix dmu_vision_bringup
```

## 正式真实相机入口

```bash
cd ~/vision_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch dmu_vision_bringup vision.launch.py
```

默认行为：

- `backend:=mvs`
- 启动相机、YOLO 和视觉障碍物节点
- 发布稳定 `/vision/*` 接口
- 不启动图像窗口
- 真实相机打开失败时关闭整个 launch，不回退 Mock

只启动相机：

```bash
cd ~/vision_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch dmu_vision_bringup vision.launch.py \
  start_detector:=false start_obstacle:=false
```

使用外部图像源，不启动相机：

```bash
cd ~/vision_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch dmu_vision_bringup vision.launch.py start_camera:=false
```

## 图像预览

每条命令都需要在已 source 的独立终端中运行。

原始相机图像：

```bash
cd ~/vision_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

QT_QPA_PLATFORM=xcb ros2 launch dmu_vision_bringup image_viewer.launch.py \
  image_topic:=/camera/image_raw
```

YOLO 检测框：

```bash
cd ~/vision_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

QT_QPA_PLATFORM=xcb ros2 launch dmu_vision_bringup image_viewer.launch.py \
  image_topic:=/vision/debug_image
```

障碍物 ROI：

```bash
cd ~/vision_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

QT_QPA_PLATFORM=xcb ros2 launch dmu_vision_bringup image_viewer.launch.py \
  image_topic:=/vision/obstacle_debug_image
```

关闭预览窗口不会停止相机和 YOLO。

## 其他入口

| 文件 | 用途 |
|---|---|
| `vision.launch.py` | 正式统一入口，默认真实 MVS |
| `preview.launch.py` | 相机 + 可选 image_proc 校正 |
| `detection.launch.py` | 旧上游 YOLO 启动，保留 `/yolo/cam0/*` |
| `calibration.launch.py` | Jazzy camera_calibration 流程 |
| `vision_demo.launch.py` | 旧 Mock 回放演示 |

## 显式 Mock 验收

```bash
cd ~/vision_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch dmu_vision_bringup vision.launch.py \
  backend:=mock \
  mock_image_path:=$HOME/vision_ws/models/demo_scene.jpg \
  mock_encoding:=bgr8 \
  mock_fps:=8.0 \
  start_rectify:=false
```

稳定接口说明见 `extras/docs/INTERFACES.md`。
