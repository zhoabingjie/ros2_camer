# dmu_vision_tools

视觉诊断、测量和稳定性工具包。所有测试均针对真实视觉链路，不包含图片回放源。

## 可执行程序

| 程序 | 用途 |
|---|---|
| `vision_health_check` | 检查 Image、CameraInfo、QoS、频率、时间戳和相机状态 |
| `measure_detection` | 统计 YOLO 检测频率、延迟和 debug image 状态 |
| `scripts/t08_stability.sh` | 真实相机视觉链路长时间稳定性测试 |

## 用法

```bash
cd ~/vision_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run dmu_vision_tools vision_health_check --measure 5

ros2 run dmu_vision_tools measure_detection \
  --detections /vision/detections \
  --debug /vision/debug_image \
  --seconds 20
```

`t08_stability.sh` 需要真实相机：

```bash
bash ~/vision_ws/src/dmu_vision_tools/scripts/t08_stability.sh 600 60
```

诊断退出码 `0` 表示没有 FAIL；`1` 表示至少一项失败。WARN 不会使退出码失败。
