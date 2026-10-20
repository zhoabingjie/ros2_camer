# dmu_vision_tools

视觉诊断、测量和稳定性工具包。

## 可执行程序

| 程序 | 用途 |
|---|---|
| `vision_health_check` | 检查 Image、CameraInfo、QoS、频率、时间戳和相机状态 |
| `measure_detection` | 统计 YOLO 检测频率、延迟和 debug image 状态 |
| `image_file_source` | 把静态图片作为显式图像源发布 |
| `scripts/t08_stability.sh` | 长时间稳定性测试 |

## 用法

```bash
ros2 run dmu_vision_tools vision_health_check --measure 5

ros2 run dmu_vision_tools measure_detection \
  --detections /vision/detections \
  --debug /vision/debug_image \
  --seconds 20
```

旧上游 YOLO 接口仍可显式传入：

```bash
ros2 run dmu_vision_tools measure_detection \
  --detections /yolo/cam0/detections \
  --debug /yolo/cam0/debug_image \
  --seconds 20
```

诊断退出码 `0` 表示没有 FAIL；`1` 表示至少一项失败。WARN 不会使退出码失败。
