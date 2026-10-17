# 模型目录

> **状态：`yolo26n.onnx` 已下载并校验通过**（9,887,916 B，与 Content-Length 一致；
> ONNX 文件头 `08 07 12 07 "pyto"` 有效）。真实检测已于 2026-10-17 验证通过。

本目录存放 YOLO 的 ONNX 权重。**权重不进版本库**（体积大、许可独立）。

`config/yolo_mono.yaml` 里 `detection.model_path` 默认指向：

```
/home/asd/vision_ws/models/yolo26n.onnx
```

## 为什么需要它

`yolo_ros` 的 `DetectionPlugin` 必须先加载一个 ONNX 模型才能激活。
没有模型时节点会在 configure 阶段**明确失败**（实测：

```
[ERROR] [yolo_node]: on_configure failed: plugin 'detection': missing parameter 'detection.plugin'
[ERROR] [yolo_node]: Failed to configure YoloNode
```

），**不会**静默地"启动成功但没有检测结果"。

## 来源与体积（实测）

上游默认从 Hugging Face 拉取 `unileon-robotics/YOLO26-ONNX`。本机 `huggingface.co`
不可达，但 `hf-mirror.com` 可达，实测 HTTP 200：

| 文件 | 实测 Content-Length |
| --- | --- |
| `yolo26n.onnx` | 9,887,916 B（约 9.4 MB） |
| `yolo26s.onnx` | 38,237,095 B（约 36.5 MB） |

```
curl -L -o yolo26n.onnx https://hf-mirror.com/unileon-robotics/YOLO26-ONNX/resolve/main/yolo26n.onnx
```

## 注意

- 模型**必须**与 `img_width`/`img_height`、`model_type` 匹配；
- 类别集来自模型自身训练数据。`yolo_ros/conf/coco.names` 是 COCO 的 80 类。
  **COCO 预训练模型不保证识别任意自定义障碍物**，不要在文档或汇报中夸大它的能力；
- 模型许可需单独确认，不要随代码分发。


## 模型元数据（从 yolo_node 实际加载日志提取，非猜测）

```
task        = detect
names       = <80 classes>            -> COCO 80 类，与 yolo_ros/conf/coco.names 一致
imgsz       = [480, 640]              -> 与 config/yolo_mono.yaml 的 img_height/img_width 匹配
stride      = 32
date        = 2026-10-09T11:59:48+02:00
license     = AGPL-3.0 License (https://ultralytics.com/license)
```

### ⚠️ 许可证：AGPL-3.0

**权重文件自报的许可证是 AGPL-3.0**（Ultralytics）。这不是宽松许可证，请注意：

- 课程/内部研究用途一般没问题；
- 但如果本项目**对外分发**（含把模型打包进镜像/产品），AGPL-3.0 的义务会传导到分发物上；
- 商用前必须让团队/导师确认许可证合规，或换用许可证更宽松的权重。

**本仓库不包含权重文件**，`models/` 只有本说明；权重由使用者按上面的来源自行下载。
