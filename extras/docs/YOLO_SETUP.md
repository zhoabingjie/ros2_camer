# YOLO 2D 检测 / 跟踪搭建说明（Phase 3）

> **状态：包已构建通过、单目配置已按真实源码逐字段核对、无模型时的失败路径已实测。
> 实际推理（真实检测结果）尚未验证 —— 需要模型文件。**

## 1. 构建（完全离线，实测通过）

上游 `yolo_ros` 的两个 vendor 包默认都要访问 `github.com`，本机实测不可达
（详见 `ENVIRONMENT_AUDIT.md` 阻塞项 B3）。用下面这条命令构建，**全程零下载**：

```bash
cd ~/vision_ws
source /opt/ros/jazzy/setup.bash

colcon build \
  --packages-select yolo_msgs yolo_onnxruntime_vendor yolo_hfhub_vendor yolo_ros yolo_bringup \
  --parallel-workers 2 \
  --cmake-args \
    -DONNXRUNTIME_ROOT=$HOME/vision_ws/thirdparty/onnxruntime-linux-x64-1.20.0 \
    -DFETCHCONTENT_SOURCE_DIR_YOLO_HFHUB=$HOME/vision_ws/thirdparty/huggingface-hub-cpp
```

两个覆盖参数各自解决一个阻塞：

| 覆盖 | 解决的问题 | 依据 |
| --- | --- | --- |
| `-DONNXRUNTIME_ROOT=<前缀>` | `yolo_onnxruntime_vendor` 默认从 GitHub Releases 下载约 6 MB 的 `onnxruntime-linux-x64-1.20.0.tgz` | 该 vendor 的 CMakeLists 里 `ONNXRUNTIME_ROOT` 分支优先级最高，要求 `<前缀>/lib/libonnxruntime.so` 存在 |
| `-DFETCHCONTENT_SOURCE_DIR_YOLO_HFHUB=<目录>` | `yolo_hfhub_vendor` 用 `FetchContent` 从 GitHub **git clone** `huggingface-hub-cpp` | CMake `FetchContent` 的标准源目录覆盖机制 |

ONNX Runtime 前缀的来源（二选一，都已就位）：

1. **官方 tarball 本地解压**（推荐，版本与上游默认一致）：
   `thirdparty/onnxruntime-linux-x64-1.20.0/`，含 `lib/libonnxruntime.so`、`include/onnxruntime_cxx_api.h`、
   `LICENSE`、`ThirdPartyNotices.txt`。实测无 CUDA/cuDNN 依赖。
2. **apt 包** `ros-jazzy-onnxruntime-vendor`（1.24.3），前缀为
   `/opt/ros/jazzy/opt/onnxruntime_vendor`。注意它**不含** `LICENSE` / `ThirdPartyNotices.txt`，
   而上游 vendor 的 `install(FILES ...)` 会安装这两个文件，所以直接用它会在 install 阶段失败。

**构建结果（实测）**：

```
Finished <<< yolo_hfhub_vendor [5.14s]
Finished <<< yolo_onnxruntime_vendor [2.12s]
Finished <<< yolo_msgs [35.8s]
Finished <<< yolo_ros [4min 3s]
Finished <<< yolo_bringup [3.36s]
Summary: 5 packages finished [4min 42s]
```

## 2. 单目配置（`dmu_vision_bringup/config/yolo_mono.yaml`）

**不是**照抄计划书 §7.2 的示意片段 —— 那份示意与实际源码有多处不一致
（见 `ENVIRONMENT_AUDIT.md` §6.7）。本配置文件按 `yolo_ros` v5.0.0 源码逐字段核对。

三条单目硬性约束的落实：

| 约束 | 落实方式 |
| --- | --- |
| 不启用 `detection3d` | `plugins: ["detection", "tracking", "debug"]`，列表里没有它 |
| 不留 depth 字段 | 完全不写 `depth_topic` / `depth_info_topic`（源码要求二者同时设置或同时不设，所以"都不写"是受支持的写法） |
| 不伪造 3D/距离 | 无 `detection3d` 插件 → 不会产生 `bbox3d`，不会输出米制距离 |

### 2.1 `image_reliability` 必须是 2

源码 `yolo_ros/utils/qos_compat.hpp` 的映射是：**0 = 系统默认，1 = RELIABLE，其它（含 2）= BEST_EFFORT**，
`camera_streams.cpp` 里默认值也是 2。我们的相机节点以 **BEST_EFFORT** 发布图像，
所以这里必须填 **2**；填 1 会造成"话题都在、但订阅端静默收不到数据"——
与 Phase 2 踩到的 `image_proc` 是同一类故障。

### 2.2 `rgb_topic` 的取舍

默认 `rgb_topic: /camera/image_rect`（在**已标定**的图像上做检测，符合计划书 §7.2）。
但未标定时 `/camera/image_rect` 没有数据（`image_proc` 会拒绝校正），
此时应改指向 `/camera/image_raw`。

### 2.3 CPU 相关参数全部显式覆盖

`detection_plugin.cpp:declare_params` 里的默认值几乎都是 GPU 取向，本机无 GPU，因此逐项覆盖：

| 参数 | 上游默认 | 本项目 | 原因 |
| --- | --- | --- | --- |
| `provider` | `"auto"` | `cpu` | 无 CUDA |
| `device` | `"cuda:0"` | `cpu` | 消除实测警告 `Provider "cpu" contradicts device "cuda:0"` |
| `cuda_graph_enable` | `true` | `false` | CUDA 专用 |
| `trt_fp16_enable` | `true` | `false` | TensorRT 专用 |
| `trt_engine_cache_enable` | `true` | `false` | TensorRT 专用 |

## 3. 模型（已获取并验证）

> **状态：已下载并验证通过。** 见 `models/README.md` 的元数据与许可证说明。

### ⚠️ 许可证提醒（重要）

从节点实际加载日志提取：`license=AGPL-3.0 License (https://ultralytics.com/license)`。
**AGPL-3.0 是强 copyleft 许可**。课程/内部研究一般可用，但**对外分发前必须确认合规**，
或换用许可更宽松的权重。本仓库不含权重文件。

### 以下是原始的获取方式说明


`yolo_ros` 的 `DetectionPlugin` **必须先加载 ONNX 模型才能激活**。没有模型时实测：

```
[ERROR] [yolo.yolo_node]: on_activate failed: plugin 'detection' exception during activate:
        No execution provider could initialize the ONNX Runtime session:
        Load model from /home/asd/vision_ws/models/yolo26n.onnx failed: File doesn't exist
[ERROR] [yolo.yolo_node]: Failed to activate YoloNode
```

**这是期望行为**：宁可启动失败，也不"启动成功但没有检测结果"。

获取（`huggingface.co` 不可达，用镜像；实测 HTTP 200）：

```bash
cd ~/vision_ws/models
curl -L -o yolo26n.onnx https://hf-mirror.com/unileon-robotics/YOLO26-ONNX/resolve/main/yolo26n.onnx
```

| 模型 | 实测体积 |
| --- | --- |
| `yolo26n.onnx` | 9,887,916 B ≈ **9.4 MB** |
| `yolo26s.onnx` | 38,237,095 B ≈ 36.5 MB |

## 4. 启动与验证

```bash
# 终端 1：图像源（无相机时用 mock；有标定才能用 image_rect）
ros2 launch dmu_vision_bringup preview.launch.py backend:=mock \
    calibration_file:=$HOME/vision_ws/install/dmu_vision_bringup/share/dmu_vision_bringup/config/mock/test_calibration.yaml

# 终端 2：检测
ros2 launch dmu_vision_bringup detection.launch.py

# 验证
ros2 topic list | grep yolo
ros2 topic hz /yolo/cam0/detections
ros2 topic echo --once /yolo/cam0/detections
ros2 run rqt_image_view rqt_image_view     # 选 /yolo/cam0/debug_image
```

**真实话题名（从源码提取，非猜测）**：

| 话题 | 类型 | 来源 |
| --- | --- | --- |
| `/yolo/cam0/detections` | `yolo_msgs/msg/DetectionArray` | `detection_plugin.hpp:53` |
| `/yolo/cam0/tracking` | `yolo_msgs/msg/DetectionArray` | `tracking_plugin.hpp:45` |
| `/yolo/cam0/debug_image` | `sensor_msgs/msg/Image` | `debug_plugin.hpp:58` |
| `/yolo/cam0/debug_bb_markers` | `visualization_msgs/msg/MarkerArray` | `debug_plugin.cpp:101` |
| `/yolo/cam0/debug_kp_markers` | `visualization_msgs/msg/MarkerArray` | `debug_plugin.cpp:102` |

## 5. 单元测试的已知限制

`yolo_ros` 自带 20+ 个 gtest。其中 **`test_yolo26n`** 与 **`test_batch`** 在测试期
**会联网下载模型**（`test_yolo26n` 还设了 `TIMEOUT 300`）。离线环境下这两个会失败。

处理方式：**显式列名排除并记录原因**，不用"删测试"或"放宽容差"来变绿。
其余 gtest 可正常运行。

## 6. 类别集与能力边界

`yolo_ros/conf/coco.names` 是 COCO 的 80 类。COCO 预训练模型**只保证识别它训练过的类别**，
不保证识别任意自定义障碍物。**不要在汇报或文档中夸大其能力**，也不要用它宣称
"可以识别所有道路障碍物"。
