# vision_ws — 无人车视觉子系统（ROS 2 Jazzy / C++17）

目标环境：Ubuntu 24.04 + ROS 2 Jazzy。计划书见 `extras/docs/CODEX_VISION_PLAN.md`，
环境审计见 `extras/docs/ENVIRONMENT_AUDIT.md`，测试记录见 `extras/docs/TEST_REPORT.md`。

## 当前进度

- **Phase 0 审计：完成**
- **Phase 1 相机最小接入：离线部分完成，真实硬件未完成**（阻塞项 B1/B2：无 Linux MVS SDK、相机未接入）
- **Phase 2 预览与标定：校正链路已实测通过；真实相机标定未完成**（需实物棋盘格）
- **Phase 3 YOLO 2D 检测：完成** —— 5 个包离线构建、模型加载、真实检测（bus 0.883 / person 0.835）、
  debug_image 可视化、**T08 十分钟稳定性通过（RSS 下降 53 MB、零错误日志）**
- **性能实测**：延迟 mean 38.8–41.4 ms（磁盘满时劣化到 102.4 ms），P95 47–175 ms；
  吞吐 2.9–5.6 Hz。**本 VM（2 vCPU / 3.8 GB / 20 GB）是瓶颈，不应承诺固定 FPS**
- **诊断工具包 `dmu_vision_tools`：完成并实测**（87 个测试全绿）

## 快速开始

**一条命令看"相机 + 识别"：**

```bash
cd ~/vision_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch dmu_vision_bringup vision_demo.launch.py
```

> 注意：这条命令用的是**静态图回放源**（`mock` 后端），用于验证"图像 → 检测"链路，
> **不是真实相机**。节点会明确日志告警，`/vision/status` 会报 `mock_source: True`。
> 接真实相机需要 Linux 版 MVS SDK，见 `extras/docs/QUICKSTART.md` 第 4 节。

其余命令：

# 合成图像源（无需相机）
ros2 launch hikrobot_camera_ros2 camera.launch.py

# 预览链路（相机 + image_proc 校正）
ros2 launch dmu_vision_bringup preview.launch.py backend:=mock

# 查看
ros2 topic hz /camera/image_raw
ros2 topic echo --once /camera/camera_info
ros2 topic echo --once /vision/status
```

> 未标定时 `/camera/image_rect` **不会**有输出：`image_proc` 检测到 K 全零会拒绝校正并打印
> `camera publishing '/camera/camera_info' is uncalibrated`。这是刻意的诚实行为，详见
> `extras/docs/CAMERA_CALIBRATION.md` 第 5.2 节。

VS Code：`Ctrl+Shift+B` 构建，`Tasks: Run Test Task` 跑测试。

## 包

| 包 | 语言 | 说明 |
| --- | --- | --- |
| `hikrobot_camera_ros2` | **C++17 / ament_cmake** | 海康 MV-CS050-10GM 适配层。后端可替换：`mock`（已实现）/ `mvs`（仅探测，待 SDK） |
| `dmu_vision_tools` | Python | 诊断 + 测量：`vision_health_check`、`image_file_source`、`measure_detection`，以及 `scripts/t08_stability.sh`（十分钟稳定性） |
| `thirdparty/` | — | 离线依赖：ONNX Runtime 1.20.0 前缀 + hfhub-cpp 源码（B3 的解法） |
| `dmu_vision_bringup` | Python（launch） | 编排层：相机 + image_proc + 标定 + YOLO，无业务逻辑 |
| `yolo_ros` / `yolo_msgs` / `yolo_bringup` + 2 个 vendor | C++（上游，已可构建） | 上游 YOLO ONNX 框架，用本地 ORT 前缀 + 本地 hfhub 源离线构建 |
| `attic/hikrobot_camera_ros2_python_impl` | Python（归档） | 被 C++ 版取代的早期实现，仅作参考，不参与构建 |

## 硬性约束（计划书 §0）

不猜测设备兼容性 · 不混用 Rolling 与 Jazzy 依赖 · 单目不启用 `detection3d` ·
不伪造距离/深度/定位 · **不控制 `/cmd_vel`** · 未实测不宣称通过。
