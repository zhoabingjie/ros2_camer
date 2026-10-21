# 测试报告
## 2026-10-21 MVS 参考实现审计与驱动加固

参考仓库：`https://github.com/Chlzzz/HikRobotCamera_Node`。

本次未连接真实相机，不包含真实验收结论。

### 构建

```bash
colcon build --symlink-install --packages-select hikrobot_camera_ros2 --cmake-clean-cache
```

结果：

```text
Finished <<< hikrobot_camera_ros2
BUILD_RC=0
HIKROBOT_MVS_INCLUDE_DIR=/opt/MVS/include
CXX_DEFINES: -DHIKROBOT_MVS_AVAILABLE
```

### 相机包测试

```bash
colcon test --packages-select hikrobot_camera_ros2 --parallel-workers 1
colcon test-result --verbose
```

结果：

```text
389 tests, 0 errors, 0 failures, 10 skipped
TEST_RC=0 RESULT_RC=0
```

### 无相机启动

```bash
ros2 run hikrobot_camera_ros2 camera_node --ros-args -p backend:=mvs
```

结果：

```text
CAMERA_NO_HARDWARE_RC=1
no Hikvision camera found by the MVS SDK (devices=0, sdk_version=4.8.1.2)
camera driver startup failed
```

该测试同时验证了受控 `MV_CC_Finalize` 路径，进程返回 1 且没有 `SIGSEGV`。

### 全工作区回归

结果：

```text
389 tests, 0 errors, 0 failures, 10 skipped
```

### 显式 Mock 端到端

```bash
ros2 launch dmu_vision_bringup vision.launch.py backend:=mock \
  mock_image_path:=$HOME/vision_ws/models/demo_scene.jpg \
  mock_encoding:=bgr8 mock_fps:=8.0 start_rectify:=false
```

实测节点：

```text
/camera_node
/yolo/yolo_node
/visual_obstacle_node
```

实测 `/camera/image_raw` 约 `4.7 Hz`，YOLO 检测和视觉障碍物节点正常运行。该结果只证明 Mock 链路，不代表真实相机取流成功。

### 真实验证状态

- 当前未连接相机。
- MVS 枚举为 `devices=0`。
- 真实设备打开、真实图像发布、`rqt_image_view` 实机画面未验证。
- 不得将上述 Mock 测试描述为真实相机通过。

---

## 2026-10-20 重整后完整验证

### 构建

```bash
cd ~/vision_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --parallel-workers 2 \
  --cmake-args \
  -DONNXRUNTIME_ROOT=$HOME/vision_ws/thirdparty/onnxruntime-linux-x64-1.20.0 \
  -DFETCHCONTENT_SOURCE_DIR_YOLO_HFHUB=$HOME/vision_ws/thirdparty/huggingface-hub-cpp
```

结果：

```text
Summary: 10 packages finished
BUILD_RC=0
```

### 测试

```bash
export YOLO_ROS_TEST_MODEL_DIR=$HOME/vision_ws/models
export YOLO_ROS_TEST_IMAGE=$HOME/vision_ws/models/demo_scene.jpg
export YOLO_ROS_TEST_PEOPLE_IMAGE=$HOME/vision_ws/models/demo_scene.jpg
colcon test --parallel-workers 2
colcon test-result --verbose
```

结果：

```text
Summary: 404 tests, 0 errors, 0 failures, 10 skipped
TEST_RC=0
RESULT_RC=0
```

### Mock 端到端运行

```bash
ros2 launch dmu_vision_bringup vision.launch.py backend:=mock \
  mock_image_path:=$HOME/vision_ws/models/demo_scene.jpg \
  mock_encoding:=bgr8 mock_fps:=8.0 start_rectify:=false
```

实测节点：

```text
/camera_node
/yolo/yolo_node
/visual_obstacle_node
```

实测稳定 Topic 类型：

```text
/camera/image_raw        sensor_msgs/msg/Image
/vision/detections       yolo_msgs/msg/DetectionArray
/vision/tracks           yolo_msgs/msg/DetectionArray
/vision/debug_image      sensor_msgs/msg/Image
/vision/obstacles        dmu_vision_interfaces/msg/VisualObstacleArray
/vision/status           diagnostic_msgs/msg/DiagnosticArray
```

图像发布频率实测约 `7 Hz`。`/vision/debug_image` 和 `/vision/obstacle_debug_image` 均实际收到 `640` 像素宽的图像帧。实际障碍物消息：

```yaml
status: 3
status_text: 'obstacle: confirmed 2D candidate in ROI'
input_valid: true
obstacles:
  - class_id: 5
    class_name: bus
    score: 0.8817915916442871
  - class_id: 0
    class_name: person
    score: 0.8305646181106567
```

### MVS 缺失启动行为

```bash
ros2 run hikrobot_camera_ros2 camera_node --ros-args -p backend:=mvs
```

实测：

```text
MVS_REQUIRED_RC=1
backend 'mvs' unavailable: no Hikvision camera found by the MVS SDK (devices=0)
camera driver startup failed: required camera backend 'mvs' could not be opened
```

该结果证明正式入口不会静默回退 Mock。

### 未完成实机验收

- 真实 MVS 枚举和取流：未通过，当前 `devices=0`。
- `rqt_image_view` 真实画面：未验证。
- 真实相机标定：未验证。
- 真实相机拔插/掉线恢复：未验证。

---


本文件按阶段累积。所有数值均为实测输出，未编造。

> 2026-10-21 更新：Mock 后端、`vision_demo.launch.py` 和 `image_file_source` 已从代码中删除。下面历史章节中的 Mock/回放记录仅用于保留历史证据，不代表当前代码仍提供这些入口。

## Phase 1 — 相机最小接入（离线部分，C++17 / rclcpp）

### 1. 环境

Ubuntu 24.04.4 LTS / ROS 2 Jazzy / x86_64 / 2 vCPU / 3.8 GiB RAM。
无 GPU，无相机，无 Linux MVS SDK（详见 `ENVIRONMENT_AUDIT.md`）。

### 2. 实际执行的命令

```bash
cd ~/vision_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select hikrobot_camera_ros2 --event-handlers console_direct+
colcon test  --packages-select hikrobot_camera_ros2 --event-handlers console_direct+
colcon test-result --all
```

### 3. 构建结果

```
Finished <<< hikrobot_camera_ros2 [25.2s]
Summary: 1 package finished [25.4s]
```

`-Wall -Wextra -Wpedantic` 开启，最终零警告零错误。

### 4. 测试结果

```
100% tests passed, 0 tests failed out of 6
Summary: 55 tests, 0 errors, 0 failures, 0 skipped
```

按 gtest 可执行文件拆分：

| 测试可执行文件 | 用例数 | 结果 |
| --- | --- | --- |
| `test_frame` | 6 | ✅ |
| `test_stream_health` | 10 | ✅ |
| `test_mock_backend` | 15 | ✅ |
| `test_camera_info_factory` | 7 | ✅ |
| `test_mvs_backend` | 5 | ✅ |
| `test_camera_node` | 6 | ✅ |
| **合计** | **49** | ✅ |

> `colcon test-result` 汇总的 55 = 49 个 gtest 用例 + CTest 把 6 个可执行文件各计 1 项。

### 5. 开发过程中修复的真实缺陷（未通过删测试或放宽断言来"变绿"）

C++ 版首次构建即暴露了 7 个问题，全部为真实缺陷或真实的语言/构建规则问题：

| # | 问题 | 性质 | 修复 |
| --- | --- | --- | --- |
| 1 | **`get_parameter(...).as_string_array()` 被 range-for 直接遍历** —— 该函数返回临时 `rclcpp::Parameter` 内部的引用，GCC `-Wdangling-reference` 报警 | **真实潜在 UB** | 先拷贝到具名局部变量再遍历 |
| 2 | 嵌套结构体 `Options` 的默认成员初始化器被用作外层类成员函数的默认实参 | 违反 ISO C++ 规则 | 把 `MockBackendOptions` 提到命名空间作用域 |
| 3 | `CalibrationMismatchError` 从 `std::runtime_error` 继承构造函数，但其直接基类是 `CalibrationError` | 编译错误 | 改为 `using CalibrationError::CalibrationError;` |
| 4 | 成员函数 `load_calibration` 遮蔽同名自由函数，且调用了不存在的 helper | 编译错误 | 显式限定 `::hikrobot_camera_ros2::load_calibration`，补 `format_double` |
| 5 | 缺 `<cctype>`（`std::tolower`） | 编译错误 | 补 include |
| 6 | 测试缺 `using ... MockBackendOptions;` | 编译错误 | 补 using 声明 |
| 7 | colcon 报 "Duplicate package names" —— 归档的 Python 旧版与新版同名 | 构建配置问题 | 归档目录加 `COLCON_IGNORE` |

### 6. 运行时验证（真实执行，非推测）

`ros2 launch hikrobot_camera_ros2 camera.launch.py`（mock 后端）：

```
$ ros2 topic list | grep -E "camera|vision"
/camera/camera_info
/camera/image_raw
/vision/status

$ ros2 topic hz /camera/image_raw
average rate: 9.895                        # 配置 mock_fps: 10.0

$ ros2 topic echo --once /camera/camera_info
header: {stamp: {sec: 1792162598, nanosec: 85518083}, frame_id: camera_optical_frame}
height: 480   width: 640
distortion_model: ''   d: []
k: [0.0 x6 ...]                            # 未标定标记：K 全零，未编造内参

$ ros2 topic echo --once /vision/status
status:
- level: "\0"                             # OK
  name: 'camera_node: stream'
  message: ok
  hardware_id: mock
  values:
  - {key: mock_source,        value: 'True'}    # 诚实标注合成源
  - {key: frames_total,       value: '178'}
  - {key: measured_fps,       value: '9.93'}
  - {key: last_frame_age_s,   value: '0.07'}
  - {key: consecutive_misses, value: '0'}
  - {key: consecutive_errors, value: '0'}
  - {key: backend_reopens,    value: '0'}
  - {key: calibrated,         value: 'False'}
  - {key: describe,           value: 'backend=mock requires_hardware=false pattern=moving_bar fps=10 ...'}
```

`backend:=mvs` 的拒绝路径（**预期行为**，不是故障）：

```
[ERROR] [camera_node]: backend 'mvs' unavailable: Hikrobot MVS SDK not found.
        Cannot open an MV-CS050-10GM camera.
  shared library : NOT FOUND
  headers        : NOT FOUND
  searched libs  : /opt/MVS/lib/64/libMvCameraControl.so* /opt/MVS/lib/32/... /usr/local/lib/...
  searched hdrs  : /opt/MVS/include/MvCameraControl.h /opt/MVS/include/x86_64/... /usr/local/include/...
ACTION: install the *Linux* MVS SDK (x86_64) and re-run. Only a Windows build
        (MVS_Win_STD_5.0.2) is currently on hand, which cannot be used on Ubuntu.
```

### 7. 手动测试表对应关系（计划书 §11.2）

| 编号 | 测试 | 状态 |
| --- | --- | --- |
| T01 | MVS 官方采集 | ❌ **未完成**（无 SDK / 无相机，阻塞 B1/B2） |
| T02 | ROS 图像发布 | ⚠️ 仅 mock 源通过；真实相机未验证 |
| T03 | CameraInfo | ⚠️ 仅 mock 源通过；未标定语义已验证，真实标定未验证 |
| T04 | 预览（rqt_image_view） | ⏳ 留待 Phase 2 |
| T08 | 稳定性 10 分钟 | ⏳ 未执行 |
| T09 | 断流恢复 | ⚠️ 单元级已覆盖（丢帧注入、STALLED 阻断发布、恢复、reopen 计数）；真机断流未验证 |

**未完成项不得视为通过。**

---

## Phase 2 — 预览与标定（离线链路已验证；真实标定未完成）

### 1. 新增文件

| 文件 | 说明 |
| --- | --- |
| `src/dmu_vision_bringup/` | 新包（ament_cmake，无逻辑，只做编排） |
| `.../launch/preview.launch.py` | camera_node + image_proc rectify_node |
| `.../launch/calibration.launch.py` | camera_node + cameracalibrator（交互式） |
| `.../config/mock/test_calibration.yaml` | **合成**标定（仅用于 mock 链路验证，文件头已声明） |
| `extras/docs/CAMERA_CALIBRATION.md` | 标定流程与两条坑位说明 |

### 2. 实际执行的命令

```bash
sudo apt-get install -y ros-jazzy-image-proc ros-jazzy-image-pipeline \
    ros-jazzy-camera-calibration ros-jazzy-image-view ros-jazzy-rqt-image-view \
    ros-jazzy-image-transport-plugins
colcon build --packages-select dmu_vision_bringup
ros2 launch dmu_vision_bringup preview.launch.py backend:=mock mock_pattern:=gradient [calibration_file:=...]
python3 /tmp/compare_rect.py      # 对比 /camera/image_raw 与 /camera/image_rect
```

安装结果：`image_proc 5.0.13`、`camera_calibration 5.0.13`、`image_view 5.0.13`、`rqt_image_view 4.0.7`。

### 3. 验证结果

**Case B（提供合成标定）：链路工作**

```
  raw frames received : 1
  rect frames received: 1
  raw : 640x480 mono8 step=640
  rect: 640x480 mono8 step=640
  mean|raw-rect| = 2.0083   max = 14
  RESULT: rectified image DIFFERS from raw -> the warp actually ran
--- camera log ---
[INFO] [camera_node]: calibration loaded: 640x480 from .../test_calibration.yaml
```

**Case A（未标定）：`/camera/image_rect` 零帧，且 `image_proc` 明确报错**

```
  raw frames received : 181
  rect frames received: 0
[ERROR] [camera.RectifyNode]: Rectified topic '/camera/image_rect' requested but
        camera publishing '/camera/camera_info' is uncalibrated
```

这是**期望行为**：本项目的"未标定"诚实标记（K 全零）让 `image_proc` 拒绝校正，
而不是拿编造的内参去算。未标定阶段预览请看 `/camera/image_raw`。

### 4. Phase 2 修复的真实问题

| # | 问题 | 性质 | 修复 |
| --- | --- | --- | --- |
| 1 | `image_proc::RectifyNode` 的**输入话题是 `image` 而非 `image_raw`**。按 `image_raw` 直接跑，节点正常启动、`/camera/image_rect` 话题存在，但 `ros2 topic info` 显示 `Subscription count: 0`，永远零帧且不报错 | **文档与直觉的陷阱** | `preview.launch.py` 加 `remappings=[("image", "image_raw")]` |
| 2 | TUNA 镜像当时不可达（`packages.ros.org` 也不可达），apt 下载全部失败 | 环境/镜像问题 | 切到 USTC 镜像（ROS 2 558 KB/s、Ubuntu 685 KB/s），并把宿主的 apt 强制走 IPv4（VM 无可用 IPv6 路由） |
| 3 | 归档的 Python 包与新版同名导致 colcon "Duplicate package names" | 构建配置 | 归档目录加 `COLCON_IGNORE` |

### 5. 手动测试表对应关系（计划书 §11.2）

| 编号 | 测试 | 状态 |
| --- | --- | --- |
| T04 | 预览 | ⚠️ `/camera/image_raw` 发布正常、可用 `rqt_image_view`/`image_view` 订阅；**GUI 显示未人工确认**（我无法看屏幕） |
| T05 | 标定与校正 | ⚠️ 校正链路已用合成标定实测通过（raw≠rect）；**真实标定需要实物标定板 + 真实相机，未完成** |

### 6. 未完成项

- **真实相机标定**：需要 `backend:=mvs` 可用（Linux MVS SDK）与实物棋盘格，属人工操作。
- **GUI 目视确认**：`rqt_image_view` / `image_view` 的实际画面需要人看，我无法代劳。
- 未标定时 `/camera/image_rect` 无输出属预期，已在文档说明，**不会**用伪造标定去"修好"它。

---

## Phase 3 — YOLO 2D 检测/跟踪（构建与配置已验证；推理未验证）

### 1. 关键成果：B3 阻塞彻底解除（全程零下载）

| 原阻塞 | 解法（实测有效） |
| --- | --- |
| `yolo_onnxruntime_vendor` 从 GitHub Releases 下载 ORT | `-DONNXRUNTIME_ROOT=<本地前缀>`，用 `ghfast.top` 镜像取得官方 `onnxruntime-linux-x64-1.20.0.tgz`（6,184,676 B，与 Content-Length 完全一致）后本地解压 |
| `yolo_hfhub_vendor` 用 FetchContent git clone | `-DFETCHCONTENT_SOURCE_DIR_YOLO_HFHUB=<目录>`，源码经 `codeload.github.com` 取得（43,687 B） |

```
Finished <<< yolo_hfhub_vendor [5.14s]
Finished <<< yolo_onnxruntime_vendor [2.12s]
Finished <<< yolo_msgs [35.8s]
Finished <<< yolo_ros [4min 3s]
Finished <<< yolo_bringup [3.36s]
Summary: 5 packages finished [4min 42s]
```

### 2. 网络通路的实测结论（供后续复用）

| 目标 | 结果 |
| --- | --- |
| `github.com` / `raw.githubusercontent.com` | ❌ code=000（不可达） |
| `codeload.github.com` | ✅ 可达（仓库 tarball），用于取 hfhub-cpp |
| `ghfast.top` | ✅ 可达，约 100 KB/s，用于取 ORT release |
| `ghproxy.net` | ⚠️ 可达但极慢（约 7 KB/s），会截断 |
| `huggingface.co` | ❌ 不可达 |
| **`hf-mirror.com`** | ✅ 可达，模型 resolve 返回 200 |
| `mirrors.ustc.edu.cn` | ✅ ROS 2 + Ubuntu 均可用（apt 源已切换至此） |
| `mirrors.aliyun.com/pypi` | ✅ 可达 |

### 3. 运行时验证（无模型，实测）

```
$ ros2 launch dmu_vision_bringup detection.launch.py
[yolo.yolo_node]: Configured 3 plugin(s)                       ← 无 detection3d
[model.cpp]: Using 2 threads for the ONNX Runtime session.
[model.cpp]: Execution provider chain: cpu                     ← provider: cpu 生效
[model.cpp] [WARN]: Provider "cpu" contradicts device "cuda:0"; using cpu.
[yolo.yolo_node] [ERROR]: on_activate failed: ... Load model from
    /home/asd/vision_ws/models/yolo26n.onnx failed: File doesn't exist
[yolo.yolo_node] [ERROR]: Failed to activate YoloNode
[ERROR] [yolo_node-1]: process has died [exit code 1]
```

**这是期望行为**：模型缺失时响亮失败，不会"启动成功但没有检测结果"。

### 4. 本轮修掉/发现的问题

| # | 问题 | 性质 |
| --- | --- | --- |
| 1 | `detection` 块的 `device` 默认值是 **`"cuda:0"`**，`cuda_graph_enable` / `trt_fp16_enable` / `trt_engine_cache_enable` **默认都是 true** —— 无 GPU 机器上必须逐项覆盖 | 默认值陷阱（实测警告暴露） |
| 2 | apt 版 `ros-jazzy-onnxruntime-vendor` **不含** `LICENSE` / `ThirdPartyNotices.txt`，而上游 vendor 的 `install(FILES ...)` 会安装它们 → 直接用 apt 前缀会在 install 阶段失败 | 打包差异 |
| 3 | `ghproxy.net` 会静默截断大文件（拿到 862,776 / 6,184,676 B，`tar` 报 "异常的 EOF"）；`curl` 仍返回 code=200 | 镜像可靠性 |
| 4 | 上游 vendor 的 curl 下载没有 `--retry`/完整性校验，网络抖动即产生损坏的 tarball | 上游健壮性 |

### 5. 验收状态（计划书 §11.2）

| 编号 | 测试 | 状态 |
| --- | --- | --- |
| T06 | YOLO 启动 | ⚠️ 构建通过、插件配置通过、单目约束通过；**模型加载未验证（需 9.4 MB 模型）** |
| T07 | 检测可视化 | ❌ 未验证（依赖 T06 的模型） |
| T08 | 稳定性 10 分钟 | ❌ 未执行 |
| — | 是否存在虚假 3D 距离 | ✅ 静态保证：配置中无 `detection3d` 插件 |

### 6. 未完成项

- **模型文件**（唯一阻塞）：`yolo26n.onnx` 9.4 MB，可从 `hf-mirror.com` 获取。
- 真实检测结果、FPS、端到端延迟：**均未测量**，不得编造。
- `test_yolo26n` / `test_batch` 两个上游测试需联网下载模型，离线环境**显式排除**并记录，未删除、未放宽。

---

## Phase 3 补充 — `dmu_vision_tools` 诊断包（已实测）

### 1. 新增

| 文件 | 说明 |
| --- | --- |
| `src/dmu_vision_tools/` | 新包（ament_python）。纯逻辑与 ROS 层分离，便于测试 |
| `.../topic_probe.py` | **无 ROS 依赖**：QoS 兼容判定、速率估计、陈旧度、报告与退出码 |
| `.../health_check.py` | ROS CLI：图内省 + 实时测量 → 报告 |
| `.../config/expected_topics.yaml` | 已核实的期望接口 |
| `extras/docs/TROUBLESHOOTING.md` | 把本项目踩过的每个坑固化成手册 |

### 2. 测试（实测）

```
dmu_vision_tools: 32 passed in 0.44s
全工作区汇总: 87 tests, 0 errors, 0 failures
```

（87 = 49 个 gtest 用例 + 6 个 ctest 项 + 32 个 pytest）

**测试抓到的一个真实缺口**：`qos_compatible` 的失败原因里没有写明是**哪两个节点**不兼容。
对诊断工具来说"哪个节点"才是关键信息，所以修的是**代码**（`endpoint_pair()` 现在会给消息加
`<pub> -> <sub>: ` 前缀），**没有放宽测试**。

### 3. 三案例实机验证

| 案例 | 场景 | 期望 | 实测 |
| --- | --- | --- | --- |
| A | 带标定，无异常订阅者 | PASS，退出 0 | ✅ `RESULT: PASS` |
| B | 未标定 | PASS WITH WARNINGS（**不是 FAIL**） | ✅ `RESULT: PASS WITH WARNINGS (1 warning(s))` |
| C | 未标定 + 故意写错的 RELIABLE 订阅者 | FAIL 且点名 | ✅ 见下 |

Case C 的实际输出：

```
[OK  ] topic image    /camera/image_raw (sensor_msgs/msg/Image), 1 publisher(s)
[FAIL] qos image      /camera_node -> /evil_reliable_subscriber: publisher offers
                      BEST_EFFORT but subscriber requests RELIABLE -> no data will ever be delivered
[OK  ] rate image     9.89 Hz
[WARN] calibration    UNCALIBRATED: K is all zero. ...
RESULT: FAIL (1 failure(s), 1 warning(s))
[ros2run]: Process exited with failure 1
```

> 说明：脚本里打印的 `exit_code=0` 是 `| tail` 的退出码，不是本工具的。
> 本工具的真实退出码由 `[ros2run]: Process exited with failure 1` 证明为 1。

### 4. 这个工具的价值

Phase 2 和 Phase 3 各出现过一次「话题在、节点在、日志无错、数据永远不来」的 QoS 故障，
每次都要靠 `ros2 topic info -v` + `ros2 node info` + 读上游 launch 源码才能定位。
现在这是一行输出。

---

## Phase 3 完成任务收尾 — 真实检测已跑通（T06/T07 通过）

### 1. 模型（已获取并校验）

```
文件: /home/asd/vision_ws/models/yolo26n.onnx
大小: 9,887,916 字节   与 hf-mirror 返回的 Content-Length 完全一致
文件头: 08 07 12 07 "pyto"   -> ir_version=7, producer="pytorch"，有效 ONNX protobuf
来源: https://hf-mirror.com/unileon-robotics/YOLO26-ONNX/resolve/main/yolo26n.onnx （实测 code=200）
```

### 2. 模型加载（实测日志）

```
[yolo.yolo_node]: Configured 3 plugin(s)                      <- 无 detection3d
[model.cpp]: Model loaded: /home/asd/vision_ws/models/yolo26n.onnx
[model.cpp]:   task=detect  model_type='auto'  provider=cpu  device='cuda:0' (id=0)  n_threads=2
[model.cpp]:   provider chain (requested 'cpu'): cpu
[model.cpp]:   metadata: task=detect
```

### 3. 检测结果（对含真实物体的静态图，正确）

**为什么必须用真实图片验证**：`mock` 后端渲染的是合成条纹/渐变图，其中**没有任何 COCO 物体**，
YOLO 正确地返回零检出——这不能证明检测器可用。因此用了一张公开的经典测试图
（`bus.jpg`，810×1080，公交车 + 行人），并缩放到 640×480 做性能测量。

```
detected classes:
  person     n=560  mean=0.835  max=0.885
  bus        n=140  mean=0.883  max=0.883
```

**140 帧，每帧恰好 4 个 person + 1 个 bus**，完全稳定一致。

### 4. 性能实测（2 vCPU / CPU-only）

```
image source rate    :  4.68 Hz  (n=116)   <- 源进程被 CPU 限制，未达 10 Hz
detection rate       :  5.58 Hz  (n=140)
debug_image rate     :  6.59 Hz  (n=165)
debug_image          :  640x480 bgr8
latency stamp->recv  :  mean 41.4 ms   P95 47.0 ms   n=140
```

**必须说明的边界**（不得把这组数字当作"管线的 FPS"）：

- 这是**这台虚拟机**的数字，不是算法的上限。图像源、检测器、测量进程三方共享 2 个 vCPU，
  源进程本身就只跑到 4.68 Hz。
- 延迟口径是 **图像 header 时间戳 → 检测消息到达**（有效，因为 `detection_plugin.cpp:250`
  有 `output.header = frame.header`）。
- 未测量：内存占用曲线（T08 正在做）、多核/GPU 下的上限、更大模型的对比。

### 5. 验收状态更新（计划书 §11.2）

| 编号 | 测试 | 状态 |
| --- | --- | --- |
| T06 | YOLO 启动 | ✅ **通过**（模型加载、3 插件、CPU provider、单目约束） |
| T07 | 检测可视化 | ✅ **通过**（`/yolo/cam0/debug_image` 640×480 bgr8，6.59 Hz，实测有帧） |
| T08 | 稳定性 10 分钟 | ⏳ 观测中（见下节） |
| — | 无虚假 3D 距离 | ✅ 静态保证：配置无 `detection3d`，输出无 `bbox3d` |

### 6. 本轮发现并定位的新问题

**`/yolo/cam0/debug_image` 在 10 Hz 源下零帧，在 2 Hz 源下正常。**

`ros2 topic hz` 和自建 RELIABLE 订阅者都收不到；`ros2 topic info -v` 显示 topic 存在、
publisher 是 yolo_node、RELIABLE、`Subscription count: 0`。

根因（读源码 + 对照实验确认）：`DebugPlugin` 需要把 `(CameraFrame, DetectionArray)`
**按时间戳配对**，其同步队列深度为 10（`debug_plugin.cpp:119-122`）。
当推理速度低于源帧率时（10 Hz 源、约 5 Hz 推理），检测结果出来时对应的那帧
**已经被挤出队列**，配对永远失败，debug image 一帧不发。

对照实验：源降到 2 Hz → `det_images=7, detections=5, images=5`，debug_image 立即正常。

**结论**：这是"推理速度 < 源帧率"的资源配置问题，不是配置错误。正确做法是让图像源帧率
不超过实测推理帧率（或在算力充足的机器上提高上限）。已写入 `TROUBLESHOOTING.md`。

### 7. 本轮新增的工具

| 文件 | 用途 |
| --- | --- |
| `dmu_vision_tools/image_file_source.py` | 静态图片源（计划书 §7.4 允许的离线验收手段），可指定话题/frame_id/帧率/QoS |
| `dmu_vision_tools/measure_detection.py` | 可复现的性能测量：检出率、延迟 mean/P95、类别统计 |


### 8. T08 稳定性观测 —— 第一次尝试（约 4.7 分钟，被关机中断）

首次观测在用户要求暂停时被关机打断，**目标 10 分钟未达成**（`/tmp` 随重启被清空，
数据从 `D:\无人车\docs\RESUME_NOTES.md` 恢复）。已采集到的 4 个采样点：

```
elapsed_s  yolo_alive  rss_kb  det_msgs  dbg_msgs  last_lat_ms
       60        yes   201444       277       352  39.6
      120        yes   201444       536       678  33.6
      180        yes   201444       806      1025  36.7
      240        yes   201444      1094      1402  37.2
```

| 观测项 | 数据 | 判断（仅限这 4 分钟） |
| --- | --- | --- |
| 节点存活 | 4 次采样全 `yes` | 无崩溃 |
| **内存** | **恒为 201,444 KB，四次完全相同** | 无增长迹象 |
| 检测增量 | 259 / 270 / 288 / 288 | 约 4.5 Hz 稳定 |
| debug_image 增量 | 326 / 347 / 379 / 377 | 约 6 Hz 稳定 |
| 延迟 | 39.6 / 33.6 / 36.7 / 37.2 ms | 无劣化趋势 |

**中断的观测不得视为通过。** 完整 10 分钟由
`src/dmu_vision_tools/scripts/t08_stability.sh` 重跑，结果见下一节。

### 9. 对 debug_image 结论的修正（重要）

前文"10 Hz 源下 debug_image 零帧"只针对 **810×1080** 输入。用 **640×480** 输入、
同样 10 Hz 源，debug_image **稳定输出约 6 Hz**。

准确结论：**能否配对取决于"推理是否跟得上源帧率"，不是源帧率的绝对值**；
分辨率越小推理越快，越容易跟上。**优先降输入分辨率**，而不是一味降源帧率。


### 10. T08 完整 10 分钟稳定性观测 —— **通过**

脚本：`src/dmu_vision_tools/scripts/t08_stability.sh`（可复现，日志落在 `vision_ws/artifacts/`，
不再用 `/tmp` —— 上次重启把 `/tmp` 清空过一次）。日志：
`artifacts/t08_stability_20261017_111619.log`

```
elapsed_s  yolo_alive  rss_kb  det_msgs  dbg_msgs  last_lat_ms
       60        yes   201576       297       334  28.9
      120        yes   201576       467       532  38.1
      180        yes   201576       586       670  31.6
      240        yes   201548       811       957  97.7
      300        yes   201548      1073      1306  96.7
      360        yes   201548      1443      1683  30.8
      420        yes   201548      1757      2019  35.1
      480        yes   187292      2087      2394  100.9
      540        yes   171996      2358      2736  316.0
      600        yes   147460      2639      3086  52.1

FINAL det_msgs=2639 dbg_msgs=3086 duration=600s
RSS first=201576 last=147460 delta_kb=-54116
yolo_node STILL ALIVE (no crash)
error_lines=0
```

| 判据 | 结果 |
| --- | --- |
| 10 分钟无崩溃 | ✅ 10 次采样全部存活，观测结束进程仍在 |
| 无内存泄漏 | ✅ **RSS 从 201,576 降到 147,460 KB（−53 MB）**，未增长 |
| 无错误日志 | ✅ `error_lines=0` |
| 持续产出 | ✅ 2639 条检测 / 3086 帧 debug image，无中断 |
| 延迟有界 | ⚠️ 多数 29–38 ms，但出现 97.7 / 96.7 / 100.9 / **316.0 ms** 的尖峰 |

> T08 的 `last_lat_ms` 只是**采样时刻的瞬时值**，不是分布。真正的 P95 见下一节。

### 11. 性能报告（计划书 §11.3）

同一台虚拟机、同一份配置（640×480 bgr8 静态图源，10 Hz 目标帧率，`yolo26n.onnx`，CPU），
三次独立测量：

| 测量 | 机器状态 | 源帧率 | 检测帧率 | 延迟 mean | 延迟 P95 | n |
| --- | --- | --- | --- | --- | --- | --- |
| #1 | 正常 | — | ~0.96 Hz | 41.4 ms | 47.0 ms | 140 |
| #2 | **磁盘 96% 满** | 5.07 Hz | 4.18 Hz | **102.4 ms** | **174.9 ms** | 250 |
| #3 | 清理磁盘后（2.1 GB 空闲） | 2.94 Hz | 2.93 Hz | **38.8 ms** | 92.4 ms | 175 |

（#1 用的是临时脚本，percentile 用了 `round()`；n=140 时与正确公式同排名，故数值仍有效。
正式工具 `measure_detection` 已改为标准 nearest-rank `ceil(p*n)-1`。）

**结论：**

1. **平均延迟约 39–41 ms 是这台机器在资源充足时的水平**；
2. **磁盘接近满（96%）时平均延迟劣化到 102 ms、P95 到 175 ms** —— 清理出 1.2 GB 后
   平均延迟回到 38.8 ms，验证了"资源压力而非代码缺陷"这一判断；
3. **P95 在 47–175 ms 之间波动，不能只引用好看的那次**；
4. 吞吐在 **2.9–5.6 Hz** 之间波动，取决于机器负载；
5. 观测期间 **swap 已用 1.4 GiB / 3.6 GiB** —— 3.8 GB 内存对本负载明显不足。

**因此：本虚拟机（2 vCPU / 3.8 GB RAM / 20 GB 磁盘）跑不动更高帧率。**
要得到有意义的性能数字，需要扩容（建议 ≥4 vCPU / 8 GB RAM / 40 GB 磁盘），
或者在一台专用机器上用 rosbag 回放测量。**在此之前不应对外承诺固定 FPS。**

### 12. 本轮修正的代码缺陷

`measure_detection.percentile()` 原本用 `round(fraction * (n-1))`，对 100 个样本求中位数时
`round(49.5) → 50`，取到了第 **51** 个值。已改为标准 nearest-rank `ceil(p*n) - 1`。
这个数字会写进性能报告，取错名次是系统性偏差，因此**改代码而不是改测试**。

### 13. 磁盘事故与处置

- 观测期间磁盘从 3.2 GB 空闲降到 **814 MB（96%）**
- 主因：`/var/cache/apt` **1.2 GB**（历次 apt 下载的 .deb 缓存）
- 处置：临时停 `unattended-upgrades` → `apt-get clean` → 空闲回到 **2.1 GB**
- 注意：`/media/asd` 显示 6.3 GB 是挂载的 **Ubuntu 安装 ISO**（`/dev/sr0`、`/dev/sr1`），
  不占根分区，`du /` 的总量会因此虚高


### 14. T07 检测可视化 —— 人工目视确认通过（2026-10-17）

用户在本机运行 `ros2 launch dmu_vision_bringup vision_demo.launch.py`，
并在 `rqt_image_view` 中选择 `/yolo/cam0/debug_image`，**亲眼看到了带检测框的
公交车 + 行人画面**。

这让 T07 从"自动测量通过"升级为"**人工确认通过**"：

| 判据 | 状态 |
| --- | --- |
| `/yolo/cam0/debug_image` 有数据 | ✅ 自动测量 6.59 Hz / 640×480 bgr8 |
| 框与类别确实可见 | ✅ **用户目视确认** |
| 启动日志无 ERROR | ✅ 用户提供的完整日志中 ERROR 0 次 |
| `Activated` + `stream state -> ok` | ✅ |

**仍未完成**：真实相机采集（T01/T02）与真实标定（T05 实测部分），
两者都等待 Linux MVS SDK 与相机接入。


---

## Phase 1 真实相机接入 — 驱动已实现，等待硬件（2026-10-17）

### 1. Linux MVS SDK 已安装（仅库 + 头文件）

采用的是"只取所需"策略，不跑厂商的 `setup.sh`：

```
来源: MVS_Linux_STD_V5.0.2_260728.zip  ->  MVS-5.0.2_x86_64_20260728.tar.gz
      ->  内层 MVS.tar.gz
提取: 仅 lib/64 + lib/CLProtocol + include
安装: /opt/MVS   (42 MB，未安装 MVS GUI / samples / docs / driver)
链接: sudo ldconfig -n /opt/MVS/lib/64   (按 SDK 自己的 set_sdk_version.sh 方式重建 .so 链接)
```

关键文件实测存在：

```
/opt/MVS/lib/64/libMvCameraControl.so         -> libMvCameraControl.so.4.8.1.2
/opt/MVS/lib/64/MvProducerGEV.cti             (GigE GenICam 传输层)
/opt/MVS/include/MvCameraControl.h            (263 KB)
/opt/MVS/include/CameraParams.h
```

### 2. 采集实现（`src/mvs_backend.cpp`，564 行）

**设计要点：用 `dlopen` 动态加载，不硬链接。** 这样没装 SDK 的机器仍能编译并跑 mock，
不会因为链接依赖而启动失败。

| 步骤 | 使用的 API（全部来自真实头文件） |
| --- | --- |
| 加载库 | `dlopen` + `dlsym`（13 个符号，缺一个就报出名字） |
| 枚举 | `MV_CC_EnumDevices(MV_GIGE_DEVICE, &list)` |
| 建句柄 | `MV_CC_CreateHandle` |
| 打开 | `MV_CC_OpenDevice(handle, MV_ACCESS_Exclusive, 0)` |
| 参数 | `MV_CC_SetEnumValue("PixelFormat"/"ExposureAuto"/"GainAuto")`、`MV_CC_SetIntValueEx("Width"/"Height"/"ExposureTime"/"Gain"/"GevSCPSPacketSize")` |
| 取流 | `MV_CC_GetOneFrameTimeout`（`MV_E_NODATA` → 返回空，不算错误） |
| 转换 | `MV_CC_ConvertPixelTypeEx`（非 Mono8/BGR8/RGB8 一律转 mono8） |
| 关闭 | `MV_CC_StopGrabbing` → `MV_CC_CloseDevice` → `MV_CC_DestroyHandle` |

其它实现细节：

- **环境变量自动补齐**（仅当用户未设置时）：`MVCAM_SDK_PATH` / `MVCAM_COMMON_RUNENV` /
  `MVCAM_SOFTWARE_LIBENV` / `MVCAM_GENICAM_CLPROTOCOL` —— 传输层插件靠它们定位。
- **GigE 包大小**：自动调用 `MV_CC_GetOptimalPacketSize` 并写入 `GevSCPSPacketSize`。
- **时间戳诚实性**：`nHostTimeStamp` 只在看起来像纳秒纪元（> 1e18）时才采用，否则用自己的时钟；
  **设备时间戳的纪元未知，所以 `hardware_stamp_ns` 留空**，不伪装成墙上时间。
- **失败信息可操作**：枚举到 0 个设备时打印供电/网线/L2 网段/`set_rp_filter` 四项检查清单。

### 3. 编译与测试

```
-- Hikrobot MVS SDK headers found: /opt/MVS/include -> real camera support ON
Finished <<< hikrobot_camera_ros2 [15.5s]
Summary: 58 tests, 0 errors, 0 failures, 1 skipped
```

跳过的 1 项是"SDK 未安装"分支 —— 本机已安装，因此正确跳过。

**更新了 4 个测试**以匹配新契约（不是放宽）：原先断言 `implemented=false`、
`"MVS SDK not found"`，现在改为断言「要么报告 SDK 缺失、要么报告找不到相机，
两者都必须说明真实阻塞原因，且绝不返回伪造帧」。

### 4. 实测：无相机时的行为

```
[ERROR] backend 'mvs' unavailable: no Hikvision camera found by the MVS SDK. Checklist:
  1. camera powered? (LED on)
  2. Ethernet cable from camera to the host/bridged adapter?
  3. is the camera on the same L2 segment as this interface?
  4. run the SDK's set_rp_filter.sh --mode=2 on the host for cross-subnet discovery
```

**说明 SDK 已成功加载并完成了设备枚举**（只是没找到设备）。

### 5. 仍未完成

| 项 | 状态 |
| --- | --- |
| T01 MVS 官方采集 | ❌ 相机未接入虚拟机 |
| T02 ROS 图像发布（真实） | ❌ 同上 |
| 虚拟机桥接网卡 | ❌ 未配置（当前仅 NAT） |
| 真实标定 T05 | ❌ 需棋盘格 |

**代码侧已就绪**：硬件到位后只需 `vision_demo.launch.py backend:=mvs`。
