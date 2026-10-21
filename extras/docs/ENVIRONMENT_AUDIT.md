# 环境与源码审计报告（Phase 0）

## 2026-10-20 重整前复核（最新）

| 项目 | 实测结果 |
|---|---|
| Windows 仓库 | `D:\无人车\ros2_camer`，分支 `main`，重整前无未提交改动 |
| 虚拟机工作区 | `/home/asd/vision_ws`，源码未纳入 Git，不能直接覆盖 |
| 虚拟机相对 Windows 的实质差异 | `mvs_backend` 中新增设备型号、序列号和 IP 枚举输出；已安全合并 |
| Ubuntu | 24.04.4 LTS |
| CPU 架构 | x86_64 |
| ROS | Jazzy |
| MVS 包 | `mvs`，`install ok installed`，安装到 `/opt/MVS` |
| MVS 动态库 | `/opt/MVS/lib/64/libMvCameraControl.so.4.8.1.2` |
| MVS 客户端 | `/opt/MVS/bin/MVS`，动态库检查通过 |
| 当前相机枚举 | `devices=0` |
| 当前真实取流 | **未通过**：宿主 Realtek 以太网仍为 `Disconnected / 0 bps`，VM 没有相机桥接网卡 |
| Jazzy 依赖 | `image_proc`、`image_view`、`rqt_image_view`、`camera_calibration`、`cv_bridge` 可用 |
| 上游 YOLO | `yolo_msgs`、`yolo_ros`、`yolo_bringup` 由工作区构建 |
| 构建 | 10 个包构建通过 |
| 测试 | 389 tests / 0 failures / 0 errors / 10 skipped |
| 安全审计 | 相机和障碍物节点测试均检查不发布 `/cmd_vel` |

### 发现的问题

1. 旧实现曾默认 Mock；Mock 和图片回放代码现已全部删除。
2. MVS 失败时节点无限重连，启动命令不会返回失败。
3. MVS 部分初始化失败后存在句柄释放风险。
4. 驱动未明确限定单台 GigE 相机，不能静默选择任意设备。
5. 缺少稳定视觉障碍物接口和独立节点。
6. YOLO 只有 `/yolo/cam0/*` 上游接口，缺少 `/vision/*` 正式接口。
7. 文档中的真实相机状态和启动命令过期。

### 本次处理

- 正式入口切换为 `backend:=mvs`。
- 增加 `require_camera_at_startup`，失败返回非零。
- 增加 MVS 部分初始化析构清理和严格的单相机数量校验。
- 新增 `dmu_vision_interfaces` 和 `visual_obstacle_avoidance`。
- 正式入口提供稳定 `/vision/*` Topic，同时保留上游旧启动方式。
- 更新主 README、接口清单、迁移说明和包级 README。

### 仍未完成

真实相机采集、`rqt_image_view` 实机画面、真实标定和相机断线恢复必须在 GigE 链路可用后验收。当前代码不提供 Mock 或回放结果作为替代。

---


| 项目 | 值 |
| --- | --- |
| 报告日期 | 2026-10-16 |
| 执行阶段 | Phase 0（审计，不改业务代码） |
| 依据计划书 | 计划书实际位于项目根目录，文件名为 URL 编码的 `无人车视觉子系统_Codex实施计划书.md`（19870 字符）。**计划书中所写的 `extras/docs/CODEX_VISION_PLAN.md` 在磁盘上并不存在**，本报告依据实际文件执行。 |
| 审计执行位置 | ROS 2 环境在 VMware 虚拟机 `192.168.95.129`（用户名 `asd`）中审计；源码在 Windows 宿主 `D:\无人车` 中审计 |
| 结论 | Phase 0 完成；**Phase 1 起存在 4 项硬性阻塞**，见第 8 节 |

> 本报告所有数值均来自实测命令输出。凡无法实测的项一律标注"未验证"，不做推测。计划书第 0 节第 1 条、第 5 条要求的"没有证据时标注待现场确认"已严格执行。

---

## 1. 审计方法

- 虚拟机侧：通过 SSH（密钥 `~/.ssh/codex_vmware_zbj`）执行 shell 命令采集系统、ROS、设备、apt 信息。
- 宿主侧：读取 `D:\无人车` 下三个源码目录、压缩包与文件属性。
- 源码审计：逐个读取 `yolo_ros-main` 与 `image_pipeline-rolling` 的 `package.xml`、`CMakeLists.txt`、`plugins.xml`、`config/*.yaml`、`launch/*.launch.py`、`msg/*.msg` 与插件实现源码，提取**真实**话题名与消息类型。

---

## 2. 系统与硬件（虚拟机实测）

| 项目 | 实测值 | 命令 |
| --- | --- | --- |
| 发行版 | Ubuntu 24.04.4 LTS (Noble Numbat) | `lsb_release -ds` |
| 内核 | 6.17.0-14-generic | `uname -r` |
| 架构 | x86_64 / amd64 | `uname -m`、`dpkg --print-architecture` |
| glibc | 2.39 | `ros2 doctor --report` |
| Python | 3.12.3 | `python3 --version` |
| CPU（物理机） | Intel(R) Core(TM) i7-14650HX | `grep -m1 "model name" /proc/cpuinfo` |
| **vCPU（分配给虚拟机）** | **2** | `nproc` |
| **内存** | **3.8 GiB 总量 / 1.9 GiB 可用** | `free -h` |
| **根分区** | **20G，已用 15G，可用 4.2G（78%）** | `df -h /` |
| GPU | VMware SVGA II Adapter（虚拟显卡） | `lspci` |
| **NVIDIA / CUDA** | **无**。`nvidia-smi` 不存在 | `nvidia-smi` |
| 网络接口 | ens33，IP 192.168.95.129（VMware NAT） | `ros2 doctor --report` |

**结论：本机是纯 CPU 环境，没有任何 NVIDIA GPU / CUDA / TensorRT。计划书中 GPU 相关选项在本机不可用。**

---

## 3. ROS 2 环境

| 项目 | 实测值 |
| --- | --- |
| 已安装发行版 | 仅 `jazzy`（`ls /opt/ros`） |
| `ros-jazzy-ros-core` | `0.11.0-1noble.20261006.035319` |
| `ros-jazzy-desktop` | `0.11.0-1noble.20261006.040109` |
| 已安装 `ros-jazzy-*` 包数 | 296 |
| 环境变量自动 source | 已写入 `~/.bashrc` |
| **rosdep** | **未初始化**（`/etc/ros/rosdep/sources.list.d/` 不存在） |
| OpenCV（系统） | 4.6.0（`libopencv-dev` 已装，`python3 -c "import cv2"` → 4.6.0） |
| 已装 ROS 图像依赖 | `ros-jazzy-cv-bridge` 4.1.0、`ros-jazzy-image-transport` 5.1.8 |

### 3.1 Jazzy 官方二进制包可用性（关键结论）

| 包名 | 候选版本 | 来源 |
| --- | --- | --- |
| `ros-jazzy-camera-calibration` | 5.0.13-1noble.20260912.164100 | 清华镜像 |
| `ros-jazzy-image-proc` | 5.0.13-1noble.20260915.175152 | 清华镜像 |
| `ros-jazzy-image-view` | 5.0.13-1noble.20260912.170248 | 清华镜像 |
| `ros-jazzy-image-pipeline` | 5.0.13-1noble.20260915.193833 | 清华镜像 |
| `ros-jazzy-v4l2-camera` | 0.7.3-1noble.20260912.170351 | 清华镜像 |
| `ros-jazzy-usb-cam` | 0.8.1-1noble.20260912.190806 | 清华镜像 |
| `ros-jazzy-vision-msgs` | 4.1.1-3noble.20260911.140407 | 清华镜像 |

**结论：计划书 §2 所述"先检查 Jazzy 系统包是否已提供"的答案是"全部已提供"。因此 `image_pipeline-rolling` 源码不需要编译（见第 7 节）。**

### 3.2 网络实测（从虚拟机发起）

| 目标 | 结果 | 说明 |
| --- | --- | --- |
| `mirrors.tuna.tsinghua.edu.cn/ros2/...` | 200，**1.5 MB/s** | ROS 2 镜像，已配置为 `ros2.list` |
| `mirrors.tuna.tsinghua.edu.cn/ubuntu/...` | 200，349 KB/s | 已替换 Ubuntu 默认源 |
| `packages.ros.org` | 200，**约 65-70 KB/s** | 官方源，慢约 20 倍 |
| `raw.githubusercontent.com` | **不可达（code=000）** | 被网络阻断 |
| `github.com` | **不可达（code=000）** | 被网络阻断 |
| `keyserver.ubuntu.com` | 可达 | 用于取 ROS GPG key |
| `huggingface.co` | **未测**（见 8.4） | 模型下载依赖它 |

### 3.3 GPG 密钥问题（已修复，记录备查）

ROS apt 源的签名校验此前失败，报错：

```
下列签名无效： EXPKEYSIG F42ED6FBAB17C654 Open Robotics <info@osrfoundation.org>
E: 仓库 "http://packages.ros.org/ros2/ubuntu noble InRelease" 没有数字签名。
```

根因：`packages.ros.org/ros.key`（以及被广泛复制的 `raw.githubusercontent.com/ros/rosdistro/master/ros.key`）是**陈旧副本**，其自签名有效期止于 **2025-06-01**；而 keyserver 上的同一密钥（指纹 `C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654`）**有效期至 2030-06-01**。已从 keyserver 导入新版并替换 `/usr/share/keyrings/ros-archive-keyring.gpg`，`apt update` 恢复正常。**部署到其他机器时必须复现此修复，否则 apt 无法使用 ROS 源。**

---

## 4. 相机与 MVS SDK 状态

### 4.1 虚拟机内

| 检查项 | 结果 |
| --- | --- |
| `/dev/video*` | **不存在**（`ls -l /dev/video*` → No such file or directory） |
| `lsusb` 输出 | 仅 VMware 虚拟设备：`1d6b:0001` root hub、`0e0f:0003` Virtual Mouse、`0e0f:0002` Virtual USB Hub、`1d6b:0002` root hub |
| MVS SDK | **未安装**。`find /opt /usr/local /usr/lib -iname "*MVS*"` 仅命中无关的 Marvell 固件 `mvsw_prestera_fw*.img.zst`；`-iname "*MvCamera*"` 无命中 |

### 4.2 宿主机侧

| 文件 | 大小 | 判断 |
| --- | --- | --- |
| `MVS_Win_STD_5.0.2_260728（1）.zip` | 455.6 MB | **Windows 版 SDK（`MVS_Win_STD`），版本 5.0.2，日期 2026-07-28**。文件名中的 `Win` 表明它**不能**用于 Ubuntu 目标机 |

**结论（对应计划书第 0 节第 4 条"海康驱动先核验"）：**
- 现场**没有任何证据**表明 MV-CS050-10GM 能被当前虚拟机采集：相机未接入、Linux SDK 未安装、无 `/dev/video*`。
- 目录中只有 Windows SDK。**Linux 版（`MVS_Linux_STD_*` 之类）缺失，需用户提供。**
- **因此不得声称任何驱动"已兼容"。** 计划书要求的"驱动做成可替换适配层"在本报告中列为设计约束。

### 4.3 MV-CS050-10GM 事实性备注

该型号为 GigE（千兆网口）工业相机，非 USB 相机。这带来两点直接影响，需现场确认：
1. **不能靠 VMware USB 直通**接入，需虚拟机的**网络**能与相机所在网段互通（该相机使用私有网段与 MVS 的 GigE 发现协议）。
2. `ros-jazzy-v4l2-camera` / `usb-cam` 依赖 `/dev/video*`，**对本相机不适用**；必须走 MVS SDK 的 `MvCameraControl` API 或可验证支持该 SDK 的 ROS 驱动。

> 上述型号与接口判断基于型号命名，**未在真机验证**。真机接入前请勿据此断言兼容性。

---

## 5. 现有可复用资产盘点

| 资产 | 位置 | 体积 | 用途 |
| --- | --- | --- | --- |
| `yolo_ros-main` | `D:\无人车\yolo_ros-main` | 1.0 MB | 上游 YOLO ONNX 检测框架（v5.0.0） |
| `yolo_ros-main.zip` | `D:\无人车` | 0.30 MB | 同上的压缩原始件 |
| `image_pipeline-rolling` | `D:\无人车\image_pipeline-rolling` | 9.4 MB | Rolling 分支源码（仅作参考，**不编译**） |
| `image_pipeline-rolling.zip` | `D:\无人车` | 7.50 MB | 同上压缩件 |
| `MVS_Win_STD_5.0.2_260728（1）.zip` | `D:\无人车` | 455.6 MB | Windows 版 MVS SDK（本项目不可用） |
| `mini-tesla-ROS2-main` | `D:\无人车\mini-tesla-ROS2-main` | - | 团队此前的 ROS 2 无人车工程（Foxy 时代），可作接口设计参考 |

`yolo_ros-main` 的 `yolo_onnxruntime_vendor` 与 `yolo_hfhub_vendor` 目录**不含预置二进制**，只有 CMake 脚本——即依赖在构建时联网获取（见 6.5）。

---

## 6. 源码审计：`yolo_ros-main`

### 6.1 包清单

| 包名 | 版本 | 构建类型 | 许可证 |
| --- | --- | --- | --- |
| `yolo_msgs` | 5.0.0 | ament_cmake（接口包） | MIT |
| `yolo_ros` | 5.0.0 | ament_cmake（C++17） | MIT |
| `yolo_bringup` | 5.0.0 | ament_cmake + ament_cmake_python | MIT |
| `yolo_onnxruntime_vendor` | - | ament_cmake（下载/构建 ONNX Runtime） | MIT |
| `yolo_hfhub_vendor` | - | ament_cmake（FetchContent 拉取 huggingface-hub-cpp 1.1.5） | MIT |

### 6.2 Jazzy 支持状态

仓库自带 `.github/workflows/jazzy-build-test.yml`，且 `yolo_ros/CMakeLists.txt` 中对 `jadzzy`/`kilted` 等发行版有显式适配分支：

- `$ENV{ROS_DISTRO}` 为 `iron`/`jazzy`/`kilted` 等时定义 `-DCV_BRIDGE_HPP`（cv_bridge 4.x 头文件改名为 `cv_bridge.hpp`）；
- jazzy 走 `-DMESSAGE_FILTERS_OLD_API` 分支。

**结论：上游明确支持 Jazzy，且我们已实测 `ros-jazzy-cv-bridge` 为 4.x（4.1.0），与上游假设一致。** 这是可编译性的正向证据，但不等于"已编译通过"——本地编译将在 Phase 3 实测。

### 6.3 消息接口（`yolo_msgs`，逐字段实读）

```
Detection.msg
  int32   class_id
  string  class_name
  float64 score
  string  id                      # 跟踪 ID（未跟踪时为空）
  yolo_msgs/BoundingBox2D bbox
  yolo_msgs/BoundingBox3D bbox3d  # 仅 detection3d 插件填充
  yolo_msgs/Mask          mask
  yolo_msgs/KeyPoint2DArray keypoints
  yolo_msgs/KeyPoint3DArray keypoints3d

DetectionArray.msg
  std_msgs/Header header
  yolo_msgs/Detection[] detections

BoundingBox2D.msg
  yolo_msgs/Pose2D  center    # Pose2D = { Point2D position{x,y}, float64 theta }
  yolo_msgs/Vector2 size      # { float64 x, float64 y } 单位：像素

BoundingBox3D.msg
  geometry_msgs/Pose    center
  geometry_msgs/Vector3 size
  string                frame_id

SetClasses.srv
  string[] classes   ->  bool success, string message
```

计划书 §7.3 要求"字段至少包括 Header、类别 ID/名称、置信度、二维框中心/宽高、可选 track ID"——**上游 `DetectionArray` 已完整覆盖**，因此**无需新建消息接口**即可满足 G 项（可在适配层做字段投影）。

### 6.4 插件与**真实话题名**（从源码提取，非 README 推测）

插件类在 `yolo_ros/plugins.xml` 中注册，共 4 个：

| 插件类 | 输出话题模板 | 消息类型 | 源码位置 |
| --- | --- | --- | --- |
| `yolo_ros/DetectionPlugin` | `"<cam>/detections"` | `yolo_msgs/DetectionArray` | `detection_plugin.hpp:53` |
| `yolo_ros/TrackingPlugin` | `"<cam>/tracking"` | `yolo_msgs/DetectionArray` | `tracking_plugin.hpp:45` |
| `yolo_ros/Detect3DPlugin` | `"<cam>/detections_3d"` | `yolo_msgs/DetectionArray` | `detect_3d_plugin.hpp:47` |
| `yolo_ros/DebugPlugin` | `"<cam>/debug_image"` | 图像（见下） | `debug_plugin.hpp:58` |
| `yolo_ros/DebugPlugin` | `"<cam>/debug_bb_markers"` | `visualization_msgs/MarkerArray` | `debug_plugin.cpp:101` |
| `yolo_ros/DebugPlugin` | `"<cam>/debug_kp_markers"` | `visualization_msgs/MarkerArray` | `debug_plugin.cpp:102` |

其中 `<cam>` 是 `cameras: [...]` 里配置的相机名。话题名是**相对名**，会落在节点的 namespace 下。

**默认配置（`namespace:=yolo`、`cameras: ["cam0"]`）解析后的绝对话题名：**

```
/yolo/cam0/detections          yolo_msgs/msg/DetectionArray   ← 2D 检测主输出
/yolo/cam0/tracking            yolo_msgs/msg/DetectionArray   ← 跟踪输出（启用 tracking 时）
/yolo/cam0/detections_3d       yolo_msgs/msg/DetectionArray   ← 单目配置必须禁用
/yolo/cam0/debug_image         sensor_msgs/msg/Image          ← 可视化
/yolo/cam0/debug_bb_markers    visualization_msgs/msg/MarkerArray
/yolo/cam0/debug_kp_markers    visualization_msgs/msg/MarkerArray
```

> 计划书 §4 表里建议的 `/vision/detections`、`/vision/debug_image` 是**建议命名**，与上游真实话题不同。按计划书"以源码实际发布名为准""不猜话题名"的要求，本报告以上表为准。适配层可用 remap 把 `cam0/detections` 映射到 `/vision/detections`，无需改上游代码。

### 6.5 构建依赖与**联网阻塞**（本项目最大技术风险）

`yolo_ros` 依赖两个 vendor 包，二者的依赖**都在构建期从 GitHub 获取**，而本机实测 **github.com 不可达（code=000）**：

1. **`yolo_onnxruntime_vendor`**
   - 默认下载 `https://github.com/microsoft/onnxruntime/releases/download/v1.20.0/onnxruntime-linux-x64-1.20.0.tgz`（约 200 MB）CPU 版；
   - 该包体提供了三个**可用覆盖入口**：
     - `-DONNXRUNTIME_URL=<url>`：换成任意可达镜像的完整 URL；
     - `-DONNXRUNTIME_ROOT=<prefix>`：直接指定本地已构建/已解压的 ONNX Runtime（要求存在 `lib/libonnxruntime.so`），**优先级最高**；
     - `scripts/prepare_offline_bundle.sh`：仓库自带的离线打包脚本；
   - GPU 变体（`ONNX_GPU=ON`）**本项目不适用**（无 NVIDIA/CUDA）。

2. **`yolo_hfhub_vendor`**
   - 用 CMake `FetchContent` 执行 `GIT_REPOSITORY https://github.com/agonzc34/huggingface-hub-cpp`、`GIT_TAG 1.1.5`；
   - 源码中**未提供** URL 覆盖参数，但 `FetchContent` 通用机制支持 `-DFETCHCONTENT_SOURCE_DIR_YOLO_HFHUB=<本地目录>` 指向已克隆的源码；
   - 该包还 `find_package(CURL REQUIRED)`。

3. **模型权重**：默认从 Hugging Face Hub 运行时下载（见 6.6）。`huggingface.co` 在本机**尚未测通**。

4. **测试**：`yolo_ros` 含 20+ 个 gtest。其中 `test_yolo26n`、`test_batch` 链接 `CURL::libcurl` 与 `JPEG::JPEG`，`test_yolo26n` 设置 `TIMEOUT 300`，**需要模型文件与网络**，离线环境预计失败。

**结论：`yolo_ros` 在完全离线环境下当前无法配置成功（CMake configure 阶段即会在 FetchContent 拉取 hfhub 时失败）。必须先解决 ONNX Runtime 与 hfhub-cpp 两个来源，见第 12 节方案。**

### 6.6 模型加载逻辑（源码实证）

`detection_plugin.cpp:24-27` 声明的参数：

```cpp
node.declare_parameter<std::string>(prefix + "model_type",     "auto");
node.declare_parameter<std::string>(prefix + "model_path",     "");
node.declare_parameter<std::string>(prefix + "model_repo",     "");
node.declare_parameter<std::string>(prefix + "model_filename", "");
```

`detection_plugin.cpp:124-147` 的加载顺序：

```cpp
if (!model_repo.empty() && !model_filename.empty()) {
    auto result = huggingface_hub::hf_hub_download_with_shards(
        model_repo, model_filename, cache_dir, force_download);
    if (result.success) { model_path = result.path; }
    else { /* 打日志，回落到本地 model_path */ }
}
model_ = yolo_ros::yolo::create_model(params);   // 最终使用 model_path
```

**结论：`model_path`（本地 ONNX 绝对路径）是受支持的一等参数。** 只要把 `model_repo` 与 `model_filename` **留空**，就不会触发任何网络下载，直接加载本地文件——这正是计划书 §7.2"模型路径优先支持本地文件"所需的机制，**无需改上游代码**。

### 6.7 参数 schema 与计划书示例的差异（重要）

计划书 §7.2 给出的 YAML 是"**结构示例**"，并明确要求"必须对照当前源码确认字段的实际名称"。实测差异如下：

| 计划书示例字段 | 上游真实情况 | 处置 |
| --- | --- | --- |
| `model_path` | **存在**（默认 `""`），但与 `model_repo`/`model_filename` 并存 | 采用：留空 repo/filename，只填 path |
| `model_repo` / `model_filename` | 计划书未提及，**上游默认走 HF 下载** | 单目离线配置中**必须留空** |
| `provider: cpu` | 存在；**上游默认 yolo.yaml 写的是 `provider: cuda`** | 本项目固定 `cpu` |
| `cam0.rgb_topic` | 存在，上游默认 `/camera/rgb/image_raw` | 改为实际话题 |
| `cam0.depth_topic` / `depth_info_topic` | 存在，**必须删除** | 删除 |
| `plugins` | 上游默认 `["detection","tracking","detection3d","debug"]` | **删除 `detection3d`**（硬性约束） |
| `image_reliability` / `depth_reliability` | 存在 | 保留 `image_reliability`，删 `depth_reliability` |
| `tracker_type` / `track_*` / `match_thresh` / `fuse_score` | 存在（tracking 块） | 保留 |
| `marker_lifetime` | 存在（debug 块） | 保留 |
| `target_frame` | 存在（detection3d 块） | 删除 |
| `img_width` / `img_height` / `n_threads` / `threshold` / `iou` | 存在 | 保留 |
| `model_type` | 计划书未提及（`auto`/`obb`/`pose`/`segment`/`classify`） | 按模型填 `auto` |
| 节点块键名 | `/yolo/yolo_node`，**键必须含 launch namespace** | 严格照抄，改 namespace 需同步改键 |

此外，`launch/yolo.launch.py` 为薄启动器：只接收 `params_file`、`namespace`，**流水线（相机、插件顺序、插件类、全部参数）都定义在 YAML 中**，并叠加 `yolo_bringup/launch_params.py` 生成的标量 CLI 覆盖。本项目应**沿用上游 launch**，只提供自己的 YAML 与覆盖参数。

---

## 7. 源码审计：`image_pipeline-rolling`

### 7.1 版本实证

| 包名 | Rolling 源码版本 | Jazzy 官方二进制版本 |
| --- | --- | --- |
| `camera_calibration` | 8.0.1 | 5.0.13 |
| `image_proc` | 8.0.1 | 5.0.13 |
| `image_view` | 8.0.1 | 5.0.13 |
| `image_pipeline` | 8.0.1 | 5.0.13 |
| `image_publisher` | 8.0.1 | 5.0.13 |
| `image_rotate` | 8.0.1 | 5.0.13 |
| `depth_image_proc` | 8.0.1 | 5.0.13 |
| `stereo_image_proc` | 8.0.1 | 5.0.13 |
| `tracetools_image_pipeline` | 8.0.1 | 5.0.13 |

顶层 `README.md` 的 CI 徽章指向 `ubuntu_resolute_amd64`——**Resolute 是 Ubuntu 26.04，远新于本机的 Noble 24.04**。确认这是 Rolling 分支，**主版本号比 Jazzy 领先 3 个大版本**。

### 7.2 决策

**不编译 `image_pipeline-rolling` 源码。** 依据：

1. 计划书第 0 节第 3 条明确禁止"混用 Rolling 与 Jazzy 依赖"，并要求"优先使用 Jazzy 官方二进制包"；
2. 第 3.1 节已实测 Jazzy 官方包**全部可用**（`image_proc`/`camera_calibration`/`image_view`/`image_pipeline` 均为 5.0.13）；
3. 8.0.1 面向 Ubuntu 26.04 / 更高版本 rclcpp，在 Jazzy 上存在 ABI/API 不兼容风险，且**没有必要**。

`image_pipeline-rolling` 目录**保持原样不修改**，仅作为 API 行为参考（例如 `image_proc` 的 `image_rect` 输出现象、`camera_calibration` 的交互流程）。

---

## 8. 阻塞项（Blockers）

| 编号 | 阻塞项 | 影响阶段 | 证据 | 解除条件 |
| --- | --- | --- | --- | --- |
| **B1** | **Linux 版 MVS SDK 缺失** | Phase 1、2 | 目录中只有 `MVS_Win_STD_5.0.2`（Windows）；虚拟机内 `find` 无任何 MVS/MvCamera 文件 | 用户提供 Linux 版 MVS SDK（`MVS_Linux_STD_*`，架构需为 x86_64） |
| **B2** | **相机未接入虚拟机** | Phase 1、2、4 | `lsusb` 无相机；`/dev/video*` 不存在；该机型为 GigE 相机，USB 直通路线本身不适用 | 用户确认相机与虚拟机的网络可达性，并提供相机 IP/序列号 |
| **B3** | **`github.com` 不可达 → `yolo_ros` 无法配置** | Phase 3 | `curl https://github.com/...` → `code=000`；`yolo_hfhub_vendor` 使用 FetchContent git clone；`yolo_onnxruntime_vendor` 默认从 GitHub Releases 下载约 200MB | 提供 ONNX Runtime 本地/镜像来源 + hfhub-cpp 本地源码，或用可复现方式绕过（见 12.3） |
| **B4** | **模型权重来源未定** | Phase 3 | 上游默认从 Hugging Face Hub 下载；`huggingface.co` 连通性**未测**；目录内**没有任何 .onnx 文件** | 用户提供本地 ONNX 模型 + 类别名文件，或确认 HF 可达 |
| **B5** | **无 GPU** | Phase 3（性能） | 仅 VMware SVGA II；无 `nvidia-smi` | 无需解除，但必须固定 `provider: cpu`，性能预期按 CPU 评估；如需 GPU 需另配硬件 |
| **B6** | **资源紧张** | Phase 1-6 | 2 vCPU / 3.8 GiB RAM / 根分区仅剩 4.2 GB | 建议扩容（加 vCPU/内存/磁盘）；否则 C++ 编译可能因内存或磁盘不足失败 |
| **B7** | **`extras/docs/CODEX_VISION_PLAN.md` 不存在** | 全阶段（文档） | 实际计划书为根目录 URL 编码文件 | 建议由其复制出 `extras/docs/CODEX_VISION_PLAN.md` 作为正式路径（见 12.1） |
| **B8** | **rosdep 未初始化** | Phase 3 | `/etc/ros/rosdep/sources.list.d/` 不存在 | `sudo rosdep init && rosdep update` |

**B1/B2 属于真实硬件验收阻塞，按计划书要求必须标记为"未完成"，不得虚报通过。** `mock/离线` 部分可以先行完成。

---

## 9. 真实话题与接口清单（供下游对接，禁止再猜）

### 9.1 上游 `yolo_ros`（namespace `yolo`，相机名 `cam0`）

| 绝对话题 | 类型 | 发布者 | 备注 |
| --- | --- | --- | --- |
| `/yolo/cam0/detections` | `yolo_msgs/msg/DetectionArray` | DetectionPlugin | 2D 检测主输出 |
| `/yolo/cam0/tracking` | `yolo_msgs/msg/DetectionArray` | TrackingPlugin | `id` 字段被填为轨迹 ID |
| `/yolo/cam0/detections_3d` | `yolo_msgs/msg/DetectionArray` | Detect3DPlugin | **本项目禁用** |
| `/yolo/cam0/debug_image` | `sensor_msgs/msg/Image` | DebugPlugin | 带框可视化 |
| `/yolo/cam0/debug_bb_markers` | `visualization_msgs/msg/MarkerArray` | DebugPlugin | |
| `/yolo/cam0/debug_kp_markers` | `visualization_msgs/msg/MarkerArray` | DebugPlugin | |

订阅参数（`cam0` 块）：`rgb_topic`、`image_reliability`；单目配置**不得出现** `depth_topic` / `depth_info_topic`。

### 9.2 本项目拟建接口（Phase 1-3 落地，命名待与团队确认）

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | 海康相机原始图（计划书建议命名，**待与团队既有命名核对**） |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 未标定时发布 `K`/`P` 全 0 且置 `calibration` 诊断，**绝不填造内参** |
| `/camera/image_rect` | `sensor_msgs/msg/Image` | `image_proc` 校正输出 |
| `/vision/detections` | `yolo_msgs/msg/DetectionArray` | 由 remap 自 `cam0/detections`，供团队稳定依赖 |
| `/vision/debug_image` | `sensor_msgs/msg/Image` | remap 自 `cam0/debug_image` |
| `/vision/status` | `diagnostic_msgs/msg/DiagnosticArray` | 断流/推理失败/延迟 |

### 9.3 明确不存在的接口（禁止假设）

- `/dev/video0`、`/dev/video*`：**不存在**
- 任何雷达话题（`/scan`、`/points` 等）：虚拟机内**未部署任何雷达驱动，未验证**
- 任何 TF 树（`base_link` 等）：**当前不存在**，Phase 4 需与雷达/定位负责人协调
- `/cmd_vel`：**本项目不发布**（硬性约束）

---

## 10. 风险清单

| 级别 | 风险 | 说明 | 缓解 |
| --- | --- | --- | --- |
| 高 | 海康 GigE 相机在 VMware NAT 环境下的可达性未知 | GigE 发现依赖私有网段与广播；NAT 模式通常无法从虚拟机主动发现宿主外的 GigE 设备 | 需与用户确认网络拓扑；可能需桥接模式或独立网卡 |
| 高 | 无 Linux MVS SDK 与任何驱动候选的实证 | 计划书禁止猜测兼容性；目前**没有任何驱动仓库被验证支持 MV-CS050-10GM + Jazzy** | 拿到 SDK 后先做最小采集验证，再决定复用/自研 wrapper |
| 高 | 离线构建 `yolo_ros` | 两个 vendor 包均依赖 GitHub | 见 12.3 的三条可选路径 |
| 中 | CPU 推理性能 | 2 vCPU 纯 CPU 跑 YOLO，帧率可能极低 | 如实实测并记录，不承诺 FPS；必要时降分辨率/换更小模型 |
| 中 | 磁盘仅剩 4.2 GB | colcon 构建 + ONNX Runtime + 模型可能撑爆 | 建议扩容；如不扩容，需 `apt clean`、限制并行、按需构建 |
| 中 | 计划书与上游 schema 不一致 | 计划书 YAML 是示意 | 已在本报告 6.7 逐项对照，实现以源码为准 |
| 低 | Rolling 源码误用 | 存在 8.0.1 源码目录 | 已在 7.2 明确禁用，仅作参考 |

---

## 11. 待用户提供 / 确认的信息

1. **Linux 版 MVS SDK**（x86_64），以及相机是否能被虚拟机网络发现（B1/B2）。
2. **本地 ONNX 模型文件**（含输入尺寸、类别集、许可证）与类别名文件；或确认 `huggingface.co` 可达（B4）。
3. **ONNX Runtime 获取方式授权**：是否允许指定镜像 URL / 使用本地解压包 / 允许 `prepare_offline_bundle.sh`（B3）。
4. **虚拟机扩容**意向：vCPU / 内存 / 磁盘（B6）。
5. **工作区落盘位置**：新建 ROS 工作区放在虚拟机（如 `/home/asd/vision_ws`，可编译）还是 Windows 宿主（如 `D:\无人车\vision_ws`，便于查看）？**建议以虚拟机为编译与运行主体，文档回传宿主**（见 12.1）。
6. **团队既有命名约定**：相机话题、`frame_id`、TF 所有权、是否已有 `map`/`odom`/`base_link` 发布者。
7. **雷达接口**（仅 Phase 4 需要）：类型、话题、坐标系、外参来源。

---

## 12. 建议实施方案

### 12.1 目录与落盘

- 在虚拟机建立工作区：`/home/asd/vision_ws/src/`，宿主 `D:\无人车` 保留源码与文档副本。
- 建议由根目录那份计划书**复制**出 `extras/docs/CODEX_VISION_PLAN.md`，使计划书引用的路径成立（不改动原文件）。
- 新增包（优先新建，不改上游）：

```
vision_ws/src/
├── yolo_ros/                  # 上游源码副本（尽量不改）
├── hikrobot_camera_ros2/      # 相机适配（按 B1/B2 结果决定实现方式）
├── dmu_vision_interfaces/     # 仅在需要新消息时才创建（6.3 表明 MVP 可复用 yolo_msgs）
├── dmu_vision_bringup/        # launch + YAML + 文档入口
└── dmu_vision_tools/          # 诊断/检查/延迟统计
```

### 12.2 阶段推进顺序（受 B1-B4 制约）

| 阶段 | 可立即开展的部分 | 被阻塞的部分 |
| --- | --- | --- |
| Phase 1 | 相机适配包的**接口骨架 + mock 图像源 + 掉线诊断**（可完整单测） | 真实相机采集（B1/B2） |
| Phase 2 | 安装 Jazzy 官方 `image_proc`/`camera_calibration`，用 mock 源验证校正链路 | 真实标定（需真实图像） |
| Phase 3 | 解决 B3 后编译 `yolo_ros`；准备单目 YAML；用静态图/rosbag 验证 | 真实相机实时检测 |
| Phase 4 | 仅设计接口与失效策略，**无外参则不做融合** | 全部实测（需雷达 + 外参） |
| Phase 5 | 写选型对比文档 | 集成（需真实数据） |

### 12.3 解除 B3 的三条可选路径（按推荐度排序）

1. **`-DONNXRUNTIME_ROOT=<本地前缀>`**：用户提供或镜像获取 `onnxruntime-linux-x64-1.20.0.tgz`，解压后指向该前缀。**最干净，完全绕开下载。**
2. **`-DONNXRUNTIME_URL=<可达镜像URL>`**：例如国内镜像站托管的同名 tarball。需用户确认来源可信。
3. **`-DFETCHCONTENT_SOURCE_DIR_YOLO_HFHUB=<本地源码目录>`** 配合 1 或 2，解决 hfhub-cpp 的 git clone。该库仅用于**运行时模型下载**；若模型走本地 `model_path`，其下载功能实际不被调用，但仍需满足 CMake 配置期依赖。

### 12.4 计划书硬性约束的落地自查表

| 约束 | 本报告的落实方式 |
| --- | --- |
| 不猜测设备兼容性 | 4.1/4.2 明确"未验证"，B1/B2 标为阻塞 |
| 不混用 Rolling 与 Jazzy 依赖 | 7.2 明确禁用 Rolling 源码，改用 Jazzy 官方 5.0.13 |
| 单目不启用 `detection3d` | 6.7 明确从 plugins 列表删除，并删除 depth 字段 |
| 不伪造距离/深度/定位 | 9.3 声明无深度来源；第 4 节不臆造外参 |
| 不控制 `/cmd_vel` | 9.3 明确不发布 |
| 未实测不宣称通过 | 全篇区分"实测"与"未验证"，Phase 1 起保留真实硬件验收为未完成 |

---

*报告结束。下一步：等待用户对第 11 节各项的确认，随后进入 Phase 1（相机适配骨架 + mock 图像源 + 诊断），真实相机验收保持"未完成"状态。*
