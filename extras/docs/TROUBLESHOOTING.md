# 故障排查手册

> 本手册里的每一条都来自本项目**实际踩过的故障**和实测输出，不是通用建议汇编。

## 0. 第一步永远是先跑体检

```bash
source /opt/ros/jazzy/setup.bash && source ~/vision_ws/install/setup.bash
ros2 run dmu_vision_tools vision_health_check --measure 5
```

它会逐项检查并给出 `PASS` / `PASS WITH WARNINGS` / `FAIL`，**退出码 0/1**，可直接接进 CI 或脚本。

```
[OK  ] topic image            /camera/image_raw (sensor_msgs/msg/Image), 1 publisher(s)
[OK  ] qos image              1 publisher(s) x 1 subscriber(s) all compatible
[OK  ] rate image             9.89 Hz
[OK  ] staleness image        last message 0.102 s old
[OK  ] frame_id consistency   both use 'camera_optical_frame'
[WARN] calibration            UNCALIBRATED: K is all zero. ...
[OK  ] stream status          reported state 'ok'
```

常用参数：`--image` `--camera-info` `--status` `--measure`（测量秒数）`--min-rate` `--max-age`。

---

## 1. 「话题都在，但一帧数据都收不到」—— 先查 QoS

这是本项目**最高频**的一类故障，特征是：`ros2 topic list` 有话题、`ros2 node list` 有节点、
日志里**没有任何报错**，但数据永远不来。

**根因**：ROS 2 的 QoS 请求-提供规则。发布者提供 BEST_EFFORT 时，RELIABLE 的订阅者
**永远收不到数据，而且不会报错**。

`vision_health_check` 会直接点名：

```
[FAIL] qos image    /camera_node -> /evil_reliable_subscriber: publisher offers
                    BEST_EFFORT but subscriber requests RELIABLE -> no data will ever be delivered
```

### 1.1 本项目已踩过的三个实例

| 实例 | 表现 | 根因 | 处置 |
| --- | --- | --- | --- |
| `image_proc` 的 `rectify_node` | `/camera/image_rect` 存在但零帧；`ros2 topic info /camera/image_raw` 显示 `Subscription count: 0` | **输入话题名叫 `image`，不是 `image_raw`** | `preview.launch.py` 加 `remappings=[("image", "image_raw")]` |
| `yolo_ros` 的 `image_reliability` | 检测节点正常启动但收不到图 | 该参数 **1 = RELIABLE**，而相机发 BEST_EFFORT。源码 `qos_compat.hpp`：0=系统默认、1=RELIABLE、其它=BEST_EFFORT | 配置里填 **2** |
| 任何自写的 RELIABLE 订阅者 | 静默无数据 | 同上 | 订阅端改 BEST_EFFORT，或发布端改 RELIABLE |

### 1.2 手动排查命令

```bash
ros2 topic info -v /camera/image_raw        # 看 Publisher / Subscription count 与 Reliability
ros2 node info /camera/RectifyNode          # 看它到底订阅了什么
```

`ros2 node info` 只显示 `/parameter_events` 一个订阅 → **它根本没订阅你的图像话题**（话题名不对，
不是 QoS 问题）。`Subscription count: 0` 同理。

> 注意：`ros2 topic list` / `ros2 topic info` 偶发返回空/Unknown，是 **DDS 发现还没完成**。
> 启动后至少等 10 秒再判定。

---

## 2. `/camera/image_rect` 没有数据

**先确认是不是没标定。** 未标定时 `image_proc` 会每帧打印：

```
[ERROR] [camera.RectifyNode]: Rectified topic '/camera/image_rect' requested but
        camera publishing '/camera/camera_info' is uncalibrated
```

这是**正确行为**（宁可不出图，也不拿编造的内参去校正）。处置：

- 未标定阶段看 `/camera/image_raw`；
- 要 `image_rect` 就按 `extras/docs/CAMERA_CALIBRATION.md` 做真实标定；
- **不要**为了让 `image_rect`"有输出"而随手造一份标定文件。

标定文件分辨率与实时流不一致时，我们的节点会**拒绝加载**并打印
`calibration rejected: ... re-run camera_calibration at the current resolution`。

---

## 3. apt 下载失败

### 3.1 IPv6 不可达导致 apt 卡住

现象：`无法发起与 mirrors.xxx:443 (2402:...) 的连接 - connect (101: 网络不可达)`。

这台虚拟机没有可用的 IPv6 路由，而 apt 会优先尝试 AAAA 记录。已通过
`/etc/apt/apt.conf.d/99force-ipv4`（`Acquire::ForceIPv4 "true"`）永久解决。

### 3.2 镜像本身不可达

实测（2026-10-16）：

| 镜像 | ROS 2 | Ubuntu |
| --- | --- | --- |
| `mirrors.tuna.tsinghua.edu.cn` | ❌ 当时整体不可达 | ❌ |
| **`mirrors.ustc.edu.cn`** | ✅ 558 KB/s | ✅ 685 KB/s（`apt update` 达 5 MB/s） |
| `mirrors.bfsu.edu.cn` | ⚠️ 302 重定向 | ✅ 1.2 MB/s |
| `mirror.sjtu.edu.cn` | ❌ 404（不镜像 ROS 2） | ⚠️ 302 |

当前配置为 USTC，原 TUNA 配置备份在 `/etc/apt/sources.list.d/ubuntu.sources.tuna.bak`。

### 3.3 ROS GPG 密钥过期

```
下列签名无效： EXPKEYSIG F42ED6FBAB17C654 Open Robotics
E: 仓库 "http://packages.ros.org/ros2/ubuntu noble InRelease" 没有数字签名。
```

`packages.ros.org/ros.key` 与被广泛复制的 `raw.githubusercontent.com/.../ros.key`
都是**陈旧副本**（自签名有效期止于 2025-06-01）。正确来源是 keyserver：

```bash
gpg --keyserver hkps://keyserver.ubuntu.com --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
gpg --export C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 | sudo tee /usr/share/keyrings/ros-archive-keyring.gpg >/dev/null
```

---

## 4. colcon 构建问题

### 4.1 `Duplicate package names not supported`

工作区里有两份同名 `package.xml`（例如归档目录里存着旧版实现）。
给不该参与构建的目录加 `COLCON_IGNORE` 即可，本项目的
`attic/`、`thirdparty/`、`src/yolo_ros-main/`（历史原因）都用这个机制。

### 4.2 `Package 'xxx' not found`

忘了 source 工作区：

```bash
source /opt/ros/jazzy/setup.bash
source ~/vision_ws/install/setup.bash     # 这一句最容易漏
```

### 4.3 `set -u` 与 ROS setup.bash 冲突

```
/opt/ros/jazzy/setup.bash: 行 8: AMENT_TRACE_SETUP_FILES: 未绑定的变量
```

`setup.bash` 会引用未定义变量，脚本里先 `set +u` 再 source。

---

## 5. GitHub 不可达时怎么构建 `yolo_ros`

实测：`github.com` ❌、`raw.githubusercontent.com` ❌、`cdn.jsdelivr.net` ❌，
但 **`codeload.github.com` ✅、`ghfast.top` ✅、`ghproxy.net` ⚠️（极慢且会静默截断）**。

```bash
colcon build --packages-select yolo_msgs yolo_onnxruntime_vendor yolo_hfhub_vendor yolo_ros yolo_bringup \
  --cmake-args \
    -DONNXRUNTIME_ROOT=$HOME/vision_ws/thirdparty/onnxruntime-linux-x64-1.20.0 \
    -DFETCHCONTENT_SOURCE_DIR_YOLO_HFHUB=$HOME/vision_ws/thirdparty/huggingface-hub-cpp
```

- 取 ORT release：**用 `ghfast.top`**（约 100 KB/s，需给足超时；`ghproxy.net` 实测只有约 7 KB/s
  且会返回 code=200 的**残缺文件**，`tar` 报"异常的 EOF"）。下完务必核对字节数与 `Content-Length` 一致。
- 取 hfhub-cpp 源码：`codeload.github.com/<owner>/<repo>/tar.gz/refs/tags/<tag>`。
- **不要**直接把 apt 的 `ros-jazzy-onnxruntime-vendor` 前缀拿来做 `ONNXRUNTIME_ROOT`：
  它**不含** `LICENSE` / `ThirdPartyNotices.txt`，而上游 vendor 的 `install(FILES ...)` 会安装它们，
  会在 install 阶段失败。

---

## 6. `yolo_node` 一启动就退出

### 6.1 缺模型（最常见）

```
[ERROR] on_activate failed: ... Load model from .../yolo26n.onnx failed: File doesn't exist
```

**这是期望行为**（宁可启动失败，也不"启动成功但无检测结果"）。按 `models/README.md` 放好模型即可。
注意模型路径、`img_width/img_height`、`model_type` 必须匹配。

### 6.2 缺插件参数

```
[ERROR] on_configure failed: plugin 'detection': missing parameter 'detection.plugin'
```

`plugins` 里列出的每个插件，都必须在 YAML 里有对应块（含 `plugin:` 字段）。

### 6.3 无 GPU 机器上的默认值陷阱

`detection` 块的这些默认值都是 GPU 取向，无 GPU 时必须显式覆盖（否则至少一条警告，
严重时尝试加载 CUDA/TensorRT 路径）：

| 参数 | 上游默认 | 本项目 |
| --- | --- | --- |
| `provider` | `"auto"` | `cpu` |
| `device` | **`"cuda:0"`** | `cpu` |
| `cuda_graph_enable` | `true` | `false` |
| `trt_fp16_enable` | `true` | `false` |
| `trt_engine_cache_enable` | `true` | `false` |

---

## 7. 虚拟机自身

| 现象 | 原因 | 处置 |
| --- | --- | --- |
| 宿主机 ping 不通虚拟机 | 宿主 `VMnet8` 网卡没拿到 DHCP 租约（只剩 169.254 地址） | `ipconfig /renew "VMware Network Adapter VMnet8"` |
| 虚拟机 SSH 连不上 | Ubuntu 桌面版默认不装 `openssh-server` | `sudo apt install openssh-server` |
| 以后要接 GigE 相机 | NAT 模式下 UDP 广播穿不过去，发现不了 GigE 设备 | 加**第二块**桥接网卡（VMnet0），不要动现有 NAT 网卡，否则会断掉 SSH/VS Code |

---

## 8. `/yolo/cam0/debug_image` 一帧都不发

**先看你的图像源帧率是不是高于实测推理帧率。**

`DebugPlugin` 要把 `(CameraFrame, DetectionArray)` **按时间戳配对**才能画框发布，
而它的同步队列深度只有 **10**（`debug_plugin.cpp:119-122`）。当推理慢于源帧率时，
检测结果出来时对应的那一帧已经被挤出队列，配对永远失败 —— 表现为：

- 话题存在、publisher 是 yolo_node、`ros2 topic info -v` 正常
- `ros2 topic hz` 和 RELIABLE 订阅者都收不到任何帧
- **日志里没有任何报错**

实测对照（同一台 2 vCPU 机器）：

| 输入分辨率 | 图像源帧率 | detections | debug_image |
| --- | --- | --- | --- |
| 810×1080 | 10 Hz | 有（约 1 Hz） | **0 帧** |
| 810×1080 | 2 Hz | 有 | 正常 |
| **640×480** | 10 Hz | 有（约 4.5 Hz） | **正常，约 6 Hz** |

**结论**：能否配对取决于"推理是否跟得上源帧率"，而不是源帧率的绝对值。
分辨率越小推理越快，越容易跟上 —— **优先降低输入分辨率**，而不是一味降源帧率。

**处置**：把图像源帧率降到不高于实测推理帧率。用
`ros2 run dmu_vision_tools measure_detection --seconds 30` 先量出真实检测帧率，再据此设源帧率。

诊断命令：

```bash
ros2 topic info -v /yolo/cam0/debug_image   # 看 Publisher count 与 Reliability
ros2 topic hz /yolo/cam0/debug_image        # 收不到 = 配对失败
```


---

## 9. 延迟突然劣化 / 帧率掉一半

**先看内存和磁盘，不要先怀疑代码。**

实测（同一份代码、同一份配置）：

| 机器状态 | 延迟 mean | 延迟 P95 |
| --- | --- | --- |
| 正常 | 38.8 ms | 92.4 ms |
| **磁盘 96% 满** | **102.4 ms** | **174.9 ms** |

清理出 1.2 GB 后平均延迟立刻回到 38.8 ms。诊断命令：

```bash
df -h /                 # 根分区是否接近满
free -h                 # 特别是 swap 用了多少
sudo du -sh /var/cache/apt   # 常见的大块头
```

本次事故：`/var/cache/apt` 积了 1.2 GB 的 `.deb` 缓存。处置：

```bash
sudo systemctl stop unattended-upgrades   # 否则 apt 会报 lock 被占
sudo apt-get clean
```

另外，本机在观测期间 swap 用了 1.4 GiB / 3.6 GiB —— **3.8 GB 内存跑 YOLO 明显不足**，
这是延迟尖峰（实测出现 316 ms）的可信来源。要稳定性能请加内存。

> 别被 `du /` 的结果骗了：`/media/asd` 下挂着两个 Ubuntu 安装 ISO（`/dev/sr0`、`/dev/sr1`，
> 共约 6.3 GB），它们是只读挂载，**不占根分区**。
