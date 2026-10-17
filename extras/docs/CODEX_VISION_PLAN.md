# 无人车视觉子系统工程实施计划书（交给 Codex 执行）

**项目环境：** Ubuntu 24.04、ROS 2 Jazzy\
**相机：** 海康机器人
MV-CS050-10GM，单台单目工业相机（最终彩色/灰度输出格式需由实际设备确认）\
**现有参考源码：**
`yolo_ros-main(1).zip`、`image_pipeline-rolling(1).zip`\
**系统上下游：**
差速底盘、Nav2、激光雷达、GPS、IMU、码盘/轮式里程计由团队其他成员负责\
**目标：**
形成可构建、可启动、可测试、接口清晰的视觉子系统，支持相机接入、图像预览、相机标定、二维目标检测/跟踪、视觉辅助避障接口，以及可选的视觉里程计/SLAM集成。

------------------------------------------------------------------------

## 0. 给 Codex 的执行规则（必须遵守）

你是本项目的 ROS 2 Jazzy
视觉子系统开发工程师。请在当前工作区内先审查现有仓库与源码，再按阶段实现，不要一开始就大规模改写。

1.  **先审查，再编码。** 输出当前仓库目录树、系统版本、CPU 架构、ROS
    发行版、可用 GPU、已安装 MVS
    SDK、现有相机驱动和现有话题。没有证据时标注"待现场确认"，不得猜测。
2.  **不要破坏原项目。** 保留 `yolo_ros` 与 `image_pipeline-rolling`
    原始目录/代码；优先在本项目新建独立包。除非有明确理由且测试证明必要，否则不要直接修改上游源码。
3.  **Jazzy 优先。** `image_pipeline-rolling` 是 Rolling
    分支，不得默认它与 Jazzy ABI/API 完全兼容。优先使用 Ubuntu 24.04 /
    Jazzy
    官方二进制包；只有缺包或确有修改需求时，才挑选必要源码包，并逐个核验依赖。
4.  **海康驱动先核验。** 查明 MVS SDK
    版本、安装路径、架构、相机是否能被官方工具采集、SDK
    示例/API、像素格式及许可。不能确认驱动仓库兼容 MV-CS050-10GM 与
    Jazzy 时，不得声称"已兼容"。把相机驱动做成可替换适配层。
5.  **不虚构设备和模型。** 不要假定存在
    `/dev/video0`、`/camera/image_raw`、GPU、CUDA、TensorRT 或某个 ONNX
    文件。先检查。所有模型下载须记录来源、文件路径、类别集、输入尺寸和许可证；网络不可用时支持本地模型路径。
6.  **单目相机没有真实深度。** 默认只实现 2D 检测/跟踪。禁止把
    `yolo_ros` 默认 `detection3d` 插件用于无深度相机；不得将单目 YOLO
    框中心直接伪装成 3D 点或米制距离。
7.  **安全优先。** 视觉模块默认只发布感知结果，不直接控制底盘，不直接向
    `/cmd_vel`
    发布运动指令。任何视觉辅助避障输出都必须有有效性、时间戳、超时和失效策略；最终安全制动由底盘/Nav2/独立安全逻辑负责。
8.  **接口参数化。**
    话题名、frame_id、相机序列号、图像编码、模型路径、阈值、处理频率、QoS、日志等级都必须参数化，提供
    YAML 示例和 launch 参数。
9.  **测试优先。**
    每个阶段完成后构建并运行测试；记录实际命令、输出、未通过项和原因。不能仅写代码不验证。
10. **分阶段提交。**
    每一阶段结束先报告文件改动、构建/测试结果、风险和下一阶段计划；不要擅自删除用户文件或执行危险系统操作。

------------------------------------------------------------------------

## 1. 项目目标和功能范围

### 1.1 必须完成（MVP）

-   A. 相机接入适配：将海康 MVS 相机图像转换为 ROS 2
    `sensor_msgs/msg/Image`，并发布有效的
    `sensor_msgs/msg/CameraInfo`（标定前可发布未标定状态，但不能伪造内参）。
-   B. 图像预览：可使用 `rqt_image_view`
    查看原始图像、校正图像和检测可视化图像。
-   C. 相机标定：使用 Jazzy 兼容的 `camera_calibration`
    工作流生成标定结果，保存 YAML，加载后发布正确 CameraInfo。
-   D. 2D 目标检测：复用现有 `yolo_ros` + ONNX
    Runtime；提供单目配置，不启用深度插件。
-   E. 目标跟踪（可配置）：沿用 yolo_ros 中的 tracking
    插件；可通过参数关闭。
-   F. 视觉可视化：发布带检测框、类别、置信度的 debug
    image，必要时发布结构化 detection topic。
-   G. 感知结果统一接口：新增本项目自己的 ROS 2 消息/节点或适配节点，将
    YOLO
    输出转换成稳定、文档化的接口，避免团队其他模块强依赖上游私有细节。
-   H.
    视觉辅助避障接口：把二维检测结果与雷达/TF/相机标定信息对接，明确支持哪些融合模式；初始版本仅输出"候选目标/风险区域"，不得宣称由单目图像独立测得准确距离。
-   I.
    基础监控：图像断流、处理超时、节点异常、模型加载失败均有清晰日志与状态话题/诊断信息。
-   J. 一键启动、安装说明、参数模板、测试和故障排查文档。

### 1.2 分阶段完成（增强功能）

-   K.
    相机与激光雷达外参标定工具/流程，以及图像目标与雷达点/扫描数据的空间关联。
-   L. 视觉里程计或 SLAM
    集成评估。优先先做可复现的集成方案，不要自行从零实现完整 SLAM。
-   M. 与轮式里程计、IMU、GPS
    的定位融合接口。具体融合节点由定位负责人确定，视觉包负责输出约定的视觉里程计/位姿消息。
-   N. 录包、回放、性能基准、室外光照测试、故障注入测试。

### 1.3 明确不在本任务内

-   不修改底盘驱动，不接管 Nav2 planner/controller，不发布最终
    `/cmd_vel`。
-   不保证单目相机可以直接提供精确物体距离。
-   不把 Rolling 源码整体强行移植到 Jazzy。
-   不承诺在未知硬件上达到固定 FPS；必须先实测。
-   不在缺乏训练数据和验证集时宣称目标识别模型适用于所有道路/障碍物。

------------------------------------------------------------------------

## 2. 开始编码前的源码与环境审计

请先执行并记录以下检查；根据实际目录调整命令，不要覆盖已有文件。

``` bash
lsb_release -a
uname -m
printenv ROS_DISTRO
which ros2
ros2 doctor --report
nvidia-smi 2>/dev/null || true
lsusb
find /opt /usr/local -maxdepth 4 \( -iname '*MVS*' -o -iname '*MvCamera*' \) 2>/dev/null | head -200
```

在工作区中查找源码：

``` bash
find . -maxdepth 3 -type f \( -name package.xml -o -name CMakeLists.txt -o -name README.md \) | sort
find . -maxdepth 4 -type f \( -name '*.yaml' -o -name '*.launch.py' -o -name '*.msg' -o -name '*.srv' \) | sort
```

审查 `yolo_ros-main`： - `yolo_ros/`：核心
`yolo_node`、插件、订阅和输出话题。 - `yolo_msgs/msg/`：实际
Detection/DetectionArray/BoundingBox 等消息定义。 -
`yolo_bringup/config/yolo.yaml`：默认
`plugins: ["detection", "tracking", "detection3d", "debug"]`，相机块包含
RGB 与 depth topic，且默认
`provider: cuda`。本项目必须复制并建立独立单目配置，移除 depth 字段和
`detection3d`，并根据硬件选 CPU 或可验证的 GPU provider。 -
`yolo_bringup/launch/yolo.launch.py`：确认参数文件节点键名与 namespace
的对应关系，遵循上游启动方式。 -
`extras/docs/models.md`、`extras/docs/build.md`、`extras/docs/pipelines.md`：确认模型格式、模型下载/本地加载、依赖和话题约定。 -
查明 `DebugPlugin`、`DetectionPlugin`、`TrackingPlugin`
实际发布的话题名和消息类型；不得只凭 README 猜接口。

审查 `image_pipeline-rolling`： - 重点核对
`camera_calibration`、`image_proc`、`image_view` 和 `image_pipeline` 的
package.xml、setup/launch、ROS API。 - 先检查 Jazzy
系统包是否已提供：`apt-cache policy ros-jazzy-camera-calibration ros-jazzy-image-proc ros-jazzy-image-view ros-jazzy-image-pipeline`。 -
若官方包存在，优先安装官方包；不要把 Rolling 包和 Jazzy
二进制依赖混用。 -
如果必须从源码构建，选择最小所需子包，并逐一确认分支、依赖、编译结果；在文档中解释为什么需要源码版本。

审计报告应保存为 `extras/docs/ENVIRONMENT_AUDIT.md`，至少包含：系统/架构、ROS
发行版、MVS 状态、相机识别状态、驱动候选与证据、YOLO
构建依赖、可用模型、实际话题、已知风险、尚需人工提供的信息。

------------------------------------------------------------------------

## 3. 推荐的软件包结构

在项目现有 ROS 2 工作区中创建独立包，不要把所有逻辑塞进一个节点：

``` text
vision_ws/
└── src/
    ├── yolo_ros/                       # 上游代码副本/子模块，尽量不改
    ├── image_pipeline/                 # 默认不编译 Rolling 全部源码
    ├── hikrobot_camera_ros2/            # 本项目相机驱动适配包（按审计决定实现/复用）
    ├── dmu_vision_interfaces/            # 本项目稳定消息接口
    ├── dmu_vision_bringup/               # launch、参数、RViz、文档入口
    ├── dmu_vision_tools/                 # 诊断、检查、可选录包工具
    ├── dmu_vision_fusion/                # 视觉与雷达关联接口，第二阶段
    └── dmu_visual_odometry/              # 仅在选定并验证具体 VO/SLAM 后加入
```

如果团队仓库已有命名规则，优先遵守现有规则，避免重复创建同类包。`hikrobot_camera_ros2`
不应在未核实 SDK API
前假装已完成：先做驱动接口调查和最小采集验证，再决定复用驱动或实现 SDK
wrapper。

### 包职责

-   `hikrobot_camera_ros2`：相机枚举、打开/关闭、采集线程、图像格式转换、时间戳、CameraInfo
    发布、错误恢复、相机参数配置。
-   `dmu_vision_interfaces`：团队稳定使用的消息接口。
-   `dmu_vision_bringup`：一键启动、YAML、launch 参数、可选组件编排。
-   `dmu_vision_tools`：相机状态检查、话题检查、延迟统计、诊断。
-   `dmu_vision_fusion`：第二阶段进行相机目标与雷达点/扫描的关联；不得在没有
    TF 和外参时做伪融合。
-   `dmu_visual_odometry`：仅作为选定 VO/SLAM
    的封装和配置，不要在该包里重新实现整套 SLAM。

------------------------------------------------------------------------

## 4. ROS 2 话题、消息和 TF 合约

以下是**建议的默认接口**。先检查团队现有接口；如已有同义话题，采用团队现有命名并更新文档，避免重复发布。

  -------------------------------------------------------------------------------------------------------------------------------------
  数据              建议话题                                   类型                                    发布方/说明
  ----------------- ------------------------------------------ --------------------------------------- --------------------------------
  原始图像          `/camera/image_raw`                        `sensor_msgs/msg/Image`                 相机驱动

  相机内参          `/camera/camera_info`                      `sensor_msgs/msg/CameraInfo`            相机驱动/标定参数加载器

  校正图像          `/camera/image_rect`                       `sensor_msgs/msg/Image`                 `image_proc`

  检测可视化        `/vision/debug_image`                      `sensor_msgs/msg/Image`                 YOLO debug 输出或适配节点

  YOLO 2D 检测      以源码实际发布名为准，建议统一到           上游 `yolo_msgs/msg/DetectionArray`     通过适配节点映射，不猜话题名
                    `/vision/detections`                       或本项目接口                            

  目标跟踪          `/vision/tracks`（若上游消息能稳定表达）   以实际消息定义为准                      可选

  感知状态          `/vision/status`                           `diagnostic_msgs/msg/DiagnosticArray`   断流、推理失败、延迟
                                                               或本项目状态消息                        

  视觉候选风险      `/vision/obstacle_candidates`              本项目定义的消息                        只表示候选风险，不冒充精确距离

  视觉里程计        `/visual_odom`                             `nav_msgs/msg/Odometry`                 仅在 VO/SLAM 阶段启用
  -------------------------------------------------------------------------------------------------------------------------------------

### TF 约定

-   相机光学坐标系建议使用 `camera_optical_frame`，但应与驱动真实
    frame_id 一致。
-   相机固定安装关系应由 URDF 或静态 TF 发布，例如
    `base_link -> camera_link -> camera_optical_frame`；不得让多个节点重复发布同一
    TF。
-   常用 REP-103 光学坐标轴约定：x 向右、y 向下、z
    向前。实际驱动若已使用其他约定，必须核验并正确转换。
-   雷达 TF 由雷达负责人维护；相机-雷达外参必须标定后才能做几何融合。
-   `map`、`odom`、`base_link` 的所有权应与
    Nav2/定位负责人协调。视觉包不能擅自发布重复的 `map->odom` 或
    `odom->base_link`。

### QoS 与时间戳

-   图像默认采用适合传感器流的 QoS（优先
    `SensorDataQoS`），并允许通过参数/launch 配置。
-   每帧使用相机采集时间戳；若设备 SDK 无法提供 ROS
    可用时钟，记录时间戳策略和时钟域。
-   检查 `header.stamp`、`frame_id`、Image 与 CameraInfo 的一致性。
-   不能为了"匹配成功"而忽略过期数据；融合模块必须设置最大时间差和超时。
-   对高分辨率图像使用合理队列深度，防止积压；优先处理最新帧。

------------------------------------------------------------------------

## 5. 相机驱动实施计划（阶段 1）

### 5.1 驱动选择顺序

1.  现场确认 MVS 官方工具能否识别并实时采集 MV-CS050-10GM。
2.  记录 CPU 架构（`x86_64` 或 `aarch64`）、Ubuntu 版本、MVS SDK
    版本和安装路径。
3.  核验候选 ROS 2 驱动是否明确支持当前
    SDK、Jazzy、设备型号和实际像素格式。候选仓库只能视为调查对象，不能直接视为已兼容。
4.  若候选驱动构建与采集通过，优先复用；否则用海康 MVS SDK 的官方 API
    实现独立 ROS 2 wrapper，并将 SDK 依赖封装在单独包内。
5.  驱动与 YOLO 节点解耦，未来可替换为其他相机源或 rosbag 回放。

### 5.2 驱动功能要求

-   参数：`camera_serial`、`camera_index`、`frame_id`、`image_topic`、`camera_info_topic`、`pixel_format`、`exposure_time`、`gain`、`frame_rate`、`width`、`height`、`trigger_mode`、`publish_camera_info`、`camera_info_url`、`reconnect_on_failure`。
-   参数必须依据 SDK
    支持能力设置；不支持的功能需返回明确错误，不可静默忽略。
-   自动枚举设备并打印型号、序列号、采集模式、像素格式。
-   支持启动/停止、正常退出时释放相机和 SDK
    资源；采集失败时有超时和重连策略。
-   检查 SDK 缓冲区生命周期：发布 Image 前必须将数据安全复制到 ROS
    消息或保证底层内存有效，不能发布后立即复用造成花屏。
-   正确处理单色/彩色格式、步长 `step`、`encoding` 和字节序；禁止把
    Mono8 冒充 BGR8。
-   相机硬件时间戳和 ROS
    时间戳转换须明确；无法转换时采用系统采集时间并记录。
-   未标定时可以发布未标定 CameraInfo（K/P
    未标定），但必须告知用户不能用于几何校正；不得填造焦距/主点。
-   相机掉线后报告诊断状态，重新连接成功后恢复发布。

### 5.3 验收

-   官方工具能采集，或明确记录无法采集的硬件/驱动阻塞原因。
-   ROS 话题持续发布，`ros2 topic hz /camera/image_raw` 有稳定输出。
-   `ros2 topic echo --once /camera/camera_info` 的 frame_id
    和图像一致；未标定时状态明确。
-   连续运行 10 分钟无崩溃、明显内存持续增长、花屏或错误编码。
-   模拟断开相机后，节点有错误日志并按配置恢复或安全停止。

------------------------------------------------------------------------

## 6. 图像预览和标定（阶段 2）

### 6.1 依赖和预览

优先安装 Jazzy 对应的二进制包，具体包名以 apt 查询结果为准。例如：

``` bash
sudo apt update
sudo apt install ros-jazzy-rqt-image-view ros-jazzy-camera-calibration ros-jazzy-image-pipeline
```

如果某个包名不可用，先用 `apt-cache policy`
查询并在文档里说明，不要因此直接编译 Rolling 全仓库。

启动预览：

``` bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
ros2 topic hz /camera/image_raw
ros2 run rqt_image_view rqt_image_view
```

在图像查看器中选择实际图像话题。再检查：

``` bash
ros2 topic info -v /camera/image_raw
ros2 topic echo --once /camera/camera_info
```

### 6.2 标定

-   用合适尺寸的棋盘格/标定板；确保相机安装位置和焦距/镜头状态与使用时一致。
-   使用 `camera_calibration` 对单目相机标定；启动参数要根据 Jazzy
    实际命令帮助和话题名填写，不得硬编码猜测。
-   采集覆盖画面中心、四角、不同距离、不同倾角的图像。
-   保存标定 YAML 到 `dmu_vision_bringup/config/camera_info.yaml`
    或用户指定位置，并将文件路径参数化。
-   检查重投影误差、图像尺寸和标定文件与当前采集分辨率是否一致。
-   若更改分辨率、ROI、binning 或镜头，必须重新验证标定参数。
-   `image_proc` 订阅原始图像与 CameraInfo，发布校正图像；验证
    `/camera/image_rect` 画面和时间戳正常。

### 6.3 验收

-   原始图像和校正图像都能查看。
-   标定参数可持久化，重启后能加载。
-   标定 YAML 与图像宽高匹配，CameraInfo 矩阵不是随意填写的默认值。
-   标定前后图像话题、编码、frame_id 和时间戳符合预期。

------------------------------------------------------------------------

## 7. YOLO 目标识别与跟踪（阶段 3）

### 7.1 复用上游，不重造轮子

先在审计报告中确认上游构建方式和参数，再编译 `yolo_ros`。其 README
说明运行时为 C++/ONNX Runtime，默认 CPU 可用于 x86_64/aarch64；GPU
需要匹配 CUDA/cuDNN 和 ONNX Runtime provider。不要仅凭"机器有 NVIDIA
显卡"就启用 CUDA。

``` bash
cd <实际工作区>
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash
```

如果上游包的 vendor 配置需要下载 ONNX
Runtime，需记录下载是否成功；离线环境需按上游脚本支持的方式准备离线依赖。不要绕过许可证或下载不明二进制。

### 7.2 创建单目参数文件

新建 `dmu_vision_bringup/config/yolo_mono.yaml`，以源码当前版本的参数
schema 为准。目标结构示意如下（字段必须在实现前与实际源码逐项核对）：

``` yaml
/yolo/yolo_node:
  ros__parameters:
    plugins: ["detection", "tracking", "debug"]
    cameras: ["cam0"]
    cam0:
      rgb_topic: /camera/image_rect
      image_reliability: 1
    detection:
      plugin: yolo_ros/DetectionPlugin
      cameras: ["cam0"]
      model_path: "/absolute/path/to/model.onnx"
      threshold: 0.5
      iou: 0.45
      n_threads: -1
      provider: cpu
    tracking:
      plugin: yolo_ros/TrackingPlugin
      cameras: ["cam0"]
      tracker_type: bytetrack
    debug:
      plugin: yolo_ros/DebugPlugin
      cameras: ["cam0"]
      marker_lifetime: 0.5
```

注意： - 上述为**结构示例**，Codex 必须对照当前源码确认
`model_path`、`rgb_topic`、`image_reliability`、`provider`
等字段的实际名称、合法值和默认值，再提交可运行配置。 - 彻底删除
`depth_topic`、`depth_info_topic` 和 `detection3d`
插件；不能留有需要深度流的配置。 - Jazzy 的 ROS 参数 YAML
不应依赖空列表表达关闭插件；要明确列出需要的插件。 - `provider` 默认先选
CPU。只有完成 GPU 驱动、CUDA/cuDNN、ONNX Runtime provider
验证和性能测试后，才提供独立 GPU 配置。 -
模型路径优先支持本地文件。模型文件不存在时启动失败应给出明确路径提示，不应静默下载不明模型。 -
目标类别集要写入文档。COCO
预训练模型只能识别其训练类别，不能保证识别任意自定义障碍物。

### 7.3 输出和适配

-   检查实际上游发布的 detection、tracking、debug image 话题和类型。
-   在 `dmu_vision_bringup` 或单独 adapter 中提供
    remap/适配，不修改上游内部消息含义。
-   如上游 DetectionArray
    不包含项目需要的稳定字段，可新建接口包；字段至少包括 Header、类别
    ID/名称、置信度、二维框中心/宽高、可选 track ID
    和检测来源。消息字段需遵循 ROS 2 interface 规范。
-   debug image 应与输入帧尽可能对应，保持合理时间戳。
-   不要把 detection2d 直接转换成 Detection3D/PointStamped 并编造 z
    距离。
-   设置阈值、推理频率、图像分辨率和队列长度参数，记录端到端延迟与丢帧率。

### 7.4 验收

-   模型成功加载，节点启动无参数解析错误。
-   `ros2 topic list` 中出现经验证的检测、跟踪（如启用）、debug image
    话题。
-   使用静态图像或 rosbag 回放时，检测类别和框位置合理。
-   摄像头实时图像进入 YOLO 后，节点稳定运行至少 10 分钟。
-   输出实际 FPS、平均/95 分位延迟、CPU/GPU 占用；不要编造性能指标。
-   无深度输入时，系统不启动 detection3d，也不发布虚假的 3D 结果。

------------------------------------------------------------------------

## 8. 视觉辅助避障：安全的分层设计（阶段 4）

### 8.1 设计原则

视觉目标检测负责"这可能是什么"，雷达/其他测距来源负责"它在哪里/距离多少"。视觉结果可以帮助对障碍物分类、识别行人/车辆/锥桶等，但仅凭单目框不能直接输出可靠距离。

### 8.2 推荐数据流

``` text
海康相机
  └── /camera/image_raw + /camera/camera_info
       ├── image_proc → /camera/image_rect
       ├── yolo_ros → 2D detections + debug image
       └── dmu_vision_fusion（第二阶段）
             ├── 订阅 2D detections
             ├── 订阅雷达数据（LaserScan/PointCloud2，按团队现有接口）
             ├── 使用 camera_info + 已标定 camera↔lidar 外参 + TF
             ├── 时间同步、投影/关联、置信度与失效判断
             └── 发布 /vision/obstacle_candidates 或项目约定的融合输出
```

### 8.3 融合实现要求

-   先和雷达/Nav2 负责人共同确定输入话题、坐标系、TF
    所有权、消息类型、更新频率和代价地图接入方式。
-   相机和雷达必须有有效外参；没有外参时只能输出 2D
    检测结果，不得输出融合距离。
-   对
    LaserScan，必须考虑扫描平面高度与相机视野差异；不能简单将框中心像素与同角度雷达束视为同一物体。
-   对
    PointCloud2，可将点变换到相机坐标系后投影到图像平面，再在检测框内进行稳健聚合；需排除无效点、离群点、时序错配。
-   结果至少应包含目标类别、置信度、关联距离（仅在有效雷达数据和标定满足时）、目标坐标系、时间戳、数据有效标志、关联点数量/质量指标。
-   需要参数：同步容差、雷达有效距离、框内点比例、最小点数、离群值过滤、超时、类别风险映射、置信度阈值。
-   雷达断流、TF
    缺失、外参未标定、图像过期、同步失败时，必须把结果标记为无效并记录诊断；不能把上次有效障碍物无限期保留。
-   输出先用于 RViz/rosbag 离线验证。Nav2
    接入由团队负责人评审后再开启；优先考虑向现有 costmap
    提供已验证的传感器数据/插件接口，而不是由视觉节点控制速度。
-   视觉不应抑制雷达已发现的障碍物；融合只允许增强语义，不得削弱已有安全障碍物层，除非团队完成安全评审。

### 8.4 验收

-   用已知距离和位置的静态目标验证关联结果。
-   在 RViz 中检查相机框、雷达点和融合目标在 TF 下的位置是否一致。
-   记录不同距离、光照、目标类别下的关联成功率和误关联情况。
-   注入雷达断流、TF 缺失、相机断流、时间戳异常，验证输出失效策略。
-   Nav2
    集成前必须由团队明确审查通过；未经审查不得实车启用视觉避障闭环。

------------------------------------------------------------------------

## 9. 视觉辅助定位 / 视觉里程计（阶段 5，单独验收）

### 9.1 先做技术选型，不要直接自研

单目 VO/SLAM
的平移尺度通常存在不确定性；相机运动模糊、光照变化、重复纹理和动态物体会影响定位。应先对比维护状态、Jazzy
支持、相机模型、资源需求、输出接口和许可，再选择已有方案，例如基于视觉/视觉惯性的
SLAM 或 VO 框架。选型结果必须写入 `extras/docs/VO_SLAM_DECISION.md`。

### 9.2 集成约束

-   输入使用已验证的图像、CameraInfo；若方案需要 IMU，使用团队已有 IMU
    topic，并明确同步要求。
-   输出标准化为 `nav_msgs/msg/Odometry` 或该方案原生定位消息；文档明确
    pose/twist 的坐标系和协方差。
-   明确 TF 发布策略，避免与现有定位融合节点重复发布 `odom->base_link`
    或 `map->odom`。
-   如果融合由 robot_localization
    或其他定位节点负责，视觉节点只发布测量值，由定位负责人配置融合。
-   单目 VO 不得假装有绝对米制尺度。只有在与
    IMU/轮速/已知尺度或其他可靠传感器完成尺度估计后，才能作为米制里程计候选。
-   必须能关闭视觉定位，不影响相机预览和 YOLO 感知。

### 9.3 验收

-   用 rosbag 回放可重复得到轨迹，记录丢跟踪、重定位、漂移和计算资源。
-   与轮式里程计/IMU/GPS 的时间戳、frame_id 和协方差约定一致。
-   通过短距离、回环/重复场景、光照变化测试；不达标则保持为实验功能，不接入导航闭环。
-   文档明确能力边界和失效检测方式。

------------------------------------------------------------------------

## 10. Bringup、参数和命令

在 `dmu_vision_bringup` 中提供： -
`camera.launch.py`：只启动相机驱动。 - `preview.launch.py`：相机 + 可选
image_proc + 图像查看工具（查看器可单独运行）。 -
`detection.launch.py`：相机话题已存在时启动 YOLO 与可选 debug/track。 -
`vision.launch.py`：按参数组合启动相机、校正、YOLO、诊断；默认不启动雷达融合和
VO/SLAM。 - `fusion.launch.py`：第二阶段，默认关闭，需要显式参数开启。 -
`vo.launch.py`：仅选型通过后提供，默认关闭。

建议参数文件：

``` text
dmu_vision_bringup/config/
├── camera.yaml
├── camera_info.yaml
├── image_proc.yaml
├── yolo_mono.yaml
├── yolo_cpu.yaml
├── yolo_gpu.yaml                 # 仅验证 GPU 后提交
├── fusion.yaml
├── diagnostics.yaml
└── rviz/vision.rviz
```

命令应以实际包名和实际 launch 参数为准。Codex 必须在
`extras/docs/QUICKSTART.md` 中写出从干净 Jazzy
环境到运行预览、检测、标定、rosbag
回放的完整命令；命令执行前检查文件和包是否存在。不要在 README 中留下
`<实际工作区>` 等未替换占位符作为唯一操作指南。

------------------------------------------------------------------------

## 11. 测试与验收体系

### 11.1 自动化测试

-   `colcon build --symlink-install` 构建全工作区。
-   `colcon test` 和 `colcon test-result --verbose`。
-   对自建 Python 节点使用 pytest/ament_pytest；C++ 节点使用
    ament_cmake_gtest。
-   对参数 YAML、launch 文件和消息接口增加基本测试。
-   用 mock 图像源测试断流、异常编码、无效
    CameraInfo、空模型路径和模型加载失败。
-   对 adapter 验证输入消息与输出消息字段、时间戳和 frame_id 不丢失。

### 11.2 手动测试表

  ------------------------------------------------------------------------------------------------------
  编号                    测试                    通过标准
  ----------------------- ----------------------- ------------------------------------------------------
  T01                     MVS 官方采集            能识别设备并持续取图；失败时明确记录阻塞原因

  T02                     ROS 图像发布            Image 持续发布，encoding/step/尺寸正确

  T03                     CameraInfo              frame_id 与图像一致；标定状态真实

  T04                     预览                    rqt_image_view 可显示原始图像

  T05                     标定与校正              YAML 可加载，校正图像正常

  T06                     YOLO 启动               ONNX 模型成功加载，无深度插件

  T07                     检测可视化              类别/框/置信度合理，debug image 可查看

  T08                     稳定性                  连续运行 10 分钟无崩溃、无持续积压

  T09                     断流恢复                相机断开、模型错误、TF 缺失时有诊断且不输出伪有效数据

  T10                     雷达融合                外参标定后，已知目标的关联位置合理

  T11                     VO/SLAM                 轨迹可重复、坐标系与融合策略明确

  T12                     实车安全                仅经团队评审后开展；验证视觉失败不削弱原雷达安全功能
  ------------------------------------------------------------------------------------------------------

### 11.3 性能报告

记录： - 相机分辨率、采集频率、图像编码； - YOLO
模型文件、输入尺寸、推理 provider； - 输入 FPS、检测
FPS、端到端延迟（平均值及 P95）、丢帧率； - CPU/GPU、内存占用； -
环境光照、测试场景、目标类别和距离； -
任何失败、限制、模型许可和已知风险。

所有数值必须来自实测，不得由 Codex 编造。

------------------------------------------------------------------------

## 12. 文档与交付物

必须提交以下文件或对应项目现有文档：

``` text
extras/docs/
├── ENVIRONMENT_AUDIT.md
├── ARCHITECTURE.md
├── TOPICS_AND_TF.md
├── QUICKSTART.md
├── CAMERA_DRIVER.md
├── CAMERA_CALIBRATION.md
├── YOLO_SETUP.md
├── OBSTACLE_FUSION.md
├── VO_SLAM_DECISION.md
├── TEST_REPORT.md
└── TROUBLESHOOTING.md
```

代码交付： - 可构建的 ROS 2 包与 `package.xml`/`CMakeLists.txt`/setup
配置； - launch 文件和 YAML 参数； - 需要时新增的 `.msg`/`.srv`； -
自动化测试； - 安装与运行脚本； - 许可证/第三方依赖说明； -
一键预览与一键检测流程； - 不包含未经授权的 SDK
二进制、密钥、个人路径或大型模型权重，模型文件按文档单独获取。

------------------------------------------------------------------------

## 13. 建议实施顺序与阶段门禁

### Phase 0：审计（不改业务代码）

交付环境报告、源码结构、依赖分析、相机驱动选择建议、真实话题/消息清单。门禁：确认系统架构、ROS
Jazzy 和 MVS 状态。

### Phase 1：相机最小接入

实现或复用相机驱动，发布
Image/CameraInfo，完成掉线诊断。门禁：真实设备稳定发布图像。

### Phase 2：预览与标定

接入 Jazzy 的 image_proc /
camera_calibration，生成并加载标定参数。门禁：原始图像、校正图像和
CameraInfo 均可验证。

### Phase 3：YOLO 2D 检测/跟踪

建立单目配置、模型加载、debug
image、统一检测接口和测试。门禁：实时/回放检测通过，不含虚假 3D 距离。

### Phase 4：雷达-视觉关联

在相机与雷达外参可用后，先离线/录包验证，再输出候选风险接口。门禁：TF、同步、失效策略和实测结果通过评审。

### Phase 5：视觉里程计/SLAM

完成方案比较、选型和封装，按定位团队约定输出位姿/里程计。门禁：轨迹与坐标系评审通过后才允许融合。

### Phase 6：集成验收

运行自动化和手动测试，提供性能报告、问题列表、启动指南、已知限制和交接说明。

每阶段结束时，Codex 应暂停并报告： 1. 新增/修改文件； 2. 执行过的命令；
3. 构建和测试结果； 4. 未解决问题及原因； 5.
是否需要用户提供设备信息或人工操作； 6. 下一阶段的计划。
如果环境缺少相机、MVS SDK、模型或雷达数据，仍可完成
mock/离线部分，但必须把真实硬件验收标为"未完成"，不能虚报通过。

------------------------------------------------------------------------

## 14. 给 Codex 的最终执行提示词

请从 Phase 0
开始，先审计当前工作区和两个已提供源码包，再按阶段实现。目标是交付可构建、可测试、文档完整的
ROS 2 Jazzy 视觉子系统。优先完成真实相机接入、图像预览/标定、YOLO 2D
检测/跟踪和诊断；再实现有真实外参支持的雷达-视觉融合；最后才评估并集成视觉里程计/SLAM。

**硬性约束：** 不猜测相机驱动兼容性，不混用 Rolling 与 Jazzy
二进制依赖，不在单目配置启用
`detection3d`，不伪造距离/深度/定位结果，不直接控制
`/cmd_vel`，不在没有实测的情况下声称功能通过。先输出审计报告，再开始代码修改。每阶段必须真实构建和测试，失败时先定位并修复，不要仅通过删测试或降低验收标准来"变绿"。
