# MVS 参考实现对比与驱动加固记录

参考仓库：<https://github.com/Chlzzz/HikRobotCamera_Node>

对比日期：2026-10-21。

参考项目的 ROS 2 环境标注为 Ubuntu 22.04 / Humble，相机型号为 `MV-CU013-A0UC`。本项目为 Ubuntu 24.04 / ROS 2 Jazzy / `MV-CS050-10GM`，因此参考项目只用于核对 MVS API 使用方式，不作为直接移植来源。

## 1. 文件对照

| 本项目 | 参考项目 | 作用 |
|---|---|---|
| `src/hikrobot_camera_ros2/src/mvs_backend.cpp` | `src/hik_camera.cpp` | MVS 设备、取流和图像转换 |
| `src/hikrobot_camera_ros2/src/camera_node.cpp` | `src/hik_camera.cpp` | ROS 2 图像与 CameraInfo 发布 |
| `src/hikrobot_camera_ros2/include/hikrobot_camera_ros2/mvs_backend.hpp` | `include/hik_camera/hik_camera.hpp` | 后端接口 |
| `src/hikrobot_camera_ros2/CMakeLists.txt` | `CMakeLists.txt` | SDK 路径和构建依赖 |

## 2. 功能对比

| 功能 | 当前 `ros2_camer` | 参考 `HikRobotCamera_Node` | 结论 |
|---|---|---|---|
| SDK 加载 | `dlopen` 动态加载 | CMake 直接链接 | 当前方式保留，Mock 机器仍可构建 |
| SDK 路径 | CMake、环境变量、实际探测路径 | 写死 `/opt/MVS/include`、`/opt/MVS/lib/64` | 已进一步改为优先实际路径 |
| SDK 初始化 | 已调用 `MV_CC_Initialize` | 未显式调用 | 已补齐 |
| SDK 反初始化 | 相机进程 `main` 受控调用 `MV_CC_Finalize` | 未调用 | 已补齐并避开静态析构崩溃 |
| SDK 版本 | 读取 `MV_CC_GetSDKVersion` | 无 | 已补齐 |
| 设备枚举 | 型号、序列号、IP、掩码 | IP 或用户名 | 当前更完整 |
| 设备选择 | 只支持一台 GigE；0 台或多台都拒绝 | 固定第一台 | 当前更明确 |
| 打开设备 | 错误抛出并进入状态诊断 | 错误直接 `exit(-1)` | 当前更适合 ROS 节点 |
| 采集参数 | 宽高、曝光、增益、像素格式、GigE 包大小 | 参数更多，但大量固定值 | 当前保留最小可控参数 |
| 取流 | `MV_CC_GetOneFrameTimeout`、超时重试 | 同一 API，失败累计后 `exit(-1)` | 当前保留 |
| 缓冲大小 | 按 `PayloadSize` 动态分配 | 固定 `4*2048*3072` | 当前保留动态方式 |
| 帧尺寸 | 使用 `MV_FRAME_OUT_INFO_EX` 实际宽高 | 转换固定 `1280x1024` | 当前正确 |
| 源长度 | 严格校验 `nFrameLen <= PayloadSize` | 固定最大长度 | 已加强 |
| 目标缓冲 | 按实际宽高计算并检查溢出 | 固定最大长度 | 已加强 |
| 转换返回值 | 检查 `MV_CC_ConvertPixelTypeEx` | 忽略返回值 | 当前更安全 |
| 转换输出长度 | 校验 `nDstLen == width*height` | 不检查 | 已加强 |
| 图像生命周期 | 复制到 `Frame::data` 后发布 | OpenCV `clone()` 后复制 | 两者安全，当前少一次无必要拷贝 |
| 像素编码 | 直通 Mono8/BGR8/RGB8；其他转 Mono8 | 固定转 BGR8 | 当前适合单目灰度相机 |
| 时间戳 | 优先相机 host timestamp | ROS 当前时间 | 当前更接近采集时刻 |
| frame_id | 配置的光学坐标系 | 帧序号字符串 | 当前正确 |
| CameraInfo | 标定 YAML、分辨率一致性校验、未标定全零 | `camera_info_manager` | 两者都可用；当前保持零伪造 |
| 断线恢复 | 状态诊断、关闭、退避重连 | 无 | 当前保留 |
| `/cmd_vel` | 不发布，测试约束 | 不发布 | 安全要求满足 |

## 3. 参考实现中的风险

1. 转换宽高固定为 `1280x1024`，与实际帧不同会导致尺寸或颜色错误。
2. 输入和输出缓冲固定为超大常量，未依据 `PayloadSize` 或实际目标格式。
3. `MV_CC_ConvertPixelType` 返回值未检查。
4. 初始化、打开、取流累计失败和析构错误直接 `exit(-1)`，不利于 ROS 2 生命周期管理。
5. 永远选择第一台相机。
6. `frame_id` 使用帧序号字符串，无法用于 TF。
7. 无掉线重连。
8. 编译时写死 MVS 路径，不支持其他安装位置。
9. 相机的触发和分辨率参数存在固定配置，未必适用于本项目设备。

## 4. 本次最小修改

### CMake

- 使用 `find_path` 搜索 `MvCameraControl.h`。
- 搜索顺序优先 `MVCAM_SDK_PATH`、`MVS_SDK_ROOT`、`HIKROBOT_MVS_ROOT`。
- `/opt/MVS` 只作为回退路径。
- 将实际找到的 include 路径传给运行时探测。

### 运行时 SDK 路径

- 环境变量和编译时实际路径优先。
- 兼容 `MVCAM_COMMON_RUNENV`。
- 不再按路径字符串长度重新排序库候选。
- 动态库加载失败时清除残留函数指针。

### SDK 生命周期

- `MV_CC_Initialize` 每个进程只调用一次。
- `MV_CC_Finalize` 在所有相机句柄关闭后，由 `camera_main` 受控调用。
- 未把 `Finalize` 放入 C++ 静态析构器；MVS 5.0.2 的 GEV 插件在该顺序下会崩溃。

### 图像取流和转换

- 检查 `nFrameLen != 0`。
- 检查 `nFrameLen <= PayloadSize`。
- Mono8 检查 `nFrameLen >= width*height`。
- BGR8/RGB8 检查 `nFrameLen >= width*height*3`。
- 直通格式只复制实际图像字节数。
- 转换输出按实际宽高分配，并做乘法和 `unsigned int` 上限检查。
- 检查转换后的 `nDstLen`。
- 不使用参考项目的固定 `1280x1024`。

## 5. 当前实测状态

| 项目 | 结果 |
|---|---|
| 当前 Linux MVS 安装包 | MVS 5.0.2 x86_64 |
| 运行时 `MV_CC_GetSDKVersion` | `4.8.1.2` |
| 动态库 | `/opt/MVS/lib/64/libMvCameraControl.so.4.8.1.2` |
| ROS 2 | Jazzy |
| 构建 | 通过 |
| 相机包测试 | 通过 |
| 全工作区测试 | 404 tests / 0 failures / 0 errors |
| 无相机启动 | 返回码 1，明确报错，Finalize 后无崩溃 |
| 真实相机枚举 | `devices=0`，当前未连接相机 |
| 真实图像发布与取流 | **未验证** |

真实相机连接后必须重新执行：

```bash
ros2 run hikrobot_camera_ros2 camera_node --ros-args -p backend:=mvs
ros2 topic hz /camera/image_raw
ros2 topic echo --once /camera/camera_info
```

只有这些命令实际产生真实数据，才能将实机接入标记为通过。
