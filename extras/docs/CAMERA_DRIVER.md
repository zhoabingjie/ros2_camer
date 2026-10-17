# 相机驱动设计与现状（Phase 1）

## 1. 结论先行

| 项目 | 状态 |
| --- | --- |
| 实现语言 | **C++17 / rclcpp（ament_cmake）** |
| 后端抽象层 | ✅ 已实现 |
| `mock` 后端（合成图像） | ✅ 已实现并测试 |
| `mvs` 后端（真实海康相机） | ✅ **已实现采集**（SDK 库 + 头文件已装到 `/opt/MVS`） |
| 真实硬件验收（计划书 T01/T02） | ❌ **未完成**（相机未接入虚拟机，缺桥接网卡） |

> 早期的 Python 实现已归档到 `~/vision_ws/attic/hikrobot_camera_ros2_python_impl/`（带
> `COLCON_IGNORE`，不参与构建），仅作参考。C++ 版的话题、参数、测试覆盖与 Python 版一致。

## 2. 为什么 `mvs` 后端故意不写采集逻辑

计划书第 0 节第 4 条要求"不能确认驱动仓库兼容 MV-CS050-10GM 与 Jazzy 时，不得声称已兼容"。

`MvCameraControl` 的符号名、结构体布局、枚举取值、像素格式常量、错误码，**必须**从 Linux SDK
实际附带的 `MvCameraControl.h` 中读取。凭记忆编写会产生"看起来完整、上车即失败"的代码——这是
最坏的结果。因此本后端只做两件事：

1. **探测** SDK（共享库 / 头文件 / 环境变量），
2. 探测失败时给出**可操作的**报错（列出搜索过的路径），并且**绝不返回伪造帧**。

`~/vision_ws/attic` 中的 Python 版还额外探测了 SDK 的 Python 绑定；C++ 版不需要，故已去掉。

一旦 SDK 到位，实现步骤是：读 `MvCameraControl.h` → 在 `src/mvs_backend.cpp` 中实现采集 →
在真机上核对像素格式与时间戳 → 再移除 `NotImplemented` 分支。

## 3. 源码结构

```
src/hikrobot_camera_ros2/
├── include/hikrobot_camera_ros2/
│   ├── frame.hpp                 # Frame + 编码/几何校验（无厂商依赖）
│   ├── camera_backend.hpp        # 后端抽象接口
│   ├── mock_backend.hpp          # 合成图像后端
│   ├── mvs_backend.hpp           # 海康 MVS 探测（不采集）
│   ├── backend_registry.hpp      # 名字 → 后端；未知名字报可用列表
│   ├── stream_health.hpp         # 流健康看门狗（线程安全）
│   ├── camera_info_factory.hpp   # CameraInfo 构建 / 标定加载与校验
│   └── camera_node.hpp
├── src/*.cpp                     # 对应实现 + camera_main.cpp
├── config/camera.yaml
├── launch/camera.launch.py
└── test/test_*.cpp               # 6 个 gtest，49 个用例
```

## 4. 运行方式

```bash
cd ~/vision_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash

# 合成图像（无硬件，任何机器可跑）
ros2 launch hikrobot_camera_ros2 camera.launch.py

# 切换后端 / frame_id / 标定文件
ros2 launch hikrobot_camera_ros2 camera.launch.py backend:=mock frame_id:=camera_optical_frame
ros2 launch hikrobot_camera_ros2 camera.launch.py calibration_file:=$PWD/config/cam0.yaml

# 真实相机（当前必然报错，属预期行为）
ros2 run hikrobot_camera_ros2 camera_node --ros-args -p backend:=mvs
```

## 5. 接口

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | 原始图像，QoS 默认 `best_effort` depth 5 |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 未标定时为全零 K/P 的"未标定"标记 |
| `/vision/status` | `diagnostic_msgs/msg/DiagnosticArray` | 流健康：状态/帧数/实测 FPS/延迟/丢帧/重开次数/标定状态/后端描述 |

**本节点不发布、也不会发布 `/cmd_vel`**（由 `CameraNodeTest.NeverAdvertisesAVelocityCommand` 守护）。

## 6. 未标定语义

`calibration_file` 为空时发布 `K = P = 全零`、`d = []`、`distortion_model = ''`。
这是 ROS 约定的"未标定"标记，`image_proc` 会据此跳过校正而不是拿假内参去算。
`/vision/status` 中 `calibrated: False` 明示这一点。**任何情况下都不会编造焦距或主点。**

标定文件的分辨率与实时流不一致时，标定会被**拒绝**（抛 `CalibrationMismatchError`），不会静默缩放。
畸变系数个数**不做固定长度校验**（`plumb_bob` 5 个、`rational_polynomial` 8 个、`equidistant` 4 个），
避免把合法的标定误判为非法。

## 7. 依赖

`rclcpp`、`sensor_msgs`、`diagnostic_msgs`、`yaml_cpp_vendor`（`libyaml-cpp-dev` 本机已装）。
未引入 `camera_info_manager`：标定校验需要显式的分辨率比对与"拒绝而非缩放"语义，自己实现更可控，
且少一层依赖（磁盘紧张的机器上这一点有意义）。
