# 海康 MV-CS050-10GM 相机接入

## 2026-10-21 驱动加固

- 已根据当前 MVS 头文件加入 `MV_CC_Initialize`。
- `MV_CC_Finalize` 在所有相机句柄释放后，由 `camera_main` 受控调用。
- 未将 `Finalize` 放入静态析构器；实测 MVS 5.0.2 的 GEV 插件在该顺序下会崩溃。
- CMake 优先搜索环境变量和实际安装路径，`/opt/MVS` 仅作为回退。
- 采集转换严格使用实际帧宽高和 `nFrameLen`，不复制参考项目的固定 `1280x1024`。
- 运行时 SDK 版本实测为 `4.8.1.2`，与当前 `libMvCameraControl.so.4.8.1.2` 一致。

详细对比见 `extras/docs/MVS_REFERENCE_COMPARISON.md`。

## 1. 实现状态

| 项目 | 状态 |
|---|---|
| 语言 | C++17 / rclcpp |
| Linux MVS SDK | 已安装到 `/opt/MVS` |
| 真实 MVS API | 已按 SDK 头文件实现 |
| 相机枚举 | 已实现，失败时输出检查清单 |
| 真实取流 | **未完成硬件验收** |
| Mock 后端 | 已实现，必须显式启用 |

当前 MVS 实测：

```text
no Hikvision camera found by the MVS SDK (devices=0)
```

当前宿主机 Realtek 以太网仍为：

```text
Status: Disconnected
LinkSpeed: 0 bps
```

因此问题在物理链路或虚拟机桥接，不在 ROS 代码。

## 2. 驱动链路

```text
MV-CS050-10GM
    |
    | GigE
    v
Windows 实体网卡
    |
    | VMware Bridged 第二网卡
    v
Ubuntu ens37/ens38
    |
    | MVS SDK
    v
camera_node
    |
    +-- /camera/image_raw
    +-- /camera/camera_info
    +-- /vision/status
```

NAT 网卡 `VMnet8` 必须保留，不能改成桥接，否则 SSH 和 VS Code 会断开。

## 3. 硬件和网络检查

1. 确认相机铭牌要求的供电方式。普通网线不保证供电。
2. 相机网口直接接电脑实体网口，确认两端网口灯亮。
3. Windows 检查：

```powershell
Get-NetAdapter -Name "以太网" |
  Format-List Name,Status,LinkSpeed,MediaConnectionState,AdminStatus
```

必须看到 `Status: Up` 和非零 `LinkSpeed`。

4. 给虚拟机增加第二块 Bridged 网卡，绑定实体以太网适配器。
5. 不要修改原 NAT 网卡。
6. 在 Ubuntu 查看新接口：

```bash
ip -brief address
ip route
```

7. 根据相机 IP 配置同网段地址。例如仅当相机确认是 `192.168.1.x/24` 时：

```bash
sudo ip addr add 192.168.1.100/24 dev ens37
sudo ip link set ens37 up
```

不要在不了解实际网段时照抄上面的 IP。

## 4. 启动和枚举

```bash
cd ~/vision_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run hikrobot_camera_ros2 camera_node \
  --ros-args -p backend:=mvs
```

正常枚举会打印类似：

```text
devices=1 [GigE model='MV-CS050-10GM' serial='...' ip=... mask=...]
```

当前驱动只支持一台 GigE 相机；检测到 0 台或多台都会拒绝启动。

## 5. 参数

| 参数 | 含义 |
|---|---|
| `backend` | `mvs` 或显式 `mock` |
| `require_camera_at_startup` | 启动失败是否返回非零 |
| `mvs_pixel_format` | `Mono8`、`BGR8`、`RGB8` |
| `mvs_width` / `mvs_height` | 0 表示保持相机当前值 |
| `mvs_exposure_us` | 0 表示保持当前值 |
| `mvs_gain` | 0 表示保持当前值 |
| `mvs_packet_size` | 0 表示使用 SDK 推荐值 |
| `calibration_file` | 标定 YAML；空表示未标定 |

## 6. 错误处理

- SDK 未找到：列出搜索路径。
- 相机数为 0：输出电源、网线、L2 网段和 `rp_filter` 检查清单。
- 枚举到的 GigE 相机数量不是 1：拒绝启动并列出设备。
- 打开设备失败：提示可能被 MVS 客户端或其他程序占用。
- 取流异常：关闭并重新打开后端。
- 退出：停止采集、关闭设备、销毁句柄。

## 7. 真实验收命令

```bash
ros2 topic hz /camera/image_raw
ros2 topic echo --once /camera/camera_info
ros2 run dmu_vision_tools vision_health_check --measure 5
```

只有这些命令在真实相机连接后实际产生数据，才能把真实相机接入标记为通过。
