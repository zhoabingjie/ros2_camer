# 相机标定流程（Phase 2）

> **状态：流程与链路已验证；真实相机标定未完成**（阻塞项 B1/B2：无 Linux MVS SDK、相机未接入）。

## 1. 需要准备的东西

| 物品 | 要求 |
| --- | --- |
| 标定板 | 棋盘格。默认按 **9×7 个方格**（= 8×6 个内角点）准备；尺寸必须**实测**，别信标称值 |
| 打印精度 | 贴在硬质平板上，不能起皱或弯曲（弯曲会让重投影误差虚高） |
| 相机状态 | 安装位置、镜头焦距、对焦环、光圈与**实际使用时完全一致**；改变分辨率/ROI/binning 必须重标 |

`--size` 填的是**内角点数**，不是方格数。9×7 方格的板子 → `size:=8x6`。
`--square` 填**米**，用卡尺量实际打印出来的方格边长。

## 2. 采集与标定

```bash
cd ~/vision_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash

ros2 launch dmu_vision_bringup calibration.launch.py \
    backend:=mvs size:=8x6 square:=0.025 camera_name:=cam0
```

`cameracalibrator` 会开一个 OpenCV 窗口，**必须移动标定板**直到四个进度条都变绿：

- **X**：标定板水平方向覆盖画面左右
- **Y**：垂直方向覆盖画面上下
- **Size**：远近都要有（占画面 1/3 ~ 满屏）
- **Skew**：要有倾斜（绕 X/Y/Z 轴都有转角）

建议采集 30~50 个姿态。然后点 **CALIBRATE → SAVE**。

> 本适配层**尚未实现** `<camera>/set_camera_info` 服务（`camera_info_manager` 那套），
> 所以 launch 里带了 `--no-service-check`，SAVE 不会把参数推给相机节点，而是落到文件：

```bash
ls -l /tmp/calibrationdata.tar.gz
```

## 3. 转换为 camera_info YAML

用官方自带的命令行工具（无需 GUI）：

```bash
ros2 run camera_calibration tarfile_calibration /tmp/calibrationdata.tar.gz --mono -s 8x6 -q 0.025
```

会生成 `ost.yaml`（单目）。把它放到项目里：

```bash
mkdir -p ~/vision_ws/config/calibration
mv ost.yaml ~/vision_ws/config/calibration/cam0.yaml
```

检查重投影误差（**这一步不能跳过**）：

```bash
grep -E "image_width|image_height|reprojection_error|fx|fy" ~/vision_ws/config/calibration/cam0.yaml
```

- `reprojection_error` 一般应 < 1.0 px；明显偏大说明标定板弯了、图糊了或姿态覆盖不足，重标。
- `image_width` / `image_height` 必须等于实际采集分辨率。

## 4. 加载并验证

```bash
ros2 launch dmu_vision_bringup preview.launch.py \
    backend:=mvs \
    calibration_file:=$HOME/vision_ws/config/calibration/cam0.yaml
```

验证要点：

1. 启动日志出现 `calibration loaded: <W>x<H> from ...`；
2. `ros2 topic echo --once /vision/status` 中 `calibrated: True`；
3. `ros2 topic hz /camera/image_rect` 有稳定输出；
4. 用 `image_view` 对比原始与校正图像：

```bash
ros2 run image_view image_view --ros-args -r image:=/camera/image_rect
```

## 5. 两条必须知道的行为（已实测，见 TEST_REPORT.md）

### 5.1 `image_proc` 的输入话题叫 `image`，不是 `image_raw`

`image_proc::RectifyNode` 的输入话题是 **`image`**。官方 `image_proc.launch.py` 里靠
`('image', 'image_mono')` / `('image', 'image_color')` 重映射。若按直觉当成 `image_raw`，
节点会照常启动、照常发布 `/camera/image_rect` 话题，**但永远不出数据，且不报错**。
`preview.launch.py` 已经做了 `remappings=[("image", "image_raw")]`。

### 5.2 未标定时 `image_proc` 会拒绝校正，并刷 ERROR 日志

CameraInfo 的 `K` 全零（本项目的"未标定"诚实标记）时，`image_proc` 每帧打印：

```
[ERROR] [camera.RectifyNode]: Rectified topic '/camera/image_rect' requested but
        camera publishing '/camera/camera_info' is uncalibrated
```

**这是正确且期望的行为**：宁可不出图，也不拿编造的内参去校正。因此：

- 未标定阶段请看 **`/camera/image_raw`**（原始图像，正常发布）；
- `/camera/image_rect` 只有在提供有效标定文件后才有数据。

不要为了让 `/camera/image_rect` "有输出" 而随手造一份标定文件——那正是项目规则禁止的。
