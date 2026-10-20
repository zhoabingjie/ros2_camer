# dmu_vision_interfaces

团队稳定使用的视觉消息接口。

## 发布消息

- `dmu_vision_interfaces/msg/Obstacle2D`
  - 单个二维障碍物候选。
  - 字段单位为像素，不包含距离、深度或三维坐标。
- `dmu_vision_interfaces/msg/VisualObstacleArray`
  - 包含状态、输入有效性和二维障碍物候选列表。
  - 发布话题：`/vision/obstacles`。

## 安全约束

单目相机只有二维检测框，不能据此生成真实米制距离。下游模块如果直接使用该消息，应只进行二维风险判断，不能宣称已完成三维避障。
