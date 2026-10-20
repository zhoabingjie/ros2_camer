# yolo_ros

上游 YOLO ONNX 推理与插件宿主包，提供 `yolo_node`、检测、跟踪和 debug 插件。本项目未修改其算法源码。

正式视觉入口 `dmu_vision_bringup/vision.launch.py` 会把输出映射到 `/vision/*`；直接使用上游 `yolo_bringup` 时仍使用 `/yolo/cam0/*`。

单目配置只启用 `detection`、`tracking`、`debug`，不启用 `detection3d`。
