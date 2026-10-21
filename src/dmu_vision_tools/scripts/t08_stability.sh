#!/bin/bash
# T08 stability observation for the REAL camera pipeline.
#
# Usage: t08_stability.sh [duration_s] [sample_interval_s]
# Default: 600 s / 60 s
#
# Requires a connected MVS camera. The script intentionally has no image replay
# fallback: if the camera cannot start, the observation fails.
set +u
DURATION="${1:-600}"
INTERVAL="${2:-60}"
WS="$HOME/vision_ws"
OUT="$WS/artifacts/t08_stability_$(date +%Y%m%d_%H%M%S).log"

cd "$WS" || exit 1
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-95}"
mkdir -p "$WS/artifacts"

pkill -f "ros2 launch dmu_vision_bringup vision.launch.py" 2>/dev/null
pkill -f "hikrobot_camera_ros2/lib/hikrobot_camera_ros2/camera_node" 2>/dev/null
pkill -f "yolo_ros/lib/yolo_ros/yolo_node" 2>/dev/null
pkill -f "visual_obstacle_avoidance/lib/visual_obstacle_avoidance/visual_obstacle_node" 2>/dev/null
sleep 2

ros2 launch dmu_vision_bringup vision.launch.py start_obstacle:=false \
  > /tmp/t08_pipeline.log 2>&1 &
sleep 5
if ! pgrep -f "lib/yolo_ros/yolo_node" >/dev/null; then
  echo "ERROR: real camera pipeline did not start" | tee -a "$OUT"
  grep -E "no Hikvision|unavailable|process has died" /tmp/t08_pipeline.log | tee -a "$OUT"
  exit 1
fi

DURATION="$DURATION" INTERVAL="$INTERVAL" OUT="$OUT" python3 - <<'PY' 2>&1 | tee -a "$OUT"
import os, subprocess, time
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import Image
from yolo_msgs.msg import DetectionArray

DURATION = float(os.environ["DURATION"])
INTERVAL = float(os.environ["INTERVAL"])
RELIABLE = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)

def rss_kb():
    pids = subprocess.run(["pgrep", "-f", "lib/yolo_ros/yolo_node"],
                          capture_output=True, text=True).stdout.split()
    if not pids:
        return -1
    out = subprocess.run(["ps", "-o", "rss=", "-p", pids[0]],
                         capture_output=True, text=True).stdout.strip()
    return int(out) if out else -1

class Monitor(Node):
    def __init__(self):
        super().__init__("t08")
        self.detections = 0
        self.debug_images = 0
        self.latency_ms = None
        self.create_subscription(
            DetectionArray, "/vision/detections", self.on_detection,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, "/vision/debug_image", self.on_debug_image, RELIABLE)

    def on_detection(self, msg):
        self.detections += 1
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if stamp_ns > 0:
            self.latency_ms = (
                self.get_clock().now().nanoseconds - stamp_ns) / 1e6

    def on_debug_image(self, _):
        self.debug_images += 1

print("warm-up 25 s (model load)...", flush=True)
start_warmup = time.time()
rclpy.init()
node = Monitor()
while time.time() - start_warmup < 25.0:
    rclpy.spin_once(node, timeout_sec=0.1)

print(f"=== T08 observation: duration={DURATION:.0f}s interval={INTERVAL:.0f}s ===",
      flush=True)
print("elapsed_s  yolo_alive  rss_kb  det_msgs  dbg_msgs  last_lat_ms", flush=True)
start = time.time()
next_sample = INTERVAL
rss_series = []
while time.time() - start < DURATION:
    rclpy.spin_once(node, timeout_sec=0.1)
    elapsed = time.time() - start
    if elapsed >= next_sample:
        rss = rss_kb()
        rss_series.append(rss)
        print(f"{elapsed:9.0f}  {'yes' if rss >= 0 else 'NO':>9}  {rss:7d}  "
              f"{node.detections:8d}  {node.debug_images:8d}  "
              f"{'n/a' if node.latency_ms is None else round(node.latency_ms, 1)}",
              flush=True)
        next_sample += INTERVAL
print(f"FINAL det_msgs={node.detections} dbg_msgs={node.debug_images} "
      f"duration={DURATION:.0f}s", flush=True)
if rss_series:
    print(f"RSS first={rss_series[0]} last={rss_series[-1]} "
          f"delta_kb={rss_series[-1]-rss_series[0]}", flush=True)
rclpy.shutdown()
PY

echo "=== after observation ===" | tee -a "$OUT"
pgrep -f "lib/yolo_ros/yolo_node" >/dev/null \
  && echo "yolo_node STILL ALIVE (no crash)" | tee -a "$OUT" \
  || echo "yolo_node DIED" | tee -a "$OUT"
echo "error_lines=$(grep -cE 'ERROR|died' /tmp/t08_pipeline.log 2>/dev/null || echo 0)" | tee -a "$OUT"
grep -E "ERROR|died" /tmp/t08_pipeline.log 2>/dev/null | head -5 | tee -a "$OUT"

pkill -f "ros2 launch dmu_vision_bringup vision.launch.py" 2>/dev/null
pkill -f "hikrobot_camera_ros2/lib/hikrobot_camera_ros2/camera_node" 2>/dev/null
pkill -f "yolo_ros/lib/yolo_ros/yolo_node" 2>/dev/null
echo "T08_DONE log=$OUT" | tee -a "$OUT"
