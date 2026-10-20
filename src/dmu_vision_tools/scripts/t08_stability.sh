#!/bin/bash
# T08 stability observation (plan section 11.2).
#
# Runs the detection pipeline on a static COCO test image for N seconds and
# samples node liveness, RSS, detection throughput and latency every interval.
#
# Usage:  t08_stability.sh [duration_s] [sample_interval_s] [image_path]
# Default: 600 s / 60 s / /tmp/bus_640.jpg
#
# Writes a durable log under vision_ws/artifacts/ (NOT /tmp, which a reboot can
# clear -- that actually happened once during this project).
set +u
DURATION="${1:-600}"
INTERVAL="${2:-60}"
IMAGE="${3:-/tmp/bus_640.jpg}"
WS="$HOME/vision_ws"
OUT="$WS/artifacts/t08_stability_$(date +%Y%m%d_%H%M%S).log"

cd "$WS" || exit 1
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-95}"

if [ ! -f "$IMAGE" ]; then
  echo "ERROR: test image not found: $IMAGE" | tee -a "$OUT"
  echo "       create one with: curl -L -o /tmp/bus.jpg https://ghfast.top/https://raw.githubusercontent.com/ultralytics/ultralytics/main/ultralytics/assets/bus.jpg" | tee -a "$OUT"
  exit 1
fi

sed 's|/camera/image_rect|/camera/image_raw|' \
  install/dmu_vision_bringup/share/dmu_vision_bringup/config/yolo_mono.yaml > /tmp/yolo_mono_raw.yaml

pkill -f yolo_ros/yolo_node 2>/dev/null
pkill -f dmu_vision_tools/image_file_source 2>/dev/null
pkill -f detection.launch.py 2>/dev/null
sleep 2

ros2 run dmu_vision_tools image_file_source --ros-args \
  -p filename:="$IMAGE" -p topic:=/camera/image_raw -p rate_hz:=10.0 > /tmp/t08_src.log 2>&1 &
ros2 launch dmu_vision_bringup detection.launch.py params_file:=/tmp/yolo_mono_raw.yaml > /tmp/t08_det.log 2>&1 &

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

class M(Node):
    def __init__(self):
        super().__init__("t08")
        self.d = 0; self.g = 0; self.lat = None
        self.create_subscription(DetectionArray, "/vision/detections",
                                 self.on_det, qos_profile_sensor_data)
        self.create_subscription(Image, "/vision/debug_image", self.on_dbg, RELIABLE)
    def on_det(self, m):
        self.d += 1
        s = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        if s > 0:
            self.lat = (self.get_clock().now().nanoseconds - s) / 1e6
    def on_dbg(self, _): self.g += 1

# let the model load before the clock starts
print("warm-up 25 s (model load)...", flush=True)
t0 = time.time()
rclpy.init(); n = M()
while time.time() - t0 < 25.0: rclpy.spin_once(n, timeout_sec=0.1)

print(f"=== T08 observation: duration={DURATION:.0f}s interval={INTERVAL:.0f}s ===", flush=True)
print("elapsed_s  yolo_alive  rss_kb  det_msgs  dbg_msgs  last_lat_ms", flush=True)
start = time.time(); next_sample = INTERVAL; rss_series = []
while time.time() - start < DURATION:
    rclpy.spin_once(n, timeout_sec=0.1)
    el = time.time() - start
    if el >= next_sample:
        r = rss_kb(); rss_series.append(r)
        print(f"{el:9.0f}  {'yes' if r >= 0 else 'NO':>9}  {r:7d}  {n.d:8d}  {n.g:8d}  "
              f"{'n/a' if n.lat is None else round(n.lat,1)}", flush=True)
        next_sample += INTERVAL
print(f"FINAL det_msgs={n.d} dbg_msgs={n.g} duration={DURATION:.0f}s", flush=True)
if rss_series:
    print(f"RSS first={rss_series[0]} last={rss_series[-1]} "
          f"delta_kb={rss_series[-1]-rss_series[0]}", flush=True)
rclpy.shutdown()
PY

echo "=== after observation ===" | tee -a "$OUT"
pgrep -f "lib/yolo_ros/yolo_node" >/dev/null \
  && echo "yolo_node STILL ALIVE (no crash)" | tee -a "$OUT" \
  || echo "yolo_node DIED" | tee -a "$OUT"
echo "error_lines=$(grep -cE 'ERROR|died' /tmp/t08_det.log 2>/dev/null || echo 0)" | tee -a "$OUT"
grep -E "ERROR|died" /tmp/t08_det.log 2>/dev/null | head -5 | tee -a "$OUT"

pkill -f yolo_ros/yolo_node 2>/dev/null
pkill -f dmu_vision_tools/image_file_source 2>/dev/null
pkill -f detection.launch.py 2>/dev/null
echo "T08_DONE  log=$OUT" | tee -a "$OUT"
