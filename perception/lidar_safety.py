#!/usr/bin/env python3
"""
lidar_safety.py — Spinal-reflex obstacle avoidance via LiDAR.

Subscribes to /scan (sensor_msgs/LaserScan) from the LD19, watches the
forward arc, and publishes a Bool danger signal that pwm_driver consults
before applying forward motion. Catches what the depth camera misses —
ferns, glass, chain-link fences, anything wispy that stereo can't lock
onto.

Hard-plumb design: pwm_driver subscribes to /lidar_safety/danger and
clamps target_linear > 0 to 0 when danger is true. Rotation and reverse
are still permitted, so the rover can self-recover by backing up or
turning to a clear heading. This replaces the older racing-on-cmd_vel
pattern where lidar_safety published Twist(0,0,0) and hoped twist_mux
gave it priority over intent publishers.

Runs inside the ROS2 docker container alongside the bridge:
    source /opt/ros/humble/setup.bash
    source /home/ws/ugv_ws/install/setup.bash
    python3 lidar_safety.py

Status is also written to /home/jetson/lidar_safety_status.json so the
heartbeat (via bridge) can surface it as Haiku context.

Forward arc convention: LaserScan starts at angle_min and increments by
angle_increment. For the LD19, angle_min is 0 and full 360° coverage means
"forward" is at 0° (or near it). We watch ±FORWARD_ARC_DEG/2 around 0.
"""

import json
import math
import os
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

sys.stdout.reconfigure(line_buffering=True)

DANGER_DISTANCE_M = 0.5    # block forward motion if anything closer than this in forward arc
CAUTION_DISTANCE_M = 1.0   # log/surface at this distance, no motion change
FORWARD_ARC_DEG = 60       # ±30° from straight ahead
MIN_VALID_RANGE = 0.05     # ignore reads below this (sensor noise / self-detection)
MIN_DETECTION_POINTS = 3   # require N close points to trigger (single-point noise filter)

# LD19 is mounted with 90° CCW yaw relative to chassis forward — verified
# 2026-04-21 via close-hand test (hand directly right of rover registers in
# code's forward cone, hand in front does not). Sensor's angle 0 points to
# rover's left. Add this offset when converting scan angles to chassis frame.
# If the sensor is ever remounted straight, set to 0.0.
SENSOR_YAW_OFFSET_RAD = math.pi / 2

# Inside the container, /home/ws is the bind mount of host's /home/jetson.
# Writing here means the bridge can read it via the same path.
STATUS_FILE = Path("/home/ws/lidar_safety_status.json")
PIDFILE = Path("/tmp/lidar_safety.pid")


class LidarSafety(Node):
    def __init__(self):
        super().__init__('lidar_safety')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Safety signal QoS: RELIABLE + TRANSIENT_LOCAL so a late-joining
        # subscriber (pwm_driver after a restart) gets the last published
        # value immediately on connect, rather than driving blind until
        # the next scan.
        danger_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self._scan_cb, sensor_qos)
        self.danger_pub = self.create_publisher(Bool, '/lidar_safety/danger', danger_qos)

        self.last_status = "init"
        self.scan_count = 0

        self.get_logger().info(
            f"lidar_safety ready — danger<{DANGER_DISTANCE_M}m, caution<{CAUTION_DISTANCE_M}m, "
            f"arc=±{FORWARD_ARC_DEG/2:.0f}° (hard plumb via /lidar_safety/danger)"
        )

    def _scan_cb(self, msg: LaserScan):
        self.scan_count += 1
        ranges = msg.ranges
        n = len(ranges)
        if n == 0:
            return

        # Compute index range for forward arc.
        # LD19 publishes 0 to 2π with angle_increment. 0 rad = forward (typical).
        half_arc_rad = math.radians(FORWARD_ARC_DEG / 2)
        # Forward indices: those where angle is in [-half_arc, +half_arc]
        # angle = angle_min + i * angle_increment, wrap to [-pi, pi]
        forward_distances = []
        for i, r in enumerate(ranges):
            if r < MIN_VALID_RANGE or r > msg.range_max or math.isinf(r) or math.isnan(r):
                continue
            # Rotate sensor frame to chassis frame by adding the known yaw offset.
            angle = msg.angle_min + i * msg.angle_increment + SENSOR_YAW_OFFSET_RAD
            # Normalise to [-pi, pi]
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi
            if -half_arc_rad <= angle <= half_arc_rad:
                forward_distances.append(r)

        if not forward_distances:
            self._write_status("no_data", 999, 0, 0)
            return

        forward_distances.sort()
        min_dist = forward_distances[0]
        danger_count = sum(1 for d in forward_distances if d < DANGER_DISTANCE_M)
        caution_count = sum(1 for d in forward_distances if d < CAUTION_DISTANCE_M)

        if danger_count >= MIN_DETECTION_POINTS:
            status = "danger"
        elif caution_count >= MIN_DETECTION_POINTS:
            status = "caution"
        else:
            status = "clear"

        # Always publish the danger Bool, every scan — pwm_driver consumes
        # this continuously and applies a staleness timeout if updates stop.
        danger_msg = Bool()
        danger_msg.data = (status == "danger")
        self.danger_pub.publish(danger_msg)

        # Log on transitions only (avoid 10Hz spam).
        if status != self.last_status:
            if status == "danger":
                self.get_logger().warn(
                    f"DANGER — {danger_count} points within {DANGER_DISTANCE_M}m (closest {min_dist:.2f}m)"
                )
            elif status == "caution":
                self.get_logger().info(f"CAUTION — closest {min_dist:.2f}m")
            else:
                self.get_logger().info(f"CLEAR — closest {min_dist:.2f}m")

        self.last_status = status
        self._write_status(status, min_dist, danger_count, caution_count)

    def _write_status(self, status, min_dist, danger_count, caution_count):
        try:
            STATUS_FILE.write_text(json.dumps({
                "status": status,
                "min_distance_m": round(min_dist, 3),
                "danger_points": danger_count,
                "caution_points": caution_count,
                "timestamp": time.time(),
            }))
        except Exception:
            pass


def _cleanup_pidfile():
    """Remove our PID file if it still belongs to us. Called on exit so a
    dead lidar_safety doesn't leave a stale lockfile blocking the next
    restart. Real incident on 2026-05-10: daemon died silently, lockfile
    pointed at PID 402, restarts kept reading the stale file. The
    cmdline-recovery in check_singleton DOES handle this, but cleaner
    to not rely on it."""
    try:
        if PIDFILE.exists() and PIDFILE.read_text().strip() == str(os.getpid()):
            PIDFILE.unlink()
    except Exception:
        pass


def check_singleton():
    import atexit
    if PIDFILE.exists():
        try:
            old_pid = int(PIDFILE.read_text().strip())
            os.kill(old_pid, 0)
            # Verify it's actually lidar_safety — PIDs get recycled by other processes,
            # which is how we got stuck in a stale-pidfile loop on 2026-04-21.
            cmdline = Path(f"/proc/{old_pid}/cmdline").read_text()
            if "lidar_safety" in cmdline:
                print(f"Another lidar_safety is running (PID {old_pid}). Exiting.")
                sys.exit(1)
        except (OSError, ValueError, FileNotFoundError):
            pass
    PIDFILE.write_text(str(os.getpid()))
    atexit.register(_cleanup_pidfile)


def main():
    check_singleton()
    rclpy.init()
    node = LidarSafety()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        STATUS_FILE.unlink(missing_ok=True)
        _cleanup_pidfile()  # belt-and-suspenders — atexit also runs this
        PIDFILE.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
