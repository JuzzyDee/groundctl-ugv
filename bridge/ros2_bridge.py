#!/usr/bin/env python3
"""
ros2_bridge.py — HTTP bridge between heartbeat and ROS2 stack.

Replaces the patches we made to app.py. Runs inside the Docker container
alongside the ROS2 nodes. Subscribes to ROS2 topics, exposes them as
HTTP endpoints matching what heartbeat.py expects.

Endpoints:
- GET  /state          — telemetry (voltage, IMU, odometry, gimbal)
- POST /send_command   — drive/gimbal/audio commands (translates to /cmd_vel etc)
- GET  /depth_status   — depth safety daemon status (file-backed)
- GET  /snapshot       — latest camera frame as JPEG (from /image_raw subscription)
- GET/POST /inbox      — voice transcription queue

Live MJPEG streaming (/stream, /stream_annotated) was removed 2026-05-11 — the
operator console polls /snapshot at 1 Hz which is all anyone actually needs,
and live full-rate streaming was burning bridge GIL competing with the ROS
spin thread (the symptom set looked like camera failure but was really
self-inflicted backpressure). Long-form video for demos is bagged via MCAP
and processed offline.

Run inside the Docker container:
    source /opt/ros/humble/setup.bash
    pip install waitress   # request/response performance under concurrent load
    python3 ros2_bridge.py

  Set ROS_BRIDGE_WAITRESS=0 to force Flask's built-in server (debug only).
"""

import json
import math
import os
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, MagneticField, JointState, CompressedImage
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from vision_msgs.msg import Detection2DArray, Detection3DArray

from flask import Flask, request, jsonify, Response, send_from_directory, abort

# `_http` alias to avoid shadowing flask's `request` symbol used everywhere here.
import requests as _http

# Both safety status files written to host's /home/jetson, bind-mounted to /home/ws inside container
DEPTH_STATUS_FILE = Path("/home/ws/depth_safety_status.json")
LIDAR_STATUS_FILE = Path("/home/ws/lidar_safety_status.json")
CLIFF_STATUS_FILE = Path("/home/ws/cliff_safety_status.json")

# Operator console (ClaudeBot React app) static files. Deployed alongside
# this bridge into the container at /tmp/web/ClaudeBot/. Override with the
# CLAUDEBOT_WEB_DIR env var if running outside the container.
WEB_DIR = Path(os.environ.get("CLAUDEBOT_WEB_DIR", "/tmp/web/ClaudeBot"))

# Upstream services we proxy through the bridge so the React operator
# console only ever talks to bridge:5000 — single Tailscale path, no
# CORS configuration required per subsystem. Override via env vars if
# the layout changes (e.g., camera_owner moves to host).
CAMERA_OWNER_URL = os.environ.get("CAMERA_OWNER_URL", "http://localhost:5001")
CONTROL_PANEL_URL = os.environ.get("CONTROL_PANEL_URL", "http://localhost:5060")

# Camera horizontal FOV (degrees). Used to convert pixel-x offsets into bearing.
# Generic USB webcam is usually ~60-75; tune via CAMERA_HFOV_DEG env var if needed.
CAMERA_HFOV_DEG = 70.0
# We assume 640x480 from v4l2_camera defaults; bearing_deg falls back gracefully
# if the detection frame size differs.
DEFAULT_FRAME_WIDTH = 640
DEFAULT_FRAME_HEIGHT = 480

# In-memory inbox for voice transcriptions from the listener daemon.
# Heartbeat polls /inbox to drain it.
_inbox_lock = threading.Lock()
_inbox_messages = []

# Latest OAK-D spatial detections: person-filtered, each with metric 3D
# position + bearing + distance + a stable per-object ID. Live source is the
# /oak/nn/spatial_detections ROS topic (depthai_ros_driver), ingested by
# _spatial_cb. The legacy POST /spatial_detections path (old host-side
# oakd_spatial daemon) still writes here for back-compat, but is no longer fed
# once oakd_spatial is disabled. Read via GET /spatial_detections and /state.
_spatial_lock = threading.Lock()
_spatial_detections: list[dict] = []
_spatial_ts: float = 0.0

# mobilenet-SSD (depthai_ros_driver default model) labels, VOC order. The
# driver detects all 20 classes; _spatial_cb person-filters to preserve the
# CLA-49 person-only spatial target list (keeps follow/approach from locking
# onto Chopper or a horse).
_VOC_LABELS = [
    "background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus",
    "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike",
    "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor",
]
# Frame-to-frame nearest-neighbour radius for the bridge-side ID assigner.
_SPATIAL_TRACK_RADIUS_M = 1.5


def _voc_label(raw) -> str:
    """mobilenet-SSD class_id → label name. The depthai driver may emit the
    label string or its numeric index (as a string); handle both."""
    s = str(raw)
    if s.isdigit():
        i = int(s)
        return _VOC_LABELS[i] if 0 <= i < len(_VOC_LABELS) else s
    return s.lower()


app = Flask(__name__)


@app.route('/', methods=['GET'])
def claudebot_root_redirect():
    """Friendliness redirect: open http://rover:5000/ on a phone, land
    on the operator console without remembering the /web/ path."""
    return Response(status=302, headers={"Location": "/web/"})


@app.route('/web/', methods=['GET'])
def claudebot_index():
    """Serve the ClaudeBot operator console index. Phone-frame React app
    living at WEB_DIR. Open http://rover:5000/ from a phone or browser."""
    index = WEB_DIR / "ClaudeBot Operator Console.html"
    if not index.exists():
        return jsonify({
            "error": f"operator console not found at {index}",
            "hint": "deploy web/ClaudeBot/ to the rover (see start_ros2.sh)",
        }), 404
    return send_from_directory(str(WEB_DIR), index.name)


@app.route('/web/<path:filename>', methods=['GET'])
def claudebot_static(filename):
    """Static-serve any sibling file under WEB_DIR (jsx, css, uploads).
    Path traversal is bounded by send_from_directory — Flask refuses
    any path that escapes WEB_DIR."""
    if not WEB_DIR.exists():
        abort(404)
    return send_from_directory(str(WEB_DIR), filename)


def _proxy(target_base: str, path: str):
    """Forward a GET/POST to an upstream service, returning its body and
    content-type unchanged. Hard 3-second timeout so a hung upstream
    can't lock up the bridge — return 502 cleanly instead, with a clear
    error so the client knows it's the upstream not the bridge.

    This proxy is for JSON + JPEG request/response only."""
    url = f"{target_base.rstrip('/')}/{path}"
    try:
        if request.method == 'POST':
            r = _http.post(url, json=request.get_json(silent=True),
                           params=request.args, timeout=3.0)
        else:
            r = _http.get(url, params=request.args, timeout=3.0)
    except _http.RequestException as e:
        return jsonify({
            "error": "upstream unavailable",
            "upstream": url,
            "detail": str(e),
        }), 502
    return Response(
        r.content,
        status=r.status_code,
        content_type=r.headers.get('Content-Type', 'application/json'),
    )


@app.route('/camera/<path:p>', methods=['GET', 'POST'])
def proxy_camera(p):
    """Proxy to camera_owner (default localhost:5001).
    Useful endpoints: /camera/health, /camera/snapshot."""
    return _proxy(CAMERA_OWNER_URL, p)


@app.route('/control/<path:p>', methods=['GET', 'POST'])
def proxy_control(p):
    """Proxy to control_panel (default localhost:5060).
    Useful endpoints: /control/heartbeat, /control/heartbeat/start,
    /control/heartbeat/stop, /control/reflection_log.
    Returns 502 until control_panel.py is deployed and running."""
    return _proxy(CONTROL_PANEL_URL, p)


class BridgeNode(Node):
    """ROS2 node that holds the latest values from each topic and publishes commands."""

    def __init__(self):
        super().__init__('ros2_bridge')

        # Latest telemetry values
        self.voltage = 0.0
        self.imu_ax = 0.0
        self.imu_ay = 0.0
        self.imu_az = 0.0
        self.imu_gx = 0.0
        self.imu_gy = 0.0
        self.imu_gz = 0.0
        # Gyro-integrated yaw (deg): relative heading isolated from the corrupted
        # magnetometer. Fed to state["heading"]; integrated in _imu_cb.
        self.gyro_yaw_deg = 0.0
        self._last_imu_t = None
        self.mag_x = 0.0
        self.mag_y = 0.0
        self.mag_z = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.linear_v = 0.0
        self.angular_v = 0.0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.create_subscription(Float32, '/voltage', self._voltage_cb, 10)
        self.create_subscription(Imu, '/imu/data', self._imu_cb, sensor_qos)
        self.create_subscription(MagneticField, '/imu/mag', self._mag_cb, sensor_qos)
        self.create_subscription(Odometry, '/odom', self._odom_cb, sensor_qos)

        # Camera: subscribe to the throttled compressed topic (2 Hz).
        # camera_owner publishes /image_raw/compressed at ~27 Hz for YOLO
        # ByteTrack continuity; topic_tools/throttle (started by
        # start_ros2_local.sh) republishes at 2 Hz on /image_raw/compressed_low,
        # which is everything /snapshot actually needs (heartbeat polls every
        # 12 s; operator console at 1 Hz). 13× less subscriber callback
        # pressure on the bridge → more rclpy spin headroom against Waitress
        # GIL contention.
        #
        # JPEG bytes are cached as-is (no decode on ingest). Full-res BGR for
        # /snapshot zoom crops is produced by a lazy decode in
        # get_decoded_frame — only runs when Haiku actually asks for a
        # snapshot, not per frame.
        self._latest_jpeg: bytes | None = None
        self._latest_jpeg_count = 0
        self._decoded_frame: np.ndarray | None = None
        self._decoded_for_count: int = -1
        self._frame_lock = threading.Lock()
        self._frame_cond = threading.Condition(self._frame_lock)
        self.create_subscription(
            CompressedImage, '/image_raw/compressed_low', self._compressed_cb, sensor_qos
        )

        # Detections subscription — caches the latest detections list for /state
        # and for the gimbal servo loop.
        self._detections: list[dict] = []
        self._detections_lock = threading.Lock()
        self._detections_count = 0  # bumped on every new /detections message
        self._frame_width = DEFAULT_FRAME_WIDTH
        self._frame_height = DEFAULT_FRAME_HEIGHT
        self._frame_dims_known = False  # flipped true on first compressed decode
        self.create_subscription(Detection2DArray, '/detections', self._detections_cb, 10)

        # OAK-D spatial detections (depthai_ros_driver, native ROS). The driver
        # publishes these as vision_msgs/Detection3DArray. Person-filtered into
        # the metric-position schema follow/approach consume, with bridge-
        # assigned IDs — the SpatialDetectionNetwork has no on-device
        # ObjectTracker, so Detection3D.id arrives empty. Replaces the old
        # host-side oakd_spatial HTTP POST path.
        self._spatial_tracks: list[dict] = []  # previous frame: [{"id", "pos"}]
        self._spatial_next_id = 0
        self.create_subscription(
            Detection3DArray, '/oak/nn/spatial_detections', self._spatial_cb, sensor_qos
        )

        # Tracking state — set via POST /track, consumed by the gimbal servo.
        self._tracking_lock = threading.Lock()
        self._tracking_target_id: str | None = None
        self._tracking_last_seen: float = 0.0  # time of last detection matching target

        # Publisher for motor commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher for gimbal commands (via joint_states)
        self.joint_state_pub = self.create_publisher(JointState, '/ugv/joint_states', 10)

        # Track current gimbal position so /state can report it
        self.gimbal_pan = 0.0
        self.gimbal_tilt = 0.0

        self.get_logger().info("ros2_bridge ready")

    def _compressed_cb(self, msg: CompressedImage):
        """Cache the camera's native JPEG bytes as-is. Zero decode, zero
        memcopy beyond the sensor_msgs payload → bytes conversion."""
        try:
            # msg.data is array.array('B', ...); bytes() copies once so the
            # cached reference is independent of rclpy's buffer lifetime.
            blob = bytes(msg.data)
            with self._frame_cond:
                self._latest_jpeg = blob
                self._latest_jpeg_count += 1
                self._frame_cond.notify_all()
            # Snapshot-time decode needs to know the source frame dims for
            # crop clamping, and the detections callback uses them to convert
            # pixel offsets to bearings. We only pay the decode once, on the
            # first compressed message we see.
            if not self._frame_dims_known:
                nparr = np.frombuffer(blob, dtype=np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                if frame is not None:
                    h, w = frame.shape[:2]
                    self._frame_width = w
                    self._frame_height = h
                    self._frame_dims_known = True
        except Exception as e:
            self.get_logger().warn(f"compressed_cb: {e}")

    def get_latest_jpeg(self) -> tuple[bytes | None, int]:
        with self._frame_cond:
            return self._latest_jpeg, self._latest_jpeg_count

    def get_decoded_frame(self) -> np.ndarray | None:
        """Return the latest full-res BGR frame, decoding JPEG on demand.

        Caches the decoded result by frame count, so repeated calls against
        the same underlying JPEG share one decode. A new compressed message
        invalidates the cache (naturally, via _latest_jpeg_count).
        """
        with self._frame_cond:
            jpeg = self._latest_jpeg
            count = self._latest_jpeg_count
            if self._decoded_for_count == count and self._decoded_frame is not None:
                return self._decoded_frame
        if jpeg is None:
            return None
        nparr = np.frombuffer(jpeg, dtype=np.uint8)
        frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        if frame is None:
            return None
        with self._frame_cond:
            # Only publish the decoded cache if no newer frame has arrived;
            # otherwise let the next caller decode the fresher one.
            if self._latest_jpeg_count == count:
                self._decoded_frame = frame
                self._decoded_for_count = count
        return frame

    def snapshot_bgr_patch(self, x1: int, y1: int, x2: int, y2: int) -> np.ndarray | None:
        """Crop a BGR ROI from the latest frame (foveated /snapshot).

        The decode is shared via get_decoded_frame's count-keyed cache, so a
        burst of snapshot calls against one frame pays for one decode.
        """
        frame = self.get_decoded_frame()
        if frame is None:
            return None
        H, W = frame.shape[:2]
        x1 = max(0, min(W - 1, x1))
        x2 = max(x1 + 1, min(W, x2))
        y1 = max(0, min(H - 1, y1))
        y2 = max(y1 + 1, min(H, y2))
        return np.ascontiguousarray(frame[y1:y2, x1:x2])

    def _detections_cb(self, msg: Detection2DArray):
        """Flatten Detection2DArray into a list of dicts, add bearing_deg."""
        cx_center = self._frame_width / 2.0
        half_hfov = CAMERA_HFOV_DEG / 2.0

        parsed = []
        for det in msg.detections:
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            size_x = det.bbox.size_x
            size_y = det.bbox.size_y
            # Bearing: pixel offset from image-centre, mapped linearly across HFOV/2.
            # Positive = right of centre, negative = left of centre.
            bearing_deg = (cx - cx_center) / cx_center * half_hfov

            class_id = ""
            score = 0.0
            if det.results:
                class_id = det.results[0].hypothesis.class_id
                score = float(det.results[0].hypothesis.score)

            parsed.append({
                "id": det.id if det.id else None,
                "class_id": class_id,
                "score": round(score, 3),
                "bbox": {
                    "cx": round(cx, 1),
                    "cy": round(cy, 1),
                    "w": round(size_x, 1),
                    "h": round(size_y, 1),
                },
                "bearing_deg": round(bearing_deg, 1),
            })

        # Sort left-to-right by bbox center x so Claude has stable spatial ordering.
        parsed.sort(key=lambda d: d["bbox"]["cx"])
        for i, d in enumerate(parsed):
            d["index"] = i

        with self._detections_lock:
            self._detections = parsed
            self._detections_count += 1

        # Update tracking "last seen" timestamp if tracked target is in this frame.
        with self._tracking_lock:
            target_id = self._tracking_target_id
        if target_id is not None and any(d["id"] == target_id for d in parsed):
            with self._tracking_lock:
                self._tracking_last_seen = time.time()

    def get_detections(self) -> list[dict]:
        with self._detections_lock:
            return list(self._detections)

    def get_detections_with_count(self) -> tuple[list[dict], int]:
        with self._detections_lock:
            return list(self._detections), self._detections_count

    def _spatial_cb(self, msg: Detection3DArray):
        """Ingest OAK-D spatial detections (depthai_ros_driver publishes them
        as vision_msgs/Detection3DArray) into the person-only, metric-position
        dict schema follow/approach consume.

        Positions are in the camera optical frame (x-right, y-down, z-forward)
        — the same convention the old oakd_spatial used — so bearing/distance
        carry over 1:1. Person-filtered (CLA-49); IDs assigned bridge-side.
        Replaces the host-side oakd_spatial HTTP POST.
        """
        global _spatial_ts
        parsed = []
        for det in msg.detections:
            if not det.results:
                continue
            if _voc_label(det.results[0].hypothesis.class_id) != "person":
                continue
            # Metric centroid (metres, optical frame) is in the hypothesis
            # pose. bbox.center is image-plane pixels in this driver build.
            pos = det.results[0].pose.pose.position
            x = pos.x
            y = pos.y
            z = pos.z
            bearing = math.degrees(math.atan2(x, z)) if z else 0.0
            horizontal = (x * x + z * z) ** 0.5
            parsed.append({
                "id": None,  # assigned by _assign_spatial_ids below
                "class_id": "person",
                "status": "TRACKED",
                "position_m": {"x": round(x, 3), "y": round(y, 3), "z": round(z, 3)},
                "bearing_deg": round(bearing, 1),
                "distance_m": round(horizontal, 2),
            })
        self._assign_spatial_ids(parsed)
        with _spatial_lock:
            _spatial_detections[:] = parsed
            _spatial_ts = time.time()

    def _assign_spatial_ids(self, dets: list[dict]) -> None:
        """Greedy nearest-neighbour ID assignment, frame-to-frame, in place.

        The driver has no on-device ObjectTracker, so we re-create stable IDs
        here: match each detection to the nearest previous-frame track within a
        horizontal radius and inherit its ID, else mint a new one. Same idea as
        the old ZERO_TERM tracker — enough stability for follow/approach's
        semantic pick and per-tick re-lock (which themselves fall back to
        position-handover when an ID drops)."""
        prev = self._spatial_tracks
        claimed: set[str] = set()
        for d in dets:
            best_id, best_dist = None, _SPATIAL_TRACK_RADIUS_M
            for t in prev:
                if t["id"] in claimed:
                    continue
                dx = d["position_m"]["x"] - t["pos"]["x"]
                dz = d["position_m"]["z"] - t["pos"]["z"]
                dist = (dx * dx + dz * dz) ** 0.5
                if dist < best_dist:
                    best_id, best_dist = t["id"], dist
            if best_id is None:
                best_id = str(self._spatial_next_id)
                self._spatial_next_id += 1
            d["id"] = best_id
            claimed.add(best_id)
        self._spatial_tracks = [{"id": d["id"], "pos": d["position_m"]} for d in dets]

    def get_tracking_state(self) -> dict:
        with self._tracking_lock:
            target_id = self._tracking_target_id
            last_seen = self._tracking_last_seen

        now = time.time()
        staleness = now - last_seen if last_seen > 0 else None
        lock = target_id is not None and staleness is not None and staleness < 1.0

        return {
            "target_id": target_id,
            "locked": lock,
            "staleness_s": round(staleness, 2) if staleness is not None else None,
        }

    def set_tracking_target(self, target_id: str | None):
        with self._tracking_lock:
            self._tracking_target_id = target_id
            if target_id is None:
                self._tracking_last_seen = 0.0

    def _voltage_cb(self, msg):
        self.voltage = msg.data

    def _imu_cb(self, msg):
        self.imu_ax = msg.linear_acceleration.x
        self.imu_ay = msg.linear_acceleration.y
        self.imu_az = msg.linear_acceleration.z
        self.imu_gx = msg.angular_velocity.x
        self.imu_gy = msg.angular_velocity.y
        self.imu_gz = msg.angular_velocity.z

        # Integrate yaw rate into a relative heading, mag-free. The magnetometer
        # heading carries the task #58 sign-flip corruption that arced the rover
        # into U-turns (see drive_distance.py), so course-hold and relative-turn
        # ride this instead. Absolute bearing returns with the GNSS fix.
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._last_imu_t is not None:
            dt = t - self._last_imu_t
            if 0.0 < dt < 0.5:
                self.gyro_yaw_deg += math.degrees(self.imu_gz) * dt
        self._last_imu_t = t

    def _mag_cb(self, msg):
        self.mag_x = msg.magnetic_field.x
        self.mag_y = msg.magnetic_field.y
        self.mag_z = msg.magnetic_field.z

    def _odom_cb(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.linear_v = msg.twist.twist.linear.x
        self.angular_v = msg.twist.twist.angular.z

    def publish_twist(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)

    def publish_gimbal(self, pan_deg, tilt_deg):
        """Update target gimbal position. The background thread publishes it continuously."""
        self.gimbal_pan = pan_deg
        self.gimbal_tilt = tilt_deg

    def gimbal_publish_loop(self):
        """Background thread: continuously publish the current gimbal target to win the race
        against joint_state_publisher's zero spam."""
        while True:
            try:
                pan_rad = self.gimbal_pan * math.pi / 180.0
                tilt_rad = self.gimbal_tilt * math.pi / 180.0
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = ['pt_base_link_to_pt_link1', 'pt_link1_to_pt_link2']
                msg.position = [pan_rad, tilt_rad]
                self.joint_state_pub.publish(msg)
            except Exception:
                pass
            time.sleep(0.02)  # 50Hz

    def gimbal_servo_loop(self):
        """Background thread: P-controller that nudges gimbal_pan/gimbal_tilt
        toward the tracked target's bbox centre.

        Rate-matched to detection arrivals — only steps once per new /detections
        message, which naturally pairs with the 10Hz detection rate and prevents
        double-application of corrections against stale data (previous bug:
        20Hz servo on 10Hz detections effectively doubled Kp, causing overshoot
        and ByteTrack losing tracks from implausible frame-to-frame jumps).

        Design:
        - Only runs when tracking.target_id is set AND the target's detection
          is present in the latest /detections (lock is healthy).
        - Pixel error → degrees via Kp. One step per detection arrival.
        - Dead zone near image centre prevents jitter when already centred.
        - Pan/tilt clamped to conservative limits.
        """
        DEAD_ZONE_PX = 20
        KP_PAN = 0.025
        KP_TILT = 0.020
        PAN_MIN, PAN_MAX = -160.0, 160.0
        TILT_MIN, TILT_MAX = -45.0, 25.0
        LOCK_TIMEOUT_S = 0.6

        last_applied_count = 0

        while True:
            try:
                with self._tracking_lock:
                    target_id = self._tracking_target_id
                    last_seen = self._tracking_last_seen

                if target_id is None:
                    time.sleep(0.05)
                    continue

                staleness = time.time() - last_seen if last_seen > 0 else float('inf')
                if staleness > LOCK_TIMEOUT_S:
                    time.sleep(0.05)
                    continue

                # Only step on a new detection message — poll frequently but do work
                # at detection rate.
                detections, count = self.get_detections_with_count()
                if count == last_applied_count:
                    time.sleep(0.01)
                    continue
                last_applied_count = count

                target = next((d for d in detections if d["id"] == target_id), None)
                if target is None:
                    continue

                cx = target["bbox"]["cx"]
                cy = target["bbox"]["cy"]
                err_x = cx - (self._frame_width / 2.0)
                err_y = cy - (self._frame_height / 2.0)

                if abs(err_x) > DEAD_ZONE_PX:
                    new_pan = self.gimbal_pan + KP_PAN * err_x
                    self.gimbal_pan = max(PAN_MIN, min(PAN_MAX, new_pan))

                if abs(err_y) > DEAD_ZONE_PX:
                    new_tilt = self.gimbal_tilt - KP_TILT * err_y
                    self.gimbal_tilt = max(TILT_MIN, min(TILT_MAX, new_tilt))

            except Exception as e:
                self.get_logger().warn(f"gimbal_servo_loop: {e}")


bridge = None


@app.route('/state', methods=['GET'])
def get_state():
    """Combined state for heartbeat and intents.

    Backwards-compatible 'base' subdict preserves original ESP32 format.
    New 'position', 'velocity', 'heading', 'gimbal' subdicts are clean ROS2-derived values."""

    # Heading: gyro-integrated yaw (relative, mag-free). The raw-magnetometer
    # atan2 is kept only as a diagnostic (heading_mag) — it carries the task #58
    # sign-flip corruption, so it no longer drives the course-hold loop.
    heading = bridge.gyro_yaw_deg % 360
    heading_mag = (math.degrees(math.atan2(bridge.mag_y, bridge.mag_x)) + 360) % 360

    return jsonify({
        "base": {
            "v": int(bridge.voltage * 100),
            "L": bridge.linear_v,
            "R": bridge.linear_v,
            "ax": int(bridge.imu_ax * 1000),
            "ay": int(bridge.imu_ay * 1000),
            "az": int(bridge.imu_az * 1000),
            "gx": int(bridge.imu_gx * 1000),
            "gy": int(bridge.imu_gy * 1000),
            "gz": int(bridge.imu_gz * 1000),
            "mx": int(bridge.mag_x * 1000),
            "my": int(bridge.mag_y * 1000),
            "mz": int(bridge.mag_z * 1000),
            "odl": int(bridge.odom_x * 1000),
            "odr": int(bridge.odom_y * 1000),
        },
        "pan_angle": bridge.gimbal_pan,
        "tilt_angle": bridge.gimbal_tilt,
        "voltage": bridge.voltage,
        "position": {"x": bridge.odom_x, "y": bridge.odom_y},
        "velocity": {"linear": bridge.linear_v, "angular": bridge.angular_v},
        "heading": heading,
        "heading_mag": heading_mag,
        "gimbal": {"pan": bridge.gimbal_pan, "tilt": bridge.gimbal_tilt},
        "detections": bridge.get_detections(),
        "tracking": bridge.get_tracking_state(),
        # OAK-D spatial detections (metric 3D positions, stable track IDs).
        # Falls back to empty + stale age when oakd_spatial isn't running.
        "spatial_detections": _get_spatial_detections_snapshot(),
    })


def _get_spatial_detections_snapshot() -> dict:
    with _spatial_lock:
        detections = list(_spatial_detections)
        ts = _spatial_ts
    age_s = round(time.time() - ts, 2) if ts else None
    return {"detections": detections, "age_s": age_s}


@app.route('/track', methods=['POST'])
def track():
    """Set or clear the follow_look tracking target.

    Body:
        {"target_id": "30"}   -- start tracking the detection with tracking ID "30"
        {"target_id": null}   -- stop tracking

    target_id is the ByteTrack tracking ID (string) from /detections[].id,
    not the array index. Heartbeat's follow_look(target_index) tool does the
    index->id resolution before calling this endpoint.
    """
    data = request.json or {}
    if "target_id" not in data:
        return jsonify({"error": "target_id required (string or null)"}), 400

    target_id = data["target_id"]
    if target_id is not None and not isinstance(target_id, str):
        # Accept int for convenience and coerce to string.
        target_id = str(target_id)

    bridge.set_tracking_target(target_id)
    return jsonify({"status": "ok", "target_id": target_id})


@app.route('/send_command', methods=['POST'])
def send_command():
    """Translate the old base -c {T:1,L:x,R:y} format to /cmd_vel.

    Differential drive: L and R wheel speeds in [-1,1].
    Linear = (L+R)/2, Angular = (R-L)/wheel_separation
    """
    cmd_str = request.form.get('command', '')

    if not cmd_str.startswith('base -c'):
        return jsonify({"status": "ignored", "command": cmd_str})

    try:
        json_str = cmd_str.split('base -c', 1)[1].strip()
        cmd = json.loads(json_str)
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 400

    t = cmd.get('T')

    if t == 1:
        # Drive command: {"T":1, "L":speed, "R":speed}
        # L, R are normalised wheel commands in [-1, +1] from the intent
        # layer. We pass linear through 1:1 (pwm_driver's MAX_PWM cap is
        # the single authoritative speed limit — no need to double-cap
        # here). Angular keeps its differential scaling so existing intent
        # gain calibrations (e.g. follow.py KP_BEARING) stay valid.
        left = float(cmd.get('L', 0))
        right = float(cmd.get('R', 0))
        linear = (left + right) / 2.0
        angular = (right - left) / 0.2
        bridge.publish_twist(linear, angular)
        return jsonify({"status": "ok", "linear": linear, "angular": angular})

    elif t == 0:
        # Emergency stop
        bridge.publish_twist(0.0, 0.0)
        return jsonify({"status": "ok", "stop": True})

    elif t == 133:
        # Gimbal: {"T":133,"X":pan,"Y":tilt,"SPD":..,"ACC":..}
        pan = float(cmd.get('X', 0))
        tilt = float(cmd.get('Y', 0))
        bridge.publish_gimbal(pan, tilt)
        return jsonify({"status": "ok", "pan": pan, "tilt": tilt})

    return jsonify({"status": "ignored", "T": t})


@app.route('/snapshot', methods=['GET'])
def snapshot():
    """Return the latest camera frame as JPEG.

    Default (no params): full camera frame downscaled to a 640-wide output at
    native aspect ratio. Claude gets maximum peripheral awareness — all the
    pixels the rover's sensor actually sees. Fisheye distortion is visible;
    we trust Claude to learn the lens (episodic memory reinforces this).

    Query params (all optional, for foveal zoom):
        cx, cy    — centre of crop region in full-res pixels (default: image centre)
        zoom      — zoom factor; 1.0 = full frame, 2.0 = half FOV, 4.0 = quarter FOV
        out_w     — output width (default 640)
        out_h     — output height (auto-computed from source aspect if not given)

    Use case: /snapshot?cx=700&cy=400&zoom=2.5 returns a ~384x216 px tile from
    the full-res source, scaled to 640x(aspect-preserved) — same token cost
    as default, but 2.5× the detail in the region of interest.

    Zoomed requests decode the cached JPEG once per frame (shared across
    concurrent /snapshot callers) and copy only the crop — not the full
    1080p BGR array.
    """
    frame = bridge.get_decoded_frame()
    if frame is None:
        return jsonify({"error": "no frame yet — is gst_camera_node running?"}), 503
    H, W = frame.shape[:2]

    source_aspect = W / H

    out_w = int(request.args.get('out_w', 640))
    # Auto-compute out_h to preserve source aspect if not explicitly given.
    if 'out_h' in request.args:
        out_h = int(request.args.get('out_h'))
    else:
        out_h = int(out_w / source_aspect)

    zoom = float(request.args.get('zoom', 1.0))
    zoom = max(1.0, min(zoom, 8.0))
    cx = int(request.args.get('cx', W // 2))
    cy = int(request.args.get('cy', H // 2))

    # Crop spans source_w/zoom × source_h/zoom, in output's aspect ratio.
    # At zoom=1, this is the full frame (letterbox to output aspect only if
    # user explicitly requested a different out_h).
    target_aspect = out_w / out_h
    if source_aspect > target_aspect:
        crop_h = int(H / zoom)
        crop_w = int(crop_h * target_aspect)
    else:
        crop_w = int(W / zoom)
        crop_h = int(crop_w / target_aspect)

    cx = max(crop_w // 2, min(W - crop_w // 2, cx))
    cy = max(crop_h // 2, min(H - crop_h // 2, cy))
    x1 = cx - crop_w // 2
    y1 = cy - crop_h // 2
    x2 = x1 + crop_w
    y2 = y1 + crop_h

    cropped = bridge.snapshot_bgr_patch(x1, y1, x2, y2)
    if cropped is None or cropped.size == 0:
        return jsonify({"error": "no frame yet — is v4l2_camera_node running?"}), 503

    resized = cv2.resize(cropped, (out_w, out_h), interpolation=cv2.INTER_AREA)

    ok, jpeg = cv2.imencode('.jpg', resized, [cv2.IMWRITE_JPEG_QUALITY, 85])
    if not ok:
        return jsonify({"error": "jpeg encode failed"}), 500
    return Response(jpeg.tobytes(), mimetype='image/jpeg')


@app.route('/spatial_detections', methods=['GET', 'POST'])
def spatial_detections():
    """OAK-D spatial detections I/O.

    POST (from oakd_spatial daemon): ingest latest tracklets with metric
        3D positions. Body: {"detections": [...], "ts": float}
    GET (from heartbeat / dev tools): read cached detections with staleness.

    Used by the upcoming `follow` intent — metric distance + stable
    tracking IDs + bearing from camera forward axis make for cleaner
    proportional control than bbox-area heuristics on the YOLO topic.
    """
    global _spatial_ts
    if request.method == 'POST':
        data = request.json or {}
        detections = data.get("detections", []) or []
        with _spatial_lock:
            _spatial_detections[:] = detections
            _spatial_ts = float(data.get("ts") or time.time())
        return jsonify({"status": "ok", "n": len(detections)})

    # GET
    with _spatial_lock:
        detections = list(_spatial_detections)
        ts = _spatial_ts
    age_s = round(time.time() - ts, 2) if ts else None
    return jsonify({"detections": detections, "ts": ts, "age_s": age_s})


@app.route('/inbox', methods=['GET', 'POST'])
def inbox():
    """Voice messages from the listener daemon flow through here.
    POST: listener adds a transcription
    GET: heartbeat drains pending messages
    """
    if request.method == 'POST':
        data = request.json or {}
        text = data.get("text", "").strip()
        source = data.get("source", "voice")
        if not text:
            return jsonify({"error": "text required"}), 400
        with _inbox_lock:
            _inbox_messages.append({"text": text, "source": source, "ts": time.time()})
        return jsonify({"status": "queued", "queued": len(_inbox_messages)})

    # GET — drain
    with _inbox_lock:
        msgs = list(_inbox_messages)
        _inbox_messages.clear()
    return jsonify({"messages": msgs})


@app.route('/depth_status', methods=['GET'])
def depth_status():
    """Read depth safety status from the depth_safety daemon (file-backed)."""
    try:
        with open(DEPTH_STATUS_FILE) as f:
            return jsonify(json.load(f))
    except FileNotFoundError:
        return jsonify({"status": "offline"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)})


@app.route('/lidar_status', methods=['GET'])
def lidar_status():
    """Read LiDAR safety status from the lidar_safety node (file-backed)."""
    try:
        with open(LIDAR_STATUS_FILE) as f:
            return jsonify(json.load(f))
    except FileNotFoundError:
        return jsonify({"status": "offline"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)})


@app.route('/cliff_status', methods=['GET'])
def cliff_status():
    """Read cliff safety status from the cliff_safety node (file-backed)."""
    try:
        with open(CLIFF_STATUS_FILE) as f:
            return jsonify(json.load(f))
    except FileNotFoundError:
        return jsonify({"status": "offline"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)})


def ros_spin():
    # MultiThreadedExecutor lets image_cb run in parallel with other callbacks
    # (scan, TF, joint_states, odom). JPEG encode was moved to a dedicated thread
    # in BridgeNode — image_cb only converts + enqueues — but extra executor
    # threads still help when YOLO / other nodes compete for the GIL.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(bridge)
    try:
        executor.spin()
    finally:
        executor.shutdown()


def main():
    global bridge
    rclpy.init()
    bridge = BridgeNode()

    spin_thread = threading.Thread(target=ros_spin, daemon=True)
    spin_thread.start()

    gimbal_thread = threading.Thread(target=bridge.gimbal_publish_loop, daemon=True)
    gimbal_thread.start()

    servo_thread = threading.Thread(target=bridge.gimbal_servo_loop, daemon=True)
    servo_thread.start()

    # Redirected logs (e.g. > /tmp/bridge.log) fully buffer stdout; ROS logger
    # flushes each line — use logger for "which HTTP server" so it is visible
    # in `head` / `tail` immediately.
    log = bridge.get_logger()
    print("=" * 50, flush=True)
    print("  ROS2 BRIDGE", flush=True)
    print("  Listening on :5000", flush=True)
    print("=" * 50, flush=True)

    # Waitress handles concurrent request/response load (8 threads default)
    # better than Flask's dev server. /stream and friends are gone now, so
    # there's no streaming-specific reason for it — just steady-state perf
    # under multiple operator-console + heartbeat clients.
    use_waitress = os.environ.get("ROS_BRIDGE_WAITRESS", "1").lower() not in (
        "0", "false", "no",
    )
    if use_waitress:
        try:
            from waitress import serve

            log.info(
                "HTTP: Waitress (set ROS_BRIDGE_WAITRESS=0 to use Flask dev server)"
            )
            try:
                sys.stdout.reconfigure(line_buffering=True)
                sys.stderr.reconfigure(line_buffering=True)
            except Exception:
                pass
            serve(
                app,
                host="0.0.0.0",
                port=5000,
                threads=8,
                channel_timeout=3600,
            )
            return
        except ImportError:
            log.warn(
                "HTTP: waitress not installed — using Flask dev server "
                "(install waitress for production-grade concurrency)"
            )
    else:
        log.info("HTTP: Flask dev server (ROS_BRIDGE_WAITRESS=0)")
    app.run(host="0.0.0.0", port=5000, threaded=True)


if __name__ == '__main__':
    main()
