#!/usr/bin/env python3
"""intent_executor.py — ROS2 node that runs the intent stack at 10Hz on the rover.

Owns the DualStack instance previously held by heartbeat.py. Exposes a
small Flask HTTP server (port 5050) so the heartbeat (or any other
client) can push/pop/clear intents and read status. Ticks the stack
locally at 10Hz, publishing cmd_vel directly to ROS — no HTTP loopback
in the control loop.

This is the architecture the intent stack was designed for from the
start (LOST_TIMEOUT_BEATS = "20 beats ~ 2s at 10 Hz" in follow.py
confirms), but heartbeat-driven ticking at 1Hz never met that contract.
The result was a start/stop motor pattern at the rover: each tick sent
one cmd_vel, then pwm_driver's 500ms deadman timed out before the next
tick arrived.

Architecture choice:
  Separate process (not folded into the bridge) so the 10Hz control
  loop has its own GIL and OS scheduling. Bridge can churn on image
  encoding / multipart streaming without blocking intent ticks. Bridge
  crashing doesn't take down the control loop, and vice versa. Direct
  emergency control path (heartbeat → executor) survives bridge issues.

Scope (first cut):
  - cmd_vel commands (T=1 drive, T=0 stop) publish directly to ROS
  - All other commands (T=133 gimbal, audio, etc.) forward to the bridge
    via HTTP — preserves existing continuous-publish behaviour for
    gimbal joint_states without racing the bridge's publisher.

Runs inside the ros2 container alongside the bridge:
    source /opt/ros/humble/setup.bash
    source /home/ws/ugv_ws/install/setup.bash
    python3 intent_executor.py
"""

import json
import os
import sys
import threading
import time
from pathlib import Path

import requests
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist

from flask import Flask, request, jsonify
from waitress import serve

from intent.intent_stack import DualStack, list_intents, list_intents_by_category
from intent import intents as _intents  # auto-registers all intents

sys.stdout.reconfigure(line_buffering=True)

# --- Config ---
TICK_HZ = 10
HTTP_PORT = 5050
BRIDGE_URL = "http://127.0.0.1:5000"   # bridge in same container
BRIDGE_REQUEST_TIMEOUT_S = 2.0

# State refresh: the bridge already composes the unified state dict that
# intents read via ctx.get_state(). Mirroring that on the executor side by
# fetching /state every tick keeps intent code unchanged. localhost call,
# negligible latency.
STATE_FETCH_TIMEOUT_S = 0.5

# Status files written by the safety daemons. Same paths bridge reads from.
LIDAR_STATUS_FILE = Path("/home/ws/lidar_safety_status.json")
CLIFF_STATUS_FILE = Path("/home/ws/cliff_safety_status.json")

PIDFILE = Path("/tmp/intent_executor.pid")


class IntentExecutor(Node):
    def __init__(self):
        super().__init__("intent_executor")

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # DualStack with our local callbacks. send_command goes through
        # _send_command_local which routes drive/stop direct to ROS and
        # forwards everything else to the bridge.
        self.stack = DualStack(
            send_command=self._send_command_local,
            get_state=self._get_state_local,
        )

        self.beat = 0
        self.tick_period_s = 1.0 / TICK_HZ
        self.create_timer(self.tick_period_s, self._tick)

        # Stack mutation must not race the timer's tick. Push/pop happens
        # from Flask threads, tick happens on the ROS executor thread.
        self.stack_lock = threading.Lock()

        intents_by_cat = list_intents_by_category()
        self.get_logger().info(
            f"intent_executor ready @ {TICK_HZ} Hz, HTTP on :{HTTP_PORT}, "
            f"intents nav={intents_by_cat['nav']} attention={intents_by_cat['attention']}"
        )

    # ------------------------------------------------------------------
    # Callbacks passed to DualStack — execute commands, fetch state.
    # ------------------------------------------------------------------
    def _send_command_local(self, cmd_str: str) -> None:
        """Route command to ROS or forward to bridge based on type.

        Drive (T=1) and stop (T=0) publish Twist directly — that's the
        critical path the executor exists to keep at 10Hz. Anything else
        (gimbal T=133, audio, etc.) forwards to the bridge via HTTP so we
        don't have to replicate every command type.
        """
        if not cmd_str.startswith("base -c"):
            # Pass non-drive commands through to the bridge.
            self._forward_to_bridge(cmd_str)
            return

        try:
            json_str = cmd_str.split("base -c", 1)[1].strip()
            cmd = json.loads(json_str)
        except Exception as e:
            self.get_logger().warn(f"failed to parse command: {cmd_str!r} ({e})")
            return

        t = cmd.get("T")

        if t == 1:
            # Drive command: {"T":1, "L":speed, "R":speed}
            # Match bridge translation exactly so the existing intent gain
            # tunings remain valid (KP_BEARING etc. are calibrated against
            # the bridge's L/R → linear/angular conversion).
            left = float(cmd.get("L", 0))
            right = float(cmd.get("R", 0))
            linear = (left + right) / 2.0
            angular = (right - left) / 0.2
            self._publish_twist(linear, angular)
            return

        if t == 0:
            self._publish_twist(0.0, 0.0)
            return

        # Other T values (gimbal etc.) go via the bridge for now.
        self._forward_to_bridge(cmd_str)

    def _publish_twist(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)

    def _forward_to_bridge(self, cmd_str: str) -> None:
        """Forward a non-drive command to the bridge's /send_command."""
        try:
            requests.post(
                f"{BRIDGE_URL}/send_command",
                data={"command": cmd_str},
                timeout=BRIDGE_REQUEST_TIMEOUT_S,
            )
        except Exception as e:
            self.get_logger().warn(f"bridge forward failed: {e}")

    def _get_state_local(self) -> dict | None:
        """Compose state dict for intents.

        First cut: pull /state from the bridge over localhost. Bridge
        already composes the unified shape (telemetry + spatial_detections
        + lidar_status + etc.) and intent code reads it via the existing
        keys. Mirrors what heartbeat.py's get_state_fn does. Localhost
        call, sub-millisecond latency under normal load.
        """
        try:
            r = requests.get(f"{BRIDGE_URL}/state", timeout=STATE_FETCH_TIMEOUT_S)
            if r.status_code != 200:
                return None
            state = r.json()
        except Exception:
            return None
        # Merge in the safety-daemon JSON files directly. Bridge does this
        # too via /lidar_status etc., but reading the file is faster than a
        # second HTTP hop, and means executor still has lidar context if
        # bridge is choked.
        for key, path in (
            ("lidar_status", LIDAR_STATUS_FILE),
            ("cliff_status", CLIFF_STATUS_FILE),
        ):
            if path.exists():
                try:
                    state[key] = json.loads(path.read_text())
                except Exception:
                    pass
        return state

    # ------------------------------------------------------------------
    # Tick — runs on ROS executor thread at TICK_HZ.
    # ------------------------------------------------------------------
    def _tick(self) -> None:
        with self.stack_lock:
            if self.stack.is_empty:
                return
            self.beat += 1
            try:
                self.stack.tick(self.beat)
            except Exception as e:
                # Don't let an intent bug crash the executor — log and
                # continue. Worst case: the offending intent is on top of
                # the stack and crashes every tick until popped.
                self.get_logger().error(f"tick error: {e}")


# ----------------------------------------------------------------------
# Flask HTTP API — push/pop/clear/status.
# ----------------------------------------------------------------------

app = Flask(__name__)
_executor: IntentExecutor | None = None


@app.route("/intent/push", methods=["POST"])
def http_push():
    body = request.get_json(silent=True) or {}
    intent_name = body.get("intent")
    params = body.get("params", {}) or {}
    if not intent_name:
        return jsonify({"status": "error", "message": "intent name required"}), 400
    with _executor.stack_lock:
        result = _executor.stack.push(intent_name, params)
    return jsonify({"status": "ok", "result": result})


@app.route("/intent/pop", methods=["POST"])
def http_pop():
    body = request.get_json(silent=True) or {}
    target_stack = body.get("stack", "nav")
    with _executor.stack_lock:
        result = _executor.stack.pop(target_stack)
    return jsonify({"status": "ok", "result": result})


@app.route("/intent/clear", methods=["POST"])
def http_clear():
    body = request.get_json(silent=True) or {}
    target_stack = body.get("stack", "all")
    with _executor.stack_lock:
        _executor.stack.clear(target_stack)
    return jsonify({"status": "ok"})


@app.route("/intent/status", methods=["GET"])
def http_status():
    with _executor.stack_lock:
        status = _executor.stack.status()
        empty = _executor.stack.is_empty
        depth = _executor.stack.depth
    return jsonify({
        "status": "ok",
        "stack_status": status,
        "is_empty": empty,
        "depth": depth,
        "beat": _executor.beat,
    })


@app.route("/intent/events", methods=["GET"])
def http_events():
    """Return per-stack just_completed flags. Use ?consume=true (default)
    to clear them after read — this mirrors the heartbeat's check_events()
    semantics. ?consume=false leaves them for the next reader (used for
    non-destructive peeks during in-flight inference)."""
    consume = request.args.get("consume", "true").lower() == "true"
    with _executor.stack_lock:
        nav_done = _executor.stack.nav.just_completed
        att_done = _executor.stack.attention.just_completed
        if consume:
            _executor.stack.nav.just_completed = False
            _executor.stack.attention.just_completed = False
    return jsonify({
        "status": "ok",
        "nav_just_completed": nav_done,
        "attention_just_completed": att_done,
    })


@app.route("/intent/list", methods=["GET"])
def http_list():
    """List all registered intents — useful for the heartbeat's tool
    definitions and for debugging."""
    return jsonify({
        "status": "ok",
        "intents": list_intents(),
        "by_category": list_intents_by_category(),
    })


# ----------------------------------------------------------------------
# Entrypoint
# ----------------------------------------------------------------------

def check_singleton():
    if PIDFILE.exists():
        try:
            old_pid = int(PIDFILE.read_text().strip())
            os.kill(old_pid, 0)
            cmdline = Path(f"/proc/{old_pid}/cmdline").read_text()
            if "intent_executor" in cmdline:
                print(f"another intent_executor running (PID {old_pid}); exiting")
                sys.exit(1)
        except (OSError, ValueError, FileNotFoundError):
            pass
    PIDFILE.write_text(str(os.getpid()))


def main():
    global _executor
    check_singleton()
    rclpy.init()
    _executor = IntentExecutor()

    # Flask in a daemon thread so ROS spin owns the main thread.
    http_thread = threading.Thread(
        target=lambda: serve(app, host="0.0.0.0", port=HTTP_PORT, threads=4),
        daemon=True,
    )
    http_thread.start()
    print(f"intent_executor: HTTP serving on 0.0.0.0:{HTTP_PORT}")

    try:
        rclpy.spin(_executor)
    except KeyboardInterrupt:
        _executor.get_logger().info("shutting down")
    finally:
        _executor.destroy_node()
        rclpy.shutdown()
        PIDFILE.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
