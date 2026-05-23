#!/usr/bin/env python3
"""session_recorder.py — bag-record lifecycle + audio feedback for capture sessions.

Watches /home/jetson/path_capture_session_toggle (written by joy_capture
inside the container when the X button is pressed). Each toggle starts or
stops a `ros2 bag record` process inside the ugv_jp6 container, with audible
WAV feedback so the operator can hear state changes without looking at a
screen.

Bag topic filter: small + compressed topics only — image compressed streams,
cmd_vel, IMU, odom, scan, joy, fix, YOLO detections, lidar_safety, UBX raw
GPS. Excludes /image_raw (raw RGB — too big) and /spatial_detections (HTTP-
only, not a ROS topic).

Companion to oakd_spatial.py which also watches the same toggle file (for
OAK-D RGB frame session capture). Both daemons react independently — same
trigger, different responsibilities.

Audio files are pre-generated via ElevenLabs (one-shot system messages, not
streaming TTS — distinct voice from Aura-2 Hyperion which is Claude's voice):

    /home/jetson/sounds/bag_started.wav
    /home/jetson/sounds/bag_stopped.wav

Run as systemd --user service (etc/session_recorder.service).
"""

import os
import signal
import subprocess
import sys
import time
from pathlib import Path

sys.stdout.reconfigure(line_buffering=True)

SESSION_TOGGLE = Path("/home/jetson/path_capture_session_toggle")
SOUND_STARTED = Path("/home/jetson/sounds/bag_started.wav")
SOUND_STOPPED = Path("/home/jetson/sounds/bag_stopped.wav")
BAG_DIR_HOST = Path("/home/jetson/bags")
BAG_DIR_CONTAINER = "/home/ws/bags"  # bind-mounted from host /home/jetson
CONTAINER = "ugv_jp6"

# Topic filter — small + compressed only. /image_raw is raw RGB (too big);
# /spatial_detections is HTTP-only (not a ROS topic, can't be bagged).
BAG_TOPIC_REGEX = (
    r".*compressed.*|/cmd_vel|/imu/.*|/odom/.*|/scan|/joy|/fix|"
    r"/detections|/lidar_safety/.*|/ubx_.*"
)

PIDFILE = Path("/tmp/session_recorder.pid")
POLL_INTERVAL_S = 0.2

_running = True


def _shutdown(_signum, _frame):
    global _running
    _running = False


def play_sound(path: Path) -> None:
    """Play a WAV via paplay. Non-blocking — fire and forget."""
    if not path.exists():
        print(f"  WARN: sound file not found: {path}")
        return
    try:
        subprocess.Popen(
            ["paplay", str(path)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except FileNotFoundError:
        print("  WARN: paplay not installed (sudo apt install pulseaudio-utils)")


def is_bag_running() -> bool:
    """True iff a ros2 bag record process is alive inside the container."""
    try:
        result = subprocess.run(
            ["docker", "exec", CONTAINER, "pgrep", "-f", "ros2 bag record"],
            capture_output=True, timeout=2.0,
        )
        return result.returncode == 0
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False


def start_bag() -> bool:
    """Start a new bag in the container. Returns True on apparent success."""
    BAG_DIR_HOST.mkdir(parents=True, exist_ok=True)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    bag_name = f"outing_{timestamp}"
    inner = (
        "source /opt/ros/humble/setup.bash && "
        f"cd {BAG_DIR_CONTAINER} && "
        f"ros2 bag record -e '{BAG_TOPIC_REGEX}' -o {bag_name} "
        "> /tmp/bag.log 2>&1"
    )
    try:
        subprocess.run(
            ["docker", "exec", "-d", CONTAINER, "bash", "-c", inner],
            check=True, timeout=5.0,
        )
        # Brief settle so we can verify it's actually up before claiming success.
        time.sleep(0.5)
        if is_bag_running():
            print(f"  BAG START: {bag_name}")
            return True
        print(f"  BAG START failed — process didn't appear; check /tmp/bag.log")
        return False
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
        print(f"  BAG START error: {e}")
        return False


def stop_bag() -> bool:
    """SIGINT the bag process for clean metadata flush. Returns True on success."""
    try:
        subprocess.run(
            ["docker", "exec", CONTAINER, "pkill", "-INT", "-f", "ros2 bag record"],
            check=True, timeout=2.0,
        )
        # Bag needs ~1s to write metadata.yaml. Wait it out.
        time.sleep(1.5)
        print("  BAG STOP")
        return True
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
        print(f"  BAG STOP error: {e}")
        return False


def check_singleton():
    if PIDFILE.exists():
        try:
            old_pid = int(PIDFILE.read_text().strip())
            os.kill(old_pid, 0)
            cmdline = Path(f"/proc/{old_pid}/cmdline").read_text()
            if "session_recorder" in cmdline:
                print(f"another session_recorder running (PID {old_pid}); exiting")
                sys.exit(1)
        except (OSError, ValueError, FileNotFoundError):
            pass
    PIDFILE.write_text(str(os.getpid()))


def main():
    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)
    check_singleton()

    # Establish baseline mtime so we don't trigger on the existing file.
    # Only create the toggle if it's absent: touch(exist_ok=True) bumps the
    # mtime even when the file already exists, and oakd_spatial.py watches this
    # same file — so bumping it here was read by oakd as a fresh toggle event
    # and auto-armed a frame-capture session on EVERY startup (it silently
    # filled the disk). Ensure-exists without the bump fixes the auto-arm.
    if not SESSION_TOGGLE.exists():
        SESSION_TOGGLE.touch()
    last_mtime = SESSION_TOGGLE.stat().st_mtime

    print("=" * 50)
    print("  SESSION RECORDER")
    print(f"  Watch:  {SESSION_TOGGLE}")
    print(f"  Bags:   {BAG_DIR_HOST}")
    print(f"  Sounds: {SOUND_STARTED.parent}")
    print(f"  Topics: {BAG_TOPIC_REGEX}")
    print("=" * 50)

    try:
        while _running:
            time.sleep(POLL_INTERVAL_S)
            try:
                current_mtime = SESSION_TOGGLE.stat().st_mtime
            except OSError:
                continue
            if current_mtime <= last_mtime:
                continue
            last_mtime = current_mtime

            # Toggle fired. Flip based on current bag state, not a tracked
            # local flag — that way we self-heal if the bag died externally.
            if is_bag_running():
                if stop_bag():
                    play_sound(SOUND_STOPPED)
            else:
                if start_bag():
                    play_sound(SOUND_STARTED)
    finally:
        PIDFILE.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
