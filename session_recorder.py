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
    """
    Request a graceful shutdown by clearing the module run flag.
    
    Parameters:
        _signum (int): Received signal number (ignored).
        _frame (types.FrameType): Current stack frame (ignored).
    """
    global _running
    _running = False


def play_sound(path: Path) -> None:
    """
    Play a WAV file for operator feedback without blocking the caller.
    
    Parameters:
        path (Path): Path to the WAV file to play. If the file does not exist a warning is printed.
        
    Notes:
        The function returns immediately after requesting playback. If the system player
        (`paplay`) is not available, a warning is printed.
    """
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
    """
    Check whether a `ros2 bag record` process is running inside the configured Docker container.
    
    Returns:
        bool: `True` if a matching process is found inside the container, `False` otherwise.
    """
    try:
        result = subprocess.run(
            ["docker", "exec", CONTAINER, "pgrep", "-f", "ros2 bag record"],
            capture_output=True, timeout=2.0,
        )
        return result.returncode == 0
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False


def start_bag() -> bool:
    """
    Start a ROS 2 bag recording inside the configured Docker container.
    
    Ensures the host bag directory exists, launches a detached container command to run `ros2 bag record` with the configured topic filter and a timestamped output name, and verifies the recording process started.
    
    Returns:
        True if the bag process appeared to start and is running, False otherwise.
    """
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
    """
    Stop the ros2 bag recording inside the configured container to allow it to flush metadata.
    
    Sends SIGINT to the bag process running in the container and waits ~1.5 seconds to allow metadata (e.g., metadata.yaml) to be written.
    
    Returns:
        True if the stop command was issued and the wait completed, False otherwise.
    """
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
    """
    Ensure only one instance of the session recorder runs by using a PID lock file.
    
    If the PID file exists, attempts to read the recorded PID and verifies a live process with that PID whose command line contains "session_recorder". If such a process is found, prints a message and exits the program with status code 1. Stale, missing, or invalid PID files are ignored. If no conflicting instance is detected, writes the current process PID to the PID file.
    """
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
    """
    Run the session recorder daemon: enforce a single instance, register signal handlers, monitor the toggle file, and start or stop the in-container ros2 bag process while playing feedback sounds.
    
    The function:
    - Registers handlers for SIGTERM and SIGINT to request shutdown.
    - Ensures only one instance runs by creating/checking a PID lock file.
    - Creates the toggle file if it is missing but does not modify its existing modification time.
    - Prints a startup banner describing watched paths and topic filter.
    - Polls the toggle file for modification-time changes; on each detected toggle, queries the container bag state and either starts or stops the bag process and plays the corresponding WAV feedback if the action succeeds.
    - Removes the PID lock file on exit.
    """
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
