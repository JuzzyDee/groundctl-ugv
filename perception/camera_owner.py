#!/usr/bin/env python3
"""
camera_owner.py — single owner of /dev/video_usb.

Architectural extraction of the most load-bearing body sense.

The contract that matters: /snapshot returns a fresh JPEG, or 503. It must
never serve a stale frame as if it were current. The previous design hung
/snapshot off the end of v4l2 → GStreamer → appsink → ROS publisher → DDS →
ros2_bridge subscriber → cached JPEG → Flask. When any link silently stalled
(observed in the field on 2026-05-09 — gst_camera_node reports 0 Hz despite
v4l2-ctl direct streaming at 30.06 fps under full ROS2 load) Haiku still got
served the last good frame and didn't know it was blind.

This process owns the camera. One open of /dev/video_usb. The capture loop
is `v4l2-ctl --stream-mmap` (the only path proven working today) piped to
stdout, with MJPEG SOI/EOI marker parsing. If the subprocess exits or stops
producing markers for CAPTURE_STALL_TIMEOUT_S, we restart it — and /health
surfaces the error so it's visible, not silent.

Endpoints:
- GET  /snapshot   latest JPEG, ONLY if frame_age < FRESHNESS_THRESHOLD_S.
                   Otherwise 503 with the actual age. Mirrors ros2_bridge's
                   /snapshot zoom API (cx, cy, zoom, out_w, out_h) so the
                   heartbeat caller doesn't have to change.
- GET  /stream     multipart MJPEG fan-out from the same buffer. Slow
                   clients drop their own frames — never backpressure the
                   capture path (the failure mode FrameBroadcaster fixed in
                   ros2_bridge.py was the same shape, just one layer up).
- GET  /health     capture state, frame age, frame count, error count, last
                   error message. The thing the heartbeat checks before
                   deciding "the rover has eyes."

ROS publish: NOT in v1. The migration plan is:
    1. Run camera_owner alongside gst_camera_node (gst_camera_node off
       during testing, since /dev/video_usb is single-owner).
    2. Validate /snapshot freshness in field.
    3. Point heartbeat at camera_owner.
    4. Add ROS publish here, point YOLO at it.
    5. Retire gst_camera_node.

Run inside the ugv_jp6 container (where v4l2-ctl is installed):
    python3 camera_owner.py
"""

import argparse
import array
import ctypes
import os
import signal
import subprocess
import sys
import threading
import time
from queue import Queue, Empty, Full

import cv2
import numpy as np
from flask import Flask, request, jsonify, Response, stream_with_context


def _find_usb_device_file(video_dev_path: str) -> str | None:
    """Resolve /dev/video_usb → /dev/bus/usb/NNN/MMM for USBDEVFS_RESET.

    Walks sysfs from /sys/class/video4linux/<name>/device up the parent
    chain until it finds the USB device node with busnum + devnum files.
    Returns the matching /dev/bus/usb path the ioctl can be issued on,
    or None if the chain doesn't lead to a USB device (built-in camera,
    or device removed mid-probe)."""
    try:
        real = os.path.realpath(video_dev_path)
        name = os.path.basename(real)
        sys_path = f"/sys/class/video4linux/{name}/device"
        if not os.path.exists(sys_path):
            return None
        abs_path = os.path.realpath(sys_path)
        # Walk parents until we hit a USB device node (busnum + devnum).
        for _ in range(8):
            if (os.path.exists(os.path.join(abs_path, "busnum"))
                    and os.path.exists(os.path.join(abs_path, "devnum"))):
                with open(os.path.join(abs_path, "busnum")) as f:
                    busnum = int(f.read().strip())
                with open(os.path.join(abs_path, "devnum")) as f:
                    devnum = int(f.read().strip())
                return f"/dev/bus/usb/{busnum:03d}/{devnum:03d}"
            parent = os.path.dirname(abs_path)
            if parent == abs_path:
                return None
            abs_path = parent
    except Exception:
        return None
    return None


def _usb_reset(video_dev_path: str) -> bool:
    """USB bus-reset the device backing video_dev_path. Returns True on success.

    The cheap UVC gimbal cam wedges in a state where v4l2 open() succeeds
    and accepts format negotiation but never delivers a frame. Killing
    v4l2-ctl doesn't recover it; the endpoint stays stuck. USBDEVFS_RESET
    bus-resets the device, which fully unsticks it — confirmed in field
    investigation 2026-05-12 (after reset: 300 frames @ 30.06 fps zero drops)."""
    import fcntl
    dev_file = _find_usb_device_file(video_dev_path)
    if dev_file is None:
        print(
            f"[camera_owner] usb_reset: cannot resolve USB device for {video_dev_path}",
            flush=True,
        )
        return False
    try:
        with open(dev_file, 'wb') as f:
            fcntl.ioctl(f, USBDEVFS_RESET, 0)
        print(f"[camera_owner] usb_reset: {dev_file} OK", flush=True)
        return True
    except Exception as e:
        print(f"[camera_owner] usb_reset: {dev_file} failed: {e}", flush=True)
        return False


def _set_pdeathsig():
    """Install PR_SET_PDEATHSIG=SIGTERM via prctl(2).

    Linux kernel will send SIGTERM to this process when its parent dies —
    by ANY signal, including SIGKILL. Without this, killing camera_owner
    with `pkill -9` orphans the v4l2-ctl subprocess, which keeps holding
    /dev/video_usb until the next reboot or manual kill. Repeated cycles
    of "old camera_owner died, new one starts, can't acquire device" is
    the failure pattern this prevents.

    Used as preexec_fn to subprocess.Popen — runs in the child after
    fork() but before exec(), so it's bound to the actual subprocess.
    """
    try:
        libc = ctypes.CDLL("libc.so.6", use_errno=True)
        PR_SET_PDEATHSIG = 1
        libc.prctl(PR_SET_PDEATHSIG, signal.SIGTERM, 0, 0, 0)
    except Exception:
        # Non-Linux or unusual environment — fail silent. The orphan
        # protection just won't be active; everything else still works.
        pass

sys.stdout.reconfigure(line_buffering=True)


JPEG_SOI = b'\xff\xd8'  # Start of Image — every JPEG begins with this
JPEG_EOI = b'\xff\xd9'  # End of Image — every JPEG ends with this

# 500ms is "very recent" relative to 30 fps capture (~15× headroom against
# scheduling jitter) and well inside heartbeat's 5s cadence.
FRESHNESS_THRESHOLD_S = 0.5

# depth=1 — slow stream clients see at most one stale frame; the capture
# thread never blocks waiting for them.
STREAM_QUEUE_DEPTH = 1

# If v4l2-ctl runs but produces no complete JPEGs for this long, the stall
# watchdog kills the subprocess AND sends USBDEVFS_RESET on the underlying
# USB device. The proven loop architecture shouldn't silently stall, but the
# cheap UVC cam wedges at the USB endpoint level on a regular basis — first
# frame on connect, then nothing. v4l2-ctl restart alone doesn't recover it;
# only a bus reset does. Tight threshold (1.5s) → recovery in ~3s end-to-end.
CAPTURE_STALL_TIMEOUT_S = 1.5

# Sanity ceiling on accumulated bytes before the first SOI is seen — protects
# against runaway buffer growth if we get garbage out of v4l2-ctl.
MAX_PRE_SOI_BUFFER = 1024 * 1024

DEFAULT_DEVICE = '/dev/video_usb'
# 720p default — cheap UVC gimbal cam ran 30s+ sustained at 30fps zero drops
# on 720p MJPG during 2026-05-12 USB-wedge investigation, vs 1080p MJPG which
# dropped buffers even in a 2s probe immediately after USB reset. The bandwidth
# margin matters more than the pixel count for Haiku's downscaled view. Pass
# --width/--height to override if a future cam handles 1080p cleanly.
DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 720
DEFAULT_FPS = 30
DEFAULT_PORT = 5001

# USBDEVFS_RESET = _IO('U', 20) — userspace bus-reset ioctl. Resets the USB
# endpoint without re-enumeration. Doesn't need host sudo — works from inside
# the docker container as long as we have write access to /dev/bus/usb/NNN/MMM.
USBDEVFS_RESET = 21780

DEFAULT_ROS_TOPIC = '/image_raw/compressed'
DEFAULT_ROS_FRAME_ID = 'camera_optical_frame'


class CameraOwner:
    """Owns /dev/video_usb. Captures continuously into a single-frame buffer
    with timestamp. /snapshot reads it (with freshness check), /stream fans
    it out to subscribers."""

    def __init__(self, device, width, height, fps):
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps

        self._lock = threading.Lock()
        self._latest_jpeg: bytes | None = None
        self._latest_ts: float = 0.0
        self._frame_count: int = 0
        self._capture_errors: int = 0
        self._last_error_msg: str | None = None
        self._capture_started_at: float = 0.0

        # Shared decode cache for foveated /snapshot zoom — keyed on frame
        # count so a burst of zoom calls against the same JPEG decodes once.
        self._decoded_frame: np.ndarray | None = None
        self._decoded_for_count: int = -1

        self._stream_subs: list[Queue] = []
        self._stream_lock = threading.Lock()

        # Optional ROS publisher — set by main() if --publish-ros is on.
        # Stays None for HTTP-only operation (the v1 default).
        self.ros_pub = None

        # Reference to the active v4l2-ctl subprocess, so the stall watchdog
        # thread can terminate() it from outside the capture loop. None
        # between runs / before first run.
        self._current_proc: subprocess.Popen | None = None

        self._stop = threading.Event()

    def start(self):
        threading.Thread(
            target=self._capture_loop, name='capture_loop', daemon=True,
        ).start()
        threading.Thread(
            target=self._stall_watchdog, name='stall_watchdog', daemon=True,
        ).start()

    def _stall_watchdog(self):
        """Independent thread. Catches the failure mode the inline watchdog
        in _consume_mjpeg can't: v4l2-ctl is alive but producing ZERO
        output (USB-level stall, kernel queue stuck, etc.). In that case
        stream.read() blocks forever — the inline check never runs.

        Fix: this thread polls _latest_ts on a 1s tick. If frames stop
        arriving for 2× the inline threshold, terminate the v4l2-ctl
        subprocess. That unblocks stream.read() (returns EOF), the inline
        loop returns naturally, the outer loop respawns v4l2-ctl.

        Real incident on 2026-05-10: camera_owner ran 7+ hours with
        capture_errors=0 but frame_count frozen for over an hour. Inline
        watchdog never fired because read() was wedged. Freshness
        contract on /health correctly reported healthy=false; this fix
        adds the recovery path."""
        threshold = CAPTURE_STALL_TIMEOUT_S * 2
        while not self._stop.is_set():
            time.sleep(1.0)
            with self._lock:
                ts = self._latest_ts
                count = self._frame_count
            # Skip if we haven't received any frames yet — first start-up
            # of v4l2-ctl can take longer than the threshold.
            if ts == 0 or count == 0:
                continue
            age = time.time() - ts
            if age <= threshold:
                continue
            print(
                f"[camera_owner] WATCHDOG: stalled {age:.1f}s — USB-reset + restart",
                flush=True,
            )
            with self._lock:
                self._capture_errors += 1
                self._last_error_msg = f"watchdog stall {age:.1f}s"
            # USB-reset FIRST. Terminating v4l2-ctl alone doesn't unstick a
            # wedged endpoint — only a bus reset does. The reset is safe to
            # call while v4l2-ctl holds the device; v4l2-ctl will then see
            # EOF on its stdout and exit, the outer loop respawns it on a
            # freshly-reset device.
            _usb_reset(self.device)
            proc = self._current_proc
            if proc is not None:
                try:
                    proc.terminate()
                except Exception:
                    pass
            # Don't spam-kill; the outer loop will respawn within a
            # second or two and the next watchdog tick will see fresh
            # frames again.
            time.sleep(2.0)

    def _capture_loop(self):
        while not self._stop.is_set():
            try:
                self._run_v4l2_ctl()
            except Exception as e:
                with self._lock:
                    self._capture_errors += 1
                    self._last_error_msg = f"capture loop: {e}"
                print(f"[camera_owner] capture loop exception: {e}", flush=True)
            time.sleep(1.0)

    def _run_v4l2_ctl(self):
        # --stream-to=/dev/stdout is portable across v4l2-utils versions.
        # --stream-count=0 means stream until killed (otherwise it stops
        # after a fixed count and we'd respawn it constantly).
        cmd = [
            'v4l2-ctl', '-d', self.device,
            f'--set-fmt-video=width={self.width},height={self.height},pixelformat=MJPG',
            '--stream-mmap',
            '--stream-count=0',
            '--stream-to=/dev/stdout',
        ]
        print(f"[camera_owner] starting: {' '.join(cmd)}", flush=True)
        with self._lock:
            self._capture_started_at = time.time()
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,  # v4l2-ctl chatter would deadlock if not drained
            bufsize=0,
            preexec_fn=_set_pdeathsig,  # SIGTERM the child if camera_owner dies (Linux-only, no-op elsewhere)
        )
        # Expose to the stall watchdog so it can terminate() us from outside
        # if v4l2-ctl goes silent without exiting.
        self._current_proc = proc
        try:
            self._consume_mjpeg(proc.stdout)
        finally:
            self._current_proc = None
            try:
                proc.terminate()
                proc.wait(timeout=2.0)
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass

    def _consume_mjpeg(self, stream):
        """Read v4l2-ctl stdout, slice into JPEGs on SOI/EOI markers.

        FFD9 (EOI) only appears once at the end of a valid JPEG — JPEG byte
        stuffing escapes any FF in the entropy-coded data as FF00, so a plain
        find(EOI) is safe. FFD8 (SOI) likewise only appears at the start.
        """
        buf = bytearray()
        last_frame_ts = time.time()

        while not self._stop.is_set():
            chunk = stream.read(65536)
            if not chunk:
                with self._lock:
                    self._capture_errors += 1
                    self._last_error_msg = "v4l2-ctl stdout closed"
                print("[camera_owner] v4l2-ctl stdout closed; restarting", flush=True)
                return
            buf.extend(chunk)

            while True:
                soi = buf.find(JPEG_SOI)
                if soi < 0:
                    if len(buf) > MAX_PRE_SOI_BUFFER:
                        del buf[:]
                    break
                eoi = buf.find(JPEG_EOI, soi + 2)
                if eoi < 0:
                    # Trim anything before the partial SOI to keep the
                    # accumulator small while we wait for the rest.
                    if soi > 0:
                        del buf[:soi]
                    break
                jpeg_end = eoi + 2  # include EOI bytes
                jpeg = bytes(buf[soi:jpeg_end])
                del buf[:jpeg_end]
                self._on_frame(jpeg)
                last_frame_ts = time.time()

            if time.time() - last_frame_ts > CAPTURE_STALL_TIMEOUT_S:
                with self._lock:
                    self._capture_errors += 1
                    self._last_error_msg = (
                        f"stall: no MJPEG markers for {CAPTURE_STALL_TIMEOUT_S}s"
                    )
                print(
                    f"[camera_owner] capture stall after "
                    f"{CAPTURE_STALL_TIMEOUT_S}s — restarting",
                    flush=True,
                )
                return

    def _on_frame(self, jpeg: bytes):
        ts = time.time()
        with self._lock:
            self._latest_jpeg = jpeg
            self._latest_ts = ts
            self._frame_count += 1

        with self._stream_lock:
            queues = list(self._stream_subs)
        for q in queues:
            if q.full():
                try:
                    q.get_nowait()
                except Empty:
                    pass
            try:
                q.put_nowait(jpeg)
            except Full:
                pass

        # ROS publish — best-effort, never let it block capture.
        if self.ros_pub is not None:
            try:
                self.ros_pub.publish_jpeg(jpeg)
            except Exception as e:
                # Don't let a ROS hiccup take down the capture loop. The
                # HTTP path is still serving fresh frames either way.
                print(f"[camera_owner] ros publish error: {e}", flush=True)

    def get_latest(self) -> tuple[bytes | None, float, int]:
        with self._lock:
            return self._latest_jpeg, self._latest_ts, self._frame_count

    def get_decoded(self) -> tuple[np.ndarray | None, int]:
        """Lazy BGR decode of latest JPEG, cached by frame count."""
        with self._lock:
            jpeg = self._latest_jpeg
            count = self._frame_count
            if self._decoded_for_count == count and self._decoded_frame is not None:
                return self._decoded_frame, count
        if jpeg is None:
            return None, count
        nparr = np.frombuffer(jpeg, dtype=np.uint8)
        frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        if frame is None:
            return None, count
        with self._lock:
            if self._frame_count == count:
                self._decoded_frame = frame
                self._decoded_for_count = count
        return frame, count

    def health(self) -> dict:
        with self._lock:
            ts = self._latest_ts
            count = self._frame_count
            errors = self._capture_errors
            err = self._last_error_msg
            started = self._capture_started_at
        now = time.time()
        age = now - ts if ts > 0 else None
        healthy = age is not None and age < FRESHNESS_THRESHOLD_S
        uptime = now - started if started > 0 else None
        avg_fps = (count / uptime) if (uptime and uptime > 1.0) else None
        return {
            "healthy": healthy,
            "frame_age_s": round(age, 3) if age is not None else None,
            "frame_count": count,
            "capture_errors": errors,
            "last_error": err,
            "uptime_s": round(uptime, 1) if uptime is not None else None,
            "avg_fps": round(avg_fps, 2) if avg_fps is not None else None,
            "device": self.device,
            "width": self.width,
            "height": self.height,
            "target_fps": self.fps,
            "freshness_threshold_s": FRESHNESS_THRESHOLD_S,
        }

    def subscribe_stream(self) -> Queue:
        q: Queue = Queue(maxsize=STREAM_QUEUE_DEPTH)
        with self._stream_lock:
            self._stream_subs.append(q)
        return q

    def unsubscribe_stream(self, q: Queue):
        with self._stream_lock:
            try:
                self._stream_subs.remove(q)
            except ValueError:
                pass


_STREAM_NOBUF_HEADERS = {
    "Cache-Control": "no-cache, no-store, must-revalidate, max-age=0",
    "Pragma": "no-cache",
    "Expires": "0",
    "X-Accel-Buffering": "no",
}


class RosJpegPublisher:
    """Publishes the camera's native MJPEG bytes to /image_raw/compressed.

    Imported and instantiated only when --publish-ros is set, so the
    HTTP-only mode keeps no ROS dependencies live. Same QoS profile as
    the gst_camera_node it replaces (best-effort, depth=1) so existing
    consumers (yolo_detector, ros2_bridge) attach without changes.
    """

    def __init__(self, topic, frame_id):
        import rclpy
        from rclpy.node import Node as RclNode
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import CompressedImage

        self._CompressedImage = CompressedImage
        rclpy.init()
        self.node = RclNode('camera_owner')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.publisher = self.node.create_publisher(CompressedImage, topic, qos)
        self.frame_id = frame_id

        # Spin in a daemon thread so the node services parameter callbacks etc.
        self._spinner = threading.Thread(
            target=lambda: rclpy.spin(self.node), daemon=True, name='ros_spin',
        )
        self._spinner.start()

    def publish_jpeg(self, jpeg: bytes):
        msg = self._CompressedImage()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.format = 'jpeg'
        msg.data = array.array('B', jpeg)
        self.publisher.publish(msg)


def _mjpeg_part(jpeg: bytes) -> bytes:
    return (b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n'
            b'Content-Length: ' + str(len(jpeg)).encode() + b'\r\n\r\n'
            + jpeg + b'\r\n')


app = Flask(__name__)
owner: CameraOwner | None = None


@app.route('/health', methods=['GET'])
def health():
    return jsonify(owner.health())


@app.route('/snapshot', methods=['GET'])
def snapshot():
    """Return latest camera JPEG IF fresh, else 503.

    Default: full frame at native aspect, downscaled to out_w (default 640).
    Foveal zoom params (cx, cy, zoom, out_w, out_h) mirror ros2_bridge's
    /snapshot so heartbeat callers don't have to change. `?raw` returns the
    untouched camera JPEG with no decode/resize.
    """
    jpeg, ts, count = owner.get_latest()
    if jpeg is None:
        return jsonify({"error": "no frame yet — capture not started"}), 503
    age = time.time() - ts
    if age > FRESHNESS_THRESHOLD_S:
        return jsonify({
            "error": "stale frame",
            "age_s": round(age, 3),
            "threshold_s": FRESHNESS_THRESHOLD_S,
            "frame_count": count,
        }), 503

    headers = {
        "X-Frame-Age-Ms": str(int(age * 1000)),
        "X-Frame-Count": str(count),
    }

    if 'raw' in request.args:
        return Response(jpeg, mimetype='image/jpeg', headers=headers)

    frame, _ = owner.get_decoded()
    if frame is None:
        return jsonify({"error": "decode failed"}), 503

    H, W = frame.shape[:2]
    source_aspect = W / H

    out_w = int(request.args.get('out_w', 640))
    if 'out_h' in request.args:
        out_h = int(request.args.get('out_h'))
    else:
        out_h = int(out_w / source_aspect)

    zoom = float(request.args.get('zoom', 1.0))
    zoom = max(1.0, min(zoom, 8.0))
    cx = int(request.args.get('cx', W // 2))
    cy = int(request.args.get('cy', H // 2))

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

    cropped = np.ascontiguousarray(frame[y1:y2, x1:x2])
    if cropped.size == 0:
        return jsonify({"error": "crop empty"}), 503

    resized = cv2.resize(cropped, (out_w, out_h), interpolation=cv2.INTER_AREA)
    ok, out_jpeg = cv2.imencode('.jpg', resized, [cv2.IMWRITE_JPEG_QUALITY, 85])
    if not ok:
        return jsonify({"error": "jpeg encode failed"}), 500

    return Response(out_jpeg.tobytes(), mimetype='image/jpeg', headers=headers)


@app.route('/stream', methods=['GET'])
def stream():
    """Multipart MJPEG fan-out. Slow clients drop their own frames; the
    capture loop is never blocked by HTTP backpressure."""
    @stream_with_context
    def generate():
        q = owner.subscribe_stream()
        try:
            while True:
                try:
                    jpeg = q.get(timeout=5.0)
                except Empty:
                    continue
                yield _mjpeg_part(jpeg)
        finally:
            owner.unsubscribe_stream(q)

    return Response(
        generate(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
        headers=dict(_STREAM_NOBUF_HEADERS),
        direct_passthrough=True,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', default=DEFAULT_DEVICE)
    parser.add_argument('--width', type=int, default=DEFAULT_WIDTH)
    parser.add_argument('--height', type=int, default=DEFAULT_HEIGHT)
    parser.add_argument('--fps', type=int, default=DEFAULT_FPS)
    parser.add_argument('--port', type=int, default=DEFAULT_PORT)
    parser.add_argument('--publish-ros', action='store_true',
                        help='Also publish frames to /image_raw/compressed')
    parser.add_argument('--ros-topic', default=DEFAULT_ROS_TOPIC)
    parser.add_argument('--ros-frame-id', default=DEFAULT_ROS_FRAME_ID)
    args = parser.parse_args()

    global owner
    owner = CameraOwner(args.device, args.width, args.height, args.fps)

    if args.publish_ros:
        print(f"[camera_owner] ROS publish: {args.ros_topic}", flush=True)
        owner.ros_pub = RosJpegPublisher(args.ros_topic, args.ros_frame_id)

    owner.start()

    try:
        from waitress import serve
        print(f"[camera_owner] waitress on :{args.port}", flush=True)
        serve(app, host='0.0.0.0', port=args.port, threads=8)
    except ImportError:
        print(f"[camera_owner] flask dev server on :{args.port}", flush=True)
        app.run(host='0.0.0.0', port=args.port, threaded=True, debug=False)


if __name__ == '__main__':
    main()
