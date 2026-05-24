#!/usr/bin/env python3
"""
oakd_spatial.py — Person tracking with metric 3D position, via OAK-D Lite.

Runs a DepthAI pipeline on the OAK-D's Myriad X VPU:
    ColorCamera → MobileNetSpatialDetectionNetwork → ObjectTracker
    StereoDepth → feeds spatial calculator inside the NN node

Publishes tracklets (metric x, y, z in camera frame, bearing, distance,
stable tracking ID) as JSON via HTTP POST to the rover bridge's
/spatial_detections endpoint. The Jetson does zero image processing here
— detections arrive structured, already tracked across frames, already
metric. This is the nav-perception layer (brainstem), distinct from
yolo_detector on the Jetson which is attention-perception (cortex).

Runs on HOST (not in container) because OAK-D USB access is host-only.
Same architecture as the old depth_safety daemon.

Env / deps:
    /home/jetson/ugv_jetson/ugv-env has depthai 3.5.0 and requests.
    No blob file needed — the NN blob is fetched from the DepthAI hub
    on first run by dai.NNModelDescription("mobilenet-ssd").

State / files:
    /tmp/oakd_spatial.pid — lockfile, self-managed.

Output format per detection:
    {
      "id": "7",                       # stable tracking ID, string
      "class_id": "person",
      "status": "TRACKED",             # TRACKED / NEW / LOST / REMOVED
      "score": 0.84,
      "position_m": {"x": 0.3, "y": -0.1, "z": 2.1},  # right+, down+, forward+
      "bearing_deg": 8.1,              # +ve = target is to the right of forward
      "distance_m": 2.12,              # horizontal distance (sqrt(x² + z²))
      "bbox_px": {"cx": 180, "cy": 150, "w": 60, "h": 220}
    }
"""

import math
import os
import signal
import sys
import time
from pathlib import Path

import cv2
import depthai as dai
import requests

sys.stdout.reconfigure(line_buffering=True)

BRIDGE_URL = "http://localhost:5000"
PIDFILE = Path("/tmp/oakd_spatial.pid")

# Path-segmenter training data capture. Two modes, both driven by joy_capture.py
# inside the container via the /home/jetson:/home/ws bind mount.
#
# Mode 1 — one-shot (Y button): joy writes a timestamp to CAPTURE_TRIGGER, we
# save the latest RGB frame to CAPTURE_DIR. Good for picking specific surfaces.
#
# Mode 2 — capture session (X button toggle): joy writes a timestamp to
# SESSION_TOGGLE; on each toggle we flip session_active. While active, every
# new RGB frame from the camera (~15 Hz) gets JPEG-encoded and saved to a
# timestamped session subdirectory. Pair with `ros2 bag record` in the
# container to harvest a full sensor stream + paired OAK-D RGB for
# Phase 2 BC training. Combine post-hoc by timestamp.
CAPTURE_TRIGGER = Path("/home/jetson/path_capture_request")
CAPTURE_DIR = Path("/home/jetson/path_frames")
SESSION_TOGGLE = Path("/home/jetson/path_capture_session_toggle")
SESSION_DIR = Path("/home/jetson/path_sessions")
SESSION_JPEG_QUALITY = 85  # OpenCV default is 95; 85 keeps quality high while
                           # roughly halving file size. Lossy by design — DINOv2
                           # / CLIP / RADIO were all trained on JPEG-compressed
                           # imagery, so artifacts are in-distribution for the
                           # eventual Phase 2 backbone.

# YOLO COCO class labels — only the ones we're likely to see on rover
# missions. Full 80-class list is at https://github.com/amikelive/coco-labels.
# We don't need all of them for tracking; unknown label IDs will render as
# "cls_N" which is fine for logs.
COCO_LABELS = {
    0: "person",
    1: "bicycle",
    2: "car",
    3: "motorcycle",
    7: "truck",
    14: "bird",
    15: "cat",
    16: "dog",
    17: "horse",
    19: "cow",
    20: "elephant",  # YOLO-COCO mislabels kangaroos as elephant — noted in CLAUDE.md
}
PERSON_LABEL_ID = 0  # YOLO-COCO "person" class

CAM_FPS = 15              # 15 Hz is plenty for following; keeps VPU headroom
DEPTH_MIN_MM = 300        # below this, depth is unreliable (too close)
DEPTH_MAX_MM = 8000       # tracking beyond 8m is unreliable on Lite

_running = True


def _shutdown(_signum, _frame):
    global _running
    _running = False


def check_singleton():
    if PIDFILE.exists():
        try:
            old_pid = int(PIDFILE.read_text().strip())
            os.kill(old_pid, 0)
            print(f"oakd_spatial already running (PID {old_pid})", file=sys.stderr)
            sys.exit(1)
        except (OSError, ValueError):
            pass
    PIDFILE.write_text(str(os.getpid()))


def build_pipeline():
    """Build the DepthAI 3.x pipeline.

    Follows the canonical recipe in luxonis/oak-examples @ neural-networks/
    object-tracking/collision-avoidance/main.py. Key RVC2-specific bits:
      - StereoDepth built with explicit left/right via .build()
      - setDepthAlign(CAM_A) + setOutputSize(*nn_input_size) required
      - setNNArchive(archive, numShaves=4) after build() for OAK-D Lite
      - ZERO_TERM_COLOR_HISTOGRAM is the only tracker type supported
        on RVC2 (SHORT_TERM_KCF / _IMAGELESS are RVC4+ only)
    """
    pipeline = dai.Pipeline()

    # Resolve the model archive explicitly so we can both extract the
    # person class ID from metadata AND re-apply it with numShaves=4
    # tuned for OAK-D Lite's smaller VPU budget.
    model_desc = dai.NNModelDescription(model="yolov6-nano", platform="RVC2")
    nn_archive = dai.NNArchive(dai.getModelFromZoo(model_desc))
    nn_input_size = nn_archive.getInputSize()  # (640, 640) for yolov6-nano
    labels = nn_archive.getConfig().model.heads[0].metadata.classes
    person_label = labels.index("person")
    print(f"  model input: {nn_input_size}, {len(labels)} classes, person={person_label}")

    # depthai 3.x unified Camera node — replaces ColorCamera and MonoCamera.
    # Socket map on OAK-D Lite:
    #   CAM_A = color (RGB, 1080p sensor)
    #   CAM_B = left mono (OV7251)
    #   CAM_C = right mono (OV7251)
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    left_cam = pipeline.create(dai.node.Camera).build(
        dai.CameraBoardSocket.CAM_B, sensorFps=float(CAM_FPS)
    )
    right_cam = pipeline.create(dai.node.Camera).build(
        dai.CameraBoardSocket.CAM_C, sensorFps=float(CAM_FPS)
    )

    # Stereo depth built via .build() — the 3.x way to wire left/right.
    # HIGH_DETAIL is the preset recommended for outdoor / range tracking on
    # RVC2; FAST_DENSITY is quicker but loses quality at distance.
    stereo = pipeline.create(dai.node.StereoDepth).build(
        left=left_cam.requestOutput((640, 400)),
        right=right_cam.requestOutput((640, 400)),
        presetMode=dai.node.StereoDepth.PresetMode.HIGH_DETAIL,
    )
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    # RVC2 requires the depth output size to equal the NN input size —
    # without setOutputSize the alignment widths aren't guaranteed to be
    # multiples of 16, which StereoDepth rejects at runtime.
    stereo.setOutputSize(*nn_input_size)
    stereo.setLeftRightCheck(True)
    stereo.setRectification(True)

    # Spatial detection network via .build() wires input + stereo + archive
    # in one call. On RVC2, re-apply the archive with numShaves=4 tuned for
    # OAK-D Lite. Larger OAKs can take 6-7 shaves; 4 is correct for Lite.
    spatial_nn = pipeline.create(dai.node.SpatialDetectionNetwork).build(
        input=cam,
        stereo=stereo,
        nnArchive=nn_archive,
        fps=float(CAM_FPS),
    )
    spatial_nn.setNNArchive(nn_archive, numShaves=4)
    spatial_nn.setConfidenceThreshold(0.4)
    spatial_nn.setBoundingBoxScaleFactor(0.5)
    spatial_nn.setDepthLowerThreshold(DEPTH_MIN_MM)
    spatial_nn.setDepthUpperThreshold(DEPTH_MAX_MM)

    # ObjectTracker: stable IDs across frames.
    #
    # Classes we track: **person only**. The spatial_detections stream is
    # consumed primarily by the `follow` intent, and a 6-wheel skid-steer
    # rover cannot meaningfully follow a dog / cat / horse / kangaroo —
    # they move faster than the rover's top speed, change direction
    # unpredictably, and duck under obstacles the rover can't pass. Offering
    # non-person targets in spatial_detections is inviting Haiku to pick a
    # target the platform can't pursue, which ends in lost-target hold
    # every time. Social awareness of animals happens via YOLO on the
    # gimbal camera (see yolo_detector.py classes param), which feeds
    # attention not navigation. This is the right asymmetry for *this*
    # platform; a faster robot with real wildlife-capture ability could
    # legitimately expand the tracker set here.
    #
    # RVC2 (Myriad X) supports only ZERO_TERM tracker types. SHORT_TERM_*
    # are RVC4+. UNIQUE_ID prevents ID ping-pong on brief detection misses.
    tracked_labels = [labels.index("person")]
    tracker = pipeline.create(dai.node.ObjectTracker)
    tracker.setDetectionLabelsToTrack(tracked_labels)
    tracker.setTrackerType(dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM)
    tracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.UNIQUE_ID)
    spatial_nn.out.link(tracker.inputDetections)
    spatial_nn.passthrough.link(tracker.inputTrackerFrame)
    spatial_nn.passthrough.link(tracker.inputDetectionFrame)

    return pipeline, tracker, spatial_nn


def _status_name(status):
    """Tracklet.TrackingStatus → short string."""
    return str(status).split(".")[-1]  # e.g. "TrackingStatus.TRACKED" → "TRACKED"


def tracklet_to_dict(t):
    sc = t.spatialCoordinates  # millimetres
    roi = t.roi.denormalize(300, 300)  # preview pixels (300×300)
    label_name = COCO_LABELS.get(t.label, f"cls_{t.label}")
    dx, dy, dz = sc.x / 1000.0, sc.y / 1000.0, sc.z / 1000.0
    # Bearing: angle of target from forward axis, +ve to the right.
    bearing = math.degrees(math.atan2(dx, dz)) if dz != 0 else 0.0
    # Horizontal distance (ignores vertical offset — what matters for nav).
    horizontal = math.sqrt(dx * dx + dz * dz)
    return {
        "id": str(t.id),
        "class_id": label_name,
        "status": _status_name(t.status),
        "score": round(float(t.srcImgDetection.confidence), 3),
        "position_m": {
            "x": round(dx, 2),
            "y": round(dy, 2),
            "z": round(dz, 2),
        },
        "bearing_deg": round(bearing, 1),
        "distance_m": round(horizontal, 2),
        "bbox_px": {
            "cx": round(roi.x + roi.width / 2, 0),
            "cy": round(roi.y + roi.height / 2, 0),
            "w": round(roi.width, 0),
            "h": round(roi.height, 0),
        },
    }


def post_to_bridge(detections):
    try:
        requests.post(
            f"{BRIDGE_URL}/spatial_detections",
            json={"detections": detections, "ts": time.time()},
            timeout=0.5,
        )
    except requests.RequestException:
        # Bridge isn't up yet, or a transient blip. Drop silently and
        # try again next frame — this is a streaming pipeline, stale
        # frames have no value.
        pass


def _safe_mtime(path: Path) -> float:
    """mtime of `path`, or 0.0 if it's absent or vanishes between check and stat."""
    try:
        return path.stat().st_mtime
    except OSError:
        return 0.0


def main():
    """
    Run the OAK-D spatial detection service: build and start the DepthAI pipeline, capture RGB frames on filesystem triggers, convert TRACKED tracklets into metric detections, and POST those detections to the configured bridge.
    
    This function installs SIGINT/SIGTERM handlers, enforces a single running instance, constructs and starts the pipeline, attaches host output queues for tracker tracklets and passthrough RGB frames, and enters the main processing loop. In the loop it:
    - saves one-shot or session-captured JPEG frames when the corresponding trigger files are updated;
    - filters for TRACKED tracklets, converts them to the outgoing detection schema, and sends them to the bridge;
    - maintains simple runtime statistics and logs a periodic summary.
    On shutdown it stops the pipeline and performs graceful cleanup.
    """
    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)
    check_singleton()

    print("=" * 50)
    print("  OAK-D SPATIAL DETECTION")
    print(f"  Model: yolov6-nano @ 640×640 (RVC2)")
    print(f"  Tracking: person (COCO class {PERSON_LABEL_ID})")
    print(f"  Depth range: {DEPTH_MIN_MM/1000:.1f} - {DEPTH_MAX_MM/1000:.1f} m")
    print(f"  Camera fps: {CAM_FPS}")
    print(f"  Bridge: {BRIDGE_URL}")
    print("=" * 50)

    print("Building pipeline (first run downloads the mobilenet-ssd archive)...")
    pipeline, tracker, spatial_nn = build_pipeline()

    # Attach a host-bound queue to the tracker's output. maxSize=2 +
    # blocking=False means we always consume the freshest tracklet; old
    # tracklets aren't useful for nav control.
    tracklet_q = tracker.out.createOutputQueue(maxSize=2, blocking=False)

    # Tap the NN's passthrough so we have a host-side handle on the latest
    # RGB frame for path-segmenter training capture. maxSize=1 + non-blocking
    # — we only ever care about the freshest frame.
    rgb_q = spatial_nn.passthrough.createOutputQueue(maxSize=1, blocking=False)
    CAPTURE_DIR.mkdir(parents=True, exist_ok=True)
    SESSION_DIR.mkdir(parents=True, exist_ok=True)
    latest_rgb = None
    # Seed "last seen" mtimes from the existing trigger/toggle files so a file
    # that's been on disk since a previous run doesn't read as a fresh event at
    # startup. Without this, a persistent SESSION_TOGGLE (mtime > 0.0) trips the
    # toggle on the first loop and auto-starts a capture session on every
    # boot/restart — which silently filled the disk with frame dumps.
    last_trigger_mtime = _safe_mtime(CAPTURE_TRIGGER)

    # Capture session state — toggled by SESSION_TOGGLE file mtime changes.
    session_active = False
    last_session_toggle_mtime = _safe_mtime(SESSION_TOGGLE)
    current_session_dir: Path | None = None
    session_frame_count = 0

    print("Starting pipeline...")
    pipeline.start()
    print("Pipeline running.")
    print(f"Path one-shot: trigger {CAPTURE_TRIGGER}, save to {CAPTURE_DIR}")
    print(f"Path session:  toggle  {SESSION_TOGGLE}, save to {SESSION_DIR}/session_<ts>/")

    last_log = time.time()
    emit_count = 0
    # IDs seen in the current 5-second log window — lets us report how
    # much ID churn is happening (e.g. "ids seen (5s): 14" means RVC2's
    # ZERO_TERM tracker recycled through 14 unique IDs in 5 seconds,
    # which is expected for a single person with a jittery NN).
    seen_ids_window: set[int] = set()

    try:
        while _running and pipeline.isRunning():
            # Refresh the latest RGB frame whenever a new one arrives.
            # If a capture session is active, save this fresh frame to disk.
            # The save lives inside the rgb_msg block so we only ever save
            # newly-arrived frames, not the same buffered frame repeatedly.
            rgb_msg = rgb_q.tryGet()
            if rgb_msg is not None:
                latest_rgb = rgb_msg.getCvFrame()
                if session_active and current_session_dir is not None:
                    ts_ms = int(time.time() * 1000)
                    save_path = current_session_dir / f"frame_{ts_ms}.jpg"
                    success, encoded = cv2.imencode(
                        ".jpg",
                        latest_rgb,
                        [cv2.IMWRITE_JPEG_QUALITY, SESSION_JPEG_QUALITY],
                    )
                    if success:
                        try:
                            save_path.write_bytes(encoded.tobytes())
                            session_frame_count += 1
                        except OSError as e:
                            print(f"  session save failed: {e}")

            # Mode 1: one-shot capture trigger.
            if CAPTURE_TRIGGER.exists():
                try:
                    mtime = CAPTURE_TRIGGER.stat().st_mtime
                    if mtime > last_trigger_mtime:
                        last_trigger_mtime = mtime
                        if latest_rgb is not None:
                            ts_ms = int(time.time() * 1000)
                            save_path = CAPTURE_DIR / f"oakd_{ts_ms}.jpg"
                            cv2.imwrite(str(save_path), latest_rgb)
                            h, w = latest_rgb.shape[:2]
                            print(f"  ONE-SHOT: {save_path.name} ({w}x{h})")
                        else:
                            print("  one-shot trigger fired but no RGB frame yet")
                except OSError:
                    pass

            # Mode 2: capture session toggle.
            if SESSION_TOGGLE.exists():
                try:
                    mtime = SESSION_TOGGLE.stat().st_mtime
                    if mtime > last_session_toggle_mtime:
                        last_session_toggle_mtime = mtime
                        session_active = not session_active
                        if session_active:
                            ts_label = time.strftime("%Y-%m-%d_%H-%M-%S")
                            current_session_dir = SESSION_DIR / f"session_{ts_label}"
                            current_session_dir.mkdir(parents=True, exist_ok=True)
                            session_frame_count = 0
                            print(f"  SESSION START: {current_session_dir.name}")
                        else:
                            print(
                                f"  SESSION STOP: {current_session_dir.name if current_session_dir else '?'} "
                                f"({session_frame_count} frames captured)"
                            )
                            current_session_dir = None
                except OSError:
                    pass

            data = tracklet_q.tryGet()
            if data is None:
                time.sleep(0.005)
                continue

            # Only emit TRACKED tracklets. NEW/LOST/REMOVED are internal
            # tracker state that confuses downstream consumers.
            detections = [
                tracklet_to_dict(t) for t in data.tracklets
                if t.status == dai.Tracklet.TrackingStatus.TRACKED
            ]
            post_to_bridge(detections)
            emit_count += 1
            for t in data.tracklets:
                if t.status == dai.Tracklet.TrackingStatus.TRACKED:
                    seen_ids_window.add(int(t.id))

            now = time.time()
            if now - last_log > 5.0:
                fps = emit_count / (now - last_log)
                active_n = len(detections)
                # Find the closest currently-tracked target — useful signal
                # for dev-eyes-on-log. In the field this line will be the
                # only repeating log output.
                closest = None
                for t in data.tracklets:
                    if t.status != dai.Tracklet.TrackingStatus.TRACKED:
                        continue
                    dz = t.spatialCoordinates.z / 1000.0
                    dx = t.spatialCoordinates.x / 1000.0
                    d = math.sqrt(dx * dx + dz * dz)
                    if closest is None or d < closest[0]:
                        closest = (d, int(t.id))
                closest_str = (
                    f" | closest: id={closest[1]} @ {closest[0]:.2f}m"
                    if closest else ""
                )
                print(
                    f"oakd_spatial: {fps:.1f}Hz | "
                    f"active: {active_n} | ids seen (5s): {len(seen_ids_window)}"
                    f"{closest_str}"
                )
                emit_count = 0
                last_log = now
                seen_ids_window.clear()
    finally:
        print("oakd_spatial shutting down pipeline")
        try:
            pipeline.stop()
        except Exception:
            pass


if __name__ == "__main__":
    try:
        main()
    finally:
        PIDFILE.unlink(missing_ok=True)
