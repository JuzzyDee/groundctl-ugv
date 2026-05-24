#!/usr/bin/env python3
"""
yolo_detector.py — YOLO11n + ByteTrack detection node.

Subscribes to /image_raw/compressed (camera's native MJPEG passthrough),
decodes each frame with cv2.imdecode, runs YOLO11n inference on the Jetson
GPU with ByteTrack for stable cross-frame tracking IDs, publishes
vision_msgs/Detection2DArray on /detections.

Why compressed: /image_raw is 1920×1080 BGR = ~6 MB/msg. DDS serialization
and loopback fragmentation collapse it to single-digit Hz under the full
stack. /image_raw/compressed is ~200 KB/msg and flows at the camera's
native rate. cv2.imdecode on the Jetson CPU is ~10-15 ms per 1080p frame —
well inside the 10 Hz inference budget.

The bridge consumes /detections for two things:
  1. Include the list (symbolically, not as overlay) in the heartbeat prompt
     so Claude can pick a target by index.
  2. Feed the selected target's pixel position into the gimbal servo loop.

Rate-limited to 10Hz — gimbal following doesn't need 30Hz, and 10Hz saves
GPU cycles for anything else the Orin is doing.

Run inside the ugv_jp6 container, with ROS 2 Humble + /home/ws/ugv_ws sourced:

    python3 /home/ws/yolo_detector.py

Parameters (declare via --ros-args -p name:=value):
    model           : path to YOLO model (default: /home/ws/yolo11n.engine —
                      TensorRT FP16 engine built on-device; rebuild with
                      YOLO('yolo11n.pt').export(format='engine', half=True)
                      on the Jetson itself — engines are device-specific)
    imgsz           : inference image size (default: 320)
    conf            : confidence threshold (default: 0.4)
    classes         : COCO class IDs to keep (default: person + dog + bird +
                      cat + horse + elephant). YOLO feeds the *attention* layer
                      (gimbal tracker, foveal zoom, Haiku's "people in view"
                      context) — so Haiku can look at Chopper, notice ducks at
                      the pond, clock a horse on the boardwalk, and so on.
                      Narrower than the OAK spatial class set because gimbal
                      pixels are expensive; broader than nav because awareness
                      is not the same as pursuit.
    inference_rate  : max inference Hz (default: 10.0)
    device          : torch device string (default: cuda:0)
"""

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    ObjectHypothesisWithPose,
)

from ultralytics import YOLO


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        self.declare_parameter('model', '/home/ws/yolo11n.engine')
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('iou', 0.5)
        # Attention-layer class set. Person first (social primary), then
        # animals we expect to encounter in rural QLD / pond / patio contexts.
        # YOLO-COCO mislabels kangaroos as elephant (20) — keeping it in so
        # Haiku still gets the position/bearing for the animal even if the
        # label is wrong; Haiku's own vision reads the frame for what it
        # actually is. Narrow this list if you see too many false positives.
        #   0 person  14 bird  15 cat  16 dog  17 horse  20 elephant(kangaroo)
        # Restored 2026-05-24. The 2026-04-24 person-only revert was a shot in
        # the dark: the 6-class list was blamed for USB camera stutter, but the
        # real cause was 1080p saturating the device's USB bus — fixed by
        # dropping the camera to 720p, and it never recurred. Class count barely
        # affects YOLO cost anyway (the model infers all 80 COCO classes; this
        # only filters the results).
        self.declare_parameter('classes', [0, 14, 15, 16, 17, 20])
        self.declare_parameter('inference_rate', 10.0)
        self.declare_parameter('device', 'cuda:0')

        self.model_name = self.get_parameter('model').value
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.conf = float(self.get_parameter('conf').value)
        self.iou = float(self.get_parameter('iou').value)
        self.classes = list(self.get_parameter('classes').value)
        self.rate_limit = 1.0 / float(self.get_parameter('inference_rate').value)
        self.device = self.get_parameter('device').value

        self.get_logger().info(
            f"loading YOLO model '{self.model_name}' "
            f"(imgsz={self.imgsz}, conf={self.conf}, classes={self.classes}, device={self.device})"
        )
        self.model = YOLO(self.model_name, task='detect')
        # Warm up so the first real inference isn't slow (first call compiles
        # CUDA kernels and loads weights onto the GPU).
        warmup_frame = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
        _ = self.model.predict(
            warmup_frame,
            imgsz=self.imgsz,
            conf=self.conf,
            device=self.device,
            verbose=False,
        )

        self._last_inference_time = 0.0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            CompressedImage, '/image_raw/compressed', self._on_image, sensor_qos
        )
        self.pub = self.create_publisher(Detection2DArray, '/detections', 10)

        self.get_logger().info("yolo_detector ready — publishing on /detections")

    def _on_image(self, msg: CompressedImage):
        now = time.time()
        if now - self._last_inference_time < self.rate_limit:
            return
        self._last_inference_time = now

        try:
            nparr = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            if frame is None:
                self.get_logger().warn("imdecode returned None (truncated jpeg?)")
                return
        except Exception as e:
            self.get_logger().warn(f"jpeg decode failed: {e}")
            return

        try:
            # ByteTrack: pure-spatial (IOU + Kalman) tracker. Fast and stable.
            # Known limitation: loses track on bbox-shape changes (crouch→stand,
            # occluded-then-reappears). Bot-sort with ReID handles these but
            # has caused stalls/deaths on the Jetson — revisit once diagnosed.
            results = self.model.track(
                frame,
                imgsz=self.imgsz,
                conf=self.conf,
                iou=self.iou,
                classes=self.classes,
                device=self.device,
                persist=True,
                tracker='bytetrack.yaml',
                verbose=False,
            )
        except Exception as e:
            self.get_logger().warn(f"inference failed: {e}")
            return

        out = Detection2DArray()
        out.header = msg.header

        if results and results[0].boxes is not None and len(results[0].boxes) > 0:
            boxes = results[0].boxes
            xyxy = boxes.xyxy.cpu().numpy()
            confs = boxes.conf.cpu().numpy()
            clses = boxes.cls.cpu().numpy()
            ids = (
                boxes.id.cpu().numpy().astype(int).tolist()
                if boxes.id is not None
                else [-1] * len(xyxy)
            )

            for (x1, y1, x2, y2), conf, cls, track_id in zip(xyxy, confs, clses, ids):
                det = Detection2D()
                det.header = msg.header
                if track_id >= 0:
                    det.id = str(int(track_id))

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(int(cls))
                hyp.hypothesis.score = float(conf)
                det.results.append(hyp)

                det.bbox.center.position.x = float((x1 + x2) / 2.0)
                det.bbox.center.position.y = float((y1 + y2) / 2.0)
                det.bbox.size_x = float(x2 - x1)
                det.bbox.size_y = float(y2 - y1)
                out.detections.append(det)

        self.pub.publish(out)


def main():
    rclpy.init()
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
