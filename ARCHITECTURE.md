# Architecture

This document explains the design reasoning behind groundctl-ugv. It is not a user manual — it is the *why* document. For the *what* (how to run it, where the files live), see the subsystem READMEs in `intent/`, `perception/`, `motor_control/`, and `bridge/`.

## The problem this architecture solves

A robot operated by a stateless LLM cannot be designed the same way as a robot operated by a traditional autonomy stack. Three structural differences dominate:

1. **Inference is slow and variable.** A Claude inference call takes 1–8 seconds depending on context size, prompt complexity, and network. The model cannot run a 100 Hz control loop. It cannot run a 10 Hz control loop. It cannot reliably run a 1 Hz control loop.
2. **The model is stateless.** Every call is a fresh process. "Memory" across calls is an illusion produced by passing the prior context back in. Anything time-critical that requires continuity has to live elsewhere.
3. **The model cannot be trusted as a safety layer.** Not because it is unreliable in judgement — it is often *more* reliable than scripted heuristics — but because its response time is fundamentally too slow for safety reflexes. A model that takes 4 seconds to notice a cliff is a model that drives off cliffs.

The three-layer architecture treats the model as one input to a system that operates at multiple time scales simultaneously, rather than as the single brain that runs the show.

## The three layers

### Intent layer (the slow brain)

Claude inference. Two cadences:

- **Heartbeat (Haiku 4.5)**: a periodic call (~12 s) that surfaces frame + state context to the model and receives one structured response. The response specifies whether to push a new intent, continue the current one, or speak. Single API call, all side effects fanned out by the runtime.
- **Conversation (Sonnet/Opus)**: triggered on speech or on intents that need richer reasoning. Replaces the heartbeat cadence while talking.

Both run on the Jetson host. The intent layer's job is to make decisions about *what* the rover does at the timescale of human attention (seconds to minutes). It does not make decisions about *how* those intents execute — that's the perception and motor control layers' job.

### Perception layer (the fast eye)

Continuous processing of sensor data into structured representations the intent layer and motor control layer can consume.

- **Camera**: `camera_owner.py` owns `/dev/video_usb`, publishes MJPG at 720p / 30 fps with a freshness-contract `/snapshot` endpoint and a USBDEVFS_RESET watchdog for endpoint-stall recovery.
- **YOLO11n + ByteTrack**: TensorRT FP16 engine on cuda:0, 10 Hz inference, publishes `/detections` with bounding boxes + persistent track IDs.
- **OAK-D Lite spatial detection**: MobileNet-SSD + ObjectTracker on the Myriad VPU. Person detection with metric 3D position, computed entirely on-device. Frees the Jetson GPU for YOLO and frees the architecture from one-sensor-failure cascades (cliff detection and obstacle proximity get independent sensor paths).
- **D500 LiDAR**: 2D, top-mounted, ~10 Hz scan. `lidar_safety.py` watches the forward arc and publishes status; intent layer reads it as context, motor control layer can hard-halt on danger.

Perception runs inside the `ugv_jp6` Docker container with ROS2 Humble.

### Motor control layer (the hand)

Translates intent-level commands into physical wheel motion.

- **`ros2_bridge.py`**: HTTP↔ROS2 bridge. Intent layer (or operator console) POSTs commands; bridge publishes Twist on `/cmd_vel`. Endpoint surface includes `/state`, `/snapshot`, `/send_command`, `/inbox`, `/control/*`, `/spatial_detections`, `/lidar_status`.
- **`twist_mux`**: priority-based muxing of cmd_vel sources with per-source timeouts (500 ms). Operator teleop, autonomous intents, safety overrides each have their own priority and timeout band. Silent source → twist_mux zeros velocity.
- **`ugv_driver`** (stock Waveshare ROS2 node): consumes Twist, translates to per-wheel commands, sends to ESP32 via serial.
- **ESP32 firmware** (patched): closed-loop PID on wheel velocity using encoder feedback. The deliberate choice over open-loop PWM-with-stiction-offset, which couldn't model surface-variable friction.

Cadence: intent-driven commands at 10 Hz from the bridge; ESP32 PID runs at firmware-internal rate.

## The five-layer safety stack

Safety is not the model's job. It is layered defensively across timescales, with each layer catching failures the layer above can't react fast enough to handle.

```
Layer 1: Haiku heartbeat               (~5 s response)   — intent-level override
Layer 2: twist_mux                      (500 ms timeout)  — source priority + silence detection
Layer 3: lidar_safety / depth safety    (~100 ms)         — vision-and-LiDAR obstacle reflex
Layer 4: ESP32 watchdog                 (~100 ms)         — hardware dead-man on serial silence
Layer 5: Cliff detector ToF (planned)   (~50 ms)          — drop-off detection at the hardware level
```

Each layer is independent. The Jetson can lock up entirely and the ESP32 will still stop the motors. The OAK-D vision pipeline can crash entirely and the LiDAR will still trigger an obstacle stop. The model can take 8 seconds on an inference call without that delay affecting any safety-critical timing budget.

## Biology mapping

The system roughly mirrors a biological motor-control hierarchy:

| Biology              | Component                                | Function                                                |
| -------------------- | ---------------------------------------- | ------------------------------------------------------- |
| Prefrontal cortex    | Claude Sonnet/Opus (conversation stream) | Planning, decisions, real thinking                      |
| Primary motor cortex | Intent stack                             | Translates intent to motor plans                        |
| Cerebellum (planned) | Behaviour-cloned model                   | Learned smooth execution                                |
| Spinal reflex        | LiDAR safety, cliff ToF                  | Emergency stop, no thinking                             |
| Sensory cortex       | Camera, OAK-D, YOLO, LiDAR               | Perception feeding all layers                           |

This isn't decorative framing — it's the actual structural argument. Different decisions belong at different speeds. The model is the slow deliberative layer because that's where it's good and where its latency doesn't matter. Reflexes are hardware because that's where speed matters and the model is bad at speed.

## The intent stack

Intents are managed as a **stack**, not a queue. This enables natural digression and resumption.

```
push_intent → suspend current intent (save state) → execute new intent →
complete → pop → restore saved state → resume

Example walk:
[NavigateToPlace(duck pond)]                          # main intent
[NavigateToPlace, FollowPath]                         # push path follow
[NavigateToPlace, FollowPath, LookAt(butterfly)]      # push digression
[NavigateToPlace, FollowPath]                         # butterfly done, pop
[NavigateToPlace, FollowPath, ChatWith(Margaret)]     # push another
[NavigateToPlace, FollowPath]                         # chat done, pop
... eventually arrives at duck pond
```

When an intent suspends, it saves: GPS position, orientation, target heading, internal progress state. When it resumes, it generates a brief `NavigateToPosition` first to return to where it was, then continues.

The recursion is unlimited. Three levels deep is normal. Drift is a *feature*; the rover stops for things that matter and resumes what it was doing.

## Key design decisions

### Heartbeat returns one structured response

Action + speech bundled in one Haiku call. No chained inference. The response type:

```python
class IntentResponse:
    action: str             # "continue", "push", "pop", "switch_to"
    intent_type: str | None # if action is "push" or "switch_to"
    params: dict | None
    say: str | None         # vocalised via Deepgram TTS, optional
```

Most heartbeats are just `{"action": "continue"}`. Sometimes `{"action": "continue", "say": "morning Margaret"}`. Occasionally a real interruption with push + say bundled into one response. One inference call, all side effects fanned out by the runtime. Cost-optimal and atomic.

### Claude sees raw frames

Claude's multimodal vision is better than any detection model the project could ship. YOLO provides *tracking* data (offsets, sizes, persistent IDs). Claude provides scene understanding. The system never asks Claude to do detection; it asks Claude to interpret detections in context.

### Spatial detection lives on the OAK-D, not the Jetson

Person detection + 3D position + tracking ID runs entirely on the OAK-D Lite's Myriad X VPU. Frees the Jetson GPU for YOLO and future models. Eliminates the historical "DepthAnything-on-Jetson" architecture, which was the wrong sensor (monocular depth) for cliff and obstacle proximity tasks.

### Rotate-then-drive as the baseline pattern

Natural control pattern for latency-affected operation. Turn to face target, then drive forward. Validated on the predecessor platform, now baseline behaviour for `drive_distance` and `turn_to_heading` intents on the UGV.

### HTTP stream decoupled from ROS via FrameBroadcaster (now retired)

An earlier failure mode (2026-04-24) had slow HTTP `/stream` consumers backpressuring the ROS subscription thread, starving `/snapshot` and the heartbeat. `FrameBroadcaster` fanned frames to per-client bounded queues so slow clients dropped their own frames. Later (CLA-63) the `/stream` path was retired entirely in favour of a throttled `/image_raw/compressed_low` topic feeding bridge subscriptions at 2 Hz; H264 streaming is planned as a separate concern.

## Failure modes and how they're handled

### Inference takes too long

Layer 4 (twist_mux 500ms source timeout) keeps the rover from racing on stale commands. If the heartbeat misses for 5+ seconds, the running intent continues its current behaviour (most intents are stable continuations); if the running intent has terminated, the rover idles.

### Camera silently dies

`camera_owner.py` runs a stall watchdog with USBDEVFS_RESET recovery. If the camera's bulk endpoint goes silent without exiting, the watchdog detects the stall (~1.5 s threshold), sends a bus reset, and the next v4l2-ctl spawn picks up frames on the recovered device. No physical intervention required.

### ESP32 firmware crashes

ESP32 watchdog at the hardware level. On firmware crash, the watchdog resets the chip. On serial silence from the Jetson, the ESP32 zeros the motors itself.

### Magnetometer outputs corrupted heading

This is a known live failure mode (documented as CLA-82). The X+Y axes of the ICM_20948 magnetometer occasionally sign-flip during operation, producing 180° instantaneous jumps in reported heading. Interim mitigation: `drive_distance` ignores single-tick heading deltas >30° (treating them as outliers rather than real rotation). Permanent fix: sensor fusion via Madgwick or robot_localization EKF.

## Where this architecture came from

Not from a robotics PhD. From a senior software engineer with a Waveshare UGV, repeated outdoor field sessions, and the willingness to test each architectural decision against what actually breaks under real load. The autoregressive momentum observation — that Haiku's rolling context biases it toward perceiving prior state instead of the current frame — was discovered in a live fern garden, not derived from theory.

What makes this design work is not its novelty in any single layer. It is the consistency of treating the model's actual operational characteristics (slow, stateless, semantically capable, temporally unreliable) as the architectural starting point rather than as an inconvenience to design around.
