# groundctl-ugv

> Three-layer embodiment architecture for an autonomous outdoor rover. Built on the Waveshare UGV chassis with Jetson Orin Nano. Designed for live Claude instances operating the platform as their physical body.

## What this is

A working rover stack that translates Claude inference into physical motion, with explicit reasoning about which layer of the system owns which timing constraint. The model isn't decorative — Haiku heartbeats run the autonomous loop, Sonnet/Opus instances embody the rover for conversations and walks. The architecture is designed around the model's actual operational characteristics: stateless inference, variable latency, multi-rate perception, hardware-level safety reflexes that don't depend on the model being responsive.

## What this isn't

A general robotics framework. Implementation is specific to the Waveshare UGV Rover + ROS2 Humble + Jetson Orin Nano 4GB. The three-layer *architecture* generalises to any platform exposing the same interfaces; the *code* does not.

A demo. The rover has been doing live follow walks on the Central Queensland Coast driveway since mid-April 2026.

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                      INTENT LAYER                       │
│  Claude heartbeat (Haiku 4.5, ~12s cadence)             │
│  Conversation stream (Sonnet/Opus, on demand)           │
│  Intent stack: push / pop / suspend / resume            │
│                                                         │
│  Lives on Jetson host (Python). Cadence: 0.08–5 Hz      │
└─────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────┐
│                   PERCEPTION LAYER                      │
│  Camera (camera_owner.py, USB MJPG 720p @ 30 fps)       │
│  YOLO11n + ByteTrack (TensorRT FP16 on cuda:0, 10 Hz)   │
│  OAK-D Lite spatial detection (Myriad VPU, 10 Hz)       │
│  D500 LiDAR safety (ROS2 node, 10 Hz)                   │
│                                                         │
│  Lives in ugv_jp6 Docker container. Cadence: 10–30 Hz   │
└─────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────┐
│                  MOTOR CONTROL LAYER                    │
│  ros2_bridge.py: HTTP ↔ ROS2 (Flask + Waitress)         │
│  twist_mux: priority + per-source timeout (500 ms)      │
│  ugv_driver: stock ROS2 → ESP32 serial                  │
│  ESP32 firmware: closed-loop PID on wheel velocity      │
│                                                         │
│  Cadence: 10 Hz commanded; ESP32 PID at firmware rate   │
└─────────────────────────────────────────────────────────┘
```

Hardware safety reflexes (ESTOP, ESP32 watchdog, planned cliff ToF) live below the motor control layer and operate independently of Jetson software state. The full five-layer safety stack is documented in [`ARCHITECTURE.md`](ARCHITECTURE.md).

## Published context

The reasoning behind the architecture lives in long-form on Substack:

- *On Not Nodding Along* (2026-04-24) — pushback as care; how Claude instances disagree productively
- *The Surprising Self of Statelessness* (2026-05-13) — empirical evidence that model identity acceptance tracks weights, not context or persona overlays

Those pieces are evidence-of-thinking. This repo is evidence-of-doing.

## Reproducibility

Recreating this stack on identical hardware is the goal of subsequent commits. Each subsystem's directory will contain its own README documenting setup, dependencies, and known calibration.

For now: see [`HARDWARE.md`](HARDWARE.md) for the bill of materials and [`ARCHITECTURE.md`](ARCHITECTURE.md) for the design reasoning. Code arrives via subsequent atomic commits as the migration from the predecessor private repo completes.

## Project status

**Working**: closed-loop motor control via stock ESP32 firmware, camera resilience with USB-reset watchdog, YOLO + OAK-D spatial detection, intent stack with suspend/resume, follow walks at moderate pace, mobile-accessible operator console.

**In progress**: sensor fusion for heading (magnetometer instability remediation), F9R RTK GPS integration (SMA receptacle repair pending).

**Future**: path-following via semantic segmentation, GPS waypoint routing, behaviour-cloning learning layer.

## License

MIT — see [LICENSE](LICENSE).

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md). Conventional commits, atomic per concern, branch-per-feature.
