# Hardware

Bill of materials and integration notes for the reference platform.

## Reference platform: Waveshare UGV Rover

The whole stack targets this specific chassis. Other ROS2-compatible platforms can host the architecture, but the implementation assumes this hardware unless noted.

| Component                | Spec                                                          | Notes                                                                                    |
| ------------------------ | ------------------------------------------------------------- | ---------------------------------------------------------------------------------------- |
| Chassis                  | Waveshare UGV Rover, 6-wheel 4WD skid-steer                   | ~$1,500 AUD all-in including Orin                                                        |
| Compute                  | NVIDIA Jetson Orin Nano 4GB (included)                        | JetPack 6.0 / R36.3, CUDA 12.2                                                           |
| Base controller          | ESP32 (included)                                              | Motors, encoders, IMU, lights, serial to Jetson                                          |
| IMU                      | ICM-20948 (on ESP32 board)                                    | 9-DoF (accel + gyro + mag). DMP must be manually enabled in firmware                     |
| Drive batteries          | 3× 18650 Li-ion (3S, 11.1V nominal)                           | ~1.5h runtime when healthy. NOT LiFePO4                                                  |
| LiDAR                    | D500 2D LiDAR, top-mounted                                    | ~360°, 12m range, 10 Hz                                                                  |
| Stereo camera            | OAK-D Lite (Movidius Myriad X VPU)                            | On-device person detection + spatial 3D                                                  |
| Gimbal camera            | Generic USB UVC, pan-tilt                                     | MJPG 720p @ 30fps. USBDEVFS_RESET recovery on stall                                      |
| Audio                    | JMTek USB PnP audio device (mic + speaker)                    | Deepgram Nova-2 in, Aura-2 Hyperion out                                                  |
| Connectivity (Jetson)    | WiFi 6 (built-in), Ethernet, optional M.2 cellular            | Tailscale for remote access                                                              |
| Expansion power          | DC5521 expansion port                                         | For external battery packs                                                               |

## GPS (optional but recommended)

| Component                | Spec                                                          | Notes                                                                                    |
| ------------------------ | ------------------------------------------------------------- | ---------------------------------------------------------------------------------------- |
| RTK GPS                  | SparkFun ZED-F9R                                              | Centimetre-precision via NTRIP; fused IMU + wheel ticks                                  |
| Antenna                  | u-blox ANN-MB1 (L1/L2)                                        | **NOT** ANN-MB5 — L1/L5 is wrong for AusCORS                                             |
| NTRIP service            | AusCORS (Geoscience Australia)                                | Free for research/personal. Roslyn Bay mountpoint RSBY00AUS0, port 443 TLS               |

## Cellular connectivity

Phased adoption:

1. **iPhone hotspot** — initial outings. No theft risk; trivial setup.
2. **SIM8200EA-M2** — permanent 5G when ready for untethered autonomous operation.

## Audio chain

- USB mic captures, streams to Deepgram Nova-2 (`listener_daemon.py`)
- Wake-word gate (`claude` / `hey claude` / `oi claude` plus phonetic mishearings: `clawed`, `claud`, `cooled`, `clod`; also `rover` as alternative)
- POST to bridge `/inbox`
- Conversation reads inbox, generates response, synthesises via Deepgram Aura-2 Hyperion (Australian male)
- Plays back via USB speaker (via PulseAudio with USB PnP sink fallback to direct ALSA)

## Planned hardware

| Component                | Status                                                        | Purpose                                                                                  |
| ------------------------ | ------------------------------------------------------------- | ---------------------------------------------------------------------------------------- |
| TF-Mini ToF (cliff)      | Pending — replaces failed VL53L5CX approach (CLA-77)          | Hardware-level drop-off detection at 45° forward-down                                    |
| Status badge LEDs        | Pending — 8-pixel WS2812B strip + 3-4 discrete pixels         | Visible status indicator: inference, speaking, listening, error, battery                 |
| Mic array (long-term)    | Sourcing — proper multi-element array                         | Direction-of-arrival for safety + conversational beamforming                             |

## Power architecture

```
3S Li-ion pack (11.1V nominal, 9.0V cutoff, 12.6V full)
    │
    ├──► Waveshare UPS module ──► Jetson Orin Nano (5V, ~10W idle / 25W under load)
    │
    └──► ESP32 + motor controllers (direct from pack rail)
```

Known issue: 18650s are undersized for the platform. Peak draw during skid-steer pivot exceeds the cells' continuous discharge rating, causing voltage sag and occasional UPS undervoltage trip. Tracked as CLA-31 (3S1P 21700 upgrade) and CLA-30 (proper BMS + LiPo replacement, long-term).

## Test environment

Central Queensland Coast. Quiet rural cul-de-sac, open grass, no through traffic, WiFi mesh coverage. First outings stay in the cul-de-sac. Eventually: regional boardwalk, duck pond, rail trail into town.

## Hardware that lives elsewhere

- **Memoria server**: home Mac, exposed via Tailscale. The rover's hippocampus.
- **Operator console**: phone, served from the bridge's `/web/` path. Mobile-first.
- **Mac (Claude Code workstation)**: now optional for rover operation — heartbeat runs on the Jetson host as a systemd-user service. Mac is the development surface.
