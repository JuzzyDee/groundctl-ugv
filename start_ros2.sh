#!/bin/bash
# start_ros2.sh — Full-stack bring-up on the rover.
#
# Order and what each step is for:
#   - Stop the legacy container so /dev/* isn't contested
#   - Restart ugv_jp6 for a clean ROS2 node slate
#   - Kill app.py (legacy Flask, superseded by ros2_bridge)
#   - bringup_lidar: LD19 + base_node + robot_state_publisher + TF + twist_mux
#   - pwm_driver: replaces stock ugv_driver + ugv_bringup after launch.
#     Stock firmware cogs on T:13 (unfiltered PID on quantised encoder).
#     pwm_driver closes the loop on the Jetson via T:11 raw PWM. Same
#     ROS node name (`ugv_driver`) + same topics + same scalings as
#     upstream, so ros2_bridge and everything else is unaffected.
#   - Kill joint_state_publisher (spams zero poses at the gimbal servo)
#   - gst_camera_node: NVDEC MJPEG decode + tee publishing raw + compressed
#   - ros2_bridge: subscribes to /image_raw/compressed (30 Hz, ~200 KB/msg
#     instead of 6 MB); serves /state, /snapshot, /stream, /inbox, /track
#   - yolo_detector: subscribes to /image_raw/compressed, cv2.imdecode,
#     ultralytics + ByteTrack → /detections at ~10 Hz
#   - lidar_safety: /scan → danger/caution status file
#   - depth_safety: DISABLED — OAK-D monocular depth is wrong sensor for
#     cliff detection. Waiting on ESP32-wired VL53L1X ToF (see CLAUDE.md
#     safety architecture). Kept in the script as comments for restoration.
#   - gps_stack: NTRIP client (AUSCORS) + ublox_dgnss (F9R via libusb) +
#     nav_sat_fix_hp (HPPOSLLH → /fix). Reads AUSCORS_* from .groundctl.env.
#   - oakd_spatial: NEW — on-device person detection + tracking with metric
#     3D position. MobileNet-SSD + ObjectTracker running entirely on the
#     OAK-D Lite's Myriad X VPU. POSTs tracklets to the bridge's
#     /spatial_detections endpoint. Feeds the upcoming `follow` intent.
#   - pulse mic: best-effort set-default-source (listener uses PULSE_SOURCE
#     env to be independent of this, but other consumers benefit)
#   - listener_daemon: Deepgram Nova-2 streaming STT → /inbox
#
# Prereq on rover: /home/jetson/.groundctl.env must exist (chmod 600) and
# export DEEPGRAM_API_KEY. When heartbeat moves to the rover it will also
# need MEMORIA_WEBHOOK_TOKEN + ANTHROPIC_API_KEY in the same file.
#
# Run from your Mac:
#   ./start_ros2.sh
#
# Then verify:
#   curl http://${ROVER}:5000/state

# Load .env if present so ROVER / ROVER_USER / CONTAINER can come from there.
[ -f .env ] && set -a && source .env && set +a

ROVER="${ROVER:?ROVER not set. Copy .env.example to .env and configure the rover Tailscale/LAN address.}"
ROVER_USER=${ROVER_USER:-jetson}
CONTAINER=${CONTAINER:-ugv_jp6}
OLD_CONTAINER=ugv_jetson_ros_humble

# --bench-mode: skip listener_daemon (Deepgram Nova-2 streaming STT is
# ~$0.46/hr — no point burning that during bench testing where nothing's
# going to Claude). Everything else still comes up: motors, lidar, camera,
# YOLO, OAK-D, GPS, bridge. Use when you're iterating on the rover at the
# desk and don't need voice input.
BENCH_MODE=false
for arg in "$@"; do
    case "$arg" in
        --bench-mode)
            BENCH_MODE=true
            ;;
    esac
done

if [ "$BENCH_MODE" = "true" ]; then
    echo "==> BENCH MODE — listener_daemon will be skipped (and stopped if running)"
fi

# Waveshare images / old docker specs set HTTP(S)_PROXY to 192.168.10.x:10809 (often
# unreachable). That breaks pip, Ultralytics, etc. Strip for every long-lived
# process we start here. To kill it permanently: see migration/launch.sh (no
# proxy in docker run) and remove any export lines from /home/jetson/.bashrc.
UNSET_PROXY='unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY all_proxy ALL_PROXY no_proxy NO_PROXY 2>/dev/null; '

echo "==> Stopping old container (${OLD_CONTAINER}) so it doesn't fight for /dev..."
ssh ${ROVER_USER}@${ROVER} "docker stop ${OLD_CONTAINER} 2>/dev/null || echo already stopped"

echo "==> Restarting ${CONTAINER} for a clean process slate (wipes ROS2 node soup)..."
ssh ${ROVER_USER}@${ROVER} "docker restart ${CONTAINER}"
sleep 8

echo "==> Killing app.py..."
ssh ${ROVER_USER}@${ROVER} "pkill -9 -f 'ugv_jetson/app.py' 2>/dev/null; sleep 2; pgrep -fa 'ugv_jetson/app.py' || echo app.py dead"

echo "==> Cleaning up any old ROS2 nodes in container..."
ssh ${ROVER_USER}@${ROVER} "docker exec ${CONTAINER} bash -c 'pkill -9 -f \"ros2 launch\" 2>/dev/null; pkill -9 -f ugv_driver 2>/dev/null; pkill -9 -f ugv_bringup 2>/dev/null; pkill -9 -f base_node 2>/dev/null; pkill -9 -f LD19 2>/dev/null; pkill -9 -f rf2o 2>/dev/null; pkill -9 -f ros2_bridge 2>/dev/null; pkill -9 -f v4l2_camera_node 2>/dev/null; pkill -9 -f usb_cam_node_exe 2>/dev/null; pkill -9 -f gst_camera_node 2>/dev/null; pkill -9 -f /tmp/camera_owner.py 2>/dev/null; pkill -9 v4l2-ctl 2>/dev/null; pkill -9 -f yolo_detector 2>/dev/null; sleep 2'"

# Brings up LD19, robot_state_publisher, joint_state_publisher, twist_mux,
# ugv_driver, TF chain — NOT usb_cam (that was launched separately).
echo "==> Launching ROS2 bringup..."
ssh ${ROVER_USER}@${ROVER} "docker exec -d ${CONTAINER} bash -c '${UNSET_PROXY}export UGV_MODEL=ugv_rover && export LDLIDAR_MODEL=ld19 && source /opt/ros/humble/setup.bash && source /home/ws/ugv_ws/install/setup.bash && ros2 launch ugv_bringup bringup_lidar.launch.py use_rviz:=false > /tmp/bringup.log 2>&1'"

echo "==> Waiting for bringup..."
sleep 12

echo "==> Temporarilg NOT Deploying pwm_driver (replaces stock ugv_driver + ugv_bringup)..."
# Stock ugv_driver writes T:13 (closed-loop PID in firmware) which cogs
# audibly at low speeds because the firmware's speed estimate is
# unfiltered and 1-pulse-quantised. pwm_driver writes T:11 (raw PWM) and
# closes the loop on the Jetson side with a proper speed filter and
# affine motor model (slope + stiction). Same ROS node name + topics +
# scalings, so ros2_bridge and rf2o_laser_odometry etc. don't notice.
#scp pwm_driver.py ${ROVER_USER}@${ROVER}:/home/jetson/pwm_driver.py
#ssh ${ROVER_USER}@${ROVER} "docker cp /home/jetson/pwm_driver.py ${CONTAINER}:/home/ws/pwm_driver.py"

#echo "==> Replacing stock motor nodes with pwm_driver..."
#ssh ${ROVER_USER}@${ROVER} "docker exec ${CONTAINER} bash -c 'pkill -f \"ugv_bringup/lib/ugv_bringup/ugv_driver\" 2>/dev/null; pkill -f \"ugv_bringup/lib/ugv_bringup/ugv_bringup\" 2>/dev/null; sleep 2'"
#ssh ${ROVER_USER}@${ROVER} "docker exec -d ${CONTAINER} bash -c '${UNSET_PROXY}source /opt/ros/humble/setup.bash && source /home/ws/ugv_ws/install/setup.bash && python3 /home/ws/pwm_driver.py > /tmp/pwm_driver.log 2>&1'"
sleep 3

echo "==> Suppressing joint_state_publisher (it fights the gimbal servo)..."
ssh ${ROVER_USER}@${ROVER} "docker exec ${CONTAINER} pkill -f joint_state_publisher 2>/dev/null; echo jsp killed"

echo "==> Deploying camera_owner (single-owner /dev/video_usb with freshness-contract /snapshot)..."
# Replaces gst_camera_node, which silently stalled in the field — see
# memory project_camera_diagnostic_2026_05_09.md for the architectural
# pivot decided on 2026-05-09 and validated on 2026-05-10. camera_owner:
#   - Owns /dev/video_usb via direct v4l2-ctl subprocess (no GStreamer —
#     proven 30fps under full ROS2 load when gst was reading 0Hz on the
#     same hardware in the same container)
#   - GET /snapshot returns 503 with explicit age if frame_age > 500ms.
#     Heartbeat never gets silently lied to about having sight.
#   - GET /stream multipart MJPEG fan-out from the same buffer
#   - GET /health surfaces capture state, frame age, error count
#   - Publishes /image_raw/compressed with same QoS as gst_camera_node had,
#     so yolo_detector and ros2_bridge attach without changes
#   - PR_SET_PDEATHSIG via prctl ensures v4l2-ctl child dies with parent
#     on ANY signal including SIGKILL — fixes the orphan-holding-/dev/video_usb
#     pattern that made restarts a fight
# Port 5001 (bridge stays on 5000). Field validation (battery + outdoor +
# motors-running) still pending — that's the failure environment we're
# really proving against.
scp perception/camera_owner.py ${ROVER_USER}@${ROVER}:/home/jetson/camera_owner.py
ssh ${ROVER_USER}@${ROVER} "docker cp /home/jetson/camera_owner.py ${CONTAINER}:/tmp/camera_owner.py"

echo "==> Killing any existing camera_owner / gst_camera_node and starting fresh..."
ssh ${ROVER_USER}@${ROVER} "docker exec ${CONTAINER} bash -c 'pkill -9 -f /tmp/camera_owner.py 2>/dev/null; pkill -9 -f /tmp/gst_camera_node.py 2>/dev/null; pkill -9 v4l2-ctl 2>/dev/null; sleep 1'"
# Note: log uses >> (append) not > (truncate) — preserves stall/restart history
# across watchdog respawns. We learned this the hard way 2026-05-10 trying to
# diagnose a multi-hour stall and finding the log had been wiped on restart.
ssh ${ROVER_USER}@${ROVER} "docker exec -d ${CONTAINER} bash -c '${UNSET_PROXY}source /opt/ros/humble/setup.bash && source /home/ws/ugv_ws/install/setup.bash && python3 /tmp/camera_owner.py --publish-ros >> /tmp/camera_owner.log 2>&1'"
sleep 5

echo "==> Deploying ClaudeBot operator console (web/ClaudeBot/)..."
# React + Babel-standalone app served by the bridge as static files at
# http://rover:5000/. Phone-as-controller surface — replaces laptop in
# the crook of the arm for outings. See web/ClaudeBot/ for sources.
rsync -a web/ClaudeBot/ ${ROVER_USER}@${ROVER}:/home/jetson/web_claudebot/
ssh ${ROVER_USER}@${ROVER} "docker cp /home/jetson/web_claudebot/. ${CONTAINER}:/tmp/web/ClaudeBot/"

echo "==> Deploying latest bridge..."
scp bridge/ros2_bridge.py ${ROVER_USER}@${ROVER}:/home/jetson/ros2_bridge.py
ssh ${ROVER_USER}@${ROVER} "docker cp /home/jetson/ros2_bridge.py ${CONTAINER}:/tmp/ros2_bridge.py"

echo "==> Killing any existing bridge and waiting for port to free..."
ssh ${ROVER_USER}@${ROVER} "docker exec ${CONTAINER} bash -c 'pkill -9 -f ros2_bridge.py 2>/dev/null; for i in 1 2 3 4 5; do nc -z localhost 5000 2>/dev/null || break; sleep 1; done'"

echo "==> Ensuring bridge deps in container (waitress for /stream, requests for /camera and /control proxies)..."
ssh ${ROVER_USER}@${ROVER} "docker exec ${CONTAINER} bash -c '${UNSET_PROXY}python3 -m pip install -q waitress requests'"

echo "==> Starting bridge..."
ssh ${ROVER_USER}@${ROVER} "docker exec -d ${CONTAINER} bash -c '${UNSET_PROXY}source /opt/ros/humble/setup.bash && source /home/ws/ugv_ws/install/setup.bash && python3 /tmp/ros2_bridge.py >> /tmp/bridge.log 2>&1'"

echo "==> Waiting for bridge..."
sleep 4

echo "==> Deploying control_panel (host-side, manages systemd-user heartbeat service + bring-up)..."
# Runs on the host (NOT in the container) because it manages the
# container itself + systemctl --user services. Port 5060. Bridge
# proxies /control/* through to it so the operator console only ever
# talks to bridge:5000 — single Tailscale path, no CORS wrangling.
scp control_panel.py ${ROVER_USER}@${ROVER}:/home/jetson/control_panel.py
ssh ${ROVER_USER}@${ROVER} "python3 -m pip install --user -q flask waitress 2>/dev/null"
ssh ${ROVER_USER}@${ROVER} "pkill -9 -f /home/jetson/control_panel.py 2>/dev/null; sleep 1"
ssh ${ROVER_USER}@${ROVER} "nohup python3 /home/jetson/control_panel.py >> /tmp/control_panel.log 2>&1 &"
sleep 2

echo "==> Deploying yolo_detector..."
scp perception/yolo_detector.py ${ROVER_USER}@${ROVER}:/home/jetson/yolo_detector.py

echo "==> Starting yolo_detector (with proxy env unset — Waveshare landmine)..."
ssh ${ROVER_USER}@${ROVER} "docker exec -d ${CONTAINER} bash -c 'unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY no_proxy NO_PROXY && source /opt/ros/humble/setup.bash && source /home/ws/ugv_ws/install/setup.bash && python3 /home/ws/yolo_detector.py > /tmp/yolo.log 2>&1'"

echo "==> Waiting for yolo warmup (CUDA kernel compilation)..."
sleep 15

echo "==> Deploying lidar_safety node..."
scp perception/lidar_safety.py ${ROVER_USER}@${ROVER}:/home/jetson/lidar_safety.py
ssh ${ROVER_USER}@${ROVER} "docker cp /home/jetson/lidar_safety.py ${CONTAINER}:/tmp/lidar_safety.py"

echo "==> Killing existing lidar_safety..."
ssh ${ROVER_USER}@${ROVER} "docker exec ${CONTAINER} bash -c 'pkill -9 -f lidar_safety.py 2>/dev/null; rm -f /tmp/lidar_safety.pid; sleep 1'"

echo "==> Starting lidar_safety in ROS2 container..."
ssh ${ROVER_USER}@${ROVER} "docker exec -d ${CONTAINER} bash -c 'source /opt/ros/humble/setup.bash && source /home/ws/ugv_ws/install/setup.bash && python3 /tmp/lidar_safety.py > /tmp/lidar_safety.log 2>&1'"

echo "==> Deploying intent_executor (10Hz intent stack on rover, replaces heartbeat-driven 1Hz tick)..."
# The executor owns the DualStack and ticks it at 10Hz natively, publishing
# cmd_vel direct to ROS. Heartbeat (Mac-side) just proxies push/pop/status
# via HTTP on :5050. This removes the start/stop motor pattern caused by
# the 500ms pwm_driver deadman timing out between 1Hz heartbeat ticks.
# Needs the groundctl Python package alongside it for intent imports.
scp intent/intent_executor.py ${ROVER_USER}@${ROVER}:/home/jetson/intent_executor.py
rsync -a --exclude '__pycache__' intent/ ${ROVER_USER}@${ROVER}:/home/jetson/intent/
ssh ${ROVER_USER}@${ROVER} "docker cp /home/jetson/intent_executor.py ${CONTAINER}:/tmp/intent_executor.py"
ssh ${ROVER_USER}@${ROVER} "docker cp /home/jetson/intent ${CONTAINER}:/tmp/intent"

echo "==> Killing existing intent_executor..."
ssh ${ROVER_USER}@${ROVER} "docker exec ${CONTAINER} bash -c 'pkill -9 -f intent_executor.py 2>/dev/null; rm -f /tmp/intent_executor.pid; sleep 1'"

echo "==> Starting intent_executor in ROS2 container..."
# cd /tmp so Python finds the groundctl package via CWD on its sys.path.
ssh ${ROVER_USER}@${ROVER} "docker exec -d ${CONTAINER} bash -c 'cd /tmp && source /opt/ros/humble/setup.bash && source /home/ws/ugv_ws/install/setup.bash && python3 /tmp/intent_executor.py > /tmp/intent_executor.log 2>&1'"

echo "==> Deploying GPS launch file..."
# Combined launch: ntrip_client (AUSCORS over TLS) + ublox_dgnss (F9R via libusb) +
# nav_sat_fix_hp (HPPOSLLH → /fix). The launch file loads AUSCORS_* creds from
# /home/ws/.groundctl.env (host's /home/jetson/.groundctl.env, bind-mounted) and
# uses ParameterValue(value_type=str) to bypass the YAML param parser — without
# that, NTRIP passwords with ':', '#', '@' etc. fail to load.
#
# Apt-installed packages: ros-humble-ublox-dgnss + ros-humble-ublox-nav-sat-fix-hp-node.
# Udev rule (etc/udev/99-rover.rules) maps the F9R to /dev/zedf9r, but the
# dgnss driver actually talks via libusb so the symlink is for human convenience.
scp etc/groundctl_gps.launch.py ${ROVER_USER}@${ROVER}:/home/jetson/groundctl_gps.launch.py
ssh ${ROVER_USER}@${ROVER} "docker cp /home/jetson/groundctl_gps.launch.py ${CONTAINER}:/tmp/groundctl_gps.launch.py"

echo "==> Killing any existing GPS stack..."
# Match by container name (set in groundctl_gps.launch.py) rather than 'ublox'
# or 'component_container' — the latter would also kill other composable
# containers like bringup_lidar's, which we don't want.
ssh ${ROVER_USER}@${ROVER} "docker exec ${CONTAINER} bash -c 'pkill -9 -f ntrip_client_container 2>/dev/null; pkill -9 -f ublox_dgnss_container 2>/dev/null; pkill -9 -f ublox_nav_sat_fix_hp_container 2>/dev/null; sleep 1'"

echo "==> Starting GPS stack (NTRIP + ublox_dgnss + NavSatFix HP)..."
ssh ${ROVER_USER}@${ROVER} "docker exec -d ${CONTAINER} bash -c '${UNSET_PROXY}set -a; source /home/ws/.groundctl.env; set +a; source /opt/ros/humble/setup.bash; ros2 launch /tmp/groundctl_gps.launch.py > /tmp/gps.log 2>&1'"
sleep 4

# depth_safety disabled: OAK-D monocular depth is the wrong sensor for cliff
# detection. Two validated failure modes:
#   1) False positives: fisheye + ground-level + tall grass reads as a convex
#      horizon / drop-off (documented in memory b873bf70 — the "scared
#      heartbeat-instance" field incident, 20 Apr).
#   2) False negatives: at very short range (e.g. chassis on the edge of a
#      desk / smoker), the OAK-D depth pipeline can't see the edge and
#      reports "no drop-off" when the rover is literally about to fall.
# The architecture's five-layer safety stack (see CLAUDE.md) always planned
# for a VL53L1X ToF sensor mounted 45° forward-down, wired directly to the
# ESP32 for sub-50ms hardware-level cliff reflex. Re-enable once that's
# fitted. Until then, LiDAR + twist_mux + ESP32 watchdog carry the load at
# body-level, and Haiku's heartbeat handles the rest.

echo "==> Restarting OAK-D spatial detection daemon (person tracking w/ 3D pos)..."
# Runs on host (USB access to the OAK-D is host-only). Same systemd-user
# pattern as listener_daemon — survives SSH disconnects thanks to linger,
# auto-restarts on failure, journalctl --user -u oakd_spatial -f for logs.
#
# First-run note: if ~/.cache/depthai doesn't already have the mobilenet
# blob, the pipeline start-up will download it (~20 MB). Service has
# TimeoutStartSec=60 to cover this.
scp perception/oakd_spatial.py ${ROVER_USER}@${ROVER}:/home/jetson/oakd_spatial.py
scp etc/oakd_spatial.service ${ROVER_USER}@${ROVER}:/home/jetson/.config/systemd/user/oakd_spatial.service
ssh ${ROVER_USER}@${ROVER} "systemctl --user daemon-reload && systemctl --user enable --now oakd_spatial && sleep 2 && systemctl --user is-active oakd_spatial"

# Best-effort pulse default source. The listener uses PULSE_SOURCE env var
# to be independent of this (default source has been observed to revert to
# onboard audio when pipewire auto-switches), but other pulse consumers on
# the rover benefit from a sensible default.
# Filter: alsa_input.*USB_PnP_Audio_Device — the .monitor loopback sorts
# first in `pactl list short sources` and would win a bare grep/awk, making
# a consumer hear speaker playback instead of the mic.
echo "==> Setting pulse default source to USB PnP mic (best-effort)..."
ssh ${ROVER_USER}@${ROVER} bash <<'EOSSH'
    SOURCE=$(pactl list short sources 2>/dev/null | grep -i "alsa_input.*USB_PnP_Audio_Device" | awk '{print $2}' | head -1)
    if [ -n "$SOURCE" ]; then
        pactl set-default-source "$SOURCE" 2>/dev/null
        echo "USB mic found: $SOURCE"
    else
        echo "WARN: USB mic not detected — voice listener will not have input"
    fi
    pactl info 2>/dev/null | grep "Default Source"
EOSSH


if [ "$BENCH_MODE" = "true" ]; then
    echo "==> Bench mode: stopping any running listener_daemon to prevent silent Deepgram billing..."
    ssh ${ROVER_USER}@${ROVER} "systemctl --user stop listener_daemon 2>/dev/null; systemctl --user is-active listener_daemon || echo listener_daemon stopped"
else
    echo "==> Restarting voice listener daemon (Deepgram Nova-2 streaming)..."
    # The listener runs as a systemd --user service (see etc/listener_daemon.service).
    # It survives SSH disconnects because user linger is enabled and is self-healing
    # via Restart=on-failure. The service unit handles env loading, PULSE_SOURCE
    # pinning, and journalctl logging — no need for setsid/nohup/PID-file gymnastics.
    #
    # Prereq done once per rover lifetime:
    #   sudo loginctl enable-linger jetson
    #   mkdir -p ~/.config/systemd/user
    #   scp etc/listener_daemon.service rover:~/.config/systemd/user/
    #   ssh rover "systemctl --user daemon-reload && systemctl --user enable listener_daemon"
    if ! ssh ${ROVER_USER}@${ROVER} "test -f /home/jetson/.groundctl.env"; then
        echo "WARN: /home/jetson/.groundctl.env missing on rover — listener will fail with"
        echo "      'DEEPGRAM_API_KEY not set'. scp your local .env there first:"
        echo "      scp .env rover:/home/jetson/.groundctl.env && ssh rover chmod 600 /home/jetson/.groundctl.env"
    fi
    scp listener_daemon.py ${ROVER_USER}@${ROVER}:/home/jetson/listener_daemon.py
    scp etc/listener_daemon.service ${ROVER_USER}@${ROVER}:/home/jetson/.config/systemd/user/listener_daemon.service
    ssh ${ROVER_USER}@${ROVER} "systemctl --user daemon-reload && systemctl --user restart listener_daemon"
    sleep 2
    ssh ${ROVER_USER}@${ROVER} "systemctl --user is-active listener_daemon"
fi

echo "==> Verifying..."
sleep 2
RESPONSE=$(curl -s --max-time 5 "http://${ROVER}:5000/state")
if [ -z "$RESPONSE" ]; then
    echo "ERROR: bridge not responding"
    exit 1
fi

echo ""
echo "Bridge response:"
echo "$RESPONSE" | python3 -m json.tool 2>/dev/null || echo "$RESPONSE"
echo ""
echo "Topics:"
ssh ${ROVER_USER}@${ROVER} "docker exec ${CONTAINER} bash -c 'source /opt/ros/humble/setup.bash && ros2 topic list'"
echo ""
echo "Done. Heartbeat ready: python heartbeat.py"
