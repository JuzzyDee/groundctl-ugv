#!/bin/bash
# start_ros2_local.sh — Full-stack bring-up, executed ON the rover.
#
# Mirror of start_ros2.sh with ssh/scp wrappers stripped. Assumes source
# files have already been deployed to /home/jetson/X.py — that's what
# the Mac-side start_ros2.sh's rsync step does, OR a manual scp ahead of
# running this on first install.
#
# Triggered three ways:
#   1) On boot via systemd-user (etc/ros2_init.service) — autonomous bring-up
#      when you power the rover at the duck pond, no laptop needed.
#   2) From the operator-console "Restart ROS2" button (control_panel:5060).
#   3) Manually: `bash ~/start_ros2_local.sh` over SSH if you're debugging.
#
# Ends with a Deepgram-generated "ROS2 initiated successfully, awaiting
# heartbeat" played through the USB PnP speaker — audio confirmation
# without needing to look at a screen. First run generates the WAV;
# subsequent runs replay the cached file (instant, no API call).
#
# Step ordering / why each — see start_ros2.sh comments (kept identical).

set +e  # don't bail mid-bringup if one component fails — others should still come up

# Defensive pulse env. The trailing paplay step needs PULSE_SERVER set, or
# it falls through to ALSA default = HDMI = silent. ros2_init.service +
# control_panel.service both set these explicitly, but ensure them here in
# case the script is invoked from anywhere else (manual SSH from a session
# without these inherited, future cron, etc).
: "${XDG_RUNTIME_DIR:=/run/user/$(id -u)}"
: "${PULSE_SERVER:=unix:${XDG_RUNTIME_DIR}/pulse/native}"
export XDG_RUNTIME_DIR PULSE_SERVER

CONTAINER=${CONTAINER:-ugv_jp6}
OLD_CONTAINER=ugv_jetson_ros_humble
HOST_HOME=${HOST_HOME:-/home/jetson}
SOUND_DIR=$HOST_HOME/.groundctl/sounds
READY_WAV=$SOUND_DIR/ros2_ready.wav

# --bench-mode: skip listener_daemon (Deepgram Nova-2 streaming STT is
# ~$0.46/hr — no point during bench testing where nothing's going to Claude).
BENCH_MODE=false
for arg in "$@"; do
    case "$arg" in
        --bench-mode) BENCH_MODE=true ;;
    esac
done

if [ "$BENCH_MODE" = "true" ]; then
    echo "==> BENCH MODE — listener_daemon will be skipped (and stopped if running)"
fi

# Waveshare image leaves stale HTTP_PROXY env vars that break pip/ultralytics.
UNSET_PROXY='unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY all_proxy ALL_PROXY no_proxy NO_PROXY 2>/dev/null; '

echo "==> Stopping old container (${OLD_CONTAINER}) so it doesn't fight for /dev..."
docker stop ${OLD_CONTAINER} 2>/dev/null || echo already stopped

echo "==> Restarting ${CONTAINER} for a clean process slate..."
docker restart ${CONTAINER}
sleep 8

echo "==> Killing app.py..."
pkill -9 -f 'ugv_jetson/app.py' 2>/dev/null
sleep 2
pgrep -fa 'ugv_jetson/app.py' || echo app.py dead

echo "==> Cleaning up any old ROS2 nodes in container..."
docker exec ${CONTAINER} bash -c 'pkill -9 -f "ros2 launch" 2>/dev/null; pkill -9 -f ugv_driver 2>/dev/null; pkill -9 -f ugv_bringup 2>/dev/null; pkill -9 -f base_node 2>/dev/null; pkill -9 -f LD19 2>/dev/null; pkill -9 -f rf2o 2>/dev/null; pkill -9 -f ros2_bridge 2>/dev/null; pkill -9 -f v4l2_camera_node 2>/dev/null; pkill -9 -f usb_cam_node_exe 2>/dev/null; pkill -9 -f gst_camera_node 2>/dev/null; pkill -9 -f /tmp/camera_owner.py 2>/dev/null; pkill -9 v4l2-ctl 2>/dev/null; pkill -9 -f yolo_detector 2>/dev/null; sleep 2'

echo "==> Launching ROS2 bringup..."
docker exec -d ${CONTAINER} bash -c "${UNSET_PROXY}export UGV_MODEL=ugv_rover && export LDLIDAR_MODEL=ld19 && source /opt/ros/humble/setup.bash && source /home/ws/ugv_ws/install/setup.bash && ros2 launch ugv_bringup bringup_lidar.launch.py use_rviz:=false > /tmp/bringup.log 2>&1"

echo "==> Waiting for bringup..."
sleep 12

echo "==> Suppressing joint_state_publisher (it fights the gimbal servo)..."
docker exec ${CONTAINER} pkill -f joint_state_publisher 2>/dev/null
echo jsp killed

echo "==> Deploying camera_owner..."
docker cp $HOST_HOME/camera_owner.py ${CONTAINER}:/tmp/camera_owner.py

echo "==> Killing any existing camera_owner / gst_camera_node and starting fresh..."
docker exec ${CONTAINER} bash -c 'pkill -9 -f /tmp/camera_owner.py 2>/dev/null; pkill -9 -f /tmp/gst_camera_node.py 2>/dev/null; pkill -9 v4l2-ctl 2>/dev/null; sleep 1'
docker exec -d ${CONTAINER} bash -c "${UNSET_PROXY}source /opt/ros/humble/setup.bash && source /home/ws/ugv_ws/install/setup.bash && python3 /tmp/camera_owner.py --publish-ros >> /tmp/camera_owner.log 2>&1"
sleep 5

# Throttle the camera-native rate down to 2 Hz for non-YOLO consumers.
# YOLO subscribes to /image_raw/compressed at full rate (ByteTrack continuity);
# bridge subscribes to /image_raw/compressed_low for /snapshot. Reason:
# bridge subscriber callback at 27 Hz competes with Waitress threads for GIL,
# and /snapshot only needs to be fresher than the heartbeat cadence (12 s).
# 2 Hz is hygiene; nothing actually polls faster than 1 Hz on the read side.
echo "==> Starting topic_tools/throttle (image_raw/compressed -> _low @ 2 Hz)..."
docker exec ${CONTAINER} bash -c 'pkill -9 -f "topic_tools.*throttle.*image_raw" 2>/dev/null; sleep 1'
docker exec -d ${CONTAINER} bash -c "source /opt/ros/humble/setup.bash && \
  ros2 run topic_tools throttle messages \
    /image_raw/compressed 2.0 /image_raw/compressed_low \
    >> /tmp/image_throttle.log 2>&1"

echo "==> Deploying ClaudeBot operator console..."
docker exec ${CONTAINER} mkdir -p /tmp/web 2>/dev/null
docker cp $HOST_HOME/web_claudebot/. ${CONTAINER}:/tmp/web/ClaudeBot/

echo "==> Deploying latest bridge..."
docker cp $HOST_HOME/ros2_bridge.py ${CONTAINER}:/tmp/ros2_bridge.py

echo "==> Killing any existing bridge and waiting for port to free..."
docker exec ${CONTAINER} bash -c 'pkill -9 -f ros2_bridge.py 2>/dev/null; for i in 1 2 3 4 5; do nc -z localhost 5000 2>/dev/null || break; sleep 1; done'

echo "==> Ensuring bridge deps in container..."
docker exec ${CONTAINER} bash -c "${UNSET_PROXY}python3 -m pip install -q waitress requests"

echo "==> Starting bridge..."
docker exec -d ${CONTAINER} bash -c "${UNSET_PROXY}source /opt/ros/humble/setup.bash && source /home/ws/ugv_ws/install/setup.bash && python3 /tmp/ros2_bridge.py >> /tmp/bridge.log 2>&1"
sleep 4

echo "==> Restarting control_panel (systemd-user — survives SSH disconnects)..."
python3 -m pip install --user -q flask waitress 2>/dev/null
# Kill any orphan nohup-launched control_panel from the legacy path before
# the systemd unit took over. systemctl restart would otherwise leave
# them fighting on port 5060.
pkill -9 -f $HOST_HOME/control_panel.py 2>/dev/null
sleep 1
systemctl --user restart control_panel 2>/dev/null
sleep 2
systemctl --user is-active control_panel

echo "==> Starting yolo_detector..."
docker exec -d ${CONTAINER} bash -c 'unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY no_proxy NO_PROXY && source /opt/ros/humble/setup.bash && source /home/ws/ugv_ws/install/setup.bash && python3 /home/ws/yolo_detector.py > /tmp/yolo.log 2>&1'
echo "==> Waiting for yolo warmup..."
sleep 15

echo "==> Deploying lidar_safety..."
docker cp $HOST_HOME/lidar_safety.py ${CONTAINER}:/tmp/lidar_safety.py
docker exec ${CONTAINER} bash -c 'pkill -9 -f lidar_safety.py 2>/dev/null; rm -f /tmp/lidar_safety.pid; sleep 1'
docker exec -d ${CONTAINER} bash -c 'source /opt/ros/humble/setup.bash && source /home/ws/ugv_ws/install/setup.bash && python3 /tmp/lidar_safety.py >> /tmp/lidar_safety.log 2>&1'

echo "==> Deploying intent_executor..."
docker cp $HOST_HOME/intent_executor.py ${CONTAINER}:/tmp/intent_executor.py
docker cp $HOST_HOME/intent ${CONTAINER}:/tmp/intent
docker exec ${CONTAINER} bash -c 'pkill -9 -f intent_executor.py 2>/dev/null; rm -f /tmp/intent_executor.pid; sleep 1'
docker exec -d ${CONTAINER} bash -c 'cd /tmp && source /opt/ros/humble/setup.bash && source /home/ws/ugv_ws/install/setup.bash && python3 /tmp/intent_executor.py >> /tmp/intent_executor.log 2>&1'

echo "==> Deploying GPS launch file..."
docker cp $HOST_HOME/groundctl_gps.launch.py ${CONTAINER}:/tmp/groundctl_gps.launch.py
docker exec ${CONTAINER} bash -c 'pkill -9 -f ntrip_client_container 2>/dev/null; pkill -9 -f ublox_dgnss_container 2>/dev/null; pkill -9 -f ublox_nav_sat_fix_hp_container 2>/dev/null; sleep 1'

echo "==> Starting GPS stack..."
docker exec -d ${CONTAINER} bash -c "${UNSET_PROXY}set -a; source /home/ws/.groundctl.env; set +a; source /opt/ros/humble/setup.bash; ros2 launch /tmp/groundctl_gps.launch.py >> /tmp/gps.log 2>&1"
sleep 4

echo "==> Restarting OAK-D spatial detection daemon..."
systemctl --user restart oakd_spatial 2>/dev/null
sleep 2
systemctl --user is-active oakd_spatial

# Wait for the USB PnP audio card to appear in pulse, then disown it.
#
# Background: this device (JMTek 0c76:1229) advertises a USB descriptor
# that pulse parses inconsistently across boots — sometimes input-only,
# sometimes output-only, never duplex. Whichever direction pulse misses
# silently breaks (mic dies → listener crashes, OR speaker dies → paplay
# falls to HDMI). Same physical card, two failure modes, lottery on every
# boot. Field-validated 2026-05-11.
#
# Fix: force pulse to release the card by setting its profile to "off",
# then everything downstream (heartbeat exec_speak, start_ros2_local.sh
# ready announcement, listener_daemon) targets raw ALSA directly via
# `hw:CARD=Device`. Deterministic. No profile dependence.
#
# We still wait for the card to appear in pulse — once it has, we set
# profile=off, after which `pactl list short sinks/sources` returns
# nothing for USB PnP, and the downstream aplay/listener paths use the
# raw ALSA hw: device names.
echo "==> Waiting for USB PnP audio card in pulse..."
wait_for_pulse_usb_card() {
    local deadline=$(( $(date +%s) + ${1:-20} ))
    while [ $(date +%s) -lt $deadline ]; do
        pactl list short cards 2>/dev/null | grep -qi "usb-Solid_State_System.*USB_PnP_Audio_Device" && return 0
        sleep 1
    done
    return 1
}
if ! wait_for_pulse_usb_card 20; then
    echo "WARN: USB PnP card not in pulse after 20s — kicking pulse to re-probe..."
    pactl unload-module module-udev-detect 2>/dev/null
    pactl load-module module-udev-detect 2>/dev/null
    wait_for_pulse_usb_card 10
fi

USB_CARD=$(pactl list short cards 2>/dev/null | grep -i "usb-Solid_State_System" | awk '{print $2}' | head -1)
if [ -n "$USB_CARD" ]; then
    echo "==> Setting USB PnP pulse card profile to off (force raw-ALSA path for both directions)..."
    pactl set-card-profile "$USB_CARD" off 2>/dev/null
    echo "USB PnP card profile: off ($USB_CARD)"
else
    echo "WARN: USB PnP card not found in pulse — audio paths may fall through to ALSA defaults"
fi

echo "==> Setting pulse default source + sink to USB PnP (best-effort)..."
SOURCE=$(pactl list short sources 2>/dev/null | grep -i "alsa_input.*USB_PnP_Audio_Device" | awk '{print $2}' | head -1)
SINK=$(pactl list short sinks 2>/dev/null | grep -i "alsa_output.*USB_PnP_Audio_Device" | awk '{print $2}' | head -1)
if [ -n "$SOURCE" ]; then
    pactl set-default-source "$SOURCE" 2>/dev/null
    echo "USB mic found: $SOURCE"
else
    echo "WARN: USB mic not detected — voice listener will not have input"
fi
if [ -n "$SINK" ]; then
    pactl set-default-sink "$SINK" 2>/dev/null
    echo "USB speaker found: $SINK"
else
    echo "WARN: USB speaker not detected — ready-announcement will fall back to ALSA"
fi

if [ "$BENCH_MODE" = "true" ]; then
    echo "==> Bench mode: stopping listener_daemon..."
    systemctl --user stop listener_daemon 2>/dev/null
else
    echo "==> Restarting voice listener daemon (Deepgram Nova-2 streaming)..."
    if [ ! -f $HOST_HOME/.groundctl.env ]; then
        echo "WARN: $HOST_HOME/.groundctl.env missing — listener will fail with 'DEEPGRAM_API_KEY not set'"
    fi
    systemctl --user restart listener_daemon 2>/dev/null
    sleep 2
    systemctl --user is-active listener_daemon
fi

echo "==> Verifying..."
sleep 2
RESPONSE=$(curl -s --max-time 5 "http://localhost:5000/state")
if [ -z "$RESPONSE" ]; then
    echo "ERROR: bridge not responding"
    BRINGUP_OK=0
else
    echo ""
    echo "Bridge response received."
    BRINGUP_OK=1
fi

# === Audio confirmation ====================================================
# Generate the ready WAV once via Deepgram, cache it, replay on subsequent
# bring-ups. First boot needs network — after that it's instant. Auras the
# Hyperion voice that matches Claude's. Falls back silently if Deepgram is
# unreachable on first run (no .wav generated, no audio played, but bring-up
# has already succeeded — the message is icing on a cake that's already baked).
mkdir -p $SOUND_DIR
if [ "$BRINGUP_OK" = "1" ] && [ ! -f $READY_WAV ]; then
    echo "==> Generating ready-sound (one-time, cached after this)..."
    set -a; source $HOST_HOME/.groundctl.env 2>/dev/null; set +a
    if [ -n "$DEEPGRAM_API_KEY" ]; then
        curl -s -X POST 'https://api.deepgram.com/v1/speak?model=aura-2-hyperion-en&encoding=linear16&sample_rate=24000' \
            -H "Authorization: Token $DEEPGRAM_API_KEY" \
            -H 'Content-Type: application/json' \
            -d '{"text": "ROS2 initiated successfully. Awaiting heartbeat."}' \
            --output $READY_WAV
        if [ -s $READY_WAV ]; then
            sox $READY_WAV /tmp/ros2_ready_loud.wav gain -n && mv /tmp/ros2_ready_loud.wav $READY_WAV
        fi
    else
        echo "WARN: no DEEPGRAM_API_KEY — skipping audio confirmation"
    fi
fi

if [ "$BRINGUP_OK" = "1" ] && [ -s $READY_WAV ]; then
    echo "==> Playing ready announcement..."
    APLAY_LOG=/tmp/aplay_boot.log
    : > $APLAY_LOG
    SINK=$(pactl list short sinks 2>/dev/null | grep -i "alsa_output.*USB_PnP_Audio_Device" | awk '{print $2}' | head -1)
    play_ready_wav() {
        if [ -n "$SINK" ]; then
            echo "--- pulse playback: $SINK ---" >> $APLAY_LOG
            paplay --device="$SINK" "$READY_WAV" >> $APLAY_LOG 2>&1 && return 0
            echo "pulse playback failed; falling back to ALSA" >> $APLAY_LOG
        fi
        echo "--- alsa playback fallback ---" >> $APLAY_LOG
        aplay -L 2>&1 | grep -A 1 "CARD=Device" >> $APLAY_LOG
        aplay -D plughw:CARD=Device "$READY_WAV" >> $APLAY_LOG 2>&1
    }

    if [ -n "$SINK" ]; then
        echo "  using pulse sink: $SINK"
    else
        echo "  no pulse USB sink; using ALSA fallback"
    fi

    # If playback returned much faster than the WAV's 5s duration, something
    # ate it silently. Wait for USB/pulse/ALSA to settle, then retry once via
    # the same explicit Pulse sink if it exists, otherwise symbolic ALSA.
    APLAY_START=$(date +%s.%N)
    play_ready_wav
    APLAY_RC=$?
    APLAY_END=$(date +%s.%N)
    APLAY_DUR=$(awk "BEGIN{print $APLAY_END - $APLAY_START}")
    echo "rc=$APLAY_RC duration=${APLAY_DUR}s" >> $APLAY_LOG
    echo "  playback attempt: rc=$APLAY_RC dur=${APLAY_DUR}s"
    if awk "BEGIN{exit !($APLAY_DUR < 4)}"; then
        echo "  playback returned suspiciously fast — sleeping 3s and retrying"
        sleep 3
        echo "--- playback retry ---" >> $APLAY_LOG
        play_ready_wav
        APLAY_RC2=$?
        echo "  playback retry: rc=$APLAY_RC2"
    fi
    echo "  see $APLAY_LOG for full output"
fi

echo ""
echo "Done. Heartbeat available via control_panel (phone)."
