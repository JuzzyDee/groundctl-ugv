#!/bin/bash
# install_udev.sh — install the rover's udev rules on the Jetson host.
#
# One-time install: creates stable /dev/lidar and /dev/video_usb symlinks
# that survive reboots, so ROS launch configs don't hardcode enumeration-
# dependent paths.
#
# Run from your Mac with the Jetson reachable over Tailscale:
#   bash migration/install_udev.sh
#
# After install, re-plug USB devices (or reboot) for the rules to take effect.

set -euo pipefail

ROVER=${ROVER:-100.102.73.83}
ROVER_USER=${ROVER_USER:-jetson}

RULES_SRC="$(dirname "$0")/../etc/udev/99-rover.rules"
RULES_DST="/etc/udev/rules.d/99-rover.rules"

if [ ! -f "$RULES_SRC" ]; then
    echo "ERROR: can't find $RULES_SRC"
    exit 1
fi

echo "==> Copying rules file to Jetson..."
scp "$RULES_SRC" "${ROVER_USER}@${ROVER}:/tmp/99-rover.rules"

echo "==> Installing to /etc/udev/rules.d/ (sudo — you'll be prompted for your password once)..."
# -t forces a TTY so sudo can prompt. All sudos in one SSH call so the
# password cache stays warm across them.
ssh -t "${ROVER_USER}@${ROVER}" "sudo bash -c '
    mv /tmp/99-rover.rules ${RULES_DST} &&
    chown root:root ${RULES_DST} &&
    chmod 644 ${RULES_DST} &&
    udevadm control --reload-rules &&
    udevadm trigger
'"

echo "==> Verifying symlinks exist..."
ssh "${ROVER_USER}@${ROVER}" "ls -la /dev/lidar /dev/zedf9r /dev/video_usb 2>&1 || echo 'symlinks not yet — re-plug devices or reboot'"

echo ""
echo "Done. Future reboots will create /dev/lidar, /dev/zedf9r, and /dev/video_usb automatically."
echo "If the symlinks didn't appear, unplug+replug the USB devices."
