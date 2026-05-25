#!/usr/bin/env bash
# oak_ros_mode.sh — put the OAK-D on the native depthai_ros_driver ROS path
# (OAK-into-ROS migration), replacing the legacy host-side oakd_spatial daemon.
#
# Transitional: a reboot returns to legacy oakd_spatial automatically (still
# enabled), so re-run this to re-enter ROS-OAK mode. Graduates to a systemd
# service once the field confirms depth quality. Run ON the rover.
set -uo pipefail
CONTAINER=ugv_jp6
export XDG_RUNTIME_DIR="/run/user/$(id -u)"

echo "[1/3] stopping legacy oakd_spatial (releasing the camera)..."
systemctl --user stop oakd_spatial || true

echo "[2/3] launching depthai_ros_driver..."
# Kill any existing driver FIRST. The OAK can only be claimed by one process,
# so without this a re-run launches a second driver that loses the device-busy
# race and dies, leaving the stale one (and its old args/model) running.
docker exec "$CONTAINER" bash -c "pkill -9 -f camera.launch.py; pkill -9 -f oak_container; pkill -9 -f oak_state_publisher; sleep 3" || true
docker exec -d "$CONTAINER" bash -c "source /opt/ros/humble/setup.bash && source /home/ws/ugv_ws/install/setup.bash && ros2 launch depthai_ros_driver camera.launch.py parent_frame:=3d_camera_link camera_model:=OAK-D-LITE > /tmp/oak_ros.log 2>&1"
sleep 12

echo "[3/3] restarting bridge so it subscribes to the live driver..."
docker exec "$CONTAINER" bash -c "pkill -9 -f ros2_bridge.py 2>/dev/null; sleep 2" || true
docker exec -d "$CONTAINER" bash -c "source /opt/ros/humble/setup.bash && source /home/ws/ugv_ws/install/setup.bash && python3 /tmp/ros2_bridge.py >> /tmp/bridge.log 2>&1"
sleep 5

echo "done. spatial_detections check:"
docker exec "$CONTAINER" curl -s localhost:5000/spatial_detections
echo
