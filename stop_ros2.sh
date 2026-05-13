#!/bin/bash
# stop_ros2.sh — Spin down the ROS2 stack and let the LiDAR rest.
#
# Use when done testing for the day. The LD19 LiDAR has a finite lifespan
# (~5000-10000 hours of spinning) — don't leave it running idle.
#
# Run from your Mac:
#   ./stop_ros2.sh

[ -f .env ] && set -a && source .env && set +a
ROVER="${ROVER:?ROVER not set. See .env.example.}"
ROVER_USER=${ROVER_USER:-jetson}
CONTAINER=ugv_jetson_ros_humble

echo "==> Killing depth_safety daemon..."
ssh ${ROVER_USER}@${ROVER} "pkill -9 -f depth_safety; rm -f /tmp/depth_safety.pid /tmp/depth_safety_status.json"

echo "==> Killing listener daemon..."
ssh ${ROVER_USER}@${ROVER} "pkill -9 -f listener_daemon; rm -f /tmp/listener_daemon.pid"

echo "==> Killing ROS2 nodes (LiDAR will spin down)..."
ssh ${ROVER_USER}@${ROVER} "docker exec ${CONTAINER} bash -c 'pkill -9 -f \"ros2 launch\"; pkill -9 -f ugv_driver; pkill -9 -f ugv_bringup; pkill -9 -f base_node; pkill -9 -f LD19; pkill -9 -f rf2o; pkill -9 -f ros2_bridge; pkill -9 -f lidar_safety; pkill -9 -f robot_state_publisher; pkill -9 -f joint_state_publisher; sleep 2'"

echo "==> Verifying stopped..."
sleep 3
ssh ${ROVER_USER}@${ROVER} "docker exec ${CONTAINER} bash -c 'pgrep -fa \"ros2|ugv|LD19|base_node|rf2o\" | grep -v daemon || echo all_clean'"

echo ""
echo "Done. ROS2 stack down, LiDAR resting."
echo "To restart: ./start_ros2.sh"
