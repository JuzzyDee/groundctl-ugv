#!/bin/bash
# launch.sh — start the new ugv_jp6 container on a clean JP6 foundation.
#
# Derived from the old container's docker inspect (migration/old_container_spec.json):
#   - same bind mounts (critical: /home/jetson:/home/ws)
#   - same --privileged, --network host, --runtime nvidia, --gpus all
#   - same device cgroup rules (full /dev passthrough)
#   - restart policy: always (survives reboots)
#
# Dropped from the old spec:
#   - Waveshare's HTTP/HTTPS proxy env (the 192.168.10.185:10809 phantom)
#   - X11 display env (headless operation)
#   - CUDA 11.8 version pins (JP6 image has CUDA 12.2 baked in)
#
# Run from the Jetson host. Exits cleanly if the container is already running.
#
# Usage:
#   ssh jetson@100.102.73.83
#   bash ~/launch.sh              # via scp'd copy, or run the docker command directly

set -euo pipefail

IMAGE="nvcr.io/nvidia/l4t-jetpack:r36.2.0"
NAME="ugv_jp6"

# Sanity: does the image exist locally?
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "ERROR: $IMAGE not found locally. Pull it first:"
    echo "  docker pull $IMAGE"
    exit 1
fi

# If a container of this name is already running, don't clobber it.
if docker ps -a --format '{{.Names}}' | grep -q "^${NAME}$"; then
    echo "Container $NAME already exists. Start/attach with:"
    echo "  docker start $NAME"
    echo "  docker exec -it $NAME bash"
    exit 0
fi

docker run -d \
    --name "$NAME" \
    --runtime nvidia \
    --privileged \
    --network host \
    --gpus all \
    --restart always \
    --security-opt label=disable \
    --shm-size=64m \
    -v /home/jetson:/home/ws \
    -v /dev:/dev \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -w /home/ws \
    -it \
    "$IMAGE" \
    /bin/bash

echo ""
echo "Container $NAME launched. Exec in with:"
echo "  docker exec -it $NAME bash"
echo ""
echo "Then run the setup script:"
echo "  /home/ws/migration/setup.sh"
