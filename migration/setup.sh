#!/bin/bash
# setup.sh — one-shot install script for the new ugv_jp6 container.
#
# Run inside the new container (based on nvcr.io/nvidia/l4t-jetpack:r36.2.0)
# as root. Installs ROS2 Humble + all Waveshare UGV deps + torch (with CUDA) +
# ultralytics, then builds the ugv_ws workspace.
#
# Bind mount at /home/ws -> host /home/jetson means the ugv_ws source is
# already present; we just need to install deps and colcon build.
#
# Prerequisites before running this script:
#   - Container launched with the docker run spec in migration/launch.sh
#   - Torch wheel staged at /tmp/torch-2.3.0-cp310-cp310-linux_aarch64.whl
#     (or downloaded by this script if TORCH_WHEEL_URL is reachable)
#
# Usage:
#   docker exec -it ugv_jp6 /bin/bash
#   /home/ws/migration/setup.sh
#
# The script is broken into stages — each stage is idempotent-ish, so if one
# fails you can usually re-run from the start without damage.

set -euo pipefail

echo "=========================================="
echo "  ugv_jp6 setup"
echo "=========================================="

# -----------------------------------------------------------------------------
# Stage 0 — sanity
# -----------------------------------------------------------------------------

if [ "$(id -u)" != "0" ]; then
    echo "ERROR: this script must run as root inside the container."
    exit 1
fi

if [ ! -d /home/ws/ugv_ws/src/ugv_main ]; then
    echo "ERROR: /home/ws/ugv_ws/src/ugv_main not found — is the bind mount working?"
    exit 1
fi

# Strip any lingering Waveshare proxy vars (should be clean from JP6 base, but safe).
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY no_proxy NO_PROXY

export DEBIAN_FRONTEND=noninteractive

# -----------------------------------------------------------------------------
# Stage 1 — base system packages
# -----------------------------------------------------------------------------

echo ""
echo "[1/7] Base system packages..."

apt update
apt install -y --no-install-recommends \
    ca-certificates \
    curl \
    wget \
    git \
    gnupg \
    lsb-release \
    software-properties-common \
    build-essential \
    cmake \
    pkg-config \
    python3 \
    python3-pip \
    python3-dev \
    nano \
    ffmpeg \
    v4l-utils \
    libopenblas0 \
    libopenblas-dev \
    libomp-dev

# -----------------------------------------------------------------------------
# Stage 2 — ROS 2 Humble apt source + key
# -----------------------------------------------------------------------------

echo ""
echo "[2/7] ROS 2 Humble apt source..."

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list

apt update

# -----------------------------------------------------------------------------
# Stage 3 — ROS 2 Humble + build tools + extras we need
# -----------------------------------------------------------------------------

echo ""
echo "[3/7] ROS 2 Humble + extras..."

apt install -y \
    ros-humble-desktop-full \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-v4l2-camera \
    ros-humble-usb-cam \
    ros-humble-image-pipeline \
    ros-humble-vision-msgs \
    ros-humble-cv-bridge \
    ros-humble-rosbridge-suite \
    ros-humble-cartographer-ros \
    ros-humble-depthai-ros-driver \
    ros-humble-rtabmap-ros \
    ros-humble-robot-localization \
    ros-humble-tf2-tools \
    ros-humble-joint-state-publisher-gui \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-rosinstall-generator

# Initialise rosdep (may already be init'd; ignore failure)
rosdep init 2>/dev/null || true
rosdep update

# -----------------------------------------------------------------------------
# Stage 4 — workspace-specific deps via rosdep
# -----------------------------------------------------------------------------

echo ""
echo "[4/7] Resolving workspace deps with rosdep..."

cd /home/ws/ugv_ws
# rosdep failures are worth surfacing — if a system dep can't be resolved,
# colcon build will fail downstream anyway. Let the install fail loudly so
# we see the root cause at this stage rather than later in a 10-minute build.
rosdep install --from-paths src --ignore-src -r -y

# -----------------------------------------------------------------------------
# Stage 5 — Python deps (mediapipe, opencv, Flask, etc.)
# -----------------------------------------------------------------------------

echo ""
echo "[5/7] Python deps..."

pip install --no-cache-dir \
    Flask \
    pyserial \
    pygame \
    opencv-contrib-python \
    mediapipe \
    pybind11 \
    "numpy<2" \
    scipy

# -----------------------------------------------------------------------------
# Stage 6 — torch (CUDA 12.2 Jetson build) + torchvision + ultralytics
# -----------------------------------------------------------------------------

echo ""
echo "[6/7] Torch + ultralytics..."

TORCH_WHEEL=/tmp/torch-2.3.0-cp310-cp310-linux_aarch64.whl
TORCH_WHEEL_URL="https://nvidia.box.com/shared/static/mp164asf3sceb570wvjsrezk1p4ftj8t.whl"

if [ ! -f "$TORCH_WHEEL" ]; then
    echo "Torch wheel not present, downloading..."
    curl -L -o "$TORCH_WHEEL" "$TORCH_WHEEL_URL"
    # Basic sanity check — should be a valid zip archive
    file "$TORCH_WHEEL" | grep -q "Zip archive data" || {
        echo "ERROR: downloaded torch wheel is not a valid zip — check the URL."
        exit 1
    }
fi

pip install --no-cache-dir "$TORCH_WHEEL"

# torchvision: build from source against our installed Jetson torch so the
# native ops (NMS, etc) register correctly with torch's dispatcher.
# PyPI's aarch64 torchvision wheel is compiled against a different torch ABI
# and fails silently at op registration time, breaking ultralytics.
# --no-build-isolation forces use of the installed torch during compilation.
# Compile takes ~20-30 min on the Orin Nano.
apt install -y ninja-build
# torchvision source isn't on PyPI (no sdist, only wheels) so we clone from
# GitHub at the matching tag and pip-install from the checkout.
# --no-build-isolation uses our installed Jetson torch during compilation.
TORCHVISION_SRC=/tmp/torchvision-src
rm -rf "$TORCHVISION_SRC"
git clone --depth 1 --branch v0.18.0 https://github.com/pytorch/vision.git "$TORCHVISION_SRC"
# MAX_JOBS=1 forces sequential compile. Torch's headers are enormous; parallel
# compile of torchvision extensions OOMs the 8GB Orin Nano. Slower but finishes.
(cd "$TORCHVISION_SRC" && MAX_JOBS=1 pip install --no-build-isolation --no-cache-dir --no-deps .)

# Install ultralytics WITHOUT letting it pull a CPU-only torch from pypi.
# --no-deps skips all its deps; we install them explicitly above or rely on apt.
pip install --no-cache-dir ultralytics --no-deps

# Ultralytics needs: opencv (have it), numpy (have it), Pillow, matplotlib,
# psutil, pandas, pyyaml, requests, scipy (have it), seaborn, tqdm.
pip install --no-cache-dir \
    Pillow \
    matplotlib \
    psutil \
    pandas \
    pyyaml \
    requests \
    seaborn \
    tqdm

# -----------------------------------------------------------------------------
# Stage 7 — colcon build the ugv workspace
# -----------------------------------------------------------------------------

echo ""
echo "[7/7] colcon build ugv_ws..."

cd /home/ws/ugv_ws

# Clear any stale build artefacts from the old container (compiled against CUDA 11.8).
rm -rf build/ install/ log/

# ROS2's setup.bash references variables it expects to be optional, which trips
# bash's nounset (-u). Relax -u for the source, then put it back.
set +u
source /opt/ros/humble/setup.bash
set -u

# Orin Nano has 8GB RAM. Default colcon parallelism fires one gcc per core;
# compiling large C++ templates (rf2o_laser_odometry, tf2 etc) OOMs the kernel.
# Sequential build is slower but deterministic. Parallel at the package level
# still happens within colcon's --parallel-workers bound.
#
# Skipped packages (fix before nav2 local-planning work):
# - costmap_converter: BlobDetector is abstract against OpenCV 4.8 (missing
#   setParams/getParams overrides). Fix = add two override stubs or pull
#   upstream master which has this resolved.
# - teb_local_planner: depends on costmap_converter. Unblocks once the above
#   is fixed.
MAKEFLAGS='-j1' colcon build --symlink-install --parallel-workers 1 \
    --packages-skip costmap_converter teb_local_planner

# -----------------------------------------------------------------------------
# Done
# -----------------------------------------------------------------------------

echo ""
echo "=========================================="
echo "  Setup complete."
echo ""
echo "Verify:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source /home/ws/ugv_ws/install/setup.bash"
echo "  python3 -c 'import torch; print(torch.cuda.is_available())'  # should be True"
echo "  python3 -c 'import ultralytics; print(ultralytics.__version__)'"
echo ""
echo "Then bring up the stack the usual way:"
echo "  ros2 launch ugv_bringup bringup_lidar.launch.py use_rviz:=false"
echo "=========================================="
