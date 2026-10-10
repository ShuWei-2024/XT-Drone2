#!/usr/bin/env bash
set -eo pipefail
astra_repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${AMENT_PREFIX_PATH:-}" == *astra_ros2_ws* || "${PYTHONPATH:-}" == *astra_ros2_ws* ]]; then
    echo 'Use a clean terminal; the 1.15 overlay is loaded.' >&2
    exit 1
fi
source /opt/ros/jazzy/setup.bash
set -u
cd "$astra_repo_dir"
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon --log-base log_117 build --build-base build_117 --install-base install_117 \
    --executor sequential --packages-up-to astra_px4_offboard xtd2_launch \
    xtd2_communication xtd2_gz_sim --cmake-args -DCMAKE_BUILD_TYPE=Release
