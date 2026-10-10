#!/usr/bin/env bash
set -euo pipefail
astra_px4_dir="${ASTRA_PX4_DIR:-$HOME/dev/px4/PX4-Autopilot-1.17}"
astra_python_env="${ASTRA_PX4_PYTHON_ENV:-$HOME/dev/px4/venv}"
if [[ "$(git -C "$astra_px4_dir" rev-parse HEAD)" != d6f12ad1c4f70ad3230afd7d86e971421e02fef4 ]]; then
    echo 'Expected the pinned PX4 v1.17.0 commit; check astra/dependencies.lock.json.' >&2
    exit 1
fi
source "$astra_python_env/bin/activate"
cd "$astra_px4_dir"
export CMAKE_BUILD_PARALLEL_LEVEL=2
make px4_sitl_default -j2
