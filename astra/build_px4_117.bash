#!/usr/bin/env bash
set -euo pipefail
astra_repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
astra_px4_dir="${ASTRA_PX4_DIR:-$HOME/dev/px4/PX4-Autopilot-1.17}"
astra_python_env="${ASTRA_PX4_PYTHON_ENV:-$HOME/dev/px4/venv}"
if [[ "$(git -C "$astra_px4_dir" rev-parse HEAD)" != d6f12ad1c4f70ad3230afd7d86e971421e02fef4 ]]; then
    echo 'Expected the pinned PX4 v1.17.0 commit; check astra/dependencies.lock.json.' >&2
    exit 1
fi
astra_hrt_patch="$astra_repo_dir/astra/baseline/px4_1_15_3_hrt.patch"
if git -C "$astra_px4_dir" apply --reverse --check "$astra_hrt_patch" 2>/dev/null; then
    echo 'Validated HRT rescheduling patch already applied.'
elif git -C "$astra_px4_dir" apply --check "$astra_hrt_patch"; then
    git -C "$astra_px4_dir" apply "$astra_hrt_patch"
else
    echo 'HRT sources differ; inspect local changes before building.' >&2
    exit 1
fi
astra_mag_patch="$astra_repo_dir/astra/config/px4_117_gz_magnetometer.patch"
if git -C "$astra_px4_dir" apply --reverse --check "$astra_mag_patch" 2>/dev/null; then
    echo 'Astra optional ENU/tesla magnetometer path already applied.'
elif git -C "$astra_px4_dir" apply --check "$astra_mag_patch"; then
    git -C "$astra_px4_dir" apply "$astra_mag_patch"
else
    echo 'Gazebo bridge sources differ; inspect before building.' >&2
    exit 1
fi
astra_dds_patch="$astra_repo_dir/astra/config/px4_117_dds_local_position.patch"
if git -C "$astra_px4_dir" apply --reverse --check "$astra_dds_patch" 2>/dev/null; then
    echo 'Astra 100 Hz local-position DDS output already applied.'
elif git -C "$astra_px4_dir" apply --check "$astra_dds_patch"; then
    git -C "$astra_px4_dir" apply "$astra_dds_patch"
else
    echo 'DDS topic configuration differs; inspect before building.' >&2
    exit 1
fi
source "$astra_python_env/bin/activate"
cd "$astra_px4_dir"
export CMAKE_BUILD_PARALLEL_LEVEL=2
make px4_sitl_default -j2
