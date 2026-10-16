#!/usr/bin/env bash
# Prepared office4 SITL only. Explicit mission Start requests takeoff.
set -eo pipefail
astra_repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
astra_run_name="${1:-rviz_goal_$(date +%Y%m%d_%H%M%S)}"
if [[ $# -gt 0 ]]; then shift; fi
cat <<'INFO'
RViz2 goal flight — isolated PX4 1.17 simulation.
3D coloured map + onboard RGB image + mission control panel.
Wait for READY, use 2D Goal Pose (G) to queue XY waypoints, then press Start.
Startup health checks may wait up to 60 seconds for the map to recover.
Ground editing has no timeout by default; the aircraft stays disarmed until Start.
Choose land at the final waypoint or hold and wait for another waypoint.
Additional airborne goals join the queue; Pause/Resume requests a fresh plan.
Land or close the control panel to request guarded landing. No automatic return.
INFO
exec env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
  -u PYTHONPATH -u LD_LIBRARY_PATH bash --noprofile --norc \
  "$astra_repo_dir/astra/run_117.bash" obstacle "$astra_run_name" \
  --world-preset replica-scan --scan-dir "$astra_repo_dir/astra/runtime/maps/replica_office4" \
  --calibrate-ground-height --executor-audit --rviz-goal "$@"
