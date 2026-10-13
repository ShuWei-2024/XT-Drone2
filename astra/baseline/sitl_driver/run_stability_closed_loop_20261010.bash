#!/usr/bin/env bash
# Explicitly arms isolated PX4 SITL; never connects a physical vehicle.
set -euo pipefail
ROOT=/home/cyf/Documents/Codex/2026-10-06/home-cyf-dev-astra-ros2-ws
SUFFIX=${1:?Supply a new run suffix.}
MODE=${2:-position_only}
case "$MODE" in
  position_only) export ASTRA_VELOCITY_FEEDFORWARD=0 ASTRA_FAULT_TARGET=none ;;
  velocity_feedforward) export ASTRA_VELOCITY_FEEDFORWARD=1 ASTRA_FAULT_TARGET=none ;;
  map_failure) export ASTRA_VELOCITY_FEEDFORWARD=1 ASTRA_FAULT_TARGET=map ;;
  *) echo 'Mode must be position_only, velocity_feedforward or map_failure.' >&2; exit 1 ;;
esac
export ASTRA_FLIGHT_SCRIPT=$ROOT/work/ego_stability_20261010/ego_sitl_flight.py
cd "$ROOT"
bash outputs/run_geometry_rich_closed_loop_20261010.bash "$SUFFIX"
