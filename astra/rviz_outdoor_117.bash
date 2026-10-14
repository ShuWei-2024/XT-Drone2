#!/usr/bin/env bash
# Baylands outdoor mission, isolated PX4 1.17 SITL; explicit Start only.
set -eo pipefail
astra_repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
astra_run_name="${1:-baylands_rviz_$(date +%Y%m%d_%H%M%S)}"
if [[ $# -gt 0 ]]; then shift; fi
astra_map_dir="$astra_repo_dir/astra/runtime/maps/baylands_park"
if [[ ! -f "$astra_map_dir/flight_precheck.json" ]]; then
    echo 'Prepare Baylands first: astra/runtime/map_tools_venv/bin/python astra/tools/prepare_baylands.py' >&2
    exit 1
fi
cat <<'INFO'
Baylands 室外公园 · PX4 1.17 仿真
场景宽 70 m；本轮开放起飞点周围 10 m 的已检查区域。
地面默认不限时。G 添加航点，然后在中文窗口点击“开始任务”。
绿色格为已检查的下降区域；橙色长箱是特意加入的绕障测试道具。
模型背景只用于显示；EGO 使用 FAST-LIVO2 的实时障碍物点云，另遵守共享下降保护边界。
可追加 --external-vision 启用已验证的 SITL 位置融合；不融合视觉航向或速度。
INFO
exec env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
  -u PYTHONPATH -u LD_LIBRARY_PATH bash --noprofile --norc \
  "$astra_repo_dir/astra/run_117.bash" obstacle "$astra_run_name" \
  --world-preset baylands --scan-dir "$astra_map_dir" \
  --calibrate-ground-height --executor-audit --isolate-estimator-cpus --rviz-goal "$@"
