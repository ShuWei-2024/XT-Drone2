#!/usr/bin/env bash
# Baylands outdoor mission, isolated PX4 1.17 SITL; explicit Start only.
set -eo pipefail
astra_repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
astra_run_name="${1:-baylands_rviz_$(date +%Y%m%d_%H%M%S)}"
if [[ $# -gt 0 ]]; then shift; fi
astra_map_dir="$astra_repo_dir/astra/runtime/maps/baylands_park"
if [[ -f "$astra_repo_dir/astra/runtime/maps/baylands_whole_map/flight_precheck.json" ]]; then
    astra_map_dir="$astra_repo_dir/astra/runtime/maps/baylands_whole_map"
fi
if [[ -f "$astra_repo_dir/astra/runtime/maps/baylands_whole_map_refined/flight_precheck.json" ]]; then
    astra_map_dir="$astra_repo_dir/astra/runtime/maps/baylands_whole_map_refined"
fi
if [[ -f "$astra_repo_dir/astra/runtime/maps/baylands_connected_full/flight_precheck.json" ]]; then
    astra_map_dir="$astra_repo_dir/astra/runtime/maps/baylands_connected_full"
fi
# Resolve an explicit override before checking or describing the scene.
astra_extra_args=("$@")
for ((astra_arg_i=0; astra_arg_i<${#astra_extra_args[@]}; astra_arg_i++)); do
    case "${astra_extra_args[astra_arg_i]}" in
        --scan-dir)
            astra_map_dir="${astra_extra_args[astra_arg_i+1]:-}"
            ;;
        --scan-dir=*) astra_map_dir="${astra_extra_args[astra_arg_i]#--scan-dir=}" ;;
    esac
done
if [[ ! -f "$astra_map_dir/flight_precheck.json" ]]; then
    echo 'Prepare Baylands first: astra/runtime/map_tools_venv/bin/python astra/tools/prepare_baylands.py' >&2
    exit 1
fi
cat <<'INFO'
Baylands 室外公园 · PX4 1.17 仿真
地面默认不限时。G 添加航点，然后在中文窗口点击“开始任务”。
灰色网格每格 0.5 m，仅为距离参考；青色轮廓／填色为静态可选航点范围。
绿色格为已检查的下降区域；蓝色为原始巡航区域，均不等于最终航点边界。
青色范围包含规划余量；点击时仍须通过实时障碍、定位和任务状态检查。
模型背景只用于显示；EGO 使用实时障碍物点云，并遵守场景的共享通行边界。
任务可选择返航降落、末点附近安全降落或限时悬停；通行区不能直接下降。
可追加 --external-vision 启用已验证的 SITL 位置融合；不融合视觉航向或速度。
INFO
python3 - "$astra_map_dir/flight_precheck.json" <<'PY'
import json,sys
p=json.load(open(sys.argv[1]))
print(f"当前场景：{p.get('scene_label','Baylands')}；检查边界：{p['allowed_xy_bounds_m']} m；目标距离上限：{p['interactive_goal_radius_m']:g} m")
if p.get('whole_map_selection'):
    print(f"可选航点面积约 {p['selectable_area_m2']:.0f} m²；地图外缘预留 {p['scene_edge_reserve_m']:g} m，障碍、净空不足和不连通区域不开放。")
if p.get('recovery_policy') == 'connected_route_to_certified_landing':
    print('已启用连通空域：异常时优先在安全位置降落，否则沿检查过的通道移至可降落区。')
PY
exec env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
  -u PYTHONPATH -u LD_LIBRARY_PATH bash --noprofile --norc \
  "$astra_repo_dir/astra/run_117.bash" obstacle "$astra_run_name" \
  --world-preset baylands --scan-dir "$astra_map_dir" \
  --calibrate-ground-height --executor-audit --isolate-estimator-cpus --rviz-goal "$@"
