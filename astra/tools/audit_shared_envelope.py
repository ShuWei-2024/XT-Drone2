#!/usr/bin/env python3
"""Verify every recorded complete curve against the exact frozen planning mask."""
import argparse
import json
from pathlib import Path
from scipy.interpolate import BSpline
from planning_envelope import PlanningEnvelope
from scan_mesh_guard import MeshGuard
from audit_height_dynamics_117 import active_isolated_simulators


def recorded_bridge_samples(directory, motion):
    """DDS loss cannot confirm disarm, so the live result omits its full array."""
    if 'bridge_samples' in motion:
        rows = motion['bridge_samples']
    elif 'bridge_sample_count' in motion:
        rows = [json.loads(line) for line in
                (directory/'bridge_samples.jsonl').read_text().splitlines() if line.strip()]
    else:
        return []
    if 'bridge_sample_count' in motion and len(rows) != motion['bridge_sample_count']:
        raise ValueError('Bridge journal count mismatch; cannot identify executed curves')
    return rows


def audit(directory):
    motion=json.loads((directory/'motion_events.json').read_text())
    frozen=next(e for e in motion['events'] if e['event']=='shared_execution_envelope_frozen')
    guard=MeshGuard(directory/'mesh_guard.npz')
    envelope=PlanningEnvelope(guard.flight_boxes,guard.bounds,frozen['ground_origin_livo'],reserve=frozen['reserve_m'],
                              z_bounds=frozen.get('z_bounds_livo_m'),
                              connected_to_origin=guard.metadata.get('whole_map_selection',False))
    failures=[]
    for msg in motion.get('bsplines',[]):
        try: envelope.validate_spline(BSpline(msg['knots'],msg['controls'],msg['order'],extrapolate=False))
        except ValueError as exc: failures.append(dict(start_ns=msg['start_ns'],reason=str(exc)))
    # A planner candidate can be rejected by the preview guard before it ever
    # becomes an Offboard setpoint. Report those separately from curves that
    # the execution bridge actually tracked.
    tracked_ids={row.get('preview_trajectory_id') for row in recorded_bridge_samples(directory,motion)
                 if row.get('state') == 'TRACK_SITL'}
    curve_by_start={row.get('start_ns'): row for row in motion.get('bsplines',[])}
    executed_failures=[failure for failure in failures
                       if curve_by_start.get(failure.get('start_ns'),{}).get('traj_id') in tracked_ids]
    rejected_failures=[failure for failure in failures if failure not in executed_failures]
    result=dict(curve_count=len(motion.get('bsplines',[])),failed_curves=failures,
        executed_curve_failures=executed_failures,
        preview_rejected_curve_failures=rejected_failures,
        tracked_trajectory_id_count=len(tracked_ids),
        all_executed_curves_inside_shared_envelope=bool(tracked_ids) and not executed_failures,
        all_complete_curves_inside_shared_envelope=bool(motion.get('bsplines') and not failures),
        reserve_m=frozen['reserve_m'],ground_reference=frozen['ground_origin_livo'],
        recorded_z_bounds_livo_m=frozen.get('z_bounds_livo_m'),
        method='Cubic Bezier convex-hull subdivision over each knot span; blocked/unknown cells rejected',
        scope='Frozen flight-volume certificate (transit when configured, otherwise descent); obstacle observations remain separate; rejected candidates reported separately')
    (directory/'shared_envelope_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run_dir',type=Path);a=p.parse_args()
    if active_isolated_simulators():p.error('Stop simulation before offline audit')
    print(json.dumps(audit(a.run_dir),indent=2))
