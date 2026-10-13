"""Trace sampled planner setpoints back to published splines, after SITL exits."""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.interpolate import BSpline
from obstacle_world_geometry_20261010 import clearances, load


def prepare(messages):
    return [dict(message=m, spline=BSpline(m['knots'],m['controls'],m['order'],extrapolate=False),
        start=float(m['knots'][m['order']]),end=float(m['knots'][-m['order']-1])) for m in messages]


def match_sample(sample, trajectories, alignment):
    stamp=sample.get('preview_pose_stamp_ns')
    if stamp is None or sample.get('target') is None:return None
    target=np.asarray(sample['target']);matrix=np.asarray(alignment['xy_matrix'])
    position=np.r_[matrix.T@(target[:2]-alignment['xy_translation']),
                   alignment['z_translation']-target[2]]
    hint=(sample.get('preview_trajectory_id'),sample.get('preview_trajectory_start_ns'))
    matches=[]
    for i,tr in enumerate(trajectories):
        m=tr['message'];elapsed=(stamp-m['start_ns'])/1e9
        if not -.02<=elapsed<=tr['end']-tr['start']+.02:continue
        t=np.clip(elapsed+tr['start'],tr['start'],tr['end'])
        if np.linalg.norm(tr['spline'](t)-position)>1e-6:continue
        matches.append((i,float(t),position))
        # A newer preview ID may precede its paired pose delivery. Trust the ID
        # only after verifying the actual sampled position and exact pose time.
        if (m.get('traj_id'),m['start_ns'])==hint:return matches[-1]
    return matches[0] if len(matches)==1 else None


def audit(root):
    motion=json.loads((root/'motion_events.json').read_text())
    hall=json.loads((root/'indoor_flight_audit.json').read_text())
    obstacles=load(root/'obstacle_geometry.json')
    trajectories=prepare(motion.get('bsplines',[]))
    samples=[s for s in motion.get('bridge_samples',[]) if s['state']=='TRACK_SITL']
    report=dict(scope='Diagnostic sampled setpoints only. Exact pose timestamps and position '
        'agreement identify splines; sparse samples do not certify continuous execution clearance. '
        'No original full-trajectory or physical-flight verdict is changed.',run=root.name,
        full_route_verified=hall['complete_route_verified'],tracking_samples=len(samples),
        samples_with_exact_pose_time=sum(s.get('preview_pose_stamp_ns') is not None for s in samples))
    used={};matched=[]
    for s in samples:
        result=match_sample(s,trajectories,motion['alignment'])
        if result is None:continue
        i,t,position=result;used.setdefault(i,[]).append(t);matched.append(position)
    report['matched_samples']=len(matched);report['unresolved_samples']=len(samples)-len(matched)
    bias=np.asarray(hall.get('ground_frozen_offsets_m',{}).get('fast_livo2',[]))
    if len(bias)==3 and matched:
        distances=clearances(np.asarray(matched)+bias,obstacles)
        report['minimum_matched_sampled_setpoint_distance_m']=float(min(distances))
        report['matched_sampled_setpoints_below_055m']=int((distances<.55).sum())
    bad=[]
    for i,(tr,published) in enumerate(zip(trajectories,hall['planned_trajectories'])):
        if published['continuous_clearance_lower_bound_m']>=.55:continue
        row=dict(index=i,traj_id=tr['message'].get('traj_id'),start_ns=tr['message']['start_ns'],
            published_continuous_clearance_lower_bound_m=published['continuous_clearance_lower_bound_m'],
            matched_sample_count=len(used.get(i,[])))
        if len(bias)==3:
            grid=np.linspace(tr['start'],tr['end'],max(2,int((tr['end']-tr['start'])/.005)+1))
            distance=clearances(tr['spline'](grid)+bias,obstacles);below=np.flatnonzero(distance<.55)
            if len(below):
                first=float(grid[below[0]])
                row['first_sampled_below_limit_elapsed_sec']=first-tr['start']
                if i in used:
                    row['last_matched_elapsed_sec']=max(used[i])-tr['start']
                    row['matched_samples_remained_before_first_violation']=max(used[i])<first
        bad.append(row)
    report['published_trajectories_below_limit']=bad
    return report


if __name__=='__main__':
    from audit_height_dynamics_117 import active_isolated_simulators
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run_dir',type=Path)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if active_isolated_simulators():p.error('Stop simulator before offline selection audit')
    report=audit(a.run_dir);a.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
