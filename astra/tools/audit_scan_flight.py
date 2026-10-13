#!/usr/bin/env python3
"""Audit scan-scene flight after simulator exit; never apply legacy box metrics."""
import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np
from scipy.interpolate import BSpline
from audit_height_dynamics_117 import active_isolated_simulators
from audit_obstacle_reliability_20261010 import audit as audit_errors, pair, positions, describe
from scan_mesh_guard import MeshGuard


def stats(x):
    x=np.asarray(x,dtype=float)
    return dict(count=len(x),minimum=float(x.min()),p95=float(np.percentile(x,95)),maximum=float(x.max())) if len(x) else {}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir',type=Path)
    a=parser.parse_args()
    if active_isolated_simulators():parser.error('Stop isolated simulation before offline analysis')
    d=a.run_dir
    motion=json.loads((d/'motion_events.json').read_text())
    events=motion['events'];guard=MeshGuard(d/'mesh_guard.npz')
    errors=audit_errors(d)
    (d/'obstacle_reliability_audit.json').write_text(json.dumps(errors,indent=2)+'\n')
    start=next((e for e in events if e['event']=='ego_control_handoff'),None)
    end=next((e for e in events if e['event']=='ego_trajectory_sampled_to_end'),None)
    abort=next((e for e in events if e['event']=='test_aborted'),None)
    goals=[e for e in events if e['event']=='ego_goal_sent']
    reached=[e for e in events if e['event']=='ego_waypoint_reached']
    report=dict(flight_complete=motion['flight_complete'],disarmed=motion['disarmed'],
                fault=motion['fault'],requested_goals=len(goals),reached_goals=len(reached),
                phases=errors.get('phases',{}),guard=guard.metadata,
                missing_geometry_certified=False,near_obstacle_avoidance_certified=False,
                scope='Short indoor motion in a clear corridor; not a blocked-path avoidance trial')
    samples=motion.get('bridge_samples',[])
    if samples:
        p=np.array([s['mesh_guard_xyz'] for s in samples])
        distances=np.array([guard.clearance(x) for x in p])
        report['online_recorded_mesh_clearance_m']=stats([s['mesh_clearance_m'] for s in samples])
        report['offline_recomputed_mesh_clearance_m']=stats(distances)
        report['actual_horizontal_path_m']=float(np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1).sum())
        report['actual_body_span_xyz_m']=np.ptp(p,axis=0).tolist()
        report['bridge_sample_max_gap_ms']=float(np.diff([s['wall_ns'] for s in samples]).max()/1e6) if len(samples)>1 else None
        report['supervisor_measurement_age_ms']=stats([1000*s['supervisor_odom_measurement_age_sec'] for s in samples])
    peaks=[];planned_distances=[]
    offset=np.asarray(errors.get('initial_truth_offset_m',{}).get('fast_livo2',[0,0,0]))
    for spline in motion.get('bsplines',[]):
        t=np.asarray(spline['knots']);k=spline['order'];b=BSpline(t,spline['controls'],k)
        grid=np.linspace(t[k],t[-k-1],max(2,int((t[-k-1]-t[k])/.02)+1))
        peaks.append([np.linalg.norm(b.derivative(i)(grid),axis=1).max() for i in (1,2)])
        planned_distances.extend(guard.clearance(x) for x in b(grid)+offset)
    report['bspline_count']=len(peaks)
    if peaks:
        report['planned_peak_speed_mps'],report['planned_peak_acceleration_mps2']=np.max(peaks,axis=0).tolist()
        report['planned_mesh_clearance_m']=stats(planned_distances)
    rows=list(csv.DictReader((d/'frame_samples.csv').open()))
    streams={kind:sorted([x for x in rows if x['source']==kind],key=lambda x:int(x['measurement_time_ns']))
             for kind in ['gazebo','fast_livo2','px4']}
    prestream=next((e for e in events if e['event']=='segment_start' and e.get('name')=='prestream'),None)
    climb=next((e for e in events if e['event']=='segment_start' and e.get('name')=='climb'),None)
    climb_end=next((e for e in events if e['event']=='segment_end' and e.get('name')=='climb'),None)
    ground_bias={}
    if prestream and climb:
        # Preserve ground offsets as well as the existing pre-execution fit.
        # Fitting only after takeoff can hide drift accumulated during climb.
        transform={'fast_livo2':np.eye(3),'px4':np.array([[0,1,0],[1,0,0],[0,0,-1]])}
        for kind in ['fast_livo2','px4']:
            initial=pair([x for x in streams[kind] if prestream['sim_ns']-2_000_000_000<=int(x['measurement_time_ns'])<=prestream['sim_ns']],streams['gazebo'])
            if initial:
                ground_bias[kind]=np.median(positions([b for _,b in initial])-positions([a for a,_ in initial])@transform[kind].T,axis=0)
        report['ground_frozen_initial_truth_offset_m']={k:v.tolist() for k,v in ground_bias.items()}
        windows={'climb':(climb['sim_ns'],(climb_end or abort or events[-1])['sim_ns'])}
        if start:windows['ego_execution']=(start['sim_ns'],(end or abort or events[-1])['sim_ns'])
        report['ground_frozen_errors']={}
        for label,(lo,hi) in windows.items():
            out={}
            for kind,bias in ground_bias.items():
                pp=pair([x for x in streams[kind] if lo<=int(x['measurement_time_ns'])<=hi],streams['gazebo'])
                if not pp:continue
                delta=positions([a for a,_ in pp])@transform[kind].T+bias-positions([b for _,b in pp])
                out[kind+'_xy_m']=describe(np.linalg.norm(delta[:,:2],axis=1))
                out[kind+'_z_m']=describe(np.abs(delta[:,2]))
                out[kind+'_signed_z_median']=float(np.median(delta[:,2]))
            report['ground_frozen_errors'][label]=out
    truth=sorted([x for x in rows if x['source']=='gazebo'],key=lambda x:int(x['measurement_time_ns']))
    report['actual_route_legs']=[]
    route=motion.get('route_offsets_m',[])
    origin=motion.get('origin_livo')
    if truth and origin is not None:
        truth_times=np.array([int(x['measurement_time_ns']) for x in truth])
        def at_event(event):
            i=int(np.argmin(np.abs(truth_times-event['sim_ns'])))
            return positions([truth[i]])[0],abs(int(truth_times[i])-event['sim_ns'])/1e6
        for goal in goals:
            index=goal['goal_index']
            finish=next((e for e in reached if e.get('goal_index')==index),None)
            begin_xyz,begin_dt=at_event(goal)
            finish_xyz,finish_dt=at_event(finish or abort or events[-1])
            previous=np.zeros(2) if index==0 else np.array(route[index-1])
            requested=np.array(route[index])-previous
            length=float(np.linalg.norm(requested))
            delta=finish_xyz[:2]-begin_xyz[:2]
            progress=float(delta@requested/length) if length else 0.
            goal_xyz=np.array(goal['goal'])+offset
            post_climb_error=float(np.linalg.norm(finish_xyz[:2]-goal_xyz[:2]))
            ground_offset=ground_bias.get('fast_livo2')
            endpoint_error=(float(np.linalg.norm(finish_xyz[:2]-(np.array(goal['goal'])+ground_offset)[:2]))
                            if ground_offset is not None else None)
            measured=begin_dt<=20. and finish_dt<=20.
            passed=bool(finish and measured and endpoint_error is not None and endpoint_error<=.10 and progress>=.5*length)
            report['actual_route_legs'].append(dict(goal_index=index,reached=finish is not None,
                requested_delta_xy_m=requested.tolist(),actual_delta_xy_m=delta.tolist(),
                actual_progress_along_requested_direction_m=progress,
                endpoint_truth_xy_error_m=endpoint_error,measurement_pair_dt_ms=[begin_dt,finish_dt],
                endpoint_error_reference='ground-frozen LIVO-to-truth translation',
                post_climb_fitted_endpoint_truth_xy_error_m=post_climb_error,
                actual_motion_verified=passed))
    # Recovery is part of the test: an execution-only distance hides overshoot
    # during fault hold and descent.
    if prestream and truth:
        disarm=next((e for e in events if e['event']=='disarmed'),events[-1])
        full=[x for x in truth if prestream['sim_ns']<=int(x['measurement_time_ns'])<=disarm['sim_ns']]
        full_xyz=positions(full)
        report['whole_flight_mesh_clearance_m']=stats([guard.clearance(x) for x in full_xyz])
        outside=(full_xyz[:,:2]<guard.bounds[0]).any(axis=1)|(full_xyz[:,:2]>guard.bounds[1]).any(axis=1)
        report['whole_flight_envelope_breach_samples']=int(outside.sum())
        report['whole_flight_envelope_passed']=not bool(outside.any())
        if outside.any():
            first=int(np.flatnonzero(outside)[0])
            report['first_envelope_breach']=dict(measurement_ns=int(full[first]['measurement_time_ns']),
                                                body_xyz_m=full_xyz[first].tolist())
    if start and truth:
        lo=start['sim_ns'];hi=(end or abort or events[-1])['sim_ns']
        truth=[x for x in truth if lo<=int(x['measurement_time_ns'])<=hi]
        xyz=np.array([[float(x[c]) for c in ['x_m','y_m','z_m']] for x in truth])
        tt=np.array([int(x['measurement_time_ns']) for x in truth])/1e9
        unique=np.r_[True,np.diff(tt)>0];xyz,tt=xyz[unique],tt[unique]
        report['measurement_time_truth_samples']=len(xyz)
        report['measurement_time_actual_mesh_clearance_m']=stats([guard.clearance(x) for x in xyz])
        report['actual_horizontal_span_m']=np.ptp(xyz[:,:2],axis=0).tolist()
        if len(xyz)>1:report['actual_peak_3d_speed_mps']=float(np.linalg.norm(np.diff(xyz,axis=0)/np.diff(tt)[:,None],axis=1).max())
        report['measurement_time_actual_horizontal_path_m']=float(np.linalg.norm(np.diff(xyz[:,:2],axis=0),axis=1).sum())
        # Endpoint settling and replacement waits use the supervisor publisher;
        # bridge-only sample gaps do not represent Offboard transmission gaps.
        sends=[s['wall_ns'] for s in samples if start['wall_ns']<=s['wall_ns']<=(end or abort or events[-1])['wall_ns']]
        if (d/'motion_sent.csv').is_file():
            native=list(csv.DictReader((d/'motion_sent.csv').open()))
            sends.extend(int(s['wall_ns']) for s in native if start['wall_ns']<=int(s['wall_ns'])<=(end or abort or events[-1])['wall_ns'])
        if len(sends)>1:
            report['combined_setpoint_send_count']=len(sends)
            report['combined_setpoint_send_max_gap_ms']=float(np.diff(sorted(sends)).max()/1e6)
        # Show only actual scanned geometry at flight height, not old synthetic boxes.
        import matplotlib;matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        tri=guard.triangles;v=tri.reshape(-1,3);z=float(np.median(xyz[:,2]))
        near=v[np.abs(v[:,2]-z)<.55][::4]
        fig,axes=plt.subplots(1,2,figsize=(12,5))
        targets=np.array([g['goal'] for g in goals])+ground_bias.get('fast_livo2',offset)
        for ax in axes:
            if len(near):ax.scatter(near[:,0],near[:,1],s=.5,c='gray',label='Scan surfaces')
            ax.plot(xyz[:,0],xyz[:,1],label='Measured Gazebo path')
            if len(targets):ax.scatter(targets[:,0],targets[:,1],marker='*',c='purple',label='Goals (ground-frozen frame)')
            ax.scatter(*xyz[0,:2],c='green',label='Planner handoff')
            ax.set(xlabel='World X (m)',ylabel='World Y (m)')
            ax.set_aspect('equal');ax.grid()
        axes[0].set_title('Replica office0: room context')
        axes[1].set(xlim=(guard.bounds[0,0]-.05,guard.bounds[1,0]+.05),
                    ylim=(guard.bounds[0,1]-.05,guard.bounds[1,1]+.05),
                    title='Four short legs: measured flight')
        axes[1].legend(fontsize=8,loc='lower right');fig.tight_layout()
        fig.savefig(d/'scan_actual_path.png',dpi=160);plt.close(fig)
    sys.path.append('/home/cyf/dev/px4/venv/lib/python3.12/site-packages')
    from pyulog import ULog
    u=ULog(str(next((d/'log').rglob('*.ulg'))));datasets={x.name:x.data for x in u.data_list}
    s=datasets['vehicle_status']
    report['armed_failsafe_samples']=int(np.count_nonzero((s['arming_state']==2)&s['failsafe'].astype(bool)))
    flags=datasets.get('estimator_status_flags',{})
    report['external_vision_active_samples']={k:int(np.count_nonzero(flags[k])) for k in ['cs_ev_pos','cs_ev_vel','cs_ev_hgt','cs_ev_yaw'] if k in flags}
    report['baseline_preserved']=json.loads((d/'run_summary.json').read_text())['original_baseline_preserved']
    report['complete_route_verified']=bool(motion['flight_complete'] and route and
        len(report['actual_route_legs'])==len(route) and
        all(x['actual_motion_verified'] for x in report['actual_route_legs']) and
        report.get('whole_flight_envelope_passed',False) and
        report.get('whole_flight_mesh_clearance_m',{}).get('minimum',0.)>=.55 and
        not report['armed_failsafe_samples'] and motion['disarmed'])
    (d/'scan_flight_audit.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
