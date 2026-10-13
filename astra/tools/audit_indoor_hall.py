#!/usr/bin/env python3
"""Measurement-time audit of a closed synthetic hall after SITL has exited."""
import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np
from scipy.interpolate import BSpline
from audit_height_dynamics_117 import active_isolated_simulators
from audit_obstacle_reliability_20261010 import pair, positions, describe
from obstacle_world_geometry_20261010 import clearances, load


def transition_hold_audit(goals, events, native_sends, abort=None):
    holds=[]
    for goal in goals:
        i=goal['goal_index']
        if not i:continue
        previous=next(g for g in goals if g['goal_index']==i-1)
        handoff=next((e for e in events if e['event']=='ego_leg_control_handoff' and e.get('goal_index')==i),None)
        hold_rows=[r for r in native_sends if goal['wall_ns']<=int(r['wall_ns'])<=(handoff or abort or events[-1])['wall_ns']]
        errors=[float(np.linalg.norm(np.array([float(r['target_'+axis]) for axis in 'xyz'])-previous['target_ned'])) for r in hold_rows]
        holds.append(dict(goal_index=i,native_send_count=len(hold_rows),
            maximum_offset_from_completed_endpoint_m=max(errors,default=0.),
            passed=not any(e>1e-5 for e in errors)))
    return holds


def audit(d):
    runtime=json.loads((d/'runtime.json').read_text())
    if runtime['world_preset']!='indoor-hall':raise ValueError('Requires closed indoor-hall world')
    motion=json.loads((d/'motion_events.json').read_text());events=motion['events']
    obstacles=load(d/'obstacle_geometry.json')
    rows=list(csv.DictReader((d/'frame_samples.csv').open()))
    streams={kind:sorted([r for r in rows if r['source']==kind],key=lambda r:int(r['measurement_time_ns']))
             for kind in ('gazebo','fast_livo2','px4')}
    find=lambda name:next((e for e in events if e['event']==name),None)
    pre=next((e for e in events if e['event']=='segment_start' and e.get('name')=='prestream'),None)
    start=find('ego_control_handoff');end=find('ego_trajectory_sampled_to_end')
    abort=find('test_aborted');disarm=find('disarmed')
    goals=[e for e in events if e['event']=='ego_goal_sent']
    reached=[e for e in events if e['event']=='ego_waypoint_reached']
    native_sends=list(csv.DictReader((d/'motion_sent.csv').open()))
    report=dict(scene='Closed synthetic indoor hall, not Replica scan certification',
                flight_complete=motion['flight_complete'],disarmed=motion['disarmed'],fault=motion['fault'],
                route_offsets_m=motion.get('route_offsets_m',runtime['route_offsets_m']),
                requested_goals=len(runtime['route_offsets_m']),sent_goals=len(goals),reached_goals=len(reached),
                truth_reference='base_link; rotated model offset [0,0,.24]',floor_contact_excluded=True,
                transform_reference='initial ground measurement pairs; fixed zero-yaw ENU/NED axes',
                frame_alignment_certified=False)
    transform={'fast_livo2':np.eye(3),'px4':np.array([[0,1,0],[1,0,0],[0,0,-1]])}
    bias={}
    if pre:
        for kind in transform:
            pairs=pair([r for r in streams[kind] if pre['sim_ns']-2_000_000_000<=int(r['measurement_time_ns'])<=pre['sim_ns']],streams['gazebo'])
            if pairs:bias[kind]=np.median(positions([b for a,b in pairs])-positions([a for a,b in pairs])@transform[kind].T,axis=0)
    report['ground_frozen_offsets_m']={k:v.tolist() for k,v in bias.items()}
    truth=streams['gazebo'];truth_times=np.array([int(r['measurement_time_ns']) for r in truth])
    def at(e):
        i=int(np.argmin(abs(truth_times-e['sim_ns'])))
        return positions([truth[i]])[0],abs(int(truth_times[i])-e['sim_ns'])/1e6
    if pre and disarm and truth:
        full=[r for r in truth if pre['sim_ns']<=int(r['measurement_time_ns'])<=disarm['sim_ns']]
        xyz=positions(full);dist=clearances(xyz,obstacles)
        report['whole_flight_minimum_surface_distance_m']=float(dist.min())
        report['whole_flight_clearance_breach_samples']=int((dist<.55).sum())
        # Online bounds apply to the outer model. Use controller truth here;
        # frame_samples carries the body origin instead.
        logged=[e['truth'] for e in events if e.get('truth') is not None and pre['sim_ns']<=e['sim_ns']<=disarm['sim_ns']]
        logged+= [s['truth'] for s in motion.get('bridge_samples',[])]
        a=np.array(logged)
        report['recorded_outer_pose_bound_breach_samples']=int(((np.linalg.norm(a[:,:2],axis=1)>9)|(a[:,2]<-.2)|(a[:,2]>2.1)).sum())
    legs=[];route=report['route_offsets_m'];ground=bias.get('fast_livo2')
    for goal in goals:
        i=goal['goal_index'];finish=next((e for e in reached if e['goal_index']==i),None)
        if not truth or ground is None:continue
        a,dt0=at(goal);b,dt1=at(finish or abort or events[-1]);target=np.array(goal['goal'])+ground
        requested=np.array(route[i])-(np.array(route[i-1]) if i else np.zeros(2))
        length=float(np.linalg.norm(requested));progress=float((b[:2]-a[:2])@requested/length) if length else 0
        error=float(np.linalg.norm(b[:2]-target[:2]));direct=np.linspace(a,target,1001)
        leg=dict(goal_index=i,reached=finish is not None,requested_delta_xy_m=requested.tolist(),
                 actual_delta_xy_m=(b[:2]-a[:2]).tolist(),actual_progress_m=progress,
                 endpoint_truth_xy_error_m=error,measurement_pair_dt_ms=[dt0,dt1],
                 direct_line_minimum_surface_distance_m=float(clearances(direct,obstacles).min()),
                 direct_line_intersects_obstacle=bool((clearances(direct,obstacles)==0).any()),
                 actual_motion_verified=bool(finish and max(dt0,dt1)<=20 and error<=.10 and progress>=length*.5))
        active=[r for r in truth if goal['sim_ns']<=int(r['measurement_time_ns'])<=(finish or abort or events[-1])['sim_ns']]
        if active:
            xyz=positions(active);distance=clearances(xyz,obstacles)
            axis=target[:2]-a[:2];norm=np.linalg.norm(axis)
            leg['actual_minimum_surface_distance_m']=float(distance.min())
            leg['actual_path_length_xy_m']=float(np.linalg.norm(np.diff(xyz[:,:2],axis=0),axis=1).sum())
            leg['actual_maximum_lateral_detour_m']=float(np.max(np.abs((xyz[:,0]-a[0])*axis[1]-(xyz[:,1]-a[1])*axis[0])/norm)) if norm else 0
        legs.append(leg)
    report['legs']=legs
    report['goal_transition_holds']=transition_hold_audit(goals,events,native_sends,abort)
    report['goal_transition_holds_passed']=all(r['passed'] for r in report['goal_transition_holds'])
    if start and truth:
        hi=(end or abort or events[-1])['sim_ns']
        active=[r for r in truth if start['sim_ns']<=int(r['measurement_time_ns'])<=hi]
        xyz=positions(active);tt=np.array([int(r['measurement_time_ns']) for r in active])/1e9
        unique=np.r_[True,np.diff(tt)>0];xyz,tt=xyz[unique],tt[unique]
        report['actual_xy_span_m']=np.ptp(xyz[:,:2],axis=0).tolist()
        report['actual_horizontal_path_m']=float(np.linalg.norm(np.diff(xyz[:,:2],axis=0),axis=1).sum())
        if len(tt)>1:report['actual_peak_speed_mps']=float(np.linalg.norm(np.diff(xyz,axis=0)/np.diff(tt)[:,None],axis=1).max())
        report['ground_frozen_errors']={}
        for kind,b in bias.items():
            pp=pair([r for r in streams[kind] if start['sim_ns']<=int(r['measurement_time_ns'])<=hi],truth)
            if not pp:continue
            delta=positions([a for a,_ in pp])@transform[kind].T+b-positions([v for _,v in pp])
            yaw=np.array([float(a['yaw_rad']) for a,_ in pp]);actual_yaw=np.array([float(v['yaw_rad']) for _,v in pp])
            if kind=='px4':yaw=np.pi/2-yaw
            report['ground_frozen_errors'][kind]=dict(xy_m=describe(np.linalg.norm(delta[:,:2],axis=1)),
                z_m=describe(np.abs(delta[:,2])),yaw_deg=describe(np.degrees(np.abs(np.arctan2(np.sin(yaw-actual_yaw),np.cos(yaw-actual_yaw))))),
                pair_dt_ms=describe([abs(int(a['measurement_time_ns'])-int(v['measurement_time_ns']))/1e6 for a,v in pp]))
        sends=[s['wall_ns'] for s in motion.get('bridge_samples',[]) if start['wall_ns']<=s['wall_ns']<=(end or abort or events[-1])['wall_ns']]
        sends += [int(r['wall_ns']) for r in native_sends if start['wall_ns']<=int(r['wall_ns'])<=(end or abort or events[-1])['wall_ns']]
        if len(sends)>1:report['combined_setpoint_max_send_gap_ms']=float(np.diff(sorted(sends)).max()/1e6)
        ages=[r['supervisor_odom_measurement_age_sec']*1000 for r in motion.get('bridge_samples',[]) if r.get('supervisor_odom_measurement_age_sec') is not None]
        report['odom_measurement_age_ms']=describe(ages)
        import matplotlib;matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle,Circle
        fig,ax=plt.subplots(figsize=(9,7));ax.plot(xyz[:,0],xyz[:,1],label='Measured Gazebo trajectory')
        for item in obstacles:
            if item['name']=='indoor_hall_ceiling':continue
            x,y=item['center'][:2]
            if item['kind']=='box':
                sx,sy=item['size'][:2];shape=Rectangle((x-sx/2,y-sy/2),sx,sy,color='gray',alpha=.5)
            else:shape=Circle((x,y),item['radius'],color='gray',alpha=.5)
            ax.add_patch(shape)
        for goal in goals:
            if ground is None:continue
            target=np.array(goal['goal'])+ground;a,_=at(goal)
            ax.plot([a[0],target[0]],[a[1],target[1]],'--',c='orange',alpha=.6)
            ax.scatter(*target[:2],c='purple',marker='*')
        ax.set(xlabel='World X (m)',ylabel='World Y (m)',title='Closed synthetic indoor hall: actual flight')
        ax.set_aspect('equal');ax.grid();ax.legend();fig.tight_layout();fig.savefig(d/'indoor_actual_path.png',dpi=150);plt.close(fig)
    planned=[]
    if ground is not None:
        for msg in motion.get('bsplines',[]):
            t=np.array(msg['knots']);k=msg['order'];spline=BSpline(t,msg['controls'],k)
            grid=np.linspace(t[k],t[-k-1],max(2,int((t[-k-1]-t[k])/.005)+1))
            speed=float(np.linalg.norm(spline.derivative(1)(grid),axis=1).max())
            acceleration=float(np.linalg.norm(spline.derivative(2)(grid),axis=1).max())
            distance=float(clearances(spline(grid)+ground,obstacles).min())
            # A B-spline derivative lies in the convex hull of its derivative
            # controls. This supplies a conservative speed bound between
            # samples, so clearance is not certified from samples alone.
            speed_bound=float(np.linalg.norm(spline.derivative(1).c,axis=1).max())
            continuous_lower=distance-speed_bound*float(np.diff(grid).max())/2
            planned.append(dict(start_ns=msg['start_ns'],speed_mps=speed,acceleration_mps2=acceleration,
                                minimum_surface_distance_m=distance,
                                continuous_clearance_lower_bound_m=continuous_lower,
                                clearance_grid_max_dt_s=float(np.diff(grid).max())))
    report['planned_trajectories']=planned
    report['planned_dynamics_passed']=bool(planned and all(p['speed_mps']<=.201 and p['acceleration_mps2']<=.251 for p in planned))
    report['planned_sampled_clearance_passed']=bool(planned and all(p['minimum_surface_distance_m']>=.55 for p in planned))
    report['planned_continuous_clearance_passed']=bool(planned and all(p['continuous_clearance_lower_bound_m']>=.55 for p in planned))
    sys.path.append('/home/cyf/dev/px4/venv/lib/python3.12/site-packages')
    from pyulog import ULog
    u=ULog(str(next((d/'log').rglob('*.ulg'))));data={x.name:x.data for x in u.data_list}
    status=data['vehicle_status'];report['armed_failsafe_samples']=int(np.count_nonzero((status['arming_state']==2)&status['failsafe'].astype(bool)))
    flags=data.get('estimator_status_flags',{})
    report['external_vision_active_samples']={k:int(np.count_nonzero(flags[k])) for k in ('cs_ev_pos','cs_ev_vel','cs_ev_hgt','cs_ev_yaw') if k in flags}
    if start:
        p=data['vehicle_local_position'];mask=(p['timestamp']>=start['sim_ns']/1000)&(p['timestamp']<=(end or abort or events[-1])['sim_ns']/1000)
        report['execution_reset_events']={k:int(np.count_nonzero(np.diff(p[k][mask].astype(int)))) for k in ('xy_reset_counter','z_reset_counter','heading_reset_counter')}
    report['restoration']=json.loads((d/'run_summary.json').read_text())
    resources=list(csv.DictReader((d/'process_resources.csv').open()));report['memory']={}
    for role in sorted({r['role'] for r in resources}):
        rr=[r for r in resources if r['role']==role];vals=np.array([float(r['rss_kib']) for r in rr])/1024
        tt=np.array([int(r['wall_ns']) for r in rr])/1e9;tail=tt>=tt[-1]-60
        report['memory'][role]=dict(first_mib=float(vals[0]),last_mib=float(vals[-1]),peak_mib=float(vals.max()),
            last_minute_growth_mib=float(vals[-1]-vals[np.flatnonzero(tail)[0]]),
            last_minute_slope_mib_per_min=float(np.polyfit(tt[tail]-tt[tail][0],vals[tail],1)[0]*60) if tail.sum()>1 else None)
    timing=list(csv.DictReader((d/'livo_timing.csv').open()))
    values=[int(r['lio_correspondences']) for r in timing if r['stage']=='LIO' and r.get('lio_correspondences')]
    if values:report['lio_minimum_correspondences']=min(values)
    report['complete_route_verified']=bool(motion['flight_complete'] and motion['disarmed'] and len(legs)==len(route) and all(l['actual_motion_verified'] for l in legs)
        and report.get('whole_flight_minimum_surface_distance_m',0)>=.55 and report['armed_failsafe_samples']==0
        and report.get('recorded_outer_pose_bound_breach_samples',1)==0
        and report['restoration']['original_baseline_preserved'] and report['restoration'].get('execution_sources_unchanged',False)
        and not any(report.get('execution_reset_events',{}).values())
        and report['planned_dynamics_passed'] and report['planned_continuous_clearance_passed']
        and report['goal_transition_holds_passed'])
    report['blocked_path_avoidance_verified']=bool(report['complete_route_verified'] and any(l['direct_line_intersects_obstacle'] and l.get('actual_minimum_surface_distance_m',0)>=.55 for l in legs))
    (d/'indoor_flight_audit.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('run_dir',type=Path)
    a=parser.parse_args()
    if active_isolated_simulators():parser.error('Stop simulator before offline audit')
    print(json.dumps(audit(a.run_dir),indent=2))
