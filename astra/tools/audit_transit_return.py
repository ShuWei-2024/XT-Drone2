#!/usr/bin/env python3
"""Offline evidence for separate transit/landing volumes and native RTL recovery."""
import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np
from audit_height_dynamics_117 import active_isolated_simulators
from audit_obstacle_reliability_20261010 import pair, positions, describe
from flight_recovery import landing_allowed
from scan_mesh_guard import MeshGuard


def audit(directory):
    motion=json.loads((directory/'motion_events.json').read_text())
    events=motion['events']; guard=MeshGuard(directory/'mesh_guard.npz')
    precheck=json.loads((directory/'flight_precheck.json').read_text())
    if guard.transit_boxes is None or not precheck.get('validation_low_box'):
        raise ValueError('Requires the prepared low-box transit/landing validation scene')
    rows=list(csv.DictReader((directory/'frame_samples.csv').open()))
    streams={kind:sorted([r for r in rows if r['source']==kind],key=lambda r:int(r['measurement_time_ns']))
             for kind in ('gazebo','fast_livo2','px4')}
    event=lambda name:next((e for e in events if e['event']==name),None)
    pre=event('segment_start')
    pre=next((e for e in events if e['event']=='segment_start' and e.get('name')=='prestream'),pre)
    start=event('native_rtl_requested'); entered=event('native_rtl_entered'); disarm=event('native_rtl_disarmed')
    transform={'fast_livo2':np.eye(3),'px4':np.array([[0,1,0],[1,0,0],[0,0,-1]])}
    biases={}
    if pre:
        for kind in transform:
            pp=pair([r for r in streams[kind] if pre['sim_ns']-2_000_000_000<=int(r['measurement_time_ns'])<=pre['sim_ns']],streams['gazebo'])
            if pp:biases[kind]=np.median(positions([b for _,b in pp])-positions([a for a,_ in pp])@transform[kind].T,axis=0)
    full=[r for r in streams['gazebo'] if pre and int(r['measurement_time_ns'])>=pre['sim_ns']]
    xyz=positions(full) if full else np.empty((0,3))
    in_box=(np.abs(xyz[:,0]+1.5)<=.25)&(np.abs(xyz[:,1])<=.35)
    not_landable=np.array([not landing_allowed(guard,p[:2]) for p in xyz],bool)
    result=dict(flight_complete=motion['flight_complete'],fault=motion['fault'],disarmed=motion['disarmed'],
        return_home_complete=bool(motion.get('return_home_complete')),
        ground_frozen_offsets_m={k:v.tolist() for k,v in biases.items()},
        measurement_truth_samples=len(full),nonlandable_ground_sample_count=int(not_landable.sum()),
        low_box_overflight_samples=int(in_box.sum()),
        low_box_overflight_body_z_m=describe(xyz[in_box,2]) if in_box.any() else {},
        low_box_overflight_clearance_m=describe([guard.clearance(p) for p in xyz[in_box]]) if in_box.any() else {},
        native_rtl_requested=start is not None,native_rtl_entered=entered is not None,
        native_rtl_disarmed=disarm is not None,
        native_recovery_verified=False,
        scope='Known static SITL cruise corridor only; body-centre surface clearance, no hardware or dynamic-obstacle certification')
    if start:
        end=(disarm or events[-1])['sim_ns']
        truth=[r for r in streams['gazebo'] if start['sim_ns']<=int(r['measurement_time_ns'])<=end]
        points=positions(truth) if truth else np.empty((0,3))
        result['native_rtl_duration_sim_s']=(end-start['sim_ns'])/1e9
        result['native_rtl_acceptance_wall_ms']=(entered['wall_ns']-start['wall_ns'])/1e6 if entered else None
        result['native_rtl_start_landing_allowed']=landing_allowed(guard,np.asarray(start['truth'])[:2])
        if len(points):
            result['native_rtl_path_xy_m']=float(np.linalg.norm(np.diff(points[:,:2],axis=0),axis=1).sum())
            result['native_rtl_minimum_mesh_clearance_m']=min(guard.clearance(p) for p in points)
            result['native_rtl_final_home_xy_error_m']=float(np.linalg.norm(points[-1,:2]))
            result['native_rtl_body_z_bounds_m']=[float(points[:,2].min()),float(points[:,2].max())]
            breaches=[]
            for p in points:
                # Home descent uses the full vertical certificate.
                transit=not guard.inside_recovery_region(p[:2])
                breaches.append(bool(guard.fault(p,guard.clearance(p),transit=transit)))
            result['native_rtl_envelope_breach_samples']=sum(breaches)
        result['native_rtl_measurement_errors']={}
        for kind,bias in biases.items():
            source=[r for r in streams[kind] if start['sim_ns']<=int(r['measurement_time_ns'])<=end]
            pp=pair(source,streams['gazebo'])
            if not pp:continue
            delta=positions([a for a,_ in pp])@transform[kind].T+bias-positions([b for _,b in pp])
            result['native_rtl_measurement_errors'][kind]=dict(paired=len(pp),total=len(source),
                xy_m=describe(np.linalg.norm(delta[:,:2],axis=1)),z_m=describe(np.abs(delta[:,2])),
                measurement_pair_dt_ms=describe([abs(int(a['measurement_time_ns'])-int(b['measurement_time_ns']))/1e6 for a,b in pp]))
        sent=list(csv.DictReader((directory/'motion_sent.csv').open()))
        result['supervisor_setpoints_after_rtl_request']=sum(int(r['sim_ns'])>start['sim_ns'] for r in sent)
        sys.path.append(str(Path.home()/'dev/px4/venv/lib/python3.12/site-packages'))
        from pyulog import ULog
        u=ULog(str(next((directory/'log').rglob('*.ulg'))))
        data={d.name:d.data for d in u.data_list if d.multi_id==0}
        flags=data['failsafe_flags'];mask=(flags['timestamp']>=start['sim_ns']/1000)&(flags['timestamp']<=end/1000)
        keys=('global_position_invalid','home_position_invalid','local_position_invalid','local_altitude_invalid')
        result['native_rtl_invalid_position_flag_counts']={k:int(flags[k][mask].sum()) for k in keys}
        control=data.get('offboard_control_mode',{})
        result['ulog_offboard_messages_after_rtl_request']=int((control.get('timestamp',np.array([]))>start['sim_ns']/1000).sum())
        result['native_recovery_verified']=bool(entered and disarm and motion['disarmed'] and
            not result['native_rtl_start_landing_allowed'] and len(points) and mask.any() and
            result['native_rtl_final_home_xy_error_m']<=.25 and
            result['native_rtl_minimum_mesh_clearance_m']>=.55 and
            not result['native_rtl_envelope_breach_samples'] and
            not any(result['native_rtl_invalid_position_flag_counts'].values()) and
            not result['supervisor_setpoints_after_rtl_request'] and
            not result['ulog_offboard_messages_after_rtl_request'])
    (directory/'transit_return_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run_dir',type=Path);a=p.parse_args()
    if active_isolated_simulators():p.error('Stop simulation before offline audit')
    print(json.dumps(audit(a.run_dir),indent=2))
