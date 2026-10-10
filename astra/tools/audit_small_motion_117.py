"""Measurement-time residuals with one frozen initial bias for a native-NED flight."""
import argparse
import csv
import json
from pathlib import Path
import sys
import numpy as np

from audit_obstacle_reliability_20261010 import pair, positions, describe
sys.path.append(str(Path.home()/'dev/px4/venv/lib/python3.12/site-packages'))
from pyulog import ULog


def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args();root=a.run
    recorded=json.loads((root/'motion_events.json').read_text());events=recorded['events'];flight=recorded['result']
    rows=list(csv.DictReader((root/'frame_samples.csv').open()))
    streams={kind:sorted([r for r in rows if r['source']==kind],key=lambda r:int(r['measurement_time_ns']))
             for kind in ('gazebo','fast_livo2','px4')}
    motion=next((e for e in events if e['event']=='segment_start' and e['name']=='north'),None)
    result={'route_complete':flight['route_complete'],'disarmed':flight['disarmed'],'fault':flight['fault'],
            'test_mode':flight['test_mode'], 'strict_stable_hold_certified':False,
            'axis_relation':'PX4 NED -> Gazebo ENU: [y,x,-z]',
            'bias_method':'one frozen 2s interval preceding north; no fit on later flight data',
            'frame_alignment_certified':False,'phases':{},'sensor_streams':{}}
    if motion:
        start=motion['sim_ns'];bias={};transforms={'fast_livo2':np.eye(3),'px4':np.array([[0,1,0],[1,0,0],[0,0,-1]])}
        for kind in ('fast_livo2','px4'):
            paired=pair([r for r in streams[kind] if start-2_000_000_000<=int(r['measurement_time_ns'])<=start],streams['gazebo'])
            if not paired:raise RuntimeError('No initial paired samples for '+kind)
            bias[kind]=np.median(positions([b for a,b in paired])-positions([a for a,b in paired])@transforms[kind].T,axis=0)
        result['initial_bias_m']={k:v.tolist() for k,v in bias.items()}
        for begin in [e for e in events if e['event']=='segment_start' and e['name'] in ('north','east','return')]:
            end=next((e for e in events if e['event']=='segment_end' and e['name']==begin['name'] and e['sim_ns']>=begin['sim_ns']),None)
            if not end:continue
            lo,hi=begin['sim_ns'],end['sim_ns'];truth=[r for r in streams['gazebo'] if lo<=int(r['measurement_time_ns'])<=hi]
            phase={'actual_delta_enu_m':(positions(truth)[-1]-positions(truth)[0]).tolist() if truth else None}
            hold_end=next((e for e in events if e['event']=='segment_end' and e.get('name')==
                {'north':'hold_north','east':'hold_east','return':'hold_final'}[begin['name']] and e['sim_ns']>=hi),None)
            if hold_end:
                phase['actual_delta_through_endpoint_observation_enu_m']=(
                    np.array(hold_end['truth'])-np.array(begin['truth'])).tolist()
            for kind in ('fast_livo2','px4'):
                paired=pair([r for r in streams[kind] if lo<=int(r['measurement_time_ns'])<=hi],streams['gazebo'])
                if not paired:continue
                source,target=[a for a,b in paired],[b for a,b in paired]
                delta=positions(source)@transforms[kind].T+bias[kind]-positions(target)
                phase[kind+'_xy_residual_m']=describe(np.linalg.norm(delta[:,:2],axis=1))
                phase[kind+'_z_residual_m']=describe(np.abs(delta[:,2]))
                phase[kind+'_pair_dt_ms']=describe([abs(int(a['measurement_time_ns'])-int(b['measurement_time_ns']))/1e6 for a,b in paired])
                yaw=np.array([float(r['yaw_rad']) for r in source]);truth_yaw=np.array([float(r['yaw_rad']) for r in target])
                if kind=='px4':yaw=np.pi/2-yaw
                phase[kind+'_absolute_yaw_residual_deg']=describe(np.abs(np.degrees(np.arctan2(np.sin(yaw-truth_yaw),np.cos(yaw-truth_yaw)))))
            result['phases'][begin['name']]=phase
        active=[r for r in streams['px4'] if int(r['measurement_time_ns'])>=start]
        if active:
            result['px4_reset_counter_changes_after_motion_start']=np.ptp(np.array(
                [[int(r[k]) for k in ('xy_reset','z_reset','heading_reset')] for r in active]),axis=0).tolist()
    logs=list((root/'log').rglob('*.ulg'))
    if logs:
        u=ULog(str(logs[-1]));datasets={d.name:d.data for d in u.data_list if d.multi_id==0}
        for name in ('vehicle_gps_position','sensor_mag','battery_status','sensor_combined'):
            if name in datasets:
                d=datasets[name];t=d['timestamp'].astype(float)
                result['sensor_streams'][name]={'samples':len(t),'first_sec':float(t[0]/1e6),'last_sec':float(t[-1]/1e6),'max_gap_ms':float(np.diff(t).max()/1000) if len(t)>1 else None}
        if 'vehicle_status' in datasets:
            d=datasets['vehicle_status'];armed=d['arming_state']==2
            result['armed_failsafe_samples']=int(np.count_nonzero(d['failsafe'][armed]))
        result['ekf2_ev_ctrl']=u.initial_parameters.get('EKF2_EV_CTRL')
    (root/'small_motion_audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':main()
