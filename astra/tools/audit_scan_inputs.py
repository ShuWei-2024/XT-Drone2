#!/usr/bin/env python3
"""Compare raw simulated observations to truth and rendered collision geometry."""
import argparse
import csv
import json
from pathlib import Path
import tempfile

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.signal import savgol_filter
from scipy.integrate import cumulative_trapezoid
from audit_height_dynamics_117 import active_isolated_simulators
from scan_mesh_guard import MeshGuard, load_stl


def describe(x):
    x=np.asarray(x)
    if not len(x): return dict(count=0,median=None,p95=None,maximum=None)
    return dict(count=len(x),median=float(np.median(x)),p95=float(np.percentile(x,95)),maximum=float(x.max()))


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('run_dir',type=Path)
    a=parser.parse_args()
    if active_isolated_simulators():parser.error('Stop simulation before auditing inputs')
    d=a.run_dir
    required=['raw_truth.csv','raw_imu.csv','raw_cloud_samples.npz','motion_events.json',
              'livo_timing.csv','models/replica_office0/meshes/collision.stl']
    missing=[name for name in required if not (d/name).is_file()]
    if missing:parser.error('Missing captured inputs: '+', '.join(missing))
    raw=list(csv.DictReader((d/'raw_truth.csv').open()))
    if len(raw)<2:parser.error('At least two truth samples are required')
    truth=np.array([[float(x[k]) for k in ['stamp_ns','x','y','z','qx','qy','qz','qw']] for x in raw])
    truth=truth[np.argsort(truth[:,0])];truth=truth[np.r_[True,np.diff(truth[:,0])>0]]
    if len(truth)<2:parser.error('Truth must cover two distinct measurement times')
    stamps=truth[:,0];t=stamps/1e9
    rot=Rotation.from_quat(truth[:,4:8])
    triangles=load_stl(d/'models/replica_office0/meshes/collision.stl')
    with tempfile.TemporaryDirectory() as temp:
        path=Path(temp)/'full_mesh.npz'
        np.savez(path,triangles=triangles,metadata=json.dumps(dict(minimum_clearance_m=.55,allowed_xy_bounds_m=[[-10,-10],[10,10]])))
        mesh=MeshGuard(path)
        data=np.load(d/'raw_cloud_samples.npz');distances=[];pair_dt=[];clouds=[]
        for stamp,xyz in zip(data['stamps'],data['xyz']):
            i=int(np.argmin(np.abs(stamps-stamp)))
            if abs(stamps[i]-stamp)>20e6:continue
            # Outer model -> inherited base_link (.24) -> scan frame (.22).
            points=rot[i].apply(xyz+[0,0,.46])+truth[i,1:4]
            valid=np.linalg.norm(xyz,axis=1)>.45
            residual=[mesh.clearance(p) for p in points[valid]]
            distances.extend(residual);pair_dt.append(abs(stamps[i]-stamp)/1e6)
            clouds.append(dict(stamp_ns=int(stamp),sensor_height_m=float(rot[i].apply([0,0,.46])[2]+truth[i,3]),surface_distance_m=describe(residual)))
    rows=list(csv.DictReader((d/'raw_imu.csv').open()))
    if len(rows)<2:parser.error('At least two IMU samples are required')
    imu=np.array([[float(x[k]) for k in ['stamp_ns','ax','ay','az','gx','gy','gz']] for x in rows])
    valid=(imu[:,0]>=stamps[0])&(imu[:,0]<=stamps[-1]);imu=imu[valid]
    if len(imu)<2:parser.error('No usable paired IMU interval')
    imu=imu[np.argsort(imu[:,0])];imu=imu[np.r_[True,np.diff(imu[:,0])>0]]
    if len(imu)<2:parser.error('No usable paired IMU interval')
    ti=imu[:,0]/1e9
    nearest=np.searchsorted(stamps,imu[:,0]);nearest=np.clip(nearest,1,len(stamps)-1)
    nearest-=np.abs(stamps[nearest-1]-imu[:,0])<np.abs(stamps[nearest]-imu[:,0])
    acceleration=rot[nearest].apply(imu[:,1:4])+[0,0,-9.81]
    imu_xyz=truth[:,1:4]+rot.apply(np.tile([0,0,.36],(len(truth),1)))
    # Uniform 100 Hz grid, descriptive smoothing to compare accelerations.
    grid=np.arange(ti[0]+.3,ti[-1]-.3,.01)
    if len(grid)<31:parser.error('Captured interval is too short for acceleration comparison')
    height=np.interp(grid,t,imu_xyz[:,2])
    measured=np.interp(grid,ti,acceleration[:,2])
    actual=savgol_filter(height,31,3,deriv=2,delta=.01)
    measured=savgol_filter(measured,31,3)
    motion=json.loads((d/'motion_events.json').read_text());events=motion['events']
    climb=next((e for e in events if e['event']=='segment_start' and e.get('name')=='climb'),None)
    if climb is None:parser.error('No climb occurred in this run')
    stop=next((e for e in events if e['event']=='segment_end' and e.get('name')=='climb'),None) or next(e for e in events if e['event']=='test_aborted')
    m=(grid>=climb['sim_ns']/1e9+.2)&(grid<=stop['sim_ns']/1e9-.2)
    if not m.any():parser.error('Climb has no usable acceleration comparison window')
    raw_mask=(ti>=climb['sim_ns']/1e9)&(ti<=stop['sim_ns']/1e9)
    raw_time=ti[raw_mask]
    if len(raw_time)<2:parser.error('Climb has too few IMU measurements')
    raw_velocity=cumulative_trapezoid(acceleration[raw_mask,2],raw_time,initial=0.)
    raw_height=cumulative_trapezoid(raw_velocity,raw_time,initial=0.)
    actual_rise=np.interp(raw_time[-1],t,imu_xyz[:,2])-np.interp(raw_time[0],t,imu_xyz[:,2])
    report=dict(raw_lidar_surface_distance_m=describe(distances),cloud_pair_dt_ms=describe(pair_dt),
                cloud_samples=clouds,
                climb_imu_acceleration_z_residual_mps2=describe(np.abs(measured[m]-actual[m])),
                climb_imu_acceleration_z_mean_bias_mps2=float(np.mean(measured[m]-actual[m])),
                climb_imu_acceleration_z_rmse_mps2=float(np.sqrt(np.mean((measured[m]-actual[m])**2))),
                scan_reference_model_offset_m=[0,0,.46],imu_reference_model_offset_m=[0,0,.36],
                raw_imu_free_integrated_rise_m=float(raw_height[-1]),
                actual_imu_reference_rise_m=float(actual_rise),
                free_integration_initial_velocity_assumed_mps=0.,
                description='Raw sensor consistency only; smoothed truth acceleration is a diagnostic, not an estimator input')
    timing=list(csv.DictReader((d/'livo_timing.csv').open()))
    states=[r for r in timing if climb['sim_ns']<=int(r['measurement_ns'])<=stop['sim_ns']]
    report['climb_state_updates']={}
    for stage in ['LIO','VIO']:
        rows=[r for r in states if r['stage']==stage]
        if not rows:continue
        propagation=np.array([float(r['imu_z'])-float(r['pre_z']) for r in rows])
        correction=np.array([float(r['update_z'])-float(r['imu_z']) for r in rows])
        detail=dict(count=len(rows),summed_propagation_z_m=float(propagation.sum()),
                    summed_correction_z_m=float(correction.sum()),
                    absolute_correction_z_m=describe(np.abs(correction)))
        if 'imu_integrated_dt_sec' in rows[0]:
            detail['summed_velocity_correction_z_mps']=sum(float(r['update_vz'])-float(r['imu_vz']) for r in rows)
            coverage=np.array([float(r['imu_integrated_dt_sec'])-
                (float(r['imu_prop_end_sec'])-float(r['imu_prop_begin_sec'])) for r in rows])
            detail['absolute_propagation_coverage_gap_sec']=describe(np.abs(coverage))
            detail['state_z']={k:describe([float(r[k]) for r in rows]) for k in
                               ['pre_vz','imu_vz','update_vz','pre_baz','update_baz',
                                'pre_gravity_z','update_gravity_z']}
            st=np.array([int(r['measurement_ns'])/1e9 for r in rows])
            actual_velocity=savgol_filter(height,31,3,deriv=1,delta=.01)
            expected=np.interp(st,grid,actual_velocity)
            detail['estimated_imu_vz_minus_truth_mps']=describe(np.abs(
                np.array([float(r['imu_vz']) for r in rows])-expected))
            detail['estimated_update_vz_minus_truth_mps']=describe(np.abs(
                np.array([float(r['update_vz']) for r in rows])-expected))
        report['climb_state_updates'][stage]=detail
    (d/'scan_input_audit.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='cloud_samples'},indent=2))


if __name__=='__main__':main()
