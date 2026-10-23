"""Separate sensor noise, estimator error and controller tracking using saved logs."""
import argparse
import csv
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.signal import butter, find_peaks, savgol_filter, sosfiltfilt
from audit_obstacle_reliability_20261010 import pair
from run_sitl_117 import is_simulation_command
sys.path.append(str(Path.home()/'dev/px4/venv/lib/python3.12/site-packages'))
from pyulog import ULog


def stats(values):
    values=np.asarray(values)
    if not len(values):return dict(samples=0)
    return dict(samples=len(values),median=float(np.median(values)),
                minimum=float(values.min()),maximum=float(values.max()),
                stddev=float(values.std()),absolute_p95=float(np.percentile(abs(values),95)))


def active_isolated_simulators():
    """Offline ULog loading/plotting must not compete with a live flight."""
    found=[]
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            env=(proc/'environ').read_bytes().split(b'\0')
            args=[x.decode(errors='replace') for x in (proc/'cmdline').read_bytes().split(b'\0') if x]
        except (OSError,PermissionError):continue
        if b'GZ_PARTITION=astra_px4_117' in env and is_simulation_command(args):found.append(proc.name)
    return found


def audit(directory,plot_path=None):
    motion=json.loads((directory/'motion_events.json').read_text())
    events={r['event']:r for r in motion['events']}
    lo=events['ego_control_handoff']['sim_ns']/1e9
    hi=(events.get('test_aborted') or events['ego_trajectory_sampled_to_end'])['sim_ns']/1e9
    with (directory/'frame_samples.csv').open() as stream:rows=list(csv.DictReader(stream))
    streams={k:sorted([r for r in rows if r['source']==k],key=lambda r:int(r['measurement_time_ns']))
             for k in ('gazebo','fast_livo2','px4')}
    bias={}
    first=next(e for e in motion['events'] if e['event']=='ego_goal_sent')['sim_ns']
    for k,sign in (('fast_livo2',1),('px4',-1)):
        pairs=pair([r for r in streams[k] if first-2_000_000_000<=int(r['measurement_time_ns'])<=first],streams['gazebo'])
        bias[k]=float(np.median([float(t['z_m'])-sign*float(p['z_m']) for p,t in pairs]))
    tt=np.array([int(r['measurement_time_ns'])/1e9 for r in streams['gazebo']])
    truth=np.array([float(r['z_m']) for r in streams['gazebo']])
    grid=np.arange(lo+1,hi-1,.05)
    z=np.interp(grid,tt,truth)
    # Derive a smooth reference velocity from measured Gazebo body positions.
    # PX4's logged groundtruth velocity is differentiated in a wall-clock
    # callback, so it is not the primary time-aligned reference here.
    truth_v_smooth=savgol_filter(z,11,3,deriv=1,delta=.05)
    u=ULog(str(next((directory/'log').rglob('*.ulg'))));data={r.name:r.data for r in u.data_list}
    model=ET.parse(directory/'models/x500_base/model.sdf')
    pressure_sigma=float(model.find('.//air_pressure/pressure/noise/stddev').text)
    def interpolate(topic,key,stamps='timestamp_sample'):
        d=data[topic];times=d[stamps].astype(float)/1e6
        if grid[0]<times[0] or grid[-1]>times[-1]:raise ValueError('Missing log coverage: '+topic)
        return np.interp(grid,times,d[key])
    estimated=-interpolate('vehicle_local_position','z')+bias['px4']
    target=-interpolate('trajectory_setpoint','position[2]','timestamp')+bias['px4']
    internal_target=-interpolate('vehicle_local_position_setpoint','z','timestamp')+bias['px4']
    livo=np.interp(grid,[int(r['measurement_time_ns'])/1e9 for r in streams['fast_livo2']],
                   [float(r['z_m']) for r in streams['fast_livo2']])+bias['fast_livo2']
    baro=interpolate('vehicle_air_data','baro_alt_meter')
    baro_scatter=baro-z-np.median(baro-z)
    raw=data['vehicle_air_data'];raw_t=raw['timestamp_sample'].astype(float)/1e6
    raw_mask=(raw_t>=grid[0])&(raw_t<=grid[-1])&(raw_t>=tt[0])&(raw_t<=tt[-1])
    raw_scatter=raw['baro_alt_meter'][raw_mask]-np.interp(raw_t[raw_mask],tt,truth)
    raw_scatter-=np.median(raw_scatter)
    truth_v=np.gradient(z,.05)
    gps_noise=interpolate('vehicle_gps_position','vel_d_m_s')+truth_v
    estimate_error=estimated-z
    control_error=estimated-target
    true_tracking=z-target
    # Exact sign identity: actual error = control error - estimator error.
    identity=float(np.max(abs(true_tracking-(control_error-estimate_error))))
    goals=[e for e in motion['events'] if e['event']=='ego_goal_sent']
    worst=int(np.argmax(abs(true_tracking)));worst_time=grid[worst]
    last_goal=max((e for e in goals if e['sim_ns']/1e9<=worst_time),key=lambda e:e['sim_ns'])
    worst_event=dict(measurement_time_sec=float(worst_time),goal_index=last_goal['goal_index'],
        seconds_after_latest_goal=float(worst_time-last_goal['sim_ns']/1e9),
        actual_tracking_error_m=float(true_tracking[worst]),
        estimator_error_m=float(estimate_error[worst]),
        controller_estimated_tracking_error_m=float(control_error[worst]),
        actual_body_height_m=float(z[worst]),target_body_height_m=float(target[worst]))
    # Fixed descriptive windows, not a claim that a settling gate passed.
    after_handoff=grid>=lo+10.
    phase_stats={'after_first_10_seconds':dict(
        px4_estimation_error_m=stats(estimate_error[after_handoff]),
        actual_tracking_error_m=stats(true_tracking[after_handoff]))}
    peak_indices,_=find_peaks(abs(true_tracking),height=.10,distance=40)
    height_transients=[]
    for index in peak_indices:
        time_sec=float(grid[index])
        goal=max((e for e in goals if e['sim_ns']/1e9<=time_sec),key=lambda e:e['sim_ns'])
        event=dict(measurement_time_sec=time_sec,goal_index=goal['goal_index'],
            seconds_after_goal=time_sec-goal['sim_ns']/1e9,
            actual_tracking_error_m=float(true_tracking[index]),
            px4_estimation_error_m=float(estimate_error[index]),
            controller_tracking_error_m=float(control_error[index]),
            seconds_after_handoff=time_sec-lo,observations={})
        # PX4 vertical quantities are NED (positive down); convert to positive
        # up for direct comparison with Gazebo z. Snapshots are descriptive,
        # not causal attribution.
        event['vertical_dynamics_positive_up']={}
        for delta in (-2.,-1.5,-1.,-.5,0.,.5,1.,2.):
            sample_time=time_sec+delta
            nearest=int(np.argmin(abs(grid-sample_time)))
            values=dict(gazebo_vertical_velocity_mps=float(truth_v_smooth[nearest]),
                px4_estimated_vertical_velocity_mps=float(-interpolate('vehicle_local_position','vz')[nearest]),
                px4_position_derivative_velocity_mps=float(-interpolate('vehicle_local_position','z_deriv')[nearest]),
                px4_estimated_vertical_acceleration_mps2=float(-interpolate('vehicle_local_position','az')[nearest]),
                px4_vertical_estimation_error_m=float(estimate_error[nearest]),
                actual_height_tracking_error_m=float(true_tracking[nearest]))
            for topic,field,outkey in (
                ('vehicle_local_position_setpoint','acceleration[2]','px4_commanded_vertical_acceleration_mps2'),
                ('vehicle_local_position_setpoint','thrust[2]','px4_commanded_ned_thrust_z')):
                try: values[outkey]=float(interpolate(topic,field,'timestamp')[nearest])
                except (KeyError,ValueError): pass
            event['vertical_dynamics_positive_up'][str(delta)]=values
        # Search a local +/-0.5 s shift after removing the local constant
        # height bias. A consistent positive shift would support a fixed
        # estimator delay; differing shifts are only evidence of dynamic
        # filtering/control effects, not a causal delay estimate.
        window=(grid>=time_sec-3.)&(grid<=time_sec+3.)
        local_truth=z[window]
        local_truth_v=truth_v_smooth[window]
        local_grid=grid[window]
        shift_candidates=np.arange(-.5,.5001,.025)
        height_shift_errors=[]
        velocity_shift_errors=[]
        for shift in shift_candidates:
            shifted_height=np.interp(local_grid+shift,grid,estimated)
            height_residual=shifted_height-local_truth
            height_residual-=np.median(height_residual)
            shifted_velocity=np.interp(local_grid+shift,grid,
                                       -interpolate('vehicle_local_position','vz'))
            velocity_residual=shifted_velocity-local_truth_v
            velocity_residual-=np.median(velocity_residual)
            height_shift_errors.append((float(np.sqrt(np.mean(height_residual**2))),float(shift),
                                       float(np.percentile(abs(height_residual),95))))
            velocity_shift_errors.append((float(np.sqrt(np.mean(velocity_residual**2))),float(shift),
                                         float(np.percentile(abs(velocity_residual),95))))
        best_height=min(height_shift_errors)
        best_velocity=min(velocity_shift_errors)
        event['local_time_shift_search']={
            'window_sec':[-3.,3.],
            'candidate_range_sec':[-.5,.5],
            'best_height_shift_sec':best_height[1],
            'best_height_rms_m':best_height[0],
            'best_height_absolute_p95_m':best_height[2],
            'best_vertical_velocity_shift_sec':best_velocity[1],
            'best_vertical_velocity_rms_mps':best_velocity[0],
            'best_vertical_velocity_absolute_p95_mps':best_velocity[2],
            'interpretation':'Local descriptive search; different height and velocity shifts do not prove a fixed sensor delay.'}
        for topic,fields in (
            ('estimator_aid_src_baro_hgt',('innovation','innovation_filtered','test_ratio','fused','innovation_rejected')),
            ('estimator_aid_src_gnss_vel',('innovation[2]','innovation_filtered[2]','test_ratio[2]','fused','innovation_rejected')),
            ('estimator_status',('filter_fault_flags','output_tracking_error[2]'))):
            d=data[topic];t=d['timestamp_sample'].astype(float)/1e6
            window=(t>=time_sec-2.)&(t<=time_sec+2.)
            event['observations'][topic]={k:stats(d[k][window]) for k in fields if k in d}
        height_transients.append(event)
    gps=data['vehicle_gps_position'];gt=gps['timestamp_sample'].astype(float)/1e6
    velocity_truth=data['vehicle_local_position_groundtruth']
    vt=velocity_truth['timestamp_sample'].astype(float)/1e6
    gps_mask=(gt>=grid[0]+.2)&(gt<=grid[-1]-.2)&(gt>=vt[0]+.2)&(gt<=vt[-1]-.2)
    gps_alignment={str(shift):stats(gps['vel_d_m_s'][gps_mask]-
        np.interp(gt[gps_mask]+shift,vt,velocity_truth['vz']))
        for shift in (-.2,-.11,-.05,0.,.05,.11,.2)}
    timing_path=directory/'livo_timing.csv'
    if timing_path.exists():
        with timing_path.open() as stream:timing=list(csv.DictReader(stream))
        neighbourhood={}
        for stage in ('LIO','VIO'):
            nearby=[r for r in timing if r['stage']==stage and
                abs(int(r['measurement_ns'])/1e9-worst_time)<=1.]
            if not nearby:continue
            detail=dict(samples=len(nearby),processing_us=stats([float(r['processing_us']) for r in nearby]),
                maximum_absolute_z_update_m=max(abs(float(r['update_z'])-float(r['imu_z'])) for r in nearby))
            for field in ('lio_correspondences','vio_accepted_patches','vio_update_rejected'):
                values=[float(r[field]) for r in nearby if r.get(field)]
                if values:detail[field]=stats(values)
            neighbourhood[stage]=detail
        worst_event['livo_one_second_neighbourhood']=neighbourhood
    worst_event['frame_neighbourhood']={}
    for kind in ('px4','fast_livo2'):
        near=[r for r in streams[kind] if abs(int(r['measurement_time_ns'])/1e9-worst_time)<=1.]
        if len(near)>1:
            worst_event['frame_neighbourhood'][kind]=dict(samples=len(near),
                measurement_gap_ms=stats(np.diff([int(r['measurement_time_ns']) for r in near])/1e6))
    sensor_correlations={}
    filtered=sosfiltfilt(butter(2,.5,fs=20,output='sos'),baro_scatter)
    for delay in (0.,.1,.2,.5,1.):
        lag=int(round(delay/.05));a=filtered[:-lag] if lag else filtered;b=estimate_error[lag:] if lag else estimate_error
        sensor_correlations[str(delay)]=float(np.corrcoef(a,b)[0,1])
    lag_errors={}
    local=data['vehicle_local_position'];lt=local['timestamp_sample'].astype(float)/1e6
    for shift in (-.3,-.2,-.1,-.05,0.,.05,.1,.2,.3):
        candidate=-np.interp(grid+shift,lt,local['z'])+bias['px4']-z
        lag_errors[str(shift)]=stats(candidate)
    status=data['estimator_status_flags'];mask=(status['timestamp_sample']/1e6>=lo)&(status['timestamp_sample']/1e6<=hi)
    # This bridge is in position mode: PX4's velocity feedforward field is
    # intentionally NaN. Derive the commanded vertical rate from the position
    # target instead, and report the unused feedforward field separately.
    planned_vz_stats=stats(np.gradient(target,.05))
    raw_planned_vz=interpolate('trajectory_setpoint','velocity[2]','timestamp')
    finite_planned_vz=raw_planned_vz[np.isfinite(raw_planned_vz)]
    feedforward_vz_stats=(stats(finite_planned_vz) if len(finite_planned_vz) else
        dict(samples=0,encoded_samples=int(len(raw_planned_vz)),available=False,
             reason='PX4 position-control setpoints encode the unused velocity field as NaN'))
    report=dict(run=directory.name,flight_complete=motion['flight_complete'],fault=motion['fault'],
        configured_pressure_noise_pa=pressure_sigma,diagnostic_noiseless_pressure=pressure_sigma==0.,
        diagnostic_reduced_pressure_noise=pressure_sigma!=3.,
        measurement_interval_sec=[float(grid[0]),float(grid[-1])],
        frozen_initial_truth_bias_m=bias,
        target_reference='Planner NED target mapped to Gazebo body height using the two seconds '
                         'before the first goal; this is an offline frozen reference, not ground calibration',
        parameters={k:u.initial_parameters.get(k) for k in ('EKF2_BARO_NOISE','EKF2_BARO_DELAY','EKF2_GPS_DELAY','EKF2_GPS_CTRL','EKF2_HGT_REF','EKF2_DELAY_MAX','EKF2_TAU_POS','EKF2_TAU_VEL')},
        fusion_active_samples={k:int(np.count_nonzero(status[k][mask])) for k in ('cs_baro_hgt','cs_gps_hgt','cs_ev_hgt','cs_gnss_vel','cs_gnd_effect') if k in status},
        planned_height_span_m=float(np.ptp(target)),internal_height_target_span_m=float(np.ptp(internal_target)),
        planned_vertical_velocity_mps=planned_vz_stats,
        trajectory_setpoint_velocity_z_feedforward_mps=feedforward_vz_stats,
        true_height_span_m=float(np.ptp(z)),
        px4_estimation_error_m=stats(estimate_error),livo_estimation_error_m=stats(livo-z),
        controller_estimated_tracking_error_m=stats(control_error),actual_tracking_error_m=stats(true_tracking),
        tracking_identity_max_residual_m=identity,
        baro_height_scatter_raw_samples_m=stats(raw_scatter),
        baro_height_scatter_interpolated_20hz_m=stats(baro_scatter),
        gps_vertical_velocity_minus_truth_mps=stats(gps_noise),
        gps_velocity_time_shift_error_mps=gps_alignment,
        gps_publication_minus_measurement_ms=stats((gps['timestamp'][gps_mask].astype(float)-
            gps['timestamp_sample'][gps_mask])/1000.),
        worst_actual_tracking_event=worst_event,descriptive_phase_statistics=phase_stats,
        actual_tracking_peaks_over_010m=height_transients,
        peak_detection_scope='20 Hz absolute tracking error local maxima above 0.10 m, '
            'at least 2 seconds apart; +/-2 second observations are descriptive, not causal attribution',
        baro_lowpass_error_correlations=sensor_correlations,
        px4_time_shift_error_m=lag_errors,
        scope='Offline common 20 Hz grid using measured Gazebo body pose and initial frozen offsets; '
              'no offset refit. Sensor scatter uses median removal only for descriptive noise. '
              'Raw baro statistics preserve sensor samples; 20 Hz interpolation smooths noise. '
              'GPS primary comparison differentiates body truth; shift comparison uses PX4 Gazebo '
              'model-origin truth finite-difference velocity, including callback timing uncertainty. '
              'The first 10 seconds exclusion is descriptive, not a stable-hold certification. '
              'Correlation and shifted timestamps do not '
              'establish causality or authorize changing the flight reference.')
    if plot_path:
        import matplotlib;matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True)
        for name,v in (('Gazebo body truth',z),('PX4 estimate',estimated),('LIVO estimate',livo),('Planner target',target)):
            axes[0].plot(grid,v,label=name,linewidth=1)
        for name,v in (('PX4 estimate minus truth',estimate_error),('Estimated tracking error',control_error),('Actual tracking error',true_tracking)):
            axes[1].plot(grid,v,label=name,linewidth=.8)
        axes[2].plot(grid,baro_scatter,label='Baro scatter (median removed)',alpha=.45,linewidth=.5)
        axes[2].plot(grid,filtered,label='Baro scatter, descriptive 0.5 Hz lowpass',linewidth=1)
        axes[0].set_ylabel('Height (m)');axes[1].set_ylabel('Error (m)');axes[2].set_ylabel('Sensor scatter (m)')
        axes[2].set_xlabel('Simulation measurement time (s)')
        for ax in axes:ax.grid(alpha=.3);ax.legend(fontsize=8,loc='upper right')
        axes[0].set_title(directory.name+': height estimation and tracking')
        fig.tight_layout();fig.savefig(plot_path,dpi=150);plt.close(fig)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run_dir',type=Path)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--plot',type=Path)
    a=p.parse_args()
    active=active_isolated_simulators()
    if active:p.error('Stop isolated Astra SITL before offline analysis/plotting; active PIDs: '+','.join(active))
    r=audit(a.run_dir,a.plot);a.output.write_text(json.dumps(r,indent=2)+'\n')
    print(json.dumps(r,indent=2))
