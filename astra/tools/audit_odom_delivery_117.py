"""Offline delivery audit separating LIO work and independent subscriber callbacks."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np

from audit_height_dynamics_117 import active_isolated_simulators


def describe(values):
    values=np.asarray(values,dtype=float)
    if not len(values):return dict(samples=0)
    return dict(samples=len(values),minimum=float(values.min()),
                median=float(np.median(values)),p95=float(np.percentile(values,95)),
                maximum=float(values.max()))


def cadence(rows,wall_field,stamp_field):
    rows=sorted(rows,key=lambda r:int(r[wall_field]))
    gaps=np.diff([int(r[wall_field]) for r in rows])/1e6
    result=dict(callback_or_update_gap_ms=describe(gaps),gaps_over_250ms=int(np.count_nonzero(gaps>250.)))
    if len(gaps):
        i=int(np.argmax(gaps))
        result['largest_gap']=dict(before_measurement_ns=int(rows[i][stamp_field]),
            after_measurement_ns=int(rows[i+1][stamp_field]),
            before_wall_ns=int(rows[i][wall_field]),after_wall_ns=int(rows[i+1][wall_field]))
    return result


def audit(root):
    motion=json.loads((root/'motion_events.json').read_text())
    events={e['event']:e for e in motion['events']}
    start=events.get('ego_control_handoff')
    if start is None:
        return dict(run=root.name,flight_complete=motion['flight_complete'],fault=motion['fault'],
                    scope='No planner execution; delivery during flight not audited')
    stop=events.get('test_aborted') or events['ego_trajectory_sampled_to_end']
    lo,hi=start['sim_ns'],stop['sim_ns']
    with (root/'livo_timing.csv').open() as stream:timing=list(csv.DictReader(stream))
    with (root/'receipt.csv').open() as stream:receipts=list(csv.DictReader(stream))
    with (root/'frame_samples.csv').open() as stream:frames=list(csv.DictReader(stream))
    lio=[r for r in timing if r['stage']=='LIO' and lo<=int(r['measurement_ns'])<=hi and int(r['odom_publish_ns'])>=0]
    monitor=[r for r in receipts if r['kind']=='odom' and lo<=int(r['stamp_ns'])<=hi]
    frame=[r for r in frames if r['source']=='fast_livo2' and lo<=int(r['measurement_time_ns'])<=hi]
    stages={}
    for stage in ('LIO','VIO'):
        rows=[r for r in timing if r['stage']==stage and lo<=int(r['measurement_ns'])<=hi]
        stages[stage]={k:describe([float(r[k])/1000. for r in rows if r.get(k)])
            for k in ('processing_us','thread_cpu_us','thread_scheduler_wait_us')}
    by_stamp={int(r['measurement_ns']):r for r in lio}
    latency={}
    for name,rows,stamp_field,wall_field in (
        ('monitor',monitor,'stamp_ns','wall_ns'),
        ('frame_auditor',frame,'measurement_time_ns','receipt_monotonic_ns')):
        matched=[(r,by_stamp[int(r[stamp_field])]) for r in rows if int(r[stamp_field]) in by_stamp]
        latency[name]=dict(matched_samples=len(matched),
            callback_minus_lio_end_wall_ms_proxy=describe([
                (int(r[wall_field])-int(t['processing_end_wall_ns']))/1e6 for r,t in matched]))
    monitor_by_stamp={int(r['stamp_ns']):int(r['wall_ns']) for r in monitor}
    cross=[(int(r['receipt_monotonic_ns'])-monitor_by_stamp[int(r['measurement_time_ns'])])/1e6
        for r in frame if int(r['measurement_time_ns']) in monitor_by_stamp]
    measurements=[(int(r['clock_ns'])-int(r['stamp_ns']))/1e6 for r in monitor if int(r['clock_ns'])]
    supervisor=[r for r in motion.get('bridge_samples',[]) if
        'supervisor_odom_receipt_age_sec' in r and lo<=r['sim_ns']<=hi]
    supervisor_detail=dict(available=bool(supervisor),samples=len(supervisor))
    if supervisor:
        for field in ('supervisor_odom_receipt_age_sec','supervisor_odom_measurement_age_sec','bridge_odom_receipt_age_sec'):
            values=[r[field]*1000 for r in supervisor if r.get(field) is not None]
            supervisor_detail[field.replace('_sec','_ms')]=describe(values)
        supervisor_detail['samples_over_250ms_receipt']=sum(r['supervisor_odom_receipt_age_sec']>.25 for r in supervisor)
        supervisor_detail['deferred_odom_callback_count_at_end']=supervisor[-1]['deferred_supervisor_odom_count']
    report=dict(run=root.name,flight_complete=motion['flight_complete'],disarmed=motion['disarmed'],
        fault=motion['fault'],execution_interval_measurement_ns=[lo,hi],processing_stages_ms=stages,
        publisher_lio_end_wall_cadence=cadence(lio,'processing_end_wall_ns','measurement_ns'),
        monitor_callback_cadence=cadence(monitor,'wall_ns','stamp_ns'),
        frame_callback_cadence=cadence(frame,'receipt_monotonic_ns','measurement_time_ns'),
        monitor_measurement_age_at_callback_ms=describe(measurements),
        frame_callback_minus_monitor_callback_ms=describe(cross),callback_after_update=latency,
        supervisor_tracking_samples=supervisor_detail,
        abort_event=events.get('test_aborted'),
        scope='Execution only. processing_end_wall_ns is an upper proxy after LIO publication, '
              'not the exact DDS send timestamp; callback-minus-end may be slightly negative. '
              'Thread CPU, scheduler wait and wall processing are separate measurements. '
              'ROS clock age includes callback ordering and transport. Independent callbacks '
              'do not prove supervisor freshness. Supervisor samples are recorded only from '
              'the instrumented run; no safety thresholds or callbacks are changed.')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run_dir',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    active=active_isolated_simulators()
    if active:p.error('Stop isolated Astra SITL before offline delivery audit; active PIDs: '+','.join(active))
    r=audit(a.run_dir);a.output.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
