"""Offline audit of bounded executor traces; never run alongside Astra SITL."""
import argparse
import csv
import json
from pathlib import Path

from audit_odom_delivery_117 import describe
from audit_height_dynamics_117 import active_isolated_simulators


def audit(root):
    trace=json.loads((root/'motion_executor_audit.json').read_text())
    motion=json.loads((root/'motion_events.json').read_text())
    start=next((e for e in motion['events'] if e['event']=='ego_control_handoff'),None)
    stop=next((e for e in motion['events'] if e['event']=='test_aborted'),None)
    stop=stop or next((e for e in motion['events'] if e['event']=='ego_trajectory_sampled_to_end'),None)
    report=dict(run=root.name,flight_complete=motion['flight_complete'],fault=motion['fault'],
        executor=motion['test_configuration'].get('execution_executor','original-single-threaded'),
        scope='Flight execution only; batch metrics cover the retained tail, not the entire flight. '
              'CPU fraction is receive-batch thread CPU / elapsed tail wall time, not total host CPU. '
              'Independent callback time differences are scheduling/transport proxies, not DDS queue latency.')
    report['gc_statistics']=trace.get('gc_statistics',{})
    report['gc_tail']=trace.get('gc_tail',[])
    if not start or not stop:return report
    lo,hi=start['wall_ns'],stop['wall_ns']
    batches=[b for b in trace['batch_tail'] if lo<=b['wall_start_ns'] and b['wall_end_ns']<=hi]
    if batches:
        span=batches[-1]['wall_end_ns']-batches[0]['wall_start_ns']
        report['batch_tail']=dict(samples=len(batches),span_sec=span/1e9,
            receive_thread_cpu_fraction=sum(b['cpu_duration_ns'] for b in batches)/span,
            wall_ms=describe([b['wall_duration_ns']/1e6 for b in batches]),
            cpu_ms=describe([b['cpu_duration_ns']/1e6 for b in batches]),
            scheduler_wait_ms=describe([b['scheduler_wait_ns']/1e6 for b in batches if b['scheduler_wait_ns'] is not None]),
            spin_calls=describe([b['spin_calls'] for b in batches]),
            budget_exhausted_fraction=sum(b['budget_exhausted'] for b in batches)/len(batches))
    callbacks=[r for r in trace['odom_callbacks'] if lo<=r['wall_start_ns']<=hi]
    with (root/'receipt.csv').open() as f:
        monitor={int(r['stamp_ns']):int(r['wall_ns']) for r in csv.DictReader(f) if r['kind']=='odom'}
    groups={label:{r['measurement_ns']:r for r in callbacks if r['label']==label}
            for label in sorted({r['label'] for r in callbacks})}
    report['odom_callbacks']={}
    for label,by_stamp in groups.items():
        rows=sorted(by_stamp.values(),key=lambda r:r['wall_start_ns'])
        gaps=[(b['wall_start_ns']-a['wall_start_ns'])/1e6 for a,b in zip(rows,rows[1:])]
        metadata=[r for r in rows if r.get('dds_received_system_ns',0)>0]
        report['odom_callbacks'][label]=dict(samples=len(rows),callback_gap_ms=describe(gaps),
            gaps_over_250ms=sum(g>250 for g in gaps),
            callback_wall_ms=describe([r['wall_duration_ns']/1e6 for r in rows]),
            dds_metadata_samples=len(metadata),
            dds_received_to_callback_ms=describe([(r['callback_start_system_ns']-r['dds_received_system_ns'])/1e6 for r in metadata]),
            callback_minus_monitor_ms_proxy=describe([(r['wall_start_ns']-monitor[r['measurement_ns']])/1e6 for r in rows if r['measurement_ns'] in monitor]))
    if groups:
        common=set.intersection(*(set(g) for g in groups.values()))
        report['same_measurement_cross_node_callback_spread_ms']=describe([
            (max(g[s]['wall_start_ns'] for g in groups.values())-
             min(g[s]['wall_start_ns'] for g in groups.values()))/1e6 for s in common])
    report['dds_metadata_limitation']=('Installed experimental EventsExecutor does not supply MessageInfo; '
        'DDS queue latency cannot be certified from this trace.' if not any(r.get('dds_received_system_ns',0)>0 for r in callbacks) else None)
    report['restoration']=json.loads((root/'run_summary.json').read_text())
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run_dir',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    active=active_isolated_simulators()
    if active:p.error('Stop Astra SITL before offline audit; active PIDs: '+','.join(active))
    result=audit(a.run_dir)
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
