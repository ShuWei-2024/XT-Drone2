"""Reconstruct acceptance failures from saved measurements; never controls SITL."""
import argparse
from bisect import bisect_left
import csv
import json
import math
from pathlib import Path
import statistics
import sys


def read_rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def describe(values):
    values = sorted(float(v) for v in values)
    if not values:
        return dict(samples=0)
    index = .95 * (len(values) - 1)
    low = math.floor(index)
    high = math.ceil(index)
    return dict(samples=len(values), minimum=values[0], median=statistics.median(values),
                p95=values[low] + (values[high] - values[low]) * (index - low),
                maximum=values[-1])


class Measurements:
    def __init__(self, rows):
        self.rows = sorted(rows, key=lambda row: int(row['measurement_time_ns']))
        self.stamps = [int(row['measurement_time_ns']) for row in self.rows]

    def nearest(self, stamp, tolerance_ns=20_000_000):
        i = bisect_left(self.stamps, stamp)
        choices = [j for j in (i - 1, i) if 0 <= j < len(self.rows)]
        if not choices:
            return None
        j = min(choices, key=lambda j: abs(self.stamps[j] - stamp))
        return self.rows[j] if abs(self.stamps[j] - stamp) <= tolerance_ns else None


def largest_gap(rows, field):
    rows = sorted(rows, key=lambda r: int(r[field]))
    if len(rows) < 2:
        return dict(samples=len(rows))
    a, b = max(zip(rows, rows[1:]), key=lambda pair: int(pair[1][field]) - int(pair[0][field]))
    return dict(samples=len(rows), gap_ms=(int(b[field]) - int(a[field])) / 1e6,
                before=a, after=b)


def flight_events(root):
    motion = json.loads((root / 'motion_events.json').read_text())
    events = {row['event']: row for row in motion['events']}
    stop = events.get('test_aborted') or events['ego_trajectory_sampled_to_end']
    return motion, events, events['ego_control_handoff'], stop


def delivery(root):
    motion, _, start, stop = flight_events(root)
    rows = [r for r in read_rows(root / 'livo_timing.csv')
            if r['stage'] == 'LIO' and start['sim_ns'] <= int(r['measurement_ns']) <= stop['sim_ns']
            and r.get('odom_output_status') == 'published']
    fields = ('measurement_ns', 'processing_us', 'thread_cpu_us', 'thread_scheduler_wait_us',
              'processing_start_wall_ns', 'processing_end_wall_ns', 'odom_publish_end_wall_ns')
    cadence_rows = [{key: row[key] for key in fields} for row in rows]
    # A measurement captured before the abort can finish publishing after it.
    # Keep that row to reconstruct the pending frame, and report this explicitly.
    exact = largest_gap(cadence_rows, 'odom_publish_end_wall_ns')
    report = dict(run=root.name, fault=motion['fault'],
                  exact_publish_call_end_cadence=exact,
                  processing_end_proxy_cadence=largest_gap(cadence_rows, 'processing_end_wall_ns'),
                  processing_wall_ms=describe(float(r['processing_us']) / 1000 for r in rows),
                  thread_cpu_ms=describe(float(r['thread_cpu_us']) / 1000 for r in rows),
                  scheduler_wait_ms=describe(float(r['thread_scheduler_wait_us']) / 1000 for r in rows),
                  scope='Execution measurement window; may include publication after abort. '
                        'Publish-call end is instrumented in-process, not DDS wire time.')
    abort = stop if stop['event'] == 'test_aborted' else None
    if abort is None or 'measurement age' not in motion['fault']:
        return report
    after = exact['after']
    report['abort'] = abort
    report['next_publish_end_relative_to_abort_ms'] = (
        int(after['odom_publish_end_wall_ns']) - abort['wall_ns']) / 1e6
    inputs = read_rows(root / 'input_audit.csv')
    publish = next((r for r in inputs if r['event'] == 'publish_lidar_start'
                    and int(r['image_ns']) == int(after['measurement_ns'])), None)
    if publish:
        lidar = next(r for r in inputs if r['event'] == 'lidar_buffered'
                     and r['stamp_ns'] == publish['stamp_ns'])
        image = next(r for r in inputs if r['event'] == 'image_enter'
                     and int(r['stamp_ns']) == int(publish['image_ns']))
        chain = [int(lidar['wall_ns']), int(image['wall_ns']), int(publish['wall_ns']),
                 int(after['processing_start_wall_ns']), int(after['odom_publish_end_wall_ns'])]
        report['pending_frame_chain'] = dict(
            lidar_stamp_ns=int(lidar['stamp_ns']), image_stamp_ns=int(image['stamp_ns']),
            lidar_buffer_to_image_callback_ms=(chain[1]-chain[0])/1e6,
            image_callback_to_sync_publish_ms=(chain[2]-chain[1])/1e6,
            sync_publish_to_lio_start_ms=(chain[3]-chain[2])/1e6,
            lio_start_to_odom_publish_end_ms=(chain[4]-chain[3])/1e6,
            lidar_buffer_to_odom_publish_end_ms=(chain[4]-chain[0])/1e6,
            image_callback_measurement_age_ms=(int(image['clock_ns'])-int(image['stamp_ns']))/1e6,
            image_callback_age_scope='Callback-local ROS clock; not exact DDS latency',
            lio_total_wall_ms=float(after['processing_us'])/1000,
            lio_thread_cpu_ms=float(after['thread_cpu_us'])/1000,
            lio_scheduler_wait_ms=float(after['thread_scheduler_wait_us'])/1000)
    nearby = [r for r in inputs if abs(int(r['wall_ns'])-abort['wall_ns']) <= 350_000_000]
    gaps = {kind: largest_gap([r for r in nearby if r['event'] == kind], 'wall_ns')
            for kind in ('sync_enter', 'imu_enter', 'image_enter', 'lidar_enter')}
    report['adapter_callback_gaps'] = gaps
    gap = gaps['imu_enter']
    if 'before' in gap:
        lo, hi = int(gap['before']['wall_ns']), int(gap['after']['wall_ns'])
        counts = {}
        for row in read_rows(root / 'receipt.csv'):
            if lo <= int(row['wall_ns']) <= hi:
                counts[row['kind']] = counts.get(row['kind'], 0) + 1
        report['independent_monitor_during_adapter_imu_gap'] = counts
        timing = json.loads((root / 'executor_timing_audit.json').read_text())
        report['execution_process_gc_overlapping_adapter_gap'] = [r for r in timing['gc_tail']
            if r['wall_start_ns'] <= hi and r['wall_end_ns'] >= lo]
    report['limits'] = ['Adapter GC, adapter thread scheduling, GIL and DDS arrival were not recorded.',
                        'Execution-process GC evidence does not exclude GC in the adapter process.',
                        'Independent telemetry continued; this does not identify the adapter stall cause.']
    return report


def height(root):
    motion, events, start, stop = flight_events(root)
    rows = read_rows(root / 'frame_samples.csv')
    streams = {kind: Measurements(r for r in rows if r['source'] == kind)
               for kind in ('gazebo', 'fast_livo2', 'px4')}
    ground_end = events['ground_height_alignment_complete']['sim_ns']
    # Freeze a descriptive truth reference while grounded, never fit it in flight.
    ground = []
    for row in streams['fast_livo2'].rows:
        t = int(row['measurement_time_ns'])
        if ground_end-2_000_000_000 <= t <= ground_end:
            truth = streams['gazebo'].nearest(t)
            if truth:
                ground.append(float(row['z_m'])-float(truth['z_m']))
    if not ground:
        raise ValueError('No measurement-paired ground truth reference: '+root.name)
    livo_bias = statistics.median(ground)
    offset = motion['ground_height_alignment']['offset_m']
    # Preserve the very same ground-frozen estimator offset used by the supervisor.
    px4_bias = livo_bias-offset
    samples = []
    for row in streams['fast_livo2'].rows:
        t = int(row['measurement_time_ns'])
        if not start['sim_ns'] <= t <= stop['sim_ns']:
            continue
        if int(row['receipt_monotonic_ns']) > stop['wall_ns']:
            continue
        px4, truth = streams['px4'].nearest(t), streams['gazebo'].nearest(t)
        if px4 is None or truth is None:
            continue
        lz, pz, gz = (float(r['z_m']) for r in (row, px4, truth))
        samples.append(dict(measurement_ns=t, px4_pair_dt_ms=(int(px4['measurement_time_ns'])-t)/1e6,
            truth_pair_dt_ms=(int(truth['measurement_time_ns'])-t)/1e6,
            frozen_height_residual_m=lz+pz-offset,
            livo_ground_referenced_error_m=lz-gz-livo_bias,
            px4_ground_referenced_error_m=-pz-gz-px4_bias))
    if not samples:
        raise ValueError('No execution height pairs: '+root.name)
    timing = [r for r in read_rows(root / 'livo_timing.csv')
              if start['sim_ns'] <= int(r['measurement_ns']) <= stop['sim_ns']
              and int(r['processing_end_wall_ns']) <= stop['wall_ns']]
    stages = {}
    for stage in ('LIO', 'VIO'):
        stage_rows = [r for r in timing if r['stage'] == stage]
        stages[stage] = dict(samples=len(stage_rows), absolute_z_correction_m=describe(
            abs(float(r['update_z'])-float(r['imu_z'])) for r in stage_rows))
        field = 'lio_correspondences' if stage == 'LIO' else 'vio_accepted_patches'
        stages[stage][field] = describe(float(r[field]) for r in stage_rows if r.get(field))
    progress = []
    for lo in range(start['sim_ns'], stop['sim_ns']+1, 60_000_000_000):
        block = [r for r in samples if lo <= r['measurement_ns'] < lo+60_000_000_000]
        if block:
            progress.append(dict(interval_ns=[lo, min(lo+60_000_000_000, stop['sim_ns'])],
                livo_error_m=describe(r['livo_ground_referenced_error_m'] for r in block),
                px4_error_m=describe(r['px4_ground_referenced_error_m'] for r in block),
                frozen_height_residual_m=describe(r['frozen_height_residual_m'] for r in block)))
    return dict(run=root.name, fault=motion['fault'], ground_reference_interval_ns=[ground_end-2_000_000_000, ground_end],
        frozen_supervisor_offset_m=offset, livo_truth_bias_m=livo_bias, px4_truth_bias_m=px4_bias,
        last_received_measurement=samples[-1],
        decomposition_identity_max_error_m=max(abs(r['frozen_height_residual_m']-(
            r['livo_ground_referenced_error_m']-r['px4_ground_referenced_error_m'])) for r in samples),
        absolute_height_residual_m=describe(abs(r['frozen_height_residual_m']) for r in samples),
        stages=stages, progress_blocks=progress,
        scope='Nearest measurement pairs <=20 ms, same truth sample in both error terms; '
              'fixed ground truth reference plus recorded supervisor offset, no airborne refit. '
              'Estimator output/body lever-arm dynamics and pairing uncertainty remain; '
              'correction magnitudes and correspondence counts do not establish causality.')


def origin(root):
    report = json.loads((root / 'global_origin.json').read_text())
    summary = json.loads((root / 'run_summary.json').read_text())
    sys.path.append(str(Path.home() / 'dev/px4/venv/lib/python3.12/site-packages'))
    from pyulog import ULog
    ulog = ULog(str(next((root / 'log').rglob('*.ulg'))), message_name_filter_list=[
        'vehicle_local_position', 'vehicle_status', 'estimator_aid_src_ev_hgt'])
    data = {r.name: r.data for r in ulog.data_list if r.multi_id == 0}
    local = data['vehicle_local_position']
    lo = report['acks'][0]['timestamp_us']
    hi = report['failure_input_snapshot']['sim_us']
    points = []
    for i, stamp in enumerate(local['timestamp']):
        if lo <= stamp <= hi:
            xyz = [float(local[k][i]) for k in ('x', 'y', 'z')]
            points.append(dict(publication_us=int(stamp), sample_us=int(local['timestamp_sample'][i]),
                xyz_m=xyz, displacement_m=math.dist(report['before_local_ned_m'], xyz),
                ref_alt_m=float(local['ref_alt'][i]),
                resets=[int(local[k][i]) for k in ('xy_reset_counter', 'z_reset_counter', 'heading_reset_counter')]))
    crossings = [r for r in points if r['displacement_m'] > .03]
    status = data['vehicle_status']
    armed = [int(t) for t, state in zip(status['timestamp'], status['arming_state']) if state == 2]
    aid = data['estimator_aid_src_ev_hgt']
    observations = []
    for lower, upper in ((lo-200_000, lo), (hi-200_000, hi)):
        values = [float(v) for t, v in zip(aid['timestamp'], aid['observation']) if lower <= t <= upper]
        observations.append(describe(values))
    return dict(run=root.name, failure=summary['failure'], before_xyz_m=report['before_local_ned_m'],
        origin_altitude_m=report['proposed_origin_lla'][2], first_logged_crossing=crossings[0] if crossings else None,
        last_logged_local_before_failure=points[-1] if points else None,
        ev_height_observation_before_after_m=observations, armed_samples=len(armed),
        scope='Saved ULog after command ACK through failure snapshot; no command or parameter change. '
              'First logged crossing need not equal exact callback that tripped the gate. '
              'EV observation datum change is not itself a physical vehicle displacement.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime-root', type=Path, default=Path('astra/runtime'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    roots = {n: args.runtime_root / f'acceptance_baylands_20261021_{n:02d}' for n in (1, 2, 3, 4, 5, 6)}
    result = dict(scope='Offline evidence only; preserves archived records and all flight gates; no new flight.',
                  delivery=[delivery(roots[n]) for n in (1, 2, 3, 5, 6)],
                  height=[height(roots[n]) for n in (1, 2, 3, 5, 6)], origin=origin(roots[4]))
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(args.output), delivery_runs=len(result['delivery']),
                         height_runs=len(result['height']), origin_run=result['origin']['run'])))


if __name__ == '__main__':
    main()
