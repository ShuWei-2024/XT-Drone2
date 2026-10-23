#!/usr/bin/env python3
"""Measurement-time height audit for a saved guarded SITL motion run."""
import argparse
import bisect
import csv
import json
import statistics
from pathlib import Path

import numpy as np


def read_csv(path):
    with path.open(newline='') as stream:
        return list(csv.DictReader(stream))


def nearest_pairs(source, target, tolerance_ns):
    target = sorted(target, key=lambda row: int(row['measurement_time_ns']))
    stamps = [int(row['measurement_time_ns']) for row in target]
    result = []
    for row in source:
        stamp = int(row['measurement_time_ns'])
        index = bisect.bisect_left(stamps, stamp)
        choices = target[max(0, index-1):min(len(target), index+1)]
        if not choices:
            continue
        match = min(choices, key=lambda item: abs(int(item['measurement_time_ns'])-stamp))
        delta = abs(int(match['measurement_time_ns'])-stamp)
        if delta <= tolerance_ns:
            result.append((row, match, delta/1e6))
    return result


def summarize(values):
    values = np.asarray(values, dtype=float)
    if not len(values):
        return dict(samples=0)
    return dict(samples=int(len(values)), median_m=float(np.median(values)),
                minimum_m=float(np.min(values)), maximum_m=float(np.max(values)),
                absolute_p95_m=float(np.percentile(np.abs(values), 95)))


def audit(run_dir, baseline_start_s=56., baseline_end_s=59.5,
          tolerance_ms=20.):
    run_dir = Path(run_dir)
    rows = read_csv(run_dir/'frame_samples.csv')
    streams = {name: sorted((r for r in rows if r['source'] == name),
                            key=lambda r: int(r['measurement_time_ns']))
               for name in ('gazebo', 'fast_livo2', 'px4')}
    truth = streams['gazebo']
    report = dict(run=run_dir.name, baseline_sim_time_s=[baseline_start_s, baseline_end_s],
        pairing_tolerance_ms=tolerance_ms,
        note='Nearest measurement-time pairs; PX4 NED Z is negated. Ground offsets are frozen from the stated preflight interval. This is descriptive and does not independently validate PX4, which fuses FAST-LIVO2 EV height.')
    errors = {}
    for source in ('fast_livo2', 'px4'):
        pairs = nearest_pairs(streams[source], truth, int(tolerance_ms*1e6))
        to_up = (lambda row: float(row['z_m'])) if source == 'fast_livo2' else (lambda row: -float(row['z_m']))
        baseline = [to_up(est)-float(gt['z_m']) for est, gt, _ in pairs
                    if baseline_start_s <= int(est['measurement_time_ns'])/1e9 <= baseline_end_s]
        if not baseline:
            raise ValueError(f'no {source} pairs in the fixed ground baseline window')
        bias = float(statistics.median(baseline))
        values = [(int(est['measurement_time_ns'])/1e9,
                   to_up(est)-float(gt['z_m'])-bias, delta)
                  for est, gt, delta in pairs]
        errors[source] = values
        report[source] = dict(input_samples=len(streams[source]), paired_samples=len(pairs),
            paired_fraction=len(pairs)/max(1, len(streams[source])),
            measurement_pair_dt_p95_ms=float(np.percentile([p[2] for p in pairs], 95)),
            measurement_pair_dt_max_ms=max(p[2] for p in pairs),
            frozen_ground_bias_m=bias,
            whole_run_error_m=summarize([p[1] for p in values]))

    events = json.loads((run_dir/'motion_events.json').read_text())['events']
    active = None
    phases = []
    for event in events:
        if event['event'] == 'segment_start':
            active = (event.get('name', 'segment'), event['sim_ns']/1e9)
        elif event['event'] == 'segment_end' and active:
            name, start = active
            end = event['sim_ns']/1e9
            phase = dict(name=name, start_sim_s=start, end_sim_s=end, estimators={})
            for source, values in errors.items():
                samples = [value for stamp, value, _ in values if start <= stamp <= end]
                phase['estimators'][source] = summarize(samples)
            phases.append(phase)
            active = None
    report['phases'] = phases
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run_dir', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--baseline', nargs=2, type=float, default=(56., 59.5),
                        metavar=('START_SIM_S', 'END_SIM_S'))
    parser.add_argument('--pair-tolerance-ms', type=float, default=20.)
    args = parser.parse_args()
    result = audit(args.run_dir, *args.baseline, args.pair_tolerance_ms)
    text = json.dumps(result, indent=2)
    print(text)
    if args.output:
        args.output.write_text(text+'\n')


if __name__ == '__main__':
    main()
