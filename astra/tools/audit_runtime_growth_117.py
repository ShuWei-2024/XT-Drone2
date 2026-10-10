"""Measure RSS trends and map counters; finite tests cannot prove unlimited stability."""
import argparse
import csv
import json
from pathlib import Path
import re

import numpy as np


def window_stats(times, values):
    duration = float(times[-1] - times[0])
    return dict(samples=len(values), duration_sec=duration,
                first_mib=float(values[0]), last_mib=float(values[-1]),
                minimum_mib=float(values.min()), maximum_mib=float(values.max()),
                delta_mib=float(values[-1] - values[0]),
                fitted_slope_mib_per_min=(float(np.polyfit(times-times[0], values, 1)[0]*60.)
                                          if duration > 0 and len(values) > 1 else None))


def audit(directory):
    with (directory / 'process_resources.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    report = dict(run=directory.name, resources={},
                  scope='Measured RSS on this finite route; slopes include allocator retention '
                        'and map expansion, and do not identify leaks by themselves',
                  unlimited_memory_stability_verified=False)
    for role in sorted({r['role'] for r in rows}):
        matching = [r for r in rows if r['role'] == role]
        pid = max({r['pid'] for r in matching}, key=lambda p: np.median(
            [int(r['rss_kib']) for r in matching if r['pid'] == p]))
        matching = [r for r in matching if r['pid'] == pid]
        times = np.array([int(r['wall_ns']) for r in matching]) / 1e9
        values = np.array([int(r['rss_kib']) for r in matching]) / 1024.
        stats = window_stats(times, values)
        stats['pid'] = int(pid)
        stats['last_windows'] = {}
        for seconds in (60, 120, 300):
            mask = times >= times[-1]-seconds
            stats['last_windows'][str(seconds)] = window_stats(times[mask], values[mask])
        stats['minute_blocks'] = []
        for begin in np.arange(times[0], times[-1]+.01, 60):
            mask = (times >= begin) & (times < begin+60)
            if not mask.any():continue
            block = window_stats(times[mask], values[mask])
            block['start_elapsed_sec'] = float(begin-times[0])
            stats['minute_blocks'].append(block)
        report['resources'][role] = stats
    diagnostics = []
    pattern = r'(lidar_voxels|visual_voxels|visual_points|observations|retained_images|image_MiB|path_poses|visual_images_pruned|visual_observations_pruned)=(\d+(?:\.\d+)?)'
    lio_rows = list(csv.DictReader((directory / 'livo_timing.csv').open()))
    lio_rows = [r for r in lio_rows if r['stage'] == 'LIO']
    for line in (directory / 'livo.log').read_text().splitlines():
        if 'retained_images=' not in line:continue
        counters = {k: float(v) for k, v in re.findall(pattern, line)}
        count = re.search(r'processing LIO=(\d+)', line)
        # Counters report completed LIO frames. Associate them with the recorded
        # end of that update; this is not the later diagnostic publication time.
        if count and 0 < int(count[1]) <= len(lio_rows):
            update = lio_rows[int(count[1])-1]
            counters['measurement_time_sec'] = int(update['measurement_ns'])/1e9
        diagnostics.append(counters)
    report['map_counter_samples'] = diagnostics
    if diagnostics:
        report['first_map_counters'] = diagnostics[0]
        report['last_map_counters'] = diagnostics[-1]
        report['counter_ranges'] = {k: {'minimum':min(x[k] for x in diagnostics if k in x),
                                       'maximum':max(x[k] for x in diagnostics if k in x)}
                                    for k in diagnostics[-1] if k!='measurement_time_sec'}
    return report


def plot(directory, report, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    with (directory/'process_resources.csv').open() as stream:
        rows=list(csv.DictReader(stream))
    fig,axes=plt.subplots(3,1,figsize=(9,8),sharex=True)
    for role,stats in report['resources'].items():
        active=[r for r in rows if r['role']==role and int(r['pid'])==stats['pid']
                and int(r['clock_ns'])>0]
        axes[0].plot([int(r['clock_ns'])/1e9 for r in active],
                     [int(r['rss_kib'])/1024. for r in active],label=role)
    counters=[r for r in report['map_counter_samples'] if 'measurement_time_sec' in r]
    times=[r['measurement_time_sec'] for r in counters]
    axes[1].plot(times,[r['image_MiB'] for r in counters],label='Retained grayscale images')
    for key in ('lidar_voxels','visual_points','observations'):
        axes[2].plot(times,[r[key] for r in counters],label=key)
    axes[0].set_ylabel('Process RSS (MiB)')
    axes[1].set_ylabel('Image data (MiB)')
    axes[2].set_ylabel('Stored entries')
    axes[2].set_xlabel('Simulation time (s)')
    for ax in axes:
        ax.grid(alpha=.3);ax.legend(fontsize=8)
    axes[0].set_title('PX4 1.17 repeated route: memory and map counters')
    fig.tight_layout();fig.savefig(output,dpi=150);plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--plot',type=Path)
    args = parser.parse_args()
    report = audit(args.run_dir)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    if args.plot:plot(args.run_dir,report,args.plot)
    print(json.dumps({k:v for k,v in report.items() if k!='map_counter_samples'},indent=2))
