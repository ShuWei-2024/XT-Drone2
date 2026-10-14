#!/usr/bin/env python3
"""Offline EV acceptance evidence from received uORB samples and EKF aid sources."""
import argparse
import bisect
import csv
import json
import sys
from pathlib import Path
import numpy as np
from audit_height_dynamics_117 import active_isolated_simulators


def stats(values):
    a = np.asarray(values, float)
    return dict(count=len(a), p95=float(np.percentile(a, 95)), maximum=float(a.max())) if len(a) else {}


def audit(directory):
    sys.path.append(str(Path.home()/'dev/px4/venv/lib/python3.12/site-packages'))
    from pyulog import ULog
    runtime = json.loads((directory/'runtime.json').read_text())
    if not runtime.get('external_vision_enabled'): raise ValueError('Not an EV experiment')
    bridge = json.loads((directory/'external_vision.json').read_text())
    sent = list(csv.DictReader((directory/'external_vision_sent.csv').open()))
    logs = list((directory/'log').rglob('*.ulg'))
    if not logs: raise ValueError('No PX4 ULog; cannot confirm actual fusion')
    u = ULog(str(logs[-1])); data = {d.name: d.data for d in u.data_list if d.multi_id == 0}
    status = data.get('vehicle_status', {})
    ts = status.get('timestamp', np.array([])); armed = status.get('arming_state', np.array([])) == 2
    def in_flight(t):
        if not len(ts): return np.zeros(len(t), bool)
        i = np.searchsorted(ts, t, side='right')-1
        return (i >= 0) & armed[np.maximum(i, 0)]
    aid = {}
    for name in ['estimator_aid_src_ev_pos', 'estimator_aid_src_ev_hgt']:
        d = data.get(name)
        if d is None: aid[name] = dict(recorded=False); continue
        mask = in_flight(d['timestamp'])
        view = mask if mask.any() else np.ones(len(mask), bool)
        last = d['time_last_fuse'][view]
        fused = d['fused'][view].astype(bool)
        rejected = d['innovation_rejected'][view].astype(bool)
        keys = [k for k in d if k == 'innovation' or k.startswith('innovation[')]
        aid[name] = dict(recorded=True, scope='armed' if mask.any() else 'ground',
            count=int(view.sum()), fused_count=int(fused.sum()),
            fused_fraction=float(fused.mean()) if len(fused) else 0.,
            rejected_count=int(rejected.sum()), last_fuse_advanced_count=int((np.diff(last)>0).sum()),
            maximum_fusion_age_ms=float(np.max((d['timestamp'][view][last>0].astype(float)-last[last>0])/1000)) if (last>0).any() else None,
            innovation_abs_m=stats(np.concatenate([np.abs(d[k][view]) for k in keys])) if keys else {})
    received = data.get('vehicle_visual_odometry')
    recorded = set(int(v) for v in received['timestamp_sample']) if received is not None else set()
    stamp_match = sum(int(r['timestamp_sample_us']) in recorded for r in sent)
    checks = dict(received_in_px4=bool(recorded),
        both_aid_sources_actually_fused=all(a.get('fused_count', 0)>0 and a.get('last_fuse_advanced_count', 0)>0 for a in aid.values()),
        sent_measurements_not_restamped=bool(sent) and all(0 <= float(r['measurement_age_ms']) <= 250 for r in sent),
        source_jump_not_detected=not bridge['source_fault'])
    agreement = {}
    if (directory/'frame_samples.csv').exists() and (directory/'external_vision_alignment.json').exists():
        alignment=json.loads((directory/'external_vision_alignment.json').read_text())
        rows=list(csv.DictReader((directory/'frame_samples.csv').open()))
        px4=sorted([r for r in rows if r['source']=='px4'],key=lambda r:int(r['measurement_time_ns']))
        times=[int(r['measurement_time_ns']) for r in px4]
        measurements=[]; errors=[]; yaw_errors=[]; paired_dt=[]
        total=0
        for r in rows:
            if r['source']!='fast_livo2': continue
            stamp=int(r['measurement_time_ns'])
            if stamp<alignment['frozen_measurement_us']*1000: continue
            total+=1
            i=bisect.bisect_left(times,stamp)
            choices=px4[max(0,i-1):min(len(px4),i+1)]
            if not choices: continue
            match=min(choices,key=lambda v:abs(int(v['measurement_time_ns'])-stamp))
            dt=abs(int(match['measurement_time_ns'])-stamp)
            if dt>20_000_000: continue
            source=np.array([float(r[k]) for k in ('x_m','y_m','z_m')])
            expected=np.array([source[1],source[0],-source[2]])+alignment['translation_ned_m']
            errors.append(np.array([float(match[k]) for k in ('x_m','y_m','z_m')])-expected)
            angle=np.pi/2-float(r['yaw_rad'])-float(match['yaw_rad'])
            yaw_errors.append(abs(np.degrees(np.arctan2(np.sin(angle),np.cos(angle)))))
            measurements.append(stamp/1000); paired_dt.append(dt/1e6)
        errors=np.array(errors)
        armed_pairs=in_flight(np.array(measurements))
        view=armed_pairs if armed_pairs.any() else np.ones(len(armed_pairs),bool)
        if len(errors):
            agreement=dict(reference='single EV ground-frozen ENU/NED translation; no airborne fit',
                scope='armed' if armed_pairs.any() else 'ground',
                paired_measurements=len(errors), pairing_fraction=len(errors)/total if total else 0.,
                matched_measurement_dt_ms=stats(paired_dt),
                xy_m=stats(np.linalg.norm(errors[view,:2],axis=1)),
                z_m=stats(np.abs(errors[view,2])),
                yaw_disagreement_deg=stats(np.array(yaw_errors)[view]))
    result = dict(checks=checks, fusion_confirmed=all(checks.values()), aid_sources=aid,
        ground_frozen_livo_px4_agreement=agreement,
        published_measurements=len(sent), received_measurements=len(recorded),
        sent_stamps_present_in_rate_limited_ulog=stamp_match,
        measurement_age_ms=stats([float(r['measurement_age_ms']) for r in sent]),
        source_sample_gap_ms=stats(np.diff([int(r['timestamp_sample_us']) for r in sent])/1000),
        bridge_armed_ev_pos_hgt_fraction=bridge.get('armed_ev_pos_hgt_fraction'),
        reset_events=bridge['resets'], parameter_overrides=runtime['parameter_overrides'],
        scope='Position-only isolated SITL; no yaw/velocity EV fusion, no hardware certification')
    (directory/'external_vision_audit.json').write_text(json.dumps(result, indent=2)+'\n')
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('run_dir', type=Path)
    a = p.parse_args()
    if active_isolated_simulators(): p.error('Stop simulation before offline ULog analysis')
    print(json.dumps(audit(a.run_dir), indent=2))
