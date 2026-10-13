"""Offline ground bias/holdout audit: never fits on the holdout interval."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from audit_obstacle_reliability_20261010 import pair, describe


def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args()
    rows=list(csv.DictReader((a.run/'frame_samples.csv').open()))
    streams={kind:sorted([r for r in rows if r['source']==kind],key=lambda r:int(r['measurement_time_ns']))
             for kind in ('gazebo','fast_livo2','px4')}
    paired=pair(streams['px4'],streams['gazebo'])
    t=np.array([int(p['measurement_time_ns'])/1e9 for p,g in paired]);delta=np.array([float(p['z_m'])+float(g['z_m']) for p,g in paired])
    lo=t[0]+1.;capture=(t>=lo)&(t<lo+4.);heldout=t>=lo+4.;bias=float(np.median(delta[capture]))
    resets=np.array([[int(p[k]) for k in ('xy_reset','z_reset','heading_reset')] for p,g in paired])
    report={'run':str(a.run),'scope':'ground only; Gazebo truth used offline only',
            'frozen_4s_bias_m':bias,'capture_interval_s':[lo,lo+4.],
            'capture_residual_m':describe(np.abs(delta[capture]-bias)),
            'heldout_residual_m':describe(np.abs(delta[heldout]-bias)),
            'heldout_duration_s':float(t[-1]-lo-4.),
            'reset_counter_span':np.ptp(resets,axis=0).tolist(),'blocks':[]}
    for start in np.arange(lo+4.,t[-1],10.):
        m=(t>=start)&(t<start+10.)
        report['blocks'].append({'start_s':float(start),'median_bias_m':float(np.median(delta[m])),
            'residual_m':describe(np.abs(delta[m]-bias))})
    for kind in ('fast_livo2','px4'):
        paired_kind=pair(streams[kind],streams['gazebo'])
        values=np.array([float(p['z_m'])*(1 if kind=='fast_livo2' else -1)-float(g['z_m']) for p,g in paired_kind])
        report[kind+'_raw_height_difference_m']=describe(values)
    (a.run/'ground_height_audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))


if __name__=='__main__':main()
