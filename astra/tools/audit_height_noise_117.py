"""Decompose a frozen-height guard event using measurement-time-paired data."""
import argparse
import csv
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from audit_obstacle_reliability_20261010 import pair

sys.path.append(str(Path.home()/'dev/px4/venv/lib/python3.12/site-packages'))
from pyulog import ULog


def audit(directory):
    motion=json.loads((directory/'motion_events.json').read_text())
    events={r['event']:r for r in motion['events']}
    begin=events['ego_control_handoff']['sim_ns']/1e9
    end=(events.get('test_aborted') or events['ego_trajectory_sampled_to_end'])['sim_ns']/1e9
    with (directory/'frame_samples.csv').open() as stream:rows=list(csv.DictReader(stream))
    streams={kind:sorted([r for r in rows if r['source']==kind],
                         key=lambda r:int(r['measurement_time_ns']))
             for kind in ('gazebo','fast_livo2','px4')}
    paired=pair([r for r in streams['fast_livo2']
                 if begin<=int(r['measurement_time_ns'])/1e9<=end],streams['px4'])
    ground=motion['ground_height_alignment']['offset_m']
    samples=[]
    for livo,px4 in paired:
        stamp=int(livo['measurement_time_ns'])
        difference=float(livo['z_m'])+float(px4['z_m'])-ground
        if abs(difference)>.15:
            truth=pair([livo],streams['gazebo'])[0][1]
            samples.append(dict(measurement_sec=stamp/1e9,
                                pair_dt_ms=abs(stamp-int(px4['measurement_time_ns']))/1e6,
                                livo_z_m=float(livo['z_m']),px4_z_ned_m=float(px4['z_m']),
                                truth_z_m=float(truth['z_m']),
                                frozen_ground_height_residual_m=difference))
    ulog=ULog(str(next((directory/'log').rglob('*.ulg'))))
    data={r.name:r.data for r in ulog.data_list}
    truth=streams['gazebo'];tt=np.array([int(r['measurement_time_ns'])/1e9 for r in truth])
    z=np.array([float(r['z_m']) for r in truth])
    tree=ET.parse(directory/'models/x500_base/model.sdf')
    sigma=float(tree.find('.//air_pressure/pressure/noise/stddev').text)
    x=data['vehicle_air_data'];t=x['timestamp_sample'].astype(float)/1e6
    active=(t>=begin)&(t<=end)
    # Gazebo air pressure includes a fixed atmosphere reference and noise.
    # Remove a mean only for estimating scatter; never apply it to flight guards.
    residual=x['baro_alt_meter'][active]-np.interp(t[active],tt,z)
    aid=data['estimator_aid_src_baro_hgt']
    aid_t=aid['timestamp_sample'].astype(float)/1e6;aid_active=(aid_t>=begin)&(aid_t<=end)
    parameters={key:ulog.initial_parameters.get(key) for key in (
        'EKF2_BARO_NOISE','EKF2_BARO_DELAY','EKF2_HGT_REF','EKF2_GPS_CTRL')}
    return dict(run=directory.name,flight_complete=motion['flight_complete'],fault=motion['fault'],
                interval_sec=[begin,end],parameters=parameters,
                configured_gazebo_pressure_sigma_pa=sigma,
                near_sea_level_equivalent_height_sigma_m=sigma/(1.225*9.80665),
                barometric_height_minus_truth=dict(samples=len(residual),
                    scatter_stddev_m=float(residual.std()),
                    absolute_deviation_from_median_p95_m=float(np.percentile(abs(residual-np.median(residual)),95))),
                fused_observation_sigma_m=float(np.sqrt(np.median(aid['observation_variance'][aid_active]))),
                baro_rejected_updates=int(np.count_nonzero(aid['innovation_rejected'][aid_active])),
                frozen_ground_height_over_015m_samples=samples,
                scope='Measurement-aligned decomposition; barometric scatter includes filtering '
                      'and is not a pure white noise estimate; no refit of flight alignment')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();report=audit(args.run_dir)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if not k.endswith('_samples')},indent=2))
