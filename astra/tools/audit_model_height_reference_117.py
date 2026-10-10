"""Separate mounting geometry from frozen-reference height changes offline."""
import argparse
import csv
import json
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from audit_obstacle_reliability_20261010 import pair
sys.path.append(str(Path.home()/'dev/px4/venv/lib/python3.12/site-packages'))
from pyulog import ULog


def transform(element):
    values = [float(x) for x in element.findtext('pose', '0 0 0 0 0 0').split()]
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler('xyz', values[3:]).as_matrix()
    matrix[:3, 3] = values[:3]
    return matrix


def describe(values):
    return dict(signed_median_m=float(np.median(values)),
                signed_min_m=float(values.min()), signed_max_m=float(values.max()),
                absolute_p95_m=float(np.percentile(abs(values), 95)))


def audit(directory):
    runtime = json.loads((directory/'runtime.json').read_text())
    env = os.environ.copy()
    env['SDF_PATH'] = str(directory.resolve()/'models') + ':' + str(
        Path(runtime['px4'])/'Tools/simulation/gz/models')
    expanded = subprocess.run(['gz', 'sdf', '-p', str(directory/'models/astra_x500_px4/model.sdf')],
                              env=env, text=True, capture_output=True, check=True)
    model = ET.fromstring(expanded.stdout).find('model')
    elements = {e.attrib['name']: e for e in model if e.tag in ('frame', 'link')}
    resolved = {'__model__': np.eye(4)}

    def resolve(name):
        if name not in resolved:
            element = elements[name]
            pose = element.find('pose')
            parent = pose.get('relative_to', '__model__') if pose is not None else '__model__'
            resolved[name] = resolve(parent) @ transform(element)
        return resolved[name]

    body, imu, lidar_link = [resolve(k) for k in ('base_link', 'imu_link', 'mid360_link')]
    lidar = lidar_link @ transform(elements['mid360_link'].find("sensor[@name='mid360_equivalent']"))
    camera = resolve('camera_color_optical_frame')
    expected_imu_lidar = np.linalg.inv(imu) @ lidar
    expected_camera_lidar = np.linalg.inv(camera) @ lidar
    config_path = directory/'estimator_config.yaml'
    if not config_path.exists():
        config_path = Path(__file__).resolve().parents[1]/'config/estimator_self_filtered.yaml'
    params = yaml.safe_load(config_path.read_text())['/**']['ros__parameters']
    config = params['extrin_calib']
    motion = json.loads((directory/'motion_events.json').read_text())
    events = {r['event']: r for r in motion['events']}
    lo = events['ego_control_handoff']['sim_ns']
    hi = (events.get('test_aborted') or events['ego_trajectory_sampled_to_end'])['sim_ns']
    with (directory/'frame_samples.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    streams = {kind: sorted([r for r in rows if r['source'] == kind],
                            key=lambda r: int(r['measurement_time_ns']))
               for kind in ('fast_livo2', 'px4')}
    paired = pair([r for r in streams['fast_livo2']
                   if lo <= int(r['measurement_time_ns']) <= hi], streams['px4'])
    raw = np.array([float(a['z_m'])+float(b['z_m']) for a, b in paired])
    ground, airborne = motion['ground_height_alignment']['offset_m'], motion['alignment']['z_translation']
    ulog = ULog(str(next((directory/'log').rglob('*.ulg'))))
    attitude = next(r.data for r in ulog.data_list if r.name == 'vehicle_attitude')
    mask = (attitude['timestamp_sample']*1000 >= lo) & (attitude['timestamp_sample']*1000 <= hi)
    rzz = 1-2*(attitude['q[1]'][mask]**2+attitude['q[2]'][mask]**2)
    lever = (np.linalg.inv(body) @ imu)[:3, 3]
    if np.linalg.norm(lever[:2]) > 1e-8:
        raise ValueError('This vertical lever calculation requires a pure-Z mount')
    return dict(run=directory.name,
        estimator_config_source=str(config_path),
        published_reference_translation_imu_m=params.get('publish',{}).get('reference_translation_imu',[0.,0.,0.]),
        published_odom_child_frame=params.get('publish',{}).get('odom_child_frame','aft_mapped'),
        geometry_model_frame_m={
            'px4_imu_base_link': body[:3, 3].tolist(),
            'livo_imu': imu[:3, 3].tolist(),
            'lidar_link': lidar_link[:3, 3].tolist(),
            'lidar_scan_origin': lidar[:3, 3].tolist()},
        livo_imu_relative_to_base_m=lever.tolist(),
        expected_lidar_to_imu_translation_m=expected_imu_lidar[:3, 3].tolist(),
        configured_lidar_to_imu_translation_m=config['extrinsic_T'],
        expected_lidar_to_camera_optical_translation_m=expected_camera_lidar[:3, 3].tolist(),
        configured_lidar_to_camera_translation_m=config['Pcl'],
        convention='p_imu=R_IL*p_lidar+t_IL as used by transformLidar; expected geometry '
                   'assumes PointCloud2 XYZ originate at the GPU lidar scan origin. '
                   'Adapter copies XYZ without translation.',
        timestamp_paired_samples=len(paired),
        frozen_ground_offset_m=ground, frozen_airborne_offset_m=airborne,
        airborne_minus_ground_offset_m=airborne-ground,
        execution_height_difference_after_ground_offset=describe(raw-ground),
        execution_height_difference_after_airborne_offset=describe(raw-airborne),
        maximum_px4_tilt_deg=float(np.degrees(np.arccos(np.clip(rzz, -1, 1))).max()),
        max_vertical_lever_change_from_level_m=float((abs(lever[2])*(1-rzz)).max()),
        ekf_imu_position_parameters={k: ulog.initial_parameters.get(k)
                                     for k in ('EKF2_IMU_POS_X','EKF2_IMU_POS_Y','EKF2_IMU_POS_Z')},
        scope='Offline model and log audit. No flight gates, estimator parameters or '
              'model positions changed; airborne offset was frozen before route execution, '
              'not refitted to flight data. Mounting discrepancies are not proof of '
              'the source of the observed height residual.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = audit(args.run_dir)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
