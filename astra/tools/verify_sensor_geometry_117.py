"""Fail before flight if expanded SDF, extrinsics, output pose or TF disagree."""
import argparse
import ast
import json
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from scipy.spatial.transform import Rotation


def matrix(element):
    v = [float(x) for x in element.findtext('pose', '0 0 0 0 0 0').split()]
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler('xyz', v[3:]).as_matrix()
    result[:3, 3] = v[:3]
    return result


def verify(models, px4, config):
    repo = Path(__file__).resolve().parents[2]
    env = os.environ.copy()
    env['SDF_PATH'] = str(models.resolve())+':'+str(px4/'Tools/simulation/gz/models')
    expanded = subprocess.run(['gz','sdf','-p',str(models/'astra_x500_px4/model.sdf')],
                              env=env, text=True, capture_output=True, check=True)
    model = ET.fromstring(expanded.stdout).find('model')
    elements = {e.get('name'): e for e in model if e.tag in ('link', 'frame')}
    poses = {'__model__': np.eye(4)}
    def resolve(name):
        if name not in poses:
            e = elements[name];p = e.find('pose')
            poses[name] = resolve(p.get('relative_to','__model__') if p is not None else '__model__') @ matrix(e)
        return poses[name]
    params = yaml.safe_load(config.read_text())['/**']['ros__parameters']
    checks = {}
    def check(name, actual, expected):
        checks[name] = bool(np.allclose(actual, expected, atol=1e-8, rtol=0))
    body, imu, scan, camera = [resolve(k) for k in
                              ('base_link','imu_link','mid360_scan_frame','camera_color_optical_frame')]
    sensor = resolve('mid360_link') @ matrix(elements['mid360_link'].find("sensor[@name='mid360_equivalent']"))
    check('scan_tf_origin_matches_sensor', scan, sensor)
    calibration = params['extrin_calib']
    for label, actual, r_key, t_key in (
            ('lidar_to_imu',np.linalg.inv(imu)@scan,'extrinsic_R','extrinsic_T'),
            ('lidar_to_camera',np.linalg.inv(camera)@scan,'Rcl','Pcl')):
        check(label+'_rotation', np.array(calibration[r_key]).reshape(3,3), actual[:3,:3])
        check(label+'_translation', calibration[t_key], actual[:3,3])
    check('published_position_is_body_centre',params['publish']['reference_translation_imu'],
          (np.linalg.inv(imu)@body)[:3,3])
    check('imu_and_body_axes_match', (np.linalg.inv(imu)@body)[:3,:3],np.eye(3))
    checks['odom_child_frame_is_body'] = params['publish']['odom_child_frame']=='base_link'
    tree = ast.parse((repo/'xtd2_launch/launch/astra_sitl.launch.py').read_text())
    specs = next(ast.literal_eval(n.value) for n in ast.walk(tree) if isinstance(n,ast.Assign)
                 and any(isinstance(t,ast.Name) and t.id=='tf_specs' for t in n.targets))
    for spec in specs:
        x,y,z,roll,pitch,yaw = map(float,spec[:6]);expected=np.eye(4)
        expected[:3,3]=[x,y,z]
        expected[:3,:3]=Rotation.from_euler('xyz',[roll,pitch,yaw]).as_matrix()
        check('static_tf_'+spec[-1],np.linalg.inv(resolve(spec[-2]))@resolve(spec[-1]),expected)
    bridge=yaml.safe_load((repo/'astra/packages/astra_gz_sensors/config/bridge.yaml').read_text())
    adapter=yaml.safe_load((repo/'astra/packages/astra_fast_livo2/config/fast_livo2_gazebo.yaml').read_text())['/**']['ros__parameters']
    checks['cloud_frame_is_scan_origin']=next(r['frame_id'] for r in bridge if r['ros_type_name']=='sensor_msgs/msg/PointCloud2')=='mid360_scan_frame'
    checks['adapter_frame_is_scan_origin']=adapter['lidar_frame_id']=='mid360_scan_frame'
    check('adapter_imu_translation', adapter['extrinsic_T_lidar_to_imu'],calibration['extrinsic_T'])
    check('adapter_camera_translation', adapter['extrinsic_T_lidar_to_camera_optical'],calibration['Pcl'])
    return dict(passed=all(checks.values()),checks=checks,
                model_to_body_translation_m=body[:3,3].tolist(),
                body_to_imu_translation_m=(np.linalg.inv(body)@imu)[:3,3].tolist(),
                body_to_scan_translation_m=(np.linalg.inv(body)@scan)[:3,3].tolist(),
                scope='Expanded SDF including inherited model offsets; checked against TF and active estimator config')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--models',type=Path,required=True)
    p.add_argument('--px4',type=Path,required=True)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();report=verify(a.models,a.px4,a.config)
    a.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    raise SystemExit(0 if report['passed'] else 1)
