"""Reject real SDF/TF/calibration disagreement before the SITL arm sequence."""
import importlib.util
import os
from pathlib import Path
import shutil

import pytest
import yaml

repo = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('geometry',repo/'astra/tools/verify_sensor_geometry_117.py')
geometry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(geometry)
px4 = Path(os.environ.get('ASTRA_PX4_DIR',str(Path.home()/'dev/px4/PX4-Autopilot-1.17')))
pytestmark = pytest.mark.skipif(not (px4/'Tools/simulation/gz/models/x500').exists(),
                                reason='Requires the pinned PX4 model checkout')
models = repo/'astra/packages/astra_gz_sensors/models'
config = repo/'astra/config/estimator_self_filtered.yaml'


def test_consistent_geometry_accepts():
    assert geometry.verify(models,px4,config)['passed']


@pytest.mark.parametrize('section,key,value,failed_check',[
    ('extrin_calib','extrinsic_T',[0.,0.,-.10],'lidar_to_imu_translation'),
    ('extrin_calib','Pcl',[0.,-.12,-.16],'lidar_to_camera_translation'),
    ('publish','reference_translation_imu',[0.,0.,0.],'published_position_is_body_centre'),
])
def test_bad_reference_or_calibration_rejects(tmp_path,section,key,value,failed_check):
    params=yaml.safe_load(config.read_text())
    params['/**']['ros__parameters'][section][key]=value
    changed=tmp_path/'config.yaml';changed.write_text(yaml.safe_dump(params))
    report=geometry.verify(models,px4,changed)
    assert not report['passed']
    assert not report['checks'][failed_check]


def test_inherited_model_origin_cannot_replace_body_origin(tmp_path):
    changed=tmp_path/'models';shutil.copytree(models,changed)
    path=changed/'astra_x500_sensors/model.sdf'
    path.write_text(path.read_text().replace('relative_to="base_link"','relative_to="__model__"'))
    report=geometry.verify(changed,px4,config)
    assert not report['passed']
    assert not report['checks']['static_tf_imu_link']
