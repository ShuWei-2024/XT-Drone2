"""Outdoor speed validation must not silently change stricter indoor audits."""
import json
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from audit_scan_flight import recorded_motion_limits


def test_old_indoor_audit_retains_its_limits(tmp_path):
    (tmp_path/'runtime.json').write_text(json.dumps(dict(world_preset='replica-scan')))
    assert recorded_motion_limits(tmp_path)['maximum_speed_mps']==.10


def test_old_outdoor_audit_requires_matching_saved_launch_configuration(tmp_path):
    (tmp_path/'runtime.json').write_text(json.dumps(dict(world_preset='baylands')))
    p=tmp_path/'execution_sources/astra/tools/run_sitl_117.py';p.parent.mkdir(parents=True)
    p.write_text('unknown launch configuration')
    with pytest.raises(ValueError):recorded_motion_limits(tmp_path)
    p.write_text("'max_vel:=0.10' if is_scan and not is_outdoor else 'max_vel:=0.20'\n"
                 "'max_acc:=0.10' if is_scan and not is_outdoor else 'max_acc:=0.25'")
    limits=recorded_motion_limits(tmp_path)
    assert limits['maximum_speed_mps']==.20 and limits['maximum_acceleration_mps2']==.25


def test_explicit_recorded_limits_are_validated(tmp_path):
    limits=dict(maximum_speed_mps=.20,maximum_acceleration_mps2=.25)
    p=tmp_path/'runtime.json';p.write_text(json.dumps(dict(planner_motion_limits=limits)))
    assert recorded_motion_limits(tmp_path)['maximum_speed_mps']==.20
    limits['maximum_speed_mps']=float('nan')
    p.write_text(json.dumps(dict(planner_motion_limits=limits)))
    with pytest.raises(ValueError):recorded_motion_limits(tmp_path)
