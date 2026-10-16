"""Prevent sampled clearance and ordinary straight flight becoming avoidance claims."""
import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from scan_mesh_guard import MeshGuard
from scan_trajectory_audit import line_audit, spline_audit, lateral_deviation


def guard(tmp_path, triangles):
    path=tmp_path/'mesh.npz'
    np.savez(path, triangles=triangles, metadata=json.dumps(dict(
        minimum_clearance_m=.55, allowed_xy_bounds_m=[[-10,-10],[10,10]])))
    return MeshGuard(path)


def test_straight_path_crosses_known_wall(tmp_path):
    g=guard(tmp_path, [[[0,-2,0],[0,2,0],[0,0,4]]])
    blocked=line_audit(g, [-1,0,1], [1,0,1])
    clear=line_audit(g, [-1,-1,1], [-1,1,1])
    assert blocked['violates_clearance_margin']
    assert blocked['sampled_minimum_clearance_m']==pytest.approx(0.)
    assert not clear['violates_clearance_margin']
    assert clear['continuous_clearance_lower_bound_m']>=.55


def test_between_sample_reserve_can_fail_while_samples_pass(tmp_path):
    g=guard(tmp_path, [[[-10,-10,0],[20,-10,0],[-10,20,0]]])
    msg=dict(start_ns=0,order=3,knots=[0,0,0,0,1,1,1,1],
             controls=[[0,0,.551],[1/3,0,.551],[2/3,0,.551],[1,0,.551]])
    r=spline_audit(g,msg,[0,0,0])
    assert r['sampled_minimum_clearance_m']>.55
    assert r['continuous_clearance_lower_bound_m']<.55
    assert r['speed_bound_mps']==pytest.approx(1.)
    assert r['acceleration_bound_mps2']==pytest.approx(0.,abs=1e-12)


def test_detour_is_measured_perpendicular_to_requested_direction():
    assert lateral_deviation([[0,0,1],[1,0,1],[2,0,1]],[0,0,1],[2,0,1])==0.
    assert lateral_deviation([[0,0,1],[1,.3,1],[2,0,1]],[0,0,1],[2,0,1])==pytest.approx(.3)
    assert lateral_deviation([[0,0,1],[.3,1,1],[0,2,1]],[0,0,1],[0,2,1])==pytest.approx(.3)
