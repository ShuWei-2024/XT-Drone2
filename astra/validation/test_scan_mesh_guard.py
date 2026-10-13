"""Distance correctness and bounded scan guard regressions; no ROS or simulator."""
import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from scan_mesh_guard import MeshGuard, triangle_distances, box_clearance_lower_bound


@pytest.mark.parametrize('point,expected',[
    ([.2,.2,1],1.), ([2,0,0],1.), ([-1,-1,0],np.sqrt(2)),
    ([.5,.5,0],0.), ([.5,.5,-.3],.3),
])
def test_triangle_interior_edges_and_both_sides(point,expected):
    tri=np.array([[[0,0,0],[1,0,0],[0,1,0]]])
    assert triangle_distances(point,tri)[0]==pytest.approx(expected)


def test_degenerate_triangle_is_a_segment():
    assert triangle_distances([.5,1,0],np.array([[[0,0,0],[1,0,0],[1,0,0]]]))[0]==pytest.approx(1.)


def make_guard(tmp_path,triangles):
    path=tmp_path/'guard.npz'
    np.savez(path,triangles=triangles,metadata=json.dumps(dict(
        minimum_clearance_m=.55,allowed_xy_bounds_m=[[-2,-2],[2,2]])))
    return MeshGuard(path)


def test_far_centroid_large_triangle_cannot_be_missed(tmp_path):
    guard=make_guard(tmp_path,np.array([
        [[0,0,0],[100,0,0],[0,100,0]],[[1,1,5],[2,1,5],[1,2,5]]]))
    assert guard.clearance([1,1,.1])==pytest.approx(.1)
    assert 'below' in guard.fault([1,1,.1])
    assert guard.fault([3,0,1])=='outside inspected scan flight envelope'


def test_accelerated_distance_matches_all_triangles(tmp_path):
    rng=np.random.default_rng(4)
    triangles=rng.uniform(-2,2,(200,3,3))
    guard=make_guard(tmp_path,triangles)
    for p in rng.uniform(-2,2,(20,3)):
        assert guard.clearance(p)==pytest.approx(triangle_distances(p,triangles).min())


def test_nonfinite_measurement_rejected(tmp_path):
    guard=make_guard(tmp_path,np.array([[[0,0,0],[1,0,0],[0,1,0]]]))
    with pytest.raises(ValueError):guard.clearance([0,np.nan,1])


def test_boundary_reserve_checks_direction_and_stationary_hold(tmp_path):
    guard=make_guard(tmp_path,np.array([[[0,0,0],[1,0,0],[0,1,0]]]))
    assert guard.approaching_boundary([1.9,0,1],[0,0,0])==''
    assert guard.approaching_boundary([1.9,0,1],[-.3,0,0])==''
    assert guard.approaching_boundary([1.9,0,1],[.3,0,0])=='approaching inspected scan flight boundary'
    assert guard.approaching_boundary([0,-1.9,1],[0,-.3,0])=='approaching inspected scan flight boundary'
    assert guard.approaching_boundary([0,0,1],[float('nan'),0,0])=='invalid scan guard velocity'


def test_continuous_recovery_box_bound_is_conservative(tmp_path):
    guard=make_guard(tmp_path,np.array([[[-10,-10,0],[20,-10,0],[-10,20,0]]]))
    result=box_clearance_lower_bound(guard,[[0,0,.6],[.1,.1,.7]])
    assert result['sampled_minimum_m']==pytest.approx(.6)
    assert .55 < result['continuous_clearance_lower_bound_m'] <= .6
    with pytest.raises(ValueError):box_clearance_lower_bound(guard,[[0,0,.6],[.1,.1,.7]],.5)
