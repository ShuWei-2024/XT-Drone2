"""Expanded goals cannot outgrow their inspected geometry or planner grid."""
import copy
import sys
from pathlib import Path
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from outdoor_envelope import outdoor_envelope


def certificate():
    meta=dict(interactive_goal_radius_m=25.,allowed_xy_bounds_m=[[-26.,-26.],[26.,26.]],
              allowed_body_z_bounds_m=[.2,2.],recovery_continuous_clearance_lower_bound_m=.65,
              recovery_certificate=dict(sampled_bounds_m=[[-26.,-26.],[26.,26.]],certified_z_bounds_m=[.2,2.]))
    pre=copy.deepcopy(meta);pre['planner_map_size_xy_m']=64.
    return pre,meta


def test_grid_covers_expanded_local_window():
    pre,meta=certificate();result=outdoor_envelope(pre,meta)
    assert result['goal_radius_m']==25.
    assert result['planner_map_size_xy_m']/2>=26.+5.


@pytest.mark.parametrize('key,value',[
    ('planner_map_size_xy_m',32.),('planner_map_size_xy_m',float('nan')),
    ('interactive_goal_radius_m',10.),('allowed_xy_bounds_m',[[-11,-11],[11,11]]),
])
def test_mismatched_or_insufficient_preparation_rejected(key,value):
    pre,meta=certificate();pre[key]=value
    with pytest.raises(ValueError):outdoor_envelope(pre,meta)


def test_old_bounds_cannot_certify_new_region():
    pre,meta=certificate();meta['recovery_certificate']['sampled_bounds_m']=[[-11,-11],[11,11]]
    with pytest.raises(ValueError,match='certificate'):outdoor_envelope(pre,meta)


def test_branch_projection_is_an_isolated_expanded_scene_setting():
    pre,meta=certificate();assert outdoor_envelope(pre,meta)['column_obstacle_max_z_m']==3.
    for d in [pre,meta]:
        d.update(interactive_goal_radius_m=10.,allowed_xy_bounds_m=[[-11.,-11.],[11.,11.]])
        d['recovery_certificate']['sampled_bounds_m']=[[-11.,-11.],[11.,11.]]
    pre['planner_map_size_xy_m']=32.
    assert outdoor_envelope(pre,meta)['column_obstacle_max_z_m']==1.5


@pytest.mark.parametrize('value',[float('nan'),True,1.])
def test_invalid_branch_height_range_rejected(value):
    pre,meta=certificate();pre['column_obstacle_max_z_m']=value
    with pytest.raises(ValueError,match='branch height'):outdoor_envelope(pre,meta)
