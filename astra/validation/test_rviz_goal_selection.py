"""Reject goals that bypass frame, freshness, live-map or recovery limits."""
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from rviz_goal_selection import validate_goal, rectangle_covered
from planning_envelope import PlanningEnvelope


def guard(boxes=None):
    boxes=np.array(boxes if boxes is not None else [[[0,-2],[3,1]]],dtype=float)
    return SimpleNamespace(recovery_boxes=boxes,
        inside_recovery_region=lambda xy:bool(np.any(np.all((xy>=boxes[:,0])&(xy<=boxes[:,1]),axis=1))))


def select(**overrides):
    args=dict(frame='camera_init',stamp_ns=9_900_000_000,now_ns=10_000_000_000,
        xyz=[1.4,-.6,0.],origin=[.2,-.1,-.35],height_m=.9,guard=guard(),
        obstacles=[[2.5,-1,.7]],obstacle_age_s=.1)
    args.update(overrides);return validate_goal(**args)


def test_click_is_relative_to_frozen_ground_estimator_and_not_rebased_during_flight():
    assert select()==pytest.approx([1.2,-.5])


def test_click_rejected_before_takeoff_if_only_old_descent_reserve_passes():
    g=guard()
    envelope=PlanningEnvelope(g.recovery_boxes,[[0,-2],[3,1]],[.2,-.1])
    # This point has the old 0.10 m reserve but not the shared 0.40 m reserve.
    assert select(xyz=[.5,-.6,0.],guard=g)==pytest.approx([.3,-.5])
    with pytest.raises(ValueError,match='shared planning/execution'):
        select(xyz=[.5,-.6,0.],guard=g,planning_envelope=envelope)
    assert select(guard=g,planning_envelope=envelope)==pytest.approx([1.2,-.5])


def test_click_rejected_before_takeoff_when_planner_endpoint_padding_blocks_it():
    g=guard()
    envelope=PlanningEnvelope(g.recovery_boxes,[[0,-2],[3,1]],[.2,-.1])
    # A point can pass the published envelope while its planner search square
    # overlaps a blocked cell. Reject at click time, before arming or queuing.
    candidates=np.argwhere(envelope.free & ~envelope.guide_free)
    assert len(candidates)
    y,x=candidates[len(candidates)//2]
    xy=envelope.origin+(np.array([x,y])+.5)*envelope.resolution
    assert envelope.contains(xy) and not envelope.guide_contains(xy)
    with pytest.raises(ValueError,match='planner detour reserve'):
        select(xyz=[*xy,0.],guard=g,planning_envelope=envelope)
    assert select(guard=g,planning_envelope=envelope)==pytest.approx([1.2,-.5])


@pytest.mark.parametrize('overrides',[
    {'frame':'map'}, {'stamp_ns':0}, {'stamp_ns':7_000_000_000},
    {'stamp_ns':10_010_000_000},{'xyz':[float('nan'),0,0]},
    {'xyz':[1.4,-.6,.9]}, {'xyz':[.2,-.1,0]},
    {'xyz':[5,-.6,0]},{'xyz':[1.4,2,0]},
    {'xyz':[.25,-.6,0]},{'obstacle_age_s':1.001},{'obstacle_age_s':-.01},
    {'obstacles':[]},{'obstacles':[[float('nan'),0,0]]},
    {'obstacles':[[1.4,-.6,.55]]},
])
def test_unsafe_goals_rejected(overrides):
    with pytest.raises(ValueError):select(**overrides)


def test_rectangle_union_cannot_hide_interior_holes():
    # The perimeter and both diagonals miss this interior hole.
    boxes=np.array([[[0,0],[1,.3]],[[0,.5],[1,1]],
                    [[0,.3],[.6,.5]],[[.8,.3],[1,.5]]])
    assert not rectangle_covered(boxes,[0,0],[1,1])
    assert rectangle_covered(np.array([[[0,0],[.5,1]],[[.5,0],[1,1]]]),[0,0],[1,1])


def test_two_hz_map_uses_its_one_second_validity_without_relaxing_odom():
    assert select(obstacle_age_s=.6)==pytest.approx([1.2,-.5])


def test_outdoor_goal_radius_requires_scene_metadata_and_preserves_recovery_check():
    g=guard([[[-11,-11],[11,11]]]);g.metadata={'interactive_goal_radius_m':10.}
    assert select(xyz=[8.2,-.1,0.],guard=g)==pytest.approx([8.,0.])
    with pytest.raises(ValueError,match='between'):select(xyz=[10.3,-.1,0.],guard=g)
    g=guard();g.metadata={'interactive_goal_radius_m':10.}
    with pytest.raises(ValueError,match='descent region'):select(xyz=[8.2,-.1,0.],guard=g)


def test_invalid_scene_radius_cannot_expand_goals():
    for radius in [float('nan'),100.,2.]:
        g=guard();g.metadata={'interactive_goal_radius_m':radius}
        with pytest.raises(ValueError,match='Invalid inspected'):select(guard=g)


def test_whole_map_corner_accepted_but_radius_does_not_override_geometry():
    g=guard([[[-30,-30],[30,30]]]);g.metadata={'interactive_goal_radius_m':43.}
    envelope=PlanningEnvelope(g.recovery_boxes,[[-30,-30],[30,30]],[.2,-.1],
                              connected_to_origin=True)
    assert select(xyz=[29.2,28.9,0.],guard=g,planning_envelope=envelope)==pytest.approx([29.,29.])
    # Still within 43 m of home, but beyond the inspected square.
    with pytest.raises(ValueError,match='descent region'):
        select(xyz=[31.2,28.9,0.],guard=g,planning_envelope=envelope)


def test_goal_tools_do_not_publish_directly_to_planner_or_fmu():
    import yaml
    config=Path(__file__).resolve().parents[1]/'config/rviz_goal_flight.rviz'
    tools=yaml.safe_load(config.read_text())['Visualization Manager']['Tools']
    assert [t['Topic']['Value'] for t in tools if 'Topic' in t]==['/astra/ui/goal','/astra/ui/land']
