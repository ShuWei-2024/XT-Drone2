import sys
from pathlib import Path

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from motion_duration_audit import MotionDurationCounter, expand_bounded_route


def test_stationary_hover_does_not_count_as_motion():
    clock=MotionDurationCounter()
    clock.update(1_000_000_000,(0.,0.),enabled=True)
    row=clock.update(1_050_000_000,(0.,0.),enabled=True)
    assert row['moving_seconds']==0.
    assert row['moving_distance_m']==0.


def test_truth_motion_counts_only_fresh_intervals_owned_by_ego():
    clock=MotionDurationCounter()
    clock.update(1_000_000_000,(0.,0.),enabled=True)
    held=clock.update(1_050_000_000,(.02,0.),enabled=False)
    assert held['moving_seconds']==0.
    moving=clock.update(1_100_000_000,(.03,0.),enabled=True)
    assert moving['moving_seconds']==pytest.approx(.05)
    assert moving['moving_distance_m']==pytest.approx(.01)


def test_stale_interval_and_truth_jump_do_not_inflate_duration():
    clock=MotionDurationCounter()
    clock.update(1_000_000_000,(0.,0.))
    gap=clock.update(1_500_000_000,(.5,0.))
    assert gap['moving_seconds']==0.
    jump=clock.update(1_550_000_000,(2.,0.))
    assert jump['moving_seconds']==0.


def test_repeated_route_is_finite_and_preserves_the_checked_waypoints():
    route=[[6.44,.817],[-9.091,19.639]]
    expanded=expand_bounded_route(route,4,maximum_goals=8)
    assert expanded==route*4
    assert expanded[0] is not route[0]


@pytest.mark.parametrize('route,cycles', [
    ([[0.,0.],[0.,0.]],2),
    ([[0.,0.],[1.,0.]],33),
    ([[0.,0.],[1.,0.]],16),
    ([[0.,0.]],2),
    ([[0.,0.],[float('nan'),1.]],2),
])
def test_repeated_route_rejects_degenerate_or_unbounded_cases(route,cycles):
    with pytest.raises(ValueError):
        expand_bounded_route(route,cycles,maximum_goals=16)
