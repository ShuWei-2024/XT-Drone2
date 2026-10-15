"""Safety-region holes and continuous cubic coverage, independent of ROS."""
import sys
from pathlib import Path
import numpy as np
import pytest
from scipy.interpolate import BSpline
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from planning_envelope import PlanningEnvelope


def curve(controls):
    return BSpline(np.arange(8, dtype=float), np.asarray(controls, float), 3, extrapolate=False)


def test_union_seams_and_reserve():
    boxes = [[[0, 0], [2, 4]], [[2, 0], [4, 4]]]
    e = PlanningEnvelope(boxes, [[0, 0], [4, 4]], [10, -10])
    assert e.contains([12, -8])  # adjoining rectangles aren't a forbidden seam
    assert not e.contains([10.2, -8])
    assert not e.contains([14.1, -8])
    e.validate_spline(curve([[11, -8, 1], [11.5, -8, 1], [12, -8, 1], [12.5, -8, 1]]))


def test_safe_endpoints_do_not_certify_curve_through_hole():
    boxes = [[[-4, -4], [-1, 4]], [[1, -4], [4, 4]], [[-1, 2], [1, 4]]]
    e = PlanningEnvelope(boxes, [[-4, -4], [4, 4]], [0, 0])
    spline = curve([[-6, 0, 1], [-2, 0, 1], [2, 0, 1], [6, 0, 1]])
    assert e.contains(spline(3)[:2]) and e.contains(spline(4)[:2])
    with pytest.raises(ValueError, match='envelope'): e.validate_spline(spline)
    e.validate_spline(curve([[-3, 3, 1], [-1, 3, 1], [1, 3, 1], [3, 3, 1]]))


def test_curve_bulge_is_checked_between_endpoints():
    e = PlanningEnvelope([[[-4, -4], [4, 4]]], [[-4, -4], [4, 4]], [0, 0])
    spline = curve([[-2, -40, 1], [-1, 10, 1], [1, 10, 1], [2, -40, 1]])
    assert e.contains(spline(3)[:2]) and e.contains(spline(4)[:2])
    with pytest.raises(ValueError): e.validate_spline(spline)


def test_guide_endpoint_mask_matches_planner_square_neighbourhood():
    e=PlanningEnvelope([[[0,0],[5,5]]],[[0,0],[5,5]],[10,-10])
    padding=2  # guarded planner 0.20 m grid / 0.10 m envelope cells
    original=e.free.copy()
    for y in range(e.free.shape[0]):
        for x in range(e.free.shape[1]):
            expected=(x>=padding and y>=padding and
                      x+padding<e.free.shape[1] and y+padding<e.free.shape[0] and
                      e.free[y-padding:y+padding+1,x-padding:x+padding+1].all())
            xy=e.origin+(np.array([x,y])+.5)*e.resolution
            assert e.guide_contains(xy)==bool(expected)
    assert np.array_equal(e.free,original)  # published/curve-check mask unchanged
    for xy in [[float('nan'),0],[10],[-100,100]]:
        assert not e.guide_contains(xy)


@pytest.mark.parametrize('origin', [[np.nan, 0], [0]])
def test_invalid_reference(origin):
    with pytest.raises(ValueError): PlanningEnvelope([[[0, 0], [4, 4]]], [[0, 0], [4, 4]], origin)
