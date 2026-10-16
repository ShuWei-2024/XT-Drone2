"""A visible waypoint cell must never bypass static admission or hide holes."""
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from planning_envelope import PlanningEnvelope
from rviz_goal_region import goal_region_mask, goal_region_geometry, mask_boundary_segments
from rviz_goal_selection import validate_goal


def test_visible_cells_pass_real_static_admission_with_translated_ground_origin():
    boxes = np.array([[[-5,-5],[1,5]], [[2,-5],[5,5]], [[1,-5],[2,1]], [[1,2],[2,5]]])
    origin = np.array([10., -4., -.35])
    e = PlanningEnvelope(boxes, [[-5,-5],[5,5]], origin[:2])
    g = NS(recovery_boxes=boxes, metadata={'interactive_goal_radius_m': 3.},
           inside_recovery_region=lambda p: bool(np.any(np.all((p>=boxes[:,0]) & (p<=boxes[:,1]),axis=1))))
    mask = goal_region_mask(e, origin[:2], 3.)
    assert mask.any() and not np.any(mask & ~e.guide_free)
    assert not mask[65,65]  # interior hole near relative (1.5, 1.5)
    for y, x in np.argwhere(mask)[::13]:
        # Test corners as well as centres, avoiding floating-point cell seams.
        for fraction in (.00001, .5, .99999):
            xy = e.origin+(np.array([x,y])+fraction)*e.resolution
            result = validate_goal('camera_init', 9_900_000_000, 10_000_000_000,
                [*xy,0.], origin, 1.5, g, [[100.,100.,1.]], .1, planning_envelope=e)
            assert result == pytest.approx(xy-origin[:2])
    boxes_out, _, area = goal_region_geometry(e, origin[:2], 3.)
    assert sum(np.prod(np.asarray(hi)-lo) for lo,hi in boxes_out) == pytest.approx(area)


def test_boundary_preserves_interior_hole_and_merges_straight_edges():
    mask = np.ones((3,3), bool); mask[1,1] = False
    edges = mask_boundary_segments(mask, [10.,-4.], .1)
    assert len(edges) == 8  # four outside, four around the hole
    assert sum(np.linalg.norm(b-a) for a,b in edges) == pytest.approx(1.6)
    assert mask_boundary_segments(np.zeros((3,3),bool), [0,0], .1) == []


def test_display_does_not_claim_live_obstacle_clearance():
    e = PlanningEnvelope([[[-4,-4],[4,4]]], [[-4,-4],[4,4]], [0,0])
    assert goal_region_mask(e, [0,0], 3.)[40,50]
    g = NS(recovery_boxes=np.array([[[-4,-4],[4,4]]]),
           inside_recovery_region=lambda _:True, metadata={})
    with pytest.raises(ValueError, match='inflated live obstacle'):
        validate_goal('camera_init', 1, 1, [1.05,.05,0.], [0,0,0], 1.5,
            g, [[1.05,.05,1.5]], .1, planning_envelope=e)
