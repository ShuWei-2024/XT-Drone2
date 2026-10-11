"""A latest preview ID must not falsely identify an older transmitted pose."""
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from audit_spline_selection_117 import prepare,match_sample


def test_stale_id_is_checked_against_actual_position_and_pose_time():
    messages=[dict(traj_id=i,start_ns=100_000_000_000,order=3,knots=list(range(12)),
                   controls=[[j,y,1.7] for j in range(8)]) for i,y in [(1,0),(2,1)]]
    sample=dict(target=[.1,.9,-2.],preview_pose_stamp_ns=100_100_000_000,
                preview_trajectory_id=2,preview_trajectory_start_ns=100_000_000_000)
    alignment=dict(xy_matrix=[[0,1],[1,0]],xy_translation=[.1,-.2],z_translation=-.3)
    match=match_sample(sample,prepare(messages),alignment)
    assert match is not None and match[0]==0
    assert np.allclose(match[2],[1.1,0,1.7])


def test_missing_exact_time_cannot_certify_a_selected_spline():
    assert match_sample(dict(target=[0,0,0]),[],{}) is None
