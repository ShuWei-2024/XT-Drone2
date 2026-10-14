"""Sparse/empty measurement phases must report zero samples, never crash."""
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from audit_obstacle_reliability_20261010 import positions,describe


def test_empty_phase_has_zero_samples_in_three_dimensions():
    delta=positions([])@np.eye(3)+np.array([1.,2.,3.])-positions([])
    assert delta.shape==(0,3)
    assert describe(np.linalg.norm(delta[:,:2],axis=1))=={'count':0}
