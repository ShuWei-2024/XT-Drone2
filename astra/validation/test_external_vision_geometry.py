import sys
from pathlib import Path
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from external_vision_geometry import convert_pose, valid_measurement


def test_axes_and_body_attitude():
    p, q = convert_pose([1, 2, 3], [0, 0, 0, 1], [.1, .2, .3])
    np.testing.assert_allclose(p, [2.1, 1.2, -2.7])
    r = Rotation.from_quat(np.r_[q[1:], q[0]])
    np.testing.assert_allclose(r.apply([1, 0, 0]), [0, 1, 0], atol=1e-12)
    np.testing.assert_allclose(r.apply([0, 0, 1]), [0, 0, 1], atol=1e-12)
    _, q = convert_pose([0, 0, 0], Rotation.from_euler('z', 90, degrees=True).as_quat(), [0, 0, 0])
    r = Rotation.from_quat(np.r_[q[1:], q[0]])
    np.testing.assert_allclose(r.apply([1, 0, 0]), [1, 0, 0], atol=1e-12)


def test_measurement_time_rejects_future_stale_and_duplicates():
    assert valid_measurement(1_000_000_000, 1_100_000_000, 900_000_000)
    assert not valid_measurement(1_000_000_000, 1_300_000_000, 900_000_000)
    assert not valid_measurement(1_000_000_000, 900_000_000, 800_000_000)
    assert not valid_measurement(1_000_000_000, 1_100_000_000, 1_000_000_000)


def test_invalid_quaternion_is_rejected():
    with pytest.raises(ValueError): convert_pose([0, 0, 0], [0, 0, 0, 0], [0, 0, 0])
