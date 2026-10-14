"""Measurement-preserving base_link FLU/ENU to PX4 body FRD/NED conversion."""
import numpy as np
from scipy.spatial.transform import Rotation

ENU_TO_NED = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
FRD_TO_FLU = np.diag([1., -1., -1.])


def convert_pose(position, quaternion_xyzw, translation):
    p, q, t = map(lambda v: np.asarray(v, float), (position, quaternion_xyzw, translation))
    if (p.shape != (3,) or q.shape != (4,) or t.shape != (3,)
            or not np.isfinite(np.r_[p, q, t]).all() or abs(np.linalg.norm(q)-1.) > .01):
        raise ValueError('Invalid external pose')
    r = ENU_TO_NED @ Rotation.from_quat(q).as_matrix() @ FRD_TO_FLU
    out = Rotation.from_matrix(r).as_quat()
    return ENU_TO_NED @ p+t, np.r_[out[3], out[:3]]


def valid_measurement(stamp_ns, now_ns, last_stamp_ns):
    return stamp_ns > last_stamp_ns and 0 <= now_ns-stamp_ns <= 250_000_000
