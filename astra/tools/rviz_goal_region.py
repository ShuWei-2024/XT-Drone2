"""Display-only footprint of the actual static waypoint admission mask.

Keep holes and planner endpoint padding. A coloured footprint never bypasses
live-map, telemetry, session, landing or timestamp checks when a user clicks.
"""
import numpy as np
from flight_recovery import compact_mask


def goal_region_mask(envelope, origin_xy, radius):
    origin_xy = np.asarray(origin_xy, float)
    if (origin_xy.shape != (2,) or not np.isfinite(origin_xy).all() or
            not np.isfinite(radius) or not 3. <= radius <= 50.):
        raise ValueError('Invalid waypoint display reference/radius')
    yy, xx = np.indices(envelope.guide_free.shape)
    lo = np.stack((xx, yy), axis=-1)*envelope.resolution+envelope.origin-origin_xy
    hi = lo+envelope.resolution
    # Show only complete cells within the radial limits. This conservatively
    # omits the handful of cells crossing the 0.25 m inner / outer circles.
    nearest = np.maximum(np.maximum(lo, -hi), 0.)
    farthest = np.maximum(np.abs(lo), np.abs(hi))
    return (envelope.guide_free & (np.linalg.norm(nearest, axis=-1) >= .25) &
            (np.linalg.norm(farthest, axis=-1) <= radius))


def mask_boundary_segments(mask, origin, step):
    """Merged edges of a cell union, including every interior hole."""
    mask = np.asarray(mask, bool)
    horizontal = np.diff(np.pad(mask.astype(np.int8), ((1, 1), (0, 0))), axis=0) != 0
    vertical = np.diff(np.pad(mask.astype(np.int8), ((0, 0), (1, 1))), axis=1) != 0
    segments = []
    for swap, rows in ((False, horizontal), (True, vertical.T)):
        for index, row in enumerate(rows):
            edges = np.diff(np.r_[False, row, False].astype(np.int8))
            for start, end in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)):
                a, b = ([index, start], [index, end]) if swap else ([start, index], [end, index])
                segments.append((np.asarray(origin)+np.asarray(a)*step,
                                 np.asarray(origin)+np.asarray(b)*step))
    return segments


def goal_region_geometry(envelope, origin_xy, radius):
    mask = goal_region_mask(envelope, origin_xy, radius)
    return (compact_mask(mask, envelope.origin, envelope.resolution),
            mask_boundary_segments(mask, envelope.origin, envelope.resolution),
            float(mask.sum()*envelope.resolution**2))
