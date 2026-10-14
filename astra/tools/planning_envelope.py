"""A conservative planning mask derived from the supervisor's descent certificate.

This is a prior safety constraint, not sensor-observed obstacle geometry. The
reference is frozen from ground odometry at the known zero-yaw SITL spawn.
"""
import math
import numpy as np
from scipy.ndimage import distance_transform_edt


class PlanningEnvelope:
    def __init__(self, boxes, bounds, origin_xy, resolution=.1, reserve=.4):
        boxes, bounds, origin_xy = map(lambda v: np.asarray(v, float), (boxes, bounds, origin_xy))
        if (boxes.ndim != 3 or boxes.shape[1:] != (2, 2) or not len(boxes)
                or bounds.shape != (2, 2) or origin_xy.shape != (2,)
                or not np.isfinite(np.r_[boxes.ravel(), bounds.ravel(), origin_xy]).all()
                or np.any(boxes[:, 1] <= boxes[:, 0]) or np.any(bounds[1] <= bounds[0])
                or not .05 <= resolution <= .2 or not .3 <= reserve <= .8):
            raise ValueError('Invalid planning envelope')
        self.resolution, self.reserve = resolution, reserve
        size = np.ceil((bounds[1]-bounds[0])/resolution).astype(int)
        self.origin = bounds[0]+origin_xy
        free = np.zeros(tuple(size[::-1]), bool)
        # Only complete cells inside a certified rectangle are accepted. Cells
        # crossing rectangle seams may be excluded conservatively.
        for lo, hi in boxes:
            begin = np.ceil((lo-bounds[0])/resolution-1e-9).astype(int)
            end = np.floor((hi-bounds[0])/resolution+1e-9).astype(int)
            begin = np.maximum(begin, 0); end = np.minimum(end, size)
            if np.all(end > begin):
                free[begin[1]:end[1], begin[0]:end[0]] = True
        # A square cell's points may be sqrt(2)*resolution closer to a blocked
        # square than their centres. Pad outside as blocked and subtract that
        # bound before reserving tracking/forecast space.
        distance = distance_transform_edt(np.pad(free, 1))[1:-1, 1:-1]*resolution
        self.free = free & (distance >= reserve+math.sqrt(2)*resolution)

    def contains(self, xy):
        xy = np.asarray(xy, float)
        if xy.shape != (2,) or not np.isfinite(xy).all(): return False
        x, y = np.floor((xy-self.origin)/self.resolution).astype(int)
        return bool(0 <= y < self.free.shape[0] and 0 <= x < self.free.shape[1] and self.free[y, x])

    def rectangle_free(self, lo, hi):
        lo, hi = np.asarray(lo), np.asarray(hi)
        a = np.floor((lo-self.origin)/self.resolution).astype(int)
        b = np.floor((hi-self.origin)/self.resolution).astype(int)
        if np.any(a < 0) or b[0] >= self.free.shape[1] or b[1] >= self.free.shape[0]: return False
        return bool(self.free[a[1]:b[1]+1, a[0]:b[0]+1].all())

    def validate_spline(self, spline):
        """Certify the entire cubic curve by subdivided Bezier convex hulls."""
        derivative = spline.derivative()
        def covered(c, depth=0):
            if self.rectangle_free(c[:, :2].min(0), c[:, :2].max(0)): return True
            if depth >= 10: return False
            a = (c[:-1]+c[1:])/2; b = (a[:-1]+a[1:])/2; mid = (b[0]+b[1])/2
            return (covered(np.array([c[0], a[0], b[0], mid]), depth+1)
                    and covered(np.array([mid, b[1], a[2], c[3]]), depth+1))
        for a, b in zip(spline.t[spline.k:-spline.k-1], spline.t[spline.k+1:-spline.k]):
            if b <= a: continue
            c = np.array([spline(a), spline(a)+(b-a)/3*derivative(a),
                          spline(b)-(b-a)/3*derivative(b), spline(b)])
            if not covered(c): raise ValueError('trajectory leaves shared planning/execution envelope')

    def message(self, stamp):
        from nav_msgs.msg import OccupancyGrid
        msg = OccupancyGrid()
        msg.header.frame_id = 'camera_init'; msg.header.stamp = stamp
        msg.info.resolution = self.resolution
        msg.info.width = self.free.shape[1]; msg.info.height = self.free.shape[0]
        msg.info.origin.position.x, msg.info.origin.position.y = map(float, self.origin)
        msg.info.origin.orientation.w = 1.
        msg.data = np.where(self.free, 0, 100).astype(np.int8).ravel().tolist()
        return msg
