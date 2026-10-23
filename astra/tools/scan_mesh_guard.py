"""Exact distance to static scan triangles; truth is used only to stop SITL.

The centre KD tree accelerates queries without dropping potentially closer
triangles: each candidate centre lies within upper_bound + max_triangle_radius.
This measures known surfaces; it cannot certify missing scan geometry.
"""
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


def triangle_distances(point, triangles):
    p = np.asarray(point, dtype=float)
    a, b, c = np.asarray(triangles, dtype=float).transpose(1, 0, 2)
    ab, ac, ap = b-a, c-a, p-a
    aa = np.einsum('ij,ij->i', ab, ab)
    cc = np.einsum('ij,ij->i', ac, ac)
    cross = np.einsum('ij,ij->i', ab, ac)
    pa = np.einsum('ij,ij->i', ap, ab)
    pc = np.einsum('ij,ij->i', ap, ac)
    determinant = aa*cc-cross*cross
    denominator = np.where(determinant > 1e-20, determinant, 1.)
    u, v = (cc*pa-cross*pc)/denominator, (aa*pc-cross*pa)/denominator
    inside = (determinant > 1e-20) & (u >= 0) & (v >= 0) & (u+v <= 1)
    projection = a + u[:, None]*ab + v[:, None]*ac
    dist2 = np.where(inside, np.sum((p-projection)**2, axis=1), np.inf)
    for start, end in ((a,b), (b,c), (c,a)):
        edge = end-start
        length2 = np.sum(edge*edge, axis=1)
        t = np.clip(np.sum((p-start)*edge,axis=1)/np.maximum(length2,1e-30),0,1)
        dist2 = np.minimum(dist2,np.sum((p-start-t[:,None]*edge)**2,axis=1))
    return np.sqrt(dist2)


class MeshGuard:
    def __init__(self, path):
        with np.load(path, allow_pickle=False) as asset:
            self.triangles = asset['triangles'].astype(float)
            self.metadata = json.loads(str(asset['metadata']))
        if (self.triangles.ndim != 3 or self.triangles.shape[1:] != (3,3)
                or not len(self.triangles) or not np.isfinite(self.triangles).all()):
            raise ValueError('Invalid guard triangles')
        self.centres = self.triangles.mean(axis=1)
        self.radii = np.linalg.norm(
            self.triangles-self.centres[:,None,:],axis=2).max(axis=1)
        self.max_radius = float(self.radii.max())
        self.tree = cKDTree(self.centres)
        self.minimum_m = float(self.metadata['minimum_clearance_m'])
        self.bounds = np.asarray(self.metadata['allowed_xy_bounds_m'])
        self.z_bounds = self.metadata.get('allowed_body_z_bounds_m')
        if (self.minimum_m != .55 or self.bounds.shape != (2,2)
                or not np.isfinite(self.bounds).all() or np.any(self.bounds[1]<=self.bounds[0])):
            raise ValueError('Unexpected mesh guard margin/bounds')
        if self.z_bounds is not None and (np.asarray(self.z_bounds).shape!=(2,)
                or not np.isfinite(self.z_bounds).all() or self.z_bounds[1]<=self.z_bounds[0]):
            raise ValueError('Invalid scan height envelope')
        boxes=self.metadata.get('recovery_xy_boxes_m')
        self.recovery_boxes=np.asarray(boxes,dtype=float) if boxes is not None else None
        if self.recovery_boxes is not None:
            lower=self.metadata.get('recovery_continuous_clearance_lower_bound_m',0.)
            if (self.recovery_boxes.ndim!=3 or self.recovery_boxes.shape[1:]!=(2,2)
                    or not len(self.recovery_boxes) or not np.isfinite(self.recovery_boxes).all()
                    or np.any(self.recovery_boxes[:,1]<=self.recovery_boxes[:,0])
                    or self.z_bounds is None or not np.isfinite(lower) or lower<.60
                    or np.any(self.recovery_boxes[:,0]<self.bounds[0])
                    or np.any(self.recovery_boxes[:,1]>self.bounds[1])):
                raise ValueError('Invalid certified descent region')
        self.transit_boxes = None
        self.transit_z_bounds = None
        if 'transit_xy_boxes_m' in self.metadata:
            self.transit_boxes = np.asarray(self.metadata['transit_xy_boxes_m'], float)
            self.transit_z_bounds = np.asarray(self.metadata.get('transit_body_z_bounds_m'), float)
            certificate = self.metadata.get('transit_certificate', {})
            if (self.transit_boxes.ndim != 3 or self.transit_boxes.shape[1:] != (2,2)
                    or not len(self.transit_boxes) or not np.isfinite(self.transit_boxes).all()
                    or np.any(self.transit_boxes[:,1] <= self.transit_boxes[:,0])
                    or np.any(self.transit_boxes[:,0] < self.bounds[0])
                    or np.any(self.transit_boxes[:,1] > self.bounds[1])
                    or self.transit_z_bounds.shape != (2,) or not np.isfinite(self.transit_z_bounds).all()
                    or self.recovery_boxes is None
                    or self.z_bounds is None or self.transit_z_bounds[0] < self.z_bounds[0]
                    or self.transit_z_bounds[1] > self.z_bounds[1]
                    or self.transit_z_bounds[1] <= self.transit_z_bounds[0]
                    or certificate.get('certified_z_bounds_m') != self.transit_z_bounds.tolist()
                    or certificate.get('sampled_bounds_m') != self.bounds.tolist()
                    or not np.isfinite(self.metadata.get('transit_continuous_clearance_lower_bound_m', 0))
                    or self.metadata.get('transit_continuous_clearance_lower_bound_m', 0) < .65
                    or self.metadata.get('recovery_policy') not in (
                        'planned_return_with_certified_rtl','connected_route_to_certified_landing')
                    or (self.metadata.get('recovery_policy') == 'connected_route_to_certified_landing' and
                        certificate.get('connected_recovery_required') is not True)):
                raise ValueError('Invalid certified transit region or recovery policy')

    @property
    def flight_boxes(self):
        if self.transit_boxes is None:
            return self.recovery_boxes
        if self.metadata.get('transit_includes_descent_region', False):
            # A full descent prism also certifies its cruise-height subset.
            # Retain existing landable airspace when adding transit-only cells.
            return np.concatenate([self.recovery_boxes, self.transit_boxes])
        return self.transit_boxes

    def inside_flight_region(self, xy):
        boxes = self.flight_boxes
        return boxes is None or bool(np.any(np.all((np.asarray(xy)>=boxes[:,0]) &
                                                    (np.asarray(xy)<=boxes[:,1]),axis=1)))

    def inside_recovery_region(self, xy):
        if self.recovery_boxes is None:return True
        xy=np.asarray(xy)
        return bool(np.any(np.all((xy>=self.recovery_boxes[:,0]) &
                                  (xy<=self.recovery_boxes[:,1]),axis=1)))

    def recovery_segment_covered(self, start, end, boxes=None):
        """Exact union-of-rectangles coverage; endpoints alone can miss a hole."""
        boxes = self.recovery_boxes if boxes is None else boxes
        if boxes is None:return True
        start,end=np.asarray(start),np.asarray(end)
        delta=end-start
        lower=np.zeros(len(boxes));upper=np.ones(len(lower))
        for axis in range(2):
            if abs(delta[axis])<1e-12:
                outside=(start[axis]<boxes[:,0,axis]) | (start[axis]>boxes[:,1,axis])
                upper[outside]=-1.
            else:
                a=(boxes[:,0,axis]-start[axis])/delta[axis]
                b=(boxes[:,1,axis]-start[axis])/delta[axis]
                lower=np.maximum(lower,np.minimum(a,b));upper=np.minimum(upper,np.maximum(a,b))
        intervals=sorted(zip(lower[upper>=lower],upper[upper>=lower]))
        covered=0.
        for begin,finish in intervals:
            if begin>covered+1e-10:return False
            covered=max(covered,finish)
            if covered>=1.-1e-10:return True
        return False

    def clearance_candidates(self, point):
        """Prune using enclosing spheres, with an exact achievable upper bound.

        A triangle's entire surface is inside its centroid sphere. Therefore
        centre distance minus radius is a lower bound, including huge triangles
        whose centroids are far away. No potentially nearer surface is dropped.
        """
        point = np.asarray(point, dtype=float)
        if point.shape != (3,) or not np.isfinite(point).all():
            raise ValueError('Non-finite or invalid guard point')
        _, nearest = self.tree.query(point)
        upper = float(triangle_distances(point, self.triangles[nearest:nearest+1])[0])
        indices = np.asarray(self.tree.query_ball_point(
            point, upper+self.max_radius+1e-9), dtype=np.intp)
        lower = np.linalg.norm(self.centres[indices]-point, axis=1)-self.radii[indices]
        return indices[lower <= upper+1e-9]

    def clearance(self, point):
        indices = self.clearance_candidates(point)
        return float(triangle_distances(point,self.triangles[indices]).min())

    def fault(self, point, clearance=None, transit=False):
        if np.any(np.asarray(point)[:2]<self.bounds[0]) or np.any(np.asarray(point)[:2]>self.bounds[1]):
            return 'outside inspected scan flight envelope'
        if self.z_bounds is not None and not self.z_bounds[0] <= point[2] <= self.z_bounds[1]:
            return 'outside inspected scan height envelope'
        if transit and self.transit_boxes is not None:
            if not self.transit_z_bounds[0] <= point[2] <= self.transit_z_bounds[1]:
                return 'outside certified transit height band'
            if not self.inside_flight_region(np.asarray(point)[:2]):
                return 'outside certified scan transit region'
        elif not self.inside_recovery_region(np.asarray(point)[:2]):
            return 'outside certified scan descent region'
        clearance = self.clearance(point) if clearance is None else clearance
        if clearance < self.minimum_m:
            return f'scan mesh clearance below {self.minimum_m:.2f} m: {clearance:.3f} m'
        return ''

    def approaching_boundary(self, point, velocity, horizon_s=.6, transit=False):
        """Early abort reserve based on measured motion; not a stopping guarantee."""
        predicted=np.asarray(point)[:2]+horizon_s*np.asarray(velocity)[:2]
        if not np.isfinite(predicted).all():return 'invalid scan guard velocity'
        if np.any(predicted<self.bounds[0]) or np.any(predicted>self.bounds[1]):
            return 'approaching inspected scan flight boundary'
        boxes = self.flight_boxes if transit else self.recovery_boxes
        if not self.recovery_segment_covered(np.asarray(point)[:2],predicted,boxes):
            return 'approaching uncertified scan transit region' if transit else 'approaching uncertified scan descent region'
        return ''


def box_clearance_lower_bound(guard, bounds, maximum_step_m=.05):
    """Conservative continuous-box bound using the 1-Lipschitz distance field."""
    bounds=np.asarray(bounds,dtype=float)
    if bounds.shape!=(2,3) or not np.isfinite(bounds).all() or np.any(bounds[1]<=bounds[0]):
        raise ValueError('Invalid recovery box')
    if not 0 < maximum_step_m <= .05:raise ValueError('Recovery grid step must be at most 0.05 m')
    axes=[np.linspace(lo,hi,int(np.ceil((hi-lo)/maximum_step_m))+1)
          for lo,hi in zip(*bounds)]
    points=np.stack(np.meshgrid(*axes,indexing='ij'),axis=-1).reshape(-1,3)
    sampled=min(guard.clearance(p) for p in points)
    covering_radius=float(np.linalg.norm([np.diff(axis).max()/2 for axis in axes]))
    return dict(sample_count=len(points),sampled_minimum_m=sampled,
                grid_covering_radius_m=covering_radius,
                continuous_clearance_lower_bound_m=sampled-covering_radius,
                bounds_m=bounds.tolist())

def load_stl(path):
    """Read the binary STL produced by the pinned scene converter."""
    with Path(path).open('rb') as stream:
        stream.read(80)
        count = int.from_bytes(stream.read(4),'little')
        data = np.fromfile(stream,dtype=[('normal','<f4',3),('vertices','<f4',(3,3)),('attr','<u2')],count=count)
    if len(data)!=count or Path(path).stat().st_size != 84+50*count:
        raise ValueError('Invalid binary STL length')
    return data['vertices'].astype(float)
