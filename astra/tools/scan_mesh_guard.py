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
        self.max_radius = float(np.linalg.norm(
            self.triangles-self.centres[:,None,:],axis=2).max())
        self.tree = cKDTree(self.centres)
        self.minimum_m = float(self.metadata['minimum_clearance_m'])
        self.bounds = np.asarray(self.metadata['allowed_xy_bounds_m'])
        self.z_bounds = self.metadata.get('allowed_body_z_bounds_m')
        if self.minimum_m != .55 or self.bounds.shape != (2,2):
            raise ValueError('Unexpected mesh guard margin/bounds')

    def clearance(self, point):
        if not np.isfinite(point).all(): raise ValueError('Non-finite guard point')
        upper, _ = self.tree.query(point)
        indices = self.tree.query_ball_point(point, float(upper)+self.max_radius+1e-9)
        return float(triangle_distances(point,self.triangles[indices]).min())

    def fault(self, point, clearance=None):
        if np.any(np.asarray(point)[:2]<self.bounds[0]) or np.any(np.asarray(point)[:2]>self.bounds[1]):
            return 'outside inspected scan flight envelope'
        if self.z_bounds is not None and not self.z_bounds[0] <= point[2] <= self.z_bounds[1]:
            return 'outside inspected scan height envelope'
        clearance = self.clearance(point) if clearance is None else clearance
        if clearance < self.minimum_m:
            return f'scan mesh clearance below {self.minimum_m:.2f} m: {clearance:.3f} m'
        return ''

    def approaching_boundary(self, point, velocity, horizon_s=.6):
        """Early abort reserve based on measured motion; not a stopping guarantee."""
        predicted=np.asarray(point)[:2]+horizon_s*np.asarray(velocity)[:2]
        if not np.isfinite(predicted).all():return 'invalid scan guard velocity'
        if np.any(predicted<self.bounds[0]) or np.any(predicted>self.bounds[1]):
            return 'approaching inspected scan flight boundary'
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
