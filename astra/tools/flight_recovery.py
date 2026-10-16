"""Geometry and policy shared by the SITL supervisor and mission UI.

Native RTL does not use EGO's map. Its opt-in corridor must be a conservative
subset of the inspected cruise volume, with a clear straight path to home.
"""
import numpy as np


def spline_height_band(guard, ground_odom_z):
    """Known zero-yaw SITL spawn: body origin 0.24 m above model origin.

    Reserve 0.10 m inside the inspected body-height interval. Never use truth
    to fit a control transform; the ground odometry reference stays frozen.
    """
    if guard.transit_z_bounds is None:return None
    if not np.isfinite(ground_odom_z):raise ValueError('Invalid ground height reference')
    return (guard.transit_z_bounds-.24+ground_odom_z+np.array([.1,-.1])).tolist()


def rectangle_covered(boxes, lower, upper):
    """Exact closed rectangle coverage by a finite rectangle union, including holes."""
    boxes=np.asarray(boxes,dtype=float)
    xs=np.unique(np.r_[lower[0],upper[0],
        np.clip(boxes[:,:,0].ravel(),lower[0],upper[0])])
    for a,b in zip(xs[:-1],xs[1:]):
        if b-a<=1e-12:continue
        x=(a+b)/2
        active=boxes[(boxes[:,0,0]<=x)&(boxes[:,1,0]>=x)]
        intervals=sorted((max(lo[1],lower[1]),min(hi[1],upper[1]))
                         for lo,hi in active if hi[1]>=lower[1] and lo[1]<=upper[1])
        end=lower[1]
        for lo,hi in intervals:
            if lo>end+1e-10:break
            end=max(end,hi)
        if end<upper[1]-1e-10:return False
    return True


def landing_allowed(guard, xy, reserve=.25):
    if guard is None or guard.recovery_boxes is None:
        return guard is None
    xy = np.asarray(xy, float)
    return (xy.shape == (2,) and np.isfinite(xy).all() and
            rectangle_covered(guard.recovery_boxes, xy-reserve, xy+reserve))


def direct_home_mask(free, home_cell, padding=2):
    """Accept only cells whose entire rectangle to home (plus reserve) is free.

    More conservative than a line check; integral sums bound preparation cost.
    Every possible direct RTL segment from an accepted cell is contained in
    this rectangle. Padding protects home/estimator/tracking uncertainty.
    """
    free = np.asarray(free, bool)
    hx, hy = home_cell
    yy, xx = np.indices(free.shape)
    x0=np.minimum(xx,hx)-padding; x1=np.maximum(xx,hx)+padding+1
    y0=np.minimum(yy,hy)-padding; y1=np.maximum(yy,hy)+padding+1
    valid=(x0>=0)&(y0>=0)&(x1<=free.shape[1])&(y1<=free.shape[0])
    sums=np.pad((~free).astype(np.int64),((1,0),(1,0))).cumsum(0).cumsum(1)
    a,b=np.clip(x0,0,free.shape[1]),np.clip(x1,0,free.shape[1])
    c,d=np.clip(y0,0,free.shape[0]),np.clip(y1,0,free.shape[0])
    return valid & ((sums[d,b]-sums[c,b]-sums[d,a]+sums[c,a])==0)


def compact_mask(mask, origin, step):
    boxes=[]; previous={}
    for y,row in enumerate(mask):
        edges=np.diff(np.r_[False,row,False].astype(int)); current={}
        for a,b in zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1)):
            key=(int(a),int(b))
            if key in previous:
                index=previous[key]; boxes[index][1][1]=float(origin[1]+(y+1)*step)
            else:
                index=len(boxes)
                boxes.append([[float(origin[0]+a*step),float(origin[1]+y*step)],
                              [float(origin[0]+b*step),float(origin[1]+(y+1)*step)]])
            current[key]=index
        previous=current
    return boxes
