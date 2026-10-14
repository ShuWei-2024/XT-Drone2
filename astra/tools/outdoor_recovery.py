"""Conservative continuous descent regions for prepared static outdoor SITL."""
import numpy as np
from scipy.ndimage import label


def recovery_rectangles(triangles,bounds,z_bounds,step=.10,margin=.65,
                        refine_guard=None,refinement_bounds=None):
    """Certify prisms by conservative triangle bounding boxes, without ray assumptions.

    Triangles farther than margin vertically need no XY exclusion. Every other
    triangle's XY rectangle is a superset, so a rectangle distance >= margin is
    a continuous lower bound on all points in the candidate descent prism.
    """
    tri=np.asarray(triangles,float);bounds=np.asarray(bounds,float)
    relevant=(tri[:,:,2].max(axis=1)>=z_bounds[0]-margin)&(tri[:,:,2].min(axis=1)<=z_bounds[1]+margin)
    footprint=tri[relevant,:,:2]
    low=footprint.min(axis=1);high=footprint.max(axis=1)
    xs=np.linspace(bounds[0,0],bounds[1,0],int(np.ceil(np.diff(bounds[:,0])[0]/step))+1)
    ys=np.linspace(bounds[0,1],bounds[1,1],int(np.ceil(np.diff(bounds[:,1])[0]/step))+1)
    free=np.ones((len(xs)-1,len(ys)-1),dtype=bool)
    for i,(x0,x1) in enumerate(zip(xs[:-1],xs[1:])):
        # A rectangle farther than margin on X cannot invalidate this row.
        # Keep equality candidates so this pruning preserves the certificate.
        dx=np.maximum(np.maximum(low[:,0]-x1,x0-high[:,0]),0)
        candidates=dx<=margin
        row_low=low[candidates,1];row_high=high[candidates,1]
        dx2=dx[candidates]**2
        for j,(y0,y1) in enumerate(zip(ys[:-1],ys[1:])):
            dy=np.maximum(np.maximum(row_low-y1,y0-row_high),0)
            free[i,j]=not np.any(dx2+dy**2<margin**2)
    refined=0
    if refine_guard is not None:
        region=np.asarray(refinement_bounds if refinement_bounds is not None else bounds,float)
        if region.shape!=(2,2) or not np.isfinite(region).all() or np.any(region[1]<=region[0]):
            raise ValueError('Invalid descent refinement bounds')
        zs=np.linspace(z_bounds[0],z_bounds[1],int(np.ceil((z_bounds[1]-z_bounds[0])/.4))+1)
        for i,j in np.argwhere(~free):
            x=(xs[i]+xs[i+1])/2;y=(ys[j]+ys[j+1])/2
            if np.any([x,y]<region[0]) or np.any([x,y]>region[1]):continue
            # Distance to triangle surfaces is 1-Lipschitz. Subtract the
            # covering radius of the entire prism, including between heights.
            cover=np.linalg.norm([(xs[i+1]-xs[i])/2,(ys[j+1]-ys[j])/2,np.diff(zs).max()/2])
            if all(refine_guard.clearance([x,y,z])-cover>=margin+1e-9 for z in zs):
                free[i,j]=True;refined+=1
    components,_=label(free)
    spawn=np.argwhere((xs[:-1,None]<=0)&(xs[1:,None]>=0)&
        (ys[None,:-1]<=0)&(ys[None,1:]>=0)&(components>0))
    if not len(spawn):raise ValueError('No certified park descent region at spawn')
    component=components[tuple(spawn[0])]
    cells=np.argwhere(components==component)
    # Merge runs with identical Y extents across adjacent X rows. The union is
    # unchanged, while online segment/goal checks no longer scan 35k tiny cells.
    boxes=[];previous={}
    for i,row in enumerate(components==component):
        edges=np.diff(np.r_[False,row,False].astype(int))
        runs=list(zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1)))
        current={}
        for begin,end in runs:
            key=(int(begin),int(end))
            if key in previous:
                index=previous[key];boxes[index][1][0]=float(xs[i+1])
            else:
                index=len(boxes);boxes.append([[float(xs[i]),float(ys[begin])],[float(xs[i+1]),float(ys[end])]])
            current[key]=index
        previous=current
    return dict(recovery_xy_boxes_m=boxes,recovery_continuous_clearance_lower_bound_m=margin,
        recovery_certificate=dict(method='continuous XY rectangle distances to conservative triangle bounding boxes; vertical gap exclusion',
            grid_step_xy_m=step,connected_cell_count=len(cells),compacted=True,rectangle_count=len(boxes),
            area_m2=float(len(cells)*np.diff(xs)[0]*np.diff(ys)[0]),
            sampled_bounds_m=bounds.tolist(),certified_z_bounds_m=list(z_bounds),missing_geometry_certified=False,
            refined_cell_count=refined,refinement_method='exact triangle distance samples minus continuous prism covering radius' if refine_guard is not None else None,
            refinement_bounds_m=region.tolist() if refine_guard is not None else None))
