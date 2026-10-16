#!/usr/bin/env python3
"""Compare static SITL envelopes and bound every newly landable prism.

Preparation/geometry audit only: this never certifies a flight or feeds truth
to a controller. Original and candidate collision geometry must be identical.
"""
import argparse
import json
from pathlib import Path
import time

import numpy as np
from planning_envelope import PlanningEnvelope
from scan_mesh_guard import MeshGuard


def rasterize_boxes(boxes, bounds, step):
    size = np.rint((bounds[1]-bounds[0])/step).astype(int)
    mask = np.zeros(tuple(size[::-1]), bool)
    for lo, hi in boxes:
        begin = np.ceil((lo-bounds[0])/step-1e-9).astype(int)
        end = np.floor((hi-bounds[0])/step+1e-9).astype(int)
        mask[begin[1]:end[1], begin[0]:end[0]] = True
    return mask


def audit(original, candidate):
    a, b = (MeshGuard(p/'mesh_guard.npz') for p in (original, candidate))
    for key in ('allowed_xy_bounds_m', 'allowed_body_z_bounds_m', 'minimum_clearance_m',
                'transit_body_z_bounds_m', 'recovery_policy', 'scene_edge_reserve_m',
                'recovery_continuous_clearance_lower_bound_m'):
        if a.metadata.get(key) != b.metadata.get(key):
            raise ValueError('Changed geometry/safety contract: '+key)
    if not np.array_equal(a.triangles, b.triangles):
        raise ValueError('Collision triangles changed')
    for name in ('world.sdf', 'astra_baylands/model.sdf'):
        if (original/name).read_bytes() != (candidate/name).read_bytes():
            raise ValueError('World/model changed: '+name)
    step = b.metadata['recovery_certificate']['grid_step_xy_m']
    old = rasterize_boxes(a.recovery_boxes, a.bounds, step)
    new = rasterize_boxes(b.recovery_boxes, b.bounds, step)
    if np.any(old & ~new):
        raise ValueError('Expansion removed originally landable cells')
    yy, xx = np.nonzero(new & ~old)
    zs = np.linspace(*b.z_bounds, int(np.ceil(np.diff(b.z_bounds)[0]/.4))+1)
    cover = float(np.linalg.norm([step/2, step/2, np.diff(zs).max()/2]))
    minimum = float('inf'); began = time.monotonic()
    for index, (x,y) in enumerate(zip(xx,yy)):
        centre = b.bounds[0]+(np.array([x,y])+.5)*step
        lower = min(a.clearance([*centre,z]) for z in zs)-cover
        minimum = min(minimum, lower)
        if lower < .65:
            raise ValueError(f'New descent prism lacks continuous margin at {centre}: {lower}')
        if (index+1) % 1000 == 0:
            print(f'Checked {index+1}/{len(xx)} new prisms', flush=True)
    ea, eb = (PlanningEnvelope(g.flight_boxes,g.bounds,[0,0],connected_to_origin=True) for g in (a,b))
    removed = int((ea.guide_free & ~eb.guide_free).sum())
    if removed:
        raise ValueError('Expansion removed formerly selectable cells')
    summary = dict(original=str(original),candidate=str(candidate),
        collision_geometry_unchanged=True, world_and_model_unchanged=True,
        safety_margins_unchanged=True, removed_selectable_cells=removed,
        checked_new_descent_prisms=len(xx), new_descent_area_m2=len(xx)*step**2,
        new_prisms_continuous_clearance_lower_bound_m=minimum if len(xx) else None,
        original_selectable_area_m2=float(ea.guide_free.sum()*ea.resolution**2),
        candidate_selectable_area_m2=float(eb.guide_free.sum()*eb.resolution**2),
        added_selectable_area_m2=float((eb.guide_free & ~ea.guide_free).sum()*eb.resolution**2),
        elapsed_sec=time.monotonic()-began,
        flight_validated=False, missing_geometry_certified=False)
    return summary, ea, eb


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--original',type=Path,required=True)
    p.add_argument('--candidate',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--plot',type=Path)
    args=p.parse_args(); summary,a,b=audit(args.original,args.candidate)
    args.output.write_text(json.dumps(summary,indent=2)+'\n')
    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.colors import ListedColormap
        image=np.zeros(a.guide_free.shape,int);image[a.guide_free]=1
        image[b.guide_free & ~a.guide_free]=2
        fig,ax=plt.subplots(figsize=(8,7))
        ax.imshow(image,origin='lower',extent=[a.origin[0],a.origin[0]+image.shape[1]*a.resolution,
            a.origin[1],a.origin[1]+image.shape[0]*a.resolution],
            cmap=ListedColormap(['#e6e7e9','#73afa8','#ffad32']),vmin=0,vmax=2)
        ax.plot(0,0,'r+',label='Takeoff')
        from matplotlib.patches import Patch
        ax.legend(handles=[Patch(color='#73afa8',label='Existing selectable area'),
            Patch(color='#ffad32',label='Added after mesh-distance checks'),
            Patch(color='#e6e7e9',label='Outside static selection mask')],loc='lower left')
        ax.set(xlabel='X (m)',ylabel='Y (m)',title='Baylands: unchanged margins, refined geometry')
        fig.tight_layout();fig.savefig(args.plot,dpi=160);plt.close(fig)
    print(json.dumps(summary,indent=2),flush=True)


if __name__=='__main__': main()
