#!/usr/bin/env python3
"""Prepare goal-only scan SITL with a certified known-surface descent region.

Geometry selects a bounded test and checks recovery, never planner inputs.
Downloaded scans and generated certificates stay under ignored runtime.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time
import tempfile

import numpy as np
from scipy.ndimage import label
from scan_mesh_guard import MeshGuard, load_stl
from scan_trajectory_audit import line_audit


def recovery_grid(guard, bounds, z_bounds, step_xy=.05, step_z=.05):
    bounds=np.asarray(bounds,float)
    if (bounds.shape!=(2,2) or not np.isfinite(bounds).all() or
            np.any(bounds[1]<=bounds[0]) or not 0<step_xy<=.10 or not 0<step_z<=.05):
        raise ValueError('Invalid recovery grid')
    axes=[np.linspace(lo,hi,int(np.ceil((hi-lo)/step_xy))+1) for lo,hi in zip(*bounds)]
    zs=np.linspace(*z_bounds,int(np.ceil((z_bounds[1]-z_bounds[0])/step_z))+1)
    distances=np.empty((len(axes[0]),len(axes[1])))
    for i,x in enumerate(axes[0]):
        for j,y in enumerate(axes[1]):
            distances[i,j]=min(guard.clearance([x,y,z]) for z in zs)
        if i%10==0:print('Recovery columns',i,len(axes[0]),flush=True)
    # Every point of a cell's complete vertical prism is within this radius
    # of a sampled corner/height. Surface distance is 1-Lipschitz.
    radius=float(np.linalg.norm([np.diff(a).max()/2 for a in [*axes,zs]]))
    cell_min=np.minimum.reduce([distances[:-1,:-1],distances[1:,:-1],
                                distances[:-1,1:],distances[1:,1:]])
    lower=cell_min-radius
    components,_=label(lower>=.60)
    spawn_cells=[(i,j) for i in range(len(axes[0])-1) for j in range(len(axes[1])-1)
                 if axes[0][i]<=0<=axes[0][i+1] and axes[1][j]<=0<=axes[1][j+1]
                 and components[i,j]]
    if not spawn_cells:raise ValueError('No certified descent cell at spawn')
    component=components[spawn_cells[0]]
    cells=np.argwhere(components==component)
    boxes=[[[axes[0][i],axes[1][j]],[axes[0][i+1],axes[1][j+1]]] for i,j in cells]
    return dict(recovery_xy_boxes_m=boxes,
                recovery_continuous_clearance_lower_bound_m=float(lower[components==component].min()),
                recovery_certificate=dict(method='corner/height samples minus 3D covering radius',
                    covering_radius_m=radius,grid_step_xy_m=step_xy,grid_step_z_m=step_z,
                    column_count=int(distances.size),height_count=len(zs),
                    connected_cell_count=len(cells),sampled_bounds_m=bounds.tolist(),
                    certified_z_bounds_m=list(z_bounds),missing_geometry_certified=False))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene-dir',type=Path,required=True)
    p.add_argument('--goal-xy',nargs=2,type=float,required=True)
    p.add_argument('--xy-bounds',nargs=4,type=float,required=True,metavar=('XMIN','YMIN','XMAX','YMAX'))
    p.add_argument('--takeoff-height-m',type=float,default=.9)
    p.add_argument('--alignment-height-m',type=float,default=1.60)
    a=p.parse_args()
    if not np.isfinite(a.goal_xy).all() or not .8<=a.takeoff_height_m<=1.7:p.error('Invalid goal/height')
    if not np.isfinite(a.alignment_height_m) or not a.takeoff_height_m<=a.alignment_height_m<=1.7:
        p.error('Invalid alignment height')
    scene=a.scene_dir.resolve();manifest=json.loads((scene/'manifest.json').read_text())
    model=manifest['model_name'];collision=scene/model/'meshes/collision.stl'
    sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
    digest=sha(collision)
    if digest!=manifest['asset_sha256']['collision.stl']:raise ValueError('Collision mesh changed')
    for path in ['mesh_guard.npz','flight_precheck.json','route.json']:
        if (scene/path).exists():raise ValueError('Refusing to overwrite prepared flight: '+path)
    triangles=load_stl(collision);keep=triangles[:,:,2].max(axis=1)>.12
    z_bounds=[.20,max(a.takeoff_height_m+.24+.50,a.alignment_height_m+.24+.16)]
    bounds=np.asarray(a.xy_bounds).reshape(2,2)
    meta=dict(collision_sha256=digest,minimum_clearance_m=.55,
        reference='base_link at rotated model-to-body offset [0,0,.24]',
        allowed_xy_bounds_m=bounds.tolist(),allowed_body_z_bounds_m=z_bounds,
        excluded_floor_max_z_m=.12,excluded_floor_triangles=int((~keep).sum()),
        retained_triangles=int(keep.sum()),missing_geometry_certified=False)
    with tempfile.NamedTemporaryFile(prefix='.mesh_guard_preparing_',suffix='.npz',dir=scene,delete=False) as temp:
        asset=Path(temp.name)
    np.savez_compressed(asset,triangles=triangles[keep],metadata=json.dumps(meta))
    try:
        guard=MeshGuard(asset);started=time.monotonic()
        meta.update(recovery_grid(guard,bounds,z_bounds))
        np.savez_compressed(asset,triangles=triangles[keep],metadata=json.dumps(meta))
        guard=MeshGuard(asset)
        cruise=a.takeoff_height_m+.24
        climb_min=min(guard.clearance([0,0,z]) for z in np.linspace(.24,a.alignment_height_m+.24+.15,100))
        goal_min=min(guard.clearance([*a.goal_xy,z]) for z in np.linspace(cruise-.15,cruise+.15,31))
        if not guard.inside_recovery_region(a.goal_xy):raise ValueError('Goal outside certified descent region')
        if min(climb_min,goal_min)<.70:raise ValueError('Insufficient nominal spawn/goal clearance')
        direct=line_audit(guard,[0,0,cruise],[*a.goal_xy,cruise])
        if not direct['violates_clearance_margin']:raise ValueError('Goal does not require known-surface detour')
        route=[a.goal_xy,[0,0]]
        precheck=dict(meta,route_offsets_m=route,takeoff_height_m=a.takeoff_height_m,
            alignment_height_m=a.alignment_height_m,
            planner_virtual_ceiling_m=a.takeoff_height_m+.6,
            optimization_clearance_m=.2,
            static_obstacle_retention_sec=180.,
            static_descent_columns=True,
            nominal_climb_minimum_m=climb_min,nominal_goal_minimum_m=goal_min,
            direct_path=direct,world_sha256=sha(scene/'world.sdf'),
            model_sdf_sha256=sha(scene/model/'model.sdf'),guard_sha256=sha(asset),
            preparation_wall_s=time.monotonic()-started,truth_used_for_setpoints=False,
            planned_detour_supplied=False,flight_validated=False)
        asset.replace(scene/'mesh_guard.npz')
        (scene/'flight_precheck.json').write_text(json.dumps(precheck,indent=2)+'\n')
        (scene/'route.json').write_text(json.dumps(route)+'\n')
        print(json.dumps({k:v for k,v in precheck.items() if k!='recovery_xy_boxes_m'},indent=2))
    except BaseException:
        asset.unlink(missing_ok=True)
        raise


if __name__=='__main__':main()
