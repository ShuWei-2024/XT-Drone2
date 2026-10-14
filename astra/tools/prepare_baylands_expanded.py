#!/usr/bin/env python3
"""Expand a copied Baylands geometry certificate; never certify flight by preparation.

Keeps the original collision, visuals and flat support plane unchanged. Current
fixed-height flight excludes terrain rising into the descent prism; this does
not implement terrain following or prove tree branches are sensor-visible.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np

from outdoor_recovery import recovery_rectangles
from outdoor_envelope import outdoor_envelope
from scan_mesh_guard import MeshGuard, load_stl
from scan_trajectory_audit import line_audit

ROUTES={
    'east':[[7,0],[7,4],[14,4],[20,6],[20,12],[12,12],[6,6],[0,0]],
    'tree': [[-7,0],[-14,0],[-21,0],[-22,10],[-14,14],[-7,7],[0,7],[0,0]],
    'shared-tree': [[-7,0],[-14,0],[-21,0],[-22,10]],
    'west': [[-7,0],[-14,0],[-21,0],[-21,-6],[-14,-6],[-7,-3],[0,-3],[0,0]],
}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--route-kind',choices=ROUTES,required=True)
    parser.add_argument('--refine-descent-prisms',action='store_true',
                        help='Exact continuous distance bounds around western trees; same 0.65 m margin')
    args=parser.parse_args();source=args.source.resolve();out=args.output.resolve()
    manifest=json.loads((source/'manifest.json').read_text())
    pre=json.loads((source/'flight_precheck.json').read_text())
    sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
    if manifest.get('scene')!='baylands' or manifest.get('model_name')!='astra_baylands':
        raise ValueError('Expected prepared Baylands scene')
    for name,digest in manifest['asset_sha256'].items():
        if sha(source/'astra_baylands/meshes'/name)!=digest:
            raise ValueError('Source asset hash mismatch: '+name)
    for name,key in [('world.sdf','world_sha256'),('astra_baylands/model.sdf','model_sdf_sha256'),('mesh_guard.npz','guard_sha256')]:
        if sha(source/name)!=pre[key]:raise ValueError('Source precheck mismatch: '+name)
    out.mkdir(parents=True,exist_ok=False)
    shutil.copytree(source/'astra_baylands',out/'astra_baylands')
    for name in ['world.sdf','LICENSE.txt','manifest.json']:shutil.copy2(source/name,out/name)
    triangles=load_stl(out/'astra_baylands/meshes/collision.stl')
    keep=triangles[:,:,2].max(1)>.12
    original_guard=MeshGuard(source/'mesh_guard.npz');meta=original_guard.metadata.copy()
    meta.update(allowed_xy_bounds_m=[[-26.,-26.],[26.,26.]],interactive_goal_radius_m=25.)
    meta.update(recovery_rectangles(triangles[keep],meta['allowed_xy_bounds_m'],meta['allowed_body_z_bounds_m'],
                refine_guard=original_guard if args.refine_descent_prisms else None,
                refinement_bounds=[[-24.,2.],[-5.,16.]]))
    np.savez_compressed(out/'mesh_guard.npz',triangles=triangles[keep],metadata=json.dumps(meta))
    guard=MeshGuard(out/'mesh_guard.npz');route=ROUTES[args.route_kind]
    for xy in route:
        if not guard.inside_recovery_region(xy):raise ValueError('Goal outside descent region: '+str(xy))
        if guard.clearance([*xy,1.14])<.7:raise ValueError('Goal lacks endpoint clearance: '+str(xy))
    starts=[[0,0],*route[:-1]]
    direct=[line_audit(guard,[*start,1.14],[*end,1.14]) for start,end in zip(starts,route)]
    for audit,start,end in zip(direct,starts,route):
        audit['direct_descent_region_covered']=guard.recovery_segment_covered(start,end)
    pre.update(meta,route_offsets_m=route,route_kind='expanded '+args.route_kind,
               planner_map_size_xy_m=64.,column_obstacle_max_z_m=3.,direct_path=direct[0],direct_route_legs=direct,
               planner_leg_timeout_s=500. if args.route_kind=='shared-tree' else 100.,
               guard_sha256=sha(out/'mesh_guard.npz'),flight_validated=False,
               terrain_following_implemented=False,flat_support_plane_preserved=True)
    outdoor_envelope(pre,meta)
    (out/'flight_precheck.json').write_text(json.dumps(pre,indent=2)+'\n')
    (out/'route.json').write_text(json.dumps(route)+'\n')
    print(json.dumps(dict(output=str(out),route=route,descent_area_m2=meta['recovery_certificate']['area_m2'],
                         direct_route_legs=direct,flight_validated=False),indent=2),flush=True)


if __name__=='__main__':main()
