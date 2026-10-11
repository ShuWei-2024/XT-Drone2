#!/usr/bin/env python3
"""Prepare a bounded, mesh-checked indoor SITL route; launches no simulation."""
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from scan_mesh_guard import MeshGuard, load_stl, box_clearance_lower_bound

REPO = Path(__file__).resolve().parents[2]


def main():
    scene=REPO/'astra/runtime/maps/replica_office0'
    manifest=json.loads((scene/'manifest.json').read_text())
    collision=scene/'replica_office0/meshes/collision.stl'
    digest=hashlib.sha256(collision.read_bytes()).hexdigest()
    if digest!=manifest['asset_sha256']['collision.stl']:raise ValueError('Mesh changed')
    triangles=load_stl(collision)
    # Ignore only triangles entirely in the near-ground support layer, which
    # must be approached during takeoff/landing. Furniture and ceiling remain.
    keep=triangles[:,:,2].max(axis=1)>.12
    meta=dict(collision_sha256=digest,minimum_clearance_m=.55,
              reference='base_link at rotated model-to-body offset [0,0,.24]',
              allowed_xy_bounds_m=[[-.40,-.25],[.30,.20]],
              allowed_body_z_bounds_m=[.20,2.34],
              excluded_floor_max_z_m=.12,excluded_floor_triangles=int((~keep).sum()),
              retained_triangles=int(keep.sum()),missing_geometry_certified=False)
    asset=scene/'mesh_guard.npz'
    np.savez_compressed(asset,triangles=triangles[keep],metadata=json.dumps(meta))
    guard=MeshGuard(asset)
    # Candidate goal coordinates are independent of measured truth. Geometry
    # only selects a test route offline and stops unsafe execution online.
    route=np.array([[-.2,0],[-.2,.10],[0,.10],[0,0]])
    line=np.vstack([np.linspace(a,b,31) for a,b in zip(np.vstack([[0,0],route[:-1]]),route)])
    positions=np.vstack([np.column_stack([line,np.full(len(line),z)]) for z in [1.79,1.94,2.09]])
    chosen=dict(route=route.tolist(),nominal_path_minimum_m=min(guard.clearance(p) for p in positions))
    # A safe cruise height alone does not ensure safe descent beside furniture.
    recovery=box_clearance_lower_bound(guard,[[-.40,-.25,.20],[.30,.20,2.34]])
    if recovery['continuous_clearance_lower_bound_m'] < .60:
        raise ValueError('Insufficient known-surface clearance in the complete descent envelope')
    climb=np.column_stack([np.zeros((100,2)),np.linspace(.24,2.09,100)])
    climb_min=min(guard.clearance(p) for p in climb)
    if min(chosen['nominal_path_minimum_m'],climb_min)<.70:
        raise ValueError('No route with sufficient nominal room margin')
    meta.update(route_offsets_m=chosen['route'],nominal_path_minimum_m=chosen['nominal_path_minimum_m'],
                recovery_box=recovery,
                world_sha256=hashlib.sha256((scene/'world.sdf').read_bytes()).hexdigest(),
                model_sdf_sha256=hashlib.sha256((scene/'replica_office0/model.sdf').read_bytes()).hexdigest(),
                nominal_climb_minimum_m=climb_min,
                guard_sha256=hashlib.sha256(asset.read_bytes()).hexdigest(),
                truth_used_for_setpoints=False,flight_validated=False)
    (scene/'flight_precheck.json').write_text(json.dumps(meta,indent=2)+'\n')
    (REPO/'astra/config/route_replica_office0.json').write_text(json.dumps(chosen['route'])+'\n')
    print(json.dumps(meta,indent=2))


if __name__=='__main__':main()
