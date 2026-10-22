#!/usr/bin/env python3
"""Copy a prepared Baylands scene into an opt-in transit/landing SITL candidate.

Existing maps and flight parameters remain recoverable. Known static geometry
certifies a cruise band independently of the full vertical descent volume.
The legacy policy requires direct RTL. Connected recovery admits winding
cruise corridors and precomputes routes to descent sites before arming.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np

from flight_recovery import compact_mask, direct_home_mask
from outdoor_envelope import outdoor_envelope
from outdoor_recovery import recovery_rectangles
from planning_envelope import PlanningEnvelope
from scan_mesh_guard import MeshGuard
from scan_trajectory_audit import line_audit


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--validation-low-box',action='store_true',
                   help='Add a 0.35 m test box at (-1.5,0); fly over it at cruise altitude')
    p.add_argument('--whole-map',action='store_true',
                   help='Inspect the complete 70 m patch interior, retaining 5 m edge reserve')
    p.add_argument('--connected-recovery',action='store_true',
                   help='Admit connected cruise space; recover along checked edges to a landing site')
    p.add_argument('--scene-edge-reserve-m',type=float,choices=(1.,5.),default=5.,
                   help='Known-map outer border; 1 m requires whole-map connected recovery')
    p.add_argument('--refine-descent-prisms',action='store_true',
                   help='Recover conservative box exclusions using exact mesh-distance bounds; same margins')
    p.add_argument('--route-kind',choices=('short','east','west','cross-map'),
                   help='Whole-map test destinations, not intermediate detour waypoints')
    a=p.parse_args(); source=a.source.resolve(); out=a.output.resolve()
    if a.scene_edge_reserve_m != 5. and not (a.whole_map and a.connected_recovery):
        p.error('Reduced map border requires --whole-map --connected-recovery')
    if a.refine_descent_prisms and a.validation_low_box:
        raise ValueError('Refinement requires unchanged source collision geometry')
    pre=json.loads((source/'flight_precheck.json').read_text())
    manifest=json.loads((source/'manifest.json').read_text())
    sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
    if manifest.get('scene')!='baylands':raise ValueError('Prepared Baylands source required')
    for name,digest in manifest['asset_sha256'].items():
        if sha(source/'astra_baylands/meshes'/name)!=digest:raise ValueError('Source asset changed')
    for name,key in [('world.sdf','world_sha256'),('astra_baylands/model.sdf','model_sdf_sha256'),('mesh_guard.npz','guard_sha256')]:
        if sha(source/name)!=pre[key]:raise ValueError('Source precheck changed: '+name)
    guard=MeshGuard(source/'mesh_guard.npz')
    if guard.transit_boxes is not None:raise ValueError('Use original descent-only source')
    out.mkdir(parents=True,exist_ok=False)
    shutil.copytree(source/'astra_baylands',out/'astra_baylands')
    for name in ('world.sdf','manifest.json','LICENSE.txt'):shutil.copy2(source/name,out/name)
    tri=guard.triangles.copy(); meta=guard.metadata.copy()
    if a.whole_map:
        if manifest.get('patch_width_m') != 70.:
            raise ValueError('Whole-map preparation requires the pinned 70 m patch')
        extent=35.-a.scene_edge_reserve_m
        meta.update(allowed_xy_bounds_m=[[-extent,-extent],[extent,extent]],
                    interactive_goal_radius_m=(49. if extent>30. else 43.),whole_map_selection=True,
                    scene_edge_reserve_m=a.scene_edge_reserve_m)
    if a.validation_low_box:
        import trimesh
        box=trimesh.creation.box(extents=[.5,.7,.35]);box.apply_translation([-1.5,0,.175])
        tri=np.concatenate([tri,box.triangles])
        tree=ET.parse(out/'world.sdf');world=tree.getroot().find('world')
        world.append(ET.fromstring('''<model name="transit_validation_low_box"><static>true</static>
<pose>-1.5 0 .175 0 0 0</pose><link name="box">
<collision name="box"><geometry><box><size>.5 .7 .35</size></box></geometry></collision>
<visual name="box"><geometry><box><size>.5 .7 .35</size></box></geometry>
<material><ambient>.8 .3 .1 1</ambient><diffuse>.8 .3 .1 1</diffuse></material></visual></link></model>'''))
        tree.write(out/'world.sdf',encoding='utf-8',xml_declaration=True)
    if a.whole_map or a.validation_low_box or a.refine_descent_prisms:
        # A test box is absent from the source distance oracle. Never refine
        # against that incomplete oracle if geometry was added in this run.
        print('Preparing continuous descent prisms; exact refinement=' +
              str(a.refine_descent_prisms), flush=True)
        meta.update(recovery_rectangles(tri,meta['allowed_xy_bounds_m'],meta['allowed_body_z_bounds_m'],
                    refine_guard=guard if a.refine_descent_prisms else None))
    band=[1.2,2.0]
    certificate=recovery_rectangles(tri,meta['allowed_xy_bounds_m'],band)
    cruise=PlanningEnvelope(certificate['recovery_xy_boxes_m'],meta['allowed_xy_bounds_m'],[0,0])
    home=np.floor(-cruise.origin/cruise.resolution).astype(int)
    cert=certificate['recovery_certificate']
    if a.connected_recovery:
        boxes=certificate['recovery_xy_boxes_m']
        cert.update(method=cert['method']+'; connected recovery graph to certified landing roots',
                    connected_recovery_required=True)
    else:
        direct=direct_home_mask(cruise.free,home,padding=2)
        boxes=compact_mask(direct,cruise.origin,cruise.resolution)
        cert.update(method=cert['method']+'; conservative subset with rectangular native RTL corridor to home',
                    direct_home_rectangle_reserve_m=.2,area_m2=float(direct.sum()*cruise.resolution**2),
                    rectangle_count=len(boxes))
    meta.update(transit_xy_boxes_m=boxes,transit_body_z_bounds_m=band,
                transit_continuous_clearance_lower_bound_m=.65,transit_certificate=cert,
                recovery_policy=('connected_route_to_certified_landing' if a.connected_recovery
                                 else 'planned_return_with_certified_rtl'),
                transit_includes_descent_region=True,retained_triangles=len(tri),
                scene_label=('Baylands 全图可达空域' if a.whole_map else 'Baylands 独立通行／降落区试验'))
    np.savez_compressed(out/'mesh_guard.npz',triangles=tri,metadata=json.dumps(meta))
    prepared=MeshGuard(out/'mesh_guard.npz')
    envelope=PlanningEnvelope(prepared.flight_boxes,prepared.bounds,[0,0],
                              connected_to_origin=a.whole_map)
    if a.connected_recovery:
        from connected_recovery import ConnectedRecovery
        recovery=ConnectedRecovery(envelope,prepared.recovery_boxes)
        print('Recovery graph checked: '+str(int((recovery.parent>=0).sum()))+' flight cells',flush=True)
    kind=a.route_kind or ('east' if a.whole_map else 'short')
    route={'short':[[-3.,0.]],'east':[[27.,10.]],'west':[[-27.,0.]],
           'cross-map':[[27.,10.],[-27.,10.],[-25.,-10.]]}[kind]
    for xy in [[0,0],*route]:
        if not envelope.guide_contains(xy):raise ValueError('Route lacks shared endpoint reserve: '+str(xy))
    pre.update(meta,route_offsets_m=route,takeoff_height_m=1.5,alignment_height_m=1.6,
               planner_virtual_ceiling_m=2.1,column_obstacle_max_z_m=2.1,
               static_descent_columns=False,route_kind='independent transit/landing and planned return: '+kind,
               direct_path=line_audit(prepared,[0,0,1.74],[*route[0],1.74]),
               guard_sha256=sha(out/'mesh_guard.npz'),world_sha256=sha(out/'world.sdf'),
               flight_validated=False,validation_low_box=a.validation_low_box,
               known_geometry_only=True,terrain_following_implemented=False)
    if a.whole_map:
        pre.update(planner_map_size_xy_m=(80. if a.scene_edge_reserve_m<5. else 70.),column_obstacle_max_z_m=3.,
                   planner_leg_timeout_s=500.,
                   whole_map_scope=f'Connected fixed-height airspace inside the 70 m patch, excluding {a.scene_edge_reserve_m:g} m outer border; no terrain following',
                   selectable_area_m2=float(envelope.guide_free.sum()*envelope.resolution**2),
                   flight_area_m2=float(envelope.free.sum()*envelope.resolution**2))
    outdoor_envelope(pre,meta)
    (out/'flight_precheck.json').write_text(json.dumps(pre,indent=2)+'\n')
    (out/'route.json').write_text(json.dumps(route)+'\n')
    print(json.dumps(dict(output=str(out),transit_area_m2=cert['area_m2'],
        landing_area_m2=meta['recovery_certificate']['area_m2'],
        over_box_transit=prepared.inside_flight_region([-1.5,0]),
        over_box_landing=prepared.inside_recovery_region([-1.5,0]),
        route=route,flight_validated=False),indent=2))


if __name__=='__main__':main()
