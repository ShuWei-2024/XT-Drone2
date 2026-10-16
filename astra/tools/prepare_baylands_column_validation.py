#!/usr/bin/env python3
"""Add a cylinder to a copied certified Baylands scene, preserving the original.

Run with map_tools_venv. Recompute descent-region checks from collision geometry;
output route is a list of goals only. No flight commands or planner map injection.
"""
from pathlib import Path
import sys,json,shutil,hashlib,xml.etree.ElementTree as ET,time,argparse
import numpy as np
import trimesh
sys.path.insert(0,str(Path(__file__).resolve().parent))
from scan_mesh_guard import MeshGuard,load_stl
from outdoor_recovery import recovery_rectangles
from scan_trajectory_audit import line_audit
def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    source=args.source.resolve();out=args.output.resolve()
    manifest=json.loads((source/'manifest.json').read_text())
    if manifest.get('scene')!='baylands' or manifest.get('model_name')!='astra_baylands':
     raise ValueError('Expected prepared Baylands scene')
    for name,digest in manifest['asset_sha256'].items():
     if hashlib.sha256((source/'astra_baylands/meshes'/name).read_bytes()).hexdigest()!=digest:
      raise ValueError('Source asset hash mismatch: '+name)
    source_pre=json.loads((source/'flight_precheck.json').read_text())
    for name,key in [('world.sdf','world_sha256'),('astra_baylands/model.sdf','model_sdf_sha256'),('mesh_guard.npz','guard_sha256')]:
     if hashlib.sha256((source/name).read_bytes()).hexdigest()!=source_pre[key]:
      raise ValueError('Source flight precheck hash mismatch: '+name)
    out.mkdir(parents=True,exist_ok=False)
    shutil.copytree(source/'astra_baylands',out/'astra_baylands')
    for name in ['world.sdf','LICENSE.txt']:shutil.copy2(source/name,out/name)
    pre=source_pre.copy()
    mesh=out/'astra_baylands/meshes/collision.stl';old=load_stl(mesh)
    column=trimesh.creation.cylinder(radius=.7,height=2.2,sections=32);column.apply_translation([-3.5,0,1.1])
    triangles=np.concatenate([old,np.asarray(column.triangles)])
    combined=trimesh.Trimesh(vertices=triangles.reshape(-1,3),faces=np.arange(triangles.size//3).reshape(-1,3),process=False)
    combined.export(mesh)
    model=out/'astra_baylands/model.sdf';tree=ET.parse(model);link=tree.getroot().find('model/link')
    v=ET.SubElement(link,'visual',name='validation_column');ET.SubElement(v,'pose').text='-3.5 0 1.1 0 0 0'
    cyl=ET.SubElement(ET.SubElement(v,'geometry'),'cylinder');ET.SubElement(cyl,'radius').text='.7';ET.SubElement(cyl,'length').text='2.2'
    mat=ET.SubElement(v,'material');ET.SubElement(mat,'ambient').text='.1 .4 .1 1';ET.SubElement(mat,'diffuse').text='.15 .7 .2 1'
    tree.write(model,encoding='utf-8',xml_declaration=True)
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    manifest['test_props'].append(dict(name='green validation cylinder',centre=[-3.5,0,1.1],radius_m=.7,height_m=2.2))
    manifest['asset_sha256']['collision.stl']=sha(mesh);manifest['collision_faces']=len(triangles)
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    triangles=load_stl(mesh);keep=triangles[:,:,2].max(1)>.12
    meta={k:v for k,v in pre.items() if k in ['minimum_clearance_m','reference','allowed_xy_bounds_m','allowed_body_z_bounds_m','excluded_floor_max_z_m','missing_geometry_certified','interactive_goal_radius_m','scene_label','alignment_height_m']}
    meta.update(collision_sha256=sha(mesh),excluded_floor_triangles=int((~keep).sum()),retained_triangles=int(keep.sum()))
    print('Certifying new cylinder recovery area',flush=True);beg=time.monotonic()
    meta.update(recovery_rectangles(triangles[keep],meta['allowed_xy_bounds_m'],meta['allowed_body_z_bounds_m']))
    np.savez_compressed(out/'mesh_guard.npz',triangles=triangles[keep],metadata=json.dumps(meta))
    g=MeshGuard(out/'mesh_guard.npz');route=[[-7,0],[-7,3],[0,3],[0,0]]
    goal_clearances=[]
    for xy in route:
     if not g.inside_recovery_region(xy):raise ValueError('Endpoint outside descent region '+str(xy))
     goal_clearances.append(min(g.clearance([*xy,z]) for z in np.linspace(1.,1.29,30)))
    climb=min(g.clearance([0,0,z]) for z in np.linspace(.24,1.99,60))
    if min([climb,*goal_clearances])<.7:raise ValueError('Insufficient spawn/endpoint clearance')
    direct=line_audit(g,[0,0,1.14],[-7,0,1.14])
    if not direct['violates_clearance_margin']:raise ValueError('Cylinder does not block the selected route')
    pre.update(meta,route_offsets_m=route,route_kind='cylinder blocked detour',flight_validated=False,
               nominal_climb_minimum_m=climb,nominal_goal_minimum_m=min(goal_clearances),
               world_sha256=sha(out/'world.sdf'),model_sdf_sha256=sha(model),
               guard_sha256=sha(out/'mesh_guard.npz'),direct_path=direct)
    (out/'flight_precheck.json').write_text(json.dumps(pre,indent=2)+'\n');(out/'route.json').write_text(json.dumps(route)+'\n')
    with (out/'LICENSE.txt').open('a') as f:f.write('Validation variant: added green cylindrical test obstacle at [-3.5,0,1.1].\n')
    print(json.dumps(dict(preparation_s=time.monotonic()-beg,out=str(out),route=route,direct_path=pre['direct_path']),indent=2),flush=True)


if __name__ == "__main__":
    main()
