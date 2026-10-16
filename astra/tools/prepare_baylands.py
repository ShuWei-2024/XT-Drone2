#!/usr/bin/env python3
"""Prepare a local Baylands park patch with exact static geometry and bounded SITL goals.

Requires map_tools_venv with trimesh, pycollada, scipy and Pillow. The public
CC BY 4.0 asset is pinned to Fuel version 3. Assets stay under ignored runtime.
The added barrier is explicitly a test prop, not part of the source park.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import json
from pathlib import Path
import shutil
from urllib.request import Request, urlopen
import xml.etree.ElementTree as ET
import zipfile

import numpy as np
from scipy.spatial import cKDTree
import trimesh

from prepare_replica_office0 import write_dae
from scan_mesh_guard import MeshGuard
from scan_trajectory_audit import line_audit
from outdoor_recovery import recovery_rectangles

REPO=Path(__file__).resolve().parents[2]
URL='https://fuel.gazebosim.org/1.0/OpenRobotics/models/baylands/3/baylands.zip'
SHA='a6576c36823b61566b408a8cee2df97e9c926c3a004f1be6855680ace67f8f9b'
SIZE=150731432


def download(root):
    archive=root/'baylands_v3.zip'
    if archive.exists() and hashlib.sha256(archive.read_bytes()).hexdigest()==SHA:
        return archive
    parts=root/'parts';parts.mkdir(parents=True,exist_ok=True)
    step=1048576;count=(SIZE+step-1)//step
    def fetch(i):
        lo=i*step;hi=min(SIZE,lo+step)-1;p=parts/f'{i:04d}'
        if p.exists() and p.stat().st_size==hi-lo+1:return
        for attempt in range(3):
            try:
                with urlopen(Request(URL,headers={'Range':f'bytes={lo}-{hi}'}),timeout=35) as response:
                    if response.status!=206 or response.headers.get('Content-Range')!=f'bytes {lo}-{hi}/{SIZE}':
                        raise ValueError('Unexpected Fuel byte range')
                    data=response.read()
                if len(data)!=hi-lo+1:raise ValueError('Incomplete asset chunk')
                p.write_bytes(data);return
            except Exception:
                if attempt==2:raise
    with ThreadPoolExecutor(max_workers=16) as pool:
        for i,future in enumerate(as_completed([pool.submit(fetch,i) for i in range(count)]),1):
            future.result()
            if i%16==0:print(f'Downloaded {i}/{count} chunks',flush=True)
    with archive.open('wb') as stream:
        for i in range(count):stream.write((parts/f'{i:04d}').read_bytes())
    if hashlib.sha256(archive.read_bytes()).hexdigest()!=SHA:
        raise ValueError('Baylands archive differs from pinned version 3')
    return archive


def native_mesh(root):
    cache=root/'native_geometry.npz'
    if cache.exists():
        with np.load(cache,allow_pickle=False) as data:
            return trimesh.Trimesh(data['vertices'],data['faces'],process=False),data['colors'].copy()
    meshes=[];colors=[]
    for path in sorted((root/'unpacked/media').glob('*.DAE')):
        scene=trimesh.load(path,force='scene',process=False)
        for mesh in scene.dump():
            meshes.append(mesh)
            colors.append(mesh.visual.to_color().vertex_colors)
        print('Loaded',path.name,flush=True)
    mesh=trimesh.util.concatenate(meshes);color=np.concatenate(colors)
    np.savez_compressed(cache,vertices=mesh.vertices,faces=mesh.faces,colors=color)
    return mesh,color




def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,default=REPO/'astra/runtime/maps/baylands_park')
    p.add_argument('--native-spawn-xy',nargs=2,type=float,default=[-230.,-197.])
    p.add_argument('--floor-z',type=float,default=.9390990734100342)
    p.add_argument('--route-kind',choices=['blocked','open'],default='blocked',
                   help='Explicitly distinguish blocked-path avoidance from open-area traversal')
    a=p.parse_args();root=REPO/'astra/runtime/map_candidates/baylands';root.mkdir(parents=True,exist_ok=True)
    archive=download(root)
    with zipfile.ZipFile(archive) as z:
        if z.testzip() is not None:raise ValueError('Corrupt Baylands zip')
        z.extractall(root/'unpacked')
    out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    model=out/'astra_baylands';meshes=model/'meshes';meshes.mkdir(parents=True)
    mesh,colors=native_mesh(root)
    translation=np.array([-a.native_spawn_xy[0],-a.native_spawn_xy[1],-a.floor_z])
    mesh.apply_translation(translation)
    # Keep exact collision triangles of a 70 m park patch, with a 20+ m reserve
    # beyond the tested envelope; clip large terrain faces instead of selecting centroids.
    for axis in range(2):
        for sign in [-1,1]:
            normal=np.zeros(3);normal[axis]=sign
            point=np.zeros(3);point[axis]=-sign*35.
            mesh=mesh.slice_plane(point,normal)
    cache=np.load(root/'native_geometry.npz',allow_pickle=False)
    nearest=cKDTree(cache['vertices']+translation).query(mesh.vertices)[1]
    color=colors[nearest]
    # A deliberately visible solid obstacle creates a repeatable blocked route.
    barrier=trimesh.creation.box(extents=[.8,3.,2.2]);barrier.apply_translation([3.5,0.,1.1])
    parts=[mesh,barrier]
    color=np.vstack([color,np.tile([200,95,45,255],(len(barrier.vertices),1))])
    combined=trimesh.util.concatenate(parts)
    combined.export(meshes/'collision.stl')
    visual=combined.simplify_quadric_decimation(face_count=min(60000,len(combined.faces)))
    nearest=cKDTree(combined.vertices).query(visual.vertices)[1]
    write_dae(meshes/'baylands.dae',visual.vertices,visual.faces,color[nearest])
    (model/'model.config').write_text('<model><name>Baylands park test patch</name><version>1</version><sdf version="1.9">model.sdf</sdf></model>\n')
    (model/'model.sdf').write_text('''<sdf version="1.9"><model name="astra_baylands"><static>true</static><link name="park">
<collision name="collision"><geometry><mesh><uri>meshes/collision.stl</uri></mesh></geometry></collision>
<visual name="visual"><geometry><mesh><uri>meshes/baylands.dae</uri></mesh></geometry><material><ambient>.5 .5 .5 1</ambient><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>meshes/scan_color.png</albedo_map><metalness>0</metalness><roughness>1</roughness></metal></pbr></material></visual>
</link></model></sdf>\n''')
    tree=ET.parse(REPO/'astra/packages/astra_gz_sensors/worlds/astra_px4_geometry_rich.sdf');world=tree.getroot().find('world')
    for m in list(world.findall('model')):
        if m.get('name')!='ground_plane':world.remove(m)
    for visual_node in list(world.find("model[@name='ground_plane']/link").findall('visual')):
        world.find("model[@name='ground_plane']/link").remove(visual_node)
    include=ET.SubElement(world,'include');ET.SubElement(include,'uri').text='model://astra_baylands'
    ET.SubElement(include,'name').text='astra_baylands'
    # The inherited test world has no lights. Explicit PBR also prevents the
    # SDF colour override from replacing the COLLADA diffuse texture in Ogre2.
    scene=ET.SubElement(world,'scene')
    ET.SubElement(scene,'ambient').text='.4 .4 .4 1'
    ET.SubElement(scene,'background').text='.65 .75 .85 1'
    light=ET.SubElement(world,'light',name='astra_daylight',type='directional')
    ET.SubElement(light,'cast_shadows').text='false'
    ET.SubElement(light,'pose').text='0 0 10 0 0 0'
    ET.SubElement(light,'diffuse').text='.9 .9 .9 1'
    ET.SubElement(light,'specular').text='.1 .1 .1 1'
    attenuation=ET.SubElement(light,'attenuation')
    for name,value in [('range','1000'),('constant','1'),('linear','0'),('quadratic','0')]:
        ET.SubElement(attenuation,name).text=value
    ET.SubElement(light,'direction').text='-.4 .2 -1'
    tree.write(out/'world.sdf',encoding='utf-8',xml_declaration=True)
    (out/'LICENSE.txt').write_text('Baylands by Cole Biesemeyer / Open Robotics, Fuel model version 3.\n'
        'Source: '+URL+'\nLicense: Creative Commons Attribution 4.0 International\n'
        'https://creativecommons.org/licenses/by/4.0/\nModified: local crop, translation, visual mesh simplification and colour baking; added test barrier; flat support plane.\n')
    sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
    manifest=dict(scene='baylands',model_name='astra_baylands',source_url=URL,source_sha256=SHA,
        license='CC BY 4.0',native_spawn_xy=a.native_spawn_xy,floor_z=a.floor_z,
        appearance='Explicit Ogre2 PBR colour atlas and directional daylight; flight retest required',
        translation_m=translation.tolist(),visual_faces=len(visual.faces),collision_faces=len(combined.faces),
        source_kind='public authored outdoor park model; not FAST-LIVO2 reconstruction or a claimed real scan',
        patch_width_m=70.,test_props=[dict(name='orange test barrier',centre=[3.5,0,1.1],size=[.8,3,2.2])],
        asset_sha256={name:sha(meshes/name) for name in ['collision.stl','baylands.dae','scan_color.png']})
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    triangles=np.asarray(combined.triangles);keep=triangles[:,:,2].max(axis=1)>.12
    bounds=[[-11.,-11.],[11.,11.]];z_bounds=[.20,2.0]
    meta=dict(collision_sha256=sha(meshes/'collision.stl'),minimum_clearance_m=.55,
        reference='base_link at rotated model-to-body offset [0,0,.24]',allowed_xy_bounds_m=bounds,
        allowed_body_z_bounds_m=z_bounds,excluded_floor_max_z_m=.12,
        excluded_floor_triangles=int((~keep).sum()),retained_triangles=int(keep.sum()),
        missing_geometry_certified=False,interactive_goal_radius_m=10.,scene_label='Baylands 室外公园',alignment_height_m=1.60)
    meta.update(recovery_rectangles(triangles[keep],bounds,z_bounds))
    guard_path=out/'mesh_guard.npz';np.savez_compressed(guard_path,triangles=triangles[keep],metadata=json.dumps(meta))
    guard=MeshGuard(guard_path)
    route=([[7.,0.],[7.,4.],[0.,4.],[0.,0.]] if a.route_kind=='blocked'
           else [[-7.,0.],[-7.,4.],[0.,4.],[0.,0.]])
    for goal in route:
        if not guard.inside_recovery_region(goal):raise ValueError('Prepared goal outside park descent region: '+str(goal))
    climb=min(guard.clearance([0,0,z]) for z in np.linspace(.24,1.99,60))
    if climb<.7:raise ValueError('Insufficient spawn clearance')
    precheck=dict(meta,route_offsets_m=route,route_kind=a.route_kind,takeoff_height_m=.9,alignment_height_m=1.60,
        planner_virtual_ceiling_m=1.5,optimization_clearance_m=.2,static_obstacle_retention_sec=180.,
        static_descent_columns=True,nominal_climb_minimum_m=climb,
        direct_path=line_audit(guard,[0,0,1.14],[*route[0],1.14]),
        world_sha256=sha(out/'world.sdf'),model_sdf_sha256=sha(model/'model.sdf'),guard_sha256=sha(guard_path),
        truth_used_for_setpoints=False,planned_detour_supplied=False,flight_validated=False)
    (out/'flight_precheck.json').write_text(json.dumps(precheck,indent=2)+'\n')
    (out/'route.json').write_text(json.dumps(route)+'\n')
    (out/'open_route.json').write_text(json.dumps([[-7,0],[-7,4],[0,4],[0,0]])+'\n')
    print(json.dumps({k:v for k,v in precheck.items() if k!='recovery_xy_boxes_m'},indent=2),flush=True)


if __name__=='__main__':main()
