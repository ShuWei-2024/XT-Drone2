#!/usr/bin/env python3
"""Convert a pinned, culled Replica scan into a passive Gazebo world.

Run with the isolated map-tools venv (trimesh, fast-simplification, scipy, Pillow).
Generated dataset assets stay under ignored astra/runtime, never in source Git.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
from scipy.spatial import cKDTree
import trimesh

REPO = Path(__file__).resolve().parents[2]
SHA256 = '0dd4b4d292cc325b52eeb596b0d089440cc333df8757cced7fa12daedb9309a6'
SOURCE_URL = ('https://huggingface.co/datasets/voviktyl/Replica-SLAM/resolve/'
              '7d7ae90b48035aa2e9baa58018f5e090bd6d5dc5/cull_replica_mesh/office0.ply')

SCENE_HASHES = {
    'office0': SHA256,
    'office1': '3e62df3ff178b2362ff6cca56f52853a7a2ba194d13520969882bb708a5f4729',
    'office2': 'dd8e3f6c4f16857641b91b25cb97394fd35f80f524066bbe540b46ed292d8f43',
    'office3': 'eba681ca5518674de40996ab7a28383e57e52852ea0b6e2edf92c42d613f4a16',
    'office4': 'dd2bbf6771825b2b3984408d32ca7f448f954420560f1c0d24c74a5e5b59d393',
    'room0': '4b1c97fd0513aab840ebb7588ee0a3a1a82913c79a4926ff4e03048417a566de',
    'room2': '61d29d90b4668e8f4af9cfb2ecc24d9ab84a477031240cc9c9dd0cd3fe23946f',
}


def write_dae(path, vertices, faces, colors):
    """Bake vertex RGB into a triangle atlas; portable COLLADA with standard UVs."""
    count = len(faces)
    tile, columns = 8, 256
    rows = (count + columns - 1) // columns
    width, height = columns * tile, rows * tile
    texture = np.zeros((height, width, 3), dtype=np.uint8)
    fc = colors[faces, :3].astype(float)
    # Texel centres and triangle UVs use the same local coordinate convention.
    for py in range(tile):
        for px in range(tile):
            b = np.clip((px + .5 - 1) / (tile - 2), 0, 1)
            c = np.clip((py + .5 - 1) / (tile - 2), 0, 1)
            b, c = b / max(1., b+c), c / max(1., b+c)
            texture[(np.arange(count)//columns)*tile+py,
                    (np.arange(count)%columns)*tile+px] = (
                        fc[:, 0]*(1-b-c) + fc[:, 1]*b + fc[:, 2]*c).astype(np.uint8)
    Image.fromarray(texture).save(path.parent/'scan_color.png')
    origins = np.column_stack(((np.arange(count) % columns)*tile,
                               (np.arange(count) // columns)*tile))
    uv = (origins[:, None, :] + np.array([[1, 1], [tile-1, 1], [1, tile-1]]))
    uv = uv.reshape(-1, 2) / [width, height]
    uv[:, 1] = 1 - uv[:, 1]
    ns = 'http://www.collada.org/2005/11/COLLADASchema'
    ET.register_namespace('', ns)
    def node(parent, tag, text=None, **attrs):
        x = ET.SubElement(parent, '{'+ns+'}'+tag, attrs)
        if text is not None: x.text = str(text)
        return x
    root = ET.Element('{'+ns+'}COLLADA', version='1.4.1')
    asset = node(root, 'asset')
    node(asset, 'unit', name='meter', meter='1')
    node(asset, 'up_axis', 'Z_UP')
    images = node(root, 'library_images')
    node(node(images, 'image', id='scan-image'), 'init_from', 'scan_color.png')
    effect = node(node(root, 'library_effects'), 'effect', id='scan-effect')
    profile = node(effect, 'profile_COMMON')
    surface = node(node(profile, 'newparam', sid='scan-surface'), 'surface', type='2D')
    node(surface, 'init_from', 'scan-image')
    node(node(node(profile, 'newparam', sid='scan-sampler'), 'sampler2D'),
         'source', 'scan-surface')
    lambert = node(node(profile, 'technique', sid='common'), 'lambert')
    node(node(lambert, 'diffuse'), 'texture', texture='scan-sampler', texcoord='UVSET0')
    material = node(node(root, 'library_materials'), 'material', id='scan-material')
    node(material, 'instance_effect', url='#scan-effect')
    mesh = node(node(node(root, 'library_geometries'), 'geometry', id='scan-geometry'), 'mesh')
    normals = np.asarray(trimesh.Trimesh(vertices, faces, process=False).vertex_normals)
    for name, data, params in [('positions', vertices, ['X','Y','Z']),
                                ('normals', normals, ['X','Y','Z']),
                                ('uv', uv, ['S','T'])]:
        source = node(mesh, 'source', id=name)
        node(source, 'float_array', ' '.join('%.7g'%v for v in data.ravel()),
             id=name+'-array', count=str(data.size))
        accessor = node(node(source, 'technique_common'), 'accessor',
                        source='#'+name+'-array', count=str(len(data)), stride=str(data.shape[1]))
        for param in params: node(accessor, 'param', name=param, type='float')
    node(node(mesh, 'vertices', id='vertices'), 'input', semantic='POSITION', source='#positions')
    tris = node(mesh, 'triangles', count=str(count), material='scan-material-symbol')
    node(tris, 'input', semantic='VERTEX', source='#vertices', offset='0')
    node(tris, 'input', semantic='NORMAL', source='#normals', offset='2')
    node(tris, 'input', semantic='TEXCOORD', source='#uv', offset='1', set='0')
    indices = np.column_stack((faces.ravel(), np.arange(count*3), faces.ravel()))
    node(tris, 'p', ' '.join(map(str, indices.ravel())))
    scene = node(node(root, 'library_visual_scenes'), 'visual_scene', id='scene')
    geom = node(node(scene, 'node', id='office0'), 'instance_geometry', url='#scan-geometry')
    im = node(node(node(geom, 'bind_material'), 'technique_common'), 'instance_material',
              symbol='scan-material-symbol', target='#scan-material')
    node(im, 'bind_vertex_input', semantic='UVSET0', input_semantic='TEXCOORD', input_set='0')
    node(node(root, 'scene'), 'instance_visual_scene', url='#scene')
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)
    return [width, height]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', choices=sorted(SCENE_HASHES), default='office0')
    parser.add_argument('--source', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--spawn-xy', type=float, nargs=2, help='Inspected native XY spawn; no scale/rotation change')
    parser.add_argument('--faces', type=int, default=100000)
    a = parser.parse_args()
    model_name='replica_'+a.scene
    a.source=a.source or REPO/f'astra/runtime/map_candidates/{model_name}/{a.scene}.ply'
    a.output=a.output or REPO/f'astra/runtime/maps/{model_name}'
    source_hash=SCENE_HASHES[a.scene]
    source_url=SOURCE_URL.rsplit('/',1)[0]+'/'+a.scene+'.ply'
    if a.spawn_xy is not None and not np.isfinite(a.spawn_xy).all():parser.error('Non-finite spawn')
    if not 20000 <= a.faces <= 200000: parser.error('Use 20000..200000 visual faces')
    if hashlib.sha256(a.source.read_bytes()).hexdigest() != source_hash:
        parser.error('Source differs from the pinned Replica-SLAM mesh')
    a.output.mkdir(parents=True, exist_ok=False)
    original = trimesh.load(a.source, process=False)
    v, f = np.asarray(original.vertices), np.asarray(original.faces)
    # Horizontal floor faces establish the native scan's Z-up plane. Ground
    # support is supplied separately because the scanned floor has holes/noise.
    tv = v[f]
    cross = np.cross(tv[:, 1]-tv[:, 0], tv[:, 2]-tv[:, 0])
    area2 = np.linalg.norm(cross, axis=1)
    centers = tv.mean(axis=1)
    horizontal = np.abs(cross[:, 2]) > .98*area2
    if a.scene=='office0':
        floor_mask = (centers[:, 2] < -1.05) & horizontal
    else:
        bins=np.arange(v[:,2].min(),v[:,2].max()+.04,.04)
        hist,edges=np.histogram(centers[horizontal,2],bins=bins,weights=area2[horizontal])
        eligible=np.flatnonzero(edges[:-1]<v[:,2].min()+.3)
        floor_band=float(edges[eligible[np.argmax(hist[eligible])]])
        floor_mask=horizontal & (centers[:,2]>=floor_band-.06) & (centers[:,2]<=floor_band+.08)
    if floor_mask.sum() < 1000: raise ValueError('Expected horizontal floor absent')
    floor = float(np.median(centers[floor_mask, 2]))
    # Select a spawn over observed floor, away from all sampled above-floor
    # surfaces. This is an initial inspection estimate, not a flight clearance
    # certificate; missing scan geometry cannot certify free space.
    band = (v[:, 2] > floor+.15) & (v[:, 2] < floor+2.1)
    tc = (centers[:, 2] > floor+.15) & (centers[:, 2] < floor+2.1)
    obstacle_xy = np.vstack((v[band, :2], centers[tc, :2]))
    candidates = centers[floor_mask, :2][::10]
    distance, _ = cKDTree(obstacle_xy).query(candidates)
    spawn = candidates[np.argmax(distance)] if a.spawn_xy is None else np.asarray(a.spawn_xy)
    spawn_clearance=float(cKDTree(obstacle_xy).query(spawn)[0])
    # Put the observed floor just below an invisible, flat ground support.
    translation = np.array([-spawn[0], -spawn[1], -floor-.03])
    reduced = original.simplify_quadric_decimation(face_count=a.faces)
    nearest = cKDTree(v).query(reduced.vertices)[1]
    colors = np.asarray(original.visual.vertex_colors)[nearest]
    vertices = np.asarray(reduced.vertices) + translation
    model = a.output/model_name
    meshes = model/'meshes'; meshes.mkdir(parents=True)
    atlas = write_dae(meshes/(a.scene+'.dae'), vertices, reduced.faces, colors)
    # Static concave collision triangles share the converted geometry. Do not
    # convexify: a room convex hull would fill the entire flight volume.
    trimesh.Trimesh(vertices, reduced.faces, process=False).export(meshes/'collision.stl')
    (model/'model.config').write_text(f'<model><name>Replica {a.scene}</name><version>1</version>'
        '<sdf version="1.9">model.sdf</sdf><description>Culled real scan; research use; '
        'passive integration, flight unvalidated.</description></model>\n')
    (model/'model.sdf').write_text(f'''<?xml version="1.0"?>
<sdf version="1.9"><model name="{model_name}"><static>true</static><link name="scan">
<collision name="scan_collision"><geometry><mesh><uri>meshes/collision.stl</uri></mesh></geometry></collision>
<visual name="scan_visual"><geometry><mesh><uri>meshes/{a.scene}.dae</uri></mesh></geometry>
<material><ambient>.3 .3 .3 1</ambient><diffuse>1 1 1 1</diffuse>
<pbr><metal><albedo_map>meshes/scan_color.png</albedo_map><metalness>0</metalness><roughness>1</roughness></metal></pbr>
</material></visual>
</link></model></sdf>
''')
    xml = ET.parse(REPO/'astra/packages/astra_gz_sensors/worlds/astra_px4_geometry_rich.sdf')
    world = xml.getroot().find('world')
    for m in list(world.findall('model')):
        if m.get('name') != 'ground_plane': world.remove(m)
    # Keep floor collision for physical support; let the scan provide RGB floor.
    ground = world.find("model[@name='ground_plane']/link")
    for visual in list(ground.findall('visual')): ground.remove(visual)
    inc = ET.SubElement(world, 'include')
    ET.SubElement(inc, 'uri').text = 'model://'+model_name
    ET.SubElement(inc, 'name').text = model_name
    light = ET.SubElement(world, 'light', name='office_fill', type='directional')
    ET.SubElement(light, 'cast_shadows').text = 'false'
    ET.SubElement(light, 'diffuse').text = '.65 .65 .65 1'
    ET.SubElement(light, 'direction').text = '.2 .1 -1'
    scene = ET.SubElement(world, 'scene')
    ET.SubElement(scene, 'ambient').text = '.3 .3 .3 1'
    xml.write(a.output/'world.sdf', encoding='utf-8', xml_declaration=True)
    shutil.copy2(a.source.parent/'REPLICA_LICENSE.txt', a.output/'REPLICA_LICENSE.txt')
    manifest = dict(scene=a.scene,model_name=model_name,source_url=source_url, source_sha256=source_hash,
        source_faces=len(f), visual_and_collision_faces=len(reduced.faces),
        source_vertices=len(v), converted_vertices=len(vertices),
        native_floor_z=floor, native_spawn_xy=spawn.tolist(),
        translation_native_to_world_m=translation.tolist(), rotation='identity; native Z up',
        scale=1., floor_normal_tolerance_cos=.98, horizontal_floor_faces=int(floor_mask.sum()),
        sampled_spawn_xy_clearance_m=spawn_clearance, clearance_certified=False,
        transformed_bounds=[vertices.min(axis=0).tolist(), vertices.max(axis=0).tolist()],
        color_method='nearest source vertex RGB, baked interpolated triangle atlas',
        atlas_pixels=atlas, source_is_culled=True, synthetic_ground_collision_z=0.,
        synthetic_ground_visual=False, px4_armed=False, flight_validated=False,
        explicit_vertex_normals=True, dependencies={'trimesh':trimesh.__version__})
    for asset in [a.scene+'.dae','collision.stl','scan_color.png']:
        manifest.setdefault('asset_sha256', {})[asset] = hashlib.sha256((meshes/asset).read_bytes()).hexdigest()
    (a.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__': main()
