"""Extract static box/cylinder footprints for isolated SITL truth guards.

Horizontal distance is conservative and is measured from the vehicle reference
point to each surface. Gazebo truth is used for a safety stop, never a setpoint.
"""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def pose(element):
    value = [float(x) for x in element.findtext('pose', '0 0 0 0 0 0').split()]
    if len(value) != 6 or not np.isfinite(value).all() or any(abs(x) > 1e-9 for x in value[3:]):
        raise ValueError('This test supports finite, unrotated static collision geometry only')
    return np.array(value[:3])


def extract(world_path):
    world = ET.parse(world_path).getroot().find('world')
    obstacles = []
    for model in world.findall('model'):
        if model.findtext('static', 'false').lower() != 'true':
            continue
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                geometry = collision.find('geometry')
                if geometry.find('plane') is not None:
                    continue
                xyz = pose(model) + pose(link) + pose(collision)
                item = dict(name=model.attrib['name'], center=xyz.tolist())
                box = geometry.find('box')
                cylinder = geometry.find('cylinder')
                if box is not None:
                    size = [float(x) for x in box.findtext('size').split()]
                    if len(size) != 3 or not np.isfinite(size).all() or min(size) <= 0:
                        raise ValueError('Invalid box size')
                    item.update(kind='box', size=size)
                elif cylinder is not None:
                    radius = float(cylinder.findtext('radius'))
                    height = float(cylinder.findtext('length'))
                    if not np.isfinite([radius, height]).all() or min(radius, height) <= 0:
                        raise ValueError('Invalid cylinder size')
                    item.update(kind='cylinder', radius=radius, height=height)
                else:
                    raise ValueError('Unsupported collision geometry in ' + model.attrib['name'])
                obstacles.append(item)
    if not obstacles:
        raise ValueError('No static obstacles extracted')
    return obstacles


def clearances(xy, obstacles):
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    distances = []
    for item in obstacles:
        center = np.array(item['center'][:2])
        if item['kind'] == 'box':
            half = np.array(item['size'][:2]) / 2
            distances.append(np.linalg.norm(np.maximum(np.abs(xy - center) - half, 0), axis=1))
        elif item['kind'] == 'cylinder':
            distances.append(np.maximum(np.linalg.norm(xy - center, axis=1) - item['radius'], 0))
        else:
            raise ValueError('Unsupported obstacle kind')
    return np.min(distances, axis=0)


def load(path):
    return json.loads(Path(path).read_text())['obstacles']


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('world', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    obstacles = extract(args.world)
    args.output.write_text(json.dumps(dict(world=str(args.world), obstacles=obstacles), indent=2) + '\n')
    print('Static collision footprints:', len(obstacles))
