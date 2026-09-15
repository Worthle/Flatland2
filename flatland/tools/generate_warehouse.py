#!/usr/bin/env python3
"""Derive matching PCD and occupancy maps from the vendored AWS collision scene."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import collada
import cv2
import numpy as np
import yaml


def pose(text):
    x, y, z, roll, pitch, yaw = map(float, (text or '0 0 0 0 0 0').split())
    cr, cp, cy = np.cos([roll, pitch, yaw])
    sr, sp, sy = np.sin([roll, pitch, yaw])
    matrix = np.eye(4)
    matrix[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                       [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                       [-sp, cp*sr, cp*cr]]
    matrix[:3, 3] = [x, y, z]
    return matrix


def mesh_triangles(path):
    mesh = collada.Collada(str(path), ignore=[collada.DaeBrokenRefError])
    if mesh.assetInfo.upaxis != 'Z_UP':
        raise ValueError(f'Expected Z_UP mesh: {path}')
    result = []
    for geometry in mesh.scene.objects('geometry'):
        for primitive in geometry.primitives():
            if hasattr(primitive, 'triangleset'):
                primitive = primitive.triangleset()
            result.append(primitive.vertex[primitive.vertex_index] * mesh.assetInfo.unitmeter)
    return np.concatenate(result)


def scene_triangles(source):
    world = ET.parse(source/'worlds/no_roof_small_warehouse.world').getroot()
    result, records = [], []
    for instance in world.findall('./world/model'):
        uri = instance.findtext('include/uri')
        if not uri:
            raise ValueError(f'Unsupported world model: {instance.attrib}')
        model_dir = source/'models'/uri.removeprefix('model://')
        model = ET.parse(model_dir/'model.sdf').getroot().find('model')
        transform = pose(instance.findtext('pose')) @ pose(model.findtext('pose'))
        count = 0
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                mesh = collision.find('geometry/mesh')
                if mesh is None:
                    raise ValueError('This dataset converter expects mesh collisions')
                path = source/'models'/mesh.findtext('uri').removeprefix('model://')
                triangles = mesh_triangles(path)
                triangles *= np.array(list(map(float, mesh.findtext('scale', '1 1 1').split())))
                matrix = transform @ pose(link.findtext('pose')) @ pose(collision.findtext('pose'))
                triangles = triangles @ matrix[:3, :3].T + matrix[:3, 3]
                result.append(triangles)
                count += len(triangles)
        records.append({'name': instance.get('name'), 'triangles': count})
    return np.concatenate(result), records


def clip_height(polygon, height, above):
    output = []
    for start, end in zip(polygon, np.roll(polygon, -1, axis=0)):
        a = start[2] >= height if above else start[2] <= height
        b = end[2] >= height if above else end[2] <= height
        if a:
            output.append(start)
        if a != b:
            output.append(start + (end-start) * ((height-start[2])/(end[2]-start[2])))
    return np.asarray(output)


def generate(source, output, resolution=0.05, spacing=0.05):
    triangles, instances = scene_triangles(source)
    output.mkdir(parents=True, exist_ok=True)
    low = np.floor(triangles.min(axis=(0, 1))[:2]/resolution)*resolution - 0.5
    high = np.ceil(triangles.max(axis=(0, 1))[:2]/resolution)*resolution + 0.5
    width, height = np.ceil((high-low)/resolution).astype(int)
    occupancy = np.full((height, width), 254, dtype=np.uint8)
    samples = []
    for triangle in triangles:
        # Barycentric lattice, including edges, limits surface sampling gaps.
        steps = max(1, int(np.ceil(max(np.linalg.norm(triangle[i]-triangle[j])
                                     for i, j in [(0, 1), (1, 2), (2, 0)])/spacing)))
        for i in range(steps+1):
            a = i/steps
            b = np.arange(steps-i+1)/steps
            samples.append(triangle[0] + a*(triangle[1]-triangle[0])
                           + b[:, None]*(triangle[2]-triangle[0]))
        clipped = clip_height(triangle, 0.12, True)
        if len(clipped) >= 3:
            clipped = clip_height(clipped, 1.8, False)
        if len(clipped) >= 3:
            pixels = np.floor((clipped[:, :2]-low)/resolution).astype(np.int32)
            pixels[:, 1] = height-1-pixels[:, 1]
            cv2.fillPoly(occupancy, [pixels], 0)
    points = np.concatenate(samples)
    # Retain original surface points; quantization is only for deduplication.
    _, indices = np.unique(np.floor(points/(spacing/2)).astype(np.int64), axis=0, return_index=True)
    points = points[np.sort(indices)].astype('<f4')
    pcd_header = ('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n'
                  'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                  f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
                  f'POINTS {len(points)}\nDATA binary\n')
    (output/'warehouse.pcd').write_bytes(pcd_header.encode()+points.tobytes())
    cv2.imwrite(str(output/'warehouse.pgm'), occupancy)
    metadata = dict(image='warehouse.pgm', mode='trinary', resolution=resolution,
                    origin=[float(low[0]), float(low[1]), 0.0], negate=0,
                    occupied_thresh=0.65, free_thresh=0.196)
    (output/'warehouse.yaml').write_text(yaml.safe_dump(metadata, sort_keys=False))
    manifest = dict(repository='https://github.com/aws-robotics/aws-robomaker-small-warehouse-world',
                    revision='3c23a698bf0b4e366ddf8b084af507c519bd3483',
                    world='worlds/no_roof_small_warehouse.world', geometry='collision meshes',
                    coordinate_frame='AWS world, metres, Z up; no recentering',
                    occupancy_height_band_m=[0.12, 1.8], resolution_m=resolution,
                    sample_spacing_m=spacing, points=len(points),
                    bounds_m=[points.min(axis=0).tolist(), points.max(axis=0).tolist()],
                    instances=instances)
    manifest['sha256'] = {name: hashlib.sha256((output/name).read_bytes()).hexdigest()
                          for name in ['warehouse.pcd', 'warehouse.pgm', 'warehouse.yaml']}
    manifest['source_sha256'] = {
        str(path.relative_to(source)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(source.rglob('*')) if path.is_file()}
    (output/'provenance.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps({k: manifest[k] for k in ['points', 'bounds_m', 'sha256']}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    generate(args.source, args.output)
