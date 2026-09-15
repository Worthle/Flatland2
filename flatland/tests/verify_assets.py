#!/usr/bin/env python3
"""Validate release assets, map alignment, provenance and plugin references."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import yaml


def verify(share, plugin_xml):
    robots = {p.stem.removesuffix('.model') for p in (share/'robot').glob('*.yaml')}
    assert robots == {'turtlebot', 'castor_wheeled_differential_robot', 'omni_drive_robot', 'forklift'}
    plugins = {c.get('type').split('::')[-1] for c in ET.parse(plugin_xml).getroot().findall('class')}
    for folder in ['robot', 'objects']:
        for path in (share/folder).glob('*.yaml'):
            model = yaml.safe_load(path.read_text())
            bodies = {b['name'] for b in model['bodies']}
            assert len(bodies) == len(model['bodies']), path
            for joint in model.get('joints', []):
                assert all(body['name'] in bodies for body in joint['bodies']), path
            for plugin in model.get('plugins', []):
                assert plugin['type'] in plugins, (path, plugin)
                assert plugin.get('body', next(iter(bodies))) in bodies, path
    folder = share/'maps/warehouse'
    provenance = json.loads((folder/'provenance.json').read_text())
    for name, digest in provenance['sha256'].items():
        assert hashlib.sha256((folder/name).read_bytes()).hexdigest() == digest, name
    for name, digest in provenance['source_sha256'].items():
        assert hashlib.sha256((folder/'source'/name).read_bytes()).hexdigest() == digest, name
    raw = (folder/'warehouse.pcd').read_bytes()
    header, data = raw.split(b'DATA binary\n', 1)
    cloud = np.frombuffer(data, dtype='<f4').reshape(-1, 3)
    assert len(cloud) == provenance['points'] and np.isfinite(cloud).all()
    assert cloud[:, 2].max() > 8 and cloud[:, 2].min() < .1
    metadata = yaml.safe_load((folder/'warehouse.yaml').read_text())
    image = cv2.imread(str(folder/metadata['image']), cv2.IMREAD_GRAYSCALE)
    assert image.shape == (440, 300)
    assert set(np.unique(image)) == {0, 254}
    low = np.array(metadata['origin'][:2])
    high = low+np.array(image.shape[::-1])*metadata['resolution']
    assert np.all(cloud[:, :2].min(axis=0) >= low)
    assert np.all(cloud[:, :2].max(axis=0) <= high)
    # Surface samples in the occupancy height band must project onto nearby
    # occupied pixels, allowing the rasterizer's one-cell boundary rounding.
    band = cloud[(cloud[:, 2] >= .12) & (cloud[:, 2] <= 1.8)]
    pixels = np.floor((band[:, :2]-low)/metadata['resolution']).astype(int)
    pixels[:, 1] = image.shape[0]-1-pixels[:, 1]
    occupied = cv2.dilate((image == 0).astype(np.uint8), np.ones((3, 3), np.uint8))
    assert occupied[pixels[:, 1], pixels[:, 0]].mean() > .999
    # The default spawn and its immediate footprint must be in the aisle.
    x, y = np.floor((np.array([0., -2.])-low)/metadata['resolution']).astype(int)
    row = image.shape[0]-1-y
    assert np.all(image[row-12:row+12, x-28:x+28] == 254)
    print(f'PASS: four robots, plugin references, PCD/PGM projection, spawn clearance and {len(cloud):,} map points')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--share', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--plugin-xml', type=Path)
    args = parser.parse_args()
    if args.plugin_xml is None:
        from ament_index_python.packages import get_package_share_directory
        args.plugin_xml = Path(get_package_share_directory('flatland_plugins'))/'flatland_plugins.xml'
    verify(args.share, args.plugin_xml)
