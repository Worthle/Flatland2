#!/usr/bin/env python3
"""Generate the original, metre-scale pallet and cage meshes using only Python."""
import argparse
from collections import defaultdict
import math
from pathlib import Path
import xml.etree.ElementTree as ET


MATERIALS = {
    'wood0': (0.64, 0.43, 0.23),
    'wood1': (0.73, 0.53, 0.31),
    'wood2': (0.68, 0.48, 0.27),
    'wood3': (0.78, 0.58, 0.35),
    'wood4': (0.71, 0.50, 0.28),
    'grain': (0.49, 0.33, 0.18),
    'steel': (0.30, 0.36, 0.38),
    'wire': (0.43, 0.49, 0.50),
    'hardware': (0.61, 0.65, 0.65),
    'nail': (0.22, 0.24, 0.23),
    'plate': (0.68, 0.68, 0.59),
}


def sub(a, b):
    return tuple(x-y for x, y in zip(a, b))


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def unit(v):
    length = math.sqrt(sum(x*x for x in v))
    return tuple(x/length for x in v)


class Mesh:
    def __init__(self):
        self.vertices = []
        self.normals = []
        self.triangles = defaultdict(list)

    def face(self, vertices, material, center):
        normal = unit(cross(sub(vertices[1], vertices[0]), sub(vertices[2], vertices[0])))
        outward = sub(vertices[0], center)
        if sum(x*y for x, y in zip(normal, outward)) < 0:
            vertices = list(reversed(vertices))
            normal = tuple(-x for x in normal)
        start = len(self.vertices)
        self.vertices.extend(vertices)
        self.normals.extend([normal]*len(vertices))
        for i in range(1, len(vertices)-1):
            self.triangles[material].extend((start, start+i, start+i+1))

    def box(self, center, size, material, bevel=0):
        half = [value/2 for value in size]
        bevel = min(bevel, min(half)*0.8)

        def face(local):
            self.face([tuple(x+c for x, c in zip(v, center)) for v in local], material, center)

        for axis in range(3):
            others = [i for i in range(3) if i != axis]
            for sign in (-1, 1):
                vertices = []
                for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                    v = [0.0]*3
                    v[axis] = sign*half[axis]
                    v[others[0]] = a*(half[others[0]]-bevel)
                    v[others[1]] = b*(half[others[1]]-bevel)
                    vertices.append(v)
                face(vertices)
        if not bevel:
            return
        for axis in range(3):
            a, b = [i for i in range(3) if i != axis]
            for sa in (-1, 1):
                for sb in (-1, 1):
                    vertices = []
                    for end, edge in ((-1, 0), (1, 0), (1, 1), (-1, 1)):
                        v = [0.0]*3
                        v[axis] = end*(half[axis]-bevel)
                        v[a] = sa*(half[a]-(bevel if edge else 0))
                        v[b] = sb*(half[b]-(0 if edge else bevel))
                        vertices.append(v)
                    face(vertices)
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    face([tuple(s*(h-(0 if i == axis else bevel))
                                for i, (s, h) in enumerate(zip((sx, sy, sz), half)))
                          for axis in range(3)])

    def rod(self, start, end, radius, material, sides=6):
        axis = unit(sub(end, start))
        reference = (0, 0, 1) if abs(axis[2]) < .9 else (0, 1, 0)
        u = unit(cross(axis, reference))
        v = cross(axis, u)
        center = tuple((a+b)/2 for a, b in zip(start, end))
        rings = [[tuple(p[j]+radius*(u[j]*math.cos(i*2*math.pi/sides)
                                   +v[j]*math.sin(i*2*math.pi/sides)) for j in range(3))
                  for i in range(sides)] for p in (start, end)]
        self.face(rings[0], material, center)
        self.face(rings[1], material, center)
        for i in range(sides):
            j = (i+1) % sides
            self.face([rings[0][i], rings[0][j], rings[1][j], rings[1][i]], material, center)

    def grain(self, x, y, z, length, width, seed):
        for i in range(4):
            offset = (i-1.5)*width/5
            start = x-length*(.41-.023*((seed+i) % 4))
            end = x+length*(.26+.031*((seed*3+i) % 5))
            mid = (start+end)/2
            self.face([(start, y+offset, z), (mid, y+offset-.0008, z),
                       (end, y+offset+.001, z), (mid, y+offset+.0012, z)],
                      'grain', (x, y, z-1))

    def write(self, path):
        root = ET.Element('COLLADA', xmlns='http://www.collada.org/2005/11/COLLADASchema', version='1.4.1')
        asset = ET.SubElement(root, 'asset')
        ET.SubElement(ET.SubElement(asset, 'contributor'), 'author').text = 'Levent Soysal'
        ET.SubElement(asset, 'created').text = '2026-01-01T00:00:00Z'
        ET.SubElement(asset, 'modified').text = '2026-01-01T00:00:00Z'
        ET.SubElement(asset, 'unit', name='meter', meter='1')
        ET.SubElement(asset, 'up_axis').text = 'Z_UP'
        effects = ET.SubElement(root, 'library_effects')
        materials = ET.SubElement(root, 'library_materials')
        for name in self.triangles:
            effect = ET.SubElement(effects, 'effect', id=name+'-effect')
            technique = ET.SubElement(ET.SubElement(effect, 'profile_COMMON'), 'technique', sid='common')
            phong = ET.SubElement(technique, 'phong')
            for prop, rgba in [('ambient', tuple(c*.7 for c in MATERIALS[name])+(1,)),
                               ('diffuse', MATERIALS[name]+(1,)),
                               ('specular', (.12, .12, .12, 1) if name in ('steel', 'wire', 'hardware') else (.015, .015, .015, 1))]:
                ET.SubElement(ET.SubElement(phong, prop), 'color').text = ' '.join(map(str, rgba))
            ET.SubElement(ET.SubElement(phong, 'shininess'), 'float').text = '24'
            material = ET.SubElement(materials, 'material', id=name, name=name)
            ET.SubElement(material, 'instance_effect', url='#'+name+'-effect')
        geometry = ET.SubElement(ET.SubElement(root, 'library_geometries'), 'geometry', id='object')
        mesh = ET.SubElement(geometry, 'mesh')
        for name, values in [('positions', self.vertices), ('normals', self.normals)]:
            source = ET.SubElement(mesh, 'source', id=name)
            array = ET.SubElement(source, 'float_array', id=name+'-array', count=str(len(values)*3))
            array.text = ' '.join(f'{v:.6f}' for point in values for v in point)
            accessor = ET.SubElement(ET.SubElement(source, 'technique_common'), 'accessor',
                                     source='#'+name+'-array', count=str(len(values)), stride='3')
            for coord in 'XYZ':
                ET.SubElement(accessor, 'param', name=coord, type='float')
        vertices = ET.SubElement(mesh, 'vertices', id='vertices')
        ET.SubElement(vertices, 'input', semantic='POSITION', source='#positions')
        for material, indices in self.triangles.items():
            triangles = ET.SubElement(mesh, 'triangles', material=material, count=str(len(indices)//3))
            ET.SubElement(triangles, 'input', semantic='VERTEX', source='#vertices', offset='0')
            ET.SubElement(triangles, 'input', semantic='NORMAL', source='#normals', offset='0')
            ET.SubElement(triangles, 'p').text = ' '.join(map(str, indices))
        scene = ET.SubElement(ET.SubElement(root, 'library_visual_scenes'), 'visual_scene', id='scene')
        instance = ET.SubElement(ET.SubElement(scene, 'node', id='model'), 'instance_geometry', url='#object')
        binding = ET.SubElement(ET.SubElement(instance, 'bind_material'), 'technique_common')
        for name in self.triangles:
            ET.SubElement(binding, 'instance_material', symbol=name, target='#'+name)
        ET.SubElement(ET.SubElement(root, 'scene'), 'instance_visual_scene', url='#scene')
        ET.indent(root)
        path.write_bytes(ET.tostring(root, encoding='utf-8', xml_declaration=True)+b'\n')
        print(f'{path.name}: {sum(len(v)//3 for v in self.triangles.values())} triangles, {path.stat().st_size//1024} KiB')


def pallet():
    mesh = Mesh()
    # Eleven boards and nine blocks; the complete stack is 144 mm high.
    for i, (y, width) in enumerate([(-.35, .1), (0, .145), (.35, .1)]):
        mesh.box((0, y, .011), (1.2, width, .022), 'wood'+str(i), .003)
        for j, x in enumerate([-.5275, 0, .5275]):
            mesh.box((x, y, .061), (.145, width, .078), 'wood'+str((i+j+2) % 5), .003)
    for i, x in enumerate([-.5275, 0, .5275]):
        mesh.box((x, 0, .111), (.145, .8, .022), 'wood'+str(i+1), .0015)
    for i, (y, width) in enumerate([(-.3275, .145), (-.16375, .1), (0, .145), (.16375, .1), (.3275, .145)]):
        mesh.box((0, y, .133), (1.2, width, .022), 'wood'+str(i), .0015)
        mesh.grain(0, y, .14401, 1.2, width, i)
        for x in [-.5275, 0, .5275]:
            for dy in [-width*.27, width*.27]:
                mesh.rod((x-.017, y+dy, .1437), (x-.017, y+dy, .1441), .0028, 'nail', 8)
    # Long-side entry keeps the stock forklift tines between the support blocks.
    mesh.vertices = [(-y, x, z) for x, y, z in mesh.vertices]
    mesh.normals = [(-y, x, z) for x, y, z in mesh.normals]
    return mesh


def cage():
    mesh = Mesh()
    # Generic Euro-size steel box pallet, with a wooden floor and two front gates.
    for x in [-.38, .38]:
        for y in [-.5825, .5825]:
            mesh.box((x, y, .008), (.075, .075, .016), 'steel', .003)
            mesh.box((x, y, .071), (.055, .055, .11), 'steel', .003)
            mesh.box((x, y, .545), (.036, .036, .85), 'steel', .002)
    for x in [-.386, .386]:
        mesh.box((x, 0, .138), (.04, 1.2, .045), 'steel', .002)
        mesh.box((x, 0, .95), (.04, 1.2, .04), 'steel', .002)
    for y in [-.5875, .5875]:
        mesh.box((0, y, .138), (.8, .04, .045), 'steel', .002)
        mesh.box((0, y, .95), (.8, .04, .04), 'steel', .002)
    for x in [-.20, .20]:
        mesh.box((x, 0, .125), (.04, 1.12, .035), 'steel')
    for i in range(4):
        y = (i-1.5)*.286
        mesh.box((0, y, .172), (.746, .28, .025), 'wood'+str(i), .002)
        mesh.grain(0, y, .18451, .746, .28, i+5)
        for x in [-.30, .30]:
            mesh.rod((x, y, .184), (x, y, .1846), .003, 'nail', 8)
    # Rear lattice and side panels. Six-sided wires stay inexpensive to render.
    for i in range(23):
        y = (i-11)*.05
        mesh.rod((-.383, y, .19), (-.383, y, .93), .0028, 'wire')
    for i in range(14):
        z = .235+i*.05
        mesh.rod((-.389, -.565, z), (-.389, .565, z), .0028, 'wire')
    for y in [-.585, .585]:
        for i in range(15):
            x = (i-7)*.05
            mesh.rod((x, y, .19), (x, y, .93), .0028, 'wire')
        for i in range(14):
            z = .235+i*.05
            mesh.rod((-.365, y, z), (.365, y, z), .0028, 'wire')
    # Closed gate halves, with hinge barrels, latch handles and a blank label plate.
    for bottom, top in [(.195, .555), (.578, .93)]:
        for z in [bottom, top]:
            mesh.box((.388, 0, z), (.024, 1.12, .024), 'steel', .001)
        for y in [-.55, .55]:
            mesh.box((.388, y, (bottom+top)/2), (.024, .024, top-bottom), 'steel', .001)
            mesh.rod((.405, y-.025, bottom), (.405, y+.025, bottom), .009, 'hardware', 8)
            mesh.box((.406, y, top-.045), (.023, .047, .065), 'steel', .002)
            mesh.rod((.416, y-.015, top-.025), (.416, y+.028, top-.025), .003, 'hardware')
        for i in range(21):
            y = (i-10)*.05
            mesh.rod((.386, y, bottom+.012), (.386, y, top-.012), .0028, 'wire')
        for i in range(1, 7):
            z = bottom+i*.05
            mesh.rod((.392, -.54, z), (.392, .54, z), .0028, 'wire')
    mesh.box((.399, -.32, .805), (.004, .24, .12), 'plate', .001)
    for y in [-.42, -.22]:
        for z in [.765, .845]:
            mesh.rod((.4, y, z), (.402, y, z), .003, 'hardware', 8)
    return mesh


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[1]/'objects/meshes')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    pallet().write(args.output/'euro_pallet.dae')
    cage().write(args.output/'cage.dae')


if __name__ == '__main__':
    main()
