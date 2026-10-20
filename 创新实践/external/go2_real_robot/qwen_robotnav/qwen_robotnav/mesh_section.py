"""Offline horizontal sections of the static COLLADA assets used by Gazebo.

This diagnostic never publishes ROS messages or supplies runtime geometry.
Only Z-up, matrix-transformed triangle meshes are supported; other primitives
and transform encodings are rejected instead of silently producing a section.
"""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

import numpy as np


def mesh_section(path, height, *, scale=1., translation=(0., 0., 0.), node=None):
    values = (height, scale, *translation)
    if len(translation) != 3 or not all(math.isfinite(x) for x in values) or scale <= 0:
        raise ValueError('Require finite height, positive uniform scale and XYZ translation')
    root = ET.parse(Path(path)).getroot()
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    if root.findtext('c:asset/c:up_axis', namespaces=ns) != 'Z_UP':
        raise ValueError('Only Z_UP assets are supported')
    unit = root.find('c:asset/c:unit', ns)
    meter = float(unit.get('meter', '1')) if unit is not None else 1.
    if not math.isfinite(meter) or meter <= 0:
        raise ValueError('Invalid asset unit')
    scene_ref = root.find('c:scene/c:instance_visual_scene', ns)
    if scene_ref is None:
        raise ValueError('Missing active visual scene')
    scene = root.find("c:library_visual_scenes/c:visual_scene[@id='%s']" %
                      scene_ref.get('url', '').removeprefix('#'), ns)
    if scene is None:
        raise ValueError('Missing referenced visual scene')
    geometries = {g.get('id'): g for g in root.findall('c:library_geometries/c:geometry', ns)}
    triangles = []
    selected = [0]

    def visit(element, parent, included=False):
        included = included or node is None or node in (element.get('id'), element.get('name'))
        matrix = parent.copy()
        for child in element:
            kind = child.tag.rsplit('}', 1)[-1]
            if kind == 'matrix':
                transform = np.fromstring(child.text or '', sep=' ')
                if len(transform) != 16 or not np.all(np.isfinite(transform)):
                    raise ValueError('Invalid node matrix')
                matrix = matrix @ transform.reshape(4, 4)
            elif kind in ('translate', 'rotate', 'scale', 'lookat', 'skew', 'instance_node'):
                raise ValueError('Unsupported transform or node reference: ' + kind)
        if included:
            for instance in element.findall('c:instance_geometry', ns):
                selected[0] += 1
                mesh = geometries[instance.get('url', '').removeprefix('#')].find('c:mesh', ns)
                if mesh is None or any(mesh.find('c:' + k, ns) is not None for k in
                                       ('polylist', 'polygons', 'trifans', 'tristrips', 'lines', 'linestrips')):
                    raise ValueError('Only triangle meshes are supported')
                for primitive in mesh.findall('c:triangles', ns):
                    inputs = primitive.findall('c:input', ns)
                    vertex = next(i for i in inputs if i.get('semantic') == 'VERTEX')
                    vertices = mesh.find("c:vertices[@id='%s']" % vertex.get('source')[1:], ns)
                    position = next(i for i in vertices.findall('c:input', ns) if i.get('semantic') == 'POSITION')
                    source = mesh.find("c:source[@id='%s']" % position.get('source')[1:], ns)
                    accessor = source.find('c:technique_common/c:accessor', ns)
                    if int(accessor.get('stride', '1')) != 3 or int(accessor.get('offset', '0')) != 0:
                        raise ValueError('Require packed XYZ positions')
                    points = np.fromstring(source.find('c:float_array', ns).text, sep=' ').reshape(-1, 3)
                    points = (np.c_[points, np.ones(len(points))] @ matrix.T)[:, :3]
                    points = points * (meter * scale) + np.asarray(translation)
                    if not np.all(np.isfinite(points)):
                        raise ValueError('Nonfinite mesh positions')
                    stride = max(int(i.get('offset', '0')) for i in inputs) + 1
                    indices = np.fromstring(primitive.find('c:p', ns).text, sep=' ', dtype=int)
                    indices = indices.reshape(-1, stride)[:, int(vertex.get('offset', '0'))].reshape(-1, 3)
                    if np.any(indices < 0) or np.any(indices >= len(points)):
                        raise ValueError('Invalid position index')
                    triangles.extend(points[indices])
        for child in element.findall('c:node', ns):
            visit(child, matrix, included)

    for element in scene.findall('c:node', ns):
        visit(element, np.eye(4))
    if not selected[0] or not triangles:
        raise ValueError('No selected triangle geometry')
    segments = []
    for triangle in triangles:
        hits = []
        for a, b in zip(triangle, np.roll(triangle, -1, axis=0)):
            if a[2] == b[2] == height:
                segments.append([a[:2], b[:2]])
            elif a[2] != b[2] and min(a[2], b[2]) <= height <= max(a[2], b[2]):
                point = (a + (b - a) * (height - a[2]) / (b[2] - a[2]))[:2]
                if not any(np.linalg.norm(point - h) < 1e-9 for h in hits):
                    hits.append(point)
        if len(hits) == 2:
            segments.append(hits)
    return np.asarray(segments, dtype=float).reshape(-1, 2, 2)


def nearest_section(segments, point):
    point = np.asarray(point, dtype=float)
    if point.shape != (2,) or not np.all(np.isfinite(point)):
        raise ValueError('Require finite XY point')
    if not len(segments):
        return None
    a, b = segments[:, 0], segments[:, 1]
    delta = b - a
    lengths = np.sum(delta * delta, axis=1)
    u = np.divide(np.sum((point - a) * delta, axis=1), lengths,
                  out=np.zeros_like(lengths), where=lengths > 0)
    closest = a + np.clip(u, 0., 1.)[:, None] * delta
    distances = np.linalg.norm(closest - point, axis=1)
    i = int(np.argmin(distances))
    return dict(distance_m=float(distances[i]), closest_xy=closest[i].tolist(),
                segment=segments[i].tolist())
