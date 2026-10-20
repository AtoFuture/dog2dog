import numpy as np
import pytest

from qwen_robotnav.mesh_section import mesh_section, nearest_section


def asset(tmp_path, *, extra='', matrix='1 0 0 100 0 1 0 0 0 0 1 0 0 0 0 1'):
    path = tmp_path / 'fixture.dae'
    path.write_text('''<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema">
      <asset><unit meter="0.01"/><up_axis>Z_UP</up_axis></asset>
      <library_geometries><geometry id="mesh"><mesh>
        <source id="positions"><float_array>0 0 0 200 0 0 0 0 200</float_array>
          <technique_common><accessor stride="3"/></technique_common></source>
        <vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices>
        <triangles count="1"><input semantic="VERTEX" source="#vertices" offset="0"/>
          <input semantic="NORMAL" source="#normals" offset="1"/><p>0 8 1 8 2 8</p></triangles>
      </mesh></geometry></library_geometries>
      <library_visual_scenes><visual_scene id="active"><node id="parent">
        <matrix>1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1</matrix>
        <node id="crate"><matrix>''' + matrix + '</matrix>' + extra + '''
          <instance_geometry url="#mesh"/></node></node></visual_scene></library_visual_scenes>
      <scene><instance_visual_scene url="#active"/></scene></COLLADA>''')
    return path


def test_units_node_translation_sdf_scale_and_world_translation_are_composed(tmp_path):
    sections = mesh_section(asset(tmp_path), .5, scale=.5, translation=(10., -2., 0.), node='crate')
    assert len(sections) == 1
    assert sorted(map(tuple, sections[0])) == [(10.5, -2.), (11., -2.)]
    nearest = nearest_section(sections, (10.7, -1.8))
    assert nearest['distance_m'] == pytest.approx(.2)
    assert nearest['closest_xy'] == pytest.approx([10.7, -2.])


def test_parent_rotation_applies_to_child_translation_and_geometry(tmp_path):
    path = asset(tmp_path)
    path.write_text(path.read_text().replace(
        '1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1',
        '0 -1 0 0 1 0 0 0 0 0 1 0 0 0 0 1'))
    sections = mesh_section(path, .5, scale=.5, translation=(10., -2., 0.))
    assert sorted(map(tuple, sections[0])) == [(10., -1.5), (10., -1.)]


def test_slice_above_geometry_is_empty_and_not_a_zero_distance_match(tmp_path):
    sections = mesh_section(asset(tmp_path), 3.)
    assert sections.shape == (0, 2, 2)
    assert nearest_section(sections, (0., 0.)) is None


def test_vertex_and_coplanar_edges_are_retained(tmp_path):
    sections = mesh_section(asset(tmp_path), 0.)
    assert nearest_section(sections, (2., 0.))['distance_m'] == 0.
    # A degenerate edge must not produce NaN in nearest-point projection.
    assert nearest_section(np.array([[[1., 2.], [1., 2.]]]), (4., 6.))['distance_m'] == 5.


@pytest.mark.parametrize('change', ['axis', 'transform', 'primitive', 'node', 'index'])
def test_unsupported_or_invalid_geometry_is_rejected(tmp_path, change):
    path = asset(tmp_path)
    text = path.read_text()
    if change == 'axis': text = text.replace('Z_UP', 'Y_UP')
    if change == 'transform': text = text.replace('<node id="crate">', '<node id="crate"><rotate>0 0 1 90</rotate>')
    if change == 'primitive': text = text.replace('<triangles count="1">', '<polylist count="1">').replace('</triangles>', '</polylist>')
    if change == 'index': text = text.replace('0 8 1 8 2 8', '0 8 1 8 9 8')
    path.write_text(text)
    with pytest.raises(ValueError):
        mesh_section(path, .5, node='missing' if change == 'node' else None)


def test_nonfinite_query_or_transform_is_not_accepted(tmp_path):
    with pytest.raises(ValueError): mesh_section(asset(tmp_path), float('nan'))
    with pytest.raises(ValueError): nearest_section(np.zeros((1, 2, 2)), (float('inf'), 0.))
