"""Dynamic stimulus geometry and closed-recording safety boundaries."""
import importlib.util
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest

scripts=Path(__file__).resolve().parents[1]/'scripts'

def load(name):
    spec=importlib.util.spec_from_file_location(name,scripts/(name+'.py'))
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m


def test_private_cart_has_matching_real_collision_and_visual(tmp_path):
    source=tmp_path/'world.sdf';source.write_text('<sdf version="1.7"><world name="world_demo"/></sdf>')
    original=source.read_bytes();m=load('prepare_dynamic_scene')
    target=m.prepare(source,tmp_path/'trial','blocked')
    assert source.read_bytes()==original
    root=ET.parse(target);cart=root.find(".//model[@name='trial_dynamic_cart']")
    assert cart.findtext('pose')=='-20 -20 .4 0 0 0'
    assert cart.findtext('.//collision/geometry/box/size')==cart.findtext('.//visual/geometry/box/size')=='0.5 1.6 0.8'
    with pytest.raises(ValueError):m.prepare(target,tmp_path/'second','blocked')


def test_cart_crossing_and_blocked_leave_the_route():
    pytest.importorskip('rclpy');m=load('drive_dynamic_cart')
    assert m.lateral('crossing',0)<0<m.lateral('crossing',15)
    assert m.lateral('crossing',5)==0
    assert m.lateral('blocked',7.9)==0 and m.lateral('blocked',15)==2.4
    with pytest.raises(ValueError):m.lateral('invalid',0)


def test_rotated_collision_box_and_closed_bag_requirement(tmp_path):
    pytest.importorskip('rclpy');m=load('evaluate_dynamic_obstacle')
    cart=dict(x=2.,y=3.,yaw=math.pi/2)
    assert m.box_distance(2,3,cart,[.5,1.6,.8])==0
    assert m.box_distance(2,4,cart,[.5,1.6,.8])==pytest.approx(.75)
    (tmp_path/'report.json').write_text(json.dumps(dict(recorder_exit_code=None)))
    with pytest.raises(ValueError,match='finalized'):m.evaluate(tmp_path)


def test_suite_does_not_accept_missing_audit_or_nonzero_launcher(tmp_path):
    m=load('run_dynamic_regression')
    for name in m.CASES:
        d=tmp_path/name;d.mkdir();(d/'dynamic_validation.json').write_text('{"success":true}')
    assert not m.summarize(tmp_path,dict(blocked=0,crossing=1))['success']
    assert m.summarize(tmp_path,dict(blocked=0,crossing=0))['success']
    (tmp_path/'crossing/dynamic_validation.json').unlink()
    assert not m.summarize(tmp_path,dict(blocked=0,crossing=0))['success']


def test_joint_world_preserves_both_people_and_adds_real_cart(tmp_path):
    source=tmp_path/'world.sdf'
    source.write_text('<sdf version="1.7"><world name="world_demo"><model name="recon_person_1"/><model name="recon_person_2"/></world></sdf>')
    target=load('prepare_dynamic_scene').prepare(source,tmp_path/'trial','crossing')
    names={m.attrib['name'] for m in ET.parse(target).findall('.//world/model')}
    assert names=={'recon_person_1','recon_person_2','trial_dynamic_cart'}
