import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest

script=Path(__file__).resolve().parents[1]/'scripts/prepare_recon_scene.py'
spec=importlib.util.spec_from_file_location('prepare_recon_scene',script)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_private_scene_rewrites_only_target_asset_and_preserves_original(tmp_path):
    world=tmp_path/'world.sdf';world.write_text('<sdf><world name="world_demo"><model name="wall"/></world></sdf>')
    asset=tmp_path/'person';(asset/'meshes').mkdir(parents=True)
    (asset/'meshes/standing.dae').write_text('test mesh')
    (asset/'model.sdf').write_text('<sdf><model name="person"><link name="body"><visual name="body"><geometry><mesh><uri>model://person/meshes/standing.dae</uri></mesh></geometry></visual></link></model></sdf>')
    output=module.prepare(world,asset,tmp_path/'output')
    root=ET.parse(output).getroot()
    assert len(root.findall('.//model'))==2
    assert root.findtext('.//model[@name="recon_person_1"]/static')=='true'
    assert root.findtext('.//uri')==str(asset/'meshes/standing.dae')
    assert 'recon_person_1' not in world.read_text()
    (asset/'model.sdf').write_text('<sdf><model name="person"><uri>../../escape.dae</uri></model></sdf>')
    with pytest.raises(ValueError):module.prepare(world,asset,tmp_path/'bad')
