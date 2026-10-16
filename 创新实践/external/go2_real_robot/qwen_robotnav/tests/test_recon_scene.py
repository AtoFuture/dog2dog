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


@pytest.mark.parametrize('scenario,panels',[('multi',0),('partial',1),('full',1)])
def test_multiple_people_and_physical_occluder_geometry(tmp_path,scenario,panels):
    import json
    world=tmp_path/'world.sdf';world.write_text('<sdf><world name="world_demo"/></sdf>')
    asset=tmp_path/'person';asset.mkdir()
    (asset/'model.sdf').write_text('<sdf><model name="person"><link name="body"/></model></sdf>')
    output=module.prepare(world,asset,tmp_path/'output',scenario)
    root=ET.parse(output).getroot()
    assert len(root.findall('.//model'))==2+panels
    manifest=json.loads((output.parent/'scene_manifest.json').read_text())
    assert len(manifest['targets'])==2 and manifest['evaluation_only']
    if panels:
        panel=root.find('.//model[@name="recon_occluder"]')
        assert panel.findtext('static')=='true'
        assert panel.findtext('link/collision/geometry/box/size')==panel.findtext('link/visual/geometry/box/size')
        assert manifest['targets'][1]['initial_visibility']==scenario+'_occlusion'


@pytest.mark.parametrize('scenario',('moving_crossing','moving_parallel','moving_occlusion'))
def test_moving_scene_manifest_matches_generated_initial_trajectory(tmp_path,scenario):
    import json
    from qwen_robotnav.person_motion import positions
    world=tmp_path/'world.sdf';world.write_text('<sdf><world name="world_demo"/></sdf>')
    asset=tmp_path/'person';asset.mkdir()
    (asset/'model.sdf').write_text('<sdf><model name="person"><link name="body"/></model></sdf>')
    output=module.prepare(world,asset,tmp_path/'output',scenario)
    manifest=json.loads((output.parent/'scene_manifest.json').read_text())
    assert manifest['moving_people'] and manifest['evaluation_only']
    assert [t['position_world_m'][:2] for t in manifest['targets']]==[list(p) for p in positions(scenario,0)]
    assert len(manifest['occluders'])==(1 if scenario=='moving_occlusion' else 0)
