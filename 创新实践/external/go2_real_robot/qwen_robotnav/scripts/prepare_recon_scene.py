"""Add a real 3-D person to a private warehouse world; truth stays evaluation-only."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def prepare(world_path, person_directory, output_directory):
    output_directory=Path(output_directory).resolve();output_directory.mkdir(parents=True,exist_ok=True)
    person_directory=Path(person_directory).resolve()
    model_file=person_directory/'model.sdf'
    model=copy.deepcopy(ET.parse(model_file).getroot().find('model'))
    if model is None:raise ValueError('Person asset has no model')
    model.set('name','recon_person_1')
    pose=model.find('pose')
    if pose is None:pose=ET.SubElement(model,'pose')
    pose.text='10.3 6.0 0 0 0 -1.5707963267948966'
    static=model.find('static')
    if static is None:static=ET.SubElement(model,'static')
    static.text='true'
    for uri in model.findall('.//uri'):
        value=(uri.text or '').strip()
        if value.startswith('model://'):
            value=value[len('model://'):].split('/',1)[1]
        elif value.startswith('file://'):value=value[len('file://'):]
        path=(person_directory/value).resolve()
        if not path.is_relative_to(person_directory) or not path.is_file():
            raise ValueError('Model asset missing or outside its directory: '+value)
        uri.text=str(path)
    world=ET.parse(world_path)
    world.getroot().find('world').append(model)
    target=output_directory/'recon_warehouse.sdf'
    world.write(target,encoding='utf-8',xml_declaration=True)
    manifest=dict(world=str(target),target=dict(name='recon_person_1',class_name='person',
        position_world_m=[10.3,6.,0.],yaw_world_rad=-1.5707963267948966),
        evaluation_only=True,source_model=str(model_file),
        source_model_sha256=hashlib.sha256(model_file.read_bytes()).hexdigest(),
        asset_source='https://fuel.gazebosim.org/1.0/OpenRobotics/models/Standing%20person',
        asset_version=3,asset_license='CC0-1.0',
        camera_input='Gazebo rendered RGB-D; no injected photograph or depth')
    (output_directory/'scene_manifest.json').write_text(json.dumps(manifest,indent=2))
    return target


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--world',type=Path,required=True)
    p.add_argument('--person-model',type=Path,required=True)
    p.add_argument('--output-dir',type=Path,required=True)
    a=p.parse_args();print(prepare(a.world,a.person_model,a.output_dir))


if __name__=='__main__':main()
