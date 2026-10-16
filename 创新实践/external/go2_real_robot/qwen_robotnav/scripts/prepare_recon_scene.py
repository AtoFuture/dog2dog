"""Add a real 3-D person to a private warehouse world; truth stays evaluation-only."""
import argparse
import copy
import hashlib
import json
import sys
from pathlib import Path
import xml.etree.ElementTree as ET


sys.path.insert(0,str(Path(__file__).resolve().parents[1]))

SCENARIOS=('single','multi','partial','full','moving_crossing','moving_parallel','moving_occlusion')


def prepare(world_path, person_directory, output_directory, scenario='single'):
    if scenario not in SCENARIOS:raise ValueError('Unknown scene scenario')
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
    root=world.getroot().find('world')
    positions=[(10.3,6.0)] if scenario=='single' else [(10.3,5.25),(10.3,7.05)]
    if scenario.startswith('moving_'):
        from qwen_robotnav.person_motion import positions as moving_positions
        positions=moving_positions(scenario,0.)
    targets=[]
    for index,(x,y) in enumerate(positions,1):
        person=copy.deepcopy(model);name=f'recon_person_{index}';person.set('name',name)
        person.find('pose').text=f'{x} {y} 0 0 0 -1.5707963267948966'
        root.append(person)
        targets.append(dict(name=name,class_name='person',position_world_m=[x,y,0.],
                            yaw_world_rad=-1.5707963267948966,
                            initial_visibility='full_occlusion' if scenario in ('full','moving_occlusion') and index==2 else
                            'partial_occlusion' if scenario=='partial' and index==2 else 'unoccluded'))
    occluders=[]
    if scenario in ('partial','full','moving_occlusion'):
        # Vertical side panel for partial: hide roughly half the silhouette rather
        # than just its feet (the camera already crops the upper body at close range).
        x=9.65;y=6.96 if scenario=='partial' else 6.83
        width=.30 if scenario=='partial' else 1.05
        size=[.12,width,2.2];pose=[x,y,1.1,0.,0.,0.]
        panel=ET.SubElement(root,'model',name='recon_occluder')
        ET.SubElement(panel,'static').text='true';ET.SubElement(panel,'pose').text=' '.join(map(str,pose))
        link=ET.SubElement(panel,'link',name='panel')
        for kind in ('collision','visual'):
            element=ET.SubElement(link,kind,name='panel')
            ET.SubElement(ET.SubElement(ET.SubElement(element,'geometry'),'box'),'size').text=' '.join(map(str,size))
            if kind=='visual':
                material=ET.SubElement(element,'material')
                ET.SubElement(material,'ambient').text='.35 .38 .42 1'
                ET.SubElement(material,'diffuse').text='.35 .38 .42 1'
        occluders.append(dict(name='recon_occluder',pose_world=pose,size_m=size))

    target=output_directory/'recon_warehouse.sdf'
    world.write(target,encoding='utf-8',xml_declaration=True)
    manifest=dict(world=str(target),target=targets[0],targets=targets,scenario=scenario,occluders=occluders,
        moving_people=scenario.startswith('moving_'),evaluation_only=True,source_model=str(model_file),
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
    p.add_argument('--scenario',choices=SCENARIOS,default='single')
    a=p.parse_args();print(prepare(a.world,a.person_model,a.output_dir,a.scenario))


if __name__=='__main__':main()
