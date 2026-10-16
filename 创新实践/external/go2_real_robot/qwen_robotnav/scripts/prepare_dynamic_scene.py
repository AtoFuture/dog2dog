"""Add one colliding cart to a private simulator world, initially parked off site."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def prepare(world, output, scenario, *, encounter_distance_m=1.5, duration_s=15.):
    if not 1.<=encounter_distance_m<=1.5 or not 10.<=duration_s<=15.:raise ValueError('Invalid scoped fixture distance/duration')
    if scenario not in ('crossing','blocked'):raise ValueError('Unknown dynamic scenario')
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    tree=ET.parse(world);root=tree.getroot().find('world')
    if root is None or root.find("model[@name='trial_dynamic_cart']") is not None:
        raise ValueError('Require an unmodified warehouse world')
    size=[.5,.6 if scenario=='crossing' else 1.6,.8]
    model=ET.SubElement(root,'model',name='trial_dynamic_cart')
    ET.SubElement(model,'static').text='true'
    ET.SubElement(model,'pose').text='-20 -20 .4 0 0 0'
    link=ET.SubElement(model,'link',name='body')
    for kind in ('collision','visual'):
        item=ET.SubElement(link,kind,name='body')
        ET.SubElement(ET.SubElement(ET.SubElement(item,'geometry'),'box'),'size').text=' '.join(map(str,size))
        if kind=='visual':
            material=ET.SubElement(item,'material')
            ET.SubElement(material,'ambient').text='.9 .35 .05 1'
            ET.SubElement(material,'diffuse').text='.9 .35 .05 1'
    target=output/'dynamic_warehouse.sdf';tree.write(target,encoding='utf-8',xml_declaration=True)
    (output/'dynamic_scene.json').write_text(json.dumps(dict(scenario=scenario,model='trial_dynamic_cart',
        size_m=size,world=str(target),encounter_distance_m=encounter_distance_m,duration_sim_s=duration_s,evaluation_only=True,stimulus='Gazebo colliding box; real rendered lidar'),indent=2))
    return target


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--world',type=Path,required=True);p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--scenario',choices=('crossing','blocked'),required=True)
    p.add_argument('--encounter-distance',type=float,default=1.5);p.add_argument('--duration-s',type=float,default=15.)
    a=p.parse_args();print(prepare(a.world,a.output_dir,a.scenario,encounter_distance_m=a.encounter_distance,duration_s=a.duration_s))
