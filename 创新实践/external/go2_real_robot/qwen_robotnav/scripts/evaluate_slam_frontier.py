"""Evaluate a saved SLAM frontier bag against the existing warehouse reference.

Registration uses the initial truth/SLAM correspondence only for this offline
measurement. The registered YAML is an evaluation artifact, never a nav input.
"""
import argparse
import json
import math
from pathlib import Path

import yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from g3_integration_eval.evaluate_bag import evaluate_bag, write_report
from g3_integration_eval.coverage import load_map

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('output_dir',type=Path)
parser.add_argument('--reference-map',type=Path,default=Path.home()/'go2_sim_ws/ROS2-Gazebo-GO2/src/navigation2/maps/warehouse_map.yaml')
args=parser.parse_args();root=args.output_dir.resolve()
r=json.loads((root/'report.json').read_text())
m=yaml.safe_load((root/'slam_map.yaml').read_text())
# Fixed trinary PGM encoding uses 205 for unknown (occupancy 0.196078...).
# A free threshold of .25 would silently reload those unknown cells as free.
normalization={}
for label,path,config in [('slam',root/'slam_map.yaml',m),
                          ('reference',args.reference_map,yaml.safe_load(args.reference_map.read_text()))]:
    pixels=set(load_map(path).data)
    if pixels <= {0,205,254} and config.get('mode','trinary')=='trinary' and config.get('free_thresh',.196)>.196:
        normalization[label]={'original_free_thresh':config['free_thresh'],'effective_free_thresh':.196}
        config['free_thresh']=.196
    config['image']=str((path.parent/config['image']).resolve())
    if label=='reference':
        reference=root/'reference_map_evaluation.yaml'
        reference.write_text(yaml.safe_dump(config,sort_keys=False))
    elif 'slam' in normalization:
        backup=root/'slam_map_original_metadata.yaml'
        if not backup.exists():backup.write_text(path.read_text())
        path.write_text(yaml.safe_dump(config,sort_keys=False))
x,y,theta=m['origin'];hx,hy,_=r['home_map'];tx,ty=r['truth_start'][:2]
a=r['world_map_yaw_offset'];c,s=math.cos(a),math.sin(a)
m['origin']=[tx+c*(x-hx)-s*(y-hy),ty+s*(x-hx)+c*(y-hy),theta+a]
m['image']=str((root/m['image']).resolve())
registered=root/'slam_map_evaluation_world.yaml'
registered.write_text(yaml.safe_dump(m,sort_keys=False))
report=evaluate_bag(str(root/'bag'),reference_map=str(reference),slam_map=str(registered),
                    battery_topic='/frontier_trial/battery')
# The shared evaluator excludes RETURN IDs from its exploration-rate denominator.
# Also retain a raw terminal-status audit across both navigation owners.
reader=rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=str(root/'bag'),storage_id='sqlite3'),
            rosbag2_py.ConverterOptions(input_serialization_format='cdr',output_serialization_format='cdr'))
status_topic='/robot1/navigate_to_pose/_action/status'
reader.set_filter(rosbag2_py.StorageFilter(topics=[status_topic]))
status_type=get_message('action_msgs/msg/GoalStatusArray')
observed=set(report['navigation']['observed_goal_ids']);terminals={goal:set() for goal in observed}
while reader.has_next():
    topic,data,stamp=reader.read_next()
    for item in deserialize_message(data,status_type).status_list:
        goal=bytes(item.goal_info.goal_id.uuid).hex()
        if goal in observed and item.status in (4,5,6):terminals[goal].add(item.status)
report['all_navigation_terminal_audit']={name:sum(states=={status} for states in terminals.values())
                                       for name,status in [('succeeded',4),('canceled',5),('aborted',6)]}
report['all_navigation_terminal_audit'].update(incomplete=sum(not v for v in terminals.values()),
    conflict=sum(len(v)>1 for v in terminals.values()))
report['trinary_unknown_threshold_corrections']=normalization
report['reference_source']=str(args.reference_map)
report['registration']={'method':'initial truth / SLAM pose correspondence, offline only',
                        'used_in_navigation':False,'yaw_offset':a}
report['frontier_validation']={k:r[k] for k in ['success','meaningful_frontier_goals',
    'truth_home_error_m','initial_known_area_m2','final_known_area_m2'] if k in r}
path=write_report(report,root/'evaluation')
print(json.dumps({'report':str(path),'navigation':report['navigation'],
                  'coverage':report['metrics']['coverage'],'return':report['metrics']['return_success_rate']}))
