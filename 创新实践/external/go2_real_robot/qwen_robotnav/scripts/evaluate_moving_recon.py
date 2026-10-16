"""Closed-bag moving identity audit against image-time Gazebo target poses."""
import argparse
from collections import defaultdict,Counter
import json
import math
from pathlib import Path
import sys
import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from action_msgs.msg import GoalStatusArray
from sensor_msgs.msg import Image
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from qwen_robotnav.recon_audit import match_targets


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path)
    a=p.parse_args();root=a.directory.resolve()
    if not (root/'bag/metadata.yaml').is_file():p.error('A finalized bag is required')
    scene=json.loads((root/'scene_manifest.json').read_text())
    if not scene.get('moving_people'):p.error('Moving scene required')
    report=json.loads((root/'report.json').read_text());mission=json.loads((root/'mission_report.json').read_text())
    motion=json.loads((root/'person_motion.json').read_text())
    evidence=json.loads((root/'detections/evidence.json').read_text())
    names=[t['name'] for t in scene['targets']];truth={name:{} for name in names}
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(root/'bag'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
    topics=['/clock','/frontier_trial/gt_pose','/g3/return_event','/mission/recon_status','/robot1/navigate_to_pose/_action/status']
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    clock=None;return_goal=None;successful=set();terminal=False
    while reader.has_next():
        topic,data,_=reader.read_next()
        if topic=='/clock':
            c=deserialize_message(data,Clock).clock;clock=c.sec*10**9+c.nanosec
        elif topic=='/frontier_trial/gt_pose':
            for tr in deserialize_message(data,TFMessage).transforms:
                if tr.child_frame_id not in truth:continue
                stamp=tr.header.stamp.sec*10**9+tr.header.stamp.nanosec;stamp=stamp or clock
                if stamp is not None:truth[tr.child_frame_id][stamp]=[tr.transform.translation.x,tr.transform.translation.y]
        elif topic=='/robot1/navigate_to_pose/_action/status':
            for status in deserialize_message(data,GoalStatusArray).status_list:
                if status.status==4:successful.add(bytes(status.goal_info.goal_id.uuid).hex())
        else:
            value=json.loads(deserialize_message(data,String).data)
            if topic=='/g3/return_event' and value.get('attempt_id')==mission['return_attempt_id'] and value.get('status')=='succeeded':
                return_goal=value.get('goal_id')
            if topic=='/mission/recon_status' and value.get('state')==mission['state'] and value.get('return_attempt_id')==mission['return_attempt_id']:
                terminal=mission['state'] in ('returned','returned_incomplete')
    ids={int(k):v for k,v in mission.get('identity_diagnostics',{}).get('assignments',{}).items()}
    observations={o['evidence_id']:o for o in evidence['observations']}
    frames=defaultdict(list)
    for eid,identity in ids.items():
        if eid in observations:frames[observations[eid]['stamp_ns']].append((identity,observations[eid]))
    start=report['truth_start'];home=report['home_map'];yaw=report['world_map_yaw_offset']
    def world(position):
        x,y=position[0]-home[0],position[1]-home[1]
        return [start[0]+math.cos(yaw)*x-math.sin(yaw)*y,start[1]+math.sin(yaw)*x+math.cos(yaw)*y]
    def at(name,stamp):
        poses=truth[name]
        nearest=min(poses,key=lambda t:abs(t-stamp)) if poses else None
        return poses[nearest] if nearest is not None and abs(nearest-stamp)<=200_000_000 else None
    audits=[];histories=defaultdict(list);movement_samples=defaultdict(list)
    for stamp,values in sorted(frames.items()):
        positions=[at(name,stamp) for name in names]
        valid=[i for i,xy in enumerate(positions) if xy is not None]
        assignment=match_targets([world(o['position_m']) for _,o in values],[positions[i] for i in valid],.5)
        for (identity,o),j in zip(values,assignment):
            name=names[valid[j]] if j is not None else None
            xy=world(o['position_m']);error=math.dist(xy,positions[valid[j]]) if j is not None else None
            jpeg_ok=cv2.imread(str(root/'detections'/o['snapshot_file'])) is not None
            audits.append(dict(evidence_id=o['evidence_id'],identity_id=identity,track_id=o['track_id'],
                stamp_ns=stamp,truth_name=name,planar_error_m=error,jpeg_decodable=jpeg_ok))
            if name:histories[identity].append(name)
            if name and motion['start_stamp_ns']<=stamp<=motion.get('end_stamp_ns',motion['start_stamp_ns']+int(motion['duration_sim_s']*1e9)):
                movement_samples[name].append((stamp,positions[valid[j]],o['evidence_id']))
    switches=sum(sum(a!=b for a,b in zip(history,history[1:])) for history in histories.values())
    majority={identity:Counter(history).most_common(1)[0][0] for identity,history in histories.items()}
    duplicates=sum(max(0,n-1) for n in Counter(majority.values()).values())
    target_audits=[];selected=set()
    for name in names:
        samples=movement_samples[name];span=(samples[-1][0]-samples[0][0])*1e-9 if samples else 0.
        displacement=max((math.dist(a[1],b[1]) for a in samples for b in samples),default=0.)
        target_audits.append(dict(name=name,associated_motion_captures=len(samples),observed_span_s=span,
            observed_truth_displacement_m=displacement,
            passed=len(samples)>=6 and span>=motion['duration_sim_s']*.7 and displacement>=.5))
        if samples:
            for index in (0,len(samples)//2,len(samples)-1):selected.add(samples[index][0])
    # Export a bounded set of exact image-time RGB-D pairs, preserving the complete bag.
    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(root/'bag'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
    image_topics=['/robot1/rgbd_d435/image','/robot1/rgbd_d435/depth_image']
    reader.set_filter(rosbag2_py.StorageFilter(topics=image_topics));images=defaultdict(dict)
    while reader.has_next():
        topic,data,_=reader.read_next();msg=deserialize_message(data,Image)
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if stamp in selected:images[stamp][topic]=msg
    out=root/'moving_validation';out.mkdir(exist_ok=True);saved=[]
    for stamp,values in sorted(images.items()):
        if any(t not in values for t in image_topics):continue
        rgb,depth=(values[t] for t in image_topics)
        if rgb.encoding not in ('rgb8','bgr8') or depth.encoding!='32FC1':continue
        array=np.ndarray((rgb.height,rgb.width,3),dtype=np.uint8,buffer=bytes(rgb.data),strides=(rgb.step,3,1)).copy()
        if rgb.encoding=='rgb8':array=cv2.cvtColor(array,cv2.COLOR_RGB2BGR)
        dtype=np.dtype('>f4' if depth.is_bigendian else '<f4')
        dm=np.ndarray((depth.height,depth.width),dtype=dtype,buffer=bytes(depth.data),strides=(depth.step,4)).copy()
        cv2.imwrite(str(out/f'{stamp}_rgb.png'),array);np.save(out/f'{stamp}_depth_m.npy',dm)
        saved.append(dict(stamp_ns=stamp,rgb=f'moving_validation/{stamp}_rgb.png',depth=f'moving_validation/{stamp}_depth_m.npy'))
    safe_return=bool(return_goal in successful and terminal and report.get('navigation_return_success')
                     and report.get('truth_home_error_m',float('inf'))<=.4)
    identity_pass=bool(motion.get('success') and audits and all(t['passed'] for t in target_audits)
        and switches==0 and duplicates==0 and all(o['truth_name'] is not None and o['jpeg_decodable'] for o in audits)
        and len(mission['targets'])==len(names) and len(set(majority.values()))==len(names)
        and selected=={s['stamp_ns'] for s in saved}
        and not mission.get('identity_diagnostics',{}).get('capacity_exceeded',True))
    success=identity_pass and safe_return and report['success'] and mission['state']=='returned'
    result=dict(success=bool(success),identity_validation_success=identity_pass,safe_return_success=safe_return,
        identity_switches=switches,duplicate_identities=duplicates,confirmed_identities=len(mission['targets']),
        unmatched_associated_captures=sum(o['truth_name'] is None for o in audits),
        target_audits=target_audits,audits=audits,exact_stamp_rgbd_samples=saved,
        truth_time_alignment='pose header or preceding bag /clock, maximum 0.2 s mismatch',
        truth_used_only_offline=True,scenario=scene['scenario'],
        scope='rendered rigid people moving during a stationary observation phase; autonomous exploration and return follow; no walking animation or hardware validation')
    (root/'moving_validation.json').write_text(json.dumps(result,indent=2))
    lines=['# 移动多人跟踪验证','',f'完整任务：{success}；身份验证：{identity_pass}；安全返航：{safe_return}',
        f'确认身份：{len(mission["targets"])}；身份交换：{switches}；重复身份：{duplicates}。',
        '',result['scope'],'']
    for target in target_audits:lines.append(f'- {target}')
    for sample in saved:lines.append(f'![{sample["stamp_ns"]}]({sample["rgb"]})')
    (root/'mission_report.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='audits'}),flush=True)
    return 0 if success else 1


if __name__=='__main__':raise SystemExit(main())
