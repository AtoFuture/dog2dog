"""Offline audit of target truth, real RGB-D samples, JPEG evidence and RETURN.

Truth and scene assets are never supplied to frontier selection or task policy.
The target metric compares planar representative points with the model origin;
it is not a full-body 3-D reconstruction accuracy measurement.
"""
import argparse
import json
import math
from pathlib import Path
import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from std_msgs.msg import String
from action_msgs.msg import GoalStatusArray
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from qwen_robotnav.recon_audit import match_targets


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    parser.add_argument('--target-error-limit',type=float,default=.5)
    a=parser.parse_args();root=a.directory.resolve()
    if not math.isfinite(a.target_error_limit) or a.target_error_limit<=0:
        parser.error('--target-error-limit must be finite and positive')
    report=json.loads((root/'report.json').read_text())
    mission=json.loads((root/'mission_report.json').read_text())
    scene=json.loads((root/'scene_manifest.json').read_text())
    start=report['truth_start'];home=report['home_map'];yaw=report['world_map_yaw_offset']
    truth_targets=scene.get('targets',[scene['target']])
    initial_path=root/'recon_initial_view.json'
    initial=json.loads(initial_path.read_text()) if initial_path.exists() else None
    initial_instances=list(initial['targets'].values()) if initial else []
    instances=list(mission['targets'].values())
    def world_xy(position):
        dx=position[0]-home[0];dy=position[1]-home[1]
        return [start[0]+math.cos(yaw)*dx-math.sin(yaw)*dy,
                start[1]+math.sin(yaw)*dx+math.cos(yaw)*dy]
    assignment=match_targets([world_xy(t['position_m']) for t in instances],
                             [t['position_world_m'] for t in truth_targets],a.target_error_limit)
    evidence_truth={eid:match for instance,match in zip(instances,assignment) for eid in instance['evidence_ids']}
    matched_truth={truth_targets[j]['name'] for j in assignment if j is not None}
    required_counts=mission.get('required_counts',{c:1 for c in mission['required_classes']})
    counts_ok=all(sum(t['class_name']==c and j is not None for t,j in zip(instances,assignment))>=count
                  for c,count in required_counts.items())
    observations=[o for t in mission['targets'].values() for o in t['observations']]
    stamps={o['stamp_ns'] for o in observations}
    stamps.update(o['stamp_ns'] for t in initial_instances for o in t['observations'])
    audits=[]
    for observation in observations:
        world=world_xy(observation['position_m'])
        match=evidence_truth[observation['evidence_id']]
        target_xy=truth_targets[match]['position_world_m'][:2] if match is not None else None
        jpeg=root/'detections'/observation['snapshot_file']
        decoded=cv2.imread(str(jpeg))
        audits.append(dict(evidence_id=observation['evidence_id'],class_name=observation['class_name'],
            position_map_m=observation['position_m'],position_world_xy_m=world,
            truth_target=truth_targets[match]['name'] if match is not None else None,
            truth_origin_world_xy_m=target_xy,planar_error_m=math.dist(world,target_xy) if target_xy else None,
            jpeg_decodable=decoded is not None,jpeg=str(jpeg.relative_to(root)),stamp_ns=observation['stamp_ns']))
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(root/'bag'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
    topics=['/robot1/rgbd_d435/image','/robot1/rgbd_d435/depth_image']
    status_topic='/robot1/navigate_to_pose/_action/status'
    return_topic='/g3/return_event';mission_topic='/mission/recon_status'
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics+[status_topic,return_topic,mission_topic]))
    matched={};saved=[];successful_action_ids=set();recorded_return_goal=None;recorded_mission_completed=False
    output=root/'recon_validation';output.mkdir(exist_ok=True)
    while reader.has_next():
        topic,data,_=reader.read_next()
        if topic==status_topic:
            for status in deserialize_message(data,GoalStatusArray).status_list:
                if status.status==4:successful_action_ids.add(bytes(status.goal_info.goal_id.uuid).hex())
            continue
        if topic in (return_topic,mission_topic):
            value=json.loads(deserialize_message(data,String).data)
            if topic==return_topic and value.get('attempt_id')==mission['return_attempt_id'] and value.get('status')=='succeeded':
                recorded_return_goal=value.get('goal_id')
            if topic==mission_topic and value.get('return_attempt_id')==mission['return_attempt_id'] and value.get('state')=='returned':
                recorded_mission_completed=True
            continue
        msg=deserialize_message(data,Image)
        stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        if stamp not in stamps:continue
        matched.setdefault(stamp,{})[topic]=msg
    for stamp, images in matched.items():
        if not all(t in images for t in topics):continue
        color=images[topics[0]];depth=images[topics[1]]
        if color.encoding not in ('rgb8','bgr8') or depth.encoding!='32FC1':continue
        rgb=np.ndarray((color.height,color.width,3),dtype=np.uint8,buffer=bytes(color.data),strides=(color.step,3,1)).copy()
        bgr=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR) if color.encoding=='rgb8' else rgb
        dtype=np.dtype('>f4' if depth.is_bigendian else '<f4')
        dm=np.ndarray((depth.height,depth.width),dtype=dtype,buffer=bytes(depth.data),strides=(depth.step,4)).copy()
        cv2.imwrite(str(output/f'{stamp}_rgb.png'),bgr);np.save(output/f'{stamp}_depth_m.npy',dm)
        scaled=np.clip(np.nan_to_num(dm,nan=0,posinf=0)/10*255,0,255).astype(np.uint8)
        cv2.imwrite(str(output/f'{stamp}_depth.png'),cv2.applyColorMap(scaled,cv2.COLORMAP_TURBO))
        saved.append(dict(stamp_ns=stamp,rgb_file=f'recon_validation/{stamp}_rgb.png',
                          depth_file=f'recon_validation/{stamp}_depth_m.npy',camera_frame=color.header.frame_id,
                          source='Gazebo RGB-D sensor messages recorded in rosbag'))
    initial_matches=match_targets([world_xy(t['position_m']) for t in initial_instances],
                                 [t['position_world_m'] for t in truth_targets],a.target_error_limit)
    target_results=[]
    for index,target in enumerate(truth_targets):
        found=[t for t,j in zip(instances,assignment) if j==index]
        target_results.append(dict(name=target['name'],initial_visibility_design=target.get('initial_visibility','unoccluded'),
            confirmed_before_motion=index in initial_matches if initial else None,
            confirmed_during_mission=bool(found),
            first_confirmation_stamp_ns=min((max(o['stamp_ns'] for o in t['observations']) for t in found),default=None)))
    recorded_return_success=recorded_return_goal is not None and recorded_return_goal in successful_action_ids
    initial_occlusion_control=None
    if initial is not None and scene.get('scenario')=='full':
        hidden=[t for t in target_results if t['initial_visibility_design']=='full_occlusion']
        visible=[t for t in target_results if t['initial_visibility_design']!='full_occlusion']
        initial_stamps={o['stamp_ns'] for t in initial_instances for o in t['observations']}
        initial_occlusion_control=bool(hidden and visible and not initial['motion_enabled']
            and all(t['confirmed_before_motion'] for t in visible)
            and all(not t['confirmed_before_motion'] for t in hidden)
            and initial_stamps<={sample['stamp_ns'] for sample in saved})
    success=(initial_occlusion_control is not False and recorded_return_success and recorded_mission_completed and (root/'bag/metadata.yaml').is_file()
             and report['success'] and mission['state']=='returned' and len(audits)>=2 and counts_ok and all(j is not None for j in assignment)
             and stamps<={sample['stamp_ns'] for sample in saved}
             and all(o['jpeg_decodable'] and o['planar_error_m'] is not None and o['planar_error_m']<=a.target_error_limit for o in audits)
             and not report['truth_in_navigation_tf'] and not report['static_map_loaded'])
    result=dict(success=bool(success),recorded_return_success=recorded_return_success,
                recorded_mission_completed=recorded_mission_completed,audits=audits,exact_stamp_rgbd_samples=saved,
                planar_error_limit_m=a.target_error_limit,truth_used_only_offline=True,
                synthetic_rgb_or_depth_injected=False,return_trigger=report['return_trigger'],
                truth_home_error_m=report.get('truth_home_error_m'),scenario=scene.get('scenario','single'),
                required_counts=required_counts,confirmed_instances=len(instances),
                initial_view=initial,initial_full_occlusion_control_pass=initial_occlusion_control,scene_target_results=target_results,one_to_one_assignment=assignment,
                unmatched_truth=[t['name'] for t in truth_targets if t['name'] not in matched_truth],
                unmatched_estimates=sum(j is None for j in assignment),
                scope='stationary 3-D people; occlusion labels describe intended initial scene geometry; inspect raw RGB-D')
    (root/'recon_validation.json').write_text(json.dumps(result,indent=2))
    lines=['# 侦查任务报告','',f'验收结果：{"通过" if success else "未通过"}',
        '',f'任务：确认 {required_counts}（每人两次带 JPEG 的跟踪观测），完成 {mission["minimum_exploration_goals"]} 个有效探索目标后返航。',
        f'有效探索目标：{mission["completed_exploration_goals"]}；返航真值误差：{report.get("truth_home_error_m","未完成返航")} 米。',
        f'实际 DeepSeek 选择：{report["deepseek_selections"]} 次；回退：{report["brain_fallback_selections"]} 次。',
        '', '人员平面位置误差按代表点与人物模型原点比较；不代表全身三维重建精度。','']
    for target in target_results:
        lines.append(f'- {target["name"]}（初始设计 {target["initial_visibility_design"]}）：静止阶段确认 {target["confirmed_before_motion"]}；任务中确认 {target["confirmed_during_mission"]}。')
    for audit in audits:lines.append(f'- 证据 {audit["evidence_id"]}：匹配 {audit["truth_target"]}，平面误差 {audit["planar_error_m"]} 米，[抓拍]({audit["jpeg"]})。')
    lines+=['','## 真实 RGB-D 帧','']
    for sample in saved:lines.append(f'![RGB {sample["stamp_ns"]}]({sample["rgb_file"]})')
    lines+=['','范围限制：静态仿真人物；遮挡标签为初始几何设计，须结合原始帧判读。未验证移动人群、近距离人员区分、真实人员检出率或真机。']
    (root/'mission_report.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(result),flush=True)
    return 0 if success else 1


if __name__=='__main__':raise SystemExit(main())
