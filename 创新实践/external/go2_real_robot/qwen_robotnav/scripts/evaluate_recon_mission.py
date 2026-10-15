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
    target_xy=scene['target']['position_world_m'][:2]
    observations=[o for t in mission['targets'].values() for o in t['observations']]
    stamps={o['stamp_ns'] for o in observations};audits=[]
    for observation in observations:
        dx=observation['position_m'][0]-home[0];dy=observation['position_m'][1]-home[1]
        world=[start[0]+math.cos(yaw)*dx-math.sin(yaw)*dy,
               start[1]+math.sin(yaw)*dx+math.cos(yaw)*dy]
        jpeg=root/'detections'/observation['snapshot_file']
        decoded=cv2.imread(str(jpeg))
        audits.append(dict(evidence_id=observation['evidence_id'],class_name=observation['class_name'],
            position_map_m=observation['position_m'],position_world_xy_m=world,
            truth_origin_world_xy_m=target_xy,planar_error_m=math.dist(world,target_xy),
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
    recorded_return_success=recorded_return_goal is not None and recorded_return_goal in successful_action_ids
    success=(recorded_return_success and recorded_mission_completed and (root/'bag/metadata.yaml').is_file()
             and report['success'] and mission['state']=='returned' and len(audits)>=2 and bool(saved)
             and all(o['jpeg_decodable'] and o['planar_error_m']<=a.target_error_limit for o in audits)
             and not report['truth_in_navigation_tf'] and not report['static_map_loaded'])
    result=dict(success=bool(success),recorded_return_success=recorded_return_success,
                recorded_mission_completed=recorded_mission_completed,audits=audits,exact_stamp_rgbd_samples=saved,
                planar_error_limit_m=a.target_error_limit,truth_used_only_offline=True,
                synthetic_rgb_or_depth_injected=False,return_trigger=report['return_trigger'],
                truth_home_error_m=report.get('truth_home_error_m'),scope='one stationary 3-D person in one warehouse scene')
    (root/'recon_validation.json').write_text(json.dumps(result,indent=2))
    lines=['# 侦查任务报告','',f'验收结果：{"通过" if success else "未通过"}',
        '',f'任务：确认 person（两次带 JPEG 的跟踪观测），完成 {mission["minimum_exploration_goals"]} 个有效探索目标后返航。',
        f'有效探索目标：{mission["completed_exploration_goals"]}；返航真值误差：{report.get("truth_home_error_m","未完成返航")} 米。',
        f'实际 DeepSeek 选择：{report["deepseek_selections"]} 次；回退：{report["brain_fallback_selections"]} 次。',
        '', '人员平面位置误差按代表点与人物模型原点比较；不代表全身三维重建精度。','']
    for audit in audits:lines.append(f'- 证据 {audit["evidence_id"]}：平面误差 {audit["planar_error_m"]:.3f} 米，[抓拍]({audit["jpeg"]})。')
    lines+=['','## 真实 RGB-D 帧','']
    for sample in saved:lines.append(f'![RGB {sample["stamp_ns"]}]({sample["rgb_file"]})')
    lines+=['','范围限制：单个静态仿真人物，未验证多人、遮挡、真实世界人员检出率或真机。']
    (root/'mission_report.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(result),flush=True)
    return 0 if success else 1


if __name__=='__main__':raise SystemExit(main())
