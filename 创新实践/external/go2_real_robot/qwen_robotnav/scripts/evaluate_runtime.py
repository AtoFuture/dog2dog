"""Read-only closed-bag acceptance of the standalone velocity runtime."""
import argparse
import json
import math
from pathlib import Path
import sqlite3
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from qwen_robotnav.runtime_profile import load_profile,profile_hash,ros_parameters,expected_runtime_roles,runtime_exit_ok


def evaluate(root):
    root=Path(root);report=json.loads((root/'report.json').read_text())
    if report.get('recorder_exit_code')!=0 or not (root/'bag/metadata.yaml').exists():raise ValueError('Require finalized closed recording')
    profile=load_profile(root/'runtime_profile.yaml');t=profile['topics']
    manifest=json.loads((root/'runtime_manifest.json').read_text());exit_info=json.loads((root/'runtime_exit.json').read_text())
    successes=set();accepted=set();returned=[];goals=set();commands=[];status=[];poses=[];states=[]
    names={t['actuator_command'],t['guard_status'],t['mission'],'/exploration/events','/g3/return_event',t['action']+'/_action/status','/frontier_trial/gt_pose'}
    brain_selected=0;brain_fallback=0
    last_pose=-math.inf
    with sqlite3.connect('file:'+str(next((root/'bag').glob('*.db3')))+'?mode=ro',uri=True) as db:
        types={i:(name,get_message(kind)) for i,name,kind in db.execute('SELECT id,name,type FROM topics') if name in names}
        for tid,stamp,data in db.execute('SELECT topic_id,timestamp,data FROM messages WHERE topic_id IN ('+','.join(map(str,types))+') ORDER BY timestamp,id'):
            name,typ=types[tid];wall=stamp*1e-9
            if name=='/frontier_trial/gt_pose' and wall-last_pose<.05:continue
            msg=deserialize_message(data,typ)
            if name==t['actuator_command']:commands.append((wall,msg.linear.x,msg.angular.z))
            elif name==t['guard_status']:status.append(json.loads(msg.data))
            elif name==t['mission']:states.append((wall,msg.data))
            elif name=='/frontier_trial/gt_pose':
                for tr in msg.transforms:
                    if tr.child_frame_id=='robot1_my_bot':poses.append((wall,tr.transform.translation.x,tr.transform.translation.y));last_pose=wall
            elif name.endswith('/_action/status'):
                successes.update(bytes(v.goal_info.goal_id.uuid).hex() for v in msg.status_list if v.status==4)
            elif name=='/g3/return_event':
                e=json.loads(msg.data)
                if e['status']=='accepted':accepted.add((e['attempt_id'],e['goal_id']))
                if e['status']=='succeeded':returned.append((e['attempt_id'],e['goal_id']))
            else:
                e=json.loads(msg.data)
                if e['status']=='brain_selected':
                    if e.get('fallback') is False:brain_selected+=1
                    else:brain_fallback+=1
                if e['status']=='succeeded' and e.get('geodesic_dist_m',0)>=.75:goals.add(e['attempt_id'])
    motion=[c for c in commands if abs(c[1])+abs(c[2])>1e-6]
    allowed=all(abs(c[1])<=profile['guard']['linear_max']+1e-8 and abs(c[2])<=profile['guard']['angular_max']+1e-8 and c[1]>=0 for c in commands)
    returned_ok=len(returned)==1 and returned[0] in accepted and returned[0][1] in successes
    home_error=math.dist(poses[0][1:],poses[-1][1:]) if len(poses)>1 else math.inf
    checks=dict(main_passed=report.get('success') is True,runtime_exited_cleanly=runtime_exit_ok(exit_info,expected_runtime_roles(profile)) and report.get('runtime_exit_code')==0,
       same_profile_hash=bool(status) and all(s['profile_sha256']==profile_hash(profile)==manifest['profile_sha256'] for s in status),
       sole_guard_publisher=report.get('actuator_publishers')==['velocity_guard'] and not any(s['publisher_conflict'] for s in status),
       no_truth_in_runtime=manifest.get('truth_used_by_runtime') is False and report.get('observer_publishes_velocity') is False,
       actual_guarded_motion=bool(motion) and bool(poses) and max(math.dist(p[1:],poses[0][1:]) for p in poses)>=.5,
       command_limits=bool(commands) and allowed,completed_frontiers=len(goals)>=2,
       real_return_action=returned_ok,body_returned_home=home_error<=.4)
    checks['single_goal_pipeline']={c['name'] for c in manifest.get('children',[])}==expected_runtime_roles(profile) and manifest.get('goal_source')=='g2_frontier'
    checks['one_effective_configuration']=yaml.safe_load((root/'runtime_ros_params.yaml').read_text())==ros_parameters(profile) and manifest.get('candidate_selector')==profile['decision']['candidate_selector']
    if profile['decision']['candidate_selector']=='deepseek':checks['real_deepseek_selection']=brain_selected>0
    # Verify zero outputs after final STANDBY (then ESTOP during cleanup).
    ends=[wall for wall,state in states if state=='standby' and returned and wall>commands[0][0] and any(c[0]<wall for c in motion)] if commands else []
    quiet=[c for c in commands if ends and c[0]>=ends[0]+.2]
    checks['final_standby_zero']=bool(quiet) and all(abs(c[1])+abs(c[2])<1e-8 for c in quiet)
    result=dict(success=all(checks.values()),checks=checks,home_error_m=home_error if math.isfinite(home_error) else None,completed_frontiers=len(goals),
                actuator_command_samples=len(commands),moving_command_samples=len(motion),profile_sha256=profile_hash(profile),
                candidate_selector=profile['decision']['candidate_selector'],deepseek_selections=brain_selected,brain_fallback_selections=brain_fallback,
                return_goal_ids=[goal for attempt,goal in returned],scope='ordinary Gazebo warehouse; standalone runtime without evaluator velocity publisher')
    (root/'runtime_validation.json').write_text(json.dumps(result,indent=2));return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('output_dir',type=Path);a=p.parse_args()
    result=evaluate(a.output_dir);print(json.dumps(result,indent=2));return 0 if result['success'] else 1


if __name__=='__main__':raise SystemExit(main())
