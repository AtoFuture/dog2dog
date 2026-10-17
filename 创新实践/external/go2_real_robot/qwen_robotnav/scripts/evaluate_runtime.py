"""Read-only closed-bag acceptance of the standalone velocity runtime."""
import argparse
import json
import math
from pathlib import Path
import sqlite3
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from qwen_robotnav.static_scenarios import load_scene,evaluate_scene,scan_wall_hits,quaternion_rpy,compose_pose,inverse_pose
from qwen_robotnav.runtime_profile import load_profile,profile_hash,ros_parameters,expected_runtime_roles,runtime_exit_ok



def autonomous_termination_ok(results, requests, commands, triggers):
    if not results or any(e != results[0] for e in results):
        return False
    result = results[0]
    reasons = {'no_frontiers': True, 'time_budget': False, 'coverage_stalled': False,
               'candidates_unavailable': False, 'navigation_failures': False}
    return (result.get('reason') in reasons
            and result.get('completed') is reasons[result['reason']]
            and isinstance(result.get('session_id'), str) and bool(result['session_id'])
            and any(e == result for e in requests)
            and 'return' not in commands and 'explore' in commands
            and triggers == ['exploration:'+result['reason']])


def evaluate(root):
    root=Path(root);report=json.loads((root/'report.json').read_text())
    if report.get('recorder_exit_code')!=0 or not (root/'bag/metadata.yaml').exists():raise ValueError('Require finalized closed recording')
    profile=load_profile(root/'runtime_profile.yaml');t=profile['topics']
    scene=load_scene(root) if (root/'static_scene.json').exists() else None
    if report.get('static_scenario') and not scene:raise ValueError('Missing static fixture metadata')
    transforms={};alignment=None
    body=[];frontier_events=[];areas=[];sensor=None;pose=None;hit_frames=[0]*len(scene['boxes']) if scene else []
    manifest=json.loads((root/'runtime_manifest.json').read_text());exit_info=json.loads((root/'runtime_exit.json').read_text())
    successes=set();accepted=set();returned=[];goals=set();commands=[];status=[];poses=[];states=[]
    names={t['actuator_command'],t['guard_status'],t['mission'],'/exploration/events','/g3/return_event',t['action']+'/_action/status','/frontier_trial/gt_pose'}
    completions=[];exploration_results=[];operator_commands=[];return_triggers=[]
    names.update({t['command'],'/exploration/completion','/g3/exploration_result',t['map']})
    names.add(t['tf'])
    if scene:names.update({t['scan'],t['tf_static']})
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
            elif name==t['mission']:
                states.append((wall,msg.data))
                if msg.data=='exploring' and alignment is None and pose is not None:
                    first=transforms.get((profile['frames']['map'],profile['frames']['odom']))
                    second=transforms.get((profile['frames']['odom'],profile['frames']['base']))
                    if first and second and max(wall-pose[0],wall-first[0],wall-second[0])<=.5:
                        alignment=compose_pose(pose[1:],inverse_pose(compose_pose(first[1:],second[1:])))
            elif name=='/frontier_trial/gt_pose':
                for tr in msg.transforms:
                    if tr.child_frame_id=='robot1_my_bot':
                        translation=tr.transform.translation
                        roll,pitch,heading=quaternion_rpy(tr.transform.rotation)
                        poses.append((wall,translation.x,translation.y));last_pose=wall
                        body.append((wall,translation.x,translation.y,translation.z,roll,pitch,heading))
                        pose=(wall,translation.x,translation.y,heading)
            elif name.endswith('/_action/status'):
                successes.update(bytes(v.goal_info.goal_id.uuid).hex() for v in msg.status_list if v.status==4)
            elif name==t['map']:areas.append((wall,sum(v>=0 for v in msg.data)*msg.info.resolution**2))
            elif name==t['tf']:
                for tr in msg.transforms:
                    transforms[(tr.header.frame_id.lstrip('/'),tr.child_frame_id.lstrip('/'))]=(wall,
                        tr.transform.translation.x,tr.transform.translation.y,quaternion_rpy(tr.transform.rotation)[2])
            elif name==t['tf_static']:
                for tr in msg.transforms:
                    if tr.child_frame_id=='velodyne' and tr.header.frame_id==profile['frames']['base']:
                        sensor=(tr.transform.translation.x,tr.transform.translation.y,quaternion_rpy(tr.transform.rotation)[2])
            elif name==t['scan']:
                if sensor is not None and pose is not None and 0<=wall-pose[0]<=.3:
                    hits=scan_wall_hits(scene,msg,pose[1:],sensor)
                    hit_frames=[count+int(hits[i]>=3) for i,count in enumerate(hit_frames)]
            elif name==t['command']:operator_commands.append(msg.data.lower())
            elif name=='/exploration/completion':completions.append(json.loads(msg.data))
            elif name=='/g3/exploration_result':exploration_results.append(json.loads(msg.data))
            elif name=='/g3/return_event':
                e=json.loads(msg.data)
                if e['status']=='accepted':
                    accepted.add((e['attempt_id'],e['goal_id']));return_triggers.append(e.get('trigger'))
                if e['status']=='succeeded':returned.append((e['attempt_id'],e['goal_id']))
            else:
                e=json.loads(msg.data)
                frontier_events.append(dict(e,recorded_time=wall))
                if e['status']=='brain_selected':
                    if e.get('fallback') is False:brain_selected+=1
                    else:brain_fallback+=1
                if e['status']=='succeeded' and e.get('geodesic_dist_m',0)>=.75:goals.add(e['attempt_id'])
    motion=[c for c in commands if abs(c[1])+abs(c[2])>1e-6]
    allowed=all(abs(c[1])<=profile['guard']['linear_max']+1e-8 and abs(c[2])<=profile['guard']['angular_max']+1e-8 and c[1]>=0 for c in commands)
    returned_ok=len(returned)==1 and returned[0] in accepted and returned[0][1] in successes
    home_reference=compose_pose(alignment,(profile['mission']['home_x'],profile['mission']['home_y'],profile['mission']['home_yaw'])) if alignment is not None else None
    home_error=math.dist(home_reference[:2],poses[-1][1:]) if poses and home_reference is not None else math.inf
    checks=dict(main_passed=report.get('success') is True,runtime_exited_cleanly=runtime_exit_ok(exit_info,expected_runtime_roles(profile)) and report.get('runtime_exit_code')==0,
       same_profile_hash=bool(status) and all(s['profile_sha256']==profile_hash(profile)==manifest['profile_sha256'] for s in status),
       sole_guard_publisher=report.get('actuator_publishers')==['velocity_guard'] and not any(s['publisher_conflict'] for s in status),
       no_truth_in_runtime=manifest.get('truth_used_by_runtime') is False and report.get('observer_publishes_velocity') is False,
       actual_guarded_motion=bool(motion) and bool(poses) and max(math.dist(p[1:],poses[0][1:]) for p in poses)>=.5,
       command_limits=bool(commands) and allowed,completed_frontiers=len(goals)>=2,
       real_return_action=returned_ok,home_reference_registered=home_reference is not None,body_returned_home=home_error<=.4)
    static_result=None
    if scene:
        mission_start=min((wall for wall,state in states if state=='exploring'),default=math.inf)
        mission_end=max((wall for wall,state in states if state=='standby' and wall>mission_start),
                        default=min((wall for wall,state in states if state=='estop' and wall>mission_start),
                                    default=max((p[0] for p in body),default=-math.inf)))
        mission_body=[p for p in body if mission_start<=p[0]<=mission_end]
        static_result=evaluate_scene(scene,mission_body,frontier_events,exploration_results,robot_radius=profile['navigation']['robot_radius_m'],world_from_map=alignment)
        static_result['wall_hit_frames']=hit_frames
        static_result['world_from_map']=alignment
        static_result['checks']['static_frame_alignment']=alignment is not None
        required_walls={'narrow':[0,1],'dead_end':[7],'unreachable':[4,5]}[scene['scenario']]
        static_result['checks']['static_real_lidar_geometry']=all(hit_frames[i]>=3 for i in required_walls)
        checks.update(static_result['checks'])
        checks['static_fixture_identity']=report.get('static_scenario')==scene['scenario']
        checks['completed_frontiers']=len(goals)>=scene['min_valid_goals']
        if scene['scenario']=='unreachable':
            del checks['actual_guarded_motion']
            # Local sensing inside the reachable room is allowed; crossing or
            # targeting the room beyond the slit is independently forbidden.
    checks['single_goal_pipeline']={c['name'] for c in manifest.get('children',[])}==expected_runtime_roles(profile) and manifest.get('goal_source')=='g2_frontier'
    checks['one_effective_configuration']=yaml.safe_load((root/'runtime_ros_params.yaml').read_text())==ros_parameters(profile,root) and manifest.get('candidate_selector')==profile['decision']['candidate_selector']
    if profile['decision']['candidate_selector']=='deepseek':
        if not (scene and scene['scenario']=='unreachable' and static_result['frontier_attempts']==0):
            checks['real_deepseek_selection']=brain_selected>0
    if report.get('autonomous_completion'):
        checks['autonomous_termination'] = autonomous_termination_ok(exploration_results,completions,operator_commands,return_triggers)
    # Verify zero outputs after final STANDBY (then ESTOP during cleanup).
    ends=[wall for wall,state in states if state=='standby' and returned and any(begin<wall for begin,value in states if value=='returning')] if commands else []
    quiet=[c for c in commands if ends and c[0]>=ends[0]+.2]
    checks['final_standby_zero']=bool(quiet) and all(abs(c[1])+abs(c[2])<1e-8 for c in quiet)
    result=dict(success=all(checks.values()),checks=checks,home_error_m=home_error if math.isfinite(home_error) else None,completed_frontiers=len(goals),home_reference=home_reference,home_reference_method='configured_map_home_at_exploration_start',
                actuator_command_samples=len(commands),moving_command_samples=len(motion),profile_sha256=profile_hash(profile),
                candidate_selector=profile['decision']['candidate_selector'],deepseek_selections=brain_selected,brain_fallback_selections=brain_fallback,
                static_scene=static_result,known_area_growth_m2=areas[-1][1]-areas[0][1] if areas else None,
                exploration_results=exploration_results,operator_commands=operator_commands,return_triggers=return_triggers,
                return_goal_ids=[goal for attempt,goal in returned],scope=('static '+scene['scenario'] if scene else 'ordinary Gazebo warehouse')+'; standalone runtime without evaluator velocity publisher')
    (root/'runtime_validation.json').write_text(json.dumps(result,indent=2));return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('output_dir',type=Path);a=p.parse_args()
    result=evaluate(a.output_dir);print(json.dumps(result,indent=2));return 0 if result['success'] else 1


if __name__=='__main__':raise SystemExit(main())
