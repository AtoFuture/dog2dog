"""Closed-bag audit of real sensor loss, stopping, G3 resume and timeout latch."""
import argparse
import json
import math
from pathlib import Path
import sqlite3


def extent(rows):
    return max((math.dist(rows[0][-2:],p[-2:]) for p in rows),default=0.) if rows else 0.


def audit_episode(start,release,states,recovery,commands,poses,raw_scan,scan,raw_tf,tf,next_start,mode):
    begin,end=start['wall'],release['wall']
    pauses=[e for e in recovery if e['status']=='paused' and begin<=e['wall']<=end]
    released=[e for e in recovery if e['status']=='navigation_released' and begin<=e['wall']<=end]
    resumes=[e for e in recovery if e['status']=='resumed' and end<=e['wall']<next_start]
    paused=pauses[0]['wall'] if pauses else end
    stopped=[c for c in commands if begin<=c[0]<=end]
    moving=[c for c in stopped if abs(c[2])+abs(c[3])>1e-6]
    last_motion=moving[-1][0] if moving else begin
    zeros=[c for c in stopped if c[0]>=last_motion and abs(c[2])+abs(c[3])<=1e-6]
    delay=zeros[0][0]-begin if zeros else None
    stop_displacement=extent([p for p in poses if begin-.1<=p[0]<=zeros[0][0]+.75]) if zeros else None
    source=start['source'];raw=raw_scan if source=='scan' else raw_tf;delivered=scan if source=='scan' else tf
    raw_count=sum(begin+.25<t<end-.1 for t in raw)
    leaked=sum(begin+.25<t<end-.1 for t in delivered)
    quiet_end=resumes[0]['wall'] if mode=='short' and resumes else end
    quiet=[c for c in commands if paused+.75<=c[0]<=quiet_end]
    drift=extent([p for p in poses if paused+.75<=p[0]<=quiet_end])
    pre_motion=extent([p for p in poses if begin-3<=p[0]<=begin])
    reasons=[e for e in recovery if e['status']=='fault' and begin<=e['wall']<=end]
    checks=dict(actual_motion_before_fault=pre_motion>=.01,source_was_cut=raw_count>=3 and leaked==0,
        correct_fault_detected=any(source in e.get('missing',[]) for e in reasons),
        paused=bool(pauses),navigation_cancel_terminal=bool(released),
        stop_delay_bounded=delay is not None and delay<=3.5,
        paused_commands_zero=bool(quiet) and all(abs(c[2])+abs(c[3])<=1e-6 for c in quiet),
        paused_body_drift_bounded=drift<=.15)
    recovered_motion=None
    if mode=='short':
        resumed=resumes[0]['wall'] if resumes else next_start
        recovered_motion=extent([p for p in poses if resumed<=p[0]<next_start])
        checks.update(resumed_after_stable_health=bool(resumes) and 2.5<=resumed-end<=12,
                      moved_after_resume=recovered_motion>=.08)
    else:
        estops=[t for t,state in states if state=='estop' and paused<=t<=end]
        estop=estops[0] if estops else end
        post=[c for c in commands if c[0]>=estop+.75]
        checks.update(timeout_latched=bool(estops) and 59<=estop-paused<=63,
            no_restart_after_restore=not resumes and not any(state=='exploring' and t>estop for t,state in states),
            no_commands_after_estop=bool(post) and all(abs(c[2])+abs(c[3])<=1e-6 for c in post),
            actual_source_restored=sum(end+.25<t for t in delivered)>=3,
            restored_observation_window=bool(poses) and poses[-1][0]-end>=7)
    return dict(source=source,success=all(checks.values()),checks=checks,stop_delay_wall_s=delay,
        paused_body_drift_m=drift,stop_body_displacement_m=stop_displacement,pre_fault_body_motion_m=pre_motion,post_resume_body_motion_m=recovered_motion,
        raw_messages_during_cut=raw_count,delivered_messages_during_cut=leaked,
        recovery_delay_wall_s=(resumes[0]['wall']-end) if resumes else None)


def released_action(statuses,begin,deadline,paused=None):
    latest={goal:status for stamp,goal,status in statuses if stamp<=begin}
    active={goal for goal,status in latest.items() if status in (1,2,3)}
    if paused is not None:
        active.update(goal for stamp,goal,status in statuses if begin<=stamp<=paused and status in (1,2,3))
    return bool(len(active)==1 and all(any(goal==g and status in (4,5,6) and begin<=stamp<deadline
                    for stamp,g,status in statuses) for goal in active))


def verified_return(events,action_success,poses):
    accepted={(e.get('attempt_id'),e.get('goal_id')) for e in events
              if e.get('status')=='accepted' and e.get('goal_id')}
    succeeded=[(e.get('attempt_id'),e.get('goal_id')) for e in events if e.get('status')=='succeeded']
    return bool(len(succeeded)==1 and succeeded[0] in accepted and succeeded[0][1] in action_success
                and len(poses)>1 and math.dist(poses[0][-2:],poses[-1][-2:])<=.4)


def plot_timeline(root,starts,faults,recovery,commands,poses):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    if not starts:return
    fig,axes=plt.subplots(len(starts),2,figsize=(12,3.4*len(starts)),squeeze=False)
    for row,start in enumerate(starts):
        begin=start['wall']
        release=next((e['wall'] for e in faults if e['status']=='drop_released' and e['source']==start['source'] and e['wall']>begin),begin+7)
        upper=min(release+9,starts[row+1]['wall'] if row+1<len(starts) else float('inf'))
        c=[p for p in commands if begin-3<=p[0]<=upper]
        p=[p for p in poses if begin-3<=p[0]<=upper]
        left,right=axes[row]
        left.plot([v[0]-begin for v in c],[v[2] for v in c],label='linear command',color='tab:blue')
        turn=left.twinx();turn.plot([v[0]-begin for v in c],[v[3] for v in c],color='tab:orange',alpha=.5)
        turn.set_ylabel('angular command (rad/s)',color='tab:orange')
        left.set_ylabel('gait linear command');left.set_title(start['source']+' command output')
        if p:right.plot([v[0]-begin for v in p],[math.dist(p[0][-2:],v[-2:]) for v in p],color='tab:purple')
        right.set_ylabel('body displacement (m)');right.set_title(start['source']+' simulator body truth')
        for ax in (left,right):
            ax.axvspan(0,release-begin,color='tab:red',alpha=.10)
            for e in recovery:
                if e['status'] in ('paused','resumed','recovery_timeout') and begin<=e['wall']<=upper:
                    ax.axvline(e['wall']-begin,linestyle='--',color='tab:green' if e['status']=='resumed' else 'tab:red',alpha=.7)
            ax.set_xlabel('wall seconds from dropout');ax.grid(alpha=.2)
    fig.tight_layout();fig.savefig(Path(root)/'fault_timeline.png',dpi=150);plt.close(fig)


def evaluate(root):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    root=Path(root);report=json.loads((root/'report.json').read_text())
    if report.get('recorder_exit_code')!=0 or not (root/'bag/metadata.yaml').is_file():
        raise ValueError('Require recorder exit 0 and finalized bag metadata')
    mode=report.get('fault_recovery_mode')
    if mode not in ('short','long'):raise ValueError('Report must declare short or long fault recovery mode')
    dbs=list((root/'bag').glob('*.db3'))
    if len(dbs)!=1:raise ValueError('Expected one closed SQLite bag')
    faults=[];recovery=[];states=[];commands=[];poses=[];raw_scan=[];scan=[];raw_tf=[];tf=[];explore=[];returns=[]
    action_success=set();action_status=[]
    status_topic='/robot1/navigate_to_pose/_action/status'
    wanted={status_topic,'/frontier_trial/recovery_fault','/g3/recovery_event','/mission_state','/robot1/cmd_vel',
        '/frontier_trial/gt_pose','/robot1/velodyne','/frontier_trial/recovery_scan','/robot1/tf',
        '/frontier_trial/recovery_tf','/exploration/events','/g3/return_event'}
    with sqlite3.connect(f'file:{dbs[0]}?mode=ro',uri=True) as db:
        topics={i:(n,get_message(t)) for i,n,t in db.execute('SELECT id,name,type FROM topics') if n in wanted}
        ids=','.join(str(i) for i in topics)
        last_pose=-math.inf
        for topic,stamp,blob in db.execute(f'SELECT topic_id,timestamp,data FROM messages WHERE topic_id IN ({ids}) ORDER BY timestamp,id'):
            name,typ=topics[topic];wall=stamp*1e-9
            if name in ('/robot1/velodyne','/frontier_trial/recovery_scan'):
                (raw_scan if name=='/robot1/velodyne' else scan).append(wall);continue
            if name=='/frontier_trial/gt_pose' and wall-last_pose<.05:continue
            msg=deserialize_message(blob,typ)
            if name in ('/robot1/tf','/frontier_trial/recovery_tf'):
                if any(t.header.frame_id.strip('/')=='map' and t.child_frame_id.strip('/')=='odom' for t in msg.transforms):
                    (raw_tf if name=='/robot1/tf' else tf).append(wall)
            elif name==status_topic:
                action_success.update(bytes(v.goal_info.goal_id.uuid).hex() for v in msg.status_list if v.status==4)
                action_status.extend((wall,bytes(v.goal_info.goal_id.uuid).hex(),v.status) for v in msg.status_list)
            elif name=='/robot1/cmd_vel':commands.append((wall,0,msg.linear.x,msg.angular.z))
            elif name=='/frontier_trial/gt_pose':
                for t in msg.transforms:
                    if t.child_frame_id=='robot1_my_bot':
                        poses.append((wall,t.transform.translation.x,t.transform.translation.y));last_pose=wall
            elif name=='/mission_state':states.append((wall,msg.data))
            else:
                e=json.loads(msg.data);e['wall']=wall
                {'/frontier_trial/recovery_fault':faults,'/g3/recovery_event':recovery,
                 '/exploration/events':explore,'/g3/return_event':returns}[name].append(e)
    starts=[e for e in faults if e['status']=='drop_started'];results=[]
    progress=True
    for i,start in enumerate(starts):
        releases=[e for e in faults if e['status']=='drop_released' and e['source']==start['source'] and e['wall']>start['wall']]
        if not releases:continue
        next_start=starts[i+1]['wall'] if i+1<len(starts) else (poses[-1][0] if poses else releases[0]['wall'])
        episode=audit_episode(start,releases[0],states,recovery,commands,poses,raw_scan,scan,raw_tf,tf,next_start,mode)
        resumed=[e['wall'] for e in recovery if e['status']=='resumed' and releases[0]['wall']<=e['wall']<next_start]
        deadline=resumed[0] if resumed else releases[0]['wall']
        pauses=[e['wall'] for e in recovery if e['status']=='paused' and start['wall']<=e['wall']<=releases[0]['wall']]
        episode['checks']['old_action_terminal_before_resume']=released_action(action_status,start['wall'],deadline,pauses[0] if pauses else None)
        episode['success']=all(episode['checks'].values());results.append(episode)
        completed={e['attempt_id'] for e in explore if e['status']=='succeeded' and e.get('geodesic_dist_m',0)>=.75 and e['wall']<start['wall']}
        progress &= completed==set(start.get('completed_before',[]))
        resumes=[e for e in faults if e['status']=='resume_verified' and e['source']==start['source']]
        if mode=='short':progress &= bool(completed) and bool(resumes) and set(resumes[0].get('completed_after',[]))==completed
    checks=dict(launcher_mission_passed=report.get('success') is True,
        all_expected_faults=len(results)==(2 if mode=='short' else 1),
        all_episodes_passed=bool(results) and all(e['success'] for e in results),task_progress_preserved=bool(progress))
    if mode=='short':
        starts_after=[e['wall'] for e in recovery if e['status']=='resumed']
        checks['new_goal_completed_after_resume']=bool(starts_after) and any(e['status']=='succeeded' and e.get('geodesic_dist_m',0)>=.75 and e['wall']>max(starts_after) for e in explore)
        checks['returned_home']=verified_return(returns,action_success,poses)
    result=dict(success=all(checks.values()),mode=mode,checks=checks,episodes=results,
                scope='actual Gazebo movement, G2 frontier and G3 recovery; no person/identity or hardware acceptance')
    plot_timeline(root,starts,faults,recovery,commands,poses)
    (root/'fault_validation.json').write_text(json.dumps(result,indent=2));return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('output_dir',type=Path);a=p.parse_args()
    result=evaluate(a.output_dir);print(json.dumps(result,indent=2));return 0 if result['success'] else 1


if __name__=='__main__':raise SystemExit(main())
