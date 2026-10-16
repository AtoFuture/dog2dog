"""Audit a closed simulator run, including optional zero-command RETURN hold."""
import argparse
import json
import math
from pathlib import Path
import re
import sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path)
    args=parser.parse_args();root=args.directory.resolve()
    report=json.loads((root/'report.json').read_text())
    if not (root/'bag/metadata.yaml').is_file() or report.get('recorder_exit_code')!=0:
        parser.error('Require a finalized recording and successful recorder exit')
    mission=json.loads((root/'mission_report.json').read_text())
    db=sqlite3.connect('file:'+str(next((root/'bag').glob('*.db3')))+'?mode=ro',uri=True)
    names=['/frontier_trial/fault_events','/robot1/cmd_vel','/g3/return_event',
           '/robot1/navigate_to_pose/_action/status','/robot1/navigate_to_pose/_action/feedback','/mission/recon_status']
    types={name:get_message(kind) for name,kind in db.execute('SELECT name,type FROM topics') if name in names}
    faults=[];active=False;held=0;nonzero=0;receipt_nonzero=0;return_goals=set();successful=set();terminal=False
    recovery_counts=[];succeeded_return_events=[]
    for topic,data in db.execute('SELECT topics.name,messages.data FROM messages JOIN topics ON topics.id=messages.topic_id WHERE topics.name IN ('+','.join('?' for _ in names)+') ORDER BY messages.timestamp',names):
        msg=deserialize_message(data,types[topic])
        if topic=='/frontier_trial/fault_events':
            event=json.loads(msg.data);faults.append(event)
            if event['status']=='return_hold_started':active=True
            if event['status']=='return_hold_released':active=False
        elif topic=='/robot1/cmd_vel':
            if active:
                held+=1
                if any(abs(v)>1e-9 for v in (msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z)):receipt_nonzero+=1
        elif topic=='/g3/return_event':
            event=json.loads(msg.data)
            if event.get('attempt_id')==mission['return_attempt_id']:
                if event['status']=='accepted':return_goals.add(event['goal_id'])
                if event['status']=='succeeded':succeeded_return_events.append(event.get('goal_id'))
        elif topic.endswith('/status'):
            for status in msg.status_list:
                if status.status==4:successful.add(bytes(status.goal_info.goal_id.uuid).hex())
        elif topic.endswith('/feedback'):
            if bytes(msg.goal_id.uuid).hex() in return_goals:recovery_counts.append(msg.feedback.number_of_recoveries)
        else:
            value=json.loads(msg.data)
            if value.get('return_attempt_id')==mission['return_attempt_id'] and value.get('state')=='returned':terminal=True
    log=(root/'nav2.log').read_text(errors='replace')
    starts=[e for e in faults if e['status']=='return_hold_started'];ends=[e for e in faults if e['status']=='return_hold_released']
    # Topics can arrive out of order at release. Use the fault's simulation-time
    # interval [start,end), not String/Twist receipt ordering. The start wall
    # timestamp excludes commands received before the producer entered the hold
    # in the same simulation tick. Retain both metrics.
    held=0;nonzero=0;boundary_excluded=0
    if len(starts)==len(ends)==1:
        clock_row=db.execute("SELECT id FROM topics WHERE name='/clock'").fetchone()
        command_row=db.execute("SELECT id FROM topics WHERE name='/robot1/cmd_vel'").fetchone()
        if clock_row and command_row:
            lo=int(starts[0]['wall_time_s']*1e9);hi=int((ends[0]['wall_time_s']+.1)*1e9)
            for stamp,data in db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? AND timestamp BETWEEN ? AND ? ORDER BY timestamp',(command_row[0],lo,hi)):
                clock=db.execute('SELECT data FROM messages WHERE topic_id=? AND timestamp<=? ORDER BY timestamp DESC LIMIT 1',(clock_row[0],stamp)).fetchone()
                if clock is None:continue
                value=deserialize_message(clock[0],Clock).clock;sim_ns=value.sec*10**9+value.nanosec
                msg=deserialize_message(data,Twist)
                moving=any(abs(v)>1e-9 for v in (msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z))
                if starts[0]['stamp_ns']<=sim_ns<ends[0]['stamp_ns']:
                    held+=1;nonzero+=int(moving)
                elif moving:boundary_excluded+=1
    db.close()
    during=[]
    for line in log.splitlines():
        if 'Stable map progress stalled;' not in line:continue
        match=re.search(r'\[(\d+\.\d+)\]',line)
        if match and starts and ends and starts[0]['wall_time_s']<=float(match.group(1))<=ends[0]['wall_time_s']+1.:during.append(line)
    injection_ok=None
    if report.get('return_hold_s',0)>0:
        injection_ok=bool(len(starts)==len(ends)==1 and held>=50 and nonzero==0
            and (ends[0]['stamp_ns']-starts[0]['stamp_ns'])*1e-9>=report['return_hold_s']
            and starts[0].get('goal_id') in return_goals and during)
    return_ok=bool(len(return_goals)==1 and len(succeeded_return_events)==1
        and succeeded_return_events[0] in return_goals & successful and terminal
        and report.get('navigation_return_success') and report.get('truth_home_error_m',math.inf)<=.4)
    loaded='Stable map progress checker enabled' in log
    success=bool(return_ok and loaded and injection_ok is not False
        and not report['truth_in_navigation_tf'] and not report['static_map_loaded'])
    result=dict(success=success,return_success=return_ok,progress_plugin_loaded=loaded,
        hold_injection_success=injection_ok,fault_events=faults,held_commands=held,
        nonzero_commands_during_hold=nonzero,nonzero_commands_in_receipt_order=receipt_nonzero,
        boundary_motion_commands_excluded=boundary_excluded,
        hold_time_alignment="producer start wall time plus preceding /clock; half-open [start,end) simulation interval",stall_warnings_during_hold=len(during),
        stable_progress_failures=log.count('Stable map progress stalled;'),
        map_transform_failures=log.count('Map progress transform unavailable:'),
        return_goal_ids=sorted(return_goals),maximum_reported_recoveries=max(recovery_counts,default=0),
        truth_home_error_m=report.get('truth_home_error_m'),truth_used_only_for_evaluation=True,
        scope='isolated simulator; zero-command hold models unavailable actuation, not a physical collision or hardware stop')
    (root/'return_recovery_validation.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result),flush=True)
    return 0 if success else 1


if __name__=='__main__':raise SystemExit(main())
