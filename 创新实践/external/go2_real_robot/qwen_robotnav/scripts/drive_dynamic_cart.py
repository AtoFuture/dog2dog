"""Evaluation stimulus only: move one manifest-declared cart, never robot commands.

Robot truth sets up a repeatable encounter; Nav2 receives only real lidar/odometry.
The cart crosses a fixed world line, rather than chasing the robot.
"""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import time
import rclpy
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def lateral(scenario, elapsed):
    if scenario=='crossing':return min(1.8,-1.5+.30*elapsed)
    if scenario=='blocked':return 0. if elapsed<8. else min(2.4,.4*(elapsed-8.))
    raise ValueError('Unknown dynamic scenario')


def set_cart_pose(request, *, before_attempt, on_retry, run=subprocess.run):
    """Bounded idempotent retries; refresh/check separation before each request."""
    error='no response'
    for attempt in range(3):
        before_attempt(attempt)
        if attempt:on_retry()
        try:
            response=run(['/usr/libexec/gz/transport11/ign-transport-service','-s','/world/world_demo/set_pose',
                '--reqtype','ignition.msgs.Pose','--reptype','ignition.msgs.Boolean','--timeout','3000','--req',request],
                capture_output=True,text=True,timeout=5)
            if response.returncode==0 and 'data: true' in response.stdout:return
            error=f'exit={response.returncode}: '+response.stdout+response.stderr
        except subprocess.TimeoutExpired:error='Pose service process timeout'
    raise RuntimeError('Scoped cart pose service failed after 3 attempts: '+error[:300])


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path)
    a=p.parse_args();root=a.directory.resolve()
    if not os.environ.get('IGN_PARTITION') or int(os.environ.get('ROS_DOMAIN_ID','0')) in (0,42,43):
        p.error('Require an isolated simulator domain and partition')
    scene=json.loads((root/'dynamic_scene.json').read_text())
    if scene['model']!='trial_dynamic_cart' or not scene['evaluation_only']:p.error('Require a generated cart scene')
    distance=scene.get('encounter_distance_m',1.5);duration=scene.get('duration_sim_s',15.)
    if not 1.<=distance<=1.5 or not 10.<=duration<=15.:p.error('Invalid fixture distance/duration')
    rclpy.init();node=rclpy.create_node('dynamic_cart_evaluation_driver')
    state=dict(clock=None,pose=None,pose_at=-math.inf,forward=False)
    def truth(m):
        for tr in m.transforms:
            if tr.child_frame_id=='robot1_my_bot':
                v=tr.transform.translation;q=tr.transform.rotation
                state['pose']=(v.x,v.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)));state['pose_at']=time.monotonic()
    node.create_subscription(Clock,'/clock',lambda m:state.update(clock=m.clock.sec+m.clock.nanosec*1e-9),10)
    node.create_subscription(TFMessage,'/frontier_trial/gt_pose',truth,10)
    node.create_subscription(Twist,'/robot1/cmd_vel',lambda m:state.update(forward=m.linear.x>.01),10)
    pub=node.create_publisher(String,'/frontier_trial/cart_events',10)
    result=dict(success=False,scenario=scene['scenario'],evaluation_only=True,requests=0,service_retries=0)
    start=None;anchor=None;last_clock=None;phase=0.;deadline=time.monotonic()+240
    stream=(root/'cart_commands.jsonl').open('w')
    try:
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            if state['clock'] is None or state['pose'] is None:continue
            if start is None:
                if not state['forward']:continue
                start=state['clock'];anchor=state['pose'];result.update(start_stamp_ns=int(start*1e9),anchor_world=anchor)
            if last_clock is not None and state['clock']-last_clock<.2:continue
            # A slow/lost RPC pauses the fixture instead of teleporting it to
            # catch up: each acknowledged step remains at most 0.2 s of motion.
            if last_clock is not None:phase+=min(.2,state['clock']-last_clock)
            t=phase
            offset=lateral(scene['scenario'],t);ax,ay,heading=anchor
            x=ax+distance*math.cos(heading)-offset*math.sin(heading)
            y=ay+distance*math.sin(heading)+offset*math.cos(heading)
            # Never teleport into the robot. Fail the trial if the test fixture
            # itself becomes unsafe; zero robot motion remains the harness's job.
            rx,ry,_=state['pose']
            dx=(rx-x)*math.cos(heading)+(ry-y)*math.sin(heading)
            dy=-(rx-x)*math.sin(heading)+(ry-y)*math.cos(heading)
            separation=math.hypot(max(0,abs(dx)-scene['size_m'][0]/2),max(0,abs(dy)-scene['size_m'][1]/2))
            if separation<.38:raise RuntimeError('Cart fixture separation guard triggered')
            request=f'name: "trial_dynamic_cart" position {{ x: {x} y: {y} z: .4 }} orientation {{ z: {math.sin(heading/2)} w: {math.cos(heading/2)} }}'
            current_separation=[separation]
            def check_placement(attempt):
                # A failed RPC can leave either an applied pose or an unapplied
                # request. Refresh evaluation truth before retrying the same pose.
                for _ in range(32):rclpy.spin_once(node,timeout_sec=0.)
                if time.monotonic()-state['pose_at']>.5:raise RuntimeError('Stale robot pose before cart placement')
                rx,ry,_=state['pose'];dx=(rx-x)*math.cos(heading)+(ry-y)*math.sin(heading)
                dy=-(rx-x)*math.sin(heading)+(ry-y)*math.cos(heading)
                current_separation[0]=math.hypot(max(0,abs(dx)-scene['size_m'][0]/2),max(0,abs(dy)-scene['size_m'][1]/2))
                if current_separation[0]<.38:raise RuntimeError('Cart fixture separation guard triggered before RPC')
            def retried():result['service_retries']+=1
            set_cart_pose(request,before_attempt=check_placement,on_retry=retried)
            separation=current_separation[0]
            event=dict(stamp_ns=int(state['clock']*1e9),elapsed_sim_s=state['clock']-start,trajectory_time_sim_s=t,x=x,y=y,yaw=heading,
                       separation_m=separation,status='cleared' if t>=duration else 'moving')
            pub.publish(String(data=json.dumps(event)));stream.write(json.dumps(event)+'\n');stream.flush()
            result['requests']+=1;last_clock=state['clock']
            if t>=duration:
                result.update(success=True,end_stamp_ns=event['stamp_ns']);break
        if not result['success']:raise RuntimeError('Cart encounter did not complete before deadline')
    except Exception as exc:result['error']=str(exc)
    finally:
        stream.close();(root/'cart_motion.json').write_text(json.dumps(result,indent=2))
        node.destroy_node();rclpy.shutdown()
    return 0 if result['success'] else 1


if __name__=='__main__':raise SystemExit(main())
