"""Drive only manifest-declared people in an isolated Gazebo scene, on sim time."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import rclpy
from rosgraph_msgs.msg import Clock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from qwen_robotnav.person_motion import positions,DURATION_S


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    a=parser.parse_args();root=a.directory.resolve()
    if not os.environ.get('IGN_PARTITION') or int(os.environ.get('ROS_DOMAIN_ID','0')) in (0,42,43):
        parser.error('An isolated simulator partition/domain is required')
    scene=json.loads((root/'scene_manifest.json').read_text());scenario=scene['scenario']
    names=[t['name'] for t in scene['targets']]
    if names!=['recon_person_1','recon_person_2'] or not scene.get('moving_people'):
        parser.error('Only generated moving-person scenes are allowed')
    service='/usr/libexec/gz/transport11/ign-transport-service'
    rclpy.init();node=rclpy.create_node('recon_person_motion_driver')
    clock=[None];node.create_subscription(Clock,'/clock',lambda m:clock.__setitem__(0,m.clock.sec+m.clock.nanosec*1e-9),10)
    result=dict(success=False,scenario=scenario,duration_sim_s=DURATION_S,requests=0,service_retries=0,
                purpose='evaluation-only model motion; not an identity/navigation input')
    started=None;last=None;deadline=time.monotonic()+220
    log=(root/'person_motion_commands.jsonl').open('w')
    try:
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            if clock[0] is None:continue
            if started is None:started=clock[0];result['start_stamp_ns']=int(started*1e9)
            elapsed=clock[0]-started
            if last is not None and elapsed-last<.2:continue
            for name,(x,y) in zip(names,positions(scenario,elapsed)):
                request=f'name: "{name}" position {{ x: {x} y: {y} z: 0 }} orientation {{ z: {-math.sqrt(.5)} w: {math.sqrt(.5)} }}'
                for attempt in range(3):
                    try:
                        response=subprocess.run([service,'-s','/world/world_demo/set_pose','--reqtype','ignition.msgs.Pose',
                            '--reptype','ignition.msgs.Boolean','--timeout','5000','--req',request],capture_output=True,text=True,timeout=7)
                        if response.returncode==0 and 'data: true' in response.stdout:break
                        error=response.stdout+response.stderr
                    except subprocess.TimeoutExpired:
                        error='Pose service process timeout'
                    result['service_retries']+=1
                else:raise RuntimeError('Scoped person pose service failed after 3 idempotent attempts: '+error)
                result['requests']+=1
                log.write(json.dumps(dict(stamp_ns=int(clock[0]*1e9),name=name,x=x,y=y))+'\n');log.flush()
            last=elapsed
            if elapsed>=DURATION_S:
                result['success']=True;result['end_stamp_ns']=int(clock[0]*1e9);break
        if not result['success']:raise RuntimeError('Moving-person sim-time deadline expired')
    except Exception as exc:result['error']=str(exc)
    finally:
        log.close();node.destroy_node();rclpy.shutdown()
        (root/'person_motion.json').write_text(json.dumps(result,indent=2))
    return 0 if result['success'] else 1


if __name__=='__main__':raise SystemExit(main())
