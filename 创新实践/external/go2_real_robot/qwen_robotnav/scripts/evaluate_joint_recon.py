"""Combine independent identity, motion, obstacle and G3 return acceptance."""
import argparse
import json
import math
from pathlib import Path
from evaluate_dynamic_obstacle import evaluate


def combine(report,moving,recovery,dynamic,person,cart,displacement,overlap):
    reacquired={t['name']:sum(o['truth_name']==t['name'] and o['stamp_ns']>cart.get('end_stamp_ns',0)
                            for o in moving.get('audits',[])) for t in moving.get('target_audits',[])}
    motion_ok=bool(person.get('wait_for_robot_motion') and person.get('success') and displacement>=.5 and overlap>=10.)
    success=bool(report.get('success') and moving.get('success') and recovery.get('success') and dynamic['success']
                 and motion_ok and len(reacquired)==2 and all(n>=2 for n in reacquired.values()))
    return dict(success=success,simultaneous_motion_success=motion_ok,robot_displacement_during_people_motion_m=displacement,
        people_cart_overlap_sim_s=overlap,post_cart_identity_captures=reacquired,identity_success=moving.get('identity_validation_success'),
        dynamic_success=dynamic['success'],return_success=recovery.get('success'),
        scope='real rendered RGB-D/lidar; robot and two rigid people move concurrently with crossing cart; no hardware or walking animation')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path);a=p.parse_args();root=a.directory
    def read(name):
        path=root/name
        return json.loads(path.read_text()) if path.is_file() else {}
    report=read('report.json')
    if not report.get('joint_recon'):p.error('Require joint recon trial')
    dynamic=evaluate(root,joint=True);moving=read('moving_validation.json');recovery=read('return_recovery_validation.json')
    person=read('person_motion.json');cart=read('cart_motion.json')
    # Require actual robot displacement, not just nonzero motor commands.
    import sqlite3
    from rclpy.serialization import deserialize_message
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage
    db=sqlite3.connect('file:'+str(next((root/'bag').glob('*.db3')))+'?mode=ro',uri=True)
    rows=dict(db.execute('SELECT name,id FROM topics'));samples=[];sim=None;last=-1.
    try:
        for row_id,ident in db.execute('SELECT id,topic_id FROM messages WHERE topic_id IN (?,?) ORDER BY timestamp',(rows['/clock'],rows['/frontier_trial/gt_pose'])):
            if ident==rows['/clock']:
                data=db.execute('SELECT data FROM messages WHERE id=?',(row_id,)).fetchone()[0]
                c=deserialize_message(data,Clock).clock;sim=c.sec*10**9+c.nanosec
            elif sim is not None and sim-last>=100_000_000:
                last=sim
                if person.get('start_stamp_ns',1)<=sim<=person.get('end_stamp_ns',0):
                    data=db.execute('SELECT data FROM messages WHERE id=?',(row_id,)).fetchone()[0]
                    for tr in deserialize_message(data,TFMessage).transforms:
                        if tr.child_frame_id=='robot1_my_bot':samples.append((sim,tr.transform.translation.x,tr.transform.translation.y))
    finally:db.close()
    displacement=max((math.hypot(x-a,y-b) for _,x,y in samples for _,a,b in samples),default=0.)
    overlap=max(0,min(person.get('end_stamp_ns',0),cart.get('end_stamp_ns',0))-max(person.get('start_stamp_ns',0),cart.get('start_stamp_ns',0)))*1e-9
    result=combine(report,moving,recovery,dynamic,person,cart,displacement,overlap)
    result['appearance_variants']=[t.get('appearance_variant','source_clone') for t in read('scene_manifest.json').get('targets',[])]
    (root/'joint_validation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
    return 0 if result['success'] else 1


if __name__=='__main__':raise SystemExit(main())
