"""Audit finalized dynamic trials. Never open a live recorder database."""
import argparse
import json
import math
from pathlib import Path
import sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def box_distance(x,y,cart,size):
    dx=x-cart['x'];dy=y-cart['y'];a=cart['yaw']
    u=math.cos(a)*dx+math.sin(a)*dy;v=-math.sin(a)*dx+math.cos(a)*dy
    return math.hypot(max(0,abs(u)-size[0]/2),max(0,abs(v)-size[1]/2))


def evaluate(root):
    root=Path(root);report=json.loads((root/'report.json').read_text())
    if not (root/'bag/metadata.yaml').is_file() or report.get('recorder_exit_code')!=0:
        raise ValueError('Require finalized metadata and recorder exit code zero')
    scene=json.loads((root/'dynamic_scene.json').read_text())
    motion=json.loads((root/'cart_motion.json').read_text()) if (root/'cart_motion.json').exists() else {}
    goals=json.loads((root/'dynamic_goals.json').read_text()) if (root/'dynamic_goals.json').exists() else []
    db=sqlite3.connect('file:'+str(next((root/'bag').glob('*.db3')))+'?mode=ro',uri=True)
    names=['/clock','/frontier_trial/cart_events','/frontier_trial/gt_pose','/robot1/cmd_vel',
           '/deepseek/nav_cmd','/robot1/velodyne','/robot1/tf_static','/robot1/local_costmap/costmap','/robot1/odometry/filtered',
           '/robot1/navigate_to_pose/_action/status']
    types={n:get_message(t) for n,t in db.execute('SELECT name,type FROM topics') if n in names}
    sim=0.;cart=None;pose=None;odom=None;minimum=math.inf;successful=set()
    commands=[];costs=[];trajectory=[];last_truth=-math.inf;laser=None;laser_hits=[];clearing=[]
    query='SELECT topics.name,messages.data FROM messages JOIN topics ON topics.id=messages.topic_id WHERE topics.name IN ('+','.join('?' for _ in names)+') ORDER BY messages.timestamp'
    try:
        for topic,data in db.execute(query,names):
            # Ground truth is high rate; sample BEFORE deserializing the large TF message.
            if topic=='/frontier_trial/gt_pose' and sim-last_truth<.05:continue
            msg=deserialize_message(data,types[topic])
            if topic=='/clock':sim=msg.clock.sec+msg.clock.nanosec*1e-9
            elif topic=='/frontier_trial/cart_events':cart=json.loads(msg.data)
            elif topic=='/frontier_trial/gt_pose':
                last_truth=sim
                for tr in msg.transforms:
                    if tr.child_frame_id=='robot1_my_bot':
                        p=tr.transform.translation;pose=(p.x,p.y,yaw(tr.transform.rotation),sim)
                        trajectory.append((sim,p.x,p.y))
                        if cart is not None:minimum=min(minimum,box_distance(p.x,p.y,cart,scene['size_m']))
            elif topic=='/robot1/odometry/filtered':
                p=msg.pose.pose;odom=(p.position.x,p.position.y,yaw(p.orientation),sim)
            elif topic=='/robot1/tf_static':
                for tr in msg.transforms:
                    if tr.child_frame_id=='velodyne' and tr.header.frame_id=='base_link':
                        laser=(tr.transform.translation.x,tr.transform.translation.y,yaw(tr.transform.rotation))
            elif topic=='/robot1/velodyne' and laser and pose and cart and sim-pose[3]<.3:
                hits=0
                for i,d in enumerate(msg.ranges):
                    if not math.isfinite(d) or not msg.range_min<d<msg.range_max:continue
                    a=msg.angle_min+i*msg.angle_increment+laser[2]
                    bx=laser[0]+d*math.cos(a);by=laser[1]+d*math.sin(a)
                    x=pose[0]+math.cos(pose[2])*bx-math.sin(pose[2])*by
                    y=pose[1]+math.sin(pose[2])*bx+math.cos(pose[2])*by
                    if box_distance(x,y,cart,scene['size_m'])<.08:hits+=1
                if hits>=3:laser_hits.append((sim,hits))
            elif topic.endswith('/status'):
                for status in msg.status_list:
                    ident=bytes(status.goal_info.goal_id.uuid).hex()
                    if status.status==4:successful.add(ident)
            elif topic in ('/robot1/cmd_vel','/deepseek/nav_cmd'):
                commands.append((sim,topic,msg.linear.x,msg.angular.z))
            elif topic=='/robot1/local_costmap/costmap' and msg.header.frame_id=='odom' and cart and pose and odom and max(sim-pose[3],sim-odom[3])<.3:
                # Evaluation-only world->odom registration at the robot pose.
                # Nav2 never receives this transform or cart coordinates.
                a=odom[2]-pose[2];dx=cart['x']-pose[0];dy=cart['y']-pose[1]
                x=odom[0]+math.cos(a)*dx-math.sin(a)*dy
                y=odom[1]+math.sin(a)*dx+math.cos(a)*dy
                r=msg.info.resolution;cx=math.floor((x-msg.info.origin.position.x)/r);cy=math.floor((y-msg.info.origin.position.y)/r)
                # Cart center lies inside its collision box; a small neighborhood
                # allows lidar to mark the visible face instead of its center.
                radius=math.ceil((max(scene['size_m'][:2])/2+.1)/r)
                cells=[msg.data[j*msg.info.width+i] for j in range(max(0,cy-radius),min(msg.info.height,cy+radius+1))
                       for i in range(max(0,cx-radius),min(msg.info.width,cx+radius+1))]
                if cells:costs.append((sim,max(cells)))
                if motion.get('anchor_world'):
                    ax,ay,heading=motion['anchor_world']
                    # Track the original visible face after the cart has left.
                    wx=ax+1.25*math.cos(heading);wy=ay+1.25*math.sin(heading)
                    dx=wx-pose[0];dy=wy-pose[1]
                    fx=odom[0]+math.cos(a)*dx-math.sin(a)*dy
                    fy=odom[1]+math.sin(a)*dx+math.cos(a)*dy
                    ix=math.floor((fx-msg.info.origin.position.x)/r);iy=math.floor((fy-msg.info.origin.position.y)/r)
                    patch=[msg.data[j*msg.info.width+i] for j in range(max(0,iy-1),min(msg.info.height,iy+2))
                           for i in range(max(0,ix-1),min(msg.info.width,ix+2))]
                    if len(patch)==9 and math.hypot(wx-pose[0],wy-pose[1])>.6:clearing.append((sim,max(patch)))
    finally:db.close()
    start=motion.get('start_stamp_ns',0)*1e-9;end=motion.get('end_stamp_ns',0)*1e-9
    nav_stops=[t for t,n,v,w in commands if n=='/deepseek/nav_cmd' and start+1<=t<=min(end,start+8) and abs(v)<.001 and abs(w)<.001]
    marked=[t for t,c in costs if start<=t<=end and c>=99]
    outbound_end=goals[0].get('end_stamp_ns',0)*1e-9 if goals else 0.
    resumed=[t for t,n,v,w in commands if n=='/robot1/cmd_vel' and end<t<=outbound_end and v>.01]
    original_marked=[t for t,c in clearing if start<=t<=end and c>=99]
    cleared=[t for t,c in clearing if t>end and c==0]
    hit_frames=[t for t,n in laser_hits if start<=t<=end]
    anchor=motion.get('anchor_world')
    lateral_excursion=max((abs(-(x-anchor[0])*math.sin(anchor[2])+(y-anchor[1])*math.cos(anchor[2]))
                           for t,x,y in trajectory if start<=t<=end),default=0.) if anchor else 0.
    actual_goals={g['goal_id'] for g in goals}
    outbound_error=None;home_error=None
    if goals and trajectory and report.get('truth_start'):
        origin=report['truth_start'];heading=origin[3]
        home_error=math.hypot(trajectory[-1][1]-origin[0],trajectory[-1][2]-origin[1])
        target=(origin[0]+2.4*math.cos(heading),origin[1]+2.4*math.sin(heading))
        goal_end=goals[0].get('end_stamp_ns',0)*1e-9
        sample=min(trajectory,key=lambda row:abs(row[0]-goal_end))
        if abs(sample[0]-goal_end)<=.25:
            outbound_error=math.hypot(sample[1]-target[0],sample[2]-target[1])
    nav_ok=home_error is not None and home_error<=.4 and outbound_error is not None and outbound_error<=.4 and len(goals)==2 and all(g.get('status')==4 for g in goals) and actual_goals<=successful
    safe=minimum>=.38 and report.get('emergency_guard_interventions',1)==0
    success=bool(report.get('success') and motion.get('success') and nav_ok and safe and marked and hit_frames and original_marked and cleared and (nav_stops or lateral_excursion>=.45) and resumed
                 and report.get('truth_in_navigation_tf') is False)
    result=dict(success=success,scenario=scene['scenario'],navigation_success=nav_ok,cart_motion_success=bool(motion.get('success')),
        minimum_body_center_to_cart_surface_m=minimum if math.isfinite(minimum) else None,
        required_surface_clearance_m=.38,maximum_encounter_lateral_excursion_m=lateral_excursion,emergency_guard_interventions=report.get('emergency_guard_interventions'),
        costmap_marked_samples=len(marked),lidar_cart_hit_frames=len(hit_frames),
        original_face_marked_samples=len(original_marked),
        costmap_cleared_samples_after_departure=len(cleared),nav2_zero_twist_samples_during_encounter=len(nav_stops),
        resumed_forward_samples_after_cart_cleared=len(resumed),truth_home_error_m=home_error,reported_truth_home_error_m=report.get('truth_home_error_m'),
        truth_outbound_goal_error_m=outbound_error,goal_ids=sorted(actual_goals),bag_confirmed_successful_goal_ids=sorted(actual_goals & successful),
        truth_in_navigation_tf=report.get('truth_in_navigation_tf'),
        scope='deterministic Nav2 out-and-back; robot truth only drives test fixture placement and evaluation; cart positions are service-acknowledged; no person/LLM validation',
        limitation='costmap neighborhood marking is supporting evidence; lidar endpoint matches use evaluated body pose and static sensor transform; no contact sensor proof for feet')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(11,4))
    if trajectory:
        axes[0].plot([p[1] for p in trajectory],[p[2] for p in trajectory],label='Robot truth')
        axes[0].scatter(*trajectory[0][1:],marker='o',label='Home')
    if (root/'cart_commands.jsonl').exists():
        track=[json.loads(line) for line in (root/'cart_commands.jsonl').read_text().splitlines()]
        axes[0].plot([p['x'] for p in track],[p['y'] for p in track],label='Cart (service acknowledged)')
    axes[0].set(xlabel='World x (m)',ylabel='World y (m)',title=scene['scenario']);axes[0].axis('equal');axes[0].legend()
    for topic,label in [('/deepseek/nav_cmd','Nav2 m/s'),('/robot1/cmd_vel','Actuator gait command')]:
        rows=[row for row in commands if row[1]==topic]
        axes[1].plot([r[0]-start for r in rows],[r[2] for r in rows],label=label)
    axes[1].axvspan(0,end-start,color='orange',alpha=.15,label='Cart motion')
    axes[1].set(xlabel='Simulation seconds after encounter start',ylabel='Forward command',title='Different command scales');axes[1].legend()
    fig.tight_layout();fig.savefig(root/'dynamic_trajectory.png',dpi=150);plt.close(fig)
    (root/'dynamic_validation.json').write_text(json.dumps(result,indent=2));return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path);a=p.parse_args()
    result=evaluate(a.directory);print(json.dumps(result),flush=True);raise SystemExit(0 if result['success'] else 1)
