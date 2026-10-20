import math
from pathlib import Path
import pytest
from qwen_robotnav.runtime_profile import load_profile
from qwen_robotnav.reverse_retreat import ReverseRetreat

P=Path(__file__).parents[1]/'config/sim_runtime.yaml'

def policy():
 p=load_profile(P);g=dict(p['guard'],robot_radius_m=.4,desired_linear_velocity_mps=.32);r=ReverseRetreat(g);r.set_mission('exploring')
 for i in range(51):r.observe_pose((i*.05,0.,0.),i*.1)
 r.set_mission('returning');return r

def walls(distance=.6):return tuple((i*.1,side*distance)for i in range(-30,31)for side in(-1,1))

def step(r,t,**kw):
 args=dict(requested=True,target=(-1.,0.),points=walls(),scan_id=t,scan_age=.05,odom_fresh=True,enabled=True,
  now=t,current_linear=0.,current_angular=0.,sensor_radius=.22,range_min=.3);args.update(kw);return r.filter(**args)

def begin(r):
 assert step(r,0.)[2]=='reverse_retreat_confirming'
 assert step(r,.1)[2]=='reverse_retreat_confirming'
 assert step(r,.2)[2]=='reverse_retreat_settling'
 assert step(r,.99)[0]==0.
 assert step(r,1.01)==(-.32,0.,'reverse_retreat')

def test_retreat_brakes_follows_recorded_trail_and_stops_in_verified_open_space():
 r=policy();begin(r);size=len(r.trail)
 r.observe_pose((2.3,0.,0.),6.);assert len(r.trail)==size
 assert step(r,1.1)[0]<0
 assert step(r,1.2,points=walls(.8),current_linear=-.024)[0]==0
 step(r,1.3,points=walls(.8));assert step(r,2.11,points=walls(.8))[2]=='reverse_retreat_complete'
 assert step(r,2.2,points=walls(.8))is None
 assert r.episodes==1

@pytest.mark.parametrize('kw',[dict(enabled=False),dict(odom_fresh=False),dict(points=None),dict(target=None),dict(target=(1.,0.)),dict(target=(-1.,.4)),dict(scan_age=.51)])
def test_no_reverse_without_required_evidence(kw):
 r=policy()
 for t in(0.,.1,.2,1.1):assert step(r,t,**kw)is None
 assert r.episodes==0

def test_no_reverse_without_recent_aligned_traversed_path():
 r=policy();r.trail.clear()
 assert step(r,0.)is None
 r=policy();r.pose=(2.5,.2,0.)
 assert step(r,0.)is None
 r=policy();r.pose=(2.5,0.,math.pi)
 assert step(r,0.)is None

def test_rear_obstacle_refuses_request_and_leaving_the_trail_stops_active_retreat():
 r=policy();assert step(r,0.,points=walls()+((-.55,0.),))is None
 r=policy();begin(r);r.observe_pose((2.45,.14,0.),6.)
 assert step(r,1.2)==(0.,0.,'reverse_retreat_path_blocked')

def test_zero_pause_and_stale_odometry_stop_without_replenishing_original_deadline():
 r=policy();begin(r)
 assert step(r,1.1,requested=False)==(0.,0.,'reverse_retreat_cancelled')
 for t in(2.,2.1,2.2):step(r,t)
 assert r.episodes==1 and r.started==.2
 assert step(r,3.1)[0]<0
 assert step(r,3.2,odom_fresh=False)[0]==0
 step(r,12.3);assert r.blocked

def test_pose_jump_clock_reset_and_distance_budget_fail_closed():
 r=policy();begin(r);r.observe_pose((3.5,0.,0.),7.)
 assert step(r,1.2)==(0.,0.,'reverse_retreat_limit')
 r=policy();begin(r);assert step(r,-1.)[0]==0 and r.blocked
 r=policy();begin(r)
 for i in range(1,9):r.observe_pose((2.5-.2*i,0.,0.),6.+i)
 assert step(r,2.)[0]==0 and r.blocked

def test_unchanged_scan_does_not_satisfy_admission_and_wide_corridor_does_not_reverse():
 r=policy()
 for t in(0.,.1,.2):assert step(r,t,scan_id=1)[2]=='reverse_retreat_confirming'
 assert r.episodes==0
 r=policy();assert step(r,0.,points=walls(.7))is None

def test_non_return_missions_never_initiate_retreat():
 r=policy();r.set_mission('exploring')
 assert step(r,0.)is None
 r=policy();begin(r);r.set_mission('estop');assert step(r,1.2)is None

def test_settling_samples_inside_current_body_do_not_hide_the_verified_rear_trail():
 r=policy()
 for point in ((2.62,.02,0.),(2.56,-.015,0.),(2.5,0.,0.)):r.trail.append(point)
 begin(r)
 r=policy();r.trail.clear();r.trail.extend(((2.2,.4,0.),(2.4,.2,0.),(2.5,0.,0.)))
 assert step(r,0.)is None

def test_exhausted_retreat_remains_stopped_and_does_not_count_subsequent_pose_motion():
 r=policy();begin(r);assert step(r,13.)[2]=='reverse_retreat_limit'
 distance=r.travel
 r.observe_pose((2.4,0.,0.),7.)
 assert r.travel==distance
 assert step(r,13.1)[2]=='reverse_retreat_limit'
 assert step(r,13.2,points=walls(.8))[2]=='reverse_retreat_limit'

def test_exit_reserves_sensor_and_gait_room_without_forcing_wide_corridor_admission():
 r=policy();assert step(r,0.,points=walls(.7))is None
 r=policy();begin(r);assert step(r,1.1,points=walls(.7))[2]=='reverse_retreat'
 assert step(r,1.2,points=walls(.8))[2]=='reverse_retreat_settling'
