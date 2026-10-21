"""Bounded straight RETURN retreat along a recently traversed odometry trail."""
from collections import deque
import math
from .velocity_safety import swept_collision,clearance_increasing_escape


class ReverseRetreat:
    def __init__(self,settings):
        self.settings=settings
        self.trail=deque(maxlen=400)
        self.pose=None
        self.pose_stamp=None
        self.mission='standby'
        self.reset()

    def reset(self):
        self.phase=None
        self.started=None
        self.stopped_since=None
        self.travel=0.
        self.extent=0.
        self.origin=None
        self.exit_budget_stop=False
        self.episodes=0
        self.blocked=False
        self.admissions=0
        self.last_scan=None
        self.wide_samples=0

    def set_mission(self,mission):
        if mission==self.mission:return
        old=self.mission;self.mission=mission;self.reset()
        if mission=='exploring' and old!='paused':self.trail.clear()

    def observe_pose(self,pose,stamp):
        if len(pose)!=3 or not all(math.isfinite(v)for v in (*pose,stamp)):
            self.pose=None;self.trail.clear();return False
        if self.pose_stamp is not None and stamp<=self.pose_stamp:return False
        previous=self.pose
        distance=math.hypot(pose[0]-previous[0],pose[1]-previous[1])if previous else 0.
        if previous and (distance>.35 or abs(math.remainder(pose[2]-previous[2],2*math.pi))>.5):
            self.trail.clear()
            if self.started is not None:self.blocked=True
        if self.started is not None and not self.blocked:
            self.travel+=distance
            # Bound the excursion of this straight retreat, rather than summing
            # every EKF settling oscillation as additional commanded travel.
            self.extent=max(self.extent,math.hypot(pose[0]-self.origin[0],pose[1]-self.origin[1]))
        self.pose=tuple(pose);self.pose_stamp=stamp
        if self.started is None and (not self.trail or math.hypot(pose[0]-self.trail[-1][0],pose[1]-self.trail[-1][1])>=.03):
            self.trail.append(tuple(pose))
        return True

    def _on_trail(self):
        if not self.pose or not self.trail:return False
        x,y,yaw=self.pose;c,s=math.cos(yaw),math.sin(yaw)
        trail=list(self.trail)
        index=min(range(len(trail)),key=lambda i:math.hypot(trail[i][0]-x,trail[i][1]-y))
        if math.hypot(trail[index][0]-x,trail[index][1]-y)>.12:return False
        history_span=min(self.settings['robot_radius_m'],
            self.settings['prediction_speed_mps']*self.settings['prediction_horizon_s']+.02)
        for p in reversed(trail[:index+1]):
            dx,dy=p[0]-x,p[1]-y;bx=c*dx+s*dy;by=-s*dx+c*dy
            # Settling odometry can move backwards over a few recent samples.
            # Require history beyond the predicted centre travel. Requiring a
            # whole body radius falsely exhausts history while approaching home;
            # the independent full scan sweep still certifies the entire body.
            if math.hypot(bx,by)<=history_span:continue
            return bx<0 and abs(by)<=.12
        # Near the beginning of the trail, the previously occupied body covers
        # the short prediction span even without an older centre waypoint.
        # Use the same traversed segment ahead to verify its direction; the
        # nearest-centre bound above and full rear scan sweep still apply.
        for p in trail[index+1:]:
            dx,dy=p[0]-x,p[1]-y;bx=c*dx+s*dy;by=-s*dx+c*dy
            if math.hypot(bx,by)<=history_span:continue
            return bx>0 and abs(by)<=.12
        return False

    def _settled(self,now,x,z):
        if abs(x)>1e-6 or abs(z)>1e-6:self.stopped_since=None;return False
        if self.stopped_since is None:self.stopped_since=now
        return now-self.stopped_since>=.8

    def filter(self,*,requested,target,points,scan_id,scan_age,odom_fresh,enabled,
               now,current_linear,current_angular,sensor_radius,range_min):
        active=self.phase is not None
        if (not enabled or self.mission!='returning' or not requested or points is None
                or not odom_fresh or self.pose is None or not math.isfinite(scan_age)or not 0<=scan_age<=.5):
            self.phase=None;self.stopped_since=None;self.admissions=0
            return (0.,0.,'reverse_retreat_cancelled')if active else None
        if self.started is not None and now-self.started< -1e-6:self.blocked=True
        budget_reached=self.started is not None and (now-self.started>12. or self.extent>self.settings.get('max_extent_m',1.5))
        if self.blocked:
            self.phase=None
            self.started=None
            return 0.,0.,'reverse_retreat_limit'
        g=self.settings;radius=g['robot_radius_m'];margin=g['clearance_margin_m'];horizon=g['prediction_horizon_s']+scan_age
        clearance=min((math.hypot(x,y)for x,y in points),default=math.inf)
        turn_space=max(radius+margin+g['prediction_speed_mps']*g['prediction_horizon_s'],sensor_radius+range_min+.04)
        # Once backing out, retain additional room for the displaced laser
        # during the ensuing turn's possible gait translation. Admission stays
        # unchanged, so an ordinary centred 1.4m corridor is not forced back.
        exit_space=max(turn_space,sensor_radius+range_min+.04+
            g['prediction_speed_mps']*g['prediction_horizon_s'])
        wide=clearance>=(exit_space if self.started is not None else turn_space)
        if budget_reached:
            # Evaluate THIS scan before latching a failure: arrival in open
            # space and the budget crossing can happen in the same callback.
            # Exhaustion permits only stationary exit verification, never any
            # additional reverse. A narrow-space exhaustion remains latched.
            if wide:self.exit_budget_stop=True
            else:
                self.blocked=True;self.phase=None;self.started=None
                return 0.,0.,'reverse_retreat_limit'
        if self.exit_budget_stop and not wide:
            self.blocked=True;self.phase=None;self.started=None
            return 0.,0.,'reverse_retreat_limit'
        # Brake with extra clearance, then retain the original exit threshold
        # while settling. Gait rebound must not chatter across a single boundary.
        exit_ready=wide and (self.started is None or self.phase=='exit_brake'
            or self.exit_budget_stop or clearance>=exit_space+margin)
        behind=(target is not None and len(target)==2 and all(math.isfinite(v)for v in target)
                and target[0]<-.25 and abs(target[1])<=.12+abs(target[0])*math.tan(.2))
        on_trail=self._on_trail()
        speed=-g['prediction_speed_mps']
        if clearance<radius+margin-.01:
            rear_safe=(clearance>=radius+.02 and clearance_increasing_escape(points,clearance,speed,horizon,(0.,0.,0.)))
        else:
            rear_safe=not any(swept_collision(points,radius+margin,(speed,0.,omega),horizon)for omega in(-.1,0.,.1))
        fresh_scan=scan_id!=self.last_scan
        if fresh_scan:self.last_scan=scan_id
        if self.phase is None:
            if self.started is not None and exit_ready:
                # Cancellation stops motion but must not silently turn an
                # unfinished retreat into an unchecked navigation handoff.
                self.phase='exit_brake';self.stopped_since=None;self.wide_samples=0
            else:
                if (wide and self.started is None) or not behind or not on_trail or not rear_safe:
                    self.admissions=0
                    return (0.,0.,'reverse_retreat_path_blocked')if self.started is not None else None
                if fresh_scan:self.admissions+=1
                if self.admissions<3:return 0.,0.,'reverse_retreat_confirming'
                if self.started is None:
                    if self.episodes>=2:self.blocked=True;return 0.,0.,'reverse_retreat_limit'
                    self.episodes+=1;self.started=now;self.travel=0.;self.extent=0.;self.origin=self.pose;self.exit_budget_stop=False
                self.phase='brake';self.stopped_since=None
        if exit_ready:
            if self.phase!='exit_brake':self.phase='exit_brake';self.stopped_since=None;self.wide_samples=0
            if fresh_scan:self.wide_samples+=1
            if self._settled(now,current_linear,current_angular)and self.wide_samples>=3:
                self.phase=None;self.started=None;self.stopped_since=None;self.admissions=0;self.exit_budget_stop=False
                return 0.,0.,'reverse_retreat_complete'
            return 0.,0.,'reverse_retreat_settling'
        if not behind or not on_trail or not rear_safe:
            self.phase='brake';self.stopped_since=None
            return 0.,0.,'reverse_retreat_path_blocked'
        if self.phase=='exit_brake':self.phase='brake';self.stopped_since=None
        if self.phase=='brake':
            if not self._settled(now,current_linear,current_angular):return 0.,0.,'reverse_retreat_settling'
            self.phase='retreat';self.stopped_since=None
        return -g['desired_linear_velocity_mps'],0.,'reverse_retreat'

    def snapshot(self):
        return dict(reverse_phase=self.phase,reverse_episodes=self.episodes,
                    reverse_distance_m=self.travel,reverse_blocked=self.blocked,
                    reverse_extent_m=self.extent,
                    reverse_extent_limit_m=self.settings.get('max_extent_m',1.5),
                    reverse_exit_budget_stop=self.exit_budget_stop,
                    reverse_trail_samples=len(self.trail))
