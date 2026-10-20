"""ROS-independent final command policy; only real sensor health is accepted."""
import math


def scan_points(ranges, angle_min, increment, range_min, range_max, sensor):
    """Project a complete measured scan into base; None means unusable coverage.

    Infinity is a valid clear ray, NaN is missing evidence. Eight angular
    sectors must each contain measured evidence; partial front scans cannot
    authorize a full-body motion check.
    """
    if not all(math.isfinite(v) for v in (angle_min,increment,range_min,range_max,*sensor)):
        return None
    if increment<=0 or not 0<=range_min<range_max or len(ranges)*increment<2*math.pi-.1:
        return None
    sectors=set();points=[];sx,sy,yaw=sensor
    for i,d in enumerate(ranges):
        a=angle_min+i*increment
        if d==math.inf or math.isfinite(d) and range_min<=d<=range_max:
            sectors.add(int(((a+math.pi)%(2*math.pi))/(math.pi/4)))
            if math.isfinite(d):points.append((sx+d*math.cos(a+yaw),sy+d*math.sin(a+yaw)))
    return tuple(points) if len(sectors)==8 else None



def swept_collision(points,radius,velocity,horizon):
    vx,vy,omega=velocity
    for step in range(1,11):
        dt=horizon*step/10
        if abs(omega)<1e-6:cx,cy=vx*dt,vy*dt
        else:
            a=omega*dt;cx=(vx*math.sin(a)-vy*(1-math.cos(a)))/omega
            cy=(vx*(1-math.cos(a))+vy*math.sin(a))/omega
        if any(math.hypot(px-cx,py-cy)<radius for px,py in points):return True
    return False


def clearance_increasing_escape(points, clearance, speed, horizon, measured_velocity):
    """Certify a straight retreat from an already occupied safety margin.

    Every sampled swept circle preserves the *current* measured clearance;
    the endpoint must gain at least 2cm. Check both signs of a small yaw lag,
    and reject residual body drift that continues decreasing clearance.
    This never authorizes rotation, missing scans, or deeper encroachment.
    """
    if not points or not all(math.isfinite(v) for v in measured_velocity):
        return False
    if swept_collision(points, clearance-1e-4, measured_velocity, horizon):
        return False
    for omega in (-.1, 0., .1):
        for step in range(1,21):
            dt=horizon*step/20
            if omega:
                cx=speed*math.sin(omega*dt)/omega
                cy=speed*(1-math.cos(omega*dt))/omega
            else:
                cx,cy=speed*dt,0.
            distance=min(math.hypot(px-cx,py-cy) for px,py in points)
            if distance < clearance-1e-4 or (step==20 and distance < clearance+.02):
                return False
    return True


class VelocitySafety:
    def __init__(self,settings,enabled=False,robot_radius=.4):
        self.settings=settings;self.enabled=enabled;self.seen={};self.stamps={}
        self.command=(0.,0.);self.mission='standby';self.front=0.;self.tilt=math.inf
        self.publisher_conflict=False;self.robot_radius=robot_radius
        self.obstacles=None;self.measured_velocity=(0.,0.,0.)

    def observe(self,source,now,stamp=None):
        if stamp is not None:
            if stamp<=self.stamps.get(source,-1):return False
            self.stamps[source]=stamp
        self.seen[source]=now;return True

    def invalidate(self,source):
        self.seen.pop(source,None)

    def output(self,now):
        g=self.settings
        if not self.enabled:return 0.,0.,'motion_disabled'
        if self.publisher_conflict:return 0.,0.,'multiple_actuator_publishers'
        for source,timeout in [('mission',g['mission_timeout_s']),('command',g['command_timeout_s']),
            ('scan',g['sensor_timeout_s']),('odometry',g['sensor_timeout_s']),('imu',g['sensor_timeout_s']),('battery',g['battery_timeout_s'])]:
            stamp=self.seen.get(source)
            if stamp is None or not 0<=now-stamp<timeout:return 0.,0.,source+'_unavailable'
        if self.mission not in ('exploring','returning'):return 0.,0.,'mission:'+self.mission
        nav_seen=self.seen.get('nav_command')
        if nav_seen is None or not 0<=now-nav_seen<g['command_timeout_s']:
            return 0.,0.,'nav_command_unavailable'
        if not math.isfinite(self.tilt) or self.tilt>g['max_tilt_rad']:return 0.,0.,'excessive_tilt'
        x,z=self.command
        if not all(math.isfinite(v) for v in (x,z)):return 0.,0.,'invalid_command'
        # Reverse is unsupported until rear clearance is independently measured.
        if x<0:return 0.,0.,'reverse_not_supported'
        x=min(g['linear_max'],x);z=max(-g['angular_max'],min(g['angular_max'],z))
        if self.obstacles is None:return 0.,0.,'footprint_evidence_unavailable'
        clearance=min((math.hypot(px,py) for px,py in self.obstacles),default=math.inf)
        radius=self.robot_radius+g['clearance_margin_m']
        # A 1cm recovery band avoids latching both outputs at a soft approach
        # boundary. The configured footprint still retains >=4cm turn margin.
        hard_radius=radius-.01
        if clearance<hard_radius:
            # The ordinary hard boundary remains unchanged. A bounded straight
            # escape is allowed only outside the physical body plus 2cm and
            # only when its full swept envelope increases existing clearance.
            if (g.get('clearance_recovery_enabled',False) and x>0 and abs(z)<1e-6
                    and clearance>=self.robot_radius+.02
                    and clearance_increasing_escape(self.obstacles,clearance,
                        max(g['prediction_speed_mps'],math.hypot(*self.measured_velocity[:2])),
                        g['prediction_horizon_s']+max(0.,now-self.seen['scan']),
                        self.measured_velocity)):
                return x,0.,'clearance_recovery'
            return 0.,0.,'footprint_clearance'
        if x or z:
            horizon=g['prediction_horizon_s']+max(0.,now-self.seen['scan'])
            # Residual/lateral gait drift matters even during an in-place turn.
            if swept_collision(self.obstacles,hard_radius,self.measured_velocity,horizon):
                return 0.,0.,'predicted_footprint_collision'
            if x>0:
                # Scaled command units are not m/s. Include angular lag and the
                # measured simulator gait envelope; do not reduce this to x.
                speed=max(g['prediction_speed_mps'],math.hypot(*self.measured_velocity[:2]))
                if any(swept_collision(self.obstacles,min(radius,clearance),v,horizon)
                       for v in ((speed,0.,0.),(speed,0.,z*.6))):
                    return 0.,z,'predicted_footprint_collision'
        if self.front<=g['front_clearance_m'] and x>0:return 0.,z,'front_obstacle'
        return x,z,'allowed'
