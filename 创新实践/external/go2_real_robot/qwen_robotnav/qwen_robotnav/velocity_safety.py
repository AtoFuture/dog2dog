"""ROS-independent final command policy; only real sensor health is accepted."""
import math


class VelocitySafety:
    def __init__(self,settings,enabled=False):
        self.settings=settings;self.enabled=enabled;self.seen={};self.stamps={}
        self.command=(0.,0.);self.mission='standby';self.front=0.;self.tilt=math.inf
        self.publisher_conflict=False

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
        if not math.isfinite(self.tilt) or self.tilt>g['max_tilt_rad']:return 0.,0.,'excessive_tilt'
        x,z=self.command
        if not all(math.isfinite(v) for v in (x,z)):return 0.,0.,'invalid_command'
        # Reverse is unsupported until rear clearance is independently measured.
        if x<0:return 0.,0.,'reverse_not_supported'
        x=min(g['linear_max'],x);z=max(-g['angular_max'],min(g['angular_max'],z))
        if self.front<=g['front_clearance_m'] and x>0:return 0.,z,'front_obstacle'
        return x,z,'allowed'
