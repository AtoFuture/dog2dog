"""Measured parallel walls and bounded guidance before the existing backend ramp."""
from dataclasses import asdict, dataclass, replace
import math
import numpy as np


@dataclass(frozen=True)
class Corridor:
    width_m: float
    heading_rad: float
    center_offset_m: float  # Positive means the corridor centre is to the left.
    wall_span_m: float
    wall_points: int
    forward_obstacle: bool = False
    backward_obstacle: bool = False
    forward_obstacle_m: float | None = None
    backward_obstacle_m: float | None = None
    positive_wall_start_m: float | None = None
    negative_wall_start_m: float | None = None


def _line(points, normal, tolerance, span):
    distances = points @ normal
    selected = distances > .2
    if np.count_nonzero(selected) < 8:
        return None
    bins = np.floor(distances[selected] / tolerance).astype(int)
    peak = np.bincount(bins).argmax()
    mask = np.abs(distances - (peak + .5) * tolerance) <= 1.5 * tolerance
    for _ in range(2):
        subset = points[mask]
        if len(subset) < 8:
            return None
        centre = subset.mean(axis=0)
        _, vectors = np.linalg.eigh((subset-centre).T @ (subset-centre))
        refined = vectors[:, 0]
        if refined @ normal < 0:
            refined = -refined
        distance = float(centre @ refined)
        mask = np.abs(points @ refined - distance) <= tolerance
    subset = points[mask]
    along = subset @ np.array([refined[1], -refined[0]])
    if len(subset) < 8 or np.ptp(along) < span:
        return None
    return refined, subset, float(np.ptp(along))


def _axis_support(values, span, approach_distance):
    """Return the wall start along one travel direction when locally usable.

    A corridor can either surround the base already or begin a short distance
    in front of it.  The latter is what lets guidance align before the body has
    crossed the entrance plane.
    """
    values = np.asarray(values, dtype=float)
    if len(values) < 8 or values.max() < .4:
        return None
    if values.min() <= -.2 and np.count_nonzero(np.abs(values) < .35) >= 3:
        return float(values.min())
    forward = values[values >= 0.]
    if (len(forward) < 8 or forward.min() > approach_distance
            or np.ptp(forward) < span):
        return None
    return float(forward.min())


def _flip_axis(geometry):
    """Express the same undirected corridor with the opposite axis sign."""
    heading = math.atan2(math.sin(geometry.heading_rad + math.pi),
                         math.cos(geometry.heading_rad + math.pi))
    return replace(
        geometry,
        heading_rad=heading,
        center_offset_m=-geometry.center_offset_m,
        forward_obstacle=geometry.backward_obstacle,
        backward_obstacle=geometry.forward_obstacle,
        forward_obstacle_m=geometry.backward_obstacle_m,
        backward_obstacle_m=geometry.forward_obstacle_m,
        positive_wall_start_m=geometry.negative_wall_start_m,
        negative_wall_start_m=geometry.positive_wall_start_m,
    )


def detect_corridor(points, settings):
    """Fit two long, opposite, parallel walls around or just ahead of the base.

    This estimates local geometry, never free space or a traversable route.
    One wall, a doorway, or a wide room is insufficient evidence.
    """
    values = np.asarray(points, dtype=float)
    if values.ndim != 2 or values.shape[1] != 2 or not np.isfinite(values).all():
        return None
    values = values[np.linalg.norm(values, axis=1) <= 3.5]
    if len(values) < 16:
        return None
    best = None
    for angle in np.arange(-math.pi/2, math.pi/2, math.pi/60):
        normal = np.array([-math.sin(angle), math.cos(angle)])
        left = _line(values, normal, settings['wall_tolerance_m'], settings['min_span_m'])
        right = _line(values, -normal, settings['wall_tolerance_m'], settings['min_span_m'])
        if left is None or right is None or left[0] @ -right[0] < math.cos(.10):
            continue
        normal = left[0] - right[0]
        normal /= np.linalg.norm(normal)
        if normal[1] < 0:
            normal = -normal
        distances = sorted((float(np.median(left[1] @ normal)), float(np.median(right[1] @ normal))))
        width = distances[1] - distances[0]
        if not distances[0] < 0 < distances[1] or not .4 < width <= settings['max_width_m']:
            continue
        across = values @ normal
        axis = np.array([normal[1],-normal[0]])
        along = values @ axis
        left_along = left[1] @ axis
        right_along = right[1] @ axis
        approach_distance = settings.get('approach_distance_m', 1.)
        positive = (_axis_support(left_along, settings['min_span_m'], approach_distance),
                    _axis_support(right_along, settings['min_span_m'], approach_distance))
        negative = (_axis_support(-left_along, settings['min_span_m'], approach_distance),
                    _axis_support(-right_along, settings['min_span_m'], approach_distance))
        positive_start = max(positive) if all(v is not None for v in positive) else None
        negative_start = max(negative) if all(v is not None for v in negative) else None
        if positive_start is None and negative_start is None:
            continue
        interior = ((across > distances[0]+.1) & (across < distances[1]-.1)
                    & (np.abs(along) < 1.5) & (np.abs(along) > .2))
        result = Corridor(
            width_m=width,
            heading_rad=math.atan2(-normal[0], normal[1]),
            center_offset_m=sum(distances)/2,
            wall_span_m=min(left[2], right[2]),
            wall_points=min(len(left[1]), len(right[1])),
            forward_obstacle=np.count_nonzero(interior & (along>0))>=3,
            backward_obstacle=np.count_nonzero(interior & (along<0))>=3,
            forward_obstacle_m=(float(np.min(along[interior & (along>0)]))
                                if np.count_nonzero(interior & (along>0))>=3 else None),
            backward_obstacle_m=(float(np.min(-along[interior & (along<0)]))
                                 if np.count_nonzero(interior & (along<0))>=3 else None),
            positive_wall_start_m=positive_start,
            negative_wall_start_m=negative_start,
        )
        if best is None or result.wall_points > best.wall_points:
            best = result
    return best


class CorridorTurnSettler:
    """Separate a corridor walk from a pure turn in the fixed-gait backend.

    Inputs ``current_*`` are scaler outputs, not measured body velocities.
    ``now`` uses the backend's ROS clock, so a slower simulation cannot shorten
    the wait. Once both ramps stop, 0.8s covers its longest auto-rest boundary:
    44 controller ticks at 60Hz. This is command sequencing, not a guarantee
    of physical stationarity. A pending stop survives loss of wall detection.
    """

    def __init__(self):
        self.pending = False
        self.stopped_since = None

    def filter(self, linear, angular, current_linear, current_angular, now, enabled):
        if enabled and (abs(linear) > 1e-6 or abs(current_linear) > 1e-6):
            self.pending = True
            self.stopped_since = None
        if not self.pending or abs(linear) > 1e-6:
            return angular, False
        if abs(current_linear) > 1e-6 or abs(current_angular) > 1e-6:
            self.stopped_since = None
            return 0., True
        if self.stopped_since is None or now < self.stopped_since:
            self.stopped_since = now
        if now - self.stopped_since < .8:
            return 0., True
        self.pending = False
        self.stopped_since = None
        return angular, False


class CorridorGuidance:
    def __init__(self, settings):
        self.settings = settings
        self.geometry = None
        self.previous = None
        self.confirmations = 0
        self.last_stamp = None
        self.alignment_confirmations = 0
        self._alignment_stamp = None
        self._aligned_direction = None
        self._reversal_sign = None

    def reset_alignment(self):
        self.alignment_confirmations = 0
        self._alignment_stamp = None
        self._aligned_direction = None
        self._reversal_sign = None

    def observe(self, points, stamp):
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return False
        self.last_stamp = stamp
        geometry = detect_corridor(points, self.settings)
        previous = self.previous
        consistent = (geometry is not None and previous is not None
            and abs(geometry.width_m-previous.width_m) < .15
            and abs(math.sin(geometry.heading_rad-previous.heading_rad)) < .15
            and abs(geometry.center_offset_m-(1 if math.cos(geometry.heading_rad-previous.heading_rad)>=0 else -1)*previous.center_offset_m) < .15)
        if consistent and math.cos(geometry.heading_rad-previous.heading_rad) < 0:
            geometry = _flip_axis(geometry)
        self.confirmations = self.confirmations+1 if consistent else int(geometry is not None)
        self.previous = geometry
        self.geometry = geometry if self.confirmations >= 2 else None
        if self.geometry is None:
            self.reset_alignment()
        return True

    def guide(self, linear, angular, target_xy=None):
        """Only guide a requested forward motion; never initiate motion at rest."""
        if not self.settings.get('steering_enabled', False):
            return linear, angular, 'observing'
        g = self.geometry
        if g is None:
            return linear, angular, 'unconfirmed'
        if linear < 0:
            return linear, angular, 'reverse_passthrough'  # Final guard refuses reverse.
        turn_limit = self.settings['turn_rate_radps']
        reversal = self._reversal_turn(linear, angular, target_xy)
        if reversal is not None:
            return 0., reversal, 'reversal_turning'
        if linear == 0:
            return 0., max(-turn_limit, min(turn_limit, angular)), 'turning' if angular else 'stationary'
        if target_xy is None or not all(math.isfinite(v) for v in target_xy):
            self.reset_alignment()
            return 0.,0.,'awaiting_path'
        limit = min(.25,turn_limit)
        # Parallel walls define an *undirected* axis. A local Nav2 path
        # lookahead chooses its forward/backward sense for exploration/return.
        projection = target_xy[0]*math.cos(g.heading_rad)+target_xy[1]*math.sin(g.heading_rad)
        direction = 1 if projection>=0 else -1
        wall_start = g.positive_wall_start_m if direction>0 else g.negative_wall_start_m
        if wall_start is None:
            self.reset_alignment()
            return linear, angular, 'direction_unconfirmed'
        obstacle = g.forward_obstacle_m if direction>0 else g.backward_obstacle_m
        if obstacle is not None and obstacle < abs(projection):
            self.reset_alignment()
            return linear,angular,'planner_obstacle_avoidance'
        heading = math.atan2(math.sin(g.heading_rad+(math.pi if direction<0 else 0)),
                             math.cos(g.heading_rad+(math.pi if direction<0 else 0)))
        if abs(abs(heading)-math.pi)<.05 and angular:
            heading = math.copysign(abs(heading),angular)
        target = heading + math.atan2(direction*g.center_offset_m, self.settings['lookahead_m'])
        target = math.atan2(math.sin(target), math.cos(target))
        correction = self.settings['heading_gain'] * target
        if direction != self._aligned_direction:
            self.reset_alignment()
            self._aligned_direction = direction
        if abs(target) > self.settings['max_heading_error_rad']:
            self.alignment_confirmations = 0
            self._alignment_stamp = None
        required = self.settings.get('alignment_confirmations', 3)
        if self.alignment_confirmations < required:
            tolerance = self.settings.get('alignment_tolerance_rad', .08)
            if abs(target) <= tolerance:
                if self._alignment_stamp != self.last_stamp:
                    self.alignment_confirmations += 1
                    self._alignment_stamp = self.last_stamp
            else:
                self.alignment_confirmations = 0
                self._alignment_stamp = None
            if self.alignment_confirmations < required:
                mode = 'entry_stabilizing' if abs(target) <= tolerance else 'entry_aligning'
                return 0., max(-turn_limit,min(turn_limit,correction)), mode
        steering = max(-limit, min(limit, correction))
        if wall_start > -.2:
            return linear, steering, 'approaching'
        return linear, steering, 'goal_approach' if math.hypot(*target_xy)<.5 else 'centering'

    def _reversal_turn(self, linear, angular, target_xy):
        """Experimental direction choice for this backend's measured turn slip.

        A positive stationary turn drifts locally right, a negative one left.
        Choose once near a corridor reversal; keep the choice across angle wrap
        and zero requests. Never create motion from a zero request, modify a
        near-goal orientation maneuver, or replace planner obstacle avoidance.
        """
        if not self.settings.get('reversal_turn_enabled', False):
            return None
        g = self.geometry
        if (target_xy is None or not all(math.isfinite(v) for v in target_xy)
                or math.hypot(*target_xy) < .5):
            self._reversal_sign = None
            return None
        projection = target_xy[0]*math.cos(g.heading_rad)+target_xy[1]*math.sin(g.heading_rad)
        direction = 1 if projection >= 0 else -1
        wall_start = g.positive_wall_start_m if direction > 0 else g.negative_wall_start_m
        obstacle = g.forward_obstacle_m if direction > 0 else g.backward_obstacle_m
        heading = math.atan2(math.sin(g.heading_rad+(math.pi if direction < 0 else 0)),
                             math.cos(g.heading_rad+(math.pi if direction < 0 else 0)))
        if wall_start is None or (obstacle is not None and obstacle < abs(projection)) or abs(heading) < .35:
            self._reversal_sign = None
            return None
        if not linear and not angular:
            return None
        if self._reversal_sign is None:
            # Require a near reversal and a measured lateral offset. At an
            # arbitrary crossing angle there is no calibrated slip prediction.
            if abs(heading) < 2.2 or abs(g.center_offset_m) < .06:
                return None
            self._reversal_sign = -math.copysign(1., g.center_offset_m*math.cos(g.heading_rad))
        rate = self.settings['turn_rate_radps']
        if not linear:
            rate = min(rate, abs(angular))
        return self._reversal_sign*rate

    def snapshot(self):
        return dict(confirmed=self.geometry is not None, confirmations=self.confirmations,
                    alignment_confirmations=self.alignment_confirmations,
                    reversal_turn_sign=self._reversal_sign,
                    geometry=asdict(self.geometry) if self.geometry is not None else None)
