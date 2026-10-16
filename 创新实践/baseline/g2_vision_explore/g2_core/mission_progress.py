"""Bounded exploration policy. Durations use active monotonic time, never ROS time."""
from dataclasses import dataclass
import math


END_REASONS = {
    'no_frontiers': True,
    'time_budget': False,
    'coverage_stalled': False,
    'candidates_unavailable': False,
    'navigation_failures': False,
}


@dataclass(frozen=True)
class ProgressLimits:
    exploration_budget_s: float = 900.0
    coverage_stall_s: float = 180.0
    coverage_min_gain_m2: float = 0.5
    candidate_wait_s: float = 90.0
    no_frontier_confirm_s: float = 10.0
    max_region_failures: int = 2
    max_consecutive_failures: int = 6

    def __post_init__(self):
        for key, value in vars(self).items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError('Expected positive finite '+key)
            if key.startswith('max_') and type(value) is not int:
                raise ValueError('Expected integer '+key)
        if self.candidate_wait_s <= 30:
            raise ValueError('candidate_wait_s must exceed the 30 second blacklist TTL')


class MissionProgress:
    def __init__(self, session_id, limits=None, *, now):
        self.session_id = session_id
        self.limits = limits or ProgressLimits()
        self.elapsed = 0.0
        self._at = now
        self.active = False
        self._area = None
        self._growth_at = 0.0
        self._unavailable_at = None
        self._empty_at = None
        self._empty_frames = set()
        self._last_empty_frame = None
        self._last_empty_at = 0.0
        self.consecutive_failures = 0
        self.succeeded_goals = 0
        self.failed_goals = 0
        self.result = None

    def advance(self, now, *, active):
        if self.active:
            self.elapsed += max(0.0, now-self._at)
        self._at = now
        self.active = active

    def observe_area(self, area):
        if not self.active or not math.isfinite(area) or area < 0:
            return
        if self._area is None or area >= self._area+self.limits.coverage_min_gain_m2:
            self._area = area
            self._growth_at = self.elapsed

    def selection(self, status, *, frame_id=None):
        if status != 'NO_FRONTIER':
            self._empty_at = None
            self._empty_frames.clear()
            self._last_empty_frame = None
        if status == 'NO_FRONTIER':
            if self._empty_at is None:
                self._empty_at = self.elapsed
            if frame_id is not None and frame_id != self._last_empty_frame:
                if len(self._empty_frames) < 3:
                    self._empty_frames.add(frame_id)
                self._last_empty_frame = frame_id
                self._last_empty_at = self.elapsed
        if status in ('NO_CANDIDATE', 'SMALL_FRONTIERS'):
            if self._unavailable_at is None:
                self._unavailable_at = self.elapsed
        else:
            self._unavailable_at = None

    def navigation_result(self, success):
        if success:
            self.succeeded_goals += 1
            self.consecutive_failures = 0
        else:
            self.failed_goals += 1
            self.consecutive_failures += 1

    def evaluate(self, *, healthy, selection_current=True):
        """Unhealthy map/pose cannot establish completion or justify a new return goal."""
        if self.result is not None or not self.active:
            return self.result
        if not healthy:
            self.selection(None)
            return None
        p = self.limits
        reason = None
        if self.elapsed >= p.exploration_budget_s:
            reason = 'time_budget'
        elif self.consecutive_failures >= p.max_consecutive_failures:
            reason = 'navigation_failures'
        elif (selection_current and self._empty_at is not None and len(self._empty_frames) >= 3
              and self._last_empty_at-self._empty_at >= p.no_frontier_confirm_s):
            reason = 'no_frontiers'
        elif selection_current and self._unavailable_at is not None and self.elapsed-self._unavailable_at >= p.candidate_wait_s:
            reason = 'candidates_unavailable'
        elif self._area is not None and self.elapsed-self._growth_at >= p.coverage_stall_s:
            reason = 'coverage_stalled'
        if reason:
            self.result = dict(session_id=self.session_id, reason=reason, completed=END_REASONS[reason],
                               active_seconds=self.elapsed, known_area_m2=self._area,
                               succeeded_goals=self.succeeded_goals, failed_goals=self.failed_goals)
        return self.result
