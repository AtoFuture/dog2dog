"""Steady-time freshness and recovery debounce; no ROS or motion side effects."""
import math


class RecoveryHealth:
    def __init__(self, sources, *, stale_s=2., stable_s=3., timeout_s=60.):
        if not sources or len(set(sources)) != len(sources):
            raise ValueError('Health sources must be nonempty and unique')
        for value in (stale_s, stable_s, timeout_s):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Recovery intervals must be finite and positive')
        if timeout_s <= stable_s:
            raise ValueError('Recovery timeout must exceed stable interval')
        self.sources = dict.fromkeys(sources)
        self.stale_s, self.stable_s, self.timeout_s = stale_s, stable_s, timeout_s
        self.healthy_since = None
        self.stamps = {}

    def observe(self, source, now, stamp=None):
        if source not in self.sources:
            raise ValueError('Unknown health source')
        if stamp is not None:
            if stamp <= self.stamps.get(source, -1):
                return
            self.stamps[source] = stamp
        self.sources[source] = now

    def invalidate(self, source):
        if source not in self.sources:
            raise ValueError('Unknown health source')
        self.sources[source] = None
        self.healthy_since = None

    def missing(self, now):
        return [name for name, stamp in self.sources.items()
                if stamp is None or not 0 <= now-stamp <= self.stale_s]

    def ready(self, now):
        if self.missing(now):
            self.healthy_since = None
            return False
        if self.healthy_since is None:
            self.healthy_since = now
        return now-self.healthy_since >= self.stable_s
