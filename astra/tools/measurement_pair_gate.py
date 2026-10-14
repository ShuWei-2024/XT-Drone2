"""Bound callback-order skew without widening the measurement pairing tolerance."""
import math


def recent_measurement_pair(measurements, local_history, now_ns, z_reset_counter=None):
    """Newest actual pair within 20 ms, no older than the 250 ms input limit.

    A missed PX4 sample need not invalidate a still-fresh earlier pair. Never
    interpolate, restamp, or use arbitrarily old pairs to cover a source outage.
    """
    if z_reset_counter is not None:
        local_history=[p for p in local_history if p.z_reset_counter==z_reset_counter]
    if not local_history:
        return None
    for stamp, z in reversed(measurements):
        if not math.isfinite(z) or not 0 <= now_ns - stamp <= 250_000_000:
            continue
        matched = min(local_history, key=lambda p: abs(p.timestamp_sample*1000-stamp))
        if (matched.z_valid and math.isfinite(matched.z) and
                0 <= now_ns-matched.timestamp_sample*1000 <= 250_000_000
                and abs(matched.timestamp_sample*1000-stamp) <= 20_000_000):
            return stamp, z, matched
    return None


class MeasurementPairGate:
    def __init__(self):
        self.cached=None
        self.pending_since=None
        self.deferred_checks=0

    def select(self, candidate, now_ns, wall_s):
        if candidate is not None:
            age=(now_ns-candidate[0])/1e9
            if not 0<=age<=.25:return None
            self.cached=candidate
            self.pending_since=None
            return candidate
        if self.pending_since is None:self.pending_since=wall_s
        # New unmatched frames must not restart the deadline indefinitely.
        if self.cached is None or wall_s-self.pending_since>.02:return None
        age=(now_ns-self.cached[0])/1e9
        if not 0<=age<=.25:return None
        self.deferred_checks+=1
        return self.cached
