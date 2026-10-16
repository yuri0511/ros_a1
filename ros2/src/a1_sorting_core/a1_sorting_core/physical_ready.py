"""Named-joint physical READY evidence, independent of ROS and motion commands.

A header stamp identifies one observation bucket, not every controller update.
Repeated stamps neither count nor refresh freshness. All payloads in a bucket
are retained as position extrema / maximum speed so deduplication cannot hide
motion. Only strictly advancing stamps contribute to the stability window.
"""
from collections import deque
import math
import threading


def publisher_identity(info, publishers=None):
    """Use DDS GID when exposed, otherwise require exactly one graph endpoint.

    This installed Jazzy build exposes timestamps/sequence numbers but no GID
    in callback info. A single graph endpoint is the only unambiguous source;
    never invent identity from receive time or merge multiple publishers.
    """
    gid = info.get('publisher_gid') if isinstance(info, dict) else getattr(info, 'publisher_gid', None)
    if gid is None:
        if publishers is None or len(publishers) != 1:
            raise ValueError('joint publisher source is not unique')
        gid = publishers[0].endpoint_gid
    identity = bytes(gid).hex()
    if not identity or not any(bytes(gid)):
        raise ValueError('missing DDS publisher identity')
    return identity


class PhysicalReadyMonitor:
    def __init__(self, window=.25, min_samples=50, pos_tol=.01, vel_tol=.3,
                 freshness=.5, max_gap=.05, future_tol=.01):
        self.window = window
        self.min_samples = min_samples
        self.pos_tol = pos_tol
        self.vel_tol = vel_tol
        self.freshness = freshness
        self.max_gap = max_gap
        self.future_tol = future_tol
        self._samples = deque(maxlen=2000)
        self._lock = threading.RLock()
        self._last_stamp = None
        self._source = None
        self._fault = None
        self._not_before = -math.inf
        self._last_clock = None
        self._clock_progress_wall = None
        self.duplicate_stamps = 0

    def invalidate(self, reason):
        with self._lock:
            self._fault = reason

    def begin(self, now_wall):
        """Begin evidence strictly after the completed controller Action."""
        with self._lock:
            self._not_before = now_wall

    def observe(self, names, positions, velocities, stamp, receipt_wall, source=None):
        with self._lock:
            if (len(names) != len(set(names)) or len(names) != len(positions)
                    or len(names) != len(velocities)):
                self._fault = 'ambiguous_or_incomplete_joint_state'
                return
            if not all(math.isfinite(float(x)) for x in
                       [stamp, receipt_wall, *positions, *velocities]):
                self._fault = 'nonfinite_joint_state'
                return
            if source is not None:
                if self._source is not None and source != self._source:
                    self._fault = 'joint_publisher_changed_or_multiple_sources'
                    return
                self._source = source
            if self._last_stamp is not None and stamp < self._last_stamp:
                self._fault = 'joint_stamp_reset' if stamp <= 0 else 'backward_joint_stamp'
                return
            p = dict(zip(names, positions))
            v = {n: abs(x) for n, x in zip(names, velocities)}
            if self._last_stamp is not None and stamp == self._last_stamp:
                self.duplicate_stamps += 1
                # Keep first receipt time and original stamp; do not add samples.
                # Distinct same-stamp payloads still constrain safety for this bucket.
                old = self._samples[-1]
                if p.keys() != old[2].keys():
                    self._fault = 'same_stamp_joint_set_changed'
                    return
                for n in names:
                    old[2][n] = min(old[2][n], p[n])
                    old[3][n] = max(old[3][n], p[n])
                    old[4][n] = max(old[4][n], v[n])
                return
            self._last_stamp = stamp
            self._samples.append((stamp, receipt_wall, p.copy(), p.copy(), v))

    def evaluate(self, names, targets, now_sim, now_wall, controller_ok, no_active_goal):
        with self._lock:
            info = dict(ready=False, reason='insufficient_samples', samples=0,
                        duplicate_stamps=self.duplicate_stamps, source=self._source)
            if not math.isfinite(now_sim) or not math.isfinite(now_wall):
                self._fault = 'nonfinite_clock'
            elif self._last_clock is not None and now_sim < self._last_clock:
                self._fault = 'simulation_reset' if now_sim <= 0 else 'simulation_clock_backward'
            elif self._last_clock is None or now_sim > self._last_clock:
                self._clock_progress_wall = now_wall
            self._last_clock = now_sim
            if self._fault:
                return dict(info, reason=self._fault)
            if now_wall - self._clock_progress_wall > self.freshness:
                # A paused clock provides no advancing evidence. Resuming requires
                # an entirely new window; it does not clear any latched fault.
                self._not_before = max(self._not_before, now_wall)
                return dict(info, reason='simulation_clock_paused')
            if not controller_ok:
                return dict(info, reason='controller_not_successful')
            if not no_active_goal:
                return dict(info, reason='controller_goal_active_or_unresolved')
            if len(names) != len(targets) or not names:
                return dict(info, reason='invalid_target')
            xs = [s for s in self._samples if s[1] >= self._not_before]
            if not xs:
                return info
            stamp, wall, _, _, _ = xs[-1]
            info.update(last_stamp=stamp, receipt_age=now_wall-wall, sim_age=now_sim-stamp)
            if not 0 <= now_wall-wall <= self.freshness:
                return dict(info, reason='stale_joint_receipt')
            if now_sim-stamp > self.freshness or stamp-now_sim > self.future_tol:
                return dict(info, reason='stale_or_future_joint_stamp')
            good = []
            reason = 'insufficient_samples'
            previous = None
            for sample in xs:
                t, _, low, high, velocity = sample
                if any(n not in low for n in names):
                    return dict(info, reason='missing_named_joint')
                speed = max(velocity[n] for n in names)
                error = max(max(abs(low[n]-q), abs(high[n]-q))
                            for n, q in zip(names, targets))
                diff = None
                if previous is not None:
                    dt = t-previous[0]  # strictly positive by construction
                    if dt > self.max_gap:
                        good = []
                        reason = 'joint_sample_gap'
                    # Conservative cross-bucket position differences; no invented
                    # dt for same-stamp samples, and no wall-time substitution.
                    diff = max(max(abs(high[n]-previous[2][n]),
                                   abs(low[n]-previous[3][n]))/dt for n in names)
                if (speed > self.vel_tol or error > self.pos_tol
                        or (diff is not None and diff > self.vel_tol)):
                    good = []
                    reason = 'joint_not_stationary_or_at_target'
                else:
                    good.append((sample, speed, error, diff))
                previous = sample
                info.update(max_reported_speed=speed, max_position_error=error,
                            max_difference_speed=diff)
            span = good[-1][0][0]-good[0][0][0] if good else 0
            info.update(samples=len(good), stable_span=span)
            if good:
                info.update(max_reported_speed=max(x[1] for x in good),
                            max_position_error=max(x[2] for x in good),
                            max_difference_speed=max((x[3] for x in good if x[3] is not None), default=0))
            ready = len(good) >= self.min_samples and span >= self.window
            return dict(info, ready=ready, reason='physical_ready' if ready else reason)
