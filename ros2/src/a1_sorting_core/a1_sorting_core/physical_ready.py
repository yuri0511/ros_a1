"""Fail-closed, named-joint physical READY evidence. No ROS or motion commands."""
from collections import deque
import math
import threading

class PhysicalReadyMonitor:
    # Standby uses the existing MoveIt joint-goal .01rad tolerance.
    # Keep original .3rad/s and .5wall freshness limits; never widen.
    def __init__(self, window=.25, min_samples=50, pos_tol=.01, vel_tol=.3,
                 freshness=.5, max_gap=.05, future_tol=.01):
        self.window=window; self.min_samples=min_samples; self.pos_tol=pos_tol
        self.vel_tol=vel_tol; self.freshness=freshness; self.max_gap=max_gap
        self.future_tol=future_tol; self._samples=deque(maxlen=2000)
        self._lock=threading.RLock(); self._last_stamp=None
        self._fault=None; self._not_before=-math.inf
        self.duplicate_stamps=0; self._last_issue=None

    def begin(self, now_wall):
        with self._lock:self._not_before=now_wall

    def observe(self, names, positions, velocities, stamp, receipt_wall):
        with self._lock:
            if (len(names)!=len(set(names)) or len(names)!=len(positions)
                    or len(names)!=len(velocities)):
                self._fault='ambiguous_or_incomplete_joint_state';return
            if not all(math.isfinite(float(x)) for x in [stamp,receipt_wall,*positions,*velocities]):
                self._fault='nonfinite_joint_state';return
            if self._last_stamp is not None and stamp<self._last_stamp:
                self._fault='backward_joint_stamp';return
            if self._last_stamp is not None and stamp==self._last_stamp:
                # Repeated simulation stamps are observed in this broadcaster.
                # They never count as new evidence, refresh age or bridge a window.
                self._samples.clear();self.duplicate_stamps+=1
                self._last_issue='duplicate_joint_stamp';return
            self._last_issue=None
            self._last_stamp=stamp
            self._samples.append((stamp,receipt_wall,dict(zip(names,positions)),dict(zip(names,velocities))))

    def evaluate(self, names, targets, now_sim, now_wall, controller_ok, no_active_goal):
        with self._lock:
            info={'ready':False,'reason':self._last_issue or 'insufficient_samples','samples':0,'duplicate_stamps':self.duplicate_stamps}
            if self._fault:return dict(info,reason=self._fault)
            if not controller_ok:return dict(info,reason='controller_not_successful')
            if not no_active_goal:return dict(info,reason='controller_goal_active_or_unresolved')
            if len(names)!=len(targets) or not names:return dict(info,reason='invalid_target')
            xs=[s for s in self._samples if s[1]>=self._not_before]
            if not xs:return info
            stamp,wall,ps,vs=xs[-1]
            info.update(last_stamp=stamp,receipt_age=now_wall-wall,sim_age=now_sim-stamp)
            if not 0<=now_wall-wall<=self.freshness:return dict(info,reason='stale_joint_receipt')
            if now_sim-stamp>self.freshness or stamp-now_sim>self.future_tol:return dict(info,reason='stale_or_future_joint_stamp')
            good=[];reason='insufficient_samples';previous=None
            for sample in xs:
                t,w,p,v=sample
                if any(n not in p or n not in v for n in names):return dict(info,reason='missing_named_joint')
                speed=max(abs(v[n]) for n in names);error=max(abs(p[n]-q) for n,q in zip(names,targets))
                diff=None
                if previous:
                    dt=t-previous[0]
                    if dt>self.max_gap:good=[];reason='joint_sample_gap'
                    diff=max(abs(p[n]-previous[2][n])/dt for n in names)
                if speed>self.vel_tol or error>self.pos_tol or (diff is not None and diff>self.vel_tol):
                    good=[];reason='joint_not_stationary_or_at_target'
                else:good.append(sample)
                previous=sample
                info.update(max_reported_speed=speed,max_position_error=error,max_difference_speed=diff)
            span=good[-1][0]-good[0][0] if good else 0
            info.update(samples=len(good),stable_span=span)
            ready=len(good)>=self.min_samples and span>=self.window
            return dict(info,ready=ready,reason='physical_ready' if ready else reason)
