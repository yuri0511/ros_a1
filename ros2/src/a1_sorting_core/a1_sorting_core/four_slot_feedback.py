"""Read-only release evidence, metres and source simulation seconds.

This monitor never sends a gripper command. Bounds are the existing candidate
collision-audit assumptions, not newly calibrated actuator performance limits.
"""
import math,time

class ReleaseEvidence:
    def __init__(self, names, freshness=.5, max_gap=.05):
        self.names=tuple(names);self.freshness=freshness;self.max_gap=max_gap
        self.phase='IDLE';self.fault=None;self.last=None;self.samples=0;self.source=None
    def begin(self,wall=None):
        self.began=time.monotonic() if wall is None else wall
        self.phase='RELEASING';self.fault=None;self.last=None;self.samples=0;self.source=None
    def observe(self,names,positions,stamp,wall,source):
        if self.phase not in ('RELEASING','EXITING'):return
        if len(names)!=len(set(names)) or len(names)!=len(positions):self.fault='ambiguous_joint_names';return
        vals=dict(zip(names,positions))
        if not source or not all(n in vals for n in self.names):self.fault='missing_feedback';return
        q=tuple(vals[n] for n in self.names)
        if not all(math.isfinite(v) for v in (*q,stamp,wall)):self.fault='nonfinite';return
        if self.source and source!=self.source:self.fault='publisher_changed';return
        self.source=source
        lo=.008 if self.phase=='EXITING' else -.003
        if any(v<lo or v>.014 for v in q):self.fault='opening_envelope_exceeded'
        if self.last:
            if stamp<self.last[0]:self.fault='time_reversed';return
            if stamp==self.last[0]:return  # violations above still count; no freshness renewal
            if stamp-self.last[0]>self.max_gap:self.fault='observation_gap'
        self.last=(stamp,wall,q);self.samples+=1
    def check(self,sim,wall):
        if self.fault:return False
        if not self.last:
            if wall-self.began>self.freshness:self.fault='missing_feedback'
            return False
        if not 0<=sim-self.last[0]<=self.freshness or not 0<=wall-self.last[1]<=self.freshness:
            self.fault='stale_feedback';return False
        return True
    def begin_exit(self,sim,wall):
        if not self.check(sim,wall):return False
        if not all(.010<=q<=.014 for q in self.last[2]):self.fault='release_opening_not_reached';return False
        self.phase='EXITING';return True
    def finish(self,sim,wall,*,geometry_clearance_verified):
        if not self.check(sim,wall) or not geometry_clearance_verified:
            self.fault=self.fault or 'exit_clearance_unverified';return False
        self.phase='EXITED';return True
