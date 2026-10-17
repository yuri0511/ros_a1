"""Read every unique callback, rather than polling the latest JointState."""
from collections import deque
from threading import Lock
import math

class LockWindow:
    def __init__(self, names):
        self.names=tuple(names); self.rows=deque(); self.lock=Lock()
        self.source=None; self.last=None; self.fault=None

    def observe(self, names, positions, stamp, source):
        with self.lock:
            if self.fault:return
            if self.source is None:self.source=source
            if source!=self.source:
                self.fault='locked_finger_source_changed';return
            values=dict(zip(names,positions))
            if any(n not in values for n in self.names) or not math.isfinite(stamp):
                self.fault='locked_finger_invalid_sample';return
            q=[values[n] for n in self.names]
            if not all(math.isfinite(x) for x in q):
                self.fault='locked_finger_invalid_sample';return
            if self.last is not None:
                if stamp==self.last:return
                if stamp<self.last:
                    self.fault='locked_finger_time_reversed';return
                if stamp-self.last>.05:
                    self.fault='locked_finger_source_gap';return
            self.last=stamp; self.rows.append((stamp,q))
            while self.rows and stamp-self.rows[0][0]>.12:self.rows.popleft()

    def snapshot(self):
        with self.lock:
            rows=[(t,list(q)) for t,q in self.rows]
            passed=(not self.fault and len(rows)>=50 and rows[-1][0]-rows[0][0]>=.1
                    and all(max(q[i] for _,q in rows)-min(q[i] for _,q in rows)<.002 for i in (0,1)))
            return bool(passed),rows,self.fault
