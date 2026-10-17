"""Read-only native evidence for the isolated fixture/support transition."""
from collections import deque
import math
import sys
EPS=64*sys.float_info.epsilon


def fixture(name):return name.startswith('calibration_red4::')
def red_support(name):return name.startswith('tabletop::') and name.endswith('::floor_red')
def finger(name):return any('arm2_gripper_finger_'+side+'_link::' in name for side in ('left','right'))


def box_bottom(position,quaternion):
    x,y,z,w=quaternion
    if not all(math.isfinite(v) for v in (*position,*quaternion)) or abs(sum(v*v for v in quaternion)-1)>1e-6:
        raise ValueError('invalid_native_fixture_pose')
    return position[2]-.025*(abs(2*(x*z-y*w))+abs(2*(y*z+x*w))+abs(1-2*(x*x+y*y)))


class SupportEvidence:
    def __init__(self):
        self.contacts=deque(maxlen=2000);self.poses=deque(maxlen=200)
        self.support=None;self.native=None;self.separated=False

    def contact(self,a,b,depths,normals,sim,wall,permit):
        if not (fixture(a) or fixture(b)):return None
        other=b if fixture(a) else a
        if not depths or not all(math.isfinite(d) for d in depths):return 'invalid_fixture_contact_depth'
        depth=max(depths)
        if depth<=0:return None
        if red_support(other):
            vertical=bool(normals) and all(all(math.isfinite(v) for v in n) and abs(n[0])<=EPS and abs(n[1])<=EPS and abs(abs(n[2])-1)<=EPS for n in normals)
            if self.contacts and sim<self.contacts[-1][0]:return 'support_contact_time_reversed'
            self.contacts.append((sim,depth))
            while self.contacts and sim-self.contacts[0][0]>.1:self.contacts.popleft()
            recent=list(self.contacts)
            self.support=dict(sim=sim,wall=wall,depth=depth,normal_vertical=vertical,
                              recent_count=len(recent),recent_span=sim-recent[0][0],recent_max_depth=max(x[1] for x in recent))
            if not vertical:return 'nonvertical_support_contact'
            phase=permit.get('phase')
            if phase=='free' or (self.separated and phase=='departing'):return 'support_recontact_after_separation'
            if phase in ('departing','landing','supported_release') and depth>permit['max_support_depth']+EPS:
                return 'native_support_penetration_grows'
        elif not finger(other):return 'unexpected_fixture_obstacle_contact'
        return None

    def pose(self,position,quaternion,sim,wall,permit):
        bottom=box_bottom(position,quaternion);self.poses.append((sim,bottom))
        while self.poses and sim-self.poses[0][0]>.1:self.poses.popleft()
        recent=list(self.poses)
        self.native=dict(sim=sim,wall=wall,position=list(position),quaternion=list(quaternion),bottom=bottom,recent_min_bottom=min(x[1] for x in recent))
        phase=permit.get('phase');top=permit.get('floor_top',.72)
        if phase=='landing':self.separated=False
        if phase in ('departing','free') and bottom>top+EPS:self.separated=True
        if phase in ('departing','landing','supported_release') and bottom<permit['native_floor_min']-EPS:
            return 'native_cube_bottom_penetration_grows'
        if phase=='free' and bottom<=top:return 'native_cube_recontacts_support'
        return None
