"""Exact original URDF forward kinematics; no physics/control/ROS connections."""
from pathlib import Path
import math, xml.etree.ElementTree as ET
import numpy as np
ROOT = Path(__file__).resolve().parents[4]
ROBOT = ET.parse(ROOT/'audit/robot-static.urdf').getroot()
PARENTS = {j.find('child').get('link'):j for j in ROBOT.findall('joint')}
SUFFIXES = ('shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint')
ARM2 = ['arm2_'+n for n in SUFFIXES]
def rot(axis, angle):
    v=np.array(axis,dtype=float);v/=np.linalg.norm(v)
    K=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
    return np.eye(3)+math.sin(angle)*K+(1-math.cos(angle))*(K@K)
def origin(x, sdf=False):
    M=np.eye(4)
    if x is None:return M
    v=list(map(float,(x.text if sdf else x.get('xyz','0 0 0')).split()))
    M[:3,3]=v[:3]
    rpy=v[3:] if sdf else list(map(float,x.get('rpy','0 0 0').split()))
    if len(rpy)<3:rpy=[0.,0.,0.]
    r,p,y=rpy;M[:3,:3]=rot([0,0,1],y)@rot([0,1,0],p)@rot([1,0,0],r)
    return M
def fk(link, values):
    cache={}
    def visit(child):
        if child in cache:return cache[child]
        if child not in PARENTS:return np.eye(4)
        j=PARENTS[child];M=visit(j.find('parent').get('link'))@origin(j.find('origin'))
        J=np.eye(4);a=j.find('axis');axis=list(map(float,a.get('xyz','1 0 0').split())) if a is not None else [1,0,0]
        q=values.get(j.get('name'),0.)
        if j.get('type') in ('revolute','continuous'):J[:3,:3]=rot(axis,q)
        elif j.get('type')=='prismatic':J[:3,3]=np.array(axis)*q
        cache[child]=M@J;return cache[child]
    return visit(link)


def quaternion_matrix(q):
    x,y,z,w=map(float,q)
    if abs(x*x+y*y+z*z+w*w-1)>1e-6:raise ValueError('nonunit quaternion')
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def matrix_quaternion(R):
    if not np.allclose(R.T@R,np.eye(3),atol=1e-10) or abs(np.linalg.det(R)-1)>1e-10:
        raise ValueError('invalid rotation matrix')
    a,b,c=R[0];d,e,f=R[1];g,h,i=R[2]
    K=np.array([[a-e-i,b+d,c+g,h-f],[b+d,e-a-i,f+h,c-g],
                [c+g,f+h,i-a-e,d-b],[h-f,c-g,d-b,a+e+i]])/3
    _,V=np.linalg.eigh(K);q=V[:,-1]
    return q if q[3]>=0 else -q
