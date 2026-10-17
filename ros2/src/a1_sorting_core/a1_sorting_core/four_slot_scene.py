"""Collision objects from candidate SDF, never visual-only approximations."""
from pathlib import Path
import xml.etree.ElementTree as E
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

def collision_objects(path):
    out=[]
    def xyz(e):return [float(x) for x in e.text.split()] if e is not None else [0.]*6
    for m in E.parse(path).findall('.//model'):
        name=m.get('name','')
        if name not in ('table','tabletop','belt','belt_side_rails','stop_plate','pusher','camera_pole') and not name.startswith('bin_'):continue
        for l in m.findall('link'):
            for c in l.findall('collision'):
                poses=[xyz(x.find('pose')) for x in (m,l,c)]
                if any(any(abs(v)>1e-12 for v in p[3:]) for p in poses):raise ValueError('rotated primitive not implemented, refuse approximate scene')
                o=CollisionObject();o.header.frame_id='world';o.id=name+'_'+c.get('name');o.operation=o.ADD
                shape=list(c.find('geometry'))[0];p=SolidPrimitive()
                if shape.tag=='box':p.type=p.BOX;p.dimensions=list(map(float,shape.find('size').text.split()))
                elif shape.tag=='cylinder':p.type=p.CYLINDER;p.dimensions=[float(shape.find('length').text),float(shape.find('radius').text)]
                else:raise ValueError('unsupported exact primitive')
                pose=Pose();pose.position.x,pose.position.y,pose.position.z=[sum(p[i] for p in poses) for i in range(3)];pose.orientation.w=1.;o.primitives=[p];o.primitive_poses=[pose];out.append(o)
    return out
