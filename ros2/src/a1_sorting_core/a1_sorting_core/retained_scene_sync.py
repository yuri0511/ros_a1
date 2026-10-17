"""Measured retained objects -> ApplyPlanningScene ACK -> read-back evidence.

Never remove an obstacle because a frame loses sight of it. An incomplete,
stale or unacknowledged snapshot must block the next planning request.
"""
import math,time
from moveit_msgs.srv import ApplyPlanningScene,GetPlanningScene
from moveit_msgs.msg import CollisionObject,PlanningSceneComponents
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

class SceneSyncError(RuntimeError):pass

def observed_object(part,position,quaternion,stamp,now):
    if len(position)!=3 or len(quaternion)!=4 or not all(math.isfinite(x) for x in (*position,*quaternion,stamp,now)):
        raise SceneSyncError('invalid_observed_pose')
    if not 0<=now-stamp<=.2:raise SceneSyncError('stale_or_future_object')
    if abs(sum(x*x for x in quaternion)-1)>1e-6:raise SceneSyncError('invalid_orientation')
    o=CollisionObject();o.id='retained_'+part;o.header.frame_id='world';o.operation=o.ADD;o.pose.orientation.w=1.
    shape=SolidPrimitive();shape.type=shape.BOX;shape.dimensions=[.05,.05,.05]
    pose=Pose();pose.position.x,pose.position.y,pose.position.z=map(float,position)
    pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=map(float,quaternion)
    o.primitives=[shape];o.primitive_poses=[pose];return o

def signature(o):
    if o.header.frame_id!='world' or len(o.primitives)!=1 or len(o.primitive_poses)!=1:raise SceneSyncError('readback_shape_changed')
    world=camera_observation(o.pose,o.primitive_poses[0],0.)
    return (o.primitives[0].type,*o.primitives[0].dimensions,*world['position'],*world['quaternion'])

class RetainedSceneSync:
    def __init__(self,apply,read):self.apply=apply;self.read=read;self.revision=0;self.ack=None
    def sync(self,inventory,observations,now):
        self.ack=None;objects=[]
        for sid,row in inventory.items():
            if row['state'] not in ('EMPTY','RESERVED','OCCUPIED','FAULT','RELEASED_UNCONFIRMED'):
                raise SceneSyncError('unknown_inventory_state')
            if row['state']=='FAULT':raise SceneSyncError('quarantined_inventory')
            if row['state'] not in ('OCCUPIED','RELEASED_UNCONFIRMED'):continue
            part=row['part']
            if part not in observations:raise SceneSyncError('occupied_part_not_observed')
            v=observations[part];objects.append(observed_object(part,v['position'],v['quaternion'],v['stamp'],now))
        if objects:
            if self.apply(objects) is not True:raise SceneSyncError('planning_scene_not_acknowledged')
            seen={o.id:o for o in self.read()}
            for o in objects:
                if o.id not in seen:raise SceneSyncError('planning_scene_readback_missing')
                # 64 IEEE-754 ulps at unit scale cover quaternion composition roundoff.
                # This is ~1.4e-14 m, not a physical clearance allowance.
                import sys
                expected,actual=signature(o),signature(seen[o.id])
                if len(expected)!=len(actual) or any(abs(a-b)>64*sys.float_info.epsilon*max(1.,abs(a),abs(b)) for a,b in zip(expected,actual)):
                    raise SceneSyncError('planning_scene_readback_mismatch')
        self.revision+=1;self.ack={'revision':self.revision,'objects':[o.id for o in objects],'sim':now};return self.ack

class RosRetainedSceneSync(RetainedSceneSync):
    def __init__(self,node):
        self.node=node;self.apply_client=node.create_client(ApplyPlanningScene,'/apply_planning_scene');self.read_client=node.create_client(GetPlanningScene,'/get_planning_scene')
        super().__init__(self._apply,self._read)
    def _call(self,client,request):
        if not client.wait_for_service(timeout_sec=1.):raise SceneSyncError('scene_service_missing')
        future=client.call_async(request);deadline=time.monotonic()+2.
        while not future.done() and time.monotonic()<deadline:time.sleep(.01)
        if not future.done():future.cancel();raise SceneSyncError('scene_service_timeout')
        return future.result()
    def _apply(self,objects):
        q=ApplyPlanningScene.Request();q.scene.is_diff=True;q.scene.world.collision_objects=objects
        return self._call(self.apply_client,q).success
    def _read(self):
        q=GetPlanningScene.Request();q.components.components=PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        return self._call(self.read_client,q).scene.world.collision_objects

def camera_observation(camera,relative,stamp):
    """Compose measured sensor world pose with measured object sensor pose."""
    import numpy as np
    def parts(p):
        q=np.array([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w],dtype=float)
        if not np.all(np.isfinite(q)) or abs(float(q@q)-1)>1e-6:raise SceneSyncError('camera_quaternion_invalid')
        return np.array([p.position.x,p.position.y,p.position.z]),q
    t,a=parts(camera);v,b=parts(relative);x,y,z,w=a
    rotation=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    xyz=t+rotation@v;q=np.r_[a[3]*b[:3]+b[3]*a[:3]+np.cross(a[:3],b[:3]),a[3]*b[3]-a[:3]@b[:3]]
    return {'position':xyz.tolist(),'quaternion':q.tolist(),'stamp':stamp}
