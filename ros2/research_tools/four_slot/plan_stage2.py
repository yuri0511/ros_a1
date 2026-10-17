"""Actual Pilz planning and FCL state checks. No execution client, no Gazebo."""
import copy,json,time,xml.etree.ElementTree as E
from pathlib import Path
import numpy as np,yaml,rclpy
from moveit_msgs.srv import GetMotionPlan,ApplyPlanningScene,GetStateValidity
from moveit_msgs.msg import Constraints,JointConstraint,CollisionObject,AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from rosidl_runtime_py.convert import message_to_ordereddict as dictmsg
from geometry import ROOT,ROBOT,SUFFIXES,fk,origin
W=Path(__file__).resolve().parents[2];cfg=json.loads((W/'src/a1_sorting_core/config/four_slot_candidate.json').read_text());base=yaml.safe_load((W/'src/a1_sorting_core/config/sorting_params.yaml').read_text())
allnames=[j.get('name') for j in ROBOT.findall('joint') if j.get('type')!='fixed'];initial={n:0. for n in allnames}
for node,prefix in [('/pick_place_executor',''),('/pick_place_relay','arm2_')]:initial.update(zip([prefix+n for n in SUFFIXES],base[node]['ros__parameters']['standby_joints']))
for n in allnames:
    if 'finger' in n:initial[n]=-.003
rclpy.init();node=rclpy.create_node('four_slot_plan_only');planner=node.create_client(GetMotionPlan,'/plan_kinematic_path');scene=node.create_client(ApplyPlanningScene,'/apply_planning_scene');valid=node.create_client(GetStateValidity,'/check_state_validity')
def rpc(client,req,timeout=15):
    if not client.wait_for_service(timeout_sec=15):raise RuntimeError('service unavailable')
    f=client.call_async(req);rclpy.spin_until_future_complete(node,f,timeout_sec=timeout)
    if not f.done():raise RuntimeError('request timeout')
    return f.result()
def box(name,xyz,size):
    o=CollisionObject();o.header.frame_id='world';o.id=name;o.operation=o.ADD;p=SolidPrimitive();p.type=p.BOX;p.dimensions=[float(x) for x in size];pose=Pose();pose.position.x,pose.position.y,pose.position.z=map(float,xyz);pose.orientation.w=1.;o.primitives=[p];o.primitive_poses=[pose];return o
obstacles=[]
world=E.parse(W/'src/a1_cell_sim/worlds/four_slot_1ms.sdf')
for m in world.findall('.//model'):
    name=m.get('name','')
    if name not in ('table','tabletop','belt','belt_side_rails','stop_plate','pusher','camera_pole') and not name.startswith('bin_'):continue
    for link in m.findall('link'):
        for c in link.findall('collision'):
            geom=list(c.find('geometry'))[0];M=origin(m.find('pose'),True)@origin(link.find('pose'),True)@origin(c.find('pose'),True)
            if geom.tag=='box':o=box(name+'_'+c.get('name'),M[:3,3],list(map(float,geom.find('size').text.split())))
            elif geom.tag=='cylinder':
                o=box(name+'_'+c.get('name'),M[:3,3],[1,1,1]);o.primitives[0].type=SolidPrimitive.CYLINDER;o.primitives[0].dimensions=[float(geom.find('length').text),float(geom.find('radius').text)]
            else:continue
            # All current selected primitives have identity world rotation.
            if not np.allclose(M[:3,:3],np.eye(3)):raise RuntimeError('rotated obstacle requires quaternion implementation')
            obstacles.append(o)
req=ApplyPlanningScene.Request();req.scene.is_diff=True;req.scene.world.collision_objects=obstacles
assert rpc(scene,req).success
(ROOT/'audit/stage2/planning-scene.json').write_text(json.dumps(dictmsg(req),indent=2))
output=[]
AUDIT=ROOT/'audit/stage2'
starts=json.loads((AUDIT/'historical-carry-clear.json').read_text())
try:
    for bn,b in cfg['bins'].items():
        req=ApplyPlanningScene.Request();req.scene.is_diff=True
        req.scene.world.collision_objects=[box('stored_'+x['id'],x['xyz'],[.05]*3) for x in b['slots']]
        assert rpc(scene,req).success
        for slot in b['slots']:
            arm=slot['arm'];prefix='' if arm=='arm1' else 'arm2_';names=[prefix+n for n in SUFFIXES];standby=[initial[n] for n in names];wp=slot['waypoints']
            # Worst occupancy: all 3 other slots in active bin. No deletion/reuse.
            req=ApplyPlanningScene.Request();req.scene.is_diff=True
            for other in b['slots']:
                o=box('stored_'+other['id'],other['xyz'],[.05]*3)
                if other['id']==slot['id']:o.operation=o.REMOVE
                req.scene.world.collision_objects.append(o)
            assert rpc(scene,req).success
            stages=[(f'carry_clear_{i}',r['positions'],wp['carry'],-.003) for i,r in enumerate(starts) if r['arm']==arm]+[('settle_top',wp['carry'],wp['settle_top'],-.003),('settle_bottom',wp['settle_top'],wp['settle_bottom'],-.003),('rise',wp['settle_bottom'],wp['carry'],.010),('return',wp['carry'],standby,.04)]
            for label,start,target,fingers in stages:
                vals=dict(initial);vals.update(zip(names,start))
                for finger in ('gripper_finger_left_joint','gripper_finger_right_joint'):vals[prefix+finger]=fingers
                occupancy=ApplyPlanningScene.Request();occupancy.scene.is_diff=True
                own=box('stored_'+slot['id'],slot['xyz'],[.05]*3)
                if label in ('rise','return'):
                    occupancy.scene.world.collision_objects=[own];assert rpc(scene,occupancy).success
                q=GetMotionPlan.Request().motion_plan_request;q.group_name=prefix+'ur_manipulator';q.pipeline_id='pilz_industrial_motion_planner';q.planner_id='LIN' if label=='rise' else 'PTP';q.num_planning_attempts=1;q.allowed_planning_time=3.;q.max_velocity_scaling_factor=.2;q.max_acceleration_scaling_factor=.2
                q.start_state.joint_state.name=allnames;q.start_state.joint_state.position=[float(vals[n]) for n in allnames];q.start_state.joint_state.velocity=[0.]*len(allnames)
                if label not in ('rise','return'):
                    attached=AttachedCollisionObject();attached.link_name=prefix+'tool0';attached.object=box('held_candidate',[0,0,.204],[.05]*3);attached.object.header.frame_id=prefix+'tool0'
                    attached.touch_links=[prefix+n for n in ('tool0','gripper_base_link','gripper_finger_left_link','gripper_finger_right_link')];q.start_state.attached_collision_objects=[attached]
                con=Constraints();con.joint_constraints=[JointConstraint(joint_name=n,position=float(x),tolerance_above=.01,tolerance_below=.01,weight=1.) for n,x in zip(names,target)];q.goal_constraints=[con]
                req=GetMotionPlan.Request();req.motion_plan_request=q;response=rpc(planner,req)
                states=[]
                for state_name,js in [('start',start),('goal',target)]:
                    v=GetStateValidity.Request();v.robot_state=copy.deepcopy(q.start_state);v.robot_state.joint_state.position=[float(dict(vals,**dict(zip(names,js)))[n]) for n in allnames];v.group_name=''
                    states.append({'name':state_name,'response':dictmsg(rpc(valid,v))})
                row={'slot':slot['id'],'stage':label,'fingers':fingers,'request':dictmsg(req),'response':dictmsg(response),'state_validity':states};output.append(row)
                (AUDIT/'plans.json').write_text(json.dumps(output))
                print(slot['id'],label,response.motion_plan_response.error_code.val,[x['response']['valid'] for x in states],flush=True)
            # Remove current bin inventory before moving to a different bin.
        req=ApplyPlanningScene.Request();req.scene.is_diff=True
        for other in b['slots']:
            o=box('stored_'+other['id'],other['xyz'],[.05]*3);o.operation=o.REMOVE;req.scene.world.collision_objects.append(o)
        assert rpc(scene,req).success
finally:node.destroy_node();rclpy.try_shutdown()
