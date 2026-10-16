"""GetMotionPlan ONLY. Never constructs an execution/control client."""
from pathlib import Path
import json,time,hashlib,xml.etree.ElementTree as ET,argparse
import yaml,rclpy
from moveit_msgs.srv import GetMotionPlan,ApplyPlanningScene
from moveit_msgs.msg import Constraints,JointConstraint,CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from rosidl_runtime_py.convert import message_to_ordereddict
from geometry import ROOT,ROBOT,SUFFIXES,ARM2,origin
R=Path(__file__).resolve().parents[1]
W=R/'worktree/ros2'
cfg=yaml.safe_load((W/'src/a1_sorting_core/config/sorting_params_pipeline.yaml').read_text())
p=cfg['/pick_place_relay']['ros__parameters']
base=yaml.safe_load((W/'src/a1_sorting_core/config/sorting_params.yaml').read_text())
standby1=base['/pick_place_executor']['ros__parameters']['standby_joints']
standby2=base['/pick_place_relay']['ros__parameters']['standby_joints']
allnames=[j.get('name') for j in ROBOT.findall('joint') if j.get('type')!='fixed']
initial={n:0. for n in allnames};initial.update(zip(SUFFIXES,standby1));initial.update(zip(ARM2,standby2))
for n in allnames:
    if 'finger' in n:initial[n]=-.003
parser=argparse.ArgumentParser();parser.add_argument('--walls',action='store_true');args=parser.parse_args()
output='planned-with-walls.json' if args.walls else 'planned-trajectories.json'
rclpy.init();node=rclpy.create_node('green_third_plan_only_client')
client=node.create_client(GetMotionPlan,'/plan_kinematic_path')
if not client.wait_for_service(timeout_sec=25):raise RuntimeError('planner unavailable')
if args.walls:
    scene=node.create_client(ApplyPlanningScene,'/apply_planning_scene')
    if not scene.wait_for_service(timeout_sec=5):raise RuntimeError('scene unavailable')
    req=ApplyPlanningScene.Request();req.scene.is_diff=True
    world=ET.parse(W/'src/a1_cell_sim/worlds/fast_sim_1ms.sdf').getroot()
    for model in world.findall('.//model'):
        if not model.get('name','').startswith('bin_'):continue
        for link in model.findall('link'):
            for c in link.findall('collision'):
                size=c.find('geometry/box/size')
                if size is None:continue
                o=CollisionObject();o.header.frame_id='world';o.id=model.get('name')+'_'+c.get('name');o.operation=CollisionObject.ADD
                primitive=SolidPrimitive();primitive.type=SolidPrimitive.BOX;primitive.dimensions=list(map(float,size.text.split()))
                M=origin(model.find('pose'),True)@origin(link.find('pose'),True)@origin(c.find('pose'),True)
                pose=Pose();pose.position.x=float(M[0,3]);pose.position.y=float(M[1,3]);pose.position.z=float(M[2,3]);pose.orientation.w=1.
                o.primitives=[primitive];o.primitive_poses=[pose];req.scene.world.collision_objects.append(o)
    f=scene.call_async(req);rclpy.spin_until_future_complete(node,f,timeout_sec=5)
    if not f.done() or not f.result().success:raise RuntimeError('scene apply failed')
    (R/'audit/planning-scene-exact-walls.json').write_text(json.dumps(message_to_ordereddict(req),indent=2))
out=[]
try:
    for slot in (4,5,6):
        carry=p[f'carry_anchor{slot}_joints'];top=p[f'carry_settle_top{slot}_joints'];bot=p[f'carry_settle_bottom{slot}_joints']
        ladder=[[a+(b-a)*i/3 for a,b in zip(top,bot)] for i in range(4)]
        points=[('approach',standby2,carry),('settle_top',carry,top)]
        points.extend((f'settle_{i}',a,b) for i,(a,b) in enumerate(zip(ladder,ladder[1:]),1))
        points.extend([('retreat',bot,carry),('return',carry,standby2)])
        for label,start,target in points:
            req=GetMotionPlan.Request();q=req.motion_plan_request
            q.group_name='arm2_ur_manipulator';q.pipeline_id='pilz_industrial_motion_planner';q.planner_id='PTP'
            q.num_planning_attempts=1;q.allowed_planning_time=5.
            q.max_velocity_scaling_factor=.2;q.max_acceleration_scaling_factor=.2
            vals=dict(initial);vals.update(zip(ARM2,start))
            q.start_state.joint_state.name=allnames
            q.start_state.joint_state.position=[float(vals[n]) for n in allnames]
            q.start_state.joint_state.velocity=[0.]*len(allnames)
            c=Constraints()
            for n,x in zip(ARM2,target):
                c.joint_constraints.append(JointConstraint(joint_name=n,position=float(x),tolerance_above=.01,tolerance_below=.01,weight=1.))
            q.goal_constraints=[c]
            f=client.call_async(req);rclpy.spin_until_future_complete(node,f,timeout_sec=15)
            if not f.done():raise TimeoutError(label)
            response=f.result();d={'slot':slot-3,'label':label,'request':message_to_ordereddict(req),
                'response':message_to_ordereddict(response)}
            out.append(d);(R/'audit'/output).write_text(json.dumps(out,indent=2))
            print(slot-3,label,response.motion_plan_response.error_code.val,len(response.motion_plan_response.trajectory.joint_trajectory.points),flush=True)
finally:
    node.destroy_node();rclpy.try_shutdown()
