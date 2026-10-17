"""Actual MoveIt replay of saved Gazebo state. No simulator or Action client."""
import copy, hashlib, json, os, subprocess, sys, threading, time
from pathlib import Path
import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, Constraints, JointConstraint, CollisionObject
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetStateValidity
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.convert import message_to_ordereddict as msgdict
from geometry import ROOT, SUFFIXES, fk, origin, ROBOT
from support_contact import SupportMatrix, OBJECT, SUPPORT, ROUND, cube_bounds, pose_matrix, support_profile, contact_landing_trajectory, verify_requested_direction
from diagnose_held import remove_char_operations
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'voice_four'))
from run_four import stop_group


def main():
    source=ROOT/'calibration_runs/C-20261017T102733-3d3e3a'
    run=ROOT/'support_transition'/('offline-'+time.strftime('%Y%m%dT%H%M%S'))
    run.mkdir(parents=True)
    os.environ.update(ROS_DOMAIN_ID='97', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_LOG_DIR=str(run/'ros'))
    log=(run/'planner.log').open('w')
    server=subprocess.Popen(['ros2','launch',str(Path(__file__).with_name('plan_only.launch.py'))],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=rclpy.create_node('support_transition_replay')
    ex=rclpy.executors.SingleThreadedExecutor();ex.add_node(node);thread=threading.Thread(target=ex.spin);thread.start()
    def wait(f,timeout=15):
        end=time.monotonic()+timeout
        while not f.done():
            if time.monotonic()>end:raise RuntimeError('RPC_timeout')
            time.sleep(.0005)
        return f.result()
    def client(cls,name):
        c=node.create_client(cls,name)
        if not c.wait_for_service(timeout_sec=15):raise RuntimeError('service_missing:'+name)
        return c
    output={'gazebo_started':False,'motion_sent':False,'source_run':str(source),'segments':[]}
    guard=None
    try:
        apply=client(ApplyPlanningScene,'/apply_planning_scene');plan=client(GetMotionPlan,'/plan_kinematic_path');valid=client(GetStateValidity,'/check_state_validity')
        data=json.loads((ROOT/'audit/stage2/planning-scene.json').read_text());remove_char_operations(data)
        q=ApplyPlanningScene.Request();set_message_fields(q,data)
        for o in q.scene.world.collision_objects:o.operation=o.ADD
        assert wait(apply.call_async(q)).success
        data=json.loads((source/'held-cycle.json').read_text())[0]['request'];remove_char_operations(data)
        original=GetMotionPlan.Request();set_message_fields(original,data)
        attached=original.motion_plan_request.start_state.attached_collision_objects[0];attached.object.operation=attached.object.ADD
        initial=dict(zip(original.motion_plan_request.start_state.joint_state.name,original.motion_plan_request.start_state.joint_state.position))
        # Test the real WORLD -> ATTACHED identity transition without dropping geometry.
        world=copy.deepcopy(attached.object);world.header.frame_id='world';world.pose=Pose();world.pose.orientation.w=1.
        M=fk(attached.link_name,initial)@pose_matrix(attached.object.pose)@pose_matrix(attached.object.primitive_poses[0])
        from geometry import matrix_quaternion
        world.primitive_poses[0].position.x,world.primitive_poses[0].position.y,world.primitive_poses[0].position.z=map(float,M[:3,3])
        quat=matrix_quaternion(M[:3,:3]);world.primitive_poses[0].orientation.x,world.primitive_poses[0].orientation.y,world.primitive_poses[0].orientation.z,world.primitive_poses[0].orientation.w=map(float,quat)
        q=ApplyPlanningScene.Request();q.scene.is_diff=True;q.scene.world.collision_objects=[world];assert wait(apply.call_async(q)).success
        guard=SupportMatrix(node,wait)
        q=ApplyPlanningScene.Request();q.scene.is_diff=True;q.scene.robot_state.is_diff=True;q.scene.robot_state.attached_collision_objects=[attached];assert wait(apply.call_async(q)).success
        scene=guard.read();assert OBJECT not in [o.id for o in scene.world.collision_objects]
        found=[o for o in scene.robot_state.attached_collision_objects if o.object.id==OBJECT]
        assert len(found)==1 and found[0].link_name=='arm2_tool0'
        output['attached_readback']=msgdict(found[0]);output['baseline_ACM']=msgdict(guard.base);output['scoped_ACM']=msgdict(guard.scoped)
        floor_obj=next(o for o in scene.world.collision_objects if o.id==SUPPORT)
        floorM=pose_matrix(floor_obj.pose)@pose_matrix(floor_obj.primitive_poses[0]);size=np.array(floor_obj.primitives[0].dimensions)
        assert np.allclose(floorM[:3,:3],np.eye(3),atol=ROUND)
        floor={'top':float(floorM[2,3]+size[2]/2),'lo':(floorM[:3,3]-size/2).tolist(),'hi':(floorM[:3,3]+size/2).tolist()}
        initial_bottom=float(cube_bounds(initial,attached)[0][2]);output.update(floor=floor,initial_bottom=initial_bottom,initial_penetration_m=floor['top']-initial_bottom)
        v=GetStateValidity.Request();v.robot_state=copy.deepcopy(original.motion_plan_request.start_state)
        baseline=wait(valid.call_async(v));output['baseline_invalid']=msgdict(baseline)
        assert not baseline.valid and all({c.contact_body_1,c.contact_body_2}=={OBJECT,SUPPORT} for c in baseline.contacts)
        config=json.loads((Path(__file__).resolve().parents[2]/'src/a1_sorting_core/config/four_slot_candidate.json').read_text());wp=config['bins']['bin_red']['slots'][3]['waypoints'];names=['arm2_'+n for n in SUFFIXES]
        current=dict(initial)
        sequence=[('depart_32mm',wp['settle_top'],'depart'),('exit_above_bin',wp['carry'],'free'),('return_above_support',wp['settle_top'],'free'),('land_original_pose',[initial[n] for n in names],'land')]
        for label,target,direction in sequence:
            request=copy.deepcopy(original);p=request.motion_plan_request;p.start_state.joint_state.position=[current[n] for n in p.start_state.joint_state.name];p.start_state.joint_state.velocity=[0.]*len(current)
            p.goal_constraints=[Constraints(joint_constraints=[JointConstraint(joint_name=n,position=x,tolerance_above=.01,tolerance_below=.01,weight=1.) for n,x in zip(names,target)])]
            goal_state=dict(current);goal_state.update(zip(names,target));verify_requested_direction(current,goal_state,attached,floor,direction,initial_bottom)
            if direction!='free':guard.enable(grasp_verified=True,support_verified=True,audited_direction=direction)
            try:response=wait(plan.call_async(request))
            finally:
                if guard.active:guard.restore()
            row={'stage':label,'direction':direction,'request':msgdict(request),'response':msgdict(response)};output['segments'].append(row)
            (run/'RESULT.json').write_text(json.dumps(output,indent=2))
            if response.motion_plan_response.error_code.val!=1:raise RuntimeError('planning_failed:'+label)
            trajectory=response.motion_plan_response.trajectory.joint_trajectory
            if direction=='land':
                import yaml
                limits=yaml.safe_load((Path(__file__).resolve().parents[2]/'src/a1_cell_sim/config/joint_limits_dual.yaml').read_text())['joint_limits']
                trajectory=contact_landing_trajectory(trajectory,ROBOT,limits,p.max_velocity_scaling_factor,p.max_acceleration_scaling_factor,exact_goal=target)
            row['execution_trajectory']=msgdict(trajectory)
            profile=support_profile(trajectory,current,attached,floor,direction,initial_bottom)
            checks=[]
            try:
                for sample in profile:
                    if sample['support_contact'] and not guard.active:guard.enable(grasp_verified=True,support_verified=True,audited_direction=direction)
                    if not sample['support_contact'] and guard.active:guard.restore()
                    state=copy.deepcopy(p.start_state);state.joint_state.position=[sample['joints'][n] for n in state.joint_state.name]
                    v=GetStateValidity.Request();v.robot_state=state;answer=wait(valid.call_async(v))
                    checks.append({'t':sample['t'],'bottom':sample['bottom'],'support_scoped':guard.active,'validity':msgdict(answer)})
                    if not answer.valid:raise RuntimeError('independent_FCL_failed:'+label)
            finally:
                if guard.active:guard.restore()
            row['independent_samples']=checks;row['min_bottom']=min(s['bottom'] for s in profile);row['max_penetration_m']=max(0.,floor['top']-row['min_bottom']);current=profile[-1]['joints']
            if label=='exit_above_bin':
                row['cube_above_wall_m']=float(cube_bounds(current,attached)[0][2]-.815)
                assert row['cube_above_wall_m']>0
            (run/'RESULT.json').write_text(json.dumps(output,indent=2));print(label,'PASS',len(checks),row['max_penetration_m'],flush=True)
        output['all_segments_passed']=True
    except Exception as e:
        output['error']=repr(e);print('BLOCKED',repr(e),flush=True)
    finally:
        if guard:
            try:guard.restore();output['ACM_restored']=True
            except Exception as e:output['restore_error']=repr(e)
        (run/'RESULT.json').write_text(json.dumps(output,indent=2))
        ex.shutdown();thread.join(timeout=2);node.destroy_node();rclpy.try_shutdown();stop_group(server);log.close()
        (ROOT/'support_transition/latest-offline.txt').write_text(str(run));print(run,flush=True)
if __name__=='__main__':main()
