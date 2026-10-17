"""The one approved, isolated support-contact calibration cycle. Not a batch node."""
import copy
import json
import time
from pathlib import Path
import numpy as np
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, Constraints, JointConstraint
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetStateValidity
from rosidl_runtime_py.convert import message_to_ordereddict as msgdict
import yaml
from geometry import ROOT, ROBOT, SUFFIXES, fk, origin, quaternion_matrix, matrix_quaternion
from support_contact import (OBJECT, SUPPORT, ROUND, SupportMatrix, pose_matrix, cube_bounds,
                             support_profile, contact_landing_trajectory, verify_requested_direction)
from a1_sorting_core.retained_scene_sync import observed_object


class LiveSupportCycle:
    def __init__(self,node,run,step,get_last,observations,emit,action,wait):
        self.node,self.run,self.step,self.get_last=node,run,step,get_last
        self.observations,self.emit,self.action,self.wait=observations,emit,action,wait
        self.guard=SupportMatrix(node,wait,emit)
        self.plan=node.create_client(GetMotionPlan,'/plan_kinematic_path')
        self.valid=node.create_client(GetStateValidity,'/check_state_validity')
        self.apply=node.create_client(ApplyPlanningScene,'/apply_planning_scene')
        self.rows=[];self.attached=None;self.detached=False;self.restored_on_departure=False
        self.deadline=time.monotonic()+90
        self.names=['arm2_'+n for n in SUFFIXES]
        config=json.loads((Path(__file__).resolve().parents[2]/'src/a1_sorting_core/config/four_slot_candidate.json').read_text())
        self.wp=config['bins']['bin_red']['slots'][3]['waypoints']
        self.limits=yaml.safe_load((Path(__file__).resolve().parents[2]/'src/a1_cell_sim/config/joint_limits_dual.yaml').read_text())['joint_limits']

    def _save(self):
        (self.run/'support-held-cycle.json').write_text(json.dumps(self.rows,indent=2))

    def _read(self,name,age=.2):
        data=json.loads((self.run/name).read_text());deadline=time.monotonic()+.5
        while self.get_last()[2]<data['sim']:
            self.step()
            if time.monotonic()>deadline:raise RuntimeError('support_evidence_clock_alignment')
        now=self.get_last()[2]
        if time.time()-data['wall']>.5 or now-data['sim']>age:
            raise RuntimeError('support_feedback_stale:'+name)
        return data

    def _permit(self,phase):
        from task_io import atomic_json
        data=dict(phase=phase,expires_wall=time.time()+90,pair=[OBJECT,SUPPORT],
                  max_support_depth=self.support_depth,floor_top=self.floor['top'],initial_bottom=self.initial_bottom,native_floor_min=self.native_floor_min)
        atomic_json(self.run/'support-contact-permit.json',data)
        self.emit(phase='support_physical_phase',permit=data)

    def _attach(self):
        self.step()
        native=self._read('fixture-native-status.json')
        support=self._read('fixture-support-status.json',.1)
        if not support['normal_vertical'] or support['recent_count']<2 or support['recent_span']<.05:
            raise RuntimeError('real_support_contact_not_established')
        self.support_depth=support['recent_max_depth'];self.native_floor_min=native['recent_min_bottom']
        if self.support_depth<=0:raise RuntimeError('no_real_support_contact_depth')
        self.initial_support=support;self.initial_native=native
        initial=copy.deepcopy(self.observations['calibration_red4']);until=time.monotonic()+.5
        while self.get_last()[2]<initial['stamp']:
            self.step()
            if time.monotonic()>until:raise RuntimeError('held_clock_alignment_timeout')
        jm,_,stamp=self.get_last();self.initial=dict(zip(jm.name,jm.position))
        M=fk('arm2_tool0',self.initial);O=np.eye(4)
        O[:3,:3]=quaternion_matrix(initial['quaternion']);O[:3,3]=initial['position'];local=np.linalg.inv(M)@O
        attached=AttachedCollisionObject();attached.link_name='arm2_tool0'
        attached.object=observed_object('calibration_red4',local[:3,3].tolist(),matrix_quaternion(local[:3,:3]).tolist(),initial['stamp'],stamp)
        attached.object.header.frame_id='arm2_tool0'
        attached.touch_links=['arm2_gripper_finger_left_link','arm2_gripper_finger_right_link']
        before=self.guard.read();world=[o for o in before.world.collision_objects if o.id==OBJECT]
        if len(world)!=1 or list(world[0].primitives[0].dimensions)!=[.05]*3:raise RuntimeError('world_object_missing_before_attach')
        q=ApplyPlanningScene.Request();q.scene.is_diff=True;q.scene.robot_state.is_diff=True
        q.scene.robot_state.attached_collision_objects=[attached]
        if not self.wait(self.apply.call_async(q)).success:raise RuntimeError('attachment_rejected')
        after=self.guard.read();found=[o for o in after.robot_state.attached_collision_objects if o.object.id==OBJECT]
        if len(found)!=1 or found[0].link_name!='arm2_tool0' or OBJECT in [o.id for o in after.world.collision_objects]:raise RuntimeError('attachment_identity_readback_failed')
        if len(found[0].object.primitives)!=1 or list(found[0].object.primitives[0].dimensions)!=[.05]*3:raise RuntimeError('attached_shape_readback_changed')
        expected=pose_matrix(attached.object.pose)@pose_matrix(attached.object.primitive_poses[0])
        actual=pose_matrix(found[0].object.pose)@pose_matrix(found[0].object.primitive_poses[0])
        if not np.allclose(actual,expected,atol=ROUND,rtol=0):raise RuntimeError('attached_transform_readback_changed')
        before_world={o.id:msgdict(o) for o in before.world.collision_objects if o.id!=OBJECT}
        after_world={o.id:msgdict(o) for o in after.world.collision_objects}
        if before_world!=after_world:raise RuntimeError('other_obstacle_geometry_changed')
        self.attached=attached
        floor_obj=next(o for o in after.world.collision_objects if o.id==SUPPORT)
        F=pose_matrix(floor_obj.pose)@pose_matrix(floor_obj.primitive_poses[0]);size=np.array(floor_obj.primitives[0].dimensions)
        if not np.allclose(F[:3,:3],np.eye(3),atol=ROUND):raise RuntimeError('support_not_horizontal')
        self.floor={'top':float(F[2,3]+size[2]/2),'lo':(F[:3,3]-size/2).tolist(),'hi':(F[:3,3]+size/2).tolist()}
        self.initial_bottom=float(cube_bounds(self.initial,attached)[0][2])
        evidence=dict(initial_observation=initial,native=native,support=support,attached_readback=msgdict(found[0]),floor=self.floor,initial_bottom=self.initial_bottom,initial_joints=msgdict(jm))
        (self.run/'support-attestation.json').write_text(json.dumps(evidence,indent=2))
        self._permit('supported')

    def _progress(self,direction):
        self.step()
        if time.monotonic()>self.deadline:raise RuntimeError('support_cycle_wall_timeout')
        native=self._read('fixture-native-status.json');camera=self.observations['calibration_red4']
        if self.get_last()[2]-camera['stamp']>.2:raise RuntimeError('held_camera_stale')
        if direction=='depart' and self.guard.active:
            x,y,z,w=camera['quaternion']
            bottom=camera['position'][2]-.025*(abs(2*(x*z-y*w))+abs(2*(y*z+x*w))+abs(1-2*(x*x+y*y)))
            if native['bottom']>self.floor['top']+ROUND and bottom>self.floor['top']+ROUND:
                self.guard.restore();self._permit('free');self.restored_on_departure=True
                self.emit(phase='actual_support_separation',native=native,camera=camera)

    def _wait_result(self,future,direction,timeout=20):
        end=time.monotonic()+timeout
        while not future.done():
            self._progress(direction)
            if time.monotonic()>end:raise RuntimeError('held_action_timeout')
        self._progress(direction)
        return future.result()

    def _stable(self,target,direction):
        rows=[];laststamp=None;deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            self._progress(direction);jm,_,stamp=self.get_last()
            if stamp==laststamp:continue
            laststamp=stamp;q=dict(zip(jm.name,jm.position));v=dict(zip(jm.name,jm.velocity))
            if any(abs(q[n]-x)>.01 or abs(v[n])>.3 for n,x in zip(self.names,target)):rows=[]
            else:rows.append(dict(sim=stamp,q=[q[n] for n in self.names],v=[v[n] for n in self.names]))
            if len(rows)>=50 and rows[-1]['sim']-rows[0]['sim']>=.25:return rows
        raise RuntimeError('held_physical_stability_timeout')

    def run_cycle(self):
        self._attach()
        sequence=[('depart_32mm',self.wp['settle_top'],'depart'),('exit_above_bin',self.wp['carry'],'free'),('return_above_support',self.wp['settle_top'],'free'),('land_original_pose',[self.initial[n] for n in self.names],'land')]
        for label,target,direction in sequence:
            self.step();jm,_,_=self.get_last();current=dict(zip(jm.name,jm.position))
            rq=GetMotionPlan.Request();p=rq.motion_plan_request
            p.group_name='arm2_ur_manipulator';p.pipeline_id='pilz_industrial_motion_planner';p.planner_id='LIN';p.allowed_planning_time=3.;p.num_planning_attempts=1;p.max_velocity_scaling_factor=.2;p.max_acceleration_scaling_factor=.2
            p.start_state.joint_state=copy.deepcopy(jm);p.start_state.joint_state.velocity=[0.]*len(jm.name);p.start_state.attached_collision_objects=[self.attached]
            p.goal_constraints=[Constraints(joint_constraints=[JointConstraint(joint_name=n,position=x,tolerance_above=.01,tolerance_below=.01,weight=1.) for n,x in zip(self.names,target)])]
            goal_state=dict(current);goal_state.update(zip(self.names,target))
            verify_requested_direction(current,goal_state,self.attached,self.floor,direction,self.initial_bottom)
            if direction!='free':self.guard.enable(grasp_verified=True,support_verified=True,audited_direction=direction)
            try:response=self.wait(self.plan.call_async(rq))
            finally:
                if self.guard.active:self.guard.restore()
            row=dict(stage=label,direction=direction,request=msgdict(rq),response=msgdict(response));self.rows.append(row);self._save()
            if response.motion_plan_response.error_code.val!=1:raise RuntimeError('held_plan_failed:'+label)
            tr=copy.deepcopy(response.motion_plan_response.trajectory.joint_trajectory)
            if direction=='land':tr=contact_landing_trajectory(tr,ROBOT,self.limits,.2,.2,exact_goal=target)
            row['execution_trajectory']=msgdict(tr)
            profile=support_profile(tr,current,self.attached,self.floor,direction,self.initial_bottom)
            checks=[]
            try:
                for sample in profile:
                    if sample['support_contact'] and not self.guard.active:self.guard.enable(grasp_verified=True,support_verified=True,audited_direction=direction)
                    if not sample['support_contact'] and self.guard.active:self.guard.restore()
                    v=GetStateValidity.Request();v.robot_state=copy.deepcopy(p.start_state);v.robot_state.joint_state.position=[sample['joints'][n] for n in jm.name]
                    answer=self.wait(self.valid.call_async(v));checks.append(dict(t=sample['t'],bottom=sample['bottom'],support_scoped=self.guard.active,validity=msgdict(answer)))
                    if not answer.valid:raise RuntimeError('independent_held_FCL_failed')
            finally:
                if self.guard.active:self.guard.restore()
            row['independent_checks']=checks;self._save()
            # All complete trajectory checks precede the real Action request.
            self.step()
            if time.monotonic()>self.deadline:raise RuntimeError('support_cycle_expired_before_action')
            if direction=='depart':
                current_support=self._read('fixture-support-status.json',.1)
                if current_support['recent_max_depth']>self.support_depth+ROUND:raise RuntimeError('support_changed_before_departure')
            if direction!='free':self.guard.enable(grasp_verified=True,support_verified=True,audited_direction=direction)
            self._permit('departing' if direction=='depart' else ('landing' if direction=='land' else 'free'))
            goal=FollowJointTrajectory.Goal();goal.trajectory=tr
            handle=self.wait(self.action.send_goal_async(goal))
            if not handle.accepted:raise RuntimeError('held_goal_rejected')
            try:rr=self._wait_result(handle.get_result_async(),direction)
            except Exception:handle.cancel_goal_async();raise
            row['action']=msgdict(rr);self._save()
            if rr.status!=4 or rr.result.error_code!=0:raise RuntimeError('held_controller_failure')
            row['post_action_stability']=self._stable(target,direction)
            if direction=='depart' and not self.restored_on_departure:raise RuntimeError('actual_support_separation_not_verified')
            native=self._read('fixture-native-status.json');row['actual_native']=native
            if direction=='depart' and native['position'][2]-self.initial_native['position'][2]<.020:
                raise RuntimeError('held_object_did_not_follow_20mm')
            actual_q=dict(zip(self.get_last()[0].name,self.get_last()[0].position))
            predicted=fk(self.attached.link_name,actual_q)@pose_matrix(self.attached.object.pose)@pose_matrix(self.attached.object.primitive_poses[0])
            error=np.abs(np.array(native['position'])-predicted[:3,3]);row['actual_attachment_position_error_m']=error.tolist()
            if error[0]>.008 or error[1]>.008 or error[2]>.010:raise RuntimeError('held_pose_not_consistent_with_attachment')
            if label=='exit_above_bin':
                if native['bottom']<=.815:raise RuntimeError('payload_not_out_of_bin')
                actual=dict(zip(self.get_last()[0].name,self.get_last()[0].position));clearances={}
                for name in ('arm2_gripper_finger_left_link','arm2_gripper_finger_right_link','arm2_gripper_base'):
                    link=next(l for l in ROBOT.findall('link') if l.get('name')==name)
                    for index,col in enumerate(link.findall('collision')):
                        size=np.fromstring(col.find('geometry/box').get('size'),sep=' ')
                        M=fk(name,actual)@origin(col.find('origin'));bottom=float(M[2,3]-np.abs(M[2,:3])@(size/2))
                        clearances[name+str(index)]=bottom-.815
                        if bottom<=.815:raise RuntimeError('gripper_not_out_of_bin')
                row['actual_gripper_above_wall_m']=clearances
            if direction=='land':
                support=self._read('fixture-support-status.json',.1)
                if support['depth']>self.support_depth+ROUND:raise RuntimeError('landing_support_penetration_grows')
                row['actual_support']=support;self._permit('supported_release')
            self._save();self.emit(phase='held_stage_pass',stage=label,native=native)
        return self

    def detach_after_release(self):
        obs=copy.deepcopy(self.observations['calibration_red4']);until=time.monotonic()+.5
        while self.get_last()[2]<obs['stamp']:
            self.step()
            if time.monotonic()>until:raise RuntimeError('detach_clock_alignment_timeout')
        q=ApplyPlanningScene.Request();q.scene.is_diff=True;q.scene.robot_state.is_diff=True
        removed=AttachedCollisionObject();removed.link_name='arm2_tool0';removed.object.id=OBJECT;removed.object.operation=removed.object.REMOVE
        q.scene.robot_state.attached_collision_objects=[removed]
        q.scene.world.collision_objects=[observed_object('calibration_red4',obs['position'],obs['quaternion'],obs['stamp'],self.get_last()[2])]
        if not self.wait(self.apply.call_async(q)).success:raise RuntimeError('detachment_rejected')
        scene=self.guard.read()
        if OBJECT not in [o.id for o in scene.world.collision_objects] or OBJECT in [o.object.id for o in scene.robot_state.attached_collision_objects]:raise RuntimeError('detachment_identity_readback_failed')
        self.guard.restore();self._permit('normal');self.detached=True
        self.emit(phase='detached_world_restored',observation=obs)

    def restore_on_abort(self):
        # No guard-dependent wait: fault cleanup must work after the world pauses.
        def bounded(f):
            deadline=time.monotonic()+3
            while not f.done():
                if time.monotonic()>deadline:raise RuntimeError('abort_restore_timeout')
                time.sleep(.005)
            return f.result()
        old=self.guard.wait;self.guard.wait=bounded
        try:self.guard.restore()
        finally:self.guard.wait=old
