"""Small Pilz-planned standby wrist probe. No parts and no task executor."""
import copy,json,time
from pathlib import Path
from moveit_msgs.srv import ApplyPlanningScene,GetMotionPlan,GetStateValidity
from moveit_msgs.msg import Constraints,JointConstraint,AttachedCollisionObject,CollisionObject
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.convert import message_to_ordereddict as msgdict
from a1_sorting_core.physical_ready import PhysicalReadyMonitor,publisher_identity
from geometry import ROOT,SUFFIXES,fk,quaternion_matrix,matrix_quaternion
import numpy as np

def probe(node,run,step,get_last,emit,observations=None,release_test=None):
 def wait(f,timeout=15):
  end=time.monotonic()+timeout
  while not f.done():
   step();time.sleep(.002)
   if time.monotonic()>end:raise RuntimeError('probe_rpc_or_action_timeout')
  return f.result()
 scene=node.create_client(ApplyPlanningScene,'/apply_planning_scene');plan=node.create_client(GetMotionPlan,'/plan_kinematic_path');valid=node.create_client(GetStateValidity,'/check_state_validity')
 for c in (scene,plan,valid):
  if not c.wait_for_service(timeout_sec=3):raise RuntimeError('probe_planner_unavailable')
 req=ApplyPlanningScene.Request();data=json.loads((ROOT/'audit/stage2/planning-scene.json').read_text())
 for obj in data['scene']['world']['collision_objects']:obj.pop('operation',None)
 set_message_fields(req,data)
 for obj in req.scene.world.collision_objects:obj.operation=obj.ADD
 if not wait(scene.call_async(req)).success:raise RuntimeError('probe_scene_rejected')
 records=[]
 empty_slot=observations is not None
 for prefix in (('arm2_',) if empty_slot else ('','arm2_')):
  names=[prefix+x for x in SUFFIXES];m,_,_=get_last();home=dict(zip(m.name,m.position));target=[home[x] for x in names];target[-1]+=.0123
  action=ActionClient(node,FollowJointTrajectory,'/'+('arm2_' if prefix else '')+'joint_trajectory_controller/follow_joint_trajectory')
  if not action.wait_for_server(timeout_sec=3):raise RuntimeError('probe_controller_missing')
  sequence=[('out',target),('return',[home[x] for x in names])]
  if empty_slot:
   cfg=json.loads((Path(__file__).resolve().parents[2]/'src/a1_sorting_core/config/four_slot_candidate.json').read_text())
   wp=cfg['bins']['bin_red']['slots'][3]['waypoints']
   sequence=[('carry',wp['carry']),('top',wp['settle_top']),('bottom',wp['settle_bottom']),('rise',wp['carry']),('return',[home[x] for x in names])]
  for label,goal in sequence:
   if empty_slot:
    from a1_sorting_core.retained_scene_sync import RosRetainedSceneSync
    if 'calibration_red4' not in observations:raise RuntimeError('fixture_not_observed')
    sync=RosRetainedSceneSync(node)
    snapshot=copy.deepcopy(observations);until=time.monotonic()+.5
    while get_last()[2]<snapshot['calibration_red4']['stamp']:
     if time.monotonic()>until:raise RuntimeError('fixture_clock_alignment_timeout')
     step()
    now=get_last()[2]
    emit(phase='fixture_scene_input',stage=label,observations=snapshot,now=now)
    ack=sync.sync({'s':{'state':'OCCUPIED','part':'calibration_red4'}},snapshot,now)
    emit(phase='fixture_scene_ack',stage=label,ack=ack,observation=observations['calibration_red4'])
   step();m,_,_=get_last();q=GetMotionPlan.Request();p=q.motion_plan_request
   p.group_name=prefix+'ur_manipulator';p.pipeline_id='pilz_industrial_motion_planner';p.planner_id='LIN' if label=='rise' else 'PTP';p.num_planning_attempts=1;p.allowed_planning_time=3.;p.max_velocity_scaling_factor=.2;p.max_acceleration_scaling_factor=.2
   p.start_state.joint_state=copy.deepcopy(m);p.start_state.joint_state.velocity=[0.]*len(m.name)
   p.goal_constraints=[Constraints(joint_constraints=[JointConstraint(joint_name=n,position=x,tolerance_above=.01,tolerance_below=.01,weight=1.) for n,x in zip(names,goal)])]
   result=wait(plan.call_async(q));row=dict(arm=prefix or 'arm1',stage=label,request=msgdict(q),response=msgdict(result));records.append(row)
   (run/'arm-probe.json').write_text(json.dumps(records,indent=2))
   if result.motion_plan_response.error_code.val!=1:raise RuntimeError('probe_planning_failed')
   traj=result.motion_plan_response.trajectory.joint_trajectory
   times=[x.time_from_start.sec*10**9+x.time_from_start.nanosec for x in traj.points]
   if not times or any(b<=a for a,b in zip(times,times[1:])):raise RuntimeError('probe_nonmonotonic_planned_trajectory')
   # Validate the full robot at each actual planner waypoint, exact world obstacles.
   for point in traj.points:
    v=GetStateValidity.Request();v.robot_state=copy.deepcopy(p.start_state)
    positions=dict(zip(m.name,m.position));positions.update(zip(traj.joint_names,point.positions));v.robot_state.joint_state.position=[positions[n] for n in m.name]
    check=wait(valid.call_async(v));row.setdefault('state_validity',[]).append(msgdict(check))
    if not check.valid:raise RuntimeError('probe_collision')
   goalmsg=FollowJointTrajectory.Goal();goalmsg.trajectory=traj
   handle=wait(action.send_goal_async(goalmsg))
   if not handle.accepted:raise RuntimeError('probe_action_rejected')
   try:response=wait(handle.get_result_async(),20)
   except Exception:
    handle.cancel_goal_async();raise
   row['action']=msgdict(response);(run/'arm-probe.json').write_text(json.dumps(records,indent=2))
   if response.status!=4 or response.result.error_code!=0:raise RuntimeError('probe_action_failed')
   # Controller success is not a physical-ready claim. Explicit feedback window below.
   samples=[];end=time.monotonic()+3;laststamp=None
   while time.monotonic()<end:
    step();jm,receipt,stamp=get_last();values=dict(zip(jm.name,jm.position));vel=dict(zip(jm.name,jm.velocity))
    if stamp==laststamp:continue
    laststamp=stamp
    if any(abs(values[n]-x)>.01 or abs(vel[n])>.3 for n,x in zip(names,goal)):samples=[]
    else:samples.append(dict(stamp=stamp,q=[values[n] for n in names],v=[vel[n] for n in names]))
    if len(samples)>=50 and samples[-1]['stamp']-samples[0]['stamp']>=.25:break
   else:raise RuntimeError('probe_physical_stability_timeout')
   if label=='bottom' and release_test is not None:
    def lift_cycle():
     from a1_sorting_core.retained_scene_sync import observed_object
     initial=copy.deepcopy(observations['calibration_red4']);until=time.monotonic()+.5
     while get_last()[2]<initial['stamp']:
      step()
      if time.monotonic()>until:raise RuntimeError('held_clock_alignment_timeout')
     jm,_,stamp=get_last()
     # Model the measured physical cube attached to the measured tool frame.
     M=fk('arm2_tool0',dict(zip(jm.name,jm.position)));O=np.eye(4)
     O[:3,:3]=quaternion_matrix(initial['quaternion']);O[:3,3]=initial['position'];local=np.linalg.inv(M)@O
     attached=AttachedCollisionObject();attached.link_name='arm2_tool0'
     attached.object=observed_object('calibration_red4',local[:3,3].tolist(),matrix_quaternion(local[:3,:3]).tolist(),initial['stamp'],stamp)
     attached.object.header.frame_id='arm2_tool0';attached.touch_links=['arm2_gripper_finger_left_link','arm2_gripper_finger_right_link']
     removal=ApplyPlanningScene.Request();removal.scene.is_diff=True
     removed=CollisionObject();removed.id=attached.object.id;removed.operation=removed.REMOVE;removal.scene.world.collision_objects=[removed]
     if not wait(scene.call_async(removal)).success:raise RuntimeError('held_scene_remove_not_acknowledged')
     held_rows=[]
     for stage,tgt in [('lift_32mm',wp['settle_top']),('replace',wp['settle_bottom'])]:
      step();jm,_,_=get_last();rq=GetMotionPlan.Request();rq.motion_plan_request=copy.deepcopy(p)
      h=rq.motion_plan_request;h.start_state.joint_state=copy.deepcopy(jm);h.start_state.joint_state.velocity=[0.]*len(jm.name);h.start_state.attached_collision_objects=[attached];h.planner_id='LIN'
      h.goal_constraints=[Constraints(joint_constraints=[JointConstraint(joint_name=n,position=x,tolerance_above=.01,tolerance_below=.01,weight=1.) for n,x in zip(names,tgt)])]
      planned=wait(plan.call_async(rq));record={'stage':stage,'request':msgdict(rq),'response':msgdict(planned)};held_rows.append(record)
      (run/'held-cycle.json').write_text(json.dumps(held_rows,indent=2))
      if planned.motion_plan_response.error_code.val!=1:raise RuntimeError('held_plan_failed')
      jt=planned.motion_plan_response.trajectory.joint_trajectory;times=[x.time_from_start.sec*10**9+x.time_from_start.nanosec for x in jt.points]
      if not times or any(b<=a for a,b in zip(times,times[1:])):raise RuntimeError('held_plan_nonmonotonic')
      for point in jt.points:
       v=GetStateValidity.Request();v.robot_state=copy.deepcopy(h.start_state);vals=dict(zip(jm.name,jm.position));vals.update(zip(jt.joint_names,point.positions));v.robot_state.joint_state.position=[vals[n] for n in jm.name]
       check=wait(valid.call_async(v));record.setdefault('validity',[]).append(msgdict(check))
       if not check.valid:raise RuntimeError('held_state_collision')
      g=FollowJointTrajectory.Goal();g.trajectory=jt;handle=wait(action.send_goal_async(g))
      if not handle.accepted:raise RuntimeError('held_goal_rejected')
      try:rr=wait(handle.get_result_async(),20)
      except Exception:handle.cancel_goal_async();raise
      record['action']=msgdict(rr)
      if rr.status!=4 or rr.result.error_code!=0:raise RuntimeError('held_controller_failure')
      after=get_last()[2];deadline=time.monotonic()+2
      while observations['calibration_red4']['stamp']<after:
       step()
       if time.monotonic()>deadline:raise RuntimeError('held_camera_feedback_missing')
      record['actual_object']=copy.deepcopy(observations['calibration_red4']);record['actual_joints']=msgdict(get_last()[0]);(run/'held-cycle.json').write_text(json.dumps(held_rows,indent=2))
      delta=record['actual_object']['position'][2]-initial['position'][2]
      if stage=='lift_32mm' and delta<.020:raise RuntimeError('held_object_did_not_follow_20mm')
      if stage=='replace' and abs(delta)>.010:raise RuntimeError('held_object_not_replaced')
     emit(phase='fixture_lift_replace_pass',rows=held_rows)
    release_test(lift_cycle)
   row['post_action_samples']=samples;(run/'arm-probe.json').write_text(json.dumps(records,indent=2));emit(phase='arm_probe_pass',arm=prefix or 'arm1',stage=label)
  action.destroy()
