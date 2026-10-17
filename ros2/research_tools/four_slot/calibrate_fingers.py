"""Stage B: empty fingers at untouched initial standby, real effort and feedback.
No arm trajectory, part spawn, dispatcher or task submission. Same candidate release
1.5 N command and 10..14 mm exit gate. Any breach latches the supervisor fault.
"""
import json,sys,time,threading,subprocess,os
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"voice_four"))
from task_io import atomic_json
import rclpy
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from ros_gz_interfaces.msg import LogicalCameraImage
from a1_sorting_core.retained_scene_sync import camera_observation
from std_msgs.msg import Float64MultiArray
from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
from a1_sorting_core.four_slot_feedback import ReleaseEvidence
from a1_sorting_core.physical_ready import publisher_identity
from a1_sorting_core.four_slot_grip_control import opening_efforts
from geometry import ROBOT,fk,origin
from lock_window import LockWindow
from joint_receiver import JointReceiver
# Load ROS motion message types before any live feedback/hold is active.
from arm_probe import probe
from live_support_cycle import LiveSupportCycle
import numpy as np
import xml.etree.ElementTree as ET

def main():
 run=Path(sys.argv[1]);rclpy.init();n=rclpy.create_node('isolated_finger_calibration');last=None;monitor=None;holds=[];grasp_active=False;support_context=[None];consumer_samples=[];clock=[None,None];lock_window=[None]
 out=(run/'finger-response.jsonl').open('w',buffering=1)
 def emit(**d):out.write(json.dumps(dict(wall=time.time(),**d))+'\n')
 def fault(reason):
  try:
   with (run/'guard-fault.json').open('x') as f:json.dump(dict(kind='finger_calibration',reason=reason,wall=time.time()),f)
  except FileExistsError:pass
 def cb(m,info,receipt,source):
  nonlocal last
  stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
  last=(m,receipt,stamp)
  consumer_samples.append(dict(stamp=stamp,receipt=receipt,clock=clock[0],q=dict(zip(m.name,m.position))))
  try:
   active_window=lock_window[0]
   if active_window is not None:active_window.observe(m.name,m.position,stamp,source)
   for mon in ([monitor] if monitor else [])+[h[2] for h in holds]:
    previous=mon.last
    mon.observe(m.name,m.position,stamp,receipt,source)
    if mon.fault:
     emit(monitor_fault=mon.fault,previous=previous,current_stamp=stamp,clock=clock[0]);fault(mon.fault)
  except Exception as e:fault('callback:'+str(e))
 observations={}
 def logical(m):
  for model in m.model:
   if model.name=='calibration_red4':observations[model.name]=camera_observation(m.pose,model.pose,m.header.stamp.sec+m.header.stamp.nanosec*1e-9)
 camera=n.create_subscription(LogicalCameraImage,'/overhead_logical',logical,10)
 def on_clock(m):clock[:]=[m.clock.sec+m.clock.nanosec*1e-9,time.monotonic()]
 clock_sub=n.create_subscription(Clock,'/clock',on_clock,qos_profile_sensor_data)
 receiver=JointReceiver(cb,fault,run/'joint-ingress.jsonl')
 pubs=[n.create_publisher(Float64MultiArray,p+'/commands',10) for p in ('/gripper_controller','/arm2_gripper_controller')]
 def send(pub,v):pub.publish(Float64MultiArray(data=[v,v] if isinstance(v,(int,float)) else v));emit(command_N=v,topic=pub.topic_name)
 def step():
  time.sleep(.002)
  if (run/'guard-fault.json').exists() or (run/'SHUTDOWN_INTENT').exists():raise RuntimeError('supervisor_stop')
  if last is None or time.monotonic()-last[1]>.5:raise RuntimeError('missing_joint_feedback')
  until=time.monotonic()+.5
  while clock[0] is None or clock[0]<last[2]:
   if time.monotonic()>until:raise RuntimeError('joint_clock_alignment_timeout')
   time.sleep(.001)
  if clock[0]-last[2]>.5 or time.monotonic()-clock[1]>.5:raise RuntimeError('joint_or_clock_stale')
  if monitor and monitor.fault:raise RuntimeError(monitor.fault)
  if grasp_active:
   q=dict(zip(last[0].name,last[0].position));pair=[q['arm2_gripper_finger_'+side+'_joint'] for side in ('left','right')]
   if min(pair)<-.0025 or max(pair)>.002:raise RuntimeError('held_finger_feedback_lost_grasp')
   contacts=json.loads((run/'fixture-contact-status.json').read_text())
   if any(last[2]-contacts[s]['sim']>.1 for s in ('left','right')):raise RuntimeError('held_bilateral_contact_lost')
   if last[2]-observations['calibration_red4']['stamp']>.2:raise RuntimeError('held_object_feedback_stale')
  for held_pub,held_names,held_monitor in holds:
   if held_monitor.fault:raise RuntimeError('previous_arm_hold:'+held_monitor.fault)
   if not held_monitor.check(clock[0],time.monotonic()):raise RuntimeError('hold_feedback_not_fresh')
 def hold_tick():
  if last is None or time.monotonic()-last[1]>.5:
   if holds:fault('hold_feedback_missing')
   return
  if clock[0] is None or clock[0]<last[2]:return
  if clock[0]-last[2]>.5:
   fault('held_feedback_sim_stale');return
  q=dict(zip(last[0].name,last[0].position))
  for pub,names,mon in holds:
   if mon.fault:continue
   send(pub,opening_efforts([q[x] for x in names]))
 timer=n.create_timer(.01,hold_tick)
 executor=rclpy.executors.SingleThreadedExecutor();executor.add_node(n)
 thread=threading.Thread(target=executor.spin);thread.start()
 begin=time.monotonic()
 try:
  while last is None or any(p.get_subscription_count()!=1 for p in pubs):
   if time.monotonic()-begin>5:raise RuntimeError('controllers_not_uniquely_connected')
   time.sleep(.01)
  # Conservative full-opening finger AABB preflight vs bin walls/table patches.
  vals=dict(zip(last[0].name,last[0].position));rows=[]
  world=ET.parse(Path(__file__).resolve().parents[2]/'src/a1_cell_sim/worlds/four_slot_1ms.sdf')
  for prefix in ('','arm2_'):
   for side in ('left','right'):
    name=prefix+'gripper_finger_'+side+'_link';joint=prefix+'gripper_finger_'+side+'_joint'
    c=next(l for l in ROBOT.findall('link') if l.get('name')==name).find('collision');sz=np.array([.012,.012,.060]);corners=[]
    for q in (-.003,.04):
     v=dict(vals);v[joint]=q;M=fk(name,v)@origin(c.find('origin'));half=np.abs(M[:3,:3])@(sz/2);corners.extend([M[:3,3]-half,M[:3,3]+half])
    lo=np.min(corners,axis=0);hi=np.max(corners,axis=0)
    for model in world.findall('.//model'):
     if not (model.get('name','').startswith('bin_') or model.get('name') in ('tabletop','belt')):continue
     for col in model.findall('link/collision'):
      box=col.find('geometry/box/size')
      if box is None:raise RuntimeError('unhandled_obstacle_geometry')
      M=origin(model.find('pose'),True)@origin(col.find('pose'),True);half=np.abs(M[:3,:3])@(np.fromstring(box.text,sep=' ')/2)
      gap=float(np.max(np.maximum(lo-(M[:3,3]+half),(M[:3,3]-half)-hi)))
      if gap<=0:raise RuntimeError('standby_swept_finger_aabb_intersection:'+name+':'+model.get('name'))
      rows.append(dict(finger=name,obstacle=model.get('name')+'/'+str(col.get('name')),aabb_gap_m=gap))
  (run/'finger-preflight.json').write_text(json.dumps(rows,indent=2))
  for prefix,pub in zip(('','arm2_'),pubs):
   names=[prefix+'gripper_finger_'+s+'_joint' for s in ('left','right')]
   # Real empty-gripper preconditioning, outside bins, without setting joint states.
   deadline=time.monotonic()+5;emit(phase='precondition',arm=prefix or 'arm1')
   while True:
    step();q=dict(zip(last[0].name,last[0].position));pair=[q[x] for x in names]
    if time.monotonic()>deadline:raise RuntimeError('precondition_timeout')
    if all(.0015<x<.0025 for x in pair):break
    send(pub,[max(-1.5,min(1.5,150*(.002-x))) for x in pair]);time.sleep(.01)
   send(pub,0.)
   monitor=ReleaseEvidence(names);monitor.begin();deadline=time.monotonic()+3;emit(phase='release',arm=prefix or 'arm1',q=pair)
   while True:
    step()
    if time.monotonic()>deadline:raise RuntimeError('release_timeout')
    q=dict(zip(last[0].name,last[0].position));pair=[q[x] for x in names]
    if max(pair)>.014:raise RuntimeError('opening_envelope_exceeded')
    if min(pair)>=.010:
     send(pub,opening_efforts(pair))
     if not monitor.begin_exit(last[2],time.monotonic()):raise RuntimeError(monitor.fault or 'begin_exit_rejected')
     break
    send(pub,opening_efforts(pair));time.sleep(.01)
   start_sim=last[2];end_wall=time.monotonic()+3
   emit(phase='feedback_effort_hold',q=pair,sim=start_sim)
   while last[2]-start_sim<.5:
    step()
    q=dict(zip(last[0].name,last[0].position));send(pub,opening_efforts([q[x] for x in names]));time.sleep(.01)
    if time.monotonic()>end_wall:raise RuntimeError('hold_timeout_or_paused')
   emit(phase='unloaded_response_pass',q=list(monitor.last[2]),sim=last[2]);holds.append((pub,names,monitor));monitor=None
  def fixture_release(lift_cycle):
   nonlocal monitor,grasp_active
   selected=next(h for h in holds if h[0] is pubs[1]);holds.remove(selected)
   pub,names,old_monitor=selected
   atomic_json(run/'fixture-contact-permit.json',dict(phase='gentle_grasp',expires_wall=time.time()+5))
   emit(phase='fixture_gentle_grasp',force_N=-1.5)
   deadline=time.monotonic()+3
   while True:
    step();send(pub,-1.5);time.sleep(.01)
    q=dict(zip(last[0].name,last[0].position));pair=[q[x] for x in names]
    if min(pair)<-.0025:raise RuntimeError('fixture_grasp_missing_before_empty_close')
    try:contacts=json.loads((run/'fixture-contact-status.json').read_text())
    except FileNotFoundError:contacts={}
    if all(s in contacts and 0<=last[2]-contacts[s]['sim']<.1 for s in ('left','right')):break
    if time.monotonic()>deadline:raise RuntimeError('fixture_bilateral_contact_timeout')
   emit(phase='fixture_bilateral_contact',q=pair,contacts=contacts)
   if '--lift-fixture' in sys.argv:
    atomic_json(run/'fixture-contact-permit.json',dict(phase='gentle_grasp',expires_wall=time.time()+120))
    # Match the existing executor's light-touch then -8 N lock stage;
    # the previous calibration incorrectly tried lifting on pre-close force alone.
    import yaml
    params=yaml.safe_load((Path(__file__).resolve().parents[2]/'src/a1_sorting_core/config/sorting_params.yaml').read_text())
    lock_force=float(params['/pick_place_relay']['ros__parameters']['gripper_closed'])
    emit(phase='fixture_original_lock',force_N=lock_force)
    send(pub,lock_force);end=time.monotonic()+.6
    while time.monotonic()<end:step()
    grasp_active=True
    lock_window[0]=LockWindow(names);deadline=time.monotonic()+2
    while time.monotonic()<deadline:
     step();passed,stable,window_fault=lock_window[0].snapshot()
     if window_fault:raise RuntimeError(window_fault)
     if passed:break
    else:
     emit(phase='locked_finger_stability_timeout',samples=stable)
     raise RuntimeError('locked_fingers_not_settled')
    lock_window[0]=None
    emit(phase='locked_finger_stability_pass',samples=stable)
    lift_cycle()
    grasp_active=False
   atomic_json(run/'fixture-contact-permit.json',dict(phase='release',expires_wall=time.time()+5))
   monitor=ReleaseEvidence(names);monitor.begin();deadline=time.monotonic()+3
   while True:
    step();q=dict(zip(last[0].name,last[0].position));pair=[q[x] for x in names];send(pub,opening_efforts(pair));time.sleep(.01)
    if min(pair)>=.010:
     if not monitor.begin_exit(last[2],time.monotonic()):raise RuntimeError(monitor.fault or 'fixture_exit_not_ready')
     break
    if time.monotonic()>deadline:raise RuntimeError('fixture_release_timeout')
   holds.append((pub,names,monitor));monitor=None
   start=last[2];deadline=time.monotonic()+3
   while last[2]-start<.3:
    step()
    if time.monotonic()>deadline:raise RuntimeError('fixture_release_settle_timeout')
   contacts=json.loads((run/'fixture-contact-status.json').read_text())
   if any(last[2]-v['sim']<.1 for v in contacts.values()):raise RuntimeError('fixture_contact_not_cleared')
   obs=observations['calibration_red4'];xyz=obs['position']
   if abs(xyz[0]-.295)>.008 or abs(xyz[1]+.235)>.008 or abs(xyz[2]-.745)>.010:raise RuntimeError('fixture_displaced_outside_slot')
   atomic_json(run/'fixture-contact-permit.json',dict(phase='forbidden',expires_wall=0))
   if support_context[0] is not None:support_context[0].detach_after_release()
   emit(phase='fixture_release_pass',observation=obs,last_contacts=contacts,sim=last[2])
  if '--arm-probe' in sys.argv:

   probe(n,run,step,lambda:last,emit,observations if '--empty-slot' in sys.argv else None,fixture_release if '--release-fixture' in sys.argv else None,support_context)
  (run/'FINGER_DONE').touch()
 except Exception as e:fault(str(e));emit(failure=str(e))
 finally:
  if grasp_active or support_context[0] is not None and not support_context[0].detached:
   # Preserve holding force until the owned simulation is actually paused.
   command=['gz','service','-s','/world/sorting_cell/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','2000','--req','pause: true']
   try:
    paused=subprocess.run(command,capture_output=True,text=True,timeout=3)
    atomic_json(run/'actor-fault-pause.json',dict(returncode=paused.returncode,stdout=paused.stdout,stderr=paused.stderr,wall=time.time()))
   except Exception as e:emit(pause_error=repr(e))
  if support_context[0] is not None:
   try:support_context[0].restore_on_abort()
   except Exception as e:fault('support_ACM_cleanup:'+str(e));emit(cleanup_error=repr(e))
  monitor=None;holds.clear();timer.cancel()
  for p in pubs:send(p,0.)
  for _ in range(5):time.sleep(.01)
  receiver.close()
  executor.shutdown(timeout_sec=2);thread.join(timeout=2);n.destroy_node();rclpy.try_shutdown();out.close()
  (run/'consumer-feedback.json').write_text(json.dumps(consumer_samples))
if __name__=='__main__':main()
