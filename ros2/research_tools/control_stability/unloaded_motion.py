"""Empty-cell diagnostic using the unchanged real executor and its motion client.

No PartInfo is fabricated; no task slot or ARM_READY event is spoofed. Test each
arm serially with existing standby and carry-anchor targets and READY monitor.
"""
import sys,time,json,threading
from pathlib import Path
import rclpy
from rosidl_runtime_py.convert import message_to_ordereddict
from a1_sorting_core.pick_place_executor_node import PickPlaceExecutorNode
from a1_sorting_core.motion_client import MotionClient
from a1_sorting_core.physical_ready import PhysicalReadyMonitor
D=Path(sys.argv[1]);rclpy.init(args=sys.argv[2:]);n=PickPlaceExecutorNode();finished=threading.Event();records=[]
# Existing carry compensation gate is .55rad; standby READY remains .01rad.
carry_monitor=PhysicalReadyMonitor(pos_tol=.55)
original_observe=n._ready_monitor.observe
def observe_both(*args,**kw):
 original_observe(*args,**kw);carry_monitor.observe(*args,**kw)
n._ready_monitor.observe=observe_both
def record(kind,**kw):
 row={'kind':kind,'wall':time.time(),'monotonic':time.monotonic(),'sim':n.get_clock().now().nanoseconds*1e-9,'arm':n._arm_name,**kw};records.append(row)
 with (D/(n._arm_name+'-motion.jsonl')).open('a') as f:f.write(json.dumps(row)+'\n')
def fault(reason):
 n._arm_fault=reason;n._at_standby=False;record('ARM_FAULT',reason=reason)
 try:
  with (D/'guard-fault.json').open('x') as f:json.dump({'kind':'empty_motion_ARM_FAULT','wall':time.time(),'arm':n._arm_name,'reason':reason},f)
 except FileExistsError:pass

def check_window(label,target):
 monitor=carry_monitor if label=='empty_carry' else n._ready_monitor
 monitor.begin(time.monotonic());start=n.get_clock().now().nanoseconds*1e-9;end=start+2.;deadline=time.monotonic()+8.;last={}
 record('window_begin',label=label,target=target,sim_duration=2.,wall_timeout=8.,position_tolerance=monitor.pos_tol,velocity_tolerance=monitor.vel_tol)
 while time.monotonic()<deadline:
  result=n._motion.last_controller_result or {}
  last=monitor.evaluate(n._joint_names,target,n.get_clock().now().nanoseconds*1e-9,time.monotonic(),bool(result.get('success')),not n._motion._own_goal_inflight and n._motion._own_controller_active==0)
  if n.get_clock().now().nanoseconds*1e-9>=end:break
  time.sleep(.02)
 record('window_end',label=label,evidence=last,controller=n._motion.last_controller_result)
 if not last.get('ready'):fault(label+':'+last.get('reason','no_window'));return False
 return True

def run():
 try:
  n._init_motion()
  original=n._motion._execute_split
  def capture(result,label,timeout,settled):
   record('planned_trajectory',label=label,trajectory=message_to_ordereddict(result.planned_trajectory.joint_trajectory))
   return original(result,label,timeout,settled)
  n._motion._execute_split=capture
  record('configuration',group=n._group,joint_names=n._joint_names,standby=n._standby_joints,carry=n._carry_anchors[0][1],vel=n._vel_scale,acc=n._acc_scale)
  # Same initial standby path used by the real executor.
  n._plan_execute(n._down_pose(n._standby,n._standby[2]),'standby')
  if not check_window('initial_standby',n._standby_joints):return
  if not n._motion.move_to_joints(n._joint_names,n._carry_anchors[0][1],'carry_anchor_empty',settle_check=False):fault('carry_action_failed');return
  if not (n._motion.last_controller_result or {}).get('success'):fault('carry_controller_not_success');return
  if not check_window('empty_carry',n._carry_anchors[0][1]):return
  if not n._motion.move_to_joints(n._joint_names,n._standby_joints,'return_standby',settle_check=False):fault('return_action_failed');return
  if not check_window('return_standby',n._standby_joints):return
  record('empty_arm_passed')
 except Exception as e:fault('exception:'+repr(e))
 finally:
  (D/(n._arm_name+'-result.json')).write_text(json.dumps({'arm':n._arm_name,'passed':bool(records and records[-1]['kind']=='empty_arm_passed'),'fault':n._arm_fault,'records':records},indent=2));finished.set()
threading.Thread(target=run,daemon=True).start()
try:
 while not finished.is_set():rclpy.spin_once(n,timeout_sec=.02)
finally:n.destroy_node();rclpy.shutdown()
