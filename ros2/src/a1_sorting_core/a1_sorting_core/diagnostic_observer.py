"""Bounded diagnostic observation and experiment abort. Not a READY repair."""
import os,time,json,threading,functools
from pathlib import Path
LOCK=threading.Lock()
def directory():return Path(os.environ['DIAG_RUN']) if os.environ.get('DIAG_RUN') else None
def snapshot(n):
 names=getattr(n,'_joint_names',[]);targets=getattr(n,'_standby_joints',[])
 pos=[getattr(n,'_joints_now',{}).get(k) for k in names];vel=[getattr(n,'_joint_vels',{}).get(k) for k in names]
 error=[None if p is None else p-v for p,v in zip(pos,targets)]
 age=time.time()-n._js_wall if getattr(n,'_js_wall',None) else None
 return {'arm':getattr(n,'_arm_name',''),'task':getattr(n,'_active_task_id',''),'at_standby_flag':getattr(n,'_at_standby',None),'joint_names':names,'positions':pos,'velocities':vel,'standby_target':targets,'error_rad':error,'joint_age_wall':age,'original_settled_criterion':bool(n._motion and n._motion._joints_settled(names,targets))}
def record(n,event,**fields):
 d=directory()
 if d:
  row={'wall':time.time(),'ros_sim':n.get_clock().now().nanoseconds*1e-9,'event':event,**snapshot(n),**fields}
  with LOCK:
   with (d/'diagnostic.jsonl').open('a') as f:f.write(json.dumps(row,default=str)+'\n')
def halt(n,kind,**fields):
 d=directory()
 if not d:return
 record(n,'ABORT_BEFORE_READY_OR_CONTINUE',kind=kind,**fields)
 try:
  with (d/'guard-fault.json').open('x') as f:json.dump({'kind':kind,'wall':time.time(),**snapshot(n),**fields},f,default=str)
 except FileExistsError:pass
 n.get_logger().error('DIAGNOSTIC_SAFETY_ABORT '+kind+'; no terminal flush/next-task continuation')
 # Park this worker BEFORE terminal publication. Parent pauses world and SIGINTs.
 # Hard bounded fallback terminates this executor if supervisor cannot stop it.
 threading.Event().wait(15)
 os._exit(86)
def check_native(n):
 d=directory()
 if d and (d/'guard-fault.json').exists():halt(n,'external_observer_fault')
def install(cls):
 if not directory():return
 for name in ('_try_return_standby','_flush_terminal_status','_publish_zone_clear','_set_state','_recover'):
  orig=getattr(cls,name)
  def wrap(fn,method):
   @functools.wraps(fn)
   def call(n,*args,**kw):
    if method in ('_flush_terminal_status','_publish_zone_clear'):check_native(n)
    record(n,method+':before',args=[getattr(x,'part_id',getattr(x,'value',str(x))) for x in args])
    result=fn(n,*args,**kw)
    record(n,method+':after')
    if method=='_try_return_standby' and (not n._at_standby or not snapshot(n)['original_settled_criterion']):halt(n,'return_standby_not_physically_verified')
    return result
   return call
  setattr(cls,name,wrap(orig,name))
