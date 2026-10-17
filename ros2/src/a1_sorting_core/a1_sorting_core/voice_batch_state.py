"""Pure fail-closed dynamic batch lifecycle. No ROS or actuator methods."""
from collections import Counter
import math
import re
import uuid
from .voice_batch_plan import COLORS,PlanRejected,validate_candidate
import json

ARM_FOR={'blue':'arm1','yellow':'arm1','red':'arm2','green':'arm2'}

def physical_ready_claim(detail):
    try:
        o=json.loads(detail);e=o['evidence'];c=o['controller_result']
        if any(type(e[k]) not in (int,float) or not math.isfinite(e[k]) for k in ('samples','stable_span','max_reported_speed','max_difference_speed','max_position_error')):return False
        return bool(e['ready'] is True and e['samples']>=50 and e['stable_span']>=.25 and 0<=e['max_reported_speed']<=.3 and 0<=e['max_difference_speed']<=.3 and 0<=e['max_position_error']<=.01 and c['success'] is True and c['status']==4 and c['error_code']==0 and o['no_active_goal'] is True)
    except (ValueError,TypeError,KeyError):return False

class VoiceBatchState:
    def __init__(self,session_id=None,retain_capacity=0):
        self.retain_capacity=retain_capacity
        self.session_id=session_id or uuid.uuid4().hex
        self.state='WAITING_FOR_PLAN';self.plan=None;self.request_id=None;self.hardware_ready=False
        self.issued=[];self.parts={};self.current=None;self.pending_color=None;self.seen_ready=set();self.recycled=set();self.recycle_wall={};self.recycle_sim={};self.events=[];self.fault=None;self.deadline=None

    def emit(self,kind,**data):self.events.append({'event':kind,**data})
    def halt(self,reason):
        if self.state!='HALTED':self.fault=reason;self.state='HALTED';self.emit('HALTED',reason=reason)

    def accept(self,session_id,request_id,confirmed,plan,plan_sha256,now):
        if session_id!=self.session_id:raise PlanRejected('stale_session')
        if self.request_id is not None:raise PlanRejected('duplicate_request' if self.request_id==request_id else 'one_plan_per_world')
        if self.state!='WAITING_FOR_PLAN':raise PlanRejected('not_waiting_for_plan')
        if confirmed is not True:raise PlanRejected('human_confirmation_required')
        if not isinstance(request_id,str) or not re.fullmatch(r'[0-9a-f]{32}',request_id):raise PlanRejected('local_uuid_required')
        if plan.sha256!=plan_sha256:raise PlanRejected('confirmation_hash_mismatch')
        if self.retain_capacity and any(n>self.retain_capacity for n in plan.counts):raise PlanRejected('bin_capacity_exceeded')
        self.plan=plan;self.request_id=request_id;self.state='PLAN_PENDING_READINESS';self.deadline=now+60
        self.emit('VOICE_PLAN',request_id=request_id,plan=plan.as_dict(),plan_sha256=plan.sha256)

    def ready(self):
        self.hardware_ready=True
        if self.plan and self.state=='PLAN_PENDING_READINESS':self.state='RUNNING'

    def reserve(self,now,can_feed,visible_parts=(),logical_wall=0,logical_sim=None):
        if self.state in ('HALTED','COMPLETED') or not self.plan or not self.hardware_ready:return None
        if self.state=='PLAN_PENDING_READINESS':self.state='RUNNING'
        if self.pending_color is not None or self.current is not None or not can_feed:return None
        if len(self.issued)>=self.plan.total:return None
        color=self.plan.order[len(self.issued)]
        # Strong serial plan: all prior pieces already PART_PLACED+READY+released.
        # Same-bin placement must additionally await actual deletion ACK and a later
        # fresh logical frame with no previous piece; elapsed recycle delay is not proof.
        same=[p for p,r in self.parts.items() if r['color']==color]
        if not self.retain_capacity and any(p not in self.recycled or p in visible_parts or logical_wall<=self.recycle_wall.get(p,float('inf')) for p in same):return None
        # A queued old image received after the delete ACK is not a fresh frame.
        # Runtime ACKs always include sim_time; compare actual source timestamps.
        if not self.retain_capacity and any(p in self.recycle_sim and (logical_sim is None or logical_sim<=self.recycle_sim[p]) for p in same):return None
        if self.retain_capacity and len(same)>=self.retain_capacity:self.halt('retained_bin_full');return None
        self.pending_color=color;self.issued.append(color);self.deadline=now+30
        self.emit('SPAWN_REQUEST',index=len(self.issued),color=color)
        return color

    def arrival(self,part,color,now):
        if self.state=='HALTED':return False
        if part in self.parts:
            if self.parts[part]['color']!=color:self.halt('vision_changed_color')
            return False
        if not part or self.pending_color is None or self.current is not None or self.state!='RUNNING':self.halt('unauthorized_or_extra_arrival');return False
        if color!=self.pending_color:self.halt('vision_plan_mismatch');return False
        self.parts[part]={'color':color,'arm':ARM_FOR[color],'placed':False,'ready':False,'released':False}
        self.current=part;self.pending_color=None;self.deadline=now+150
        self.emit('PartInfo',part=part,color=color,index=len(self.parts))
        return True

    def observe_entities(self,names):
        if self.state=='HALTED':return
        unseen=set(names)-set(self.parts)
        if self.pending_color is None and unseen:self.halt('unrequested_gazebo_entity')
        if self.pending_color is not None and len(unseen)>1:self.halt('multiple_unrequested_entities')

    def status(self,part,arm,state,detail):
        if self.state=='HALTED':return
        if state in ('ARM_FAULT','PART_REJECTED','FAILED'):self.halt('executor_'+state);return
        if state not in ('PART_PLACED','ARM_READY'):return
        if part not in self.parts or self.parts[part]['arm']!=arm:self.halt('unexpected_part_or_arm_status');return
        row=self.parts[part]
        if state=='PART_PLACED':
            try:d=json.loads(detail)
            except (ValueError,TypeError):self.halt('invalid_placement_ack');return
            target='bin_'+row['color']
            if d.get('result')!='PLACED' or d.get('target_bin')!=target or d.get('final_bin')!=target:self.halt('wrong_placement');return
            row['placed']=True
        else:
            if not physical_ready_claim(detail):self.halt('invalid_physical_ready');return
            row['ready']=True
        self.emit(state,part=part,arm=arm)

    def release(self,part,arm,now):
        if self.state=='HALTED':return
        row=self.parts.get(part)
        if not row or row['arm']!=arm or not row['placed'] or not row['ready']:self.halt('release_without_placement_and_ready');return
        if row['released']:return
        if self.current!=part:self.halt('wrong_current_release');return
        row['released']=True;self.current=None;self.deadline=now+45
        self.emit('SLOT_RELEASED',part=part,arm=arm)
        if len(self.parts)==self.plan.total and all(r['released'] for r in self.parts.values()):
            self.state='COMPLETED';self.deadline=None;self.emit('COMPLETED',counts=dict(Counter(r['color'] for r in self.parts.values())))

    def recycle(self,part,now,sim_time=None):
        if part not in self.parts or not self.parts[part]['placed']:self.halt('unexpected_recycle_ack');return
        if sim_time is not None:
            if type(sim_time) not in (int,float) or not math.isfinite(sim_time) or sim_time<0:self.halt('invalid_recycle_source_time');return
            self.recycle_sim[part]=sim_time
        self.recycled.add(part);self.recycle_wall[part]=now;self.emit('PART_RECYCLED',part=part)

    def check_timeout(self,now):
        if self.state not in ('HALTED','COMPLETED') and self.deadline is not None and now>self.deadline:self.halt('wall_timeout')

    def snapshot(self):
        return {'session_id':self.session_id,'state':self.state,'request_id':self.request_id,'plan':self.plan.as_dict() if self.plan else None,'plan_sha256':self.plan.sha256 if self.plan else None,'issued':list(self.issued),'parts':self.parts,'fault':self.fault,'hardware_ready':self.hardware_ready}
