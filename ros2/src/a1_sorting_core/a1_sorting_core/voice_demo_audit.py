"""Offline/runtime audit of observed batch events; never sends commands."""
import json
from .voice_batch_plan import PlanRejected
from .voice_batch_state import physical_ready_claim

class BatchEventAudit:
    def __init__(self, plan, request_id):
        self.plan=plan;self.request_id=request_id;self.accepted=False
        self.issued=[];self.parts={};self.current=None;self.recycled=set()
        self.ready_claims=set();self.slot_releases=set();self.completed=False

    def event(self, e):
        kind=e['event']
        if kind=='HALTED':raise PlanRejected('batch_halted:'+str(e.get('reason')))
        if kind=='VOICE_PLAN':
            if self.accepted or e.get('request_id')!=self.request_id or e.get('plan_sha256')!=self.plan.sha256 or e.get('plan')!=self.plan.as_dict():raise PlanRejected('unexpected_plan_acceptance')
            self.accepted=True
        elif kind=='SPAWN_REQUEST':
            i=len(self.issued)
            if not self.accepted or self.current is not None or i>=self.plan.total or e.get('color')!=self.plan.order[i] or e.get('index')!=i+1:raise PlanRejected('unauthorized_order_or_extra_feed')
            if any(p not in self.recycled for p,r in self.parts.items() if r['color']==e['color']):raise PlanRejected('same_bin_without_recycle')
            self.issued.append(e['color']);self.current='AWAITING_VISION'
        elif kind=='PartInfo':
            part=e['part'];color=e['color']
            if self.current!='AWAITING_VISION' or part in self.parts or color!=self.issued[-1]:raise PlanRejected('unexpected_visual_part')
            self.parts[part]={'color':color,'placed':False,'ready':False,'released':False};self.current=part
        elif kind in ('PART_PLACED','ARM_READY','SLOT_RELEASED'):
            part=e['part'];r=self.parts.get(part)
            if r is None:raise PlanRejected('unknown_lifecycle_part')
            if kind=='PART_PLACED':r['placed']=True
            if kind=='ARM_READY':r['ready']=True
            if kind=='SLOT_RELEASED':
                if self.current!=part or not r['placed'] or not r['ready'] or r['released']:raise PlanRejected('release_without_placement_ready')
                r['released']=True;self.current=None;self.slot_releases.add(part)
        elif kind=='PART_RECYCLED':
            if e['part'] not in self.parts or not self.parts[e['part']]['placed']:raise PlanRejected('unexpected_recycle')
            self.recycled.add(e['part'])
        elif kind=='COMPLETED':
            if len(self.parts)!=self.plan.total or list(self.issued)!=list(self.plan.order) or any(not r['released'] for r in self.parts.values()):raise PlanRejected('premature_completion')
            self.completed=True

    def ready(self, part, detail):
        if not physical_ready_claim(detail):raise PlanRejected('invalid_raw_physical_ready')
        self.ready_claims.add(part)

    def finish(self):
        if not self.completed or set(self.parts)!=self.ready_claims or set(self.parts)!=self.slot_releases:raise PlanRejected('incomplete_ready_release_audit')
        return {'total':len(self.parts),'issued':self.issued,'physical_ready_parts':sorted(self.ready_claims),'released_parts':sorted(self.slot_releases),'complete':True}
