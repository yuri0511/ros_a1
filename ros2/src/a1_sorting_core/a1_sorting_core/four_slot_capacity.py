"""Fail-closed retained inventory. No modulo reuse, no time-based reclamation."""
import json,math,threading
from pathlib import Path
class CapacityError(RuntimeError):pass
class SlotBank:
    def __init__(self,bins,path=None):
        self.lock=threading.RLock();self.path=Path(path) if path else None
        self.slots={s['id']:{'bin':b,'xyz':s['xyz'],'state':'EMPTY','part':None} for b,entry in bins.items() for s in entry['slots']}
        if len(self.slots)!=len(bins)*4:raise CapacityError('four_unique_slots_per_bin_required')
        if self.path and self.path.exists():
            old=json.loads(self.path.read_text())
            if set(old)!=set(self.slots):raise CapacityError('inventory_configuration_mismatch')
            for sid,row in old.items():
                if row['xyz']!=self.slots[sid]['xyz']:raise CapacityError('inventory_geometry_changed')
                self.slots[sid]=row
                if row['state']=='RESERVED':row['state']='FAULT';row['reason']='restart_during_reservation'
            self.save()
    def save(self):
        if self.path:
            self.path.parent.mkdir(parents=True,exist_ok=True);tmp=self.path.with_suffix('.tmp');tmp.write_text(json.dumps(self.slots,indent=2));tmp.replace(self.path)
    def reserve(self,bin_name,part):
        with self.lock:
            existing=[sid for sid,r in self.slots.items() if r['part']==part]
            if existing:raise CapacityError('duplicate_part_reservation')
            sid=next((sid for sid,r in self.slots.items() if r['bin']==bin_name and r['state']=='EMPTY'),None)
            if sid is None:raise CapacityError('bin_full_or_quarantined')
            self.slots[sid].update(state='RESERVED',part=part);self.save();return sid
    def occupied(self,sid,part,position,source_sim,now_sim):
        with self.lock:
            row=self.slots[sid]
            if row['state']!='RESERVED' or row['part']!=part:raise CapacityError('unexpected_placement')
            if len(position)!=3 or not all(math.isfinite(x) for x in [*position,source_sim,now_sim]):raise CapacityError('invalid_observation')
            if not 0<=now_sim-source_sim<=.2:raise CapacityError('stale_or_future_placement')
            # Full cube footprint must stay in its non-overlapping 90mm allocation cell.
            if any(abs(position[i]-row['xyz'][i])>.008 for i in (0,1)):raise CapacityError('outside_allocated_slot')
            if abs(position[2]-row['xyz'][2])>.01:raise CapacityError('part_not_settled_at_floor_height')
            row.update(state='OCCUPIED',observed_position=position,source_sim=source_sim);self.save()
    def fault(self,sid,reason):
        with self.lock:self.slots[sid].update(state='FAULT',reason=reason);self.save()
    def clear(self,sid,part,*,delete_ack,delete_sim,frame_sim,visible):
        with self.lock:
            row=self.slots[sid]
            if row['part']!=part or row['state'] not in ('OCCUPIED','FAULT'):raise CapacityError('wrong_clear_identity')
            if delete_ack is not True or not all(math.isfinite(x) for x in (delete_sim,frame_sim)) or frame_sim<=delete_sim or part in visible:raise CapacityError('deletion_not_proven')
            self.slots[sid]={'bin':row['bin'],'xyz':row['xyz'],'state':'EMPTY','part':None};self.save()

def load_motion_candidate(path, trial_permit=""):
    if trial_permit:
        from .four_slot_trial import load_trial
        load_trial(trial_permit,path)
        return json.loads(Path(path).read_text())
    data=json.loads(Path(path).read_text())
    if data.get('schema')!=1 or data.get('motion_validated') is not True:
        raise CapacityError('four_slot_motion_review_not_passed; plan-only permitted')
    return data
