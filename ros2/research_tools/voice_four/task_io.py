"""Local fixture loading. No network, credentials, ROS or actuator imports."""
import json,re
from pathlib import Path
from a1_sorting_core.voice_batch_plan import validate_candidate,audit_utterance,PlanRejected
from a1_sorting_core.voice_batch_safety import ensure_geometry_review

def load_task(path):
    d=json.loads(Path(path).read_text())
    plan=validate_candidate(json.dumps(d['plan']))
    audit_utterance(d['transcript_unmodified'],plan);ensure_geometry_review(plan)
    if plan.counts!=(0,1,1,2) or plan.order!=('yellow','red','green','green'):raise PlanRejected('this_entry_requires_exact_four_demo')
    if d.get('plan_sha256')!=plan.sha256 or not re.fullmatch('[0-9a-f]{32}',d.get('request_id','')):raise PlanRejected('local_task_identity_mismatch')
    return d,plan

def atomic_json(path,value):
    path=Path(path);tmp=path.with_suffix(path.suffix+'.tmp');tmp.write_text(json.dumps(value,ensure_ascii=False,indent=2));tmp.replace(path)
