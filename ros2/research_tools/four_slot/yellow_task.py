"""Single yellow, independent from historical four/five-part records."""
import json,re
from pathlib import Path
from a1_sorting_core.voice_batch_plan import validate_candidate,audit_utterance,PlanRejected

def load_task(path):
    d=json.loads(Path(path).read_text());p=validate_candidate(json.dumps(d['plan']));audit_utterance(d['transcript_unmodified'],p)
    if p.counts!=(0,1,0,0) or p.order!=('yellow',):raise PlanRejected('single_yellow_only')
    if d.get('plan_sha256')!=p.sha256 or not re.fullmatch('[0-9a-f]{32}',d.get('request_id','')):raise PlanRejected('single_task_identity_invalid')
    return d,p
