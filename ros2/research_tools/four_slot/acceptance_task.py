"""Bounded B/C task records; C reuses the saved genuine ASR/LLM record."""
import json,uuid,hashlib
from pathlib import Path
from a1_sorting_core.voice_batch_plan import validate_candidate,audit_utterance,PlanRejected
SOURCE=Path('/home/yu/yuri_gazebo_performance/voice_batch_aliyun/audit/real-speech-validated.json')
def five_record():
    d=json.loads(SOURCE.read_text());p=validate_candidate(json.dumps(d['plan']));audit_utterance(d['transcript_unmodified'],p)
    if p.counts!=(0,1,1,3) or p.order!=('yellow','red','green','green','green') or p.sha256!=d['plan_sha256']:raise PlanRejected('historical_five_record_invalid')
    return dict(d,acceptance_stage='C',request_id=uuid.uuid4().hex,historical_record=str(SOURCE),historical_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),note='Reused historical ASR/LLM; no new cloud request or physical success claim')
def load_task(path):
    d=json.loads(Path(path).read_text())
    if d.get('acceptance_stage')!='C':
        from yellow_task import load_task as yellow
        return yellow(path)
    p=validate_candidate(json.dumps(d['plan']));audit_utterance(d['transcript_unmodified'],p)
    original=json.loads(SOURCE.read_text())
    if p.counts!=(0,1,1,3) or p.order!=('yellow','red','green','green','green') or d['plan']!=original['plan'] or d['transcript_unmodified']!=original['transcript_unmodified'] or d['historical_sha256']!=hashlib.sha256(SOURCE.read_bytes()).hexdigest():raise PlanRejected('five_record_modified')
    return d,p
