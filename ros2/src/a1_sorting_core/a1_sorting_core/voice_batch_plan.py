"""Untrusted cloud candidates become immutable plans only after closed validation.

No ROS imports, actuators, entity creation, network calls or automatic confirmation.
"""
from collections import Counter
from dataclasses import dataclass
import hashlib
import json
import re

COLORS=('blue','yellow','red','green')
FIELDS={'schema_version','action','counts','priority_first','proposed_order'}

class PlanRejected(ValueError):
    pass

def _object(pairs):
    obj={}
    for key,value in pairs:
        if key in obj:raise PlanRejected('duplicate_json_key')
        obj[key]=value
    return obj

@dataclass(frozen=True)
class BatchPlan:
    counts:tuple
    priority_first:str
    order:tuple

    @property
    def total(self):return sum(self.counts)

    def as_dict(self):
        return {'schema_version':1,'action':'sort_batch','counts':dict(zip(COLORS,self.counts)),
                'priority_first':self.priority_first,'proposed_order':list(self.order)}

    @property
    def sha256(self):
        return hashlib.sha256(json.dumps(self.as_dict(),sort_keys=True,separators=(',',':')).encode()).hexdigest()

def validate_candidate(payload,*,max_per_color=3):
    if not isinstance(payload,str) or len(payload.encode())>4096:raise PlanRejected('payload_size')
    try:obj=json.loads(payload,object_pairs_hook=_object,parse_constant=lambda _: (_ for _ in ()).throw(PlanRejected('nonfinite_json')))
    except (ValueError,TypeError) as e:raise PlanRejected('invalid_json') from e
    if not isinstance(obj,dict) or set(obj)!=FIELDS:raise PlanRejected('closed_schema')
    if type(obj['schema_version']) is not int or obj['schema_version']!=1 or obj['action']!='sort_batch':raise PlanRejected('unsupported_action_or_schema')
    counts=obj['counts']
    if not isinstance(counts,dict) or set(counts)!=set(COLORS):raise PlanRejected('four_color_counts_required')
    if any(type(v) is not int or not 0<=v<=max_per_color for v in counts.values()):raise PlanRejected('integer_count_0_to_3')
    total=sum(counts.values())
    if not 1<=total<=8:raise PlanRejected('total_1_to_8')
    first=obj['priority_first'];order=obj['proposed_order']
    if not isinstance(first,str) or first not in COLORS or counts[first]==0:raise PlanRejected('explicit_first_color_required')
    if not isinstance(order,list) or len(order)!=total or any(not isinstance(c,str) or c not in COLORS for c in order):raise PlanRejected('invalid_order')
    if Counter(order)!=Counter({c:n for c,n in counts.items() if n}):raise PlanRejected('inventory_order_mismatch')
    if order[0]!=first:raise PlanRejected('priority_mismatch')
    return BatchPlan(tuple(counts[c] for c in COLORS),first,tuple(order))

# A deliberately limited semantic checker, not an ASR substitute or an LLM fallback.
# It refuses ambiguous/free-form requests; failed cloud parsing must never be repaired
# into the demonstration plan. Only the supported explicit count/priority grammar is audited.
COLOR_ZH={'蓝':'blue','黄':'yellow','红':'red','绿':'green'}
NUMBERS={'零':0,'一':1,'二':2,'两':2,'三':3,'四':4,'五':5,'六':6,'七':7,'八':8,'九':9,'十':10}

def audit_utterance(text,plan):
    if not isinstance(text,str) or not text.strip() or len(text)>512:raise PlanRejected('empty_or_long_transcript')
    compact=re.sub(r'[\s，。,、；;！!：:]','',text)
    if any(x in compact for x in ('不','别','取消','不要','停止','改成','或者','随便','再加','少','减','负','.','-','点')):raise PlanRejected('unsupported_or_ambiguous_semantics')
    if not any(x in compact for x in ('分拣','分类','搬运')):raise PlanRejected('sort_intent_required')
    quantities=list(re.finditer(r'([0-9]+|[零一二两三四五六七八九十]+)(?:个|件|块)?([蓝黄红绿])(?:色的?|的)?(?:工件|物块|块)?',compact))
    if not quantities:raise PlanRejected('explicit_quantities_required')
    counts=dict.fromkeys(COLORS,0)
    for match in quantities:
        raw=match[1];number=int(raw) if raw.isascii() and raw.isdigit() else NUMBERS.get(raw)
        if number is None:raise PlanRejected('unsupported_number')
        counts[COLOR_ZH[match[2]]]+=number
    priority=re.findall(r'(?:先(?:分拣|搬运|处理|做|完成)?)([蓝黄红绿])(?:色的?|的)?|([蓝黄红绿])(?:色的?|的)?优先',compact)
    priorities=[COLOR_ZH[a or b] for a,b in priority]
    if len(priorities)!=1:raise PlanRejected('exactly_one_explicit_priority')
    # Every color occurrence must belong to a recognized quantity or priority clause.
    covered=set()
    for match in quantities:covered.update(range(match.start(),match.end()))
    for match in re.finditer(r'(?:先(?:分拣|搬运|处理|做|完成)?)([蓝黄红绿])(?:色的?|的)?|([蓝黄红绿])(?:色的?|的)?优先',compact):covered.update(range(match.start(),match.end()))
    if any(i not in covered for i,c in enumerate(compact) if c in COLOR_ZH):raise PlanRejected('unparsed_color_reference')
    residual=''.join(c for i,c in enumerate(compact) if i not in covered)
    for word in ('开始分拣','开始分类','开始搬运','请帮我','请','帮我','工件','物块','分拣','分类','搬运','开始','需要','我要','把','和','及','各'):
        residual=residual.replace(word,'')
    if residual:raise PlanRejected('unparsed_utterance_semantics')
    if tuple(counts[c] for c in COLORS)!=plan.counts or priorities[0]!=plan.priority_first:raise PlanRejected('transcript_plan_mismatch')
    return True
