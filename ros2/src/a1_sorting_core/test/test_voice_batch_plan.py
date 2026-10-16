import copy,json
import pytest
from a1_sorting_core.voice_batch_plan import validate_candidate,audit_utterance,PlanRejected

DEMO=dict(schema_version=1,action='sort_batch',counts=dict(blue=0,yellow=1,red=1,green=3),priority_first='yellow',proposed_order=['yellow','red','green','green','green'])

def valid(obj=DEMO):return validate_candidate(json.dumps(obj))

@pytest.mark.parametrize('text',['开始分拣。三个绿色，一个红色，一个黄的。先分拣黄色。','请帮我分拣3件绿色工件、1件红色工件、1件黄色工件，黄色优先。'])
def test_original_and_alternative_language_audit(text):
    p=valid();assert p.counts==(0,1,1,3) and p.total==5 and p.order[0]=='yellow';assert audit_utterance(text,p)

@pytest.mark.parametrize('obj',[dict(DEMO,joints=[1]),dict(DEMO,request_id='model_generated'),dict(DEMO,action='stop'),dict(DEMO,schema_version=True),dict(DEMO,priority_first=''),dict(DEMO,priority_first='blue'),dict(DEMO,proposed_order=['red','yellow','green','green','green']),dict(DEMO,proposed_order=['yellow','red','green','green']),dict(DEMO,counts=dict(blue=0,yellow=1,red=1,green=4)),dict(DEMO,counts=dict(blue=0,yellow=1,red=1,green=-1)),dict(DEMO,counts=dict(blue=0,yellow=1,red=1,green=2.5)),dict(DEMO,counts=dict(blue=0,yellow=1,red=1,green=True)),dict(DEMO,counts=dict(blue=0,yellow=1,red=1,green=3,purple=1)),dict(DEMO,counts=dict(blue=3,yellow=3,red=3,green=0))])
def test_invalid_cloud_candidate_is_rejected(obj):
    with pytest.raises(PlanRejected):valid(obj)

@pytest.mark.parametrize('raw',['{}','[]','not json','{"schema_version":1,"schema_version":1}','{"x":NaN}','x'*4097])
def test_closed_raw_json(raw):
    with pytest.raises(PlanRejected):validate_candidate(raw)

@pytest.mark.parametrize('text',['','开始分拣三个绿色一个红色一个黄的','不要分拣三个绿色一个红色一个黄的，先黄色','开始分拣三个绿色一个红色一个黄色一个紫色，先黄色','开始分拣三个绿色一个红色两个黄色，先黄色','开始分拣三个绿色一个红色一个黄的，先红色','开始分拣-3个绿色一个红色一个黄的先黄色','开始分拣3.0个绿色一个红色一个黄的先黄色','启动Gazebo，分拣三个绿色一个红色一个黄的先黄色','开始分拣三个绿色一个红色一个黄的先黄色，关节转到0'])
def test_semantic_checker_never_repairs_unsafe_asr_or_llm(text):
    with pytest.raises(PlanRejected):audit_utterance(text,valid())

def test_order_variation_remaining_is_not_hardcoded():
    o=copy.deepcopy(DEMO);o['proposed_order']=['yellow','green','red','green','green'];assert valid(o).order[1]=='green'

def test_immutable_plan_hash_same_despite_json_key_order():assert valid().sha256==validate_candidate(json.dumps(DEMO,sort_keys=True)).sha256
