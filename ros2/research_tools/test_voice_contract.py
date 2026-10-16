import json
import pytest
from voice_contract import plan_intent,IntentRejected
PARTS=[dict(part_id='real_part_7',category='red',available=True,busy=False)]
def test_real_part_identity_used():assert plan_intent('{"action":"sort","color":"red"}',PARTS)['part_id']=='real_part_7'
@pytest.mark.parametrize('catalog',[[],[dict(PARTS[0],busy=True)],[dict(PARTS[0],available=False)],PARTS+[dict(PARTS[0],part_id='real_part_8')]])
def test_no_invented_or_ambiguous_part(catalog):
 with pytest.raises(IntentRejected):plan_intent('{"action":"sort","color":"red"}',catalog)
def test_left_mapping_required_never_guessed():
 with pytest.raises(IntentRejected):plan_intent('{"action":"sort","color":"red","target":"left"}',PARTS)
 p=plan_intent('{"action":"sort","color":"red","target":"left"}',PARTS,target_map={'left':'bin_yellow'});assert p['command']=='dispatch_to' and p['target_bin']=='bin_yellow'
@pytest.mark.parametrize('payload',['broken','[]','{"action":"execute_shell"}','{"action":"sort","part_id":"invented"}','{"action":"sort","color":"yellow"}','{"action":"sort","target":"bin_red"}'])
def test_schema_closed(payload):
 with pytest.raises(IntentRejected):plan_intent(payload,PARTS)
@pytest.mark.parametrize('fields',[{'halted':True},{'mode':'automatic_rule'},{'mode':'automatic_q'}])
def test_voice_cannot_bypass_scheduler(fields):
 with pytest.raises(IntentRejected):plan_intent('{"action":"sort","color":"red"}',PARTS,**fields)
def test_stop_never_becomes_a_sort_request_or_process_kill():
 p=plan_intent('{"action":"stop"}',PARTS,halted=True,mode='automatic_rule');assert not p['implemented'] and p['sort_task_request'] is None and p['must_not_kill_processes']
def test_status_is_read_only_unimplemented_proposal():assert not plan_intent('{"action":"status"}',PARTS)['implemented']
def test_batch_means_existing_catalog_not_fake_eight_parts():assert plan_intent('{"action":"sort"}',[])['command']=='start_batch'
