"""Proposed voice JSON contract only. No ROS client, audio, LLM or motion code."""
import json

class IntentRejected(ValueError):pass

def plan_intent(payload,catalog,*,target_map=None,mode='manual_voice',halted=False):
    if not isinstance(payload,str) or len(payload.encode('utf-8'))>4096:raise IntentRejected('invalid_payload_size')
    try:o=json.loads(payload)
    except (ValueError,TypeError) as e:raise IntentRejected('invalid_json') from e
    if not isinstance(o,dict) or set(o)-{'action','color','target'}:raise IntentRejected('unknown_fields')
    action=o.get('action');color=o.get('color','none');target=o.get('target','none')
    if action not in ('sort','stop','status') or color not in ('red','blue','green','none') or target not in ('left','right','none'):raise IntentRejected('unsupported_enum')
    if action=='status':return {'kind':'read_only_status_proposal','implemented':False}
    if action=='stop':return {'kind':'safe_stop_api_required','implemented':False,'sort_task_request':None,'must_not_kill_processes':True}
    if halted or mode!='manual_voice':raise IntentRejected('scheduler_not_available_for_voice')
    if color=='none' and target=='none':return {'kind':'sort_task_proposal','command':'start_batch','part_id':'','target_bin':''}
    target_bin=''
    if target!='none':
        target_bin=(target_map or {}).get(target,'')
        if target_bin not in ('bin_blue','bin_yellow','bin_red','bin_green'):raise IntentRejected('explicit_target_mapping_required')
    candidates=[x for x in catalog if isinstance(x,dict) and x.get('available') is True and x.get('busy') is False and isinstance(x.get('part_id'),str) and x['part_id'] and (color=='none' or x.get('category')==color)]
    if len(candidates)!=1:raise IntentRejected('no_unique_observed_part')
    return {'kind':'sort_task_proposal','command':'dispatch_to' if target_bin else 'dispatch_single','part_id':candidates[0]['part_id'],'target_bin':target_bin}
