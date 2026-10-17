import json,pytest
from a1_sorting_core.four_slot_capacity import SlotBank,CapacityError,load_motion_candidate

def bins():return {'bin_green':{'slots':[{'id':f'bin_green/slot_{i+1}','xyz':[.86+.08*(i%2),-.32+.08*(i//2),.745]} for i in range(4)]}}
def test_four_retained_and_fifth_rejected(tmp_path):
    bank=SlotBank(bins(),tmp_path/'inventory.json')
    for i,s in enumerate(bins()['bin_green']['slots']):
        sid=bank.reserve('bin_green',str(i));assert sid==s['id'];bank.occupied(sid,str(i),s['xyz'],1,1.1)
    with pytest.raises(CapacityError):bank.reserve('bin_green','5')
    assert all(x['state']=='OCCUPIED' for x in SlotBank(bins(),tmp_path/'inventory.json').slots.values())
def test_reservation_failure_and_restart_are_not_reusable(tmp_path):
    bank=SlotBank(bins(),tmp_path/'inventory.json');sid=bank.reserve('bin_green','part')
    with pytest.raises(CapacityError):bank.reserve('bin_green','part')
    bank=SlotBank(bins(),tmp_path/'inventory.json');assert bank.slots[sid]['state']=='FAULT'
    assert bank.reserve('bin_green','other')!=sid

def test_actual_delete_and_later_frame_required():
    b=SlotBank(bins());sid=b.reserve('bin_green','p');b.fault(sid,'uncertain')
    for ack,frame,visible in [(False,2,()),(True,1,()),(True,2,('p',))]:
        with pytest.raises(CapacityError):b.clear(sid,'p',delete_ack=ack,delete_sim=1,frame_sim=frame,visible=visible)
    b.clear(sid,'p',delete_ack=True,delete_sim=1,frame_sim=2,visible=());assert b.reserve('bin_green','q')==sid

def test_placement_identity_freshness_and_xy():
    b=SlotBank(bins());sid=b.reserve('bin_green','p');xyz=b.slots[sid]['xyz']
    for part,pos,stamp in [('wrong',xyz,1),('p',[0,0,.745],1),('p',xyz,0),('p',xyz,2)]:
        with pytest.raises(CapacityError):b.occupied(sid,part,pos,stamp,1.1)
    assert b.slots[sid]['state']=='RESERVED'

def test_uncertified_motion_is_blocked(tmp_path):
    p=tmp_path/'candidate.json';p.write_text(json.dumps({'schema':1,'motion_validated':False}))
    with pytest.raises(CapacityError):load_motion_candidate(p)

def test_contact_categories_do_not_relax_wall_stop():
    from a1_sorting_core.contact_policy_four import classify
    finger='ur::gripper_finger_left_link::collision'
    assert classify(finger,'part_1::link',.001,held_part='part_1').action=='RECORD'
    assert classify(finger,'part_2::link',1e-8,held_part='part_1').action=='STOP'
    assert classify(finger,'bin_yellow::wall',55.1e-6).action=='STOP'
    assert classify('part_1::link','tabletop::link::floor_green',.0001,placed_bins={'part_1':'bin_green'}).action=='RECORD'
    assert classify(finger,'ur::arm2_forearm_link::c',0).action=='STOP'

def test_retained_voice_mode_does_not_require_recycling():
    from a1_sorting_core.voice_batch_state import VoiceBatchState
    from a1_sorting_core.voice_batch_plan import BatchPlan
    s=VoiceBatchState(retain_capacity=4);s.state='RUNNING';s.hardware_ready=True;s.plan=BatchPlan((0,0,0,4),'green',('green',)*4)
    s.issued=['green'];s.parts={'part_1':{'color':'green','placed':True,'ready':True,'released':True}}
    assert s.reserve(1,True,['part_1'],logical_wall=1,logical_sim=1)=='green'
    legacy=VoiceBatchState();legacy.state='RUNNING';legacy.hardware_ready=True;legacy.plan=s.plan;legacy.issued=['green'];legacy.parts=s.parts
    assert legacy.reserve(1,True,['part_1'],logical_wall=1,logical_sim=1) is None

def test_candidate_world_floor_and_recorder_are_consistent():
    from pathlib import Path
    import xml.etree.ElementTree as E
    import yaml
    root=Path(__file__).resolve().parents[3]
    cfg=json.loads((root/'src/a1_sorting_core/config/four_slot_candidate.json').read_text())
    params=yaml.safe_load((root/'src/a1_sorting_core/config/four_slot_overlay.yaml').read_text())
    world=E.parse(root/'src/a1_cell_sim/worlds/four_slot_1ms.sdf')
    assert float(world.find('.//physics/max_step_size').text)==.001
    assert cfg['motion_validated'] is False
    rp=params['/result_recorder']['ros__parameters'];assert rp['recycle_delay']==0
    models={m.get('name'):m for m in world.findall('.//model')}
    for bn,b in cfg['bins'].items():
        assert list(map(float,models[bn].find('pose').text.split()))[:2]==b['center']
        assert rp[f'bins.{bn}.inner_half_x']==cfg['inner'][0]/2
        assert rp[f'bins.{bn}.inner_half_y']==cfg['inner'][1]/2
        assert rp[f'bins.{bn}.x']==b['center'][0]
        floor=models['tabletop'].find(f"link/collision[@name='floor_{bn[4:]}']")
        assert list(map(float,floor.find('geometry/box/size').text.split()))[:2]==cfg['outer']
        assert len({s['id'] for s in b['slots']})==4
        for s in b['slots']:
            assert all(abs(s['xyz'][i]-b['center'][i])+cfg['cube']/2<cfg['inner'][i]/2 for i in (0,1))

def test_four_per_color_validation_is_opt_in():
    from a1_sorting_core.voice_batch_plan import validate_candidate,PlanRejected
    p=json.dumps({'schema_version':1,'action':'sort_batch','counts':{'blue':0,'yellow':0,'red':0,'green':4},'priority_first':'green','proposed_order':['green']*4})
    with pytest.raises(PlanRejected):validate_candidate(p)
    assert validate_candidate(p,max_per_color=4).total==4
