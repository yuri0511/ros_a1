"""Actual MoveIt services, synthetic object observations, no hardware/Gazebo."""
import json,threading,math
from pathlib import Path
import rclpy
from a1_sorting_core.retained_scene_sync import RosRetainedSceneSync,SceneSyncError
from a1_sorting_core.four_slot_capacity import SlotBank,CapacityError
ROOT=Path(__file__).resolve().parents[4];cfg=json.loads((ROOT/'worktree/ros2/src/a1_sorting_core/config/four_slot_candidate.json').read_text())
rclpy.init();node=rclpy.create_node('retained_scene_offline_test');executor=rclpy.executors.SingleThreadedExecutor();executor.add_node(node);thread=threading.Thread(target=executor.spin);thread.start();rows=[]
try:
    sync=RosRetainedSceneSync(node)
    for bn,b in cfg['bins'].items():
        bank=SlotBank({bn:b});observations={}
        for i,slot in enumerate(b['slots']):
            part=f'synthetic_{bn}_{i+1}';sid=bank.reserve(bn,part);xyz=slot['xyz'].copy();xyz[0]+=.001
            bank.occupied(sid,part,xyz,1.,1.);observations[part]={'position':xyz,'quaternion':[0.,0.,math.sin(.05),math.cos(.05)],'stamp':1.}
            ack=sync.sync(bank.slots,observations,1.);assert len(ack['objects'])==i+1
            rows.append({'bin':bn,'count':i+1,'ack':ack,'inventory':json.loads(json.dumps(bank.slots)),'observations':json.loads(json.dumps(observations))})
        try:bank.reserve(bn,'fifth');raise AssertionError('fifth incorrectly accepted')
        except CapacityError:rows.append({'bin':bn,'fifth':'REJECTED'})
        try:sync.sync(bank.slots,{},1.);raise AssertionError('missing observations accepted')
        except SceneSyncError:rows.append({'bin':bn,'missing_frame':'BLOCKED','ack':sync.ack})
finally:
    (ROOT/'audit/stage2/live-scene-results.json').write_text(json.dumps({'synthetic_observations':True,'real_moveit_services':True,'gazebo':False,'rows':rows},indent=2))
    executor.shutdown();thread.join(timeout=3);node.destroy_node();rclpy.try_shutdown()
print('PASS: 16 sequential retained scene updates; 4 fifth-part refusals; 4 missing-frame blocks')
