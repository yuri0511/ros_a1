import sys,time,csv,json
from pathlib import Path
import rclpy
from ros_gz_interfaces.srv import SpawnEntity,DeleteEntity
from ros_gz_interfaces.msg import Entity
run=Path(sys.argv[1]);rclpy.init();node=rclpy.create_node('contact_positive_control')
spawn=node.create_client(SpawnEntity,'/world/sorting_cell/create');remove=node.create_client(DeleteEntity,'/world/sorting_cell/remove')
def call(client,request):
 f=client.call_async(request);rclpy.spin_until_future_complete(node,f,timeout_sec=5);return f.result()
proof={'fixture_only':True,'boxes':[],'verified':False}
try:
 assert spawn.wait_for_service(timeout_sec=25) and remove.wait_for_service(timeout_sec=10)
 time.sleep(3)
 for arm,y in ((1,.410),(2,-.410)):
  name=f'positive_shoulder_box_{arm}';q=SpawnEntity.Request();q.entity_factory.name=name
  q.entity_factory.sdf=f'<sdf version="1.11"><model name="{name}"><static>true</static><link name="link"><collision name="collision"><geometry><box><size>0.04 0.04 0.04</size></box></geometry></collision></link></model></sdf>'
  q.entity_factory.pose.position.x=.600;q.entity_factory.pose.position.y=y;q.entity_factory.pose.position.z=.602;q.entity_factory.pose.orientation.w=1.
  response=call(spawn,q);assert response and response.success
  proof['boxes'].append(name)
 deadline=time.monotonic()+12;found={}
 while time.monotonic()<deadline:
  with (run/'contacts.csv').open() as f:
   for row in csv.DictReader(f):
    for name in proof['boxes']:
     if name in row['collision1']+row['collision2'] and 'shoulder_link_collision' in row['collision1']+row['collision2']:
      found[name]=row
  if len(found)==2:break
  time.sleep(.5)
 proof['contacts']=found;proof['verified']=len(found)==2
 assert proof['verified'],proof
finally:
 for name in proof['boxes']:
  q=DeleteEntity.Request();q.entity.name=name;q.entity.type=Entity.MODEL
  proof.setdefault('remove_results',{})[name]=bool(getattr(call(remove,q),'success',False))
 proof['heartbeat']=(run/'contacts.heartbeat').read_text() if (run/'contacts.heartbeat').exists() else None
 (run/'positive-control.json').write_text(json.dumps(proof,indent=2));node.destroy_node();rclpy.shutdown()
print(json.dumps(proof))
