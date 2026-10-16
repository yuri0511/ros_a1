import time,json,sys
from pathlib import Path
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
run=Path(sys.argv[1]);poses={};g=Node()
def cb(m):
 for p in m.pose:
  if p.name.startswith('part_') and p.name[5:].isdigit():poses[p.name]={'xyz':[p.position.x,p.position.y,p.position.z],'quaternion':[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]}
for t in ('/world/sorting_cell/pose/info','/world/sorting_cell/dynamic_pose/info'):g.subscribe(Pose_V,t,cb)
deadline=time.monotonic()+2
while time.monotonic()<deadline:time.sleep(.1)
(run/'final-positions.json').write_text(json.dumps({'wall':time.time(),'world_frame':True,'parts':poses},indent=2))
