from gz.transport13 import Node
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.scene_pb2 import Scene
import json,sys
from pathlib import Path
g=Node();ok,res=g.request('/world/sorting_cell/scene/info',Empty(),Empty,Scene,3000)
print('OK',ok)
if ok:
 out={m.name:[m.pose.position.x,m.pose.position.y,m.pose.position.z] for m in res.model if m.name.startswith('part_')}
 print(json.dumps(out));Path(sys.argv[1]).write_text(json.dumps(out,indent=2))
