import os,json,time,subprocess
from pathlib import Path
R=Path('/home/yu/yuri_gazebo_performance/fast_sim_8da998b');case=Path((R/'latest-case.txt').read_text());m=json.loads((case/'metadata.json').read_text());os.environ.update(m['runs'][0]['env'])
from gz.transport13 import Node
import rclpy
from controller_manager_msgs.srv import ListControllers
rclpy.init();rn=rclpy.create_node('readiness_diagnostic');cl=rn.create_client(ListControllers,'/controller_manager/list_controllers');gn=Node();out=[]
for n in range(3):
 item={'time':time.time(),'pass':n}
 item['gz_initial']=str(gn.topic_info('/model/pusher/joint/pusher_slide/cmd_vel'))
 fresh=Node();item['gz_fresh']=str(fresh.topic_info('/model/pusher/joint/pusher_slide/cmd_vel'))
 p=subprocess.run(['gz','topic','-i','-t','/model/pusher/joint/pusher_slide/cmd_vel'],capture_output=True,text=True,timeout=4);item['cli']={'returncode':p.returncode,'out':p.stdout,'err':p.stderr}
 if cl.wait_for_service(timeout_sec=2):
  f=cl.call_async(ListControllers.Request());rclpy.spin_until_future_complete(rn,f,timeout_sec=3)
  item['controllers']=str(f.result()) if f.done() else 'TIMEOUT'
 out.append(item);print(json.dumps(item),flush=True)
 time.sleep(1)
(case/'discovery-results.json').write_text(json.dumps(out,indent=2));rn.destroy_node();rclpy.shutdown()
