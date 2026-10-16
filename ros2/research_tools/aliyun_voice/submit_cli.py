#!/usr/bin/env python3
"""Human-confirmed candidate submission. Cloud clients cannot import this path."""
import argparse,json,sys,time,uuid
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src/a1_sorting_core'))
from a1_sorting_core.voice_batch_plan import validate_candidate,audit_utterance
from a1_sorting_core.voice_batch_safety import ensure_geometry_review

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--candidate',type=Path,required=True);p.add_argument('--receipt',type=Path,required=True);a=p.parse_args()
 evidence=json.loads(a.candidate.read_text());transcript=evidence['transcript_unmodified'] if 'transcript_unmodified' in evidence else evidence['transcript'];candidate=evidence.get('plan')
 plan=validate_candidate(json.dumps(candidate) if candidate else evidence['raw_candidate']);audit_utterance(transcript,plan)
 print('ASR原文：'+transcript);print('已校验候选：'+json.dumps(plan.as_dict(),ensure_ascii=False));print('计划SHA256：'+plan.sha256)
 # Current green3 restriction applies even before user confirmation or ROS contact.
 ensure_geometry_review(plan)
 if not sys.stdin.isatty():raise SystemExit('必须由用户在本机交互终端确认，拒绝脚本管道自动批准。')
 if a.receipt.exists():raise SystemExit('已有提交记录不能覆盖或自动重发，请查看服务端状态。')
 if input('这会开始真实送料和运动。确认上述计划请输入 CONFIRM：').strip()!='CONFIRM':raise SystemExit('用户未确认，零提交。')
 import rclpy
 from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
 from std_msgs.msg import String
 from a1_interfaces.srv import SubmitBatchPlan
 rclpy.init();node=rclpy.create_node('human_voice_plan_submitter');status={}
 sub=node.create_subscription(String,'/voice_batch/status',lambda m:status.update(json.loads(m.data)),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
 try:
  limit=time.monotonic()+10
  while not status and time.monotonic()<limit:rclpy.spin_once(node,timeout_sec=.1)
  if status.get('state')!='WAITING_FOR_PLAN':raise SystemExit('新鲜服务会话未就绪或已有计划；不提交。')
  client=node.create_client(SubmitBatchPlan,'/voice_batch/submit')
  if not client.wait_for_service(timeout_sec=5):raise SystemExit('批次服务不可用，零提交。')
  req=SubmitBatchPlan.Request();req.session_id=status['session_id'];req.request_id=uuid.uuid4().hex;req.user_confirmed=True;req.plan_sha256=plan.sha256;req.blue,req.yellow,req.red,req.green=plan.counts;req.priority_first=plan.priority_first;req.proposed_order=list(plan.order)
  a.receipt.parent.mkdir(parents=True,exist_ok=True)
  # Write intent before the network call; disconnect must not create a new UUID/retry.
  receipt={'request_id':req.request_id,'session_id':req.session_id,'plan':plan.as_dict(),'plan_sha256':plan.sha256,'human_confirmed':True,'outcome':'pending_unknown_do_not_resend'};a.receipt.write_text(json.dumps(receipt,ensure_ascii=False,indent=2))
  future=client.call_async(req);rclpy.spin_until_future_complete(node,future,timeout_sec=10)
  if not future.done():raise SystemExit('结果未知，保留request_id并查询状态，禁止自动重发。')
  response=future.result();receipt.update(accepted=response.accepted,message=response.message,outcome='accepted' if response.accepted else 'rejected');a.receipt.write_text(json.dumps(receipt,ensure_ascii=False,indent=2));print(json.dumps(receipt,ensure_ascii=False,indent=2))
 finally:node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':
 from a1_sorting_core.voice_batch_plan import PlanRejected
 try:main()
 except PlanRejected as e:raise SystemExit('执行安全门阻断：'+str(e)) from None
