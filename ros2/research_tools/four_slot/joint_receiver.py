"""Dedicated JointState ingress. No discovery, file IO or safety work in callback.

KEEP_LAST DDS depth remains 100. The separate Python queue never overwrites:
overflow or processing age > original .5s freshness latches a fault.
"""
import json,queue,threading,time
from pathlib import Path
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import JointState
from a1_sorting_core.physical_ready import publisher_identity

class JointReceiver:
    def __init__(self, handler, fault, log, *, topic='/joint_states', capacity=2048):
        self.handler=handler;self.fault_handler=fault;self.log=Path(log)
        self.queue=queue.Queue(capacity);self.stop=threading.Event();self.fault=None
        self.received=0;self.processed=0;self.dropped=0;self.highwater=0;self.max_callback=0.;self.max_age=0.
        self.publishers=[];self.discovery_wall=0.;self.discovery_rows=[]
        self.node=rclpy.create_node('isolated_joint_ingress')
        self.topic=topic
        self.sub=self.node.create_subscription(JointState,topic,self._receive,QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.executor=SingleThreadedExecutor();self.executor.add_node(self.node)
        self.threads=[threading.Thread(target=self.executor.spin,name='joint-rx'),threading.Thread(target=self._discover,name='joint-discovery'),threading.Thread(target=self._process,name='joint-consumer')]
        for t in self.threads:t.start()

    def fail(self,reason):
        if self.fault is None:self.fault=reason

    def _receive(self,msg,info):
        start=time.monotonic();wall=time.time();self.received+=1
        item=(msg,info,start,wall,time.monotonic(),self.received)
        try:self.queue.put_nowait(item)
        except queue.Full:self.dropped+=1;self.fail('joint_ingress_queue_overflow')
        self.highwater=max(self.highwater,self.queue.qsize());self.max_callback=max(self.max_callback,time.monotonic()-start)

    def _discover(self):
        while not self.stop.is_set():
            try:
                start=time.monotonic();pubs=self.node.get_publishers_info_by_topic(self.topic)
                self.publishers=pubs;self.discovery_wall=time.monotonic()
                row={'wall':time.time(),'query_seconds':self.discovery_wall-start,'publishers':[{'node':p.node_name,'gid':list(p.endpoint_gid),'qos':str(p.qos_profile)} for p in pubs]}
                self.discovery_rows.append(row)
                if len(pubs)>1:self.fail('joint_publisher_not_unique')
            except Exception as e:self.fail('joint_discovery:'+str(e))
            self.stop.wait(.1)

    def _process(self):
        announced=False
        with self.log.open('w',buffering=65536) as f:
            while not self.stop.is_set() or not self.queue.empty():
                if self.fault and not announced:self.fault_handler(self.fault);announced=True
                try:msg,info,receipt,wall,enqueued,seq=self.queue.get(timeout=.02)
                except queue.Empty:continue
                while not self.publishers and time.monotonic()-receipt<.5 and not self.stop.is_set():time.sleep(.001)
                begin=time.monotonic();age=begin-receipt;self.max_age=max(self.max_age,age)
                source=None;error=None
                try:
                    if age>.5:raise RuntimeError('joint_processing_stale')
                    if begin-self.discovery_wall>.5:raise RuntimeError('joint_discovery_stale')
                    source=publisher_identity(info,self.publishers)
                    if self.fault is None:self.handler(msg,info,receipt,source)
                except Exception as e:error=str(e);self.fail('joint_consumer:'+error)
                self.processed+=1
                stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                f.write(json.dumps({'seq':seq,'stamp':stamp,'receipt_wall':wall,'receipt_mono':receipt,'enqueue_mono':enqueued,'process_mono':begin,'done_mono':time.monotonic(),'queue_length':self.queue.qsize(),'dropped':self.dropped,'source':source,'error':error,'positions':list(msg.position),'names':list(msg.name),'message_info':str(info)})+'\n')
            if self.fault and not announced:self.fault_handler(self.fault)

    def close(self):
        self.executor.shutdown(timeout_sec=2);self.threads[0].join(2)
        self.stop.set()
        for t in self.threads[1:]:t.join(3)
        self.node.destroy_node()
        result={'received':self.received,'processed':self.processed,'overflow_drops':self.dropped,'queue_highwater':self.highwater,'max_callback_seconds':self.max_callback,'max_processing_age_seconds':self.max_age,'fault':self.fault,'threads_stopped':all(not t.is_alive() for t in self.threads),'discovery':self.discovery_rows}
        self.log.with_suffix('.summary.json').write_text(json.dumps(result,indent=2))
        return result
