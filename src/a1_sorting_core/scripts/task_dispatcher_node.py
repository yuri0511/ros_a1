#!/usr/bin/env python3
"""任务调度节点:来料登记 -> 任务队列 -> 串行派发(ROS1 移植自组长 task_dispatcher_node.py)。

接口(与组长版相同,均为 a1_interfaces 自定义接口):
- 订阅 /parts/arrival (PartInfo):来料登记(spawn_parts 或视觉节点发布);
- 订阅 /task/status (TaskStatus):执行节点的状态回报,用于推进队列;
- 发布 /task/command (PartInfo):向执行节点派发单件任务;
- 服务 /sort_task (SortTask):start_batch / dispatch_single 外部入口;
  ROS1 版新增 reset:清空来料目录与队列(run_trials 每轮重新生成同名 part_1..3 前调用)。

串行保证:同一时刻只有一个任务在跑,当前件终态后延迟 dispatch_delay 派发下一件。
推进决策仍由纯逻辑类 RelaySequencer 负责(原样沿用,relay_mode=false 即单臂)。

相对组长版删掉的部分(Panda 单臂 + 无 Gazebo 用不到):双臂接力 / 并联分色、
按需来料 demand_spawn、推板 belt_control、halt_on_wedge。
"""
import threading

import rospy
from std_msgs.msg import String

from a1_interfaces.msg import PartInfo, TaskStatus
from a1_interfaces.srv import SortTask, SortTaskResponse
from a1_sorting_core.batch_protocol import (BATCH_TOPIC, READY_PREFIX,
                                            parse_event)
from a1_sorting_core.relay_sequencer import DISPATCH, RelaySequencer

TERMINAL_STATES = ('COMPLETED', 'FAILED')


class TaskDispatcherNode(object):

    def __init__(self):
        self._auto_start = bool(rospy.get_param('~auto_start', False))
        self._dispatch_delay = float(rospy.get_param('~dispatch_delay', 1.0))
        self._lock = threading.Lock()
        self._next_timer = None
        self._hold = False
        self._reset_state()

        self._pub_command = rospy.Publisher('/task/command', PartInfo, queue_size=10)
        self._pub_batch = rospy.Publisher(BATCH_TOPIC, String, queue_size=10, latch=True)
        rospy.Subscriber('/parts/arrival', PartInfo, self._on_arrival)
        rospy.Subscriber('/task/status', TaskStatus, self._on_status)
        rospy.Subscriber(BATCH_TOPIC, String, self._on_batch_event)
        rospy.Service('/sort_task', SortTask, self._on_sort_task)
        rospy.set_param(READY_PREFIX + '/task_dispatcher', True)
        rospy.on_shutdown(lambda: rospy.set_param(READY_PREFIX + '/task_dispatcher', False))
        rospy.loginfo('调度器就绪(单臂): /sort_task 服务可用 (start_batch/dispatch_single/reset),'
                      'auto_start=%s', self._auto_start)

    def _reset_state(self):
        self._catalog = {}   # part_id -> 最新来料信息
        self._order = []     # 来料顺序
        self._seq = RelaySequencer(relay_mode=False, relay_delay=0.0,
                                   dispatch_delay=self._dispatch_delay)

    # ---------------------------------------------------------------- 来料
    def _on_arrival(self, msg):
        with self._lock:
            if msg.part_id not in self._catalog:
                self._order.append(msg.part_id)
                rospy.loginfo('来料登记: %s (类别 %s)', msg.part_id, msg.category)
                if self._auto_start:
                    self._seq.enqueue(msg.part_id)
            self._catalog[msg.part_id] = msg
        if self._auto_start:
            self._try_dispatch()

    # ---------------------------------------------------------------- 服务
    def _on_sort_task(self, req):
        accepted, message = False, ''
        with self._lock:
            if req.command == 'start_batch':
                added = 0
                for part_id in self._order:
                    if self._seq.enqueue(part_id):
                        added += 1
                accepted, message = True, '新入队 %d 件,队列现有 %d 件' % (added, self._seq.queue_size)
            elif req.command == 'dispatch_single':
                if req.part_id not in self._catalog:
                    message = '未知工件: %s' % req.part_id
                elif self._seq.enqueue(req.part_id):
                    accepted, message = True, '%s 已入队' % req.part_id
                else:
                    reason = self._seq.block_reason(req.part_id)
                    if reason == 'current':
                        message = '%s 正在执行,没有重复入队' % req.part_id
                    elif reason == 'done':
                        message = '%s 本轮已完成,没有重复入队' % req.part_id
                    else:
                        message = '%s 已在队列中,没有重复入队' % req.part_id
            elif req.command == 'reset':
                if self._seq.current is not None:
                    message = '当前件 %s 未完成,不能 reset' % self._seq.current
                else:
                    self._destroy_next_timer()
                    self._reset_state()
                    self._hold = False
                    accepted, message = True, '已清空来料目录与队列'
            else:
                message = '未知命令: %s' % req.command
        rospy.loginfo('/sort_task: %s', message)
        if accepted and req.command == 'reset':
            self._pub_batch.publish(String(data='reset'))
        elif accepted:
            self._try_dispatch()
        return SortTaskResponse(accepted, message)

    def _on_batch_event(self, msg):
        kind, _batch_id = parse_event(msg.data)
        if kind == 'begin':
            # 新的一批是明确的重新开始。若手臂仍回不到待机位,下一件会再次停住。
            self._hold = False

    # ---------------------------------------------------------------- 状态
    def _on_status(self, msg):
        if msg.state not in TERMINAL_STATES:
            return
        with self._lock:
            accepted, actions = self._seq.on_terminal(msg.part_id, msg.arm, msg.state)
            if accepted:
                summary = ('%s 分拣完成(%s)' % (msg.part_id, msg.detail) if msg.state == 'COMPLETED'
                           else '%s 分拣失败(%s)' % (msg.part_id, msg.detail))
                rospy.loginfo('任务终态: %s,队列剩余 %d 件', summary, self._seq.queue_size)
                if msg.detail == 'standby_failed':
                    self._hold = True
                    actions = []
                    rospy.logerr('手臂未能回待机位,停止自动派发。复位后可再开始。')
        for kind, delay in actions:
            if kind == DISPATCH and self._next_timer is None:
                self._next_timer = threading.Timer(delay, self._try_dispatch)
                self._next_timer.daemon = True
                self._next_timer.start()

    # ---------------------------------------------------------------- 派发
    def _try_dispatch(self):
        self._destroy_next_timer()
        if self._hold:
            rospy.logwarn('派发挂起: 上一件结束后手臂没有回到待机位')
            return
        if self._pub_command.get_num_connections() == 0:
            # ROS1 特有:执行节点要等 move_group 就绪才订阅 /task/command,
            # 此前发布会丢消息 -> 1s 后重试(组长版靠执行器懒连接 move_group 规避)
            rospy.loginfo_throttle(5.0, '派发挂起: 等待执行节点订阅 /task/command')
            self._next_timer = threading.Timer(1.0, self._try_dispatch)
            self._next_timer.daemon = True
            self._next_timer.start()
            return
        with self._lock:
            part_id = self._seq.try_dispatch('arm1')
            command = self._catalog[part_id] if part_id is not None else None
        if command is not None:
            self._pub_command.publish(command)
            rospy.loginfo('派发任务: %s (类别 %s)', part_id, command.category)

    def _destroy_next_timer(self):
        if self._next_timer is not None:
            self._next_timer.cancel()
            self._next_timer = None


if __name__ == '__main__':
    rospy.init_node('task_dispatcher')
    TaskDispatcherNode()
    rospy.spin()
