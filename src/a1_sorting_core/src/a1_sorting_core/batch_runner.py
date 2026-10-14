"""在已经启动的仿真里跑一轮或一件。本模块会在 20.04 上 import rospy。"""
from __future__ import annotations

import rospy
from a1_interfaces.msg import PartResult
from a1_interfaces.srv import SortTask, SpawnParts
from a1_sorting_core.batch_protocol import (PART_RESULT_TOPIC,
                                            batch_id_from_spawn_message)


class BatchRunner(object):

    def __init__(self):
        self.results = {}
        self._batch_filter = ''
        rospy.Subscriber(PART_RESULT_TOPIC, PartResult, self._on_result, queue_size=20)
        for name in ('/sort_task', '/parts/spawn'):
            rospy.wait_for_service(name, 30.0)
        self._sort = rospy.ServiceProxy('/sort_task', SortTask)
        self._spawn = rospy.ServiceProxy('/parts/spawn', SpawnParts)

    def _on_result(self, msg):
        if self._batch_filter and msg.batch_id != self._batch_filter:
            return
        self.results[msg.part_id] = msg

    def clear_part_strategies(self):
        if rospy.has_param('/a1/strategy_for'):
            rospy.delete_param('/a1/strategy_for')

    def use_one_strategy(self, strategy_id):
        """整批工件使用同一种策略。"""
        self.clear_part_strategies()
        rospy.set_param('/a1/active_strategy', int(strategy_id))

    def use_part_strategies(self, mapping):
        """每件单独指定策略。mapping 的键是 part_id。"""
        self.clear_part_strategies()
        rospy.set_param('/a1/active_strategy', -1)
        for part_id, strategy_id in mapping.items():
            rospy.set_param('/a1/strategy_for/' + part_id, int(strategy_id))

    def run_parts(self, colors, timeout):
        """reset、生成、start_batch，然后等到每件都有 /result/part。

        返回 (batch_id, part_ids)。超时的工件不会出现在 results 里。
        """
        self.results = {}
        self._batch_filter = ''
        if not self._wait_reset():
            raise RuntimeError('调度器一直忙，无法 reset')
        spawned = self._spawn(','.join(colors))
        if not spawned.success:
            raise RuntimeError('生成工件失败: %s' % spawned.message)
        batch_id = batch_id_from_spawn_message(spawned.message)
        self._batch_filter = batch_id
        accepted = self._sort('start_batch', '')
        if not accepted.accepted:
            raise RuntimeError('start_batch 被拒绝: %s' % accepted.message)
        part_ids = ['part_%d' % index for index in range(1, len(colors) + 1)]
        deadline = rospy.get_time() + float(timeout)
        while rospy.get_time() < deadline and not rospy.is_shutdown():
            if set(part_ids) <= set(self.results):
                return batch_id, part_ids
            rospy.sleep(0.3)
        return batch_id, part_ids

    def _wait_reset(self, timeout=120.0):
        deadline = rospy.get_time() + timeout
        while rospy.get_time() < deadline and not rospy.is_shutdown():
            if self._sort('reset', '').accepted:
                return True
            rospy.sleep(0.5)
        return False
