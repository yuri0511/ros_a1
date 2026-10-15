"""结果记录节点:任务终态 -> 真值位姿判定 -> CSV 台账 + 数量反馈。

接口:
- 订阅 /task/status (TaskStatus):COMPLETED/FAILED 时判定并记录;
- 订阅 /parts/arrival (PartInfo):建立 part_id→category 表,使 CSV 的
  category 列记录视觉识别出的真实颜色(TaskStatus 不含 category,且
  "以外部观测为准判定结果"的原则不变——category 只作台账标注,不参与
  PLACED/MISPLACED 判定);
- 订阅逻辑相机话题(ros_gz_interfaces/LogicalCameraImage):
  工件位姿(传感器系相对值,与已知安装位姿复合成世界系);
- 发布 /result/counts (Int32MultiArray):[料区A数, 料区B数, 失败数, 放错数];
- 发布 /result/markers (MarkerArray):料区可视框与计数标签(RViz 用)。

判定逻辑:以逻辑相机观测的工件俯视坐标与料区 AABB 的包含关系为准,
与执行节点"自说自话"的完成回报相互独立,构成真实的结果闭环。
带壁箱子场景(契约 §9)判定用内口 AABB(bins.<name>.inner_half_x/y,
NaN=未设置回落 half_x/half_y);RViz 可视框仍画外沿 half_x/half_y。

双臂接力扩展:TaskStatus.arm 区分执行臂(空串归一化为 arm1),状态
去重键为 (part_id, arm),CSV 行追加 arm 与 phase(arm2->relay,否则
sort);expect_relay=true 时本轮结束条件改为"每件都有 arm2 终态行或
任意 FAILED 行"。料区列表由 bin_names 参数化(双臂含 bin_out)。
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

from a1_interfaces.msg import PartInfo, TaskStatus
from a1_sorting_core.bins import (Bin, RESULT_FAILED, RESULT_MISPLACED,
                                  RESULT_NOT_FOUND, RESULT_OUTSIDE,
                                  RESULT_PLACED, ResultBook, judge_position)
from ros_gz_interfaces.msg import Entity, LogicalCameraImage
from ros_gz_interfaces.srv import DeleteEntity


class ResultRecorderNode(Node):

    def __init__(self):
        super().__init__('result_recorder')
        # ---- 料区几何(俯视 AABB,与 a1_cell_sim 的世界模型保持一致) ----
        # 料区列表参数化:双臂模式含 bin_out(同款 half_x/half_y 默认值)
        self.declare_parameter('bin_names', ['bin_A', 'bin_B'])
        bin_names = [str(n) for n in self.get_parameter('bin_names').value]
        self._bins = []
        for name in bin_names:
            prefix = f'bins.{name}.'
            self.declare_parameter(prefix + 'x', 0.0)
            self.declare_parameter(prefix + 'y', 0.0)
            self.declare_parameter(prefix + 'half_x', 0.12)
            self.declare_parameter(prefix + 'half_y', 0.10)
            # 内口半宽(带壁箱子 PLACED 判定,契约 §9):标量 double 默认
            # NaN 表示"未设置 -> 回落 half_x/half_y"(rclpy 允许 NaN
            # 默认值;数组默认才需要 [0.0]*6 这类非空占位)
            self.declare_parameter(prefix + 'inner_half_x', float('nan'))
            self.declare_parameter(prefix + 'inner_half_y', float('nan'))
            p = lambda suffix: float(self.get_parameter(prefix + suffix).value)
            ix, iy = p('inner_half_x'), p('inner_half_y')
            self._bins.append(Bin(
                name, p('x'), p('y'), p('half_x'), p('half_y'),
                inner_half_x=None if math.isnan(ix) else ix,
                inner_half_y=None if math.isnan(iy) else iy))

        self.declare_parameter('csv_path', '~/.ros/a1_sorting_results.csv')
        self.declare_parameter('logical_camera_topic', '/overhead_logical')
        self.declare_parameter('expected_parts', ['part_1', 'part_2', 'part_3'])
        self.declare_parameter('table_top_z', 0.72)
        self.declare_parameter('markers_frame', 'world')
        # 双臂接力:结束时要求每件有 arm2 终态行或任意 FAILED 行
        self.declare_parameter('expect_relay', False)
        # ---- 入箱件延时回收(持续演示防堆叠) ----
        # PLACED(且双臂模式下须是接力终态)后 recycle_delay 秒,经
        # /world/<world>/remove 删除该工件实体,腾出料区格位;0=关闭
        # (legacy 一轮 3 件用不到)。不回收 FAILED/OUTSIDE:失败件留在
        # 现场供复盘。删除后 a1_vision 侧靠 stale_timeout 自动剔除跟踪。
        self.declare_parameter('recycle_delay', 0.0)
        self.declare_parameter('world_name', 'sorting_cell')
        # 相机安装位姿(与 a1_vision/config/vision_params.yaml 一致)
        self.declare_parameter('camera.x', 0.65)
        self.declare_parameter('camera.y', 0.0)
        self.declare_parameter('camera.z', 1.55)
        self.declare_parameter('camera.pitch_deg', 90.0)

        gp = lambda k: self.get_parameter(k).value
        csv_path = str(gp('csv_path'))
        self._book = ResultBook(csv_path, [b.name for b in self._bins])
        self._expected = list(gp('expected_parts'))
        self._expect_relay = bool(gp('expect_relay'))
        self._finished: set[tuple[str, str]] = set()  # 去重键 (part_id, arm)
        self._failed_parts: set[str] = set()          # 有 FAILED 行的件
        self._table_z = float(gp('table_top_z'))
        self._frame = str(gp('markers_frame'))

        # 相机安装位姿(俯视:roll/yaw 为 0,pitch=90° 视线向下)
        pitch = np.radians(float(gp('camera.pitch_deg')))
        cp, sp = np.cos(pitch), np.sin(pitch)
        self._cam_rot = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
        self._cam_pos = np.array([
            float(gp('camera.x')), float(gp('camera.y')), float(gp('camera.z'))])
        self._poses: dict[str, tuple[float, float]] = {}
        # part_id -> category(视觉来料登记的真值,仅用于台账标注)
        self._categories: dict[str, str] = {}
        # 回收调度:part_id -> 删除到期时刻(仿真钟秒);由常驻扫描
        # 定时器消费(不销毁自身定时器,rclpy 回调内 destroy 语义不稳)
        self._recycle_due: dict[str, float] = {}
        self._recycle_delay = float(gp('recycle_delay'))
        self._del_client = None
        if self._recycle_delay > 0.0:
            self._del_client = self.create_client(
                DeleteEntity,
                f'/world/{str(gp("world_name"))}/remove')
            # 0.5s 常驻扫描;仅在有到期件时动作
            self.create_timer(0.5, self._tick_recycle)
            self.get_logger().info(
                f'入箱件回收已开启:PLACED 后 {self._recycle_delay}s 删除实体')

        self._pub_counts = self.create_publisher(Int32MultiArray, '/result/counts', 10)
        self._pub_markers = self.create_publisher(MarkerArray, '/result/markers', 10)
        self.create_subscription(
            LogicalCameraImage, str(gp('logical_camera_topic')),
            self._on_logical, 10)
        self.create_subscription(PartInfo, '/parts/arrival', self._on_arrival, 10)
        self.create_subscription(TaskStatus, '/task/status', self._on_status, 10)

        self.get_logger().info(
            f'结果记录器就绪,CSV: {csv_path},'
            f'料区: {[(b.name, round(b.center_x, 2), round(b.center_y, 2)) for b in self._bins]}')
        self._publish_bin_markers()

    # ------------------------------------------------------------- 逻辑相机
    def _on_logical(self, msg: LogicalCameraImage):
        """工件相对位姿(传感器系) -> 世界系俯视坐标。"""
        for model in msg.model:
            if not model.name.startswith('part_'):
                continue
            rel = np.array([model.pose.position.x,
                            model.pose.position.y,
                            model.pose.position.z])
            world = self._cam_pos + self._cam_rot @ rel
            self._poses[model.name] = (world[0], world[1])

    # ------------------------------------------------------------- 来料登记
    def _on_arrival(self, msg: PartInfo):
        """视觉来料登记:缓存 part_id→category 供台账标注(不做判定)。"""
        if msg.category:
            self._categories[msg.part_id] = msg.category

    # ------------------------------------------------------------- 状态判定
    def _on_status(self, msg: TaskStatus):
        if msg.state not in ('COMPLETED', 'FAILED'):
            return
        arm = msg.arm or 'arm1'          # 空串视为 arm1(兼容旧消息)
        key = (msg.part_id, arm)         # 去重键 (part_id, arm)
        if key in self._finished:
            return
        self._finished.add(key)
        phase = 'relay' if arm == 'arm2' else 'sort'

        # arm1 行:目标=detail(bin_A/bin_B);arm2 行:目标=detail(bin_out),
        # 工件若仍在原料区则 final_bin != target_bin 判 MISPLACED
        # (judge_position 语义不变,天然支持)
        target_bin = msg.detail if msg.state == 'COMPLETED' else None
        xy = self._poses.get(msg.part_id)
        if msg.state == 'FAILED':
            result, final_bin = RESULT_FAILED, None
        elif xy is None:
            result, final_bin = RESULT_NOT_FOUND, None
        else:
            final_bin = judge_position(xy[0], xy[1], self._bins)
            if final_bin is None:
                result = RESULT_OUTSIDE
            elif final_bin == target_bin:
                result = RESULT_PLACED
            else:
                result = RESULT_MISPLACED
        if result == RESULT_FAILED:
            self._failed_parts.add(msg.part_id)

        # 入箱成功 -> 排定回收(仅一次;同件重复终态被上面的 _finished
        # 去重挡住)。双臂接力件等 arm2 终态(PLACED)再回收,arm1 的
        # 中间 PLACED 不删(工件还要被接力)。
        if (result in (RESULT_PLACED, RESULT_MISPLACED)
                and self._recycle_delay > 0.0
                and self._recyclable_now(arm, result)):
            self._schedule_recycle(msg.part_id)

        self._book.record(msg.part_id, self._categories.get(msg.part_id, ''),
                          target_bin, final_bin, result,
                          arm=arm, phase=phase)
        if xy is not None:
            self.get_logger().info(
                f'{msg.part_id}[{arm}] -> {result} 类别='
                f"{self._categories.get(msg.part_id, '?')} "
                f'目标={target_bin} '
                f'实际={final_bin} 观测=({xy[0]:.3f}, {xy[1]:.3f})')
        else:
            self.get_logger().info(
                f'{msg.part_id}[{arm}] -> {result} (无观测位姿)')

        self._publish_counts()
        self._publish_bin_markers()
        if self._batch_finished():
            c = self._book.counts
            bins_txt = ' '.join(f'{b.name}={c.get(b.name, 0)}'
                                for b in self._bins)
            self.get_logger().info(
                f'== 本轮分拣结束: {bins_txt} '
                f'失败={c.get("FAILED", 0)} 放错={c.get("MISPLACED", 0)} ==')

    def _recyclable_now(self, arm: str, result: str) -> bool:
        """回收时机:接力模式下只认 arm2(最终)终态,arm1 的中间 PLACED
        不回收(工件还要被接力);单臂模式任意终态可回收。
        MISPLACED(放错箱)件同样回收腾格,计数已入台账,不影响判定。"""
        if result not in (RESULT_PLACED, RESULT_MISPLACED):
            return False
        if self._expect_relay and arm != 'arm2':
            return False
        return True

    def _schedule_recycle(self, part_id: str) -> None:
        """登记回收到期时刻(同件去重:已在表内则保留原到期)。"""
        if part_id in self._recycle_due:
            return
        self._recycle_due[part_id] = (
            self.get_clock().now().nanoseconds * 1e-9
            + self._recycle_delay)

    def _tick_recycle(self) -> None:
        """常驻扫描:处理到期的回收件(弹出即删,失败不重试)。"""
        if not self._recycle_due:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        for part_id in [p for p, t in self._recycle_due.items()
                        if t <= now]:
            del self._recycle_due[part_id]
            self._do_delete(part_id)

    def _do_delete(self, part_id: str) -> None:
        """经 remove 服务删除工件实体。"""
        client = self._del_client
        if client is None or not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'{part_id} 回收跳过:remove 服务不可用')
            return
        req = DeleteEntity.Request()
        req.entity.name = part_id
        req.entity.type = Entity.MODEL

        def done(future):
            resp = future.result()
            ok = bool(resp and resp.success)
            if ok:
                self.get_logger().info(f'{part_id} 回收完成(已从世界删除)')
            else:
                self.get_logger().warn(f'{part_id} 回收失败(gz 拒绝)')

        client.call_async(req).add_done_callback(done)

    def _batch_finished(self) -> bool:
        """本轮结束判定。

        expect_relay=false:每件都有任一终态行(与旧单臂逻辑等价);
        expect_relay=true:每件都有 arm2 终态行,或该件有 FAILED 行
        (arm1 失败时调度器跳过接力,不会再有 arm2 行)。
        """
        if not self._expected:
            return False
        if self._expect_relay:
            relay_done = {p for p, arm in self._finished if arm == 'arm2'}
            return all(p in relay_done or p in self._failed_parts
                       for p in self._expected)
        return set(self._expected) <= {p for p, _ in self._finished}

    # ------------------------------------------------------------- 输出
    def _publish_counts(self):
        c = self._book.counts
        msg = Int32MultiArray()
        # 布局 v2:按 bin_names 顺序逐箱计数 + [FAILED, MISPLACED] 收尾
        # (两箱配置下与旧口径 [bin_A,bin_B,FAILED,MISPLACED] 逐位一致,
        #  四箱配置下为 6 元;消费侧仅 RViz/演示,验收走 CSV 台账)
        msg.data = ([c.get(b.name, 0) for b in self._bins]
                    + [c.get('FAILED', 0), c.get('MISPLACED', 0)])
        self._pub_counts.publish(msg)

    def _publish_bin_markers(self):
        """料区可视框 + 计数标签,供 RViz 演示。"""
        z = self._table_z
        array = MarkerArray()
        for idx, b in enumerate(self._bins):
            box = Marker()
            box.header.frame_id = self._frame
            box.ns = 'bins'
            box.id = idx
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x = b.center_x
            box.pose.position.y = b.center_y
            box.pose.position.z = z + 0.003
            box.pose.orientation.w = 1.0
            box.scale.x = 2 * b.half_x
            box.scale.y = 2 * b.half_y
            box.scale.z = 0.005
            # 布局 v2:四箱标记色按箱名后缀配色(与 spawn_parts/箱底色同源);
            # 未知箱名回落旧暖冷口径(bin_A 橙红、其余绿),保持零回归
            _MARK_COLORS = {
                'bin_blue': (0.15, 0.35, 0.90, 0.35),
                'bin_yellow': (0.95, 0.80, 0.10, 0.35),
                'bin_red': (0.85, 0.10, 0.10, 0.35),
                'bin_green': (0.10, 0.75, 0.15, 0.35),
            }
            color = _MARK_COLORS.get(
                b.name, (0.92, 0.45, 0.12, 0.35) if b.name == 'bin_A'
                else (0.3, 0.85, 0.3, 0.35))
            box.color.r, box.color.g, box.color.b, box.color.a = color
            array.markers.append(box)

            text = Marker()
            text.header.frame_id = self._frame
            text.ns = 'bin_labels'
            text.id = idx
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = b.center_x
            text.pose.position.y = b.center_y
            text.pose.position.z = z + 0.08
            text.scale.z = 0.05
            text.color.r = text.color.g = text.color.b = 0.1
            text.color.a = 1.0
            count = self._book.counts.get(b.name, 0)
            text.text = f'{b.name}: {count}'
            array.markers.append(text)
        self._pub_markers.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = ResultRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
