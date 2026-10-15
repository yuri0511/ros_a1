#!/usr/bin/env python3
"""工件生成节点(双模式,经桥接的 /world/<world>/create 服务生成工件)。

mode=batch(默认,legacy 回归基线):参数 part_colors 逗号分隔决定颜色序列
(如 "red,green,red"),逐件生成到传送带上静置槽 (0.30,0)/(0.36,0)/(0.42,0),
槽位按件序对 3 取模循环(允许给出多于 3 件颜色);生成高度 FEED_Z=0.76
(契约 §3 起调值,落带不弹为准;带面静置中心 0.805);全部生成完成后节点退出。

mode=service(pipeline 按需来料):常驻节点,订阅 /parts/request_spawn
(std_msgs/String,data ∈ {random, red, green, blue, yellow});每收到一条
请求生成下一件 part_<n>(n 从 1 自增)到带尾 (0.18, 0, 0.805)(带顶 0.78
上的静置中心);data 为 random 时从 spawn_parts.RANDOM_COLORS 等概率选取
(red/green/blue/yellow 四色;2026-10-03 加 yellow 并开放 blue);空 data
忽略。订阅用 transient_local QoS,
发布方(dispatcher)启动即发首件请求,本节点迟到启动仍能收到最近一条。
不发布回执,工件出现在逻辑相机视野即事实回执。

工件实体名 part_<n>,尺寸/质量(0.05 立方、0.2 kg)与颜色 RGB 表与契约 §3
一致;坐标与 a1_cell_sim/worlds/sorting_cell.sdf 的带/箱几何对齐。
"""
import random
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.srv import SpawnEntity
from std_msgs.msg import String

# batch 模式:带上静置槽(世界系 xy;带面 v2 已拉长覆盖 x 0.15..0.75),
# 件数超 3 时按 i%3 循环取槽。
# 布局 v2(2026-10-05 Lead 裁决,风险点3=方案 a 变体):基座已搬
# (0.60,±0.47),旧槽 (0.30/0.36/0.42) 对 arm1 的 r 变成 0.503~0.558、
# 未被 G0 预检覆盖;新槽 (0.50/0.58/0.66) 的 r=0.4805/0.4704/0.4738,
# 全部落在 G0 带尾已验域(r=0.470,τ载 18.1~19.6)±1.1cm 内;相邻间距
# 8cm(记忆纪律:格间距 ≥8cm 防互挤);最远槽件前缘 0.685 距挡停板
# 内面 0.705 留 20mm(记忆定罪:"恰共面接触积累抖动",不可贴板)。
FEED_SLOTS = [(0.50, 0.0), (0.58, 0.0), (0.66, 0.0)]
# batch(legacy)模式生成高度:带面静置中心 0.805(带顶 0.78+半边)。
# 2026-10-03 从 0.76 改:旧值落在桌面 0.72 上(rest 0.745),取件目标
# tool0 0.956 在半径 0.36 处深于可达极限 ~1.002,pick_settle 必超时;
# 带面高度下取件目标 1.016 可达(pipeline 同半径 1.013 实证)
FEED_Z = 0.805
# service 模式:带尾生成位 = 带顶 0.78 + 半边 0.025(即带面静置中心,直接落位)
SERVICE_SPAWN_X = 0.18
SERVICE_SPAWN_Y = 0.0
SERVICE_SPAWN_Z = 0.805
SPAWN_REQUEST_TOPIC = '/parts/request_spawn'
# transient_local + reliable:dispatcher 启动即发首件请求,本节点可能迟到
# (TimerAction 延迟启动),volatile 会丢请求导致死锁,故须与发布方同为
# transient_local,迟到订阅仍能收到最近一条
SPAWN_REQUEST_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

PART_SIZE = 0.05
PART_MASS = 0.2

COLOR_RGB = {
    'red': (0.85, 0.10, 0.10),
    'green': (0.10, 0.75, 0.15),
    'blue': (0.15, 0.35, 0.90),
    # yellow:OpenCV HSV H≈24.6(换算实测),与 red(0-10/170-179)、
    # green(35-85)双向留隙;俯视相机带面光照下四类可分
    # (a1_vision DEFAULT_RANGES 的 yellow 区间与此同步)
    'yellow': (0.95, 0.80, 0.10),
}

# service 模式 random 的默认色池(2026-10-03:演示要求持续多色,四色
# 等概率;参数 random_colors 可覆盖,如压测只投 red/green)
RANDOM_COLORS = ['red', 'green', 'blue', 'yellow']


def part_sdf(index: int, color: str) -> str:
    r, g, b = COLOR_RGB[color]
    half = PART_SIZE / 2
    inertia = PART_MASS / 12 * (PART_SIZE ** 2 + PART_SIZE ** 2)
    # 注意:生成位置由 EntityFactory.pose 字段给出(gz-sim 会忽略
    # EntityFactory 提交的 SDF 内嵌 <pose>),这里只定义形状与外观。
    return f"""<?xml version="1.0" ?>
<sdf version="1.11">
  <model name="part_{index}">
    <link name="link">
      <inertial>
        <mass>{PART_MASS}</mass>
        <inertia><ixx>{inertia}</ixx><iyy>{inertia}</iyy><izz>{inertia}</izz></inertia>
      </inertial>
      <visual name="visual">
        <geometry><box><size>{PART_SIZE} {PART_SIZE} {PART_SIZE}</size></box></geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
          <specular>0.15 0.15 0.15 1</specular>
        </material>
      </visual>
      <collision name="collision">
        <geometry><box><size>{PART_SIZE} {PART_SIZE} {PART_SIZE}</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>"""


def make_spawn_request(index: int, color: str,
                       x: float, y: float, z: float) -> SpawnEntity.Request:
    request = SpawnEntity.Request()
    request.entity_factory.name = f'part_{index}'
    request.entity_factory.allow_renaming = False
    request.entity_factory.sdf = part_sdf(index, color)
    request.entity_factory.pose.position.x = x
    request.entity_factory.pose.position.y = y
    request.entity_factory.pose.position.z = z
    return request


def run_batch_mode(node: Node, client, colors) -> None:
    """legacy batch 行为:逐件生成到带上静置槽,完成后返回(节点随即退出)。"""
    if not client.wait_for_service(timeout_sec=20.0):
        node.get_logger().error('create 服务不可用,放弃生成工件')
        return
    for i, color in enumerate(colors, start=1):
        slot = FEED_SLOTS[(i - 1) % len(FEED_SLOTS)]
        request = make_spawn_request(i, color, slot[0], slot[1], FEED_Z)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        ok = future.result() is not None and future.result().success
        node.get_logger().info(
            f'生成工件 part_{i} ({color}) @({slot[0]}, {slot[1]}, {FEED_Z}): '
            f'{"成功" if ok else "失败"}')
        time.sleep(0.3)


def run_service_mode(node: Node, client, random_colors) -> None:
    """常驻模式:订阅 /parts/request_spawn,按请求逐件生成到带尾,不主动退出。"""
    state = {'count': 0}

    def log_result(future, name: str, color: str) -> None:
        response = future.result()
        ok = response is not None and response.success
        node.get_logger().info(f'生成工件 {name} ({color}): {"成功" if ok else "失败"}')

    def on_request(msg: String) -> None:
        data = msg.data.strip()
        if not data:
            node.get_logger().warn('忽略空 spawn 请求(data 为空)')
            return
        if data == 'random':
            color = random.choice(random_colors)
        elif data in COLOR_RGB:
            color = data
        else:
            node.get_logger().warn(f'忽略无法识别的 spawn 请求 data="{msg.data}"')
            return
        state['count'] += 1
        index = state['count']
        name = f'part_{index}'
        node.get_logger().info(
            f'收到请求 "{data}":生成 {name} ({color}) '
            f'@({SERVICE_SPAWN_X}, {SERVICE_SPAWN_Y}, {SERVICE_SPAWN_Z})')
        request = make_spawn_request(index, color,
                                     SERVICE_SPAWN_X, SERVICE_SPAWN_Y, SERVICE_SPAWN_Z)
        # 回调内不能用 spin_until_future_complete(单线程执行器会死锁),
        # 用 done 回调异步打结果日志
        future = client.call_async(request)
        future.add_done_callback(lambda f, n=name, c=color: log_result(f, n, c))

    node.create_subscription(String, SPAWN_REQUEST_TOPIC, on_request, SPAWN_REQUEST_QOS)
    node.get_logger().info(
        f'service 模式启动,订阅 {SPAWN_REQUEST_TOPIC}(transient_local),等待来料请求')
    # 先订阅再等服务:create 服务未就绪期间到达的请求由 DDS 队列/transient_local 保留
    while rclpy.ok() and not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().warn('create 服务暂不可用,每 5s 重试等待中')
    if not rclpy.ok():
        return
    rclpy.spin(node)


def main():
    rclpy.init()
    node = rclpy.create_node('spawn_parts')
    node.declare_parameter('mode', 'batch')
    node.declare_parameter('part_colors', 'red,green,red')
    node.declare_parameter('world_name', 'sorting_cell')
    node.declare_parameter('random_colors',
                           '')  # 空=内置 RANDOM_COLORS 四色池;可覆盖收窄
    mode = str(node.get_parameter('mode').value).strip().lower()
    colors = [c.strip() for c in
              str(node.get_parameter('part_colors').value).split(',') if c.strip()]
    random_colors = [c.strip() for c in
                     str(node.get_parameter('random_colors').value).split(',')
                     if c.strip() in COLOR_RGB] or RANDOM_COLORS
    world = str(node.get_parameter('world_name').value)

    client = node.create_client(SpawnEntity, f'/world/{world}/create')

    try:
        if mode == 'service':
            run_service_mode(node, client, random_colors)
        else:
            if mode != 'batch':
                node.get_logger().warn(f'未知 mode "{mode}",回退 batch 行为')
            run_batch_mode(node, client, colors)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
