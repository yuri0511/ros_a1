"""/sort_task 服务契约的纯逻辑(与 ROS 解耦,便于单元测试)。

背景(2026-10-06 语言指挥模式):演示/语音指令需要"把 X 色的放到 Y 箱"
这种**指定放置目标箱**的派发。抓取段一律不变(仍走既有锚表路径),
只有放置目标箱被命令覆盖,因此本模块只负责"命令 -> 决策"这段:

- 命令合法性(已知/未知 command、part_id 是否存在);
- dispatch_to 的目标箱校验(非法箱名 -> 拒绝并给出回执文案);
- 生效臂建议(箱 -> 臂 映射表,并联分色模式下覆盖按色路由);
- dispatch_single 的残留覆盖清除(否则上一轮的 target_bin 会串到本轮)。

节点(task_dispatcher_node)只做 ROS 桥接:把 SortTask.Request 的字段喂给
plan_sort_task(),按返回的 DispatchDecision 落地队列/覆盖表/派发。
箱臂映射由参数 bin_arm_map(字符串 'bin_blue:arm1,bin_red:arm2,...')
提供;空表 = 调度器不校验箱名(交由执行器按本臂 bin_names 校验),
路由退回既有按色路由,行为与扩展前完全一致。
"""
from dataclasses import dataclass, field

COMMAND_START_BATCH = 'start_batch'
COMMAND_DISPATCH_SINGLE = 'dispatch_single'
COMMAND_DISPATCH_TO = 'dispatch_to'

KNOWN_COMMANDS = (COMMAND_START_BATCH, COMMAND_DISPATCH_SINGLE,
                  COMMAND_DISPATCH_TO)

ARM1 = 'arm1'
ARM2 = 'arm2'


class UnknownTargetBin(Exception):
    """放置目标箱无法解析(未知类别或非法指定箱)。"""


def parse_bin_arm_map(spec: str) -> dict[str, str]:
    """'bin_blue:arm1,bin_red:arm2' -> {'bin_blue':'arm1','bin_red':'arm2'}。

    空串/空白项忽略;非法项(缺冒号、臂名不是 arm1/arm2)直接丢弃,
    保证"参数写错 = 退化为不校验",而不是让节点起不来。
    """
    out: dict[str, str] = {}
    for item in str(spec or '').split(','):
        item = item.strip()
        if not item or ':' not in item:
            continue
        bin_name, _, arm = item.partition(':')
        bin_name = bin_name.strip()
        arm = arm.strip()
        if bin_name and arm in (ARM1, ARM2):
            out[bin_name] = arm
    return out


@dataclass
class DispatchDecision:
    """一次 /sort_task 请求的决策结果(节点据此落地副作用)。"""

    accepted: bool
    message: str
    command: str = ''
    part_id: str = ''
    target_bin: str = ''          # 生效的放置目标箱覆盖;'' = 无覆盖
    arm: str = ARM1               # 建议执行臂(仅并联派发时用到)
    clear_override: bool = False  # True:清除该件残留的 target_bin 覆盖
    enqueue: bool = False         # True:把 part_id 入队
    enqueue_all: bool = False     # start_batch:全部来料入队
    extra: dict = field(default_factory=dict)


def plan_sort_task(command: str, part_id: str, target_bin: str,
                   known_parts, queue_size: int = 0,
                   bin_arm_map: dict[str, str] | None = None,
                   default_arm: str = ARM1,
                   busy_parts=None) -> DispatchDecision:
    """把一条 SortTask.Request 解析成决策(纯函数,无副作用)。

    参数:
    - command/part_id/target_bin:请求原文(本函数负责 strip);
    - known_parts:调度器目录里已登记的 part_id 集合(用于存在性校验);
    - queue_size:当前队列长度,仅用于回执文案;
    - bin_arm_map:箱 -> 臂 映射;非空时启用调度器侧箱名校验与按箱路由,
      空/None 时不校验箱名(dispatch_to 仍要求 target_bin 非空),
      路由沿用 default_arm(调用方给的既有按色路由结果);
    - busy_parts:该件已"在队列中或执行中"的 part_id 集合。auto_start
      模式下来料会被自动入队,语音/外部指令若对同一件再次派发,会在
      该件终态后被重复执行一遍(2026-10-06 E2E 实证撞车),故非空集合
      内的件直接拒绝并在 message 说明。
    """
    cmd = str(command or '').strip()
    pid = str(part_id or '').strip()
    tb = str(target_bin or '').strip()
    bin_map = bin_arm_map or {}
    busy = set(busy_parts or ())

    if cmd == COMMAND_START_BATCH:
        return DispatchDecision(
            accepted=True, command=cmd, enqueue_all=True,
            message='已入队全部已到料工件,开始分拣')

    if cmd in (COMMAND_DISPATCH_SINGLE, COMMAND_DISPATCH_TO):
        if pid not in set(known_parts):
            return DispatchDecision(
                accepted=False, command=cmd, part_id=pid,
                message=f'未知工件: {pid}' if pid else '未知工件:(part_id 为空)')
        if pid in busy:
            return DispatchDecision(
                accepted=False, command=cmd, part_id=pid, target_bin=tb,
                message=f'{pid} 已在队列或正在执行,不重复派发')
        if cmd == COMMAND_DISPATCH_SINGLE:
            return DispatchDecision(
                accepted=True, command=cmd, part_id=pid, enqueue=True,
                clear_override=True, arm=default_arm,
                message=f'{pid} 已入队(按类别默认料区)')
        # dispatch_to:必须给箱名
        if not tb:
            return DispatchDecision(
                accepted=False, command=cmd, part_id=pid,
                message='dispatch_to 缺少 target_bin(目标箱名)')
        if bin_map and tb not in bin_map:
            legal = '、'.join(sorted(bin_map))
            return DispatchDecision(
                accepted=False, command=cmd, part_id=pid,
                message=f'非法目标箱: {tb}(合法箱:{legal})')
        arm = bin_map.get(tb, default_arm)
        return DispatchDecision(
            accepted=True, command=cmd, part_id=pid, target_bin=tb,
            enqueue=True, arm=arm,
            message=f'{pid} 已入队,指定放置 {tb}'
                    + (f'(执行臂 {arm})' if tb in bin_map else ''))

    return DispatchDecision(accepted=False, command=cmd, part_id=pid,
                            message=f'未知命令: {cmd}')


def resolve_target_bin(category: str, override: str,
                       color_to_bin: dict[str, str],
                       valid_bins) -> str:
    """执行器侧放置目标箱决策(纯函数;抓取段与此无关)。

    - override 为空串 -> 返回既有 类别->料区 映射结果;未知类别抛
      UnknownTargetBin(由调用方转 FatalGraspError('unknown_category...')),
      行为与扩展前完全一致;
    - override 非空 -> **仅改放置目标箱**:必须在 valid_bins(本臂
      bin_names)内,否则抛 UnknownTargetBin('dispatch_target_bin_invalid(...)');
      非法放置不拒收任务而是让该件走既有 FAILED 通道,台账可考。
    """
    ovr = str(override or '').strip()
    if ovr:
        if ovr not in set(valid_bins):
            raise UnknownTargetBin(
                f'dispatch_target_bin_invalid({ovr})')
        return ovr
    target = color_to_bin.get(str(category or ''), '')
    if not target or target not in set(valid_bins):
        raise UnknownTargetBin(f'unknown_category({category})')
    return target
