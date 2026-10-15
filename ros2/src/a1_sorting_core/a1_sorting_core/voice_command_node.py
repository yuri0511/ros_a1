"""语言指挥节点:中文指令 -> 意图 JSON -> /sort_task -> 中文回执播报。

数据流(2026-10-06 语言指挥模式):
1. 接收指令:话题 /voice/command (std_msgs/String,现场演示可由
   `ros2 topic pub --once` 或 ASR 上游节点发布),或命令行交互模式
   (--text / stdin 逐行);
2. 解析:双层解析器 voice_intent.parse_intent —— 主层 LLM(OpenAI
   兼容 /chat/completions,env 配置,超时默认 3s),兜底关键词规则层
   (LLM 超时/异常/输出非法自动切换,规则层零网络依赖独立可跑);
3. 派发:意图 -> /sort_task (SortTask) 服务调用。来料色但无显式
   part_id 时,节点订阅 /parts/arrival 自建目录,按 FIFO 选未入箱件;
   dispatch_to 携带 target_bin(指定箱派发,抓取锚表路径不变);
4. 回执:服务响应 + 订阅 /task/status (TaskStatus) 该件终态,播报
   中文结果(发布 /voice/feedback,String;同时打日志),形成
   "说 -> 动 -> 回执"三段式演示闭环。

LLM 环境配置(env,均可缺省):
- VOICE_LLM_BASE_URL  OpenAI 兼容根(如 http://127.0.0.1:11434/v1),
  未设置 = 跳过 LLM 层直接用规则层(离线可演示);
- VOICE_LLM_MODEL     模型名(默认 gpt-4o-mini / 服务侧任意);
- VOICE_LLM_API_KEY   可选 Bearer key;
- VOICE_LLM_TIMEOUT   秒,默认 3.0;超时即降级(任务书硬要求)。

节点参数 voice_llm_base_url 等可覆盖 env(便于 launch 配置)。
"""
import json
import os
import threading
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from a1_interfaces.msg import PartInfo, TaskStatus
from a1_interfaces.srv import SortTask
from a1_sorting_core.voice_intent import extract_json, llm_prompt, parse_intent

# 类别/箱名 -> 中文播报词
CN = {'red': '红色', 'green': '绿色', 'blue': '蓝色', 'yellow': '黄色',
      'bin_red': '红箱', 'bin_green': '绿箱',
      'bin_blue': '蓝箱', 'bin_yellow': '黄箱'}

_BIN_OF_CATEGORY = {c: f'bin_{c}' for c in ('red', 'green', 'blue',
                                            'yellow')}


def _default_llm_config():
    return {
        'base_url': os.environ.get('VOICE_LLM_BASE_URL', '').rstrip('/'),
        'model': os.environ.get('VOICE_LLM_MODEL', 'gpt-4o-mini'),
        'api_key': os.environ.get('VOICE_LLM_API_KEY', ''),
        'timeout': float(os.environ.get('VOICE_LLM_TIMEOUT', '3.0')),
    }


def make_llm_fn(cfg):
    """构造 llm_fn(text)->dict|None;未配置 base_url 返回 None(纯规则)。"""
    base = cfg['base_url']
    if not base:
        return None

    def llm_fn(text):
        payload = json.dumps({
            'model': cfg['model'],
            'messages': llm_prompt(text),
            'temperature': 0.0,
            'response_format': {'type': 'json_object'},
        }).encode('utf-8')
        req = urllib.request.Request(
            base + '/chat/completions', data=payload,
            headers={'Content-Type': 'application/json'})
        if cfg['api_key']:
            req.add_header('Authorization',
                           'Bearer ' + cfg['api_key'])
        with urllib.request.urlopen(req, timeout=cfg['timeout']) as resp:
            body = json.loads(resp.read().decode('utf-8'))
        content = body['choices'][0]['message']['content']
        return extract_json(content)
    return llm_fn


class VoiceCommandNode(Node):

    def __init__(self):
        super().__init__('voice_command')
        self.declare_parameter('command_topic', '/voice/command')
        self.declare_parameter('feedback_topic', '/voice/feedback')
        self.declare_parameter('sort_task_service', '/sort_task')
        # LLM 配置:参数优先,回落 env;base_url 空 = 纯规则层
        self.declare_parameter('voice_llm_base_url', '')
        self.declare_parameter('voice_llm_model', '')
        self.declare_parameter('voice_llm_timeout', 0.0)
        cfg = _default_llm_config()
        p = lambda k: str(self.get_parameter(k).value).strip()
        cfg['base_url'] = (p('voice_llm_base_url').rstrip('/')
                           or cfg['base_url'])
        cfg['model'] = p('voice_llm_model') or cfg['model']
        t = float(self.get_parameter('voice_llm_timeout').value)
        cfg['timeout'] = t if t > 0 else cfg['timeout']
        self._llm_fn = make_llm_fn(cfg)
        self.get_logger().info(
            '语言指挥节点就绪:解析层 = '
            + (f'LLM({cfg["model"]}@{cfg["base_url"]},超时 '
               f'{cfg["timeout"]}s 降级规则层)' if self._llm_fn
               else '规则层(LLM 未配置 VOICE_LLM_BASE_URL)'))

        self._pub_feedback = self.create_publisher(
            String, str(self.get_parameter('feedback_topic').value), 10)
        self.create_subscription(
            String, str(self.get_parameter('command_topic').value),
            self._on_command, 10)
        self._client = self.create_client(
            SortTask, str(self.get_parameter('sort_task_service').value))
        # 自建来料目录(与调度器独立;FIFO 选件用)
        self._catalog: dict[str, PartInfo] = {}
        self._order: list[str] = []
        self.create_subscription(PartInfo, '/parts/arrival',
                                 self._on_arrival, 10)
        # 在途件回执:part_id -> 播报上下文 dict
        self._pending: dict[str, dict] = {}
        self._lock = threading.Lock()
        self.create_subscription(TaskStatus, '/task/status',
                                 self._on_status, 10)

    # ------------------------------------------------------------ 输入
    def _on_arrival(self, msg: PartInfo):
        with self._lock:
            if msg.part_id not in self._catalog:
                self._order.append(msg.part_id)
            self._catalog[msg.part_id] = msg

    def _on_command(self, msg: String):
        self.handle_text(msg.data)

    def handle_text(self, text: str) -> dict:
        """同步处理一条中文指令(话题回调与交互模式共用);返回意图。"""
        errors: list[str] = []
        intent = parse_intent(text, llm_fn=self._llm_fn,
                              llm_error=errors)
        if errors:
            self.get_logger().warn(
                f'LLM 层失败已降级规则层: {errors[0]}')
        self.get_logger().info(
            f'指令「{intent["raw"]}」-> 意图 {intent} '
            f'(parser={intent["parser"]})')
        self._dispatch_intent(intent)
        return intent

    # ------------------------------------------------------------ 派发
    def _pick_part(self, intent: dict) -> str:
        """按意图选 part_id:显式 > 颜色 FIFO(未入箱/未在途) > 目录首件。"""
        with self._lock:
            if intent['part_id'] and intent['part_id'] in self._catalog:
                return intent['part_id']
            for pid in self._order:
                if pid in self._pending:
                    continue
                if intent['part_color'] and \
                        self._catalog[pid].category != intent['part_color']:
                    continue
                return pid
        return intent.get('part_id', '')

    def _dispatch_intent(self, intent: dict):
        action = intent['action']
        if action == 'unknown':
            self._say(f'没听懂指令「{intent["raw"]}」,可以说:'
                      '把红色的放到蓝色箱子、开始分拣')
            return
        req = SortTask.Request()
        if action == 'start_batch':
            req.command = 'start_batch'
        else:
            part_id = self._pick_part(intent)
            if not part_id:
                self._say('当前没有符合条件的在库工件,请先上料'
                          + (f'(等待{CN.get(intent["part_color"], "")}块)'
                             if intent['part_color'] else ''))
                return
            req.part_id = part_id
            if action == 'dispatch_to':
                req.command = 'dispatch_to'
                req.target_bin = intent['target_bin']
            else:
                req.command = 'dispatch_single'
            ctx = {'part_id': part_id, 'intent': intent}
            with self._lock:
                self._pending[part_id] = ctx
        if not self._client.wait_for_service(timeout_sec=2.0):
            self._say('调度器未在线(/sort_task 服务不可用)')
            return

        def done(fut, req=req):
            resp = fut.result()
            if not resp.accepted:
                with self._lock:
                    self._pending.pop(req.part_id, None)
                self._say(f'派发被拒绝: {resp.message}')
                return
            head = f'收到,正在把'
            if req.command == 'start_batch':
                self._say(f'收到,{resp.message}')
                return
            tb = req.target_bin if req.command == 'dispatch_to' else ''
            cat = ''
            with self._lock:
                info = self._catalog.get(req.part_id)
                ctx = self._pending.get(req.part_id)
            if info is not None:
                cat = CN.get(info.category, info.category)
            if ctx is not None:
                ctx['acked'] = resp.message
            tgt = f'{cat}块放到{CN.get(tb, tb)}' if tb else \
                f'{cat}块按默认料区分拣' if cat else '工件分拣'
            self._say(f'{head}{tgt}')

        self._client.call_async(req).add_done_callback(done)

    # ------------------------------------------------------------ 回执
    def _on_status(self, msg: TaskStatus):
        if msg.state not in ('COMPLETED', 'FAILED'):
            return
        with self._lock:
            ctx = self._pending.pop(msg.part_id, None)
        if ctx is None:
            return   # 非本节点派发的件(自动批次)不打扰语音回执
        intent = ctx['intent']
        tb = intent.get('target_bin', '')
        if msg.state == 'COMPLETED':
            bin_name = msg.detail or tb
            self._say(f'{msg.part_id} 已放进{CN.get(bin_name, bin_name)},'
                      '任务完成')
        else:
            self._say(f'{msg.part_id} 执行失败({msg.detail})')

    def _say(self, text: str):
        self.get_logger().info(f'播报: {text}')
        msg = String()
        msg.data = text
        self._pub_feedback.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
