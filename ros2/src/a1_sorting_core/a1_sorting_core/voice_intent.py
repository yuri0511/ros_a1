"""中文指令 -> 分拣意图 JSON 的纯逻辑(与 ROS 解耦,可离线单测)。

双层解析(2026-10-06 语言指挥模式):
- 主层 LLM:OpenAI 兼容 /chat/completions(env 配置,见
  voice_command_node),提示词内置四色/四箱/两臂术语表,要求严格
  JSON 输出;超时(默认 3s)/异常/输出不合法 -> 自动降级;
- 兜底规则层:关键词/语序规则,**必须独立可跑**(本模块零网络依赖,
  parse_intent_rule 是纯字符串函数)。

意图 JSON(dict)固定字段:
  action      : 'dispatch_to' | 'dispatch_single' | 'start_batch'
                | 'unknown'
  part_color  : 'red'|'green'|'blue'|'yellow'|''   (来料颜色约束)
  part_id     : ''(空 = 由节点按 FIFO/目录选件)
  target_bin  : 'bin_red'|'bin_green'|'bin_blue'|'bin_yellow'|''
  parser      : 'rule' | 'llm'(实际生效层,便于回执与调试)
  raw         : 原始指令文本

术语表(布局 v2 定档,一箱一色):
  红色 red -> bin_red(南排,arm2)   绿色 green -> bin_green(南排,arm2)
  蓝色 blue -> bin_blue(北排,arm1)  黄色 yellow -> bin_yellow(北排,arm1)

规则层解析口径:
- "把红色的放到蓝色箱子":两色词,首=来料色,末(邻箱/有到链路)=目标箱;
  "红→蓝"/"红到蓝"/"红色放蓝箱" 同理;
- 单色词紧邻箱/盒标记且前面有放置链路 -> 仅指定目标箱(part_color 空
  = 任意来料);否则该色词=来料色;
- "开始(批量)分拣" -> start_batch;
- "把 part_2 放到 bin_red" -> 显式 part_id;
- 无法识别 -> action='unknown'(节点回执"没听懂")。
"""
import json
import re

COLORS = ('red', 'green', 'blue', 'yellow')

# 中文颜色词 -> 类别(扫描时按词长降序取最长匹配)
COLOR_WORDS: dict[str, str] = {
    '红色': 'red', '红': 'red', '红块': 'red', '红方块': 'red',
    '绿色': 'green', '绿': 'green', '绿块': 'green', '绿方块': 'green',
    '蓝色': 'blue', '蓝': 'blue', '蓝块': 'blue', '蓝方块': 'blue',
    '黄色': 'yellow', '黄': 'yellow', '黄块': 'yellow', '黄方块': 'yellow',
}
# 类别 -> 规范箱名(布局 v2:一箱一色)
CATEGORY_TO_BIN = {c: f'bin_{c}' for c in COLORS}
BINS = tuple(CATEGORY_TO_BIN.values())

# 目标箱动作语境标记(色词后 3 字内出现 => 该词指"箱")
_BIN_MARKERS = ('箱', '盒子', '框', '桶')
_PLACE_VERBS = ('放到', '放入', '投到', '投进', '扔进', '扔到', '丢进',
                '丢到', '放进', '投入', '送到', '归到')
_ARROWS = ('->', '=>', '→', '➡', '>>', '到', '至')
_ARROW_RE = re.compile(r'(?:->|=>|→|➡|>>)')
_BATCH_WORDS = ('开始分拣', '开始批量', '启动分拣', '批量分拣',
                '全部开始', '开始吧', '开始', '启动')
_PART_ID_RE = re.compile(r'part[_\-\s]?(\d+)', re.IGNORECASE)
_BIN_NAME_RE = re.compile(r'bin[_\-]?(red|green|blue|yellow)', re.IGNORECASE)
# 站外色名(本工作站只有四色四箱):"紫色箱子"等 = 非法箱指称,规则层
# 必须**保守拒绝**(unknown -> 节点回执"没听懂"),绝不能降级成默认派发
_OTHER_COLOR_WORDS = ('紫', '橙', '黑', '白', '粉', '灰', '金', '银', '棕',
                      '褐', '青')


def _iter_color_mentions(text: str) -> list[tuple[int, int, str]]:
    """按出现顺序扫描颜色词:返回 (起点, 终点, 类别),重叠取最长。"""
    spans: list[tuple[int, int, str]] = []
    words = sorted(COLOR_WORDS, key=len, reverse=True)
    i = 0
    while i < len(text):
        for w in words:
            if text.startswith(w, i):
                spans.append((i, i + len(w), COLOR_WORDS[w]))
                i += len(w)
                break
        else:
            i += 1
    return spans


def _near_bin(text: str, end: int) -> bool:
    """颜色词结束后 3 字符内出现箱/盒类标记 => 该词指"箱"而非"块"。"""
    return any(m in text[end:end + 3] for m in _BIN_MARKERS)


def parse_intent_rule(text: str) -> dict:
    """兜底关键词规则解析(纯函数,零依赖,LLM 不可用时的独立层)。"""
    raw = str(text or '').strip()
    result = {'action': 'unknown', 'part_color': '', 'part_id': '',
              'target_bin': '', 'parser': 'rule', 'raw': raw}
    if not raw:
        return result

    m = _PART_ID_RE.search(raw)          # 显式 part_id(part_2/PART 2)
    if m:
        result['part_id'] = f'part_{m.group(1)}'
    bin_name_m = _BIN_NAME_RE.search(raw)  # bin 名直写(bin_red)
    mentions = _iter_color_mentions(raw)

    # 站外色名守卫:出现"紫/橙/黑…(+色)"或紧邻箱标记的站外色 => 整句
    # 保守拒绝(unknown),避免把"放到紫色箱子"误降级成默认派发
    for w in _OTHER_COLOR_WORDS:
        if w + '色' in raw:
            return result
        if any(m2 in raw[w_i + 1:w_i + 3]
               for m2 in _BIN_MARKERS
               for w_i in [i for i in range(len(raw)) if raw[i] == w]):
            return result

    # 批量指令:不含颜色/箱名/part_id 时才判"开始"(含色时"开始"是噪声)
    if not mentions and not bin_name_m and not m:
        if any(w in raw for w in _BATCH_WORDS):
            result['action'] = 'start_batch'
            return result

    target_bin = CATEGORY_TO_BIN[bin_name_m.group(1).lower()] \
        if bin_name_m else ''
    part_color = ''
    has_arrow = bool(_ARROW_RE.search(raw)) or any(
        a in raw for a in ('到', '至'))
    has_place_verb = any(v in raw for v in _PLACE_VERBS)

    if len(mentions) >= 2:
        # 双色词:首=来料色;末色邻箱 或 有"到/→/放"链路 => 目标箱,
        # 否则("红色的和蓝色的都有"式并列)不产出 target
        part_color = mentions[0][2]
        if (_near_bin(raw, mentions[-1][1]) or has_arrow
                or has_place_verb):
            target_bin = CATEGORY_TO_BIN[mentions[-1][2]]
    elif len(mentions) == 1 and not bin_name_m:
        pos, end, c = mentions[0]
        near = _near_bin(raw, end)
        prefix = raw[:pos]
        linked = (any(v in prefix for v in _PLACE_VERBS) or has_arrow
                  or prefix.rstrip().endswith(('把', '将', '进', '到', '至')))
        if near and linked:
            # "放到蓝色箱子":仅目标箱,来料色留空(FIFO 选件)
            target_bin = CATEGORY_TO_BIN[c]
        else:
            # "把红色的拿走"/"红色的放到蓝箱"(首色不邻箱):来料色
            part_color = c

    if bin_name_m and mentions and not part_color:
        # bin 名直写 + 中文色词并存("红色的放 bin_blue"):首个与
        # 直写箱不同色的色词当来料色
        direct_c = bin_name_m.group(1).lower()
        for _pos, _end, c in mentions:
            if c != direct_c:
                part_color = c
                break

    if result['part_id'] or part_color or target_bin:
        result['action'] = ('dispatch_to' if target_bin
                            else 'dispatch_single')
    result['part_color'] = part_color
    result['target_bin'] = target_bin
    return result


def validate_intent(obj, raw: str = '') -> dict | None:
    """校验 LLM 输出的 JSON(严格白名单),非法返回 None(触发降级)。"""
    if not isinstance(obj, dict):
        return None
    action = str(obj.get('action', '')).strip()
    if action not in ('dispatch_to', 'dispatch_single', 'start_batch',
                      'unknown'):
        return None
    part_color = str(obj.get('part_color', '') or '').strip().lower()
    if part_color and part_color not in COLORS:
        return None
    target_bin = str(obj.get('target_bin', '') or '').strip().lower()
    if target_bin and target_bin not in BINS:
        return None
    part_id = str(obj.get('part_id', '') or '').strip()
    if part_id and not re.fullmatch(r'part_\d+', part_id):
        return None
    if action == 'dispatch_to' and not target_bin:
        return None
    return {'action': action, 'part_color': part_color,
            'part_id': part_id, 'target_bin': target_bin,
            'parser': 'llm', 'raw': raw}


def extract_json(text: str):
    """从 LLM 回复里抠出第一个 JSON 对象(容忍 ```json 围栏/前后废话)。"""
    t = str(text or '').strip()
    if t.startswith('```'):
        t = re.sub(r'^```(?:json)?\s*|\s*```$', '', t, flags=re.S)
    start = t.find('{')
    if start < 0:
        return None
    depth = 0
    for i in range(start, len(t)):
        if t[i] == '{':
            depth += 1
        elif t[i] == '}':
            depth -= 1
            if depth == 0:
                try:
                    return json.loads(t[start:i + 1])
                except json.JSONDecodeError:
                    return None
    return None


# 内置术语表提示词(四色/四箱/两臂;要求严格 JSON)
SYSTEM_PROMPT = """你是 A1 分拣工作站的中文指令解析器。把用户指令解析为
**一个严格 JSON 对象**,不输出任何其它文字。字段:
{"action": "dispatch_to|dispatch_single|start_batch|unknown",
 "part_color": "red|green|blue|yellow|空串",
 "part_id": "part_数字或空串",
 "target_bin": "bin_red|bin_green|bin_blue|bin_yellow|空串"}
术语表(布局 v2,一箱一色):
- 颜色:红=red,绿=green,蓝=blue,黄=yellow
- 料箱:红箱=bin_red,绿箱=bin_green,蓝箱=bin_blue,黄箱=bin_yellow
- 执行臂:arm1 管北排 bin_blue/bin_yellow,arm2 管南排 bin_red/bin_green
规则:
1. "把X色的放到Y箱"/"X→Y" => action=dispatch_to, part_color=X, target_bin=Y;
2. 只说目标箱不说来料色 => part_color 留空;
3. "开始分拣/批量开始" => action=start_batch,其余字段留空;
4. 与分拣无关或无法解析 => action=unknown。"""


def llm_prompt(raw: str) -> list[dict]:
    """OpenAI messages 数组(术语表系统提示 + 用户指令)。"""
    return [{'role': 'system', 'content': SYSTEM_PROMPT},
            {'role': 'user', 'content': raw}]


def parse_intent(text: str, llm_fn=None, llm_error=None) -> dict:
    """双层解析入口:LLM 主层(llm_fn(text)->dict|None),返回 None/
    非法 JSON/抛异常(含超时)时自动切换兜底规则层。

    llm_fn 由 voice_command_node 注入(urllib 调 OpenAI 兼容接口,
    超时默认 3s);单元测试注入假函数即可覆盖两层路径。
    """
    if llm_fn is not None:
        try:
            checked = validate_intent(llm_fn(text), raw=text)
            if checked is not None:
                return checked
        except Exception as e:  # noqa: BLE001 —— 兜底层的意义就是吞掉一切
            if llm_error is not None:
                llm_error.append(repr(e))
    return parse_intent_rule(text)
