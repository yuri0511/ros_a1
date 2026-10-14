"""批次事件的名字。调度、执行、记录和来料节点共用这些字符串。

reset: 清空本轮状态，不开始新轮。
begin:<batch_id>: 开始一轮，batch_id 由 spawn_parts 生成。
"""

BATCH_TOPIC = '/batch/event'
MANIFEST_TOPIC = '/parts/manifest'
PART_RESULT_TOPIC = '/result/part'
READY_PREFIX = '/a1/ready'


def begin_event(batch_id):
    return 'begin:%s' % batch_id


def parse_event(text):
    """返回 (kind, batch_id)。kind 为 reset、begin 或 unknown。"""
    data = '' if text is None else str(text)
    if data == 'reset':
        return 'reset', ''
    prefix = 'begin:'
    if data.startswith(prefix) and len(data) > len(prefix):
        return 'begin', data[len(prefix):]
    return 'unknown', ''


def batch_id_from_spawn_message(message):
    """从 spawn 的返回文字里取出 batch_0001。对不上时返回空串。"""
    text = '' if message is None else str(message)
    open_at = text.find('(')
    close_at = text.find(')', open_at + 1)
    if open_at < 0 or close_at < 0:
        return ''
    return text[open_at + 1:close_at]
