"""从实验 CSV 生成文字摘要和 SVG 图。不依赖 numpy 或 matplotlib。"""
from __future__ import annotations

import csv
from collections import defaultdict
from pathlib import Path


def read_csv(path):
    with open(path, newline='', encoding='utf-8') as handle:
        return list(csv.DictReader(handle))


def _float(text):
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


def strategy_summary(rows):
    """按策略汇总成功率、平均耗时和失败原因。失败留在分母里。"""
    grouped = defaultdict(list)
    for row in rows:
        grouped[row.get('strategy_name', '')].append(row)
    summaries = []
    for name in sorted(grouped):
        items = grouped[name]
        elapsed = [value for value in (_float(row.get('elapsed_sec')) for row in items)
                   if value is not None]
        placed = sum(1 for row in items if row.get('result') == 'PLACED')
        reasons = defaultdict(int)
        for row in items:
            if row.get('result') == 'PLACED':
                continue
            reason = row.get('detail') or row.get('result') or 'UNKNOWN'
            reasons[reason] += 1
        summaries.append({
            'strategy_name': name,
            'count': len(items),
            'placed': placed,
            'success_rate': (placed / len(items)) if items else 0.0,
            'mean_elapsed': (sum(elapsed) / len(elapsed)) if elapsed else 0.0,
            'failures': dict(reasons),
        })
    return summaries


def failure_counts(rows):
    counts = defaultdict(int)
    for row in rows:
        if row.get('result') == 'PLACED':
            continue
        reason = row.get('detail') or row.get('result') or 'UNKNOWN'
        counts[reason] += 1
    return dict(counts)


def mean_reward(rows, phase=None):
    chosen = rows
    if phase is not None:
        chosen = [row for row in rows if row.get('phase') == phase]
    values = [value for value in (_float(row.get('reward')) for row in chosen)
              if value is not None]
    if not values:
        return None
    return sum(values) / len(values)


def mean_reward_from_results(rows):
    """策略实测表没有 reward 列时，用和训练相同的公式逐件计算。"""
    from a1_sorting_core.q_learning import placement_reward
    values = [placement_reward(row.get('result'), row.get('elapsed_sec'))
              for row in rows if row.get('result')]
    if not values:
        return None
    return sum(values) / len(values)


def _svg_header(width, height, title):
    return (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d">\n'
        '<rect width="100%%" height="100%%" fill="white"/>\n'
        '<text x="24" y="32" font-size="18">%s</text>\n'
    ) % (width, height, _escape(title))


def _escape(text):
    return (str(text).replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;'))


def bar_svg(title, labels, values, ylabel, path):
    width, height = 720, 420
    left, top, plot_h = 70, 60, 280
    parts = [_svg_header(width, height, title)]
    parts.append('<text x="16" y="%d" font-size="12">%s</text>' % (top + 12, _escape(ylabel)))
    peak = max(values) if values else 1.0
    if peak <= 0:
        peak = 1.0
    slot = 560 / max(1, len(values))
    for index, (label, value) in enumerate(zip(labels, values)):
        bar_h = plot_h * (value / peak)
        x = left + index * slot + slot * 0.2
        y = top + 40 + plot_h - bar_h
        parts.append(
            '<rect x="%.1f" y="%.1f" width="%.1f" height="%.1f" fill="#3b6ea5"/>'
            % (x, y, slot * 0.6, bar_h))
        parts.append(
            '<text x="%.1f" y="%d" font-size="12" text-anchor="middle">%s</text>'
            % (x + slot * 0.3, top + 40 + plot_h + 24, _escape(label)))
        parts.append(
            '<text x="%.1f" y="%.1f" font-size="12" text-anchor="middle">%.3g</text>'
            % (x + slot * 0.3, y - 6, value))
    parts.append('</svg>\n')
    Path(path).write_text(''.join(parts), encoding='utf-8')


def line_svg(title, values, path):
    width, height = 720, 420
    left, top, plot_w, plot_h = 60, 60, 620, 280
    parts = [_svg_header(width, height, title)]
    if not values:
        parts.append('<text x="60" y="120" font-size="14">没有数据</text></svg>\n')
        Path(path).write_text(''.join(parts), encoding='utf-8')
        return
    low = min(values)
    high = max(values)
    if high == low:
        high = low + 1.0
    coords = []
    for index, value in enumerate(values):
        x = left + (plot_w * index / max(1, len(values) - 1))
        y = top + plot_h - (plot_h * (value - low) / (high - low))
        coords.append('%.1f,%.1f' % (x, y))
    parts.append('<polyline fill="none" stroke="#b45309" stroke-width="2" points="%s"/>'
                 % ' '.join(coords))
    parts.append('<text x="60" y="%d" font-size="12">最低 %.3g，最高 %.3g</text>'
                 % (top + plot_h + 36, low, high))
    parts.append('</svg>\n')
    Path(path).write_text(''.join(parts), encoding='utf-8')


def write_report(strategy_csv, q_csv, out_dir):
    """写出摘要和五张图。返回摘要文字。"""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    strategy_rows = read_csv(strategy_csv) if strategy_csv else []
    q_rows = read_csv(q_csv) if q_csv else []
    summaries = strategy_summary(strategy_rows)
    lines = ['策略实测（失败计入分母）']
    for item in summaries:
        lines.append(
            '%s: %d 件, 成功 %d, 成功率 %.3f, 平均耗时 %.3f 秒, 失败 %s'
            % (item['strategy_name'], item['count'], item['placed'],
               item['success_rate'], item['mean_elapsed'], item['failures'] or '{}'))
    train_mean = mean_reward(q_rows, 'train')
    frozen_mean = mean_reward(q_rows, 'frozen')
    lines.append('训练期 train 平均奖励: %s' % ('无' if train_mean is None else '%.3f' % train_mean))
    lines.append('冻结期 frozen 平均奖励: %s' % ('无' if frozen_mean is None else '%.3f' % frozen_mean))
    text = '\n'.join(lines) + '\n'
    (out / 'summary.txt').write_text(text, encoding='utf-8')

    bar_svg('成功率', [item['strategy_name'] for item in summaries],
            [item['success_rate'] for item in summaries], 'PLACED / 全部件数',
            out / 'success_rate.svg')
    bar_svg('平均耗时', [item['strategy_name'] for item in summaries],
            [item['mean_elapsed'] for item in summaries], '秒',
            out / 'mean_time.svg')
    reasons = failure_counts(strategy_rows + q_rows)
    if not reasons:
        reasons = {'无失败': 0.0}
    bar_svg('失败原因', list(reasons), [float(reasons[key]) for key in reasons],
            '次数', out / 'failure_reasons.svg')
    rewards = [_float(row.get('reward')) for row in q_rows]
    rewards = [value for value in rewards if value is not None]
    line_svg('每件奖励', rewards, out / 'reward_curve.svg')

    labels = []
    values = []
    grouped = defaultdict(list)
    for row in strategy_rows:
        grouped[row.get('strategy_name', '')].append(row)
    for name in sorted(grouped):
        score = mean_reward_from_results(grouped[name])
        if score is None:
            continue
        labels.append(name)
        values.append(score)
    if frozen_mean is not None:
        labels.append('frozen')
        values.append(frozen_mean)
    bar_svg('学习前后平均奖励', labels, values, '奖励', out / 'before_after.svg')
    return text
