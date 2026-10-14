"""一步一工件的表格 Q-learning。

每个工件是独立的一步：状态在搬之前就知道，动作是策略编号，
奖励来自记录器的真实结果。gamma 为 0，因为放下之后没有下一状态。
冻结后只能查询，不能再改表。
"""
from __future__ import annotations

import json
import random

# 和 sorting_params.yaml 的颜色规则一致：暖色进 A，冷色进 B。
BIN_FOR = {
    'red': 'bin_A',
    'yellow': 'bin_A',
    'green': 'bin_B',
    'blue': 'bin_B',
}

# 成功和失败相差 20。耗时大约十几秒，乘 0.2 后只扣几分，
# 所以放对仍然比放错重要，但更快的成功会留下更高的奖励。
REWARD_TIME_COEF = 0.2
PLACED_BONUS = 10.0
FAILURE_PENALTY = -10.0


def state_key(color):
    """状态 = 颜色|目标料区。料区由固定规则决定，不看执行结果。"""
    name = '' if color is None else str(color)
    return '%s|%s' % (name, BIN_FOR.get(name, 'unknown'))


def placement_reward(result, elapsed_sec, time_coef=REWARD_TIME_COEF):
    """PLACED 得 +10，其余结果得 -10，再减去耗时乘系数。"""
    base = PLACED_BONUS if result == 'PLACED' else FAILURE_PENALTY
    try:
        elapsed = float(elapsed_sec)
    except (TypeError, ValueError):
        elapsed = 0.0
    if elapsed < 0.0:
        elapsed = 0.0
    return base - float(time_coef) * elapsed


class TabularQ(object):

    def __init__(self, actions, alpha=0.4, gamma=0.0, epsilon=0.3,
                 seed=7, time_coef=REWARD_TIME_COEF):
        self.actions = [int(a) for a in actions]
        self.alpha = float(alpha)
        self.gamma = float(gamma)
        self.epsilon = float(epsilon)
        self.time_coef = float(time_coef)
        self.rng = random.Random(seed)
        self.seed = int(seed)
        self.frozen = False
        self.q = {}

    def _ensure(self, state):
        row = self.q.get(state)
        if row is None:
            row = {action: 0.0 for action in self.actions}
            self.q[state] = row
        return row

    def greedy_actions(self, state):
        row = self._ensure(state)
        best = max(row[action] for action in self.actions)
        return [action for action in self.actions if row[action] == best]

    def choose(self, state):
        """训练时按 epsilon 探索。冻结时选 Q 最大的动作，并列时取最小编号。"""
        self._ensure(state)
        if not self.frozen and self.rng.random() < self.epsilon:
            return self.rng.choice(self.actions)
        bests = self.greedy_actions(state)
        if not self.frozen:
            return self.rng.choice(bests)
        return min(bests)

    def update(self, state, action, reward, next_state=None):
        """Q(s,a) += alpha * (r + gamma * max Q(s') - Q(s,a))。"""
        if self.frozen:
            raise RuntimeError('冻结后的策略不能再更新')
        row = self._ensure(state)
        action = int(action)
        future = 0.0
        if next_state is not None and self.gamma != 0.0:
            nxt = self._ensure(next_state)
            future = max(nxt[a] for a in self.actions)
        old = row[action]
        row[action] = old + self.alpha * (float(reward) + self.gamma * future - old)

    def freeze(self):
        self.frozen = True

    def to_dict(self):
        table = {}
        for state in sorted(self.q):
            table[state] = {str(action): self.q[state][action] for action in self.actions}
        return {
            'alpha': self.alpha,
            'gamma': self.gamma,
            'epsilon': self.epsilon,
            'time_coef': self.time_coef,
            'seed': self.seed,
            'frozen': self.frozen,
            'actions': list(self.actions),
            'q': table,
        }

    def save(self, path):
        with open(path, 'w', encoding='utf-8') as handle:
            json.dump(self.to_dict(), handle, ensure_ascii=False, indent=2, sort_keys=True)
            handle.write('\n')
