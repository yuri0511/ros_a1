"""Deterministic discrete-event model with one feeder and one pick zone.

Decisions occur only at t=0, safe pick exit, or an arm-ready event. A feed
action reserves the single feed slot immediately and advances through pick
exit. Other arm events are processed during this advance. Exact event times
are internal: the observation uses configured phase-duration estimates.
"""
from bisect import bisect_left
from dataclasses import dataclass
import hashlib
import heapq
import json
import math
from pathlib import Path

COLORS = ("blue", "yellow", "red", "green")
ACTIONS = ("FEED_BLUE", "FEED_YELLOW", "FEED_RED", "FEED_GREEN", "WAIT_NEXT_EVENT")
WAIT = 4
STATE_FIELDS = ("remaining[4]", "arm_phase[2]", "eta_bin[2]", "active_color[2]")


def load_config(path):
    config = json.loads(Path(path).read_text())
    validate_config(config)
    return config


def validate_config(c):
    if c.get("schema_version") != 1 or c.get("color_order") != list(COLORS):
        raise ValueError("Unsupported configuration schema/color order")
    if c.get("color_to_arm") != [0, 0, 1, 1]:
        raise ValueError("Routing must remain blue/yellow -> arm1, red/green -> arm2")
    for field in ("is_placeholder", "overlap_validated", "allow_arm_overlap", "allow_prefeed_busy_arm"):
        if type(c.get(field)) is not bool:
            raise ValueError(f"{field} must be a boolean")
    if not isinstance(c.get("timing_model_version"), str) or not c["timing_model_version"]:
        raise ValueError("Missing timing model version")
    for field in ("feed_seconds", "pick_seconds", "post_pick_seconds"):
        if set(c.get(field, {})) != set(COLORS):
            raise ValueError(f"{field} must contain exactly four colors")
        for x in c[field].values():
            if type(x) not in (int, float) or not math.isfinite(x) or x <= 0:
                raise ValueError(f"{field} durations must be finite and positive")
    edges = c.get("eta_bin_edges_seconds")
    if not isinstance(edges, list) or not edges:
        raise ValueError("Missing ETA bins")
    if any(type(x) not in (int, float) or not math.isfinite(x) or x <= 0 for x in edges):
        raise ValueError("ETA boundaries must be finite and positive")
    if any(a >= b for a, b in zip(edges, edges[1:])):
        raise ValueError("ETA boundaries must be strictly increasing")
    w = c.get("idle_weight")
    if type(w) not in (int, float) or not math.isfinite(w) or w < 0:
        raise ValueError("idle_weight must be finite and nonnegative")
    delay = c.get("request_delay_seconds")
    if type(delay) not in (int, float) or not math.isfinite(delay) or delay < 0:
        raise ValueError("request_delay_seconds must be finite and nonnegative")


def config_hash(config):
    return hashlib.sha256(json.dumps(config, sort_keys=True, allow_nan=False).encode()).hexdigest()


@dataclass
class Arm:
    phase: str = "READY"
    color: int = -1
    phase_start: float = 0.0
    part: int = -1


class Model:
    def __init__(self, config, batch):
        validate_config(config)
        if len(batch) != 4 or any(type(n) is not int or n < 0 for n in batch):
            raise ValueError("Batch must be four nonnegative integer counts")
        self.config = config
        self.initial = tuple(batch)
        self.remaining = list(batch)
        self.arms = [Arm(), Arm()]
        self.time = 0.0
        self.idle = 0.0  # integrated ready-arm seconds with unserved demand
        self.events = []
        self.trace = []
        self.slot = None
        self.pick_owner = None
        self.completed = 0
        self.requested = 0
        self._event_seq = 0

    def _duration(self, field, color):
        return self.config[field][COLORS[color]]

    def state(self):
        eta = []
        for arm in self.arms:
            if arm.phase == "READY":
                eta.append(0)
            else:
                estimate = self._duration("post_pick_seconds", arm.color)
                if arm.phase == "PICK":
                    estimate += self._duration("pick_seconds", arm.color)
                remaining = max(0.0, estimate - (self.time - arm.phase_start))
                eta.append(1 + bisect_left(self.config["eta_bin_edges_seconds"], remaining))
        return tuple(self.remaining) + tuple(a.phase for a in self.arms) + tuple(eta) + tuple(a.color for a in self.arms)

    @property
    def done(self):
        return self.completed == sum(self.initial)

    def legal_actions(self):
        if self.done:
            return []
        result = []
        if self.slot is None:
            for color, count in enumerate(self.remaining):
                arm = self.arms[self.config["color_to_arm"][color]]
                if count and (self.config["allow_prefeed_busy_arm"] or arm.phase == "READY"):
                    result.append(color)
        if self.events:
            result.append(WAIT)
        return result

    def _log(self, event, arm=None, color=None, part=None):
        self.trace.append(dict(time=self.time, event=event, arm=arm, color=color, part=part))

    def _schedule(self, delay, event, arm):
        self._event_seq += 1
        heapq.heappush(self.events, (self.time + delay, self._event_seq, event, arm))

    def _advance_clock(self, time):
        for i, arm in enumerate(self.arms):
            waiting = sum(n for c, n in enumerate(self.remaining) if self.config["color_to_arm"][c] == i)
            if self.slot is not None and self.config["color_to_arm"][self.slot[0]] == i:
                waiting += 1
            if arm.phase == "READY" and waiting:
                self.idle += time - self.time
        self.time = time

    def _try_pick(self):
        if self.slot is None or not self.slot[2] or self.pick_owner is not None:
            return
        color, part, _ = self.slot
        i = self.config["color_to_arm"][color]
        if self.arms[i].phase != "READY":
            return
        if not self.config["allow_arm_overlap"] and any(a.phase != "READY" for a in self.arms):
            return
        self.arms[i] = Arm("PICK", color, self.time, part)
        self.pick_owner = i
        self._log("PICK_ENTER", i, color, part)
        self._schedule(self._duration("pick_seconds", color), "PICK_EXIT", i)

    def _next_events(self):
        if not self.events:
            raise RuntimeError("Deadlock: unfinished batch without future events")
        next_time = self.events[0][0]
        self._advance_clock(next_time)
        # Co-timed events are all applied before exposing another decision.
        while self.events and self.events[0][0] == next_time:
            _, _, event, i = heapq.heappop(self.events)
            if event == "FEED_REQUEST":
                color, part, _ = self.slot
                self._log(event, None, color, part)
                self._schedule(self._duration("feed_seconds", color), "FEED_READY", None)
            elif event == "FEED_READY":
                color, part, _ = self.slot
                self.slot = (color, part, True)
                self._log(event, None, color, part)
            elif event == "PICK_EXIT":
                arm = self.arms[i]
                self._log(event, i, arm.color, arm.part)
                arm.phase = "POST_PICK"
                arm.phase_start = self.time
                self.pick_owner = None
                self.slot = None
                self._schedule(self._duration("post_pick_seconds", arm.color), "ARM_READY", i)
            elif event == "ARM_READY":
                arm = self.arms[i]
                self._log(event, i, arm.color, arm.part)
                self.completed += 1
                self.arms[i] = Arm()
        self._try_pick()

    def step(self, action):
        if type(action) is not int or action not in self.legal_actions():
            raise ValueError(f"Illegal action: {action}")
        start, idle = self.time, self.idle
        if action == WAIT:
            self._next_events()
        else:
            self.remaining[action] -= 1  # reservation on request, never on arrival
            self.requested += 1
            self.slot = (action, self.requested, False)
            self._log("FEED_RESERVED", None, action, self.requested)
            delay = self.config["request_delay_seconds"] if self.requested > 1 else 0.0
            self._schedule(delay, "FEED_REQUEST", None)
            while self.slot is not None:
                self._next_events()
        # Include the last tasks' full return time in the final transition.
        if not any(self.remaining):
            while self.events:
                self._next_events()
        reward = -(self.time - start) - self.config["idle_weight"] * (self.idle - idle)
        return self.state(), reward, self.done


def rule_action(model):
    """Ready arm first; otherwise earliest estimated arm availability, color tie."""
    legal = model.legal_actions()
    feeds = [a for a in legal if a != WAIT]
    if not feeds:
        return WAIT
    state = model.state()
    return min(feeds, key=lambda a: (state[6 + model.config["color_to_arm"][a]], a))


def run(config, batch, policy="rule", order=None):
    model = Model(config, batch)
    if policy == "serial":
        # A supplied order makes comparison with a prescribed original batch fair.
        order = list(order) if order is not None else [c for c, n in enumerate(batch) for _ in range(n)]
        if any(type(c) is not int or c not in range(4) for c in order) or [order.count(c) for c in range(4)] != list(batch):
            raise ValueError("Serial order does not match batch counts")
    while not model.done:
        if policy == "serial":
            action = WAIT if any(a.phase != "READY" for a in model.arms) else order.pop(0)
        elif policy == "rule":
            action = rule_action(model)
        elif callable(policy):
            action = policy(model)
        else:
            raise ValueError("Unknown policy")
        model.step(action)
    return model
