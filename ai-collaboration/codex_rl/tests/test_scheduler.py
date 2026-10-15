import copy
import itertools
import json
from pathlib import Path
import tempfile
import unittest

from dual_arm_rl.model import Model, WAIT, load_config, rule_action, run, validate_config
from dual_arm_rl.qtable import choose, load, new_table, save, state_key, validate
from dual_arm_rl.train import train, epsilon_at
from dual_arm_rl.evaluate import evaluate


def config():
    c = load_config(Path(__file__).resolve().parents[1] / "config/timing.placeholder.json")
    for field, seconds in (("feed_seconds", 2.0), ("pick_seconds", 3.0), ("post_pick_seconds", 5.0)):
        c[field] = dict.fromkeys(c["color_order"], seconds)
    c["request_delay_seconds"] = 0.0
    return c


class ModelTests(unittest.TestCase):
    def test_empty(self):
        m = run(config(), [0]*4)
        self.assertTrue(m.done)
        self.assertEqual((m.time, m.completed, m.legal_actions()), (0, 0, []))

    def test_single_includes_return_and_reward(self):
        m = Model(config(), [1, 0, 0, 0])
        _, reward, done = m.step(0)
        self.assertTrue(done)
        self.assertEqual(m.time, 10)
        self.assertAlmostEqual(reward, -10 - config()["idle_weight"]*2)

    def test_same_arm_prefeed_and_serial(self):
        self.assertEqual(run(config(), [2, 0, 0, 0], "serial").time, 20)
        self.assertEqual(run(config(), [2, 0, 0, 0], "rule").time, 18)

    def test_cross_arm_overlap(self):
        m = run(config(), [1, 0, 1, 0])
        self.assertEqual(m.time, 15)
        self.assertEqual(run(config(), [1, 0, 1, 0], "serial", [2, 0]).time, 20)

    def test_nonoverlap(self):
        c = config()
        c["allow_arm_overlap"] = False
        self.assertEqual(run(c, [1, 0, 1, 0]).time, 18)

    def test_no_prefeed(self):
        c = config()
        c["allow_prefeed_busy_arm"] = False
        m = Model(c, [2, 0, 0, 0])
        m.step(0)
        self.assertEqual(m.legal_actions(), [WAIT])
        before = m.time
        m.step(WAIT)
        self.assertGreater(m.time, before)
        m.step(0)
        self.assertEqual(m.time, 20)

    def test_delay_in_serial(self):
        c = config()
        c["request_delay_seconds"] = 1.5
        self.assertEqual(run(c, [2, 0, 0, 0], "serial").time, 21.5)

    def test_illegal_actions_do_not_mutate(self):
        m = Model(config(), [1, 0, 0, 0])
        for action in (WAIT, 2, -1, True, 1.5):
            with self.assertRaises(ValueError):
                m.step(action)
        self.assertEqual(m.state()[:4], (1, 0, 0, 0))
        self.assertEqual(m.time, 0)

    def test_inventory_and_mutual_exclusion_many_batches(self):
        for counts in itertools.product(range(3), repeat=4):
            m = run(config(), counts)
            self.assertEqual(m.completed, sum(counts))
            self.assertEqual(m.requested, sum(counts))
            self.assertEqual(m.remaining, [0]*4)
            slot, owner = None, None
            for e in m.trace:
                if e["event"] == "FEED_RESERVED":
                    self.assertIsNone(slot)
                    slot = e["part"]
                elif e["event"] == "PICK_ENTER":
                    self.assertIsNone(owner)
                    self.assertEqual(e["part"], slot)
                    owner = e["arm"]
                elif e["event"] == "PICK_EXIT":
                    self.assertEqual(owner, e["arm"])
                    owner, slot = None, None
            self.assertIsNone(slot)

    def test_bad_config_and_batch(self):
        for value in (0, -1, float("inf"), float("nan"), True):
            c = config()
            c["pick_seconds"]["blue"] = value
            with self.assertRaises(ValueError):
                validate_config(c)
        for batch in ([1], [-1, 0, 0, 0], [True, 0, 0, 0]):
            with self.assertRaises(ValueError):
                Model(config(), batch)


class QTableTests(unittest.TestCase):
    def test_unknown_and_mask(self):
        c = config()
        t = new_table(c)
        m = Model(c, [1, 0, 1, 0])
        self.assertEqual(choose(t, m), rule_action(m))
        t["q_values"][state_key(m.state())] = [-5, 999, -1, 999, 999]
        self.assertEqual(choose(t, m), 2)  # illegal WAIT and colors never win

    def test_reject_corruption(self):
        c = config()
        t = new_table(c)
        key = state_key(Model(c, [1, 0, 0, 0]).state())
        t["q_values"][key] = [0.0]*5
        for row in ([0]*4, [0, 0, float("nan"), 0, 0], [True]*5):
            bad = copy.deepcopy(t)
            bad["q_values"][key] = row
            with self.assertRaises(ValueError):
                validate(bad, c)
        for field in ("schema_version", "config_sha256", "action_order", "deployment_ready"):
            bad = copy.deepcopy(t)
            bad[field] = "wrong"
            with self.assertRaises(ValueError):
                validate(bad, c)

    def test_bad_state(self):
        c = config()
        for state in ([1, 0, 0, 0, "READY", "READY", 1, 0, -1, -1],
                      [1, 0, 0, 0, "POST_PICK", "READY", 1, 0, 2, -1]):
            t = new_table(c)
            t["q_values"][state_key(state)] = [0]*5
            with self.assertRaises(ValueError):
                validate(t, c)

    def test_training_reproducible_and_round_trip(self):
        c = config()
        a = train(c, episodes=100, batch_size=4, seed=11)
        self.assertEqual(a, train(c, episodes=100, batch_size=4, seed=11))
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)/"q_table.json"
            save(p, a, c)
            self.assertEqual(load(p, c), a)
            c["feed_seconds"]["blue"] += 1
            with self.assertRaises(ValueError):
                load(p, c)
            p.write_text('{"schema_version":1,"schema_version":1}')
            with self.assertRaises(ValueError):
                load(p, config())

    def test_single_task_bellman_update(self):
        c = config()
        t = train(c, episodes=1, batch=[1, 0, 0, 0], alpha=1, epsilon=0)
        key = state_key(Model(c, [1, 0, 0, 0]).state())
        self.assertAlmostEqual(t["q_values"][key][0], -10.1)

    def test_small_batch_learns_enumerated_optimum(self):
        c = config()
        c["idle_weight"] = 0
        batch = [1, 1, 1, 0]
        costs = []
        for order in itertools.permutations([0, 1, 2]):
            actions = iter(order)
            costs.append(run(c, batch, lambda _: next(actions)).time)
        t = train(c, episodes=1500, batch=batch, seed=7)
        self.assertEqual(run(c, batch, lambda m: choose(t, m)).time, min(costs))

    def test_linear_epsilon_schedule(self):
        self.assertEqual(epsilon_at(0, 50000), 0.5)
        self.assertAlmostEqual(epsilon_at(49999, 50000), 0.05)
        self.assertAlmostEqual(epsilon_at(1, 3), 0.275)
        self.assertEqual(epsilon_at(0, 1), 0.5)
        with self.assertRaises(ValueError):
            epsilon_at(0, 10, 0.05, 0.5)
        table = train(config(), episodes=3, batch_size=2)
        self.assertEqual(table["training"]["epsilon_start"], 0.5)
        self.assertEqual(table["training"]["epsilon_end"], 0.05)

    def test_fallback_counter_only_counts_missing_rows(self):
        c = config()
        t = new_table(c)
        m = Model(c, [1, 0, 1, 0])
        stats = {}
        choose(t, m, stats)
        self.assertEqual(stats, {"decision_steps": 1, "unknown_state_fallback_steps": 1})
        t["q_values"][state_key(m.state())] = [0.0]*5
        choose(t, m, stats)  # known row tie uses rule, but is NOT unknown fallback
        self.assertEqual(stats, {"decision_steps": 2, "unknown_state_fallback_steps": 1})

    def test_empty_table_evaluation_counts_every_q_decision(self):
        rows = evaluate(config(), new_table(config()), batches=3, batch_size=4)
        for row in rows:
            if row["policy"] == "q_learning":
                self.assertGreaterEqual(row["q_decision_steps"], 4)
                self.assertEqual(row["q_decision_steps"], row["unknown_state_fallback_steps"])
            else:
                self.assertEqual(row["q_decision_steps"], 0)
                self.assertEqual(row["unknown_state_fallback_steps"], 0)

    def test_paired_evaluation(self):
        c = config()
        rows = evaluate(c, new_table(c), batches=3, batch_size=4)
        self.assertEqual(len(rows), 9)
        for i in range(0, 9, 3):
            self.assertEqual(len({r["counts"] for r in rows[i:i+3]}), 1)
            self.assertTrue(all(r["completed"] == 4 for r in rows[i:i+3]))


if __name__ == "__main__":
    unittest.main()
