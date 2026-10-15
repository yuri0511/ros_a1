"""Paired comparison using identical batch multiset and initial conditions."""
import argparse
import csv
import json
import hashlib
from statistics import mean
from pathlib import Path
import random
from .model import load_config, run
from .qtable import choose, load


def evaluate(config, table, batches=30, batch_size=8, seed=2026):
    if batches <= 0 or batch_size <= 0:
        raise ValueError("Evaluation sizes must be positive")
    rng = random.Random(seed)
    rows = []
    for batch_id in range(batches):
        order = [rng.randrange(4) for _ in range(batch_size)]
        counts = [order.count(c) for c in range(4)]
        stats = {"decision_steps": 0, "unknown_state_fallback_steps": 0}
        for name, policy in (("serial", "serial"), ("rule", "rule"),
                             ("q_learning", lambda m: choose(table, m, stats))):
            model = run(config, counts, policy, order)
            rows.append(dict(batch_id=batch_id, counts="/".join(map(str, counts)),
                             original_order="/".join(map(str, order)), policy=name,
                             makespan_seconds=model.time, demand_idle_arm_seconds=model.idle,
                             completed=model.completed, is_placeholder=config["is_placeholder"],
                             q_decision_steps=stats["decision_steps"] if name == "q_learning" else 0,
                             unknown_state_fallback_steps=stats["unknown_state_fallback_steps"] if name == "q_learning" else 0))
    return rows


def summarize(rows):
    summary = {}
    for policy in ("serial", "rule", "q_learning"):
        group = [r for r in rows if r["policy"] == policy]
        steps = sum(r["q_decision_steps"] for r in group)
        fallbacks = sum(r["unknown_state_fallback_steps"] for r in group)
        summary[policy] = dict(batches=len(group),
                               mean_makespan_seconds=mean(r["makespan_seconds"] for r in group),
                               q_decision_steps=steps, unknown_state_fallback_steps=fallbacks,
                               unknown_state_fallback_rate=fallbacks / steps if steps else None)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/timing.placeholder.json")
    parser.add_argument("--table", default="artifacts/q_table.json")
    parser.add_argument("--output", default="artifacts/comparison.csv")
    parser.add_argument("--batches", type=int, default=30)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--seed", type=int, default=2026)
    args = parser.parse_args()
    config = load_config(args.config)
    table = load(args.table, config)
    rows = evaluate(config, table, args.batches, args.batch_size, args.seed)
    path = Path(args.output)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    summary = summarize(rows)
    summary_path = path.with_suffix(".summary.json")
    summary_path.write_text(json.dumps(dict(evaluation_seed=args.seed, batch_size=args.batch_size,
                                           config=args.config, table=args.table,
                                           table_sha256=hashlib.sha256(Path(args.table).read_bytes()).hexdigest(),
                                           training=table.get("training"),
                                           is_placeholder=config["is_placeholder"], results=summary),
                                       indent=2, allow_nan=False) + "\n")
    print(json.dumps(summary, indent=2))
    print(f"Wrote {len(rows)} model-only rows to {path}; not Gazebo measurements")


if __name__ == "__main__":
    main()
