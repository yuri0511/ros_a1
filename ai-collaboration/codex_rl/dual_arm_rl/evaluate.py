"""Paired comparison using identical batch multiset and initial conditions."""
import argparse
import csv
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
        for name, policy in (("serial", "serial"), ("rule", "rule"),
                             ("q_learning", lambda m: choose(table, m))):
            model = run(config, counts, policy, order)
            rows.append(dict(batch_id=batch_id, counts="/".join(map(str, counts)),
                             original_order="/".join(map(str, order)), policy=name,
                             makespan_seconds=model.time, demand_idle_arm_seconds=model.idle,
                             completed=model.completed, is_placeholder=config["is_placeholder"]))
    return rows


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
    rows = evaluate(config, load(args.table, config), args.batches, args.batch_size, args.seed)
    path = Path(args.output)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"Wrote {len(rows)} model-only rows to {path}; not Gazebo measurements")


if __name__ == "__main__":
    main()
