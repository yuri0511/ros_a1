"""Tabular Q-learning for finite batches with undiscounted elapsed-time cost."""
import argparse
import json
import random
from .model import Model, load_config
from .qtable import choose, new_table, save, state_key


def epsilon_at(episode, episodes, start=0.5, end=0.05):
    """Linear episode schedule, including both endpoints (one episode uses start)."""
    if episodes < 1 or not 0 <= episode < episodes or not 0 <= end <= start <= 1:
        raise ValueError("Invalid epsilon schedule")
    return start + (end - start) * episode / max(1, episodes - 1)


def train(config, episodes=50000, batch_size=8, seed=42, alpha=0.2, epsilon=None,
          batch=None, epsilon_start=0.5, epsilon_end=0.05):
    if type(episodes) is not int or episodes <= 0 or type(batch_size) is not int or batch_size <= 0:
        raise ValueError("episodes and batch_size must be positive integers")
    if not 0 < alpha <= 1:
        raise ValueError("Invalid learning rate")
    # Keep explicit constant epsilon for controlled tests and existing callers.
    if epsilon is not None:
        epsilon_start = epsilon_end = epsilon
    epsilon_at(0, episodes, epsilon_start, epsilon_end)
    rng = random.Random(seed)
    table = new_table(config)
    for episode in range(episodes):
        current_epsilon = epsilon_at(episode, episodes, epsilon_start, epsilon_end)
        counts = list(batch) if batch is not None else [0]*4
        if batch is None:
            for _ in range(batch_size):
                counts[rng.randrange(4)] += 1
        model = Model(config, counts)
        while not model.done:
            key = state_key(model.state())
            row = table["q_values"].setdefault(key, [0.0]*5)
            action = rng.choice(model.legal_actions()) if rng.random() < current_epsilon else choose(table, model)
            next_state, reward, done = model.step(action)
            continuation = 0.0
            if not done:
                next_row = table["q_values"].get(state_key(next_state), [0.0]*5)
                continuation = max(next_row[a] for a in model.legal_actions())
            # gamma=1: undiscounted episodic time cost; terminal tail is in reward.
            row[action] += alpha * (reward + continuation - row[action])
    table["training"] = dict(episodes=episodes, seed=seed, batch_size=batch_size,
                             fixed_batch=batch, alpha=alpha, epsilon_schedule="linear",
                             epsilon_start=epsilon_start, epsilon_end=epsilon_end, gamma=1.0)
    return table


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/timing.placeholder.json")
    parser.add_argument("--output", default="artifacts/q_table.json")
    parser.add_argument("--episodes", type=int, default=50000)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--epsilon-start", type=float, default=0.5)
    parser.add_argument("--epsilon-end", type=float, default=0.05)
    args = parser.parse_args()
    config = load_config(args.config)
    table = train(config, args.episodes, args.batch_size, args.seed,
                  epsilon_start=args.epsilon_start, epsilon_end=args.epsilon_end)
    save(args.output, table, config)
    print(json.dumps(dict(output=args.output, states=len(table["q_values"]),
                         is_placeholder=config["is_placeholder"], deployment_ready=False)))


if __name__ == "__main__":
    main()
