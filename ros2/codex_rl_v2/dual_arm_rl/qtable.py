"""Versioned JSON table; strict validation before use, legal-action fallback."""
import json
import math
from pathlib import Path
from .model import ACTIONS, COLORS, STATE_FIELDS, config_hash, rule_action


def state_key(state):
    return json.dumps(list(state), separators=(",", ":"), ensure_ascii=True)


def new_table(config):
    return {
        "schema_version": 2,
        "color_order": list(COLORS), "action_order": list(ACTIONS),
        "state_fields": list(STATE_FIELDS), "color_to_arm": [0, 0, 1, 1],
        "eta_bin_edges_seconds": config["eta_bin_edges_seconds"],
        "timing_model_version": config["timing_model_version"],
        "config_sha256": config_hash(config),
        "is_placeholder": config["is_placeholder"],
        "overlap_validated": config["overlap_validated"],
        "deployment_ready": False,
        "fallback_policy": "ready_arm_then_eta_then_color",
        "q_values": {},
    }


def validate(table, config):
    expected = new_table(config)
    for field, value in expected.items():
        if field != "q_values" and (table.get(field) != value or type(table.get(field)) is not type(value)):
            raise ValueError(f"Q table metadata mismatch: {field}")
    rows = table.get("q_values")
    if not isinstance(rows, dict):
        raise ValueError("q_values must be an object")
    for key, row in rows.items():
        try:
            state = json.loads(key)
        except (ValueError, TypeError) as exc:
            raise ValueError("Invalid state key") from exc
        if not isinstance(state, list) or len(state) != 10 or state_key(state) != key:
            raise ValueError("State key must be canonical ten-element JSON array")
        if any(type(n) is not int or n < 0 for n in state[:4]):
            raise ValueError("Invalid remaining counts")
        for i in range(2):
            phase, eta, color = state[4+i], state[6+i], state[8+i]
            # PICK/RECOVERY cannot be observable at this prototype's decision epochs.
            if phase not in ("READY", "POST_PICK"):
                raise ValueError("Unsupported decision phase")
            if type(eta) is not int or not 0 <= eta <= len(config["eta_bin_edges_seconds"]) + 1:
                raise ValueError("Invalid ETA bin")
            if type(color) is not int or color not in (-1, 0, 1, 2, 3):
                raise ValueError("Invalid active color")
            if phase == "READY" and (eta != 0 or color != -1):
                raise ValueError("READY arm must have ETA=0 and color=-1")
            if phase == "POST_PICK" and (eta == 0 or color == -1 or config["color_to_arm"][color] != i):
                raise ValueError("Invalid busy arm state")
        if not isinstance(row, list) or len(row) != 5:
            raise ValueError("Every Q row needs five values")
        if any(type(q) not in (int, float) or not math.isfinite(q) for q in row):
            raise ValueError("Q values must be finite numbers")
    return table


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Duplicate JSON key: {key}")
        result[key] = value
    return result


def load(path, config):
    table = json.loads(Path(path).read_text(), object_pairs_hook=_unique_object)
    if not isinstance(table, dict):
        raise ValueError("Q table must be an object")
    return validate(table, config)


def save(path, table, config):
    validate(table, config)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(table, indent=2, allow_nan=False) + "\n")
    tmp.replace(path)


def choose(table, model, stats=None):
    legal = model.legal_actions()
    if not legal:
        raise ValueError("No decision in terminal state")
    if stats is not None:
        stats["decision_steps"] = stats.get("decision_steps", 0) + 1
    row = table["q_values"].get(state_key(model.state()))
    if row is None:
        if stats is not None:
            stats["unknown_state_fallback_steps"] = stats.get("unknown_state_fallback_steps", 0) + 1
        return rule_action(model)
    best = max(row[a] for a in legal)
    candidates = [a for a in legal if row[a] == best]
    fallback = rule_action(model)
    return fallback if fallback in candidates else min(candidates)


def main():
    import argparse
    from .model import load_config
    parser = argparse.ArgumentParser(description="Validate an offline Q table against its exact config")
    parser.add_argument("--config", default="config/timing.placeholder.json")
    parser.add_argument("--table", default="artifacts/q_table.json")
    args = parser.parse_args()
    table = load(args.table, load_config(args.config))
    print(f"Valid schema: {len(table['q_values'])} states; deployment_ready=false")


if __name__ == "__main__":
    main()
