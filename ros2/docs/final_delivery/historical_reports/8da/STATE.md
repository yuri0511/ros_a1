# Completed execution checkpoint

All requested experiments and final evidence export completed 2026-10-16T15:01:07.652975. No active coordinator or simulation. See REPORT.md and outputs deliverables.

{
  "base_commit": "8da998b963eed3c70fda076329f04a75c8627ef3",
  "initial_matched_pair_python_wall_saved_s": 22.98594641685486,
  "initial_matched_pair_wall_reduction_percent": 10.247396346291925,
  "fastest_observed_single_case": "B-repeat-1ms-event",
  "fastest_observed_single_s": 169.40284752845764,
  "fastest_observed_single_rtf": 1.0412430305103324,
  "recommended_instances": 2,
  "recommended_physics_ms": 1,
  "pooled_successful_batches_per_hour": 24.286580254222358,
  "including_cleanup_batches_per_hour": 23.655325621215976,
  "successful_batches_in_recommended_group": 4,
  "rounds_in_recommended_group": 2,
  "step2_qualified": false,
  "no_q_training": true
}

Full pinned worktree remains at worktree/ros2, branch perf/fast-simulation-8da998b, v3 current source. Original /home/yu/ros_a1 remains clean083a515; 135 protected hashes and original16-row CSV unchanged. q_feed.py, pixi.lock, original1ms world unchanged. No push or RL training.

A/B initial matched224.3→201.3s. Fastest v1 observation169.4s RTF1.041; currentv3 single175.9/184.3s RTF1.028, first has1standbyabort. v3 contact8/8 nointerarm but18gripperbase–part points max0.616mm observed14ms, not physically equivalent proof. 2ms C1/8 D0/8 rejected. v3dual299.2/293.7s,4/4batches32/32 correct, no controllerabort, pooled24.2866ep/h (inclcleanup23.6553). v3triple413.2s2/3successful, instance3 part3OUTSIDE,23/24correct,17.4253ep/h, rejecttriple.

All logs/rawcontacts preserved. EVIDENCE.zip in current task outputs contains lossless gzip copies where available; originals remain here. Original old083a experiments untouched. Reports copied to /home/yu/Documents/Codex/2026-10-16/files-pasted-by-the-user-ubuntu/outputs/fast_sim_8da998b/. Do not rerun historical edit/coordinator scripts on already-patched source. Current manual launcher run_fast.sh [1|2|3], recommendation2; leaves results isolated.
