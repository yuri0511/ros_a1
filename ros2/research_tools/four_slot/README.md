# Four-slot candidate — blocked before dynamic execution

This is an isolated research candidate, not a validated replacement for the eight-part workstation.
The candidate JSON deliberately has `motion_validated: false`. Do not flip it to bypass review.

- `generate.py --spread-x`: generate the 240 mm candidate, 16 unique slots and 48 numerical IK targets.
- `plan_only.launch.py`: MoveIt only, trajectory execution disabled; requires the exact URDF retained in the delivery audit directory.
- `plan_audit.py`: 80 real Pilz planning requests with three nominal stored cubes and static workstation collision geometry.
- `sweep.py`: 2 ms command spline SAT sampling with explicit pose/opening assumptions. This does not measure physical tracking or contacts.
- `release_audit.py`: static sensitivity showing why bounded release opening matters.
- `check_gate.py`: report blockers, never starts a robot or simulator.

The parent delivery directory contains the nine Chinese reports, raw planning requests/results, test logs, configuration hashes and an explicitly labelled offline layout image.
Runtime retained-object planning scene updates, measured held-object orientation, full pickup-to-placement trajectory coverage and continuous release-opening verification remain unfinished. The config overlay is a candidate template, not a complete launch profile.

2026-10-17 controlled calibration update: see `docs/four_slot_candidate/CALIBRATION_PROGRESS_CN.md`.
`calibrate_idle.py` is an isolated, bounded simulation calibration tool, not a formal batch launcher.
Measured red-slot release/exit passed in a limited fixture test; loaded lift planning is blocked
by measured cube/support-floor contact. The formal gate and exit inhibit remain closed.
