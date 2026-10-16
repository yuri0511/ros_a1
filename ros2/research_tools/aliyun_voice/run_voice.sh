#!/usr/bin/env bash
# Current demo safety entry; deliberately refuses before any Gazebo process.
set -eo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
WS=$(cd -- "$HERE/../.." && pwd)
PREFIX=${YURI_PIXI_PREFIX:-$WS/.pixi/envs/default}
[[ -x "$PREFIX/bin/python" ]] || { echo 'Set YURI_PIXI_PREFIX to the existing Jazzy environment.' >&2; exit 2; }
export CONDA_PREFIX="$PREFIX" PATH="$PREFIX/bin:$PATH" PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
export CMAKE_PREFIX_PATH="$PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
for activation in "$PREFIX"/etc/conda/activate.d/*.sh; do source "$activation"; done
# No simulation starts while the requested green3 path remains unsafe/unreviewed.
PYTHONPATH="$WS/src/a1_sorting_core" python - <<'PY'
from a1_sorting_core.voice_batch_plan import validate_candidate
from a1_sorting_core.voice_batch_safety import ensure_geometry_review
import json
plan=validate_candidate(json.dumps(dict(schema_version=1,action='sort_batch',counts=dict(blue=0,yellow=1,red=1,green=3),priority_first='yellow',proposed_order=['yellow','red','green','green','green'])))
ensure_geometry_review(plan)
PY
echo 'Physical demonstration supervisor is pending a separately reviewed green3 fix; do not bypass this gate.' >&2
exit 2
