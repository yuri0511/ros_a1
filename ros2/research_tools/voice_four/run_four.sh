#!/usr/bin/env bash
set -eo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
WS=$(cd -- "$HERE/../.." && pwd)
PREFIX=${YURI_PIXI_PREFIX:-/home/yu/ros_a1/ros2/.pixi/envs/default}
[[ -x "$PREFIX/bin/python" ]] || { echo 'Existing Jazzy environment required.' >&2; exit 2; }
export CONDA_PREFIX="$PREFIX" PATH="$PREFIX/bin:$PATH" PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
export CMAKE_PREFIX_PATH="$PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
for activation in "$PREFIX"/etc/conda/activate.d/*.sh; do source "$activation"; done
[[ -f "$WS/install/setup.bash" ]] || { echo 'Build this worktree first.' >&2; exit 2; }
source "$WS/install/setup.bash"
exec python "$HERE/run_four.py" "$@"
