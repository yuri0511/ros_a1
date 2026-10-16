#!/usr/bin/env bash
# Explicit verified physics/control profile; never falls back to upstream gain2 or Q.
set -eo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
WS=$(cd -- "$HERE/../.." && pwd)
PREFIX=${YURI_PIXI_PREFIX:-$WS/.pixi/envs/default}
if [[ ! -x "$PREFIX/bin/python" ]]; then
  echo 'Set YURI_PIXI_PREFIX to the existing Jazzy Pixi environment (no installation performed).' >&2; exit 2
fi
export CONDA_PREFIX="$PREFIX" PATH="$PREFIX/bin:$PATH" PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
export CMAKE_PREFIX_PATH="$PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
for activation in "$PREFIX"/etc/conda/activate.d/*.sh; do [[ ! -f "$activation" ]] || source "$activation"; done
[[ -f "$WS/install/setup.bash" ]] || { echo 'Build ros2 first with colcon build --symlink-install --base-paths src' >&2; exit 2; }
source "$WS/install/setup.bash"
exec python "$HERE/run_rule.py" "$@"
