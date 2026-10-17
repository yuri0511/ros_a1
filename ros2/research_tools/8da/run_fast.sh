#!/usr/bin/env bash
# Local rule policy, validated 1ms physics; never touches the accepted workspace.
set -e
FAST_ROOT=/home/yu/yuri_gazebo_performance/fast_sim_8da998b
FAST_COUNT=${1:-1}
case "$FAST_COUNT" in 1|2|3) ;; *) echo 'Usage: run_fast.sh [1|2|3]'; exit 2;; esac
exec 9>"$FAST_ROOT/manual-launch.lock"
flock -n 9 || { echo 'Another manual fast-simulation run is active'; exit 1; }
unset PERF_CONTACT PERF_FIXTURE PERF_ISOLATION_ONLY PERF_EGL PERF_RMW FASTDDS_BUILTIN_TRANSPORTS LP_NUM_THREADS
export PERF_LP_THREADS=2
source "$FAST_ROOT/activate.sh"
exec python "$FAST_ROOT/tools/run_case.py" "manual-v3-${FAST_COUNT}instances" 1 true "$FAST_COUNT" 900
