#!/usr/bin/env bash
set -e
POLICY_ROOT=/home/yu/yuri_gazebo_performance/integration_72ab6f2
POLICY_MODE=${1:-rule}
POLICY_COUNT=${2:-1}
case "$POLICY_MODE" in rule|q) ;; *) echo 'Usage: run_policy.sh [rule|q] [1|2]'; exit 2;; esac
case "$POLICY_COUNT" in 1|2) ;; *) echo 'Only 1 or 2 instances allowed'; exit 2;; esac
exec 9>"$POLICY_ROOT/manual.lock"
flock -n 9 || exit 1
unset PERF_FIXTURE PERF_CONTACT PERF_EGL PERF_POLICY_PARAMS PERF_RMW PERF_ISOLATION_ONLY
export PERF_POLICY="$POLICY_MODE" PERF_LP_THREADS=2
if [[ "$POLICY_MODE" == q ]]; then export PERF_POLICY_PARAMS="$POLICY_ROOT/training/q-params.yaml"; fi
source "$POLICY_ROOT/activate.sh"
exec python "$POLICY_ROOT/tools/run_case.py" "manual-${POLICY_MODE}" 1 true "$POLICY_COUNT" 900
