#!/bin/bash
set -e
source /home/yu/yuri_gazebo_performance/voice_batch_aliyun/activate.sh
source "$(dirname "$0")/../../install/setup.bash"
exec python "$(dirname "$0")/idle_scene.py"
