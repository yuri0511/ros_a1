#!/bin/bash
set -e
source /home/yu/yuri_gazebo_performance/voice_batch_aliyun/activate.sh
export QT_QPA_PLATFORM=xcb
exec python "$(dirname "$0")/app.py" "$@"
