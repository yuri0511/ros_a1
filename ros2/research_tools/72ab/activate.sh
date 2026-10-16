#!/bin/bash
export CONDA_PREFIX="/home/yu/ros_a1/ros2/.pixi/envs/default"
export PATH="$CONDA_PREFIX/bin:$PATH"
export CMAKE_PREFIX_PATH="$CONDA_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export PYTHONNOUSERSITE=1
export PYTHONDONTWRITEBYTECODE=1
for activation in "$CONDA_PREFIX"/etc/conda/activate.d/*.sh; do source "$activation"; done
