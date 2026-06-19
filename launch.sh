#!/bin/bash
# WSL → Windows kısayolundan çağrılır

export DISPLAY="${DISPLAY:-:0}"
export XDG_RUNTIME_DIR="/run/user/$(id -u)"

source /opt/ros/humble/setup.bash
source /home/emin/ros2-end-effector/install/setup.bash

python3 /home/emin/ros2-end-effector/start_robot.py 2>&1 | \
    tee /tmp/end_effector_launch.log
