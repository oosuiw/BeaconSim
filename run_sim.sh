#!/usr/bin/env bash
# 비콘 시뮬레이터 실행. 인자는 ros2 launch로 전달 (예: ./run_sim.sh camera:=false)
set -e
cd "$(dirname "$(readlink -f "$0")")"
source /opt/ros/humble/setup.bash
if [ ! -f robot/meshes/robot_parts.npz ]; then
  python3 tools/build_robot.py
fi
exec ros2 launch launch/sim.launch.py "$@"
