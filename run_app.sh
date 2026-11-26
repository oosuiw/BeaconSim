#!/usr/bin/env bash
# 고속도로 맵 생성기 GUI 실행
set -e
cd "$(dirname "$(readlink -f "$0")")"
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash   # 맵 검증(lanelet2)용, 없어도 실행됨
if [ ! -f robot/meshes/robot_parts.npz ]; then
  python3 tools/build_robot.py
fi
exec .venv/bin/python -m beacon_sim.app "$@"
