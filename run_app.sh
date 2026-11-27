#!/usr/bin/env bash
# 기존 GUI (Dear ImGui + moderngl). 기본 GUI 는 ./run_godot.sh — Godot 이 없거나 문제가 있을 때 쓴다.
set -e
cd "$(dirname "$(readlink -f "$0")")"
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash   # 맵 검증(lanelet2)용, 없어도 실행됨
if [ ! -f robot/meshes/robot_parts.npz ]; then
  python3 tools/build_robot.py
fi
exec .venv/bin/python -m beacon_sim.app "$@"
