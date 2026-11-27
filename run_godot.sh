#!/usr/bin/env bash
# BeaconSim (Godot 프런트엔드, 개발 중) — 파이썬 백엔드 + Godot 4 창
#   필요: third_party/godot/Godot_v4.x_linux.x86_64 (또는 BEACONSIM_GODOT), 로봇 모델(robot/meshes)
set -e
cd "$(dirname "$(readlink -f "$0")")"
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
exec .venv/bin/python -m beacon_sim.server "$@"
