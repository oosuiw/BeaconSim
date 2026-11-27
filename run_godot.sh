#!/usr/bin/env bash
# BeaconSim (기본 GUI) — 파이썬 백엔드 + Godot 4 창
#   필요: third_party/godot/Godot_v4.x_linux.x86_64 (또는 BEACONSIM_GODOT)
#   옵션: --no-ros, --size 3440x1400, --mode dev|test|regress, --scenario X.yaml, --set env.weather=rain, --robot-stl CAD.stl
set -e
cd "$(dirname "$(readlink -f "$0")")"
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
exec .venv/bin/python -m beacon_sim.server "$@"
