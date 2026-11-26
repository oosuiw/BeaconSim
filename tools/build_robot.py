"""비콘 CAD(STL) → 로봇 메시 + 센서 장착 포즈. 실제 구현은 beacon_sim/robot_build.py."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from beacon_sim.robot_build import main  # noqa: E402

if __name__ == '__main__':
    main()
