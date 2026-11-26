"""헤드리스 ROS2 시뮬레이터 노드 (GUI 없이). SimEngine + RosBridge.

비콘은 상위 제어기의 /cmd_vel 로만 움직인다. GUI가 필요하면 run_app.sh (같은 엔진 + 시각화 + 키보드 조작).
"""
import argparse
import time

import rclpy
from rclpy.node import Node

from .common import load_yaml
from .engine import EngineThread, SimEngine
from .ros_bridge import RosBridge


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', default='config/sim.yaml')
    ap.add_argument('--robot', default='config/robot.yaml')
    ap.add_argument('--markers', action='store_true', help='RViz용 마커도 발행')
    ap.add_argument('--no-static-tf', action='store_true', help='/tf_static 발행 안 함 (robot_state_publisher와 같이 쓸 때)')
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = Node('beacon_sim_core')
    log = node.get_logger()
    cfg = load_yaml(args.config)
    eng = SimEngine(cfg, load_yaml(args.robot), log=log.info)
    bridge = RosBridge(node, eng, cfg, markers=args.markers, static_tf=not args.no_static_tf)
    sm = eng.scn.summary()
    log.info(f'맵 시드 {eng.model.params.seed}, 시나리오 시드 {eng.scn.sp.seed} | {sm["side"]} {sm["lane"]}차로, '
             f'사고 s={sm["accident_s"]} m, 비콘 배치 s={sm["beacon_start_s"]} m (역방향, /cmd_vel 대기) | '
             f'환경 {eng.env.label()} | 교통 {"켜짐" if eng.traffic.tp.enabled else "꺼짐(검증 모드)"}')
    th = EngineThread(eng, cfg['sim']['real_time_factor'], bridge.on_step)
    th.start()

    def report():
        c = eng.traffic.counts
        s = ', '.join(f'{k} {v:.1f}ms' for k, v in eng.take_stats().items())
        log.info(f'sim {eng.t:.1f}s RTF {th.measured_rtf:.2f} | 차량 {int(eng.traffic.active.sum())} 인지 {c["noticed"]} '
                 f'차로변경 {c["lane_change"]} 감속 {c["brake"]} SCC인식 {c["scc_lock"]} 추돌 {c["crash"]}'
                 f'(비콘 {c["beacon_hit"]}) | {s}')
    node.create_timer(10.0, report)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        th.stop()
        eng.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
