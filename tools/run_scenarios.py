"""시나리오 회귀 테스트 실행기.

사용:
  .venv/bin/python tools/run_scenarios.py scenarios/            # 전체 (ROS 없이, 실시간보다 빠르게)
  .venv/bin/python tools/run_scenarios.py scenarios/S03_tunnel_entry.yaml --out results/
  source /opt/ros/humble/setup.bash && .venv/bin/python tools/run_scenarios.py scenarios/ --ros
      → ROS2 토픽 발행 + /cmd_vel 수신 (상위 제어기 연동), 실시간 진행, 측위/차선검출 토픽 평가

시나리오 YAML: name, description, duration, map/scenario/traffic/environment/gnss (sim.yaml 덮어쓰기),
  faults: [{sensor, t0, t1}], beacon_script: [{t0, t1, v, w}] (ROS 상위 제어기가 없을 때 비콘 명령),
  topics: {localization: <토픽>, lane_detection: <PointCloud2 토픽>}, criteria: {기준: 값}
결과: <out>/report.md, report.json, <시나리오>/timeseries.csv. 하나라도 FAIL이면 종료 코드 1.
"""
import argparse
import copy
import csv
import glob
import json
import os
import sys
import time

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from beacon_sim.common import ROOT, load_yaml  # noqa: E402
from beacon_sim.engine import EngineThread, SimEngine  # noqa: E402
from beacon_sim.evaluate import Metrics, judge  # noqa: E402

COLS = ['t', 'x', 'y', 's', 't_lat', 'v', 'vehicles', 'noticed', 'lane_change', 'brake', 'scc_lock', 'crash',
        'beacon_hit', 'damaged', 'gnss_status']


def deep_merge(a, b):
    for k, v in (b or {}).items():
        if isinstance(v, dict) and isinstance(a.get(k), dict):
            deep_merge(a[k], v)
        else:
            a[k] = v
    return a


def script_cmd(script, t):
    for seg in script or []:
        if seg.get('t0', 0) <= t < seg.get('t1', 1e9):
            return seg.get('v', 0.0), seg.get('w', 0.0)
    return None


def run_one(sc, base, robot, ros=None, log=print):
    cfg = deep_merge(copy.deepcopy(base), {k: sc[k] for k in ('map', 'scenario', 'traffic', 'environment', 'gnss')
                                           if k in sc})
    cfg['faults'] = sc.get('faults', [])
    eng = SimEngine(cfg, robot, log=log)
    met = Metrics(eng)
    dur = float(sc.get('duration', 120))
    script = sc.get('beacon_script')
    t0 = time.time()
    info = {'map_seed': eng.model.params.seed, 'scenario_seed': eng.scn.sp.seed, 'environment': eng.env.label(),
            'scenario': eng.scn.summary()}
    if ros is None:
        pending = {}
        while eng.t < dur:
            c = script_cmd(script, eng.t)
            if c is not None:
                eng.set_cmd('key', *c)
            fired, ev = eng.step()
            pending.update(fired)
            if eng.tick % 5 == 0 or ev:
                met.update(pending, ev)
                pending = {}
    else:
        node, bridge_cls, rclpy = ros
        bridge = bridge_cls(node, eng, cfg, markers=False)
        topics = sc.get('topics', {}) or {}
        subs = _subscribe_eval(node, met, eng, topics)
        state = {'pending': {}}

        def on_step(e, fired, ev):
            bridge.on_step(e, fired, ev)
            c = script_cmd(script, e.t)
            if c is not None:
                e.set_cmd('key', *c)
            state['pending'].update(fired)
            if e.tick % 5 == 0 or ev:
                met.update(state['pending'], ev)
                state['pending'] = {}
        th = EngineThread(eng, cfg['sim']['real_time_factor'], on_step)
        th.start()
        while eng.t < dur and rclpy.ok():
            time.sleep(0.2)
        th.stop()
        for s in subs:
            node.destroy_subscription(s)
    res = met.result()
    res['wall_s'] = round(time.time() - t0, 1)
    res['speedup'] = round(dur / max(res['wall_s'], 1e-3), 1)
    eng.close()
    return info, res, met.samples


def _subscribe_eval(node, met, eng, topics):
    """측위(Odometry/PoseStamped/PoseWithCovarianceStamped)·차선검출(PointCloud2) 토픽 구독."""
    import math
    subs = []
    loc = topics.get('localization')
    if loc:
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
        from nav_msgs.msg import Odometry

        def pose_of(m):
            p = m.pose.pose if hasattr(m.pose, 'pose') else m.pose
            q = p.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            met.add_localization(eng.t, p.position.x, p.position.y, yaw)
        typ = {'odom': Odometry, 'pose': PoseStamped, 'pose_cov': PoseWithCovarianceStamped}[topics.get('localization_type', 'odom')]
        subs.append(node.create_subscription(typ, loc, pose_of, 10))
    lane = topics.get('lane_detection')
    if lane:
        import numpy as np
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs_py import point_cloud2

        def lane_cb(m):
            gt = eng.latest.get('pandar40p', {}).get('gt')
            if gt is None:
                return
            a = point_cloud2.read_points(m, field_names=['x', 'y'])
            met.add_lane_detection(np.c_[a['x'], a['y']], gt)
        subs.append(node.create_subscription(PointCloud2, lane, lane_cb, 5))
    return subs


def fmt(v):
    if isinstance(v, float):
        return f'{v:g}'
    return '-' if v is None else str(v)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('paths', nargs='+')
    ap.add_argument('--out', default=os.path.join(ROOT, 'results', time.strftime('%Y%m%d_%H%M%S')))
    ap.add_argument('--ros', action='store_true', help='ROS2 발행/구독 + 실시간 (상위 제어기·측위·차선검출 연동)')
    ap.add_argument('--config', default=os.path.join(ROOT, 'config', 'sim.yaml'))
    a = ap.parse_args()
    files = []
    for p in a.paths:
        files += sorted(glob.glob(os.path.join(p, '*.yaml'))) if os.path.isdir(p) else [p]
    base, robot = load_yaml(a.config), load_yaml(os.path.join(ROOT, 'config', 'robot.yaml'))
    ros = None
    if a.ros:
        import rclpy
        from rclpy.node import Node
        from beacon_sim.ros_bridge import RosBridge
        rclpy.init()
        node = Node('beacon_scenario_runner')
        import threading
        threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
        ros = (node, RosBridge, rclpy)
    os.makedirs(a.out, exist_ok=True)
    report, all_ok = [], True
    for f in files:
        with open(f) as fh:
            sc = yaml.safe_load(fh)
        name = sc.get('name', os.path.splitext(os.path.basename(f))[0])
        print(f'▶ {name}: {sc.get("description", "")}', flush=True)
        info, res, samples = run_one(sc, base, robot, ros, log=lambda *x: None)
        rows, ok = judge(res, sc.get('criteria'))
        all_ok &= ok
        d = os.path.join(a.out, name)
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, 'timeseries.csv'), 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(COLS)
            w.writerows(samples)
        entry = {'name': name, 'file': f, 'description': sc.get('description', ''), 'pass': ok, 'info': info,
                 'metrics': res, 'criteria': [dict(zip(['criterion', 'expected', 'actual', 'status'], r)) for r in rows]}
        with open(os.path.join(d, 'result.json'), 'w') as fh:
            json.dump(entry, fh, ensure_ascii=False, indent=1, default=str)
        report.append(entry)
        fails = [r for r in rows if r[3] == 'FAIL']
        print(f'  {"PASS" if ok else "FAIL"}  ({res["wall_s"]} s, x{res["speedup"]})  추돌 {res["crashes"]} 비콘추돌 '
              f'{res["beacon_hits"]} 최소TTC {fmt(res["min_ttc_s"])} 인지 {res["noticed"]} SCC {res["scc_locks"]} '
              f'GNSS FIX {res["gnss_fix_ratio"]:.0%}' + (f'  ← {", ".join(r[0] for r in fails)}' if fails else ''),
              flush=True)
    with open(os.path.join(a.out, 'report.json'), 'w') as fh:
        json.dump(report, fh, ensure_ascii=False, indent=1, default=str)
    keys = ['crashes', 'beacon_hits', 'min_ttc_s', 'noticed', 'notice_dist_mean_m', 'lane_changes', 'scc_locks',
            'pass_speed_max_kph', 'beacon_damaged', 'beacon_contacts', 'gnss_fix_ratio', 'gnss_reconverge_max_s']
    md = ['# 시나리오 회귀 테스트 결과', '', f'- 실행: {time.strftime("%Y-%m-%d %H:%M:%S")}  모드: {"ROS 연동" if a.ros else "단독(고속)"}',
          f'- 결과: {"전체 통과" if all_ok else "실패 있음"} ({sum(r["pass"] for r in report)}/{len(report)})', '',
          '| 시나리오 | 결과 | 환경 | ' + ' | '.join(keys) + ' |', '|' + '---|' * (len(keys) + 3)]
    for r in report:
        md.append(f'| {r["name"]} | {"✅" if r["pass"] else "❌"} | {r["info"]["environment"]} | '
                  + ' | '.join(fmt(r['metrics'].get(k)) for k in keys) + ' |')
    md += ['', '## 기준 판정', '']
    for r in report:
        md.append(f'### {r["name"]} — {r["description"]}')
        md.append(f'시드: 맵 {r["info"]["map_seed"]}, 시나리오 {r["info"]["scenario_seed"]} · 사고 {r["info"]["scenario"]}')
        md.append('')
        md.append('| 기준 | 기대 | 실제 | 판정 |')
        md.append('|---|---|---|---|')
        for c in r['criteria']:
            md.append(f'| {c["criterion"]} | {fmt(c["expected"])} | {fmt(c["actual"])} | {c["status"]} |')
        md.append('')
    with open(os.path.join(a.out, 'report.md'), 'w') as fh:
        fh.write('\n'.join(md))
    print(f'\n리포트: {os.path.join(a.out, "report.md")}  → {"전체 통과" if all_ok else "실패 있음"}')
    if ros is not None:
        ros[0].destroy_node()
        ros[2].shutdown()
    sys.exit(0 if all_ok else 1)


if __name__ == '__main__':
    main()
