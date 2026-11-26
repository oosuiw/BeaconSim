"""ROS 없이 씬 + 센서 3종을 한 번씩 돌려 출력 통계를 확인한다.

사용: .venv/bin/python tools/smoke_test.py
"""
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from beacon_sim import rgl, scene  # noqa: E402
from beacon_sim.common import load_robot_parts, load_yaml, sensor_mounts  # noqa: E402
from beacon_sim.roadgen import RoadParams, generate  # noqa: E402
from beacon_sim.traffic import Scenario, ScenarioParams, TrafficParams, TrafficSim, resolve_seed  # noqa: E402
from beacon_sim.sensors import ARS408, ID_TO_CLASS, Pandar40P, Tele15, entity_id  # noqa: E402


def main():
    cfg, rcfg = load_yaml('config/sim.yaml'), load_yaml('config/robot.yaml')
    rgl.configure_logging(rgl.LOG_WARN)
    cfg['map']['seed'] = resolve_seed(cfg['map']['seed'])
    cfg['scenario']['seed'] = resolve_seed(cfg['scenario']['seed'])
    L = generate(RoadParams(**cfg['map']))
    keep = []
    for k, p in enumerate(scene.build_static(L)):
        keep.append(rgl.Entity(rgl.Mesh(p.v, p.f), entity_id=entity_id(p.cls, k), laser_retro=p.retro))
    scn = Scenario(L, ScenarioParams(**cfg['scenario']))
    tr = TrafficSim(L, scn, TrafficParams(**cfg['traffic']), cfg['scenario']['seed'])
    npc_e = [rgl.Entity(rgl.Mesh(*scene.vehicle_mesh(sp.kind, sp.length, sp.width, sp.height)),
                        entity_id=entity_id('vehicle', i), laser_retro=0.25) for i, sp in enumerate(tr.specs)]
    for k, (sp, T) in enumerate(scn.accident_poses()):
        e = rgl.Entity(rgl.Mesh(*scene.vehicle_mesh(sp.kind, sp.length, sp.width, sp.height)),
                       entity_id=entity_id('accident', k), laser_retro=0.25)
        e.set_transform(T)
        keep.append(e)
    ign = {'pandar40p': 1001, 'tele15': 1002, 'ars408': 1003}
    rob = [rgl.Entity(rgl.Mesh(v, f), entity_id=entity_id('robot', k), laser_retro=0.3, ignored_by_sensor=ign.get(nm))
           for k, (nm, (v, f)) in enumerate(load_robot_parts().items())]
    T_wb = scn.beacon_pose(scn.beacon_s0)
    mounts = sensor_mounts(rcfg)
    pandar, tele, radar = Pandar40P(cfg['pandar40p']), Tele15(cfg['tele15']), ARS408(cfg['ars408'])
    kinds = {i: sp.kind for i, sp in enumerate(tr.specs)}
    for step in range(3):
        t = 1.0 + 0.1 * step
        rgl.set_scene_time(int(t * 1e9))
        for e in rob:
            e.set_transform(T_wb)
        tr.step(scn.beacon_s0, scn.lane_t)
        for T, e in zip(tr.poses_at()[0], npc_e):
            e.set_transform(T)
        t0 = time.perf_counter()
        p, gt = pandar.run(T_wb @ mounts['pandar40p'][1])
        t1 = time.perf_counter()
        lv = tele.run(T_wb @ mounts['tele15'][1], int(t * 1e9))
        t2 = time.perf_counter()
        c = radar.run(T_wb @ mounts['ars408'][1], npc_kind=kinds)
        t3 = time.perf_counter()
    print(f'Pandar40P: {len(p)} pts ({1e3 * (t1 - t0):.1f} ms)  채널 {p["channel"].min()}~{p["channel"].max()}  '
          f'거리 {p["distance"].min():.2f}~{p["distance"].max():.1f} m')
    labs, cnt = np.unique(gt['label'], return_counts=True)
    print('  라벨 분포:', {ID_TO_CLASS.get(int(a), a): int(b) for a, b in zip(labs, cnt)})
    for lab in ['road', 'lane_white', 'lane_yellow', 'guardrail']:
        m = gt['label'] == scene.CLASS[lab]
        if m.any():
            d = np.hypot(gt['x'][m], gt['y'][m])
            print(f'  {lab:12s} intensity 평균 {gt["intensity"][m].mean():5.1f}, 최대 거리 {d.max():6.1f} m')
    rb = gt['label'] == scene.CLASS['robot']
    print(f'  로봇 자체 메시 히트: {rb.sum()} 점')
    print(f'Tele-15: {len(lv)} pts ({1e3 * (t2 - t1):.1f} ms)  x {lv["x"].min():.1f}~{lv["x"].max():.1f} m, '
          f'az ±{np.degrees(np.abs(np.arctan2(lv["y"], lv["x"])).max()):.1f}°')
    print(f'ARS408: {len(c)} clusters ({1e3 * (t3 - t2):.1f} ms)  far {int(c["far"].sum())}')
    labs, cnt = np.unique(c['label'], return_counts=True)
    print('  라벨 분포:', {ID_TO_CLASS.get(int(a), a): int(b) for a, b in zip(labs, cnt)})
    v = c[c['label'] == scene.CLASS['vehicle']]
    if len(v):
        print('  차량 클러스터 예: d=%s vr=%s' % (np.round(v['distance'][:5], 1), np.round(v['radial_speed'][:5], 1)))


if __name__ == '__main__':
    main()
