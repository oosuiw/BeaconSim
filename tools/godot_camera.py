"""Godot 카메라 시제품: 같은 시뮬레이션 순간을 기존 카메라(moderngl)와 Godot 으로 렌더해 비교 이미지를 만든다.

사용:
  .venv/bin/python tools/godot_camera.py --out results/godot --times 8 14 \\
      --set environment.time_of_day=night --set environment.weather=rain --set environment.intensity=0.7
  --live 20   : 비교 대신 20초 동안 실시간으로 Godot 창에 영상 송출 (카메라 주기)
출력: <out>/compare_<t>.png (위: 기존, 아래: Godot), godot_<t>.png (왜곡 적용), scene/ (glb·json)
"""
import argparse
import copy
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from beacon_sim.common import load_robot_parts, load_yaml, results_root  # noqa: E402
from beacon_sim.engine import SimEngine  # noqa: E402
from beacon_sim.godot_link import GodotLink, camera_T_b_opt, export_scene  # noqa: E402


def parse_set(cfg, items):
    for kv in items:
        k, v = kv.split('=', 1)
        grp, name = k.split('.')
        cur = cfg.setdefault(grp, {}).get(name)
        if isinstance(cur, bool):
            v = v.lower() in ('1', 'true', 'yes')
        elif isinstance(cur, (int, float)) and not isinstance(cur, bool):
            v = type(cur)(v)
        cfg[grp][name] = v


def label(img, text):
    out = img.copy()
    cv2.rectangle(out, (0, 0), (len(text) * 15 + 30, 46), (0, 0, 0), -1)
    cv2.putText(out, text, (14, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default=os.path.join(results_root(), 'godot', time.strftime('%Y%m%d_%H%M%S')))
    ap.add_argument('--times', type=float, nargs='*', default=[8.0])
    ap.add_argument('--set', action='append', default=[])
    ap.add_argument('--beacon-v', type=float, default=0.0, help='비콘 속도 명령 [m/s] (음수 = 후진)')
    ap.add_argument('--live', type=float, default=0.0)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    cfg = load_yaml('config/sim.yaml')
    cfg['map']['seed'] = cfg['map'].get('seed', 7)
    cfg['scenario']['seed'] = 103 if cfg['scenario'].get('seed', -1) < 0 else cfg['scenario']['seed']
    cfg['environment']['seed'] = 3 if cfg['environment'].get('seed', -1) < 0 else cfg['environment']['seed']
    parse_set(cfg, a.set)
    for k in ('pandar40p', 'tele15', 'ars408'):          # 카메라 비교에는 LiDAR·레이더 불필요 (속도)
        cfg[k]['enabled'] = False
    robot = load_yaml('config/robot.yaml')
    eng = SimEngine(copy.deepcopy(cfg), robot, log=lambda *x: None)
    print(f'환경 {eng.env.label()} · 사고 {eng.scn.summary()}', flush=True)

    # 기존 카메라 (moderngl EGL)
    import moderngl
    from beacon_sim.render import CameraRenderer, SceneRenderer
    ctx = moderngl.create_context(standalone=True, backend='egl')
    camr = CameraRenderer(ctx, cfg['camera'])
    scr = SceneRenderer(ctx, eng.model, load_robot_parts(), exclude_robot=('camera',), parts=eng.parts, env=eng.env)
    T_b_opt = camera_T_b_opt(robot)

    t0 = time.time()
    glb, js, origin = export_scene(eng, os.path.join(a.out, 'scene'), cfg['camera'], camr.Wr, camr.Hr, camr.K[1, 1])
    print(f'장면 내보내기 {time.time() - t0:.1f} s ({os.path.getsize(glb) / 1e6:.1f} MB)', flush=True)
    t0 = time.time()
    gl = GodotLink(js, origin, T_b_opt, kin=eng.kin)
    print(f'Godot 준비 {time.time() - t0:.1f} s', flush=True)
    try:
        if a.live > 0:
            nxt = 0.0
            while eng.t < a.live:
                eng.set_cmd('key', a.beacon_v, 0.0)
                eng.step()
                if eng.t >= nxt:
                    s = eng.snapshot()
                    gl.send_state(eng, s['T_wb'], s['traffic_T'], s['active'])
                    nxt = eng.t + 1.0 / cfg['camera']['rate_hz']
                    time.sleep(1.0 / cfg['camera']['rate_hz'])
            return
        for tc in sorted(a.times):
            while eng.t < tc:
                if a.beacon_v:
                    eng.set_cmd('key', a.beacon_v, 0.0)
                eng.step()
            s = eng.snapshot()
            T_wb = s['T_wb']
            veh = list(eng.accident) + list(zip(eng.traffic.specs, s['traffic_T']))
            t1 = time.time()
            ours = camr.render(scr, T_wb @ T_b_opt, T_wb, veh, eng.env)
            t_ours = time.time() - t1
            raw = os.path.join(a.out, f'godot_raw_{tc:g}.png')
            t1 = time.time()
            rep_ = gl.send_state(eng, T_wb, s['traffic_T'], s['active'], capture=os.path.abspath(raw))
            t_godot = time.time() - t1
            g = cv2.imread(raw)[..., ::-1]
            g = cv2.remap(g, camr.map_x, camr.map_y, cv2.INTER_LINEAR)          # 같은 렌즈 왜곡
            cv2.imwrite(os.path.join(a.out, f'godot_{tc:g}.png'), g[..., ::-1])
            cmp_ = np.vstack([label(ours, f'moderngl (now)  t={tc:g}s  {eng.env.time_of_day}/{eng.env.weather} {eng.env.intensity:.1f}'),
                              label(g, 'Godot 4 Forward+ (prototype)')])
            p = os.path.join(a.out, f'compare_{tc:g}.png')
            cv2.imwrite(p, cv2.resize(cmp_, (cmp_.shape[1] // 2, cmp_.shape[0] // 2), interpolation=cv2.INTER_AREA)[..., ::-1])
            print(f't={tc:g}s  기존 {t_ours * 1000:.0f} ms · Godot 캡처 {t_godot:.1f} s (안정화 대기 포함), Godot {rep_.get("fps")} fps → {p}', flush=True)
    finally:
        gl.close()
        eng.close()


if __name__ == '__main__':
    main()
