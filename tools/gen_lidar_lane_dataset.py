#!/usr/bin/env python3
"""라이다 차선 검출 학습 데이터 생성기 (화면 없음, RGL 만) — K-Lane 라벨 형식.

맵마다 도로·날씨·교통을 무작위로 바꾸고, 비콘을 맵 위 무작위 지점(차로·차로 경계·갓길, 정/역방향, 각도 흔들림)에
놓은 뒤 짧은 연속 스캔(클립)을 저장한다. 비콘은 대부분 서 있고 일부는 천천히 움직인다 (스캔 누적 학습용 연속 프레임).

출력: <out>/<worker>/<맵>_<클립>.npz
  pts_<k>   (N,4) float32  x, y, z, intensity — 라이다 좌표계 (ROI + 여유만)
  T_<k>     (4,4) float64  map ← 라이다 (누적할 때 이전 스캔을 현재 좌표로 옮기는 데 씀)
  cls_<k>   (144,144) uint8  칸별로 무엇이 있나 (비트: 1 노면, 2 도색, 4 가드레일·분리대·터널벽·기둥, 8 흙(도로 밖), 16 차량·보행자)
            [h, w] = 디코드 좌표 (h: 가까운 곳부터 0.32 m, w: 오른쪽(-y)부터 0.16 m) — 오검출 위치 분석용
  label_<k> (144,144) uint8 K-Lane 원본 라벨 형식: [0,0] = 가장 먼 왼쪽, 값 0~5 = 차선 슬롯, 255 = 없음
            슬롯: 비콘 기준 왼쪽 선들 2,1,0 (가까운 순), 오른쪽 선들 3,4,5 (가까운 순)
  meta      JSON 문자열 (맵·환경·자세)
사용:
  python tools/gen_lidar_lane_dataset.py --out /data/klane_sim --workers 8 --gpus 0,1,2,3 --maps 400 --clips 40
"""
import argparse
import copy
import json
import multiprocessing as mp
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

ROI_X, ROI_Y = (0.0, 46.08), (-11.52, 11.52)
CELL_X, CELL_Y = 0.32, 0.16
KEEP = dict(x=(-2.0, 48.0), y=(-13.0, 13.0), z=(-2.5, 2.0))       # 저장할 점 (ROI + 여유, 증강용)


def rasterize(lines):
    """{line_id: (N,2) x,y (라이다 좌표, x 오름차순)} → K-Lane 원본 라벨 (144,144) uint8"""
    lab = np.full((144, 144), 255, np.uint8)
    refs = []
    for lid, P in lines.items():
        if len(P) < 2:
            continue
        xr = float(np.clip(5.0, P[0, 0], P[-1, 0]))
        refs.append((float(np.interp(xr, P[:, 0], P[:, 1])), lid))
    left = sorted([r for r in refs if r[0] > 0], key=lambda r: r[0])[:3]
    right = sorted([r for r in refs if r[0] <= 0], key=lambda r: -r[0])[:3]
    slot = {lid: s for s, (_, lid) in zip((2, 1, 0), left)}
    slot.update({lid: s for s, (_, lid) in zip((3, 4, 5), right)})
    xc = (np.arange(144) + 0.5) * CELL_X                       # 행 h 중심 (가까운 곳부터)
    for lid, s in slot.items():
        P = lines[lid]
        ok = (xc >= P[0, 0]) & (xc <= P[-1, 0])
        y = np.interp(xc, P[:, 0], P[:, 1])
        col = np.floor((y - ROI_Y[0]) / CELL_Y).astype(int)
        ok &= (col >= 0) & (col < 144)
        h = np.nonzero(ok)[0]
        lab[143 - h, 143 - col[h]] = s                         # 원본 형식: 위 = 먼 곳, 왼쪽 = +y
    return lab


CLS_BITS = {1: 1, 2: 2, 3: 2, 4: 4, 5: 4, 7: 4, 10: 4, 11: 4, 6: 8, 8: 16, 12: 16, 13: 16, 14: 16, 15: 16, 16: 16}


def cell_classes(gt):
    """라이다 정답 점(라벨) → 칸별 비트 (144,144) uint8, 디코드 좌표"""
    out = np.zeros((144, 144), np.uint8)
    h = np.floor(gt['x'] / CELL_X).astype(int)
    w = np.floor((gt['y'] - ROI_Y[0]) / CELL_Y).astype(int)
    ok = (h >= 0) & (h < 144) & (w >= 0) & (w < 144)
    lut = np.zeros(256, np.uint8)
    for k, v in CLS_BITS.items():
        lut[k] = v
    bits = lut[np.clip(gt['label'][ok], 0, 255).astype(int)]
    np.bitwise_or.at(out, (h[ok], w[ok]), bits)
    return out


def lane_lines(lgt, s_b, T_ws):
    """정답 차선 → {line_id: (N,2)} 라이다 좌표, ROI 안, x 오름차순 (뒤로 꺾이는 구간은 단조 부분만)"""
    a = lgt.window(s_b, 70.0, 70.0, T_ws)
    out = {}
    for lid in np.unique(a['line_id']):
        b = a[a['line_id'] == lid]
        P = np.c_[b['x'], b['y']].astype(float)
        m = (P[:, 0] > ROI_X[0] - 1) & (P[:, 0] < ROI_X[1] + 1) & (P[:, 1] > ROI_Y[0] - 1) & (P[:, 1] < ROI_Y[1] + 1)
        P = P[m]
        if len(P) < 2:
            continue
        P = P[np.argsort(P[:, 0])]
        keep = np.r_[True, np.diff(P[:, 0]) > 1e-3]
        out[int(lid)] = P[keep]
    return out


def rand_cfg(base, rng, seed):
    cfg = copy.deepcopy(base)
    m = cfg['map']
    m.update(seed=int(seed), length=2500.0, lanes=int(rng.choice([2, 3, 4], p=[0.45, 0.4, 0.15])),
             design_speed=int(rng.choice([80, 100, 120], p=[0.25, 0.4, 0.35])), tunnels=int(rng.integers(0, 3)),
             gantries=int(rng.integers(0, 4)), grade_max=float(rng.uniform(0.02, 0.06)),
             wear=float(rng.uniform(0.0, 0.7)), lane_width=float(rng.uniform(3.2, 3.8)),
             inner_shoulder=float(rng.uniform(0.5, 1.5)), outer_shoulder=float(rng.uniform(0.5, 3.6)),
             outer_rail=str(rng.choice(['guardrail', 'concrete', 'mixed'], p=[0.35, 0.3, 0.35])),
             roadside_clutter=float(rng.choice([0.0, rng.uniform(0.2, 1.0)], p=[0.25, 0.75])),
             line_width=float(rng.uniform(0.12, 0.2)), dash_len=float(rng.choice([8.0, 10.0])),
             dash_gap=float(rng.choice([8.0, 10.0, 12.0])))
    w = str(rng.choice(['clear', 'rain', 'fog', 'snow'], p=[0.55, 0.2, 0.15, 0.1]))
    cfg['environment'].update(seed=int(seed), weather=w, time_of_day='day', intensity=float(rng.uniform(0.2, 1.0)))
    cfg['traffic'].update(enabled=True, flow=float(rng.uniform(200, 1800)), flow_opposite=float(rng.uniform(200, 1500)),
                          truck_ratio=float(rng.uniform(0.05, 0.4)))
    cfg['scenario'].update(seed=int(seed), accident_type='random', pedestrians=-1)
    cfg['tele15']['enabled'] = False
    cfg['ars408']['enabled'] = False
    return cfg


def rand_robot(base, rng):
    """라이다 장착 자세 흔들림 (높이 ±3 cm, 롤·피치 ±1.5°) — 실기 조립 오차·차체 기울기 (실기 노면 높이 편차 ±8 cm)"""
    r = copy.deepcopy(base)
    s = r['sensors']['pandar40p']
    s['xyz'] = [s['xyz'][0], s['xyz'][1], s['xyz'][2] + float(rng.uniform(-0.03, 0.03))]
    s['rpy'] = [s['rpy'][0] + float(np.radians(rng.uniform(-1.5, 1.5))),
                s['rpy'][1] + float(np.radians(rng.uniform(-1.5, 1.5))), s['rpy'][2]]
    return r


def place(e, rng):
    """비콘을 무작위 지점에: 차로·경계·갓길 (중앙분리대 바깥), 정/역방향, 각도 흔들림, 센서타워 펴짐"""
    m = e.model
    s = float(rng.uniform(80.0, m.length - 80.0))
    side = float(rng.choice([-1, 1]))
    t_lo, t_hi = m.params.barrier_half + 0.4, m.edge_t - 0.4
    t = side * float(rng.uniform(t_lo, t_hi))
    P0, P1 = np.asarray(m.point(s, t), float), np.asarray(m.point(s + 1.0, t), float)
    head = float(np.arctan2(P1[1] - P0[1], P1[0] - P0[0]))
    if rng.random() < 0.5:
        head += np.pi
    e.x, e.y = float(P0[0]), float(P0[1])
    e.yaw = head + float(np.radians(rng.normal(0.0, 4.0)))
    e.mast_q = e.mast_target = e.kin.q_dep
    e.tower_auto_done = True
    e.damaged = False
    e.traffic.beacon_hits.clear()                      # 예전 추돌 기록이 남아 있으면 옮기자마자 다시 파손 처리됨
    e.T_wb, e.s_b, e.t_b = e._ground_pose()
    v = 0.0 if rng.random() < 0.7 else float(rng.uniform(0.3, 2.0))
    e.cmd['key'] = (v, float(rng.normal(0, 0.05)) if v else 0.0, 1e12)     # 시간 초과 없는 키보드 명령 = 일정 속도


def worker(wid, gpu, args):
    os.environ['CUDA_VISIBLE_DEVICES'] = str(gpu)
    import yaml
    from beacon_sim.engine import SimEngine
    from beacon_sim.gt import LaneGT
    base = yaml.safe_load(open(os.path.join(ROOT, 'config/sim.yaml')))
    rob = yaml.safe_load(open(args.robot))
    out = os.path.join(args.out, f'w{wid:02d}')
    os.makedirs(out, exist_ok=True)
    rng = np.random.default_rng(args.seed * 1000 + wid)
    n_done, t0 = 0, time.time()
    for mi in range(args.maps):
        seed = args.seed * 100000 + wid * 1000 + mi
        if os.path.exists(os.path.join(out, f'm{seed}_c{args.clips - 1:03d}.npz')):
            continue                                            # 이어하기
        cfg = rand_cfg(base, rng, seed)
        try:
            e = SimEngine(cfg, rand_robot(rob, rng), log=lambda *a: None)
        except Exception as ex:      # noqa: BLE001
            print(f'[w{wid}] 맵 {seed} 생성 실패: {ex}', flush=True)
            continue
        lgt = LaneGT(e.model, 0.25)
        e.next_fire['pandar40p'] = 1e9
        for _ in range(int(args.warmup / e.DT)):               # 교통 예열 (라이다 없이)
            e.step()
        ci, tries = 0, 0
        while ci < args.clips and tries < args.clips * 3:
            tries += 1
            place(e, rng)
            for _ in range(int(rng.uniform(0.5, 3.0) / e.DT)):  # 교통이 새 비콘 위치에 반응할 시간
                e.step()
            e.next_fire['pandar40p'] = e.t
            clip, k = {}, 0
            while k < args.frames and not e.damaged:
                fired, _ = e.step()
                f = fired.get('pandar40p')
                if f is None:
                    continue
                p = f['points']
                xyz = np.c_[p['x'], p['y'], p['z']]
                m = ((xyz[:, 0] > KEEP['x'][0]) & (xyz[:, 0] < KEEP['x'][1]) & (xyz[:, 1] > KEEP['y'][0])
                     & (xyz[:, 1] < KEEP['y'][1]) & (xyz[:, 2] > KEEP['z'][0]) & (xyz[:, 2] < KEEP['z'][1]))
                clip[f'pts_{k}'] = np.c_[xyz[m], p['intensity'][m]].astype(np.float32)
                clip[f'T_{k}'] = f['T_ws']
                clip[f'label_{k}'] = rasterize(lane_lines(lgt, e.s_b, f['T_ws']))
                clip[f'cls_{k}'] = cell_classes(f['gt'])
                k += 1
            e.next_fire['pandar40p'] = 1e9
            if e.damaged or k < args.frames:                     # 차량이 비콘을 들이받음 (밀려나고 돌아감) → 버리고 다시
                continue
            meta = dict(map_seed=seed, lanes=cfg['map']['lanes'], lane_width=cfg['map']['lane_width'],
                        weather=e.env.weather, intensity=e.env.intensity, s=e.s_b, t=e.t_b,
                        in_tunnel=bool(e.model.in_tunnel(e.s_b)), v=e.cmd['key'][0], frames=args.frames)
            np.savez(os.path.join(out, f'm{seed}_c{ci:03d}.npz'), meta=json.dumps(meta), **clip)
            n_done += args.frames
            ci += 1
        e.close()
        el = time.time() - t0
        print(f'[w{wid}] 맵 {mi + 1}/{args.maps} 완료, 누적 {n_done} 프레임, {n_done / max(el, 1e-9):.1f} 프레임/s', flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--robot', default=os.path.join(ROOT, 'config/robot.yaml'))
    ap.add_argument('--workers', type=int, default=4)
    ap.add_argument('--gpus', default='0', help='쉼표 구분 GPU 번호 (작업자를 돌아가며 배정)')
    ap.add_argument('--maps', type=int, default=10, help='작업자당 맵 수')
    ap.add_argument('--clips', type=int, default=40, help='맵당 클립 수')
    ap.add_argument('--frames', type=int, default=6, help='클립당 연속 스캔 수 (10 Hz)')
    ap.add_argument('--warmup', type=float, default=40.0, help='맵마다 교통 예열 [s]')
    ap.add_argument('--seed', type=int, default=1)
    args = ap.parse_args()
    gpus = [g for g in args.gpus.split(',') if g != '']
    ps = [mp.Process(target=worker, args=(w, gpus[w % len(gpus)], args)) for w in range(args.workers)]
    for p in ps:
        p.start()
    for p in ps:
        p.join()


if __name__ == '__main__':
    mp.set_start_method('spawn')
    main()
