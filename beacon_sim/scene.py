"""도로 모델(roadgen.RoadModel) → 3D 씬 메시와 NPC 교통.

같은 정의를 sim_core(RGL 레이캐스트), camera_node(래스터 렌더), app(뷰어), export(PCD)가 함께 쓴다.
NPC 위치는 시뮬레이션 시각의 결정적 함수라 프로세스 간 상태 공유가 필요 없다.
"""
from dataclasses import dataclass

import numpy as np

from .roadgen import RoadModel

# 시맨틱 클래스 id (RGL entity id = 점별 정답 라벨)
CLASS = {
    'road': 1, 'lane_white': 2, 'lane_yellow': 3, 'guardrail': 4, 'barrier': 5,
    'terrain': 6, 'tunnel': 7, 'vehicle': 8, 'robot': 9, 'sign': 10, 'pole': 11, 'accident': 12, 'pedestrian': 13,
    'truck': 14, 'bus': 15, 'trailer': 16,
}
# 주행 차량 라벨: 승용(car) = 'vehicle'(8), 트럭 = 14, 버스 = 15, 세미 트레일러의 트레일러 = 16 (라이다 정답에서 차종으로 다시 매김). 사고 차량은 12

# 반사율(0~1). LiDAR intensity 계산과 raydrop에 쓴다. 실차 로그로 보정할 값.
RETRO = {
    'road': 0.10, 'lane_white': 0.55, 'lane_yellow': 0.45, 'guardrail': 0.35, 'barrier': 0.30,
    'terrain': 0.15, 'tunnel': 0.40, 'vehicle': 0.25, 'robot': 0.30, 'sign': 0.90, 'pole': 0.30,
    'accident': 0.25, 'pedestrian': 0.20, 'truck': 0.30, 'bus': 0.30, 'trailer': 0.30,
}

# 레이더 RCS (dBsm)
RCS = {'vehicle_car': 10.0, 'vehicle_truck': 20.0, 'truck': 20.0, 'bus': 20.0, 'trailer': 20.0, 'guardrail': 0.0, 'barrier': -5.0,
       'tunnel': 5.0, 'sign': 15.0, 'pole': -5.0, 'road': -50.0, 'terrain': -50.0,
       'lane_white': -50.0, 'lane_yellow': -50.0, 'robot': 0.0, 'accident': 10.0, 'pedestrian': -5.0}

COLOR = {
    'road': (0.22, 0.22, 0.23), 'lane_white': (0.92, 0.92, 0.90), 'lane_yellow': (0.95, 0.75, 0.10),
    'guardrail': (0.70, 0.72, 0.74), 'barrier': (0.62, 0.62, 0.60), 'terrain': (0.30, 0.42, 0.22),
    'tunnel': (0.55, 0.55, 0.52), 'sign': (0.05, 0.40, 0.20), 'pole': (0.55, 0.55, 0.55),
}

MESH_DS = 2.0          # 노면/구조물 종방향 샘플 간격 [m]
PAINT_Z = 0.003        # 차선 도료 두께 (z-fighting 방지)
TUNNEL_WALL_H = 4.0
TUNNEL_CROWN = 8.0
TUNNEL_COVER = 12.0    # 터널 위 산 높이 (노면 기준)
TERRAIN_W = 80.0


@dataclass
class Part:
    name: str
    cls: str
    v: np.ndarray
    f: np.ndarray
    color: tuple = None
    retro: float = None
    sth: np.ndarray = None     # 정점별 (s, t, h): 터널 조명 등 후처리용

    def __post_init__(self):
        self.v = np.asarray(self.v, dtype=np.float32)
        self.f = np.asarray(self.f, dtype=np.int32)
        if self.color is None:
            self.color = COLOR.get(self.cls, (0.5, 0.5, 0.5))
        if self.retro is None:
            self.retro = RETRO.get(self.cls, 0.2)


_BOX_F = [[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1],
          [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]]


def _box_pts(lo, hi):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])


class MeshBuilder:
    def __init__(self, model: RoadModel = None):
        self.m = model
        self.v, self.f, self.sth, self.n = [], [], [], 0

    def add(self, v, f, sth=None):
        v = np.asarray(v, dtype=np.float64).reshape(-1, 3)
        self.v.append(v)
        self.f.append(np.asarray(f).reshape(-1, 3) + self.n)
        self.sth.append(np.zeros((len(v), 3)) if sth is None else np.asarray(sth, float).reshape(-1, 3))
        self.n += len(v)

    def grid(self, s, t, h, closed=False, flip=False):
        """(ns, nt) 격자 (s, t, h) → 노면 좌표 메시. 각 행은 같은 s의 횡단 단면."""
        s = np.asarray(s, float)
        if s.ndim == 1:
            s = s[:, None]
        S, Tt, H = np.broadcast_arrays(s, np.asarray(t, float), np.asarray(h, float))
        P = self.m.point(S, Tt, H)
        ns, nt = P.shape[:2]
        idx = np.arange(ns * nt).reshape(ns, nt)
        cols = nt if closed else nt - 1
        j0, j1 = np.arange(cols), (np.arange(cols) + 1) % nt
        a, b, c, d = idx[:-1][:, j0], idx[:-1][:, j1], idx[1:][:, j1], idx[1:][:, j0]
        f = np.concatenate([np.stack([a, b, c], -1).reshape(-1, 3), np.stack([a, c, d], -1).reshape(-1, 3)])
        if flip:
            f = f[:, ::-1]
        self.add(P.reshape(-1, 3), f, np.stack([S, Tt, H], -1).reshape(-1, 3))

    def sweep(self, s0, s1, profile, closed=False, flip=False, ds=MESH_DS):
        """횡단 프로파일 [(t, h), ...]를 s0→s1로 스윕."""
        n = max(int(np.ceil((s1 - s0) / ds)), 1)
        s = np.linspace(s0, s1, n + 1)
        prof = np.asarray(profile, float)
        self.grid(s, prof[None, :, 0], prof[None, :, 1], closed=closed, flip=flip)

    def box_at(self, s, t, lo, hi, reverse=False):
        """노면 포즈(s, t) 로컬 상자. lo/hi: (전방, 좌측, 위) 범위."""
        T = self.m.pose(s, t, reverse)
        p = _box_pts(lo, hi)
        w = p @ T[:3, :3].T + T[:3, 3]
        sth = np.c_[np.full(8, s), np.full(8, t), p[:, 2]]
        self.add(w, _BOX_F, sth)

    def local_box(self, lo, hi):
        self.add(_box_pts(lo, hi), _BOX_F)

    def build(self, name, cls, **kw):
        if not self.v:
            return None
        return Part(name, cls, np.concatenate(self.v), np.concatenate(self.f), sth=np.concatenate(self.sth), **kw)


def _open_intervals(a, b, holes):
    """[a, b]에서 holes 구간을 뺀 구간 목록."""
    out = [(a, b)]
    for h0, h1 in sorted(holes):
        nxt = []
        for x0, x1 in out:
            if h1 <= x0 or h0 >= x1:
                nxt.append((x0, x1))
            else:
                if h0 > x0:
                    nxt.append((x0, h0))
                if h1 < x1:
                    nxt.append((h1, x1))
        out = nxt
    return [(x0, x1) for x0, x1 in out if x1 - x0 > 0.05]


def tunnel_profile(m: RoadModel, side):
    """한 방향 터널 내공 단면 [(t, h)] : 바깥 벽 하단 → 아치 → 중앙 벽 하단."""
    t_in, t_out = side * m.params.barrier_half, side * (m.edge_t + 0.3)
    tc, half = (t_in + t_out) / 2, abs(t_in - t_out) / 2
    th = np.linspace(0, np.pi, 25)
    arch_t = tc + np.sign(t_out - t_in) * half * np.cos(th)          # 바깥 → 안쪽
    arch_h = TUNNEL_WALL_H + (TUNNEL_CROWN - TUNNEL_WALL_H) * np.sin(th)
    return np.vstack([[t_out, -0.02], np.c_[arch_t, arch_h], [t_in, -0.02]])


def _portal_outer(ring, t_lo, t_hi, h_top):
    """내공 단면 각 점을 중심에서 바깥 사각형으로 방사 투영."""
    c = np.array([(ring[0, 0] + ring[-1, 0]) / 2, TUNNEL_WALL_H * 0.5])
    out = []
    for q in ring:
        d = q - c
        best = np.inf
        for k, (lo, hi) in enumerate([(t_lo, t_hi), (-0.02, h_top)]):
            if abs(d[k]) < 1e-9:
                continue
            for bnd in (lo, hi):
                tt = (bnd - c[k]) / d[k]
                if tt > 0:
                    qq = c + tt * d
                    if t_lo - 1e-6 <= qq[0] <= t_hi + 1e-6 and -0.02 - 1e-6 <= qq[1] <= h_top + 1e-6:
                        best = min(best, tt)
        out.append(c + best * d)
    return np.array(out)


def _rail_segments(p, side, a, b):
    """바깥 방호시설 구간 [(시작, 끝, 'guardrail'|'concrete')] — mixed 는 200~800 m 구간마다 무작위 (맵 시드로 결정)"""
    kind = getattr(p, 'outer_rail', 'guardrail')
    if kind in ('guardrail', 'concrete'):
        return [(a, b, kind)]
    rng = np.random.default_rng(p.seed * 7 + (3 if side > 0 else 5))
    out, s0 = [], a
    k = 'concrete' if rng.random() < 0.5 else 'guardrail'
    while s0 < b:
        s1 = min(b, s0 + rng.uniform(200, 800))
        out.append((s0, s1, k))
        k = 'guardrail' if k == 'concrete' else 'concrete'
        s0 = s1
    return out


def build_static(m: RoadModel):
    p = m.params
    L = m.length
    parts = []
    tun = m.tunnels

    # 노면: 중앙분리대 아래 평탄 + 양쪽 횡단경사
    mb = MeshBuilder(m)
    pv = m.paved_t
    mb.sweep(0, L, [(-pv, 0), (-p.barrier_half, 0), (p.barrier_half, 0), (pv, 0)])
    parts.append(mb.build('road', 'road'))

    # 차선 도색: 구간별 상태 (양호 / 희미함 / 끊김 / 희소). 결정적 난수
    parts += _paint(m)

    # 중앙분리대 (콘크리트 방호벽 0.81m). 터널 구간은 터널 벽으로 대체
    mb = MeshBuilder(m)
    bh = p.barrier_half
    prof = [(bh, 0.0), (0.18, 0.08), (0.12, 0.81), (-0.12, 0.81), (-0.18, 0.08), (-bh, 0.0)]
    for a, b in _open_intervals(0, L, tun):
        mb.sweep(a, b, prof, flip=True)
    parts.append(mb.build('barrier', 'barrier'))

    # 바깥 방호시설, 터널 구간 제외: W빔 가드레일(상자 근사 + 지주 4m 간격) 또는 콘크리트 방호벽(갓길 쪽 면이 가장자리)
    mb, mbc = MeshBuilder(m), MeshBuilder(m)
    for side in (-1, 1):
        tg = side * m.guardrail_t
        for a, b in _open_intervals(0, L, [(x0 - 5, x1 + 5) for x0, x1 in tun]):
            for a2, b2, kind in _rail_segments(p, side, a, b):
                if kind == 'concrete':
                    tc = tg + side * 0.25                         # 방호벽 중심 (도로 쪽 아랫면 ≈ 가장자리 + 0.05 m)
                    prof = [(tc + x, z) for x, z in ((0.3, 0.0), (0.18, 0.08), (0.12, 0.81), (-0.12, 0.81),
                                                      (-0.18, 0.08), (-0.3, 0.0))]      # 중앙분리대와 같은 단면·방향
                    mbc.sweep(a2, b2, prof, flip=True)
                else:
                    mb.sweep(a2, b2, [(tg - 0.05, 0.45), (tg + 0.05, 0.45), (tg + 0.05, 0.76), (tg - 0.05, 0.76)], closed=True)
                    for sp in np.arange(a2 + 1.0, b2, 4.0):
                        mb.box_at(sp, tg + side * 0.10, (-0.06, -0.06, 0.0), (0.06, 0.06, 0.70))
    parts.append(mb.build('guardrail', 'guardrail'))
    if mbc.v:
        parts.append(mbc.build('barrier', 'barrier'))
    g = _gutter(m)
    if g is not None:
        parts.append(g)

    # 조명주 (양쪽 50m 간격, 터널 밖)
    mb = MeshBuilder(m)
    for side in (-1, 1):
        tp = side * (m.guardrail_t + 1.0)
        for sp in np.arange(25.0, L, 50.0):
            if m.in_tunnel(sp, margin=10):
                continue
            mb.box_at(sp, tp, (-0.1, -0.1, 0.0), (0.1, 0.1, 10.0))
            arm = (-side) * 2.0
            mb.box_at(sp, tp, (-0.1, min(0, arm), 9.8), (0.1, max(0, arm), 10.0))
    parts.append(mb.build('poles', 'pole'))

    # 갠트리 + 표지판 (하행 쪽)
    pole, sign = MeshBuilder(m), MeshBuilder(m)
    for g in m.gantries:
        tr, tl = -(m.guardrail_t + 0.6), -0.8
        for t in (tr, tl):
            pole.box_at(g, t, (-0.2, -0.2, 0), (0.2, 0.2, 7.2))
        tm, hw_ = (tr + tl) / 2, abs(tl - tr) / 2
        pole.box_at(g, tm, (-0.25, -hw_, 6.8), (0.25, hw_, 7.2))
        sign.box_at(g, tm + 1.0, (-0.35, -3.5, 4.6), (-0.25, 3.5, 6.8))
    parts.append(pole.build('gantry', 'pole'))
    parts.append(sign.build('sign', 'sign'))

    # 터널 (방향별 단독 터널) + 갱문
    lining, portal = MeshBuilder(m), MeshBuilder(m)
    for s0, s1 in tun:
        for side in (-1, 1):
            ring = tunnel_profile(m, side)
            lining.sweep(s0, s1, ring, flip=(side > 0))
            t_out = ring[0, 0]
            t_lo, t_hi = sorted([t_out + side * 6.0, 0.0])
            outer = _portal_outer(ring, t_lo, t_hi, TUNNEL_COVER)
            for sg in (s0, s1):
                portal.grid(np.full((2, len(ring)), sg), np.stack([ring[:, 0], outer[:, 0]]),
                            np.stack([ring[:, 1], outer[:, 1]]))
    parts.append(lining.build('tunnel', 'tunnel'))
    parts.append(portal.build('portal', 'tunnel'))

    # 지형: 성토 사면 + 터널 구간 산지
    parts.append(_terrain(m))
    cl = _clutter(m)
    if cl is not None:
        parts.append(cl)
    return [q for q in parts if q is not None]


def _paint(m: RoadModel):
    """차선 도색 메시. 상태가 나쁜 구간은 끊김/희소 처리하고, 밝기가 다른 구간은 반사율·색 대비를 바꾼 별도 파트.
    밝기 배율 g: 1 = 기본, < 1 바램(노면 쪽으로), > 1 새 도색(더 밝음). 라이다 세기·raydrop·카메라 색 모두 g 를 따름."""
    p = m.params
    L = m.length
    hw = p.line_width / 2
    rng = np.random.default_rng(p.seed * 7 + 99)
    asphalt = np.array(COLOR['road'])
    builders = {}                              # (cls, 밝기 배율 g) → MeshBuilder
    var = float(getattr(p, 'paint_var', 0.0))
    vrng = np.random.default_rng(p.seed * 7 + 131)  # 밝기 편차 전용 (기존 마모 난수 순서는 그대로)

    def line_gain_fn():
        """선 하나의 밝기 배율 g(s) = 바탕 × 느린 흐려짐 (파장 30~200 m 두 개).
        바탕: 0.6·var 확률로 거의 지워진 선 U(0.03, 0.35), 나머지는 exp(N(0.5·var, 0.6·var)) (보통 ~ 새 도색).
        var = 1 ≈ 실기 bag 4개 (정렬 좋은 bag: 한 장면 안 밝은 선 / 어두운 선 중앙값 8~10배, 선별 밝기 7 ~ 50~79)"""
        if var <= 0:
            return None
        if vrng.random() < 0.6 * var:
            base = vrng.uniform(0.03, 0.35)
        else:
            base = float(np.exp(vrng.normal(0.5 * var, 0.6 * var)))
        waves = [(vrng.uniform(0, 0.25 * var), vrng.uniform(30.0, 200.0), vrng.uniform(0, 2 * np.pi)) for _ in range(2)]
        return lambda x: float(np.clip(base * (1 - sum(a * (0.5 + 0.5 * np.sin(2 * np.pi * x / lam + ph))
                                                       for a, lam, ph in waves)), 0.03, 3.3))

    def mb_for(cls, g):
        key = (cls, round(g, 1) if g <= 1 else round(g * 4) / 4)
        if key not in builders:
            builders[key] = MeshBuilder(m)
        return builders[key]

    for side in (-1, 1):
        for ln in m.lines():
            if ln.kind == 'road_border':
                continue
            cls = 'lane_yellow' if ln.kind == 'yellow_solid' else 'lane_white'
            if ln.kind == 'white_dashed':
                dashes, a = [], 0.0
                while a < L:
                    dashes.append((a, min(a + p.dash_len, L)))
                    a += p.dash_len + p.dash_gap
            else:
                dashes = [(0.0, L)]
            segs = sorted((a, b, k, st) for sd, key, a, b, k, st in m.wear if sd == side and key == ln.key)
            t = side * ln.offset
            lf = line_gain_fn()

            def cond_at(x):
                for a, b, k, st in segs:
                    if a <= x < b:
                        return k, st
                return 'good', 0.0

            # 상태 경계에서 도색 조각을 나눈다
            cuts = sorted({0.0, L} | {a for a, _, _, _ in segs} | {b for _, b, _, _ in segs})
            for d0, d1 in dashes:
                inner = [c for c in cuts if d0 < c < d1]
                bounds = [d0] + inner + [d1]
                for x0, x1 in zip(bounds[:-1], bounds[1:]):
                    k, st = cond_at((x0 + x1) / 2)
                    pieces = [(x0, x1)]
                    fade = 0.0
                    if k == 'faded':
                        fade = 0.4 + 0.5 * st
                    elif k == 'broken':                        # 벗겨진 자리: 0.3~3m 구멍이 st 비율만큼
                        out, x = [], x0
                        while x < x1:
                            seg = float(rng.uniform(0.3, 3.0))
                            if rng.random() > 0.65 * st:
                                out.append((x, min(x + seg, x1)))
                            x += seg
                        pieces = out
                    elif k == 'sparse':
                        if ln.kind == 'white_dashed':           # 점선: 일부 도막만 남음
                            pieces = [(x0, x1)] if rng.random() > 0.5 + 0.4 * st else []
                        else:                                   # 실선: 짧은 토막만
                            out, x = [], x0
                            while x < x1:
                                out.append((x, min(x + 1.0, x1)))
                                x += 4.0 + 6.0 * st
                            pieces = out
                    for a, b in pieces:
                        if b - a <= 0.05:
                            continue
                        if lf is None:
                            mb_for(cls, 1 - fade).sweep(a, b, [(t - hw, PAINT_Z), (t + hw, PAINT_Z)], ds=2.5)
                            continue
                        n = max(1, int(np.ceil((b - a) / 5.0)))             # 5 m 토막마다 선 밝기 편차를 겹침
                        edges = np.linspace(a, b, n + 1)
                        for c0, c1 in zip(edges[:-1], edges[1:]):
                            g = (1 - fade) * lf((c0 + c1) / 2)
                            mb_for(cls, max(g, 0.03)).sweep(c0, c1, [(t - hw, PAINT_Z), (t + hw, PAINT_Z)], ds=2.5)
    out = []
    for (cls, g), mb in sorted(builders.items()):
        base = np.array(COLOR[cls])
        col = tuple(np.clip(asphalt + (base - asphalt) * g, 0, 1))          # 밝은 새 도색은 더 하얗게 (상한 1)
        retro = min(1.6, RETRO['road'] + (RETRO[cls] - RETRO['road']) * g)     # 새 도색 상한 약 3.3배
        name = cls if g == 1 else f'{cls}_faded{int(round(g * 100))}'       # '_faded' = Godot 라벨에서 차선으로 묶는 표시
        part = mb.build(name, cls, color=col, retro=retro)
        if part is not None:
            out.append(part)
    return out


def _mountain(m: RoadModel, s):
    """터널 위/주변 산 높이 (노면 기준)."""
    s = np.asarray(s, float)
    H = np.zeros_like(s)
    for a, b in m.tunnels:
        ramp = 120.0
        k = np.minimum(np.clip((s - (a - ramp)) / ramp, 0, 1), np.clip(((b + ramp) - s) / ramp, 0, 1))
        H = np.maximum(H, TUNNEL_COVER * k * k * (3 - 2 * k))
    return H


def _gutter(m: RoadModel):
    """측구: 방호시설 앞 노면 위 콘크리트 띠 (L형 측구, 방호벽 쪽으로 살짝 올라감) + 가드레일 구간 연석.
    실기 라이다에서 노면 높이의 밝은 띠(아스팔트 5~8 → 11~17)로 찍혀 차선과 헷갈리는 것을 재현.
    라벨은 콘크리트 구조물이라 'barrier' (밝기 모델도 방호벽과 같음)."""
    p, L, tun = m.params, m.length, m.tunnels
    wo = min(float(getattr(p, 'gutter_w', 0.0)), p.outer_shoulder + 0.25 - 0.3)       # 바깥 차선(실선)과 겹치지 않게
    wm = min(float(getattr(p, 'gutter_med_w', 0.0)), p.inner_shoulder - 0.25)
    ch = float(getattr(p, 'curb_h', 0.0))
    if wo <= 0.05 and wm <= 0.05:
        return None
    mb = MeshBuilder(m)
    open_iv = _open_intervals(0, L, [(x0 - 5, x1 + 5) for x0, x1 in tun])
    for side in (-1, 1):
        if wo > 0.05:
            t1 = m.edge_t + 0.25                                   # 콘크리트 방호벽 아랫면 = 가드레일 지주 앞
            for a, b in open_iv:
                for a2, b2, kind in _rail_segments(p, side, a, b):
                    prof = [(side * (t1 - wo), 0.004), (side * (t1 - 0.3 * wo), 0.006), (side * t1, 0.03)]
                    if kind != 'concrete' and ch > 0.02:               # 가드레일 구간: 측구 바깥 연석
                        prof += [(side * t1, ch), (side * (t1 + 0.15), ch), (side * (t1 + 0.15), 0.0)]
                    mb.sweep(a2, b2, prof, flip=side < 0)     # 노면과 같은 방향 (t 증가 = 위쪽 면)
        if wm > 0.05:
            t0 = p.barrier_half
            for a, b in _open_intervals(0, L, tun):
                mb.sweep(a, b, [(side * (t0 + wm), 0.004), (side * (t0 + 0.3 * wm), 0.006), (side * t0, 0.03)],
                         flip=side > 0)
    return mb.build('gutter', 'barrier')


def _clutter(m: RoadModel):
    """도로 밖 수풀·덤불 (지형 클래스 상자): 포장 끝 바깥 0.3~9 m, 성토 사면 높이를 따라. 밀도 = roadside_clutter"""
    p = m.params
    dens = float(getattr(p, 'roadside_clutter', 0.0))
    if dens <= 0:
        return None
    rng = np.random.default_rng(p.seed * 11 + 7)
    pv = m.paved_t
    mb = MeshBuilder(m)
    for side in (-1, 1):
        s = 2.0
        while s < m.length - 2.0:
            s += float(rng.exponential(6.0 / dens))
            if m.in_tunnel(s, margin=5) or s >= m.length - 2.0:
                continue
            Hm = float(_mountain(m, np.array([s]))[0])
            for _ in range(int(rng.integers(1, 4))):                  # 덤불 무리 (포장 끝 가까이 많이)
                d = 0.3 + float(rng.exponential(1.8))
                if d > 9.0:
                    continue
                # 지형 모델(_terrain)과 같은 높이: 성토 1:2 사면(최대 1.5 m 아래)과 절토 사면 중 높은 쪽, 포장 끝 높이 기준
                ground = max(-min(0.5 * d, 1.5), Hm * float(np.clip(d / max(Hm, 1e-3), 0, 1)))
                ground += float(m.surface_z(s, side * pv) - m.surface_z(s, side * (pv + d)))
                h = float(rng.choice([rng.uniform(0.1, 0.4), rng.uniform(0.4, 1.6)], p=[0.6, 0.4]))
                L, W = float(rng.uniform(0.3, 2.5)), float(rng.uniform(0.3, 1.5))
                mb.box_at(s + rng.uniform(-1.5, 1.5), side * (pv + d), (-L / 2, -W / 2, ground - 0.05), (L / 2, W / 2, ground + h))
    return mb.build('terrain', 'terrain')


def _terrain(m: RoadModel):
    pv = m.paved_t
    ts = np.array([pv, pv + 1, pv + 2, pv + 4, pv + 7, pv + 11, pv + 16, pv + 24, pv + 35, pv + 50, TERRAIN_W])
    s = np.arange(0, m.length, 5.0)
    s = np.r_[s, m.length]
    Hm = _mountain(m, s)[:, None]
    mb = MeshBuilder(m)
    for side in (-1, 1):
        t = side * ts[None, :]
        d = ts[None, :] - pv
        base = -np.minimum(0.5 * d, 1.5)                       # 1:2 성토 사면, 1.5m 아래 평지
        cut = np.clip(d / np.maximum(Hm, 1e-3), 0, 1)          # 1:1 절토 사면으로 산까지
        h = np.maximum(base, Hm * cut)
        # h는 '포장 끝 노면 높이' 기준 → point()가 쓰는 해당 t의 노면 높이 기준으로 환산
        h = h + m.surface_z(s[:, None], side * pv) - m.surface_z(s[:, None], t)
        mb.grid(s, t, h, flip=(side > 0))
    # 터널 구간 노면 위를 덮는 산
    for a, b in m.tunnels:
        st = np.r_[np.arange(a, b, 5.0), b]
        tt = np.linspace(-pv, pv, 9)[None, :]
        h = _mountain(m, st)[:, None] + m.surface_z(st[:, None], 0.0) - m.surface_z(st[:, None], tt)
        mb.grid(st, tt, h)
    return mb.build('terrain', 'terrain')


# ---------------------------------------------------------------- 차량 메시

def vehicle_mesh(kind, length, width, height, model='', variant=0):
    """차량 로컬 좌표(원점: 바닥 중심, x 전방) 메시. model 이 있으면 차종별 절차 메시 (vehicles)."""
    if model:
        from . import vehicles
        return vehicles.mesh(model, variant)
    mb = MeshBuilder()
    if kind == 'pedestrian':                   # 사람: 다리 2 · 몸통 · 팔 2 · 머리 (x 전방, length = 몸 두께, width = 어깨 폭)
        h, d, w = height, length, width
        leg = 0.47 * h
        for y0, y1 in ((0.06, 0.30), (-0.30, -0.06)):                       # 다리
            mb.local_box([-d / 2 * 0.7, y0 * w, 0.0], [d / 2 * 0.7, y1 * w, leg])
        for y0, y1 in ((0.38, 0.5), (-0.5, -0.38)):                          # 팔
            mb.local_box([-d / 2 * 0.5, y0 * w, leg + 0.08], [d / 2 * 0.5, y1 * w, 0.82 * h])
        mb.local_box([-d / 2, -w * 0.36, leg], [d / 2, w * 0.36, 0.84 * h])
        mb.local_box([-0.11, -0.09, 0.85 * h], [0.11, 0.09, h])
        return np.concatenate(mb.v).astype(np.float32), np.concatenate(mb.f).astype(np.int32)
    if kind == 'car':
        mb.local_box([-length / 2, -width / 2, 0.25], [length / 2, width / 2, 0.85])
        mb.local_box([-length / 2 + 0.9, -width / 2 + 0.1, 0.85], [length / 2 - 1.3, width / 2 - 0.1, height])
        for sx in (-1, 1):
            for sy in (-1, 1):
                cx, cy = sx * (length / 2 - 0.8), sy * (width / 2 - 0.15)
                mb.local_box([cx - 0.33, cy - 0.12, 0.0], [cx + 0.33, cy + 0.12, 0.66])
    else:
        mb.local_box([length / 2 - 2.2, -width / 2, 0.4], [length / 2, width / 2, 2.9])
        mb.local_box([-length / 2, -width / 2, 0.9], [length / 2 - 2.4, width / 2, height])
        for sx in (-0.38, -0.25, 0.38):
            for sy in (-1, 1):
                cx, cy = sx * length, sy * (width / 2 - 0.2)
                mb.local_box([cx - 0.5, cy - 0.2, 0.0], [cx + 0.5, cy + 0.2, 1.0])
    return np.concatenate(mb.v).astype(np.float32), np.concatenate(mb.f).astype(np.int32)


def vehicle_lamps(kind, length, width, height, model='', variant=0):
    """전조등(흰색)·후미등(빨강) 상자 메시 → [(v, f, color)]. 렌더링 전용 (야간/악천후 발광)."""
    if model:
        from . import vehicles
        return vehicles.lamps(model, variant)
    out = []
    if kind == 'pedestrian':
        return out
    zl, zh = (0.55, 0.75) if kind == 'car' else (0.6, 0.85)
    for sy in (-1, 1):
        y0 = sy * (width / 2 - 0.35)
        for x, col in [(length / 2 + 0.01, (1.0, 0.97, 0.85)), (-length / 2 - 0.06, (1.0, 0.08, 0.05))]:
            mb = MeshBuilder()
            mb.local_box([x - 0.05 if x > 0 else x, y0 - 0.18, zl], [x if x > 0 else x + 0.05, y0 + 0.18, zh])
            out.append((np.concatenate(mb.v).astype(np.float32), np.concatenate(mb.f).astype(np.int32), col))
    return out


def street_lights(m: RoadModel, with_s=False):
    """조명주 램프 위치 (map 좌표, 터널 밖) — 야간 렌더 광원. with_s 면 각 램프의 s 도 함께."""
    pts, ss = [], []
    for side in (-1, 1):
        tp = side * (m.guardrail_t + 1.0)
        for sp in np.arange(25.0, m.length, 50.0):
            if m.in_tunnel(sp, margin=10):
                continue
            pts.append(m.point(sp, tp - side * 1.8, 9.7))
            ss.append(sp)
    P = np.array(pts) if pts else np.zeros((0, 3))
    return (P, np.array(ss)) if with_s else P


def street_lights_on(m: RoadModel, s_lamps, env, seed=0):
    """램프별 켜짐 (bool 배열). 가로등 있는 구간(partial: 400~1500 m 덩어리, 터널 입출구 앞뒤 150 m 는 항상) + 고장."""
    n = len(s_lamps)
    if env is None or env.time_of_day == 'day' or env.street_mode == 'none' or n == 0:
        return np.zeros(n, bool)
    rng = np.random.default_rng(int(env.seed) * 7919 + seed + 17)
    lit = np.ones(n, bool)
    if env.street_mode == 'partial':
        lit[:] = False
        a = float(rng.uniform(-500, 0))
        while a < m.length:
            L = float(rng.uniform(400, 1500))
            if rng.random() < env.street_coverage:
                lit |= (s_lamps >= a) & (s_lamps < a + L)
            a += L
        for ta, tb in m.tunnels:                     # 터널 입출구 조명 (완화 구간)
            lit |= ((s_lamps > ta - 150) & (s_lamps < ta)) | ((s_lamps > tb) & (s_lamps < tb + 150))
    lit &= rng.random(n) >= env.lamp_fail
    return lit


def tunnel_light(m: RoadModel, sth):
    """정점별 조명 계수 (1=주간, 터널 안쪽으로 40m에 걸쳐 0.4까지 감소)."""
    s, t, h = sth[:, 0], sth[:, 1], sth[:, 2]
    k = np.ones(len(s))
    for a, b in m.tunnels:
        inside = (s > a) & (s < b) & (np.abs(t) < m.edge_t + 0.5) & (h < TUNNEL_CROWN - 0.05)
        depth = np.minimum(s - a, b - s)
        k = np.where(inside, np.minimum(k, 1.0 - 0.6 * np.clip(depth / 40.0, 0, 1)), k)
    return k
