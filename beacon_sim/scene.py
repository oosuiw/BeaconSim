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
    'terrain': 6, 'tunnel': 7, 'vehicle': 8, 'robot': 9, 'sign': 10, 'pole': 11, 'accident': 12,
}

# 반사율(0~1). LiDAR intensity 계산과 raydrop에 쓴다. 실차 로그로 보정할 값.
RETRO = {
    'road': 0.10, 'lane_white': 0.55, 'lane_yellow': 0.45, 'guardrail': 0.35, 'barrier': 0.30,
    'terrain': 0.15, 'tunnel': 0.40, 'vehicle': 0.25, 'robot': 0.30, 'sign': 0.90, 'pole': 0.30,
    'accident': 0.25,
}

# 레이더 RCS (dBsm)
RCS = {'vehicle_car': 10.0, 'vehicle_truck': 20.0, 'guardrail': 0.0, 'barrier': -5.0,
       'tunnel': 5.0, 'sign': 15.0, 'pole': -5.0, 'road': -50.0, 'terrain': -50.0,
       'lane_white': -50.0, 'lane_yellow': -50.0, 'robot': 0.0, 'accident': 10.0}

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

    # 가드레일 (W빔 상자 근사 + 지주 4m 간격), 터널 구간 제외
    mb = MeshBuilder(m)
    for side in (-1, 1):
        tg = side * m.guardrail_t
        for a, b in _open_intervals(0, L, [(x0 - 5, x1 + 5) for x0, x1 in tun]):
            mb.sweep(a, b, [(tg - 0.05, 0.45), (tg + 0.05, 0.45), (tg + 0.05, 0.76), (tg - 0.05, 0.76)], closed=True)
            for sp in np.arange(a + 1.0, b, 4.0):
                mb.box_at(sp, tg + side * 0.10, (-0.06, -0.06, 0.0), (0.06, 0.06, 0.70))
    parts.append(mb.build('guardrail', 'guardrail'))

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
    return [q for q in parts if q is not None]


def _paint(m: RoadModel):
    """차선 도색 메시. 상태가 나쁜 구간은 끊김/희소 처리하고, 희미한 구간은 반사율·색 대비를 낮춘 별도 파트."""
    p = m.params
    L = m.length
    hw = p.line_width / 2
    rng = np.random.default_rng(p.seed * 7 + 99)
    asphalt = np.array(COLOR['road'])
    builders = {}                              # (cls, faded_level) → MeshBuilder

    def mb_for(cls, fade):
        key = (cls, round(fade, 1))
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
                        if b - a > 0.05:
                            mb_for(cls, fade).sweep(a, b, [(t - hw, PAINT_Z), (t + hw, PAINT_Z)], ds=2.5)
    out = []
    for (cls, fade), mb in sorted(builders.items()):
        base = np.array(COLOR[cls])
        col = tuple(base * (1 - fade) + asphalt * fade)
        retro = RETRO[cls] * (1 - fade) + RETRO['road'] * fade
        name = cls if fade == 0 else f'{cls}_faded{int(fade * 10)}'
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

def vehicle_mesh(kind, length, width, height):
    """차량 로컬 좌표(원점: 바닥 중심, x 전방) 메시."""
    mb = MeshBuilder()
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


def vehicle_lamps(kind, length, width, height):
    """전조등(흰색)·후미등(빨강) 상자 메시 → [(v, f, color)]. 렌더링 전용 (야간/악천후 발광)."""
    out = []
    zl, zh = (0.55, 0.75) if kind == 'car' else (0.6, 0.85)
    for sy in (-1, 1):
        y0 = sy * (width / 2 - 0.35)
        for x, col in [(length / 2 + 0.01, (1.0, 0.97, 0.85)), (-length / 2 - 0.06, (1.0, 0.08, 0.05))]:
            mb = MeshBuilder()
            mb.local_box([x - 0.05 if x > 0 else x, y0 - 0.18, zl], [x if x > 0 else x + 0.05, y0 + 0.18, zh])
            out.append((np.concatenate(mb.v).astype(np.float32), np.concatenate(mb.f).astype(np.int32), col))
    return out


def street_lights(m: RoadModel):
    """조명주 램프 위치 (map 좌표, 터널 밖) — 야간 렌더 광원."""
    pts = []
    for side in (-1, 1):
        tp = side * (m.guardrail_t + 1.0)
        for sp in np.arange(25.0, m.length, 50.0):
            if m.in_tunnel(sp, margin=10):
                continue
            pts.append(m.point(sp, tp - side * 1.8, 9.7))
    return np.array(pts) if pts else np.zeros((0, 3))


def tunnel_light(m: RoadModel, sth):
    """정점별 조명 계수 (1=주간, 터널 안쪽으로 40m에 걸쳐 0.4까지 감소)."""
    s, t, h = sth[:, 0], sth[:, 1], sth[:, 2]
    k = np.ones(len(s))
    for a, b in m.tunnels:
        inside = (s > a) & (s < b) & (np.abs(t) < m.edge_t + 0.5) & (h < TUNNEL_CROWN - 0.05)
        depth = np.minimum(s - a, b - s)
        k = np.where(inside, np.minimum(k, 1.0 - 0.6 * np.clip(depth / 40.0, 0, 1)), k)
    return k
