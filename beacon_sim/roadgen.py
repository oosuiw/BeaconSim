"""시드 기반 고속도로 도로 모델 생성기.

도로 모델(중심선 + 횡단 구성 + 터널/부대시설) 하나에서
  - 3D 씬 메시 (scene.py, RGL/카메라/뷰어)
  - Lanelet2 OSM, PCD (export.py)
  - NPC 경로, 로봇 초기 위치
를 모두 파생한다. 같은 (시드, 파라미터, GENERATOR_VERSION)이면 항상 같은 맵이 나온다.

좌표계 (map): MGRS 100km 격자 로컬 좌표 (Autoware MGRS 투영과 동일, local_x/local_y).
  지역 위경도(region_lat/lon)가 속한 격자 원점에서 (start_x, start_y) 떨어진 곳에서 시작한다.
  좌표값을 수 km 이내로 유지해 RGL/OpenGL float32 정밀도 문제를 피한다.

횡단 좌표 t: 중심선(중앙분리대 중심) 기준 좌측 +. 하행(비콘 쪽) 차로는 t<0, +s 방향 주행.
"""
from dataclasses import asdict, dataclass, field

import numpy as np

GENERATOR_VERSION = '0.2.0'      # 0.2: 차선 상태(희미/끊김/희소) 구간 모델

# 설계속도별 기준 (도로의 구조·시설 기준에 관한 규칙 참고값)
DESIGN = {
    80: dict(r_min=280.0, l_clothoid_min=70.0, grade_max=0.04),
    100: dict(r_min=460.0, l_clothoid_min=85.0, grade_max=0.03),
    120: dict(r_min=710.0, l_clothoid_min=100.0, grade_max=0.03),
}


@dataclass
class RoadParams:
    seed: int = 1
    length: float = 3000.0           # 본선 길이 [m]
    lanes: int = 2                   # 방향별 차로 수
    design_speed: int = 100          # km/h (80 | 100 | 120)
    grade_max: float = 0.0           # 최대 종단 경사 (0 = 설계속도 기준값: 80 → 4%, 100·120 → 3%). 산악 구간은 5~6%
    tunnels: int = 1                 # 최대 터널 수 (실제 개수는 시드와 공간에 따라)
    tunnel_len_min: float = 200.0
    tunnel_len_max: float = 700.0
    gantries: int = 2
    wear: float = 0.3                # 차선 상태가 나쁜 구간 비율 0~1 (렌더/센서에만 반영, Lanelet 맵에는 선 유지)
    wear_faded: float = 1.0          # 나쁜 구간 중 유형별 가중치: 희미함(반사율·대비 저하)
    wear_broken: float = 1.0         #   끊김(도색이 군데군데 벗겨짐)
    wear_sparse: float = 1.0         #   희소(점선 일부만 남음 / 실선이 짧은 토막)
    paint_var: float = 0.0           # 도색 밝기 편차 0~1: 선마다 거의 지워진 선(0.6·값 확률) ~ 새 도색, 선을 따라 수십~백여 m 에
                                     #   걸쳐 서서히 흐려짐 (마모 구간과 겹침). 0 = 모든 선 같은 밝기, 1 ≈ 실기 bag
    lane_width: float = 3.6
    inner_shoulder: float = 1.0
    outer_shoulder: float = 3.0
    roadside_clutter: float = 0.0    # 도로 밖 수풀·덤불 밀도 0~1 (라이다 잡물. 0 = 매끈한 지형만)
    outer_rail: str = 'guardrail'    # 바깥 방호시설: guardrail(W빔) | concrete(콘크리트 방호벽) | mixed(구간마다 섞임)
    gutter_w: float = 0.0            # 바깥 측구(콘크리트 L형, 방호시설 앞 노면 높이의 밝은 띠) 폭 [m], 0 = 없음
    gutter_med_w: float = 0.0        # 중앙분리대 쪽 측구 폭 [m]
    curb_h: float = 0.0              # 가드레일 구간 측구 바깥 연석 높이 [m], 0 = 없음
    barrier_half: float = 0.3
    line_width: float = 0.15
    dash_len: float = 10.0           # 백색 점선 도색 길이
    dash_gap: float = 10.0           # 빈 길이
    region_lat: float = 36.50        # 지역 (이 위경도가 속한 MGRS 100km 격자를 쓴다)
    region_lon: float = 127.50
    start_x: float = 3000.0          # 격자 내 시작점 [m]
    start_y: float = 3000.0
    start_elevation: float = 80.0

    def to_dict(self):
        return asdict(self)


@dataclass
class Line:
    """차선/경계선 정의 (한 방향 기준, |t| 오프셋)."""
    key: str
    offset: float           # |t|
    kind: str               # 'yellow_solid' | 'white_solid' | 'white_dashed' | 'road_border'


@dataclass
class RoadModel:
    params: RoadParams
    s: np.ndarray           # 샘플 위치 (ds 간격)
    x: np.ndarray
    y: np.ndarray
    z: np.ndarray
    heading: np.ndarray
    kappa: np.ndarray
    cross_slope: np.ndarray  # (N, 2): [하행(c=-1), 상행(c=+1)] 바깥쪽 방향 dz/d|t|
    tunnels: list = field(default_factory=list)     # [(s0, s1)]
    gantries: list = field(default_factory=list)    # [s]
    wear: list = field(default_factory=list)        # [(side, line_key, s0, s1, kind, strength)] kind: faded|broken|sparse
    mgrs_grid: str = ''
    utm_zone: int = 52
    grid_origin_utm: tuple = (0.0, 0.0)             # 격자 원점 (UTM E, N)
    segments: list = field(default_factory=list)    # 선형 구성 요약

    # ---------------- 횡단 구성
    @property
    def length(self):
        return float(self.s[-1])

    def lines(self):
        p = self.params
        L1 = p.barrier_half + p.inner_shoulder
        out = [Line('median_border', p.barrier_half, 'road_border'), Line('yellow', L1, 'yellow_solid')]
        for k in range(1, p.lanes):
            out.append(Line(f'div{k}', L1 + k * p.lane_width, 'white_dashed'))
        out.append(Line('edge', L1 + p.lanes * p.lane_width, 'white_solid'))
        out.append(Line('outer_border', L1 + p.lanes * p.lane_width + p.outer_shoulder, 'road_border'))
        return out

    def lane_center_t(self, lane, side=-1):
        """lane 0 = 1차로(중앙분리대 쪽). side -1 = 하행(+s 주행)."""
        p = self.params
        return side * (p.barrier_half + p.inner_shoulder + (lane + 0.5) * p.lane_width)

    @property
    def edge_t(self):
        p = self.params
        return p.barrier_half + p.inner_shoulder + p.lanes * p.lane_width + p.outer_shoulder

    @property
    def paved_t(self):
        return self.edge_t + 0.6

    @property
    def guardrail_t(self):
        return self.edge_t + 0.3

    def in_tunnel(self, s, margin=0.0):
        s = np.asarray(s)
        m = np.zeros(s.shape, bool)
        for a, b in self.tunnels:
            m |= (s >= a - margin) & (s <= b + margin)
        return m

    # ---------------- 기하
    def _interp(self, s, arr):
        return np.interp(s, self.s, arr)

    def frame(self, s):
        """중심선 위치(x,y,z), 방위각."""
        s = np.asarray(s, float)
        th = np.interp(s, self.s, self.heading)          # heading은 누적 적분값이라 이미 연속
        return self._interp(s, self.x), self._interp(s, self.y), self._interp(s, self.z), th

    def surface_z(self, s, t):
        """노면 높이 (편경사/횡단경사 포함). 중앙분리대 구간은 중심선 높이."""
        s, t = np.broadcast_arrays(np.asarray(s, float), np.asarray(t, float))
        zc = self._interp(s, self.z)
        sl_dn = self._interp(s, self.cross_slope[:, 0])
        sl_up = self._interp(s, self.cross_slope[:, 1])
        d = np.maximum(np.abs(t) - self.params.barrier_half, 0.0)
        return zc + np.where(t < 0, sl_dn, sl_up) * d

    def point(self, s, t, h=0.0):
        """(s, t) 노면 위 h 높이의 map 좌표. 배열 브로드캐스트."""
        s, t, h = np.broadcast_arrays(np.asarray(s, float), np.asarray(t, float), np.asarray(h, float))
        x, y, _, th = self.frame(s)
        px = x - np.sin(th) * t
        py = y + np.cos(th) * t
        pz = self.surface_z(s, t) + h
        return np.stack([px, py, pz], -1)

    def pose(self, s, t, reverse=False):
        """노면에 붙은 4x4 포즈 (x: 진행방향, z: 노면 법선 근사)."""
        ds = 0.5
        P = self.point(np.array([s, s + ds, s - ds, s, s]), np.array([t, t, t, t + ds, t - ds]))
        p0, a, b = P[0], P[1] - P[2], P[3] - P[4]
        fx = a / np.linalg.norm(a)
        fz = np.cross(fx, b)
        fz /= np.linalg.norm(fz)
        fy = np.cross(fz, fx)
        if reverse:
            fx, fy = -fx, -fy
        T = np.eye(4)
        T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = fx, fy, fz, p0
        return T

    def poses(self, s, t, reverse):
        """pose()의 벡터화 버전 → (N, 4, 4)."""
        s, t, rev = np.asarray(s, float), np.asarray(t, float), np.asarray(reverse, bool)
        ds = 0.5
        P = self.point(np.stack([s, s + ds, s - ds, s, s]), np.stack([t, t, t, t + ds, t - ds]))
        p0, a, b = P[0], P[1] - P[2], P[3] - P[4]
        fx = a / np.linalg.norm(a, axis=-1, keepdims=True)
        fz = np.cross(fx, b)
        fz /= np.linalg.norm(fz, axis=-1, keepdims=True)
        fy = np.cross(fz, fx)
        sg = np.where(rev, -1.0, 1.0)[:, None]
        T = np.tile(np.eye(4), (len(s), 1, 1))
        T[:, :3, 0], T[:, :3, 1], T[:, :3, 2], T[:, :3, 3] = fx * sg, fy * sg, fz, p0
        return T

    def project(self, x, y):
        """map (x, y) → (s, t). 가장 가까운 샘플 후 접선 방향 보정."""
        d2 = (self.x - x) ** 2 + (self.y - y) ** 2
        i = int(np.argmin(d2))
        th = self.heading[i]
        dx, dy = x - self.x[i], y - self.y[i]
        s = self.s[i] + dx * np.cos(th) + dy * np.sin(th)
        s = float(np.clip(s, self.s[0], self.s[-1]))
        cx, cy, _, th = self.frame(s)
        t = float(-(x - cx) * np.sin(th) + (y - cy) * np.cos(th))
        return s, t

    def summary(self):
        p = self.params
        r = 1 / np.maximum(np.abs(self.kappa), 1e-9)
        grade = np.gradient(self.z, self.s)
        return {
            'seed': p.seed, 'length_m': round(self.length, 1), 'lanes': p.lanes, 'design_speed': p.design_speed,
            'min_radius_m': round(float(r.min()), 1),
            'max_grade_pct': round(float(np.abs(grade).max() * 100), 2),
            'max_superelevation_pct': round(float(np.abs(self.cross_slope).max() * 100), 2),
            'tunnels': [(round(a), round(b)) for a, b in self.tunnels],
            'gantries': [round(g) for g in self.gantries],
            'lane_wear_pct': round(100 * sum(b - a for _, _, a, b, _, _ in self.wear)
                                   / max(1.0, self.length * 2 * (len(self.lines()) - 2)), 1),
            'mgrs_grid': self.mgrs_grid,
            'heading_change_deg': round(float(np.degrees(self.heading[-1] - self.heading[0])), 1),
        }


# ---------------------------------------------------------------- MGRS

def mgrs_grid_of(lat, lon):
    """위경도 → (UTM zone, MGRS 100km 격자 이름, 격자 원점 UTM E/N). 기존 Autoware 맵 local_x로 검증."""
    from pyproj import Proj
    zone = int((lon + 180) // 6) + 1
    E, N = Proj(proj='utm', zone=zone, ellps='WGS84')(lon, lat)
    set_ = zone % 6 or 6
    cols = ['ABCDEFGH', 'JKLMNPQR', 'STUVWXYZ'][(set_ - 1) % 3]
    col = cols[int(E // 1e5) - 1]
    rows = 'ABCDEFGHJKLMNPQRSTUV'
    row = rows[(int(N // 1e5) + (5 if set_ % 2 == 0 else 0)) % 20]
    band = 'CDEFGHJKLMNPQRSTUVWX'[int((lat + 80) // 8)]
    return zone, f'{zone}{band}{col}{row}', (float(E // 1e5 * 1e5), float(N // 1e5 * 1e5))


def local_to_latlon(model, x, y):
    from pyproj import Proj
    p = Proj(proj='utm', zone=model.utm_zone, ellps='WGS84')
    lon, lat = p(np.asarray(x) + model.grid_origin_utm[0], np.asarray(y) + model.grid_origin_utm[1], inverse=True)
    return lat, lon


# ---------------------------------------------------------------- 생성

def _smooth(a, n):
    if n <= 1:
        return a
    k = np.ones(n) / n
    pad = np.r_[np.full(n, a[0]), a, np.full(n, a[-1])]
    return np.convolve(pad, k, mode='same')[n:-n]


def generate(params: RoadParams, ds=1.0) -> RoadModel:
    p = params
    rng = np.random.default_rng(p.seed)
    dz = dict(DESIGN[p.design_speed])
    if p.grade_max > 0:
        dz['grade_max'] = float(p.grade_max)
    V = float(p.design_speed)

    # ---- 평면선형: 직선 → (완화곡선 → 원곡선 → 완화곡선) → 직선 ...
    segs = []                        # (type, length, k0, k1)
    total = 0.0
    first = 250.0 + rng.uniform(0, 150)          # 시작 직선 (로봇 배치 구간)
    segs.append(('straight', first, 0.0, 0.0))
    total += first
    heading_budget = 0.0
    while total < p.length + 200:
        R = float(np.exp(rng.uniform(np.log(dz['r_min']), np.log(4000.0))))
        Lc = float(np.clip(rng.uniform(R / 9, R / 4), dz['l_clothoid_min'], 250.0))
        La = float(rng.uniform(80, 500))
        # 누적 방위 변화를 ±60° 안으로 유지하도록 곡선 방향 선택
        sweep = (La + Lc) / R
        sign = -np.sign(heading_budget) if abs(heading_budget) > np.radians(20) else rng.choice([-1.0, 1.0])
        if abs(heading_budget + sign * sweep) > np.radians(60):
            sign = -sign
        k = sign / R
        segs += [('clothoid', Lc, 0.0, k), ('arc', La, k, k), ('clothoid', Lc, k, 0.0)]
        heading_budget += sign * sweep
        Ls = float(rng.uniform(150, 700))
        segs.append(('straight', Ls, 0.0, 0.0))
        total += 2 * Lc + La + Ls
    kappa = []
    for typ, L, k0, k1 in segs:
        n = max(int(round(L / ds)), 1)
        kappa.append(np.linspace(k0, k1, n, endpoint=False))
    kappa = np.concatenate(kappa)[:int(round(p.length / ds))]     # 목표 길이에서 자름
    N = len(kappa) + 1
    s = np.arange(N) * ds
    kappa = np.r_[kappa, kappa[-1]]
    th0 = rng.uniform(-np.pi, np.pi)
    heading = th0 + np.r_[0.0, np.cumsum((kappa[:-1] + kappa[1:]) / 2 * ds)]
    x = p.start_x + np.r_[0.0, np.cumsum((np.cos(heading[:-1]) + np.cos(heading[1:])) / 2 * ds)]
    y = p.start_y + np.r_[0.0, np.cumsum((np.sin(heading[:-1]) + np.sin(heading[1:])) / 2 * ds)]

    # ---- 종단선형: 구간별 일정 경사 + 이동평균(종단곡선 근사)
    grade = np.zeros(N)
    pos = 0.0
    g = 0.0
    while pos < s[-1]:
        L = rng.uniform(400, 1200)
        g = float(np.clip(g + rng.normal(0, 0.015), -dz['grade_max'], dz['grade_max']))
        grade[(s >= pos) & (s < pos + L)] = g
        pos += L
    grade[s < 150] = 0.0                                      # 시작부 평탄
    grade = _smooth(grade, int(150 / ds))
    z = p.start_elevation + np.r_[0.0, np.cumsum((grade[:-1] + grade[1:]) / 2 * ds)]

    # ---- 횡단경사/편경사: 직선부 바깥쪽 2%, 곡선부는 최소반경에서 6% → 반경이 커질수록 감소(최소 2%)
    R = 1.0 / np.maximum(np.abs(kappa), 1e-9)
    e_abs = np.where(np.abs(kappa) > 1e-6, np.clip(0.06 * (dz['r_min'] / R) ** 0.8, 0.02, 0.06), 0.0)
    sgn = np.sign(kappa)
    cross = np.zeros((N, 2))
    for j, c in enumerate((-1, 1)):
        target = -c * sgn * e_abs                       # 곡선 바깥쪽이 올라감
        w = np.clip(e_abs / 0.02, 0, 1)
        cross[:, j] = np.where(e_abs >= 0.02, target, -0.02 + (target + 0.02) * w)
        cross[:, j] = np.where(np.abs(kappa) < 1e-6, -0.02, cross[:, j])
        cross[:, j] = _smooth(cross[:, j], int(40 / ds))

    zone, grid, origin = mgrs_grid_of(p.region_lat, p.region_lon)
    m = RoadModel(params=p, s=s, x=x, y=y, z=z, heading=heading, kappa=kappa, cross_slope=cross,
                  mgrs_grid=grid, utm_zone=zone, grid_origin_utm=origin,
                  segments=[(t, round(L, 1), round(1 / k1, 1) if k1 else 0.0) for t, L, k0, k1 in segs])

    # ---- 터널: 시작 600m 이후, 서로 300m 이상 떨어지게
    tunnels = []
    for _ in range(p.tunnels):
        for _try in range(30):
            L = rng.uniform(p.tunnel_len_min, p.tunnel_len_max)
            a = rng.uniform(600, max(601, s[-1] - L - 200))
            if a + L > s[-1] - 150:
                continue
            if all(a > b1 + 300 or a + L < b0 - 300 for b0, b1 in tunnels):
                tunnels.append((float(a), float(a + L)))
                break
    m.tunnels = sorted(tunnels)

    # ---- 갠트리 (터널 밖)
    gan = []
    for _ in range(p.gantries):
        for _try in range(30):
            g = float(rng.uniform(150, s[-1] - 100))
            if not m.in_tunnel(g, margin=80) and all(abs(g - h) > 200 for h in gan):
                gan.append(g)
                break
    m.gantries = sorted(gan)

    # ---- 차선 마모 구간 (점선/실선 도색이 사라진 구간)
    wear = []
    if p.wear > 0:
        kinds = ['faded', 'broken', 'sparse']
        w = np.array([p.wear_faded, p.wear_broken, p.wear_sparse], float)
        w = w / w.sum() if w.sum() > 0 else np.array([1.0, 0, 0])
        for side in (-1, 1):
            for ln in m.lines():
                if ln.kind == 'road_border':
                    continue
                a = 0.0
                while a < s[-1]:
                    L = float(rng.uniform(40, 200))
                    if rng.random() < p.wear:
                        k = kinds[int(rng.choice(3, p=w))]
                        wear.append((side, ln.key, a, min(a + L, float(s[-1])), k, float(rng.uniform(0.4, 1.0))))
                    a += L
    m.wear = wear
    return m
