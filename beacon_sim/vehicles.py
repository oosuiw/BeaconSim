"""차종 카탈로그 + 절차적 차량 메시 (외부 모델 파일 없음).

차종마다 실제 국내 판매 차급의 치수(전장·전폭·전고, 공개 제원 기준 근사)와 차체 형태(세단·해치/SUV·MPV·캡오버 트럭·버스)를
코드로 만든다. 실차 디자인(얼굴·로고)을 베끼지 않고, '그 차급처럼 보이는' 실루엣과 비율만 맞춘다.
메시 하나를 화면(Godot)과 라이다(RGL)가 같이 쓰므로 반사점과 정답 박스가 정확히 맞는다.

정답 클래스(kind): car | truck | bus (Autoware CAR/TRUCK/BUS). 세부 차종(model)은 메타데이터.
좌표: 차량 로컬, 원점 = 바닥 중심, x 전방, y 좌측, z 위.
부품 태그: body(차체 도색) glass tire rim trim(검은 플라스틱) cargo(적재함 — 차종별 고정색)
"""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Model:
    name: str
    ko: str
    kind: str                   # car | truck | bus (정답 클래스)
    body: str                   # sedan | hatch | suv | mpv | van | pickup | cargo | box | wing | dump | tank | bus
    L: float
    W: float
    H: float
    weight: float               # 같은 그룹 안 비율
    wheel_r: float = 0.33
    fo: float = 0.95            # 앞 오버행 (앞 끝 → 앞 차축) [m]
    ro: float = 1.05            # 뒤 오버행
    ev: bool = False            # 전기차: 막힌 그릴
    axles: tuple = ()           # 트럭·버스: 앞 끝에서 각 차축까지 [m] (비우면 fo/ro 로 2축)
    dual: tuple = ()            # 복륜(뒤 차축) 여부
    cargo_color: tuple = (0.78, 0.79, 0.80)
    note: str = ''


# 승용·소형 (차량 1대마다 대형차 비율 truck_ratio 를 뺀 나머지에서 고름). 치수 = 공개 제원 근사.
LIGHT = [
    Model('sedan_compact', '준중형 세단', 'car', 'sedan', 4.71, 1.825, 1.42, 0.13, 0.31, 0.90, 1.10,
          note='아반떼·K3급'),
    Model('sedan_mid', '중형 세단', 'car', 'sedan', 4.91, 1.86, 1.445, 0.13, 0.32, 0.95, 1.14, note='쏘나타·K5급'),
    Model('sedan_large', '준대형 세단', 'car', 'sedan', 5.035, 1.88, 1.47, 0.10, 0.33, 0.95, 1.16,
          note='그랜저·K8급'),
    Model('sedan_import', '수입 중형 세단', 'car', 'sedan', 4.95, 1.87, 1.47, 0.06, 0.33, 0.85, 1.10,
          note='5시리즈·E클래스급'),
    Model('niro_ev', '소형 SUV (니로 EV)', 'car', 'hatch', 4.42, 1.825, 1.57, 0.06, 0.33, 0.90, 0.80, ev=True,
          note='니로 EV 2세대 제원 4420/1825/1570, 휠베이스 2720'),
    Model('suv_compact', '소형 SUV', 'car', 'hatch', 4.35, 1.825, 1.58, 0.06, 0.33, 0.90, 0.85, note='코나·셀토스급'),
    Model('ev_crossover', '전기 크로스오버', 'car', 'hatch', 4.655, 1.89, 1.605, 0.04, 0.35, 0.85, 0.80, ev=True,
          note='아이오닉5·EV6급'),
    Model('suv_mid', '중형 SUV', 'car', 'suv', 4.63, 1.865, 1.665, 0.12, 0.35, 0.92, 0.96, note='투싼·스포티지급'),
    Model('suv_large', '준대형 SUV', 'car', 'suv', 4.81, 1.90, 1.70, 0.10, 0.36, 0.95, 1.05, note='쏘렌토·싼타페급'),
    Model('suv_full', '대형 SUV', 'car', 'suv', 4.995, 1.975, 1.75, 0.05, 0.37, 0.95, 1.13, note='팰리세이드급'),
    Model('suv_import', '수입 SUV', 'car', 'suv', 4.92, 2.00, 1.75, 0.03, 0.38, 0.95, 1.00, note='X5·GLE급'),
    Model('mpv', '미니밴', 'car', 'mpv', 5.155, 1.995, 1.775, 0.06, 0.35, 0.95, 1.12, note='카니발급'),
    Model('van_large', '승합 밴', 'car', 'van', 5.255, 1.995, 1.99, 0.03, 0.34, 0.90, 1.08, note='스타리아급'),
    Model('pickup_1t', '1톤 트럭', 'truck', 'pickup', 5.105, 1.74, 1.97, 0.06, 0.33, 0.70, 1.65,
          cargo_color=(0.72, 0.73, 0.75), note='포터·봉고급'),
]

# 대형 (truck_ratio). 국내 고속도로 대형차: 고속·광역버스, 카고, 윙바디, 덤프, 탑차, 탱크로리
HEAVY = [
    Model('bus_coach', '고속버스', 'bus', 'bus', 12.0, 2.49, 3.5, 0.22, 0.52, axles=(2.55, 8.65), dual=(False, True),
          note='유니버스·그랜버드급'),
    Model('truck_box', '중형 탑차', 'truck', 'box', 7.2, 2.3, 3.15, 0.14, 0.45, axles=(1.25, 5.3), dual=(False, True),
          cargo_color=(0.86, 0.87, 0.88), note='4.5~5톤 탑차'),
    Model('truck_cargo', '카고 트럭', 'truck', 'cargo', 9.5, 2.49, 3.1, 0.18, 0.52,
          axles=(1.4, 2.9, 7.0, 8.3), dual=(False, False, True, True), cargo_color=(0.55, 0.57, 0.60),
          note='엑시언트·트라고 카고급'),
    Model('truck_wing', '윙바디 트럭', 'truck', 'wing', 11.3, 2.49, 3.9, 0.20, 0.52,
          axles=(1.4, 2.9, 8.6, 9.9), dual=(False, False, True, True), cargo_color=(0.80, 0.81, 0.83),
          note='25톤 윙바디'),
    Model('truck_dump', '덤프트럭', 'truck', 'dump', 9.0, 2.49, 3.3, 0.16, 0.52,
          axles=(1.4, 2.9, 6.6, 7.9), dual=(False, False, True, True), cargo_color=(0.62, 0.36, 0.12),
          note='25톤 덤프'),
    Model('truck_tank', '탱크로리', 'truck', 'tank', 10.5, 2.49, 3.4, 0.10, 0.52,
          axles=(1.4, 2.9, 8.0, 9.3), dual=(False, False, True, True), cargo_color=(0.84, 0.85, 0.87)),
    # 세미 트레일러: 트랙터(6.0 m, 캡 3.3 m) + 트레일러(13.2 m, 높이 4.0 m), 전체 16.6 m. 트레일러는 킹핀에서 꺾인다
    Model('semi_box', '세미 트레일러 (박스)', 'truck', 'semi', 16.6, 2.49, 4.0, 0.14, 0.52, axles=(1.4, 5.0),
          dual=(False, True), cargo_color=(0.88, 0.88, 0.89), note='트랙터 + 13.2 m 박스 트레일러'),
    Model('semi_container', '컨테이너 트레일러', 'truck', 'semi', 16.6, 2.49, 4.0, 0.10, 0.52, axles=(1.4, 5.0),
          dual=(False, True), cargo_color=(0.16, 0.36, 0.62), note='트랙터 + 40ft 컨테이너'),
]

# 세미 트레일러 치수 (트레일러 좌표: 원점 = 킹핀(연결점), x 전방)
SEMI = dict(tractor_len=6.0, cab_h=3.3, hitch_from_rear=1.3, trailer_len=13.2, kingpin=1.3,
            trailer_axles=(-8.6, -9.9, -11.2))

MODELS = {m.name: m for m in LIGHT + HEAVY}

# 도색 (국내 등록 차량 색 분포 근사: 흰색·회색·검정이 대부분)
CAR_COLORS = [((0.92, 0.92, 0.91), 0.32), ((0.62, 0.63, 0.65), 0.14), ((0.33, 0.34, 0.36), 0.15),
              ((0.03, 0.03, 0.035), 0.17), ((0.07, 0.13, 0.30), 0.07), ((0.50, 0.05, 0.06), 0.04),
              ((0.40, 0.55, 0.65), 0.04), ((0.42, 0.37, 0.30), 0.03), ((0.22, 0.32, 0.30), 0.04)]
CAB_COLORS = [((0.92, 0.92, 0.91), 0.40), ((0.12, 0.25, 0.60), 0.20), ((0.62, 0.63, 0.65), 0.15),
              ((0.60, 0.08, 0.07), 0.08), ((0.15, 0.40, 0.22), 0.07), ((0.85, 0.65, 0.10), 0.05),
              ((0.03, 0.03, 0.035), 0.05)]
BUS_COLORS = [((0.92, 0.92, 0.91), 0.35), ((0.10, 0.20, 0.45), 0.20), ((0.55, 0.06, 0.08), 0.15),
              ((0.12, 0.35, 0.25), 0.10), ((0.03, 0.03, 0.035), 0.10), ((0.62, 0.63, 0.65), 0.10)]


def _pick(rng, items):
    w = np.array([x[1] if isinstance(x, tuple) else x.weight for x in items], float)
    i = int(rng.choice(len(items), p=w / w.sum()))
    return items[i]


TRUCK_BODIES = ('pickup', 'cargo', 'box', 'wing', 'dump', 'tank', 'semi')
RIMS = [(0.62, 0.63, 0.65), (0.25, 0.26, 0.28), (0.05, 0.05, 0.055)]       # 휠 색: 은색·짙은 회색·검정
V_RACK, V_LOAD = 1, 2                                                       # 외관 변형 비트: 루프랙, 적재물


def _style(rng, m):
    """외관 변형: (변형 비트, 휠 색 번호, 유리 선팅 밝기)"""
    var = 0
    if m.body in ('hatch', 'suv', 'mpv', 'van') and rng.random() < 0.25:
        var |= V_RACK
    if m.body in ('pickup', 'cargo') and rng.random() < 0.6:
        var |= V_LOAD
    rim = int(rng.choice(3, p=[0.6, 0.25, 0.15])) if m.kind == 'car' else 0
    tint = float(rng.choice([0.015, 0.035, 0.07], p=[0.4, 0.4, 0.2]))      # 짙은 선팅 ~ 옅은 선팅
    return var, rim, tint


def sample(rng, heavy_ratio):
    """차량 1대 → (Model, 도색, 외관 변형). heavy_ratio = 대형차(버스·트럭) 비율."""
    m = _pick(rng, HEAVY) if rng.random() < heavy_ratio else _pick(rng, LIGHT)
    pal = BUS_COLORS if m.kind == 'bus' else CAB_COLORS if m.body in TRUCK_BODIES else CAR_COLORS
    return m, _pick(rng, pal)[0], _style(rng, m)


def sample_truck(rng):
    """사고 유형 '대형차 차로 막음' 용 대형 트럭 (버스·세미 트레일러 제외: 꺾인 채 멈춘 모습은 따로 다루지 않음)"""
    m = _pick(rng, [h for h in HEAVY if h.kind == 'truck' and h.body != 'semi'])
    return m, _pick(rng, CAB_COLORS)[0], _style(rng, m)


# ------------------------------------------------------------------ 메시 기본 도형

class _MB:
    def __init__(self):
        self.parts = {}

    def _add(self, tag, v, f):
        v = np.asarray(v, np.float64)
        f = np.asarray(f, np.int64)
        c = v.mean(0)                          # 볼록 도형: 법선이 바깥(중심 반대)을 향하게 감기 방향 정리
        a, b, d = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
        n = np.cross(b - a, d - a)
        flip = np.einsum('ij,ij->i', n, (a + b + d) / 3 - c) < 0
        f[flip] = f[flip][:, ::-1]
        lst = self.parts.setdefault(tag, [[], [], 0])
        lst[0].append(v)
        lst[1].append(f + lst[2])
        lst[2] += len(v)

    def box(self, tag, lo, hi):
        x0, y0, z0 = lo
        x1, y1, z1 = hi
        v = [[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0], [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]]
        f = [[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5],
             [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]]
        self._add(tag, v, f)

    def loft(self, tag, xs, secs):
        """x 위치마다 단면 다각형 (K,2)=(y,z) → 닫힌 입체 (양 끝 뚜껑)"""
        K = len(secs[0])
        v = np.concatenate([np.c_[np.full(K, x), s] for x, s in zip(xs, secs)])
        f = []
        for i in range(len(xs) - 1):
            for k in range(K):
                a, b = i * K + k, i * K + (k + 1) % K
                c, d = (i + 1) * K + (k + 1) % K, (i + 1) * K + k
                f += [[a, b, c], [a, c, d]]
        n = len(xs)
        ca, cb = len(v), len(v) + 1                     # 뚜껑 중심점
        v = np.r_[v, [np.r_[xs[0], secs[0].mean(0)]], [np.r_[xs[-1], secs[-1].mean(0)]]]
        for k in range(K):
            f.append([ca, k, (k + 1) % K])
            f.append([cb, (n - 1) * K + (k + 1) % K, (n - 1) * K + k])
        self._add(tag, v, f)

    def cyl_y(self, tag, x, y0, y1, z, r, n=14):
        """y 축 원기둥 (바퀴)"""
        a = np.linspace(0, 2 * np.pi, n, endpoint=False)
        ring = np.c_[x + r * np.cos(a), z + r * np.sin(a)]
        v = np.r_[np.c_[ring[:, 0], np.full(n, y0), ring[:, 1]], np.c_[ring[:, 0], np.full(n, y1), ring[:, 1]],
                  [[x, y0, z], [x, y1, z]]]
        f = []
        for k in range(n):
            k2 = (k + 1) % n
            f += [[k, k2, n + k2], [k, n + k2, n + k], [2 * n, k2, k], [2 * n + 1, n + k, n + k2]]
        self._add(tag, v, f)

    def merge(self, other, dx=0.0):
        for tag, p in other.parts.items():
            off = 0                                 # other 쪽 누적 정점 번호 → 이쪽 번호로
            for v, f in zip(p[0], p[1]):
                lst = self.parts.setdefault(tag, [[], [], 0])
                lst[0].append(v + [dx, 0.0, 0.0])
                lst[1].append(f - off + lst[2])
                lst[2] += len(v)
                off += len(v)

    def out(self):
        return [(np.concatenate(p[0]).astype(np.float32), np.concatenate(p[1]).astype(np.int32), tag)
                for tag, p in self.parts.items()]


def _sec(hw_lo, hw_hi, zlo, zhi, r=0.10):
    """둥근 모서리 단면 (8점): 아래 폭 hw_lo, 위 폭 hw_hi"""
    rr = min(r, 0.3 * max(zhi - zlo, 1e-3))
    pts = [(-hw_lo + rr * 0.6, zlo), (-hw_lo, zlo + rr), (-hw_hi, zhi - rr), (-hw_hi + rr * 0.8, zhi)]
    return np.array(pts + [(-y, z) for y, z in reversed(pts)])


# ------------------------------------------------------------------ 승용 (세단·해치·SUV·MPV·밴)

# 차체 형태: 지상고, 앞 끝 보닛 높이, 벨트라인(창 아래) 높이/전고, 보닛 길이(앞 끝→앞유리 아래), 앞유리 수평 길이,
#           지붕 끝 → 뒤 끝 거리, 뒷유리 아래 → 뒤 끝 거리, 뒷유리 위 높이 비(해치), 트렁크 높이(세단)
CAR_SHAPE = {
    'sedan': dict(zc=0.15, nose=0.66, belt=0.66, hood=1.45, ws=0.85, roof_r=1.60, glass_r=1.05, deck=0.02),
    'hatch': dict(zc=0.17, nose=0.78, belt=0.62, hood=1.20, ws=0.80, roof_r=0.38, glass_r=0.12, deck=None),
    'suv': dict(zc=0.19, nose=0.90, belt=0.60, hood=1.25, ws=0.75, roof_r=0.30, glass_r=0.10, deck=None),
    'mpv': dict(zc=0.17, nose=0.85, belt=0.55, hood=1.05, ws=0.95, roof_r=0.22, glass_r=0.08, deck=None),
    'van': dict(zc=0.18, nose=0.92, belt=0.52, hood=0.80, ws=0.90, roof_r=0.15, glass_r=0.06, deck=None),
}


def _car(m, mb, var=0):
    L, W, H = m.L, m.W, m.H
    S = CAR_SHAPE[m.body]
    zc, nose = S['zc'], S['nose']
    belt = S['belt'] * H
    xf, xr = L / 2, -L / 2
    x_ws = xf - S['hood']                       # 앞유리 아래
    x_rf = x_ws - S['ws']                       # 지붕 앞
    x_rr = xr + S['roof_r']                     # 지붕 뒤
    x_rg = xr + S['glass_r']                    # 뒷유리 아래 (해치는 거의 뒤 끝)
    hw = W / 2
    # 아래 차체: 앞 끝(범퍼) → 보닛 → 벨트라인 → 트렁크/해치 → 뒤 끝
    xs = [xr, xr + 0.06, xr + 0.25, xr + 0.7, x_rg, 0.5 * (x_rg + x_ws), x_ws, xf - 0.55, xf - 0.22, xf - 0.06, xf]
    xs = sorted(set(round(x, 4) for x in xs))
    deck_z = belt + (S['deck'] or 0.0)

    def ztop(x):
        if x >= x_ws:                           # 보닛: 앞 끝 nose → 앞유리 아래 belt (살짝 볼록)
            u = (x - x_ws) / max(xf - x_ws, 1e-3)
            return belt + (nose - belt) * u ** 1.6 - 0.04 * (x >= xf - 1e-6)
        if x <= x_rg:                           # 트렁크(세단) / 해치 아래 (뒤 끝으로 살짝 낮아짐)
            u = (x_rg - x) / max(x_rg - xr, 1e-3)
            return deck_z - 0.06 * u ** 2
        return belt

    def zlo(x):
        e = min(x - xr, xf - x)
        return zc + 0.14 * max(0.0, 1 - e / 0.5)          # 범퍼 아래가 살짝 들림

    def hwx(x, lo):
        e = min(x - xr, xf - x)
        t = 1 - max(0.0, 1 - e / 0.45) ** 2 * (0.14 if x > 0 else 0.10)   # 끝으로 갈수록 좁아짐 (앞이 더 둥금)
        return (hw if lo else hw - 0.05) * t

    mb.loft('body', xs, [_sec(hwx(x, True), hwx(x, False), zlo(x), ztop(x), 0.12) for x in xs])
    # 유리 (그린하우스): 앞유리 아래 → 지붕 → 뒷유리 아래. 지붕은 차체색 판으로 덮음
    gh_top = H - 0.03
    rear_top = deck_z + 0.03 if S['deck'] is not None else H - 0.10

    def gz(x):
        if x >= x_rf:
            return belt + 0.03 + (gh_top - belt - 0.03) * ((x_ws - x) / max(x_ws - x_rf, 1e-3)) ** 0.8
        if x <= x_rr:
            u = (x_rr - x) / max(x_rr - x_rg, 1e-3)
            return gh_top + (rear_top - gh_top) * u ** (1.3 if S['deck'] is not None else 1.0)
        return gh_top + 0.02 * np.sin(np.pi * (x - x_rr) / max(x_rf - x_rr, 1e-3))

    def gsec(x, infl=0.0, zhi=None):
        zt = gz(x) if zhi is None else zhi
        f = (zt - belt) / max(H - belt, 1e-3)
        return _sec(hw - 0.06 + infl, hw - 0.06 - 0.20 * f + infl, belt - 0.02, zt + infl, 0.05)

    gx = np.r_[np.linspace(x_rg, x_rr, 4), np.linspace(x_rr, x_rf, 4)[1:], np.linspace(x_rf, x_ws, 4)[1:]]
    mb.loft('glass', list(gx), [gsec(x) for x in gx])
    rx = np.linspace(x_rr - 0.05, x_rf + 0.08, 5)
    mb.loft('body', list(rx), [_sec(hw - 0.24, hw - 0.30, gz(x) - 0.03, gz(x) + 0.025, 0.04) for x in rx])
    # 기둥 (B·C, 해치·SUV 는 D 까지)
    xb = 0.5 * (x_rf + x_rr) + 0.15
    pillars = [xb]
    if S['deck'] is None:
        pillars += [x_rr + 0.15]
        if m.body in ('mpv', 'van', 'suv'):
            pillars += [0.5 * (xb + x_rr)]
    else:
        pillars += [x_rr - 0.05]
    for px in pillars:
        pxs = [px - 0.06, px + 0.06]
        mb.loft('body', pxs, [gsec(x, 0.012) for x in pxs])
    # 범퍼·그릴 (검은 플라스틱), 사이드미러
    mb.box('trim', [xf - 0.10, -hw * 0.80, zc + 0.04], [xf + 0.012, hw * 0.80, zc + 0.24])
    mb.box('trim', [xr - 0.012, -hw * 0.82, zc + 0.06], [xr + 0.10, hw * 0.82, zc + 0.24])
    if not m.ev:
        mb.box('trim', [xf - 0.03, -W * 0.22, nose - 0.30], [xf + 0.008, W * 0.22, nose - 0.10])
    else:
        mb.box('trim', [xf - 0.03, -W * 0.18, zc + 0.26], [xf + 0.008, W * 0.18, zc + 0.36])
    for sy in (-1, 1):
        y0, y1 = sorted((sy * (hw - 0.05), sy * (hw + 0.10)))
        mb.box('body', [x_ws - 0.30, y0, belt + 0.04], [x_ws - 0.12, y1, belt + 0.17])
    if var & V_RACK:                            # 루프랙 (레일 2줄 + 가로대)
        for sy in (-1, 1):
            y0, y1 = sorted((sy * (hw - 0.36), sy * (hw - 0.30)))
            mb.box('trim', [x_rr + 0.05, y0, H - 0.01], [x_rf - 0.05, y1, H + 0.05])
        for xx in (x_rr + 0.35, x_rf - 0.35):
            mb.box('trim', [xx - 0.025, -hw + 0.30, H + 0.03], [xx + 0.025, hw - 0.30, H + 0.06])
    _wheels(mb, m, [xf - m.fo, xr + m.ro], [False, False], tire_w=0.23, flush=0.04)
    head = (nose - 0.17, nose - 0.04, hw - 0.36, 0.34)
    tail = (belt - 0.20, belt - 0.04, hw - 0.30, 0.32) if S['deck'] is not None else (belt - 0.05, belt + 0.15, hw - 0.22, 0.20)
    return head, tail


def _wheels(mb, m, xs, duals, tire_w=0.3, flush=0.0):
    """flush: 바퀴 바깥면을 차체 옆면(W/2)에서 이만큼 바깥으로 (승용: 휠하우스 없이도 바퀴가 보이게)"""
    r = m.wheel_r
    for x, dual in zip(xs, duals):
        w = tire_w * (1.9 if dual else 1.0)
        for sy in (-1, 1):
            yo = m.W / 2 - 0.03 + flush
            y0, y1 = sorted((sy * yo, sy * (yo - w)))
            mb.cyl_y('tire', x, y0, y1, r, r)
            ya = sy * (yo + 0.004)
            yb = sy * (yo - 0.02)
            mb.cyl_y('rim', x, min(ya, yb), max(ya, yb), r, r * 0.6, 10)


# ------------------------------------------------------------------ 캡오버 트럭 (1톤·카고·탑차·윙·덤프·탱크)

def _truck(m, mb, var=0):
    L, W, H = m.L, m.W, m.H
    hw = W / 2
    xf, xr = L / 2, -L / 2
    small = m.body == 'pickup'
    cab_len = 1.75 if small else 2.25
    cab_h = H if small else min(H, 3.15) if m.body != 'tractor' else H
    cz0 = 0.42 if small else 0.62
    xc = xf - cab_len
    # 캡 (앞 위 모서리가 둥근 상자)
    xs = [xc, xc + 0.08, xf - 0.35, xf - 0.10, xf]
    mb.loft('body', xs, [_sec(hw - (0.03 if x > xf - 0.2 else 0.0), hw - (0.10 if x > xf - 0.2 else 0.04), cz0,
                              cab_h - (0.25 if x >= xf - 1e-6 else 0.08 if x > xf - 0.2 else 0.0), 0.12) for x in xs])
    gz0 = cz0 + 0.52 * (cab_h - cz0)
    mb.box('glass', [xf - 0.03, -hw + 0.14, gz0], [xf + 0.012, hw - 0.14, cab_h - 0.32])
    for sy in (-1, 1):
        y0, y1 = sorted((sy * (hw - 0.01), sy * (hw + 0.008)))
        mb.box('glass', [xf - cab_len * 0.55, y0, gz0], [xf - 0.18, y1, cab_h - 0.35])
        ym0, ym1 = sorted((sy * (hw - 0.02), sy * (hw + 0.14)))
        mb.box('trim', [xf - 0.12, ym0, gz0 - 0.1], [xf - 0.02, ym1, gz0 + 0.45])         # 큰 사이드미러
    mb.box('trim', [xf - 0.10, -hw + 0.02, cz0 - 0.14], [xf + 0.03, hw - 0.02, cz0 + 0.20])   # 앞 범퍼
    if not small:
        mb.box('trim', [xf - 0.02, -hw * 0.6, cz0 + 0.30], [xf + 0.01, hw * 0.6, gz0 - 0.10])  # 그릴
    # 차대 · 뒤 범퍼
    fz = m.wheel_r + 0.05
    mb.box('trim', [xr + 0.15, -0.48, fz - 0.15], [xc + 0.05, 0.48, fz + 0.18])
    mb.box('trim', [xr, -hw + 0.08, fz - 0.10], [xr + 0.10, hw - 0.08, fz + 0.08])
    xb0, xb1 = xr + 0.02, xc - 0.10
    floor = (0.78 if small else 1.22)
    if m.body in ('pickup', 'cargo'):           # 낮은 적재함 (문짝)
        gate = 0.40 if small else 0.62
        mb.box('cargo', [xb0, -hw, floor - 0.08], [xb1, hw, floor])
        for sy in (-1, 1):
            y0, y1 = sorted((sy * hw, sy * (hw - 0.05)))
            mb.box('cargo', [xb0, y0, floor], [xb1, y1, floor + gate])
        mb.box('cargo', [xb0, -hw, floor], [xb0 + 0.05, hw, floor + gate])
        mb.box('cargo', [xb1 - 0.06, -hw, floor], [xb1, hw, floor + gate + (0.25 if not small else 0.55)])
        if var & V_LOAD:                        # 적재물 (방수포 덮은 짐)
            ln = (xb1 - xb0) * (0.55 if small else 0.75)
            mb.box('load', [xb1 - 0.12 - ln, -hw + 0.12, floor], [xb1 - 0.12, hw - 0.12, floor + (0.75 if small else 1.15)])
    elif m.body in ('box', 'wing'):             # 탑차·윙바디: 큰 상자
        mb.box('cargo', [xb0, -hw, floor - 0.12], [xb1, hw, H])
        mb.box('trim', [xb0, -hw - 0.005, floor - 0.12], [xb1, hw + 0.005, floor])
        if m.body == 'wing':
            mb.box('trim', [xb0, -hw - 0.006, H - 0.10], [xb1, hw + 0.006, H - 0.04])
    elif m.body == 'dump':                      # 덤프: 두꺼운 벽 적재함 + 캡 보호판
        top = H - 0.35
        mb.box('cargo', [xb0, -hw, floor], [xb1, hw, floor + 0.15])
        for sy in (-1, 1):
            y0, y1 = sorted((sy * hw, sy * (hw - 0.10)))
            mb.box('cargo', [xb0, y0, floor], [xb1, y1, top])
        mb.box('cargo', [xb0, -hw, floor], [xb0 + 0.10, hw, top])
        mb.box('cargo', [xb1 - 0.12, -hw, floor], [xb1, hw, H])
        mb.box('cargo', [xb1 - 0.12, -hw + 0.05, H - 0.12], [xb1 + 0.75, hw - 0.05, H])
    elif m.body == 'tractor':                   # 세미 트레일러 트랙터: 오륜(커플러) 판
        hx = xr + SEMI['hitch_from_rear']
        mb.box('trim', [hx - 0.6, -0.9, fz + 0.45], [hx + 0.6, 0.9, fz + 0.62])
    elif m.body == 'tank':                      # 탱크로리: 타원 원통
        zc_, rz = 0.5 * (floor + H), 0.5 * (H - floor)
        a = np.linspace(0, 2 * np.pi, 18, endpoint=False)
        ring = np.c_[(hw - 0.05) * np.cos(a), zc_ + rz * np.sin(a)]
        txs = [xb0 + 0.10, xb0 + 0.25, xb1 - 0.25, xb1 - 0.05]
        secs = [np.c_[ring[:, 0] * k, zc_ + (ring[:, 1] - zc_) * k] for k in (0.92, 1.0, 1.0, 0.92)]
        mb.loft('cargo', txs, secs)
        mb.box('trim', [xb0 + 0.3, -0.6, floor - 0.12], [xb1 - 0.3, 0.6, floor + 0.05])
    ax = [xf - a for a in m.axles] if m.axles else [xf - m.fo, xr + m.ro]
    du = list(m.dual) if m.dual else [False] + [True] * (len(ax) - 1)
    _wheels(mb, m, ax, du, tire_w=0.22 if small else 0.30)
    head = (cz0 + 0.02, cz0 + 0.17, hw - 0.38, 0.30)
    tail = (fz - 0.05, fz + 0.12, hw - 0.25, 0.25)
    return head, tail


# ------------------------------------------------------------------ 버스

def _bus(m, mb):
    L, W, H = m.L, m.W, m.H
    hw = W / 2
    xf, xr = L / 2, -L / 2
    z0 = 0.33
    xs = [xr, xr + 0.12, xf - 0.40, xf - 0.08, xf]
    mb.loft('body', xs, [_sec(hw - (0.04 if x in (xr, xf) else 0.0), hw - (0.08 if x in (xr, xf) else 0.03), z0,
                              H - (0.18 if x >= xf - 1e-6 else 0.05 if x > xf - 0.2 else 0.0), 0.16) for x in xs])
    for sy in (-1, 1):                          # 옆 창 띠
        y0, y1 = sorted((sy * (hw - 0.02), sy * (hw + 0.006)))
        mb.box('glass', [xr + 0.55, y0, 1.55], [xf - 1.15, y1, H - 0.55])
        mb.box('glass', [xf - 1.0, y0, 0.95], [xf - 0.15, y1, H - 0.55])      # 앞문 (유리)
    mb.box('glass', [xf - 0.03, -hw + 0.10, 1.05], [xf + 0.012, hw - 0.10, H - 0.30])    # 앞유리
    mb.box('glass', [xr - 0.012, -hw + 0.25, 2.05], [xr + 0.03, hw - 0.25, H - 0.45])    # 뒷창
    mb.box('trim', [xf - 0.10, -hw + 0.02, z0 - 0.02], [xf + 0.03, hw - 0.02, z0 + 0.35])
    mb.box('trim', [xr - 0.03, -hw + 0.02, z0 - 0.02], [xr + 0.10, hw - 0.02, z0 + 0.35])
    for sy in (-1, 1):
        ym0, ym1 = sorted((sy * (hw - 0.02), sy * (hw + 0.30)))
        mb.box('trim', [xf - 0.10, ym0, H - 0.75], [xf + 0.25, ym1, H - 0.65])          # 버스 미러 팔
    ax = [xf - a for a in m.axles]
    _wheels(mb, m, ax, list(m.dual), tire_w=0.30, flush=0.035)
    return (0.55, 0.75, hw - 0.40, 0.35), (0.75, 1.05, hw - 0.20, 0.18)


# ------------------------------------------------------------------ 세미 트레일러

def _tractor_model(m):
    from dataclasses import replace
    return replace(m, L=SEMI['tractor_len'], H=SEMI['cab_h'], body='tractor')


def hitch_x(name):
    """세미 트레일러: 차량 좌표(전체 길이 중심 원점)에서 킹핀 x [m]. 아니면 None"""
    m = MODELS[name]
    if m.body != 'semi':
        return None
    return m.L / 2 - SEMI['tractor_len'] + SEMI['hitch_from_rear']


def _trailer(m):
    """트레일러 부품 (트레일러 좌표: 원점 = 킹핀, x 전방) + 등화 정보"""
    mb = _MB()
    hw = m.W / 2
    x0, x1 = SEMI['kingpin'] - SEMI['trailer_len'], SEMI['kingpin']
    floor, H = 1.25, m.H
    mb.box('cargo', [x0, -hw, floor], [x1, hw, H])
    if m.name == 'semi_container':              # 컨테이너 골 (세로 보강재)
        for xx in np.linspace(x0 + 0.4, x1 - 0.4, 8):
            mb.box('cargo', [xx - 0.04, -hw - 0.012, floor + 0.05], [xx + 0.04, hw + 0.012, H - 0.05])
    mb.box('trim', [x0, -hw + 0.05, floor - 0.25], [x1 - 0.3, hw - 0.05, floor])          # 하부 프레임
    mb.box('trim', [x0 - 0.02, -hw + 0.10, 0.55], [x0 + 0.10, hw - 0.10, 0.70])           # 뒤 범퍼 (언더런 가드)
    for sy in (-1, 1):                                                                      # 옆 보호대
        y0, y1 = sorted((sy * (hw - 0.04), sy * (hw - 0.08)))
        mb.box('trim', [-7.6, y0, 0.55], [-1.0, y1, 0.75])
    _wheels(mb, m, list(SEMI['trailer_axles']), [True] * 3, tire_w=0.30)
    return mb, (0.80, 1.00, hw - 0.25, 0.25), x0


_CACHE = {}


def build(name, var=0):
    """차종 → (부품 [(v, f, tag)], 등화 {'head': (z0,z1,y중심,폭), 'tail': ...}). 결과는 캐시.
    세미 트레일러는 트랙터만 (트레일러는 trailer())."""
    key = (name, var)
    if key in _CACHE:
        return _CACHE[key]
    m = MODELS[name]
    mb = _MB()
    if m.body == 'bus':
        head, tail = _bus(m, mb)
    elif m.body == 'semi':
        t = _MB()
        head, tail = _truck(_tractor_model(m), t, var)
        mb.merge(t, m.L / 2 - SEMI['tractor_len'] / 2)
        tail = None
    elif m.body in TRUCK_BODIES:
        head, tail = _truck(m, mb, var)
    else:
        head, tail = _car(m, mb, var)
    _CACHE[key] = (mb.out(), {'head': head, 'tail': tail})
    return _CACHE[key]


def trailer(name):
    """세미 트레일러 → {'parts', 'lamps', 'hitch_x', 'L2'(킹핀~뒤 차축 묶음), 'center', 'size'}, 아니면 None.
    부품·등화는 트레일러 좌표 (원점 = 킹핀)."""
    m = MODELS[name]
    if m.body != 'semi':
        return None
    key = ('trailer', name)
    if key not in _CACHE:
        mb, tail, x0 = _trailer(m)
        x1 = SEMI['kingpin']
        _CACHE[key] = {'parts': mb.out(), 'lamps': _lamp_boxes(None, tail, None, x0, m.W),
                       'hitch_x': hitch_x(name), 'L2': float(-np.mean(SEMI['trailer_axles'])),
                       'center': (0.5 * (x0 + x1), 0.0, 0.0), 'size': (x1 - x0, m.W, m.H)}
    return _CACHE[key]


def hitch_T(hx, phi):
    """차량 좌표 → 트레일러 좌표 변환: 킹핀(x = hx)으로 이동 후 z 축 회전 phi (꺾임각)"""
    c, s = np.cos(phi), np.sin(phi)
    H = np.eye(4)
    H[:2, :2] = [[c, -s], [s, c]]
    H[0, 3] = hx
    return H


def tractor_box(name):
    """세미 트레일러 트랙터 정답 박스 (차량 좌표 중심 x, 길이) — 정답 객체용"""
    m = MODELS[name]
    return m.L / 2 - SEMI['tractor_len'] / 2, SEMI['tractor_len'], SEMI['cab_h']


def mesh(name, var=0, with_trailer=True):
    """라이다·예전 렌더러용: 모든 부품을 합친 (v, f). 세미 트레일러는 꺾임 0 의 트레일러까지 (with_trailer)"""
    parts, _ = build(name, var)
    parts = list(parts)
    tr = trailer(name) if with_trailer else None
    if tr is not None:
        parts += [(v + np.float32([tr['hitch_x'], 0, 0]), f, t) for v, f, t in tr['parts']]
    vs, fs, n = [], [], 0
    for v, f, _t in parts:
        vs.append(v)
        fs.append(f + n)
        n += len(v)
    return np.concatenate(vs).astype(np.float32), np.concatenate(fs).astype(np.int32)


HEAD_COL, TAIL_COL, BRAKE_COL, SIG_COL = (1.0, 0.97, 0.85), (1.0, 0.08, 0.05), (1.0, 0.02, 0.01), (1.0, 0.55, 0.05)


def _lamp_boxes(head, tail, xf, xr, W):
    """등화 상자 → [(v, f, color, kind)] kind: head | tail | brake | sigl | sigr (sigl = 왼쪽 방향지시등)"""
    out = []
    hw = W / 2

    def box(x, outward, y0, y1, z0, z1, col, kind, depth=0.06, stick=0.018):
        mb = _MB()
        xa, xb = (x - depth, x + stick) if outward > 0 else (x - stick, x + depth)
        mb.box('lamp', [xa, y0, z0], [xb, y1, z1])
        v, f, _ = mb.out()[0]
        out.append((v, f, col, kind))

    if head is not None:
        z0, z1, yc, w = head
        for sy in (-1, 1):
            box(xf, 1, sy * yc - w / 2, sy * yc + w / 2, z0, z1, HEAD_COL, 'head')
            yo = min(hw - 0.04, yc + w / 2 + 0.10)                       # 앞 방향지시등: 전조등 바깥쪽
            box(xf, 1, *sorted((sy * (yo - 0.10), sy * yo)), z0, z1, SIG_COL, 'sigl' if sy > 0 else 'sigr', 0.05, 0.012)
    if tail is not None:
        z0, z1, yc, w = tail
        for sy in (-1, 1):
            box(xr, -1, sy * yc - w / 2, sy * yc + w / 2, z0, z1, TAIL_COL, 'tail')
            box(xr, -1, sy * yc - w / 2 + 0.02, sy * yc + w / 2 - 0.02, z0 + 0.01, z1 - 0.01, BRAKE_COL, 'brake',
                0.03, 0.026)                                             # 브레이크등: 후미등 바로 바깥 (밟을 때만 보임)
            yo = min(hw - 0.03, yc + w / 2 + 0.10)                       # 뒤 방향지시등
            box(xr, -1, *sorted((sy * (yo - 0.10), sy * yo)), z0, z1, SIG_COL, 'sigl' if sy > 0 else 'sigr', 0.05, 0.012)
    return out


def lamps(name, var=0, kinds=False):
    """등화 상자 → [(v, f, color)] (kinds 면 [(v, f, color, kind)]). 세미 트레일러는 트랙터 앞 등화만 (뒤는 trailer())"""
    m = MODELS[name]
    _, L = build(name, var)
    out = _lamp_boxes(L['head'], L['tail'], m.L / 2, -m.L / 2, m.W)
    if not kinds:
        out = [(v, f, c) for v, f, c, k in out if k in ('head', 'tail')]
    return out


def head_z(name):
    z0, z1, _, _ = build(name)[1]['head']
    return 0.5 * (z0 + z1)


def wheelbase(name):
    """앞 차축 ~ 뒤 차축(묶음 중심) 거리 [m] — 차선 변경 기구학(자전거 모델)용"""
    m = MODELS[name]
    if m.axles:
        front = [a for a, d in zip(m.axles, m.dual or (False,) * len(m.axles)) if not d] or [m.axles[0]]
        rear = [a for a, d in zip(m.axles, m.dual or (False,) * len(m.axles)) if d] or [m.axles[-1]]
        return float(np.mean(rear) - np.mean(front))
    return m.L - m.fo - m.ro

