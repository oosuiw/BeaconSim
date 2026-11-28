"""u-blox ZED-F9P RTK GNSS 모델 (상태 머신 + 상관 오차).

상태 (NMEA GGA quality): 4 RTK FIX, 5 RTK FLOAT, 1 SINGLE(3D), 0 NO FIX
  - 개활지: FIX 유지
  - 터널 안(입구에서 depth_lost 이상 들어감): NO FIX
  - 갱문 주변(바깥 approach m 이내)과 터널 입구부: SINGLE (다중경로/차폐)
  - 터널을 빠져나오면: SINGLE → (t_single 후) FLOAT → (재수렴 시간 후) FIX
위치 오차: 상태별 표준편차를 갖는 1차 Gauss-Markov (시간 상관) 잡음.
실환경 열화 (사용자 지정, 0 이면 끔):
  - sigma_scale: 상태별 표준편차 배율
  - drop_rate: 개활지에서도 FIX 를 잃는 빈도 [회/분] (고가·육교·대형차 가림·전리층) → FLOAT 로 떨어졌다 재수렴
  - outlier_rate / outlier_m: 다중경로 튐 [회/분] / 크기 [m] — 1~5 s 동안 위치가 튀지만 수신기가 알리는 정확도(hacc)는 그대로
  - bias_m: 고정 오프셋 [m] (기준국 좌표·측지계 불일치), 방향은 무작위
  - latency_s: 출력 지연 [s] (메시지 시각은 지금, 위치는 그만큼 이전)
"""
from dataclasses import asdict, dataclass

import numpy as np

FIX, FLOAT, SINGLE, NONE = 4, 5, 1, 0
NAME = {FIX: 'RTK FIX', FLOAT: 'RTK FLOAT', SINGLE: 'SINGLE', NONE: 'NO FIX'}
SIGMA = {FIX: (0.015, 0.03), FLOAT: (0.30, 0.50), SINGLE: (1.5, 3.0), NONE: (0.0, 0.0)}   # (수평, 수직) [m]
TAU = {FIX: 5.0, FLOAT: 20.0, SINGLE: 30.0, NONE: 1.0}                                     # 상관 시간 [s]


@dataclass
class GnssParams:
    enabled: bool = True
    topic: str = '/sensing/gnss/fix'
    frame_id: str = 'gnss_link'
    antenna_xyz: tuple = (0.0, 0.0, 0.0)
    rate_hz: float = 10.0
    depth_lost: float = 15.0        # 터널 입구에서 이만큼 들어가면 신호 단절 [m]
    approach: float = 20.0          # 갱문 바깥 이 거리 안은 SINGLE [m]
    t_single_min: float = 1.0       # 터널 진출 후 SINGLE 유지 시간 범위 [s]
    t_single_max: float = 3.0
    t_conv_min: float = 10.0        # FLOAT → FIX 재수렴 시간 범위 [s]
    t_conv_max: float = 60.0
    sigma_scale: float = 1.0        # 상태별 위치 표준편차 배율
    drop_rate: float = 0.0          # 개활지 FIX 끊김 [회/분]
    outlier_rate: float = 0.0       # 다중경로 튐 [회/분]
    outlier_m: float = 1.0          # 튐 크기 [m] (0.5~1.5배 무작위)
    bias_m: float = 0.0             # 고정 오프셋 [m]
    latency_s: float = 0.0          # 출력 지연 [s]

    def to_dict(self):
        return asdict(self)


class GnssF9P:
    def __init__(self, model, gp: GnssParams, seed=0):
        self.m, self.gp = model, gp
        self.rng = np.random.default_rng(seed * 92821 + 7)
        self.state = FIX
        self.err = np.zeros(3)
        self.t_state = 0.0               # 현재 상태 진입 시각
        self.t_need = 0.0                # 다음 상태로 넘어가기까지 필요한 시간
        self.t = 0.0
        ang = self.rng.uniform(0, 2 * np.pi)
        self.bias_dir = np.array([np.cos(ang), np.sin(ang), 0.0])
        self.out_until, self.out_vec = -1.0, np.zeros(3)
        self.hist = []                   # (t, 위치) — 출력 지연용

    def _env(self, s):
        """노면 위치 s → 'open' | 'portal' | 'tunnel'."""
        for a, b in self.m.tunnels:
            if a + self.gp.depth_lost <= s <= b - self.gp.depth_lost:
                return 'tunnel'
            if a - self.gp.approach <= s <= b + self.gp.approach:
                return 'portal'
        return 'open'

    def _set(self, st, t, need=0.0):
        if st != self.state:
            self.state, self.t_state, self.t_need = st, t, need

    def step(self, t, T_wb, s):
        """시각 t, 정답 포즈 T_wb, 노면 s → dict 출력 (rate_hz 주기로 호출)."""
        dt = max(t - self.t, 1e-3) if self.t else 1.0 / self.gp.rate_hz
        self.t = t
        env = self._env(s)
        gp = self.gp
        if env == 'tunnel':
            self._set(NONE, t)
        elif env == 'portal':
            if self.state in (FIX, FLOAT, NONE):
                self._set(SINGLE, t, self.rng.uniform(gp.t_single_min, gp.t_single_max))
        else:
            if self.state == NONE:
                self._set(SINGLE, t, self.rng.uniform(gp.t_single_min, gp.t_single_max))
            elif self.state == SINGLE and t - self.t_state >= self.t_need:
                self._set(FLOAT, t, self.rng.uniform(gp.t_conv_min, gp.t_conv_max))
            elif self.state == FLOAT and t - self.t_state >= self.t_need:
                self._set(FIX, t)
            elif self.state == FIX and gp.drop_rate > 0 and self.rng.random() < gp.drop_rate / 60.0 * dt:
                self._set(FLOAT, t, self.rng.uniform(gp.t_conv_min, gp.t_conv_max) * 0.5)    # 개활지 FIX 끊김
        sh, sv = (x * max(gp.sigma_scale, 0.0) for x in SIGMA[self.state])
        a = np.exp(-dt / TAU[self.state])
        q = np.sqrt(1 - a * a)
        self.err = a * self.err + q * self.rng.standard_normal(3) * np.array([sh, sh, sv])
        truth = T_wb[:3, 3].copy()
        if gp.latency_s > 0:                                   # 출력 지연: latency 만큼 이전 위치
            self.hist.append((t, truth))
            while len(self.hist) > 2 and self.hist[1][0] <= t - gp.latency_s:
                self.hist.pop(0)
            truth = self.hist[0][1]
        if self.state != NONE and gp.outlier_rate > 0 and t > self.out_until \
                and self.rng.random() < gp.outlier_rate / 60.0 * dt:              # 다중경로 튐 시작
            self.out_until = t + self.rng.uniform(1.0, 5.0)
            a = self.rng.uniform(0, 2 * np.pi)
            mag = gp.outlier_m * self.rng.uniform(0.5, 1.5)
            self.out_vec = np.array([np.cos(a) * mag, np.sin(a) * mag, self.rng.normal(0, 0.7 * mag)])
        jump = self.out_vec if t <= self.out_until else 0.0
        p = truth + ((self.err + gp.bias_m * self.bias_dir + jump) if self.state != NONE else 0.0)
        from .roadgen import local_to_latlon
        lat, lon = local_to_latlon(self.m, p[0], p[1])
        return {'t': t, 'status': self.state, 'status_name': NAME[self.state], 'xyz': p, 'truth': T_wb[:3, 3].copy(),
                'lat': float(lat), 'lon': float(lon), 'alt': float(p[2]), 'hacc': sh, 'vacc': sv,
                'valid': self.state != NONE, 'env': env}
