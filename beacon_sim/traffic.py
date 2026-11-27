"""2차 사고 예방 시나리오: 사고 현장 + 역주행 비콘 + 접근 차량 반응 교통 시뮬레이션.

좌표 규약
  side  : -1 하행(t<0, 교통 흐름 +s), +1 상행(t>0, 교통 흐름 -s)
  dir   : 교통 흐름 방향의 s 부호 (side -1 → +1, side +1 → -1)
  u     : 진행 좌표 = dir * s (차량이 갈수록 증가)
  lane  : 0 = 1차로(중앙분리대 쪽). 차로변경 중에는 실수값

배치 (교통 흐름 기준):  [접근 차량] → → → [비콘(역방향)] [사고 차량]
  시뮬레이터는 비콘을 사고 차량 후미에 교통 흐름 반대 방향으로 '배치'만 한다.
  이동은 상위 제어기가 /cmd_vel 로 한다.
  운전자 입장에서 비콘이 먼저 보이고 그 앞에 사고 현장이 있다.

접근 차량 거동 (비콘 쪽 차로): IDM 추종 + 비콘 인지 시 운전자 반응 + SCC(적응형 순항)
  SCC     : 탑재 차량은 레이더로 앞차를 추종 (차간 시간·최대 감속도는 상용차 범위에서 랜덤).
            상용 SCC처럼 정지 물체는 무시하고, 비콘이 교통 흐름 방향으로 움직이면(사고 현장 쪽으로 후진)
            추종 대상으로 잡는다. 한 번 잡으면 비콘이 멈춰도 계속 추종(추적 유지).
  lane_change : 막힌 차로면 인접 빈 차로로 변경 (간격 수용 판단)
  brake       : 저속으로 감속 후, 장애물 가까이에서 차로 변경 (안 되면 정지)
  crash       : 무반응(부주의) → 비콘/사고 차량에 추돌
"""
from dataclasses import asdict, dataclass

import numpy as np

from .roadgen import RoadModel

BEHAVIORS = ('none', 'lane_change', 'brake', 'crash')
B_NONE, B_LC, B_BRAKE, B_CRASH = range(4)
PALETTE = [(0.75, 0.1, 0.1), (0.1, 0.2, 0.6), (0.85, 0.85, 0.85), (0.1, 0.1, 0.1), (0.5, 0.5, 0.55),
           (0.9, 0.9, 0.9), (0.2, 0.35, 0.2)]
HIDDEN_Z = -1000.0
BEACON_HALF_LEN, BEACON_HALF_W = 0.30, 0.25


@dataclass
class ScenarioParams:
    seed: int = 1
    side: str = 'random'          # random | down(하행) | up(상행)
    lane: int = -1                # -1 랜덤, 0 = 1차로(중앙분리대 쪽)
    accident_s: float = -1.0      # 사고 위치 s [m], -1 이면 accident_where 로 결정
    accident_where: str = 'random'  # random | open(개활지) | tunnel_entry | tunnel(터널 깊숙이) | tunnel_exit
    beacon_light: bool = True     # 비콘 경광등 (야간 인지 거리에 영향)
    accident_vehicles: int = 2    # 1~3
    start_gap: float = 6.0        # 비콘 배치 위치: 사고 차량 후미에서 상류로 [m]
    tower_auto: bool = True        # 접이식 센서타워: tower_deploy_at 에 자동으로 펴기 (끄면 버튼·U 키·/beacon/deploy 로만)
    tower_deploy_at: float = 0.0   # 자동 펴기 시작 시각 [s]

    def to_dict(self):
        return asdict(self)


@dataclass
class TrafficParams:
    enabled: bool = True          # False = 기능 검증 모드 (접근/반대 방향 차량 없음)
    flow: float = 900.0           # 비콘 쪽 차로당 교통량 [대/h]
    flow_opposite: float = 600.0  # 반대 방향 차로당 교통량 [대/h]
    speed_mean: float = 27.0      # 희망속도 평균 [m/s]
    speed_std: float = 3.0
    truck_ratio: float = 0.2
    p_lane_change: float = 0.6    # 반응 비율 (합으로 정규화)
    p_brake: float = 0.3
    p_crash: float = 0.1
    notice_min: float = 60.0      # 비콘 인지 거리 [m]
    notice_max: float = 250.0
    max_vehicles: int = 200
    p_scc: float = 0.4            # SCC 탑재(작동) 비율
    scc_gap_min: float = 1.0      # 차간 시간 설정 범위 [s]
    scc_gap_max: float = 2.5
    scc_decel_min: float = 2.5    # SCC 최대 감속도 범위 [m/s²] (상용 ACC 수준)
    scc_decel_max: float = 5.0
    scc_range: float = 150.0      # 레이더 추종 거리 [m]
    scc_moving_min: float = 0.5   # 비콘을 '움직이는 물체'로 인식하는 최소 속도 (교통 흐름 방향) [m/s]

    def to_dict(self):
        return asdict(self)


@dataclass
class VehicleSpec:
    kind: str
    length: float
    width: float
    height: float
    color: tuple

    @property
    def key(self):
        return (self.kind, self.length, self.width, self.height, self.color)


def _spec(rng, truck_ratio):
    if rng.random() < truck_ratio:
        return VehicleSpec('truck', 10.0, 2.5, 3.6, (0.85, 0.85, 0.8))
    return VehicleSpec('car', 4.6, 1.85, 1.45, PALETTE[rng.integers(len(PALETTE))])


def resolve_seed(seed):
    """seed < 0 → 실행마다 랜덤 (결정된 값을 로그/토픽으로 남겨 재현 가능)."""
    return int(np.random.default_rng().integers(0, 2 ** 31 - 1)) if seed is None or int(seed) < 0 else int(seed)


def slot_specs(tp: TrafficParams, seed):
    """차량 슬롯 외형 (결정적). 심/카메라/앱이 같은 슬롯 번호 → 같은 외형."""
    rng = np.random.default_rng(seed * 104729 + 3)
    return [_spec(rng, tp.truck_ratio) for _ in range(tp.max_vehicles)]


def _rotz(a):
    c, s = np.cos(a), np.sin(a)
    R = np.eye(4)
    R[:2, :2] = [[c, -s], [s, c]]
    return R


class Scenario:
    def __init__(self, m: RoadModel, sp: ScenarioParams):
        self.m, self.sp = m, sp
        rng = np.random.default_rng(sp.seed * 7919 + 17)
        side = {'down': -1, 'up': 1}.get(sp.side, int(rng.choice([-1, 1])))
        self.side = side
        self.dir = 1 if side < 0 else -1
        lanes = m.params.lanes
        self.lane = int(rng.integers(lanes)) if sp.lane < 0 else int(min(sp.lane, lanes - 1))
        self.lane_t = m.lane_center_t(self.lane, side)
        L = m.length
        # 사고 위치: 상류에 접근 구간 600m, 하류에 60m 여유
        need_up, need_dn = 600.0, 60.0
        s_tun = self._where(m, sp.accident_where, rng, need_up, need_dn) if sp.accident_s < 0 else None
        if sp.accident_s >= 0:
            s_acc = float(np.clip(sp.accident_s, 0, L))
        elif s_tun is not None:
            s_acc = s_tun
        else:
            u_lo, u_hi = need_up, L - need_dn
            if u_hi <= u_lo:
                u_lo = u_hi = L * 0.6
            u = float(rng.uniform(u_lo, u_hi))
            s_acc = u if self.dir > 0 else L - u
        self.accident_s = s_acc

        # 사고 차량: 비콘 차로에 1대(비스듬히), 나머지는 인접 차로/길어깨, 약간 하류
        self.accident = []                                  # [(VehicleSpec, s, t, yaw_off)]
        n_acc = int(np.clip(sp.accident_vehicles, 1, 3))
        cand = [self.lane]                 # 막히는 차로: 최대 lanes-1개 (최소 한 차로는 통행 가능)
        for c in (self.lane - 1, self.lane + 1):
            if 0 <= c < lanes and len(cand) < lanes - 1:
                cand.append(c)
        for k in range(n_acc):
            spec = _spec(rng, 0.0)          # 사고 차량은 승용차 (비스듬한 대형차는 옆 차로까지 막음)
            if k == 0:
                ln, ds = self.lane, 0.0
            elif k < len(cand):
                ln, ds = cand[k], rng.uniform(4, 12)
            else:
                ln, ds = None, rng.uniform(8, 20)                                    # 바깥 길어깨
            if ln is None:
                p_ = m.params
                t = side * (m.edge_t - p_.outer_shoulder / 2 + 0.4)
                yaw = float(rng.choice([-1, 1]) * rng.uniform(np.radians(3), np.radians(10)))
            else:
                t = m.lane_center_t(ln, side) + rng.uniform(-0.2, 0.2)
                yaw = float(rng.choice([-1, 1]) * rng.uniform(np.radians(8), np.radians(25)))
            s = s_acc + self.dir * ds
            self.accident.append((spec, float(s), float(t), yaw))
        first = self.accident[0][0]
        self.beacon_s0 = s_acc - self.dir * (first.length / 2 + sp.start_gap)
        self.beacon_s0 = float(np.clip(self.beacon_s0, 0, L))

    def _where(self, m, where, rng, need_up, need_dn):
        """accident_where → 사고 위치 s (조건을 만족하는 후보가 없으면 None → 랜덤)."""
        L = m.length
        u_ok = lambda s: need_up <= (s if self.dir > 0 else L - s) <= L - need_dn      # noqa: E731
        if where == 'open':
            for _ in range(200):
                s = float(rng.uniform(0, L))
                if u_ok(s) and not m.in_tunnel(s, margin=150):
                    return s
            return None
        if where in ('tunnel_entry', 'tunnel', 'tunnel_exit') and m.tunnels:
            cands = []
            for a, b in m.tunnels:
                entry, exit_ = (a, b) if self.dir > 0 else (b, a)
                s = {'tunnel_entry': entry + self.dir * 25.0, 'tunnel_exit': exit_ - self.dir * 25.0,
                     'tunnel': (a + b) / 2}[where]
                if u_ok(s):
                    cands.append(s)
            if cands:
                return float(cands[int(rng.integers(len(cands)))])
        return None

    # ---- 비콘 (역주행: 교통 흐름 반대 방향을 바라봄)
    @property
    def beacon_reverse(self):
        """model.pose(reverse=...) 인자. 비콘 진행 방향 = -dir."""
        return self.dir > 0

    def beacon_pose(self, s, t=None):
        return self.m.pose(s, self.lane_t if t is None else t, reverse=self.beacon_reverse)

    def accident_poses(self):
        out = []
        for spec, s, t, yaw in self.accident:
            T = self.m.pose(s, t, reverse=self.dir < 0) @ _rotz(yaw)
            out.append((spec, T))
        return out

    def summary(self):
        return {'side': '하행' if self.side < 0 else '상행', 'lane': self.lane + 1,
                'accident_s': round(self.accident_s, 1), 'beacon_start_s': round(self.beacon_s0, 1),
                'accident_vehicles': len(self.accident)}


class TrafficSim:
    """고정 스텝(DT) 차량 시뮬레이션. 같은 입력(비콘 위치 시퀀스)이면 결과가 같다."""
    DT = 0.05
    IDM = dict(a=1.5, b=3.0, s0=2.0, T=1.2, b_max=8.0)
    LC_TIME = 3.5

    def __init__(self, m: RoadModel, scn: Scenario, tp: TrafficParams, seed=1):
        self.m, self.scn, self.tp = m, scn, tp
        self.rng = np.random.default_rng(seed * 6007 + 11)
        self.specs = slot_specs(tp, seed)
        N = tp.max_vehicles
        self.N = N
        self.len = np.array([s.length for s in self.specs])
        self.wid = np.array([s.width for s in self.specs])
        self.active = np.zeros(N, bool)
        self.side = np.zeros(N, int)
        self.u = np.zeros(N)
        self.v = np.zeros(N)
        self.v0 = np.zeros(N)
        self.lane = np.zeros(N)                 # 현재 횡위치 (차로 번호, 실수)
        self.lane_from = np.zeros(N, int)
        self.lane_to = np.zeros(N, int)
        self.lc = np.ones(N)                    # 차로변경 진행률 (1 = 완료)
        self.beh = np.zeros(N, int)
        self.noticed = np.zeros(N, bool)
        self.crashed = np.zeros(N, bool)
        self.notice_d = np.zeros(N)
        self.slow_v = np.zeros(N)
        self.lat_vel = np.zeros(N)
        self.scc = np.zeros(N, bool)
        self.scc_T = np.zeros(N)
        self.scc_b = np.zeros(N)
        self.scc_lock = np.zeros(N, bool)      # 비콘을 추종 대상으로 잡았는지
        self.scc_target = np.full(N, -1)       # -1 없음, -2 비콘, j 앞차 슬롯
        self.t = 0.0
        self.events = []                        # (t, 종류, 슬롯, 내용)
        self.counts = {'spawned': 0, 'noticed': 0, 'lane_change': 0, 'brake': 0, 'crash': 0, 'beacon_hit': 0,
                       'scc_lock': 0}
        self.lanes = m.params.lanes
        self.L = m.length
        self.next_spawn = {}
        for side in (-1, 1):
            for ln in range(self.lanes):
                self.next_spawn[(side, ln)] = self._exp(side)
        self.beacon_u = scn.dir * scn.beacon_s0
        self.beacon_t = scn.lane_t
        self.beacon_speed = 0.0                 # 차량 쪽으로 다가오는 속도 (+)
        self.beacon_hits = []                   # (t, 차량, 충돌 속도 m/s) — 엔진이 비콘 파손 처리
        self.notice_scale = 1.0                 # 환경·경광등에 따른 인지 거리 배율 (엔진이 설정)
        self.visibility = 1e9                   # 가시거리 상한 [m]
        self.los_fn = None                      # (eyes (K,3), target (3,)) → 보임 여부 (K,) — 엔진이 RGL로 제공
        self.notice_at = {}                     # 차량 → 인지 시 비콘까지 거리 (평가용)
        self._pose_cache = None
        self._rev = 0                         # 상태가 바뀔 때마다 증가 → poses_at 캐시 무효화
        self._pa_cache = (None, None)
        self.beacon_vu = 0.0                    # 교통 흐름 방향 속도 (+ = 사고 현장 쪽으로 후진)
        if tp.enabled:
            self._prefill()

    # ---- 유틸
    def _dir(self, side):
        return 1 if side < 0 else -1

    def _u_range(self, side):
        return (0.0, self.L) if side < 0 else (-self.L, 0.0)

    def _flow(self, side):
        if not self.tp.enabled:
            return 0.0
        return self.tp.flow if side == self.scn.side else self.tp.flow_opposite

    def _exp(self, side):
        f = self._flow(side)
        return self.rng.exponential(3600.0 / f) if f > 0 else np.inf

    def _lane_t(self, side, lane):
        return self.m.lane_center_t(lane, side)

    def _activate(self, side, lane, u, v=None):
        self._rev += 1
        free = np.nonzero(~self.active)[0]
        if len(free) == 0:
            return None
        i = int(free[self.rng.integers(len(free))])
        self.scc_target[self.scc_target == i] = -1      # 재사용 슬롯을 가리키던 SCC 대상 해제
        self.active[i] = True
        self.side[i] = side
        self.u[i] = u
        self.v0[i] = max(10.0, self.rng.normal(self.tp.speed_mean, self.tp.speed_std))
        if self.specs[i].kind == 'truck':
            self.v0[i] = min(self.v0[i], 25.0)
        self.v[i] = self.v0[i] if v is None else v
        self.lane[i] = lane
        self.lane_from[i] = self.lane_to[i] = lane
        self.lc[i] = 1.0
        self.noticed[i] = False
        self.crashed[i] = False
        self.lat_vel[i] = 0.0
        if side == self.scn.side:
            p = np.array([self.tp.p_lane_change, self.tp.p_brake, self.tp.p_crash], float)
            p = p / p.sum() if p.sum() > 0 else np.array([1.0, 0, 0])
            self.beh[i] = 1 + int(self.rng.choice(3, p=p))
        else:
            self.beh[i] = B_NONE
        self.notice_d[i] = self.rng.uniform(self.tp.notice_min, max(self.tp.notice_min, self.tp.notice_max))
        self.slow_v[i] = self.rng.uniform(8.0, 15.0)
        tp = self.tp
        self.scc[i] = self.rng.random() < tp.p_scc
        self.scc_T[i] = self.rng.uniform(tp.scc_gap_min, max(tp.scc_gap_min, tp.scc_gap_max))
        self.scc_b[i] = self.rng.uniform(tp.scc_decel_min, max(tp.scc_decel_min, tp.scc_decel_max))
        self.scc_lock[i] = False
        self.scc_target[i] = -1
        self.counts['spawned'] += 1
        return i

    def _prefill(self):
        """시작 시 도로에 차량 배치 (비콘~사고 구간과 그 상류 300m는 비움)."""
        acc_u = self.scn.dir * self.scn.accident_s
        for side in (-1, 1):
            u0, u1 = self._u_range(side)
            f = self._flow(side)
            if f <= 0:
                continue
            for ln in range(self.lanes):
                u = u0 + self.rng.uniform(0, 30)
                while u < u1 - 10:
                    skip = side == self.scn.side and (self.beacon_u - 300.0 < u < acc_u + 30.0)
                    if not skip:
                        if self._activate(side, ln, u) is None:
                            return
                    u += max(25.0, self.rng.exponential(self.tp.speed_mean * 3600.0 / f))

    def obstacles(self):
        """(side, u, half_len, t, half_w, kind) 목록: 비콘, 사고 차량, 추돌로 멈춘 차량."""
        obs = [(self.scn.side, self.beacon_u, BEACON_HALF_LEN, self.beacon_t, BEACON_HALF_W, 'beacon')]
        for spec, s, t, yaw in self.scn.accident:
            hl = abs(np.cos(yaw)) * spec.length / 2 + abs(np.sin(yaw)) * spec.width / 2
            hw = abs(np.sin(yaw)) * spec.length / 2 + abs(np.cos(yaw)) * spec.width / 2
            obs.append((self.scn.side, self.scn.dir * s, hl, t, hw, 'accident'))
        for i in np.nonzero(self.active & self.crashed)[0]:
            obs.append((int(self.side[i]), self.u[i], self.len[i] / 2, self._lane_t(self.side[i], self.lane[i]),
                        self.wid[i] / 2, 'vehicle'))
        return obs

    # ---- 한 스텝
    def step(self, beacon_s=None, beacon_t=None):
        self._rev += 1
        try:
            self._step(beacon_s, beacon_t)
        finally:
            self._rev += 1

    def _step(self, beacon_s=None, beacon_t=None):
        dt = self.DT
        if beacon_s is not None:                 # 실제 로봇 위치 (상위 제어기가 움직임)
            u_new = self.scn.dir * beacon_s
            self.beacon_vu = (u_new - self.beacon_u) / dt
            self.beacon_speed = max(0.0, -self.beacon_vu)                  # 차량 쪽으로 다가오는 속도
            self.beacon_u = u_new
            self.beacon_t = beacon_t
        self.t += dt
        self._pose_cache = None
        # 생성
        for (side, ln), ts in self.next_spawn.items():
            if self.t >= ts:
                u0, _ = self._u_range(side)
                same = self.active & (self.side == side) & (np.abs(self.lane - ln) < 0.6)
                gap_ok = not same.any() or (self.u[same] - self.len[same] / 2).min() - u0 > 25.0
                if gap_ok:
                    v = None
                    if same.any():
                        j = np.nonzero(same)[0][np.argmin(self.u[same])]
                        v = min(self.v0.max(), self.v[j] + 2.0)
                    self._activate(side, ln, u0, v)
                    self.next_spawn[(side, ln)] = self.t + self._exp(side)
                else:
                    self.next_spawn[(side, ln)] = self.t + 0.5
        obs = self.obstacles()
        idx = np.nonzero(self.active & ~self.crashed)[0]
        acc = np.zeros(self.N)
        p = self.IDM
        # 선행 차량 (벡터화): 점유 차로(차로변경 중이면 출발·목표 둘 다)가 겹치는 바로 앞 차량
        inlc = self.lc < 1
        oa = np.where(inlc, np.round(self.lane_from), np.round(self.lane)).astype(int)
        ob = np.where(inlc, self.lane_to, np.round(self.lane)).astype(int)
        ovl = ((oa[:, None] == oa[None, :]) | (oa[:, None] == ob[None, :]) |
               (ob[:, None] == oa[None, :]) | (ob[:, None] == ob[None, :]))
        du = self.u[None, :] - self.u[:, None]
        cand = (ovl & (self.side[:, None] == self.side[None, :]) & self.active[None, :] & ~self.crashed[None, :]
                & (du > 0))
        G = np.where(cand, du - self.len[None, :] / 2 - self.len[:, None] / 2, np.inf)
        lead = np.argmin(G, axis=1)
        lead_gap = G[np.arange(self.N), lead]
        for i in idx:
            side = int(self.side[i])
            front = self.u[i] + self.len[i] / 2
            gap, dv = lead_gap[i], 0.0
            if np.isfinite(gap):
                dv = self.v[i] - self.v[lead[i]]
            self.scc_target[i] = int(lead[i]) if (self.scc[i] and gap < self.tp.scc_range) else -1
            # SCC: 비콘이 같은 차로 앞, 레이더 거리 안 → 움직이고 있으면 추종 대상으로 잡음(이후 유지)
            if self.scc[i] and side == self.scn.side:
                bt = self._lane_t(side, self.lane_to[i])
                in_lane = abs(self.beacon_t - bt) < self.m.params.lane_width / 2
                bgap = self.beacon_u - BEACON_HALF_LEN - front
                if in_lane and 0 < bgap < self.tp.scc_range:
                    if not self.scc_lock[i] and self.beacon_vu > self.tp.scc_moving_min:
                        self.scc_lock[i] = True
                        self.counts['scc_lock'] += 1
                        self.events.append((self.t, 'scc_lock', int(i), f'SCC가 비콘을 앞차로 인식 ({bgap:.0f} m)'))
                    if self.scc_lock[i] and bgap < gap:
                        gap, dv = bgap, self.v[i] - self.beacon_vu
                        self.scc_target[i] = -2
                else:
                    self.scc_lock[i] = False
            # 비콘 인지: 인지 거리(환경·경광등 배율, 가시거리 상한) 안 + 시야가 가리지 않아야 함
            if side == self.scn.side and not self.noticed[i] and self.beh[i] != B_CRASH:
                d_eff = min(self.notice_d[i] * self.notice_scale, self.visibility)
                dist_b = self.beacon_u - front
                if 0 < dist_b < d_eff and self._visible(i):
                    self.noticed[i] = True
                    self.notice_at[int(i)] = float(dist_b)
                    self.counts['noticed'] += 1
                    if self.beh[i] == B_BRAKE:
                        self.v0[i] = min(self.v0[i], self.slow_v[i])
                        self.counts['brake'] += 1
                        self.events.append((self.t, 'brake', int(i), f'{self.slow_v[i] * 3.6:.0f} km/h로 감속'))
            # 장애물 (무반응 차량은 무시)
            blocked_gap = np.inf
            if self.beh[i] != B_CRASH:
                # 차로변경 중에는 현재 횡위치~목표 차로 사이 전체를 점유한 것으로 본다
                t_cur, t_tgt = self._lane_t(side, self.lane[i]), self._lane_t(side, self.lane_to[i])
                t_lo, t_hi = min(t_cur, t_tgt), max(t_cur, t_tgt)
                for (os_, ou, ohl, ot, ohw, kind) in obs:
                    if os_ != side or ou + ohl <= front - self.len[i]:
                        continue
                    if not self.noticed[i] and kind != 'vehicle' and ou - front > 30.0:
                        continue
                    margin = ohw + self.wid[i] / 2 + 0.1
                    if t_lo - margin < ot < t_hi + margin:
                        g = ou - ohl - front
                        ov = self.beacon_vu if kind == 'beacon' else 0.0               # 장애물 속도(흐름 방향)
                        if abs(ot - t_tgt) < margin and g < blocked_gap:
                            blocked_gap = g
                        if g < gap:
                            gap, dv = g, self.v[i] - ov
            # 차로 변경 요청
            if self.lc[i] >= 1 and np.isfinite(blocked_gap) and self.noticed[i]:
                want = self.beh[i] == B_LC or (self.beh[i] == B_BRAKE and blocked_gap < 60.0)
                if want:
                    self._try_lane_change(i, obs)
            # IDM
            v, v0 = self.v[i], self.v0[i]
            a_free = p['a'] * (1 - (v / max(v0, 0.1)) ** 4)
            if np.isfinite(gap):
                Tg = self.scc_T[i] if self.scc[i] else p['T']
                s_star = p['s0'] + max(0.0, v * Tg + v * dv / (2 * np.sqrt(p['a'] * p['b'])))
                a = a_free - p['a'] * (s_star / max(gap, 0.1)) ** 2
            else:
                a = a_free
            # 감속 한계: 운전자가 반응 중이면 사람 급제동(8), SCC만 작동 중이면 SCC 최대 감속도
            driver = self.noticed[i] and self.beh[i] != B_CRASH
            cap = p['b_max'] if (driver or not self.scc[i]) else self.scc_b[i]
            acc[i] = max(a, -cap)
        # 적분
        self.v[idx] = np.maximum(0.0, self.v[idx] + acc[idx] * dt)
        self.u[idx] += self.v[idx] * dt
        prev_lane = self.lane.copy()
        lc = idx[self.lc[idx] < 1]
        self.lc[lc] = np.minimum(1.0, self.lc[lc] + dt / self.LC_TIME)
        k = self.lc[lc]
        sm = k * k * (3 - 2 * k)
        self.lane[lc] = self.lane_from[lc] + (self.lane_to[lc] - self.lane_from[lc]) * sm
        self.lat_vel[:] = (self.lane - prev_lane) * self.m.params.lane_width / dt
        # 추돌 판정 (무반응 차량 포함, 실제 횡위치 기준)
        for i in idx:
            side = int(self.side[i])
            front, rear = self.u[i] + self.len[i] / 2, self.u[i] - self.len[i] / 2
            ti = self._lane_t(side, self.lane[i])
            for (os_, ou, ohl, ot, ohw, kind) in obs:
                if os_ != side or (kind == 'vehicle' and abs(ou - self.u[i]) < 1e-6):
                    continue
                if front >= ou - ohl and rear <= ou + ohl and abs(ti - ot) < ohw + self.wid[i] / 2:
                    self.crashed[i] = True
                    what = {'beacon': '비콘', 'accident': '사고 차량'}.get(kind, '정지 차량')
                    extra = f', 비콘 {self.beacon_vu:+.1f} m/s(+: 사고 쪽 후진)' if kind == 'beacon' else ''
                    self.events.append((self.t, 'crash', int(i),
                                        f'{what} 추돌 (차량 {self.v[i] * 3.6:.0f} km/h{extra}, {BEHAVIORS[self.beh[i]]})'))
                    self.counts['crash'] += 1
                    if kind == 'beacon':
                        self.counts['beacon_hit'] += 1
                        self.beacon_hits.append((self.t, int(i), float(self.v[i])))
                    self.v[i] = 0.0
                    break
        # 소멸
        for side in (-1, 1):
            _, u1 = self._u_range(side)
            gone = self.active & (self.side == side) & (self.u - self.len / 2 > u1 + 5) & ~self.crashed
            self.active[gone] = False
        # 사라진 차량을 추종 대상으로 들고 있던 SCC 해제
        tg = self.scc_target
        stale = (tg >= 0) & ~self.active[np.clip(tg, 0, self.N - 1)]
        tg[stale] = -1
        if len(self.events) > 500:
            self.events = self.events[-500:]

    def _visible(self, i):
        """운전자 눈(앞범퍼 앞 0.3 m, 높이 1.2 m)에서 비콘이 보이는지 (가림: 차량·지형·터널 벽)."""
        if self.los_fn is None:
            return True
        if self._pose_cache is None:
            self._pose_cache = self.poses_at()[0]
        T = self._pose_cache[i]
        eye = T[:3, 3] + T[:3, 0] * (self.len[i] / 2 + 0.3) + T[:3, 2] * 1.2
        bs = self.scn.dir * self.beacon_u
        tgt = self.m.point(bs, self.beacon_t, 0.6)
        return bool(self.los_fn(eye[None, :], tgt)[0])

    def _block_dist(self, i, side, c, obs):
        """차로 c에서 차량 i 앞 400m 안의 첫 장애물까지 거리 (없으면 inf)."""
        ct = self._lane_t(side, c)
        front = self.u[i] + self.len[i] / 2
        d = np.inf
        for (os_, ou, ohl, ot, ohw, kind) in obs:
            if os_ == side and ou + ohl > self.u[i] - self.len[i] and ou - front < 400 \
                    and abs(ot - ct) < ohw + self.wid[i] / 2 + 0.1:
                d = min(d, ou - ohl - front)
        return d

    def _try_lane_change(self, i, obs):
        """막힌 차로에서 벗어나기. 옆 차로가 비었거나, 막혀 있어도 장애물이 더 멀고
        그 너머에 빈 차로가 있으면(열린 차로 방향으로 한 칸씩) 옮긴다."""
        side = int(self.side[i])
        cur = int(self.lane_to[i])
        dist = [self._block_dist(i, side, c, obs) for c in range(self.lanes)]
        free = [c for c in range(self.lanes) if np.isinf(dist[c])]
        if not free:
            return False
        target = min(free, key=lambda c: (abs(c - cur), self.rng.random()))     # 가장 가까운 빈 차로
        if target == cur:
            return False
        c = cur + (1 if target > cur else -1)
        # 거쳐 가는 차로: 장애물이 더 멀거나, 다음 차로변경을 마칠 여유(경유 거리)가 있으면 허용
        room = max(15.0, 0.6 * self.v[i] * self.LC_TIME + 8.0)
        if not (np.isinf(dist[c]) or dist[c] > dist[cur] + 3.0 or dist[c] > room):
            return False
        # 간격 수용 (범퍼 간 거리): 앞 ≥ 2 + 0.5·v_i, 뒤 ≥ 2 + 0.9·v_뒤차·max(0, v_뒤차-v_i)/v_뒤차 성분
        same = self.active & (self.side == side) & (np.abs(self.lane - c) < 0.75)
        same[i] = False
        ok = True
        if same.any():
            j_ = np.nonzero(same)[0]
            front_i, rear_i = self.u[i] + self.len[i] / 2, self.u[i] - self.len[i] / 2
            gap_ahead = (self.u[j_] - self.len[j_] / 2) - front_i
            gap_behind = rear_i - (self.u[j_] + self.len[j_] / 2)
            ah, bh = gap_ahead > -self.len[i] - self.len[j_], gap_behind > -self.len[i] - self.len[j_]
            amask = self.u[j_] >= self.u[i]
            if (amask & ah).any() and gap_ahead[amask].min() < 2.0 + 0.5 * self.v[i]:
                ok = False
            bmask = ~amask
            if bmask.any():
                k = np.argmax(np.where(bmask, self.u[j_], -np.inf))      # 바로 뒤차
                vf = self.v[j_[k]]
                need = 2.0 + 0.6 * vf + 0.8 * max(0.0, vf - self.v[i])  # 뒤차가 빠를수록 더 필요
                if gap_behind[k] < need:
                    ok = False
        for c in [c]:
            if ok:
                self.lane_from[i], self.lane_to[i], self.lc[i] = cur, c, 0.0
                self.counts['lane_change'] += 1
                self.events.append((self.t, 'lane_change', int(i), f'{cur + 1}→{c + 1}차로'))
                return True
        return False

    # ---- 출력
    def poses_at(self, t_query=None):
        """(N,4,4) 포즈 (비활성은 지하로 숨김), 속도 벡터 (N,3). t_query로 등속 외삽."""
        dtq = 0.0 if t_query is None else float(t_query - self.t)
        key = (self._rev, self.t, dtq)
        if self._pa_cache[0] == key:
            return self._pa_cache[1]
        out = self._poses_at(dtq)
        for a in out:
            a.flags.writeable = False             # 캐시 공유본: 호출자가 고치면 바로 오류가 나게
        self._pa_cache = (key, out)
        return out

    def _poses_at(self, dtq):
        N = self.N
        T = np.tile(np.eye(4), (N, 1, 1))
        T[:, 2, 3] = HIDDEN_Z
        vel = np.zeros((N, 3))
        idx = np.nonzero(self.active)[0]
        if len(idx) == 0:
            return T, vel
        u = self.u[idx] + np.where(self.crashed[idx], 0.0, self.v[idx] * dtq)
        side = self.side[idx]
        d = np.where(side < 0, 1.0, -1.0)
        s = np.clip(d * u, 0.0, self.L)
        t = side * (self.m.params.barrier_half + self.m.params.inner_shoulder + (self.lane[idx] + 0.5) * self.m.params.lane_width)
        P = self.m.poses(s, t, side > 0)
        # 차로 번호 증가 = 바깥쪽 = 두 방향 모두 차량 기준 오른쪽 → 좌측 속도 = -lat_vel
        yaw = np.arctan2(-self.lat_vel[idx], np.maximum(self.v[idx], 0.5))
        c, sn = np.cos(yaw), np.sin(yaw)
        R = np.tile(np.eye(4), (len(idx), 1, 1))
        R[:, 0, 0], R[:, 0, 1], R[:, 1, 0], R[:, 1, 1] = c, -sn, sn, c
        T[idx] = P @ R
        vel[idx] = T[idx, :3, 0] * self.v[idx, None]
        return T, vel

    def active_mask(self):
        return self.active.copy()
