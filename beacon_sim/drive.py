"""비콘 주행 하드웨어 모델: 좌우 모터(차동·스키드 스티어) + 엔코더 → 휠 오도메트리, IMU.

구동 (실기 제어기 동작 기준)
  상위 명령 (v, w) → 좌우 바퀴 목표 속도 (v ∓ w·B/2, B = 윤거) → 모터 최대 회전수 제한
  → 바퀴 속도는 1차 지연 + 가속 한계로 따라감
  → 실제 차체 운동: 스키드 스티어라 회전이 바퀴 속도차보다 덜 됨 (유효 윤거 = 윤거 × track_eff_factor) + 미끄럼 잡음
  → 엔코더: 바퀴 회전을 그대로 셈 (차체가 막혀도 바퀴가 헛돌면 카운트는 올라감), int32 로 넘어감
휠 오도메트리: 엔코더 증분 → (윤거 × odom_track_factor) 로 적분. 실제와의 차이 = 미끄럼·스키드 오차
IMU: 실제 자세 변화(각속도)·가속도 + 중력(경사·편경사) + 잡음 + 바이어스 랜덤워크

값이 (임시) 로 표시된 파라미터는 실기 사양으로 고칠 것 (config/sim.yaml drive:, imu:)
"""
from dataclasses import dataclass, field, fields

import numpy as np

G = 9.80665


def _pick(cls, d):
    names = {f.name for f in fields(cls)}
    return cls(**{k: v for k, v in (d or {}).items() if k in names})


@dataclass
class DriveParams:
    gear_ratio: float = 10.0          # 모터 → 바퀴 감속비 (임시: 5GB 감속기 + 풀리)
    counts_per_motor_rev: float = 4096.0   # 엔코더 카운트 / 모터 1회전 (임시)
    motor_max_rpm: float = 3000.0     # (임시)
    tau: float = 0.12                 # 바퀴 속도 응답 시정수 [s]
    accel_max: float = 1.5            # 바퀴 선가속 한계 [m/s²]
    track_eff_factor: float = 1.35    # 스키드 스티어 유효 윤거 배율 (회전이 덜 됨)
    odom_track_factor: float = 1.35   # 오도메트리 계산에 쓰는 윤거 배율 (보정됨 = track_eff_factor 와 같음)
    slip_std: float = 0.01            # 바퀴 접지 속도 잡음 (비율)
    wait_tower_up: bool = True        # 자율(ROS) 주행 명령은 타워가 다 펴진 뒤에만 (실기 AUTO 동작)
    odom_rate_hz: float = 50.0        # 엔코더 피드백 주기


@dataclass
class ImuParams:
    enabled: bool = True
    rate_hz: float = 100.0
    frame_id: str = 'imu_link'
    xyz: list = field(default_factory=lambda: [0.0, 0.0, 0.25])   # base_link 기준 장착 위치 (임시)
    gyro_noise: float = 0.002         # rad/s (백색)
    gyro_bias_walk: float = 2e-5      # rad/s/√s
    accel_noise: float = 0.05         # m/s²
    accel_bias_walk: float = 5e-4     # m/s²/√s


class DriveModel:
    def __init__(self, cfg, wheel_radius, track, seed=0):
        self.p = _pick(DriveParams, cfg)
        self.r, self.B = float(wheel_radius), float(track)
        self.rng = np.random.default_rng(seed + 101)
        self.wheel_v = np.zeros(2)        # 좌, 우 바퀴 접지 속도 [m/s]
        self.wheel_pos = np.zeros(2)      # 바퀴 회전각 누적 [rad]
        self.counts = np.zeros(2)         # 엔코더 누적 (실수, 보고할 때 int32 로)

    @property
    def max_wheel_v(self):
        return self.p.motor_max_rpm / self.p.gear_ratio * 2 * np.pi / 60.0 * self.r

    def step(self, dt, v_cmd, w_cmd):
        """→ 실제 차체 (v, w)"""
        p = self.p
        tgt = np.array([v_cmd - w_cmd * self.B / 2, v_cmd + w_cmd * self.B / 2])
        m = self.max_wheel_v
        if np.abs(tgt).max() > m:          # 회전수 한계: 비율 유지하며 줄임
            tgt *= m / np.abs(tgt).max()
        dv = (tgt - self.wheel_v) * (1 - np.exp(-dt / max(p.tau, 1e-3)))
        dv = np.clip(dv, -p.accel_max * dt, p.accel_max * dt)
        self.wheel_v += dv
        om = self.wheel_v / self.r                       # 바퀴 각속도
        self.wheel_pos += om * dt
        self.counts += om / (2 * np.pi) * p.gear_ratio * p.counts_per_motor_rev * dt
        ground = self.wheel_v * (1 + p.slip_std * self.rng.standard_normal(2))
        v = float(ground.mean())
        w = float((ground[1] - ground[0]) / (self.B * p.track_eff_factor))
        return v, w

    def encoder_int32(self):
        c = np.floor(self.counts).astype(np.int64)
        return ((c + 2 ** 31) % 2 ** 32 - 2 ** 31).astype(np.int64)

    def motor_rpm(self):
        return self.wheel_v / self.r * 60 / (2 * np.pi) * self.p.gear_ratio


class WheelOdom:
    """엔코더 증분 적분 (실기 PC 쪽 드라이버가 하는 계산). int32 넘어감 안전."""

    def __init__(self, drive: DriveModel, x0=0.0, y0=0.0, yaw0=0.0):
        self.d = drive
        self.x, self.y, self.yaw = x0, y0, yaw0
        self.v = self.w = 0.0
        self.prev = drive.encoder_int32()

    def update(self, dt):
        d = self.d
        c = d.encoder_int32()
        dc = c - self.prev
        dc = (dc + 2 ** 31) % 2 ** 32 - 2 ** 31                    # 넘어감 처리
        self.prev = c
        dist = dc / (d.p.gear_ratio * d.p.counts_per_motor_rev) * 2 * np.pi * d.r
        ds = float(dist.mean())
        dth = float((dist[1] - dist[0]) / (d.B * d.p.odom_track_factor))
        self.x += ds * np.cos(self.yaw + dth / 2)
        self.y += ds * np.sin(self.yaw + dth / 2)
        self.yaw += dth
        self.v, self.w = ds / dt, dth / dt
        return self


class ImuModel:
    def __init__(self, cfg, seed=0):
        self.p = _pick(ImuParams, cfg)
        self.rng = np.random.default_rng(seed + 202)
        self.bg = np.zeros(3)
        self.ba = np.zeros(3)
        self.prev = None                 # (R, p, v)

    def step(self, dt, T_wb):
        """정답 자세 T_wb → (각속도 body, 비력 body) + 잡음. 처음 호출은 None."""
        p = self.p
        R = T_wb[:3, :3]
        lever = np.asarray(p.xyz, float)
        pos = T_wb[:3, 3] + R @ lever
        if self.prev is None:
            self.prev = (R, pos, np.zeros(3))
            return None
        R0, p0, v0 = self.prev
        dR = R0.T @ R
        ang = np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1))
        if ang < 1e-9:
            om = np.zeros(3)
        else:
            om = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]]) * ang / (2 * np.sin(ang)) / dt
        v = (pos - p0) / dt
        a_w = (v - v0) / dt
        f = R.T @ (a_w + np.array([0.0, 0.0, G]))      # 가속도계 = 비력 (정지 시 +g 위로)
        self.prev = (R, pos, v)
        s = np.sqrt(dt)
        self.bg += p.gyro_bias_walk * s * self.rng.standard_normal(3)
        self.ba += p.accel_bias_walk * s * self.rng.standard_normal(3)
        gyro = om + self.bg + p.gyro_noise * self.rng.standard_normal(3)
        acc = f + self.ba + p.accel_noise * self.rng.standard_normal(3)
        return gyro, acc
