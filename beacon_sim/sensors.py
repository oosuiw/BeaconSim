"""RGL 기반 센서 모델: Hesai Pandar40P, Livox Tele-15, Continental ARS408.

모든 센서는 run(T_world_sensor, t_ns, ...) 으로 한 프레임을 만들고,
센서 좌표계 기준 numpy 구조체 배열을 돌려준다. ROS 메시지 변환은 sim_core가 한다.
"""
import os

import numpy as np

from . import rgl
from .common import ROOT
from .scene import CLASS, RCS, RETRO

# entity id = 클래스*10000 + 인스턴스 → 점별 시맨틱/인스턴스 정답
ID_SCALE = 10000
ID_TO_CLASS = {v: k for k, v in CLASS.items()}


def entity_id(cls, instance=0):
    return CLASS[cls] * ID_SCALE + instance


_WET_FACTOR = {CLASS['road']: 0.5, CLASS['lane_white']: 0.35, CLASS['lane_yellow']: 0.35}


# 실기 Hesai Pandar40P 밝기 모델 (2026-11-28, 실기 bag 4개 측정에 맞춤: 장착 높이 약 0.77 m)
#   클래스 → (A, k, σ): 밝기 = A · cos(입사각)^k · exp(N(0, σ))
#   노면: 멀어질수록(비스듬할수록) 급격히 어두워짐 (실측 중앙값 2~10 m 7 → 10~20 m 5 → 20~30 m 2)
#   도색: 유리알 재귀반사라 각도에 둔감, 점마다 편차 큼 (실측 상위 10% 28~42 가 먼 곳까지 유지)
#   가드레일·배리어(수직면): 거리와 무관하게 약 18
HESAI_V1 = {'road': (22.0, 0.6, 0.15), 'lane_white': (30.0, 0.15, 0.45), 'lane_yellow': (26.0, 0.15, 0.45),
            'guardrail': (19.0, 0.15, 0.2), 'barrier': (18.5, 0.15, 0.2), 'terrain': (14.0, 0.6, 0.3)}


def _hesai_intensity(retro, cos_i, eid, rng, old):
    cls = eid // ID_SCALE
    out = old.copy()
    n = len(cos_i)
    road_a, road_k, road_s = HESAI_V1['road']
    road_i = road_a * cos_i ** road_k * np.exp(rng.normal(0, road_s, n))
    for name, (a, k, s) in HESAI_V1.items():
        m = cls == CLASS[name]
        if not m.any():
            continue
        v = a * cos_i[m] ** k * np.exp(rng.normal(0, s, m.sum()))
        if name.startswith('lane'):            # 도색 밝기 배율 w (메시 반사율): < 1 노면 밝기 쪽으로 섞음, > 1 그만큼 밝게
            w = np.clip((retro[m] - RETRO['road']) / max(RETRO[name] - RETRO['road'], 1e-6), 0.0, 3.5)
            v = np.where(w <= 1, w * v + (1 - w) * road_i[m], w * v)
        out[m] = v + rng.normal(0, 1.0, m.sum())
    return out


def lidar_intensity_and_drop(retro, inc, dist, range_at_10pct, rng, env=None, eid=None, model=None):
    """간이 LiDAR 방정식: 수신 세기 ∝ 반사율·cos(입사각)·exp(-2αd)/R².
    유효 최대거리 = R10 * sqrt(반사율·cos·감쇠 / 0.1). 이를 넘으면 raydrop.
    env: 대기 감쇠 α, 노면 젖음(정반사로 되돌아오는 빛 감소 → 노면/차선 반사율 저하)."""
    retro = np.asarray(retro, np.float64)
    if env is not None and env.wet > 0 and eid is not None:
        cls = eid // ID_SCALE
        for c, f in _WET_FACTOR.items():
            retro = np.where(cls == c, retro * (1 - f * env.wet), retro)
    cos_i = np.clip(np.cos(inc), 0.0, 1.0)
    att = np.exp(-2.0 * env.alpha * dist) if env is not None and env.alpha > 0 else 1.0
    hesai = model == 'hesai_v1' and eid is not None
    power = retro * (np.sqrt(cos_i) if hesai else cos_i) * att      # 실기: 비스듬한 노면도 40 m 넘게 낮은 밝기로 돌아옴
    eff = range_at_10pct * np.sqrt(np.maximum(power, 1e-6) / 0.1)
    keep = dist < eff * np.exp(0.08 * rng.standard_normal(len(dist)))
    inten = 100.0 * retro * (0.35 + 0.65 * cos_i) * np.sqrt(att) + rng.normal(0, 1.5, len(dist))
    if hesai:
        inten = _hesai_intensity(retro, cos_i, eid, rng, inten) * np.sqrt(att)
    return np.clip(inten, 0, 255), keep


def false_returns(env, n_rays, rng, max_d=15.0):
    """비/눈/안개 입자에 의한 근거리 허위점: (선택된 레이 인덱스, 거리, 세기)."""
    if env is None or env.false_rate <= 0:
        return np.zeros(0, int), np.zeros(0), np.zeros(0)
    k = rng.poisson(env.false_rate * n_rays)
    idx = rng.integers(0, n_rays, k)
    far = 25.0 if env.weather == 'snow' else max_d
    d = 0.5 + rng.exponential(far / 3, k).clip(0, far)
    inten = rng.uniform(1, 12 if env.weather == 'snow' else 6, k)
    return idx, d, inten


# ---------------------------------------------------------------- Pandar40P

# AWSIM LaserArrayLibrary.HesaiPandar40P (Unity 좌표: 수직 오프셋 +가 아래, 수평 오프셋 +가 시계방향)
# (horizontalAngularOffsetDeg, verticalAngularOffsetDeg, ringId, timeOffset[ms])
_PANDAR40P_LASERS = [
    (-1.042, -15.0, 40, 0.01334), (-1.042, -11.0, 39, 0.02709), (-1.042, -8.0, 38, 0.03952),
    (-1.042, -5.0, 37, 0.05194), (-1.042, -3.0, 36, 0.01007), (-1.042, -2.0, 35, 0.02382),
    (3.125, -1.67, 34, 0.0081), (-5.208, -1.33, 33, 0.00089), (-1.042, -1.0, 32, 0.03494),
    (3.125, -0.67, 31, 0.02185), (-5.208, -0.33, 30, 0.01465), (-1.042, -0.0, 29, 0.04737),
    (3.125, 0.33, 28, 0.03494), (-5.208, 0.67, 27, 0.0284), (-1.042, 1.0, 26, 0.00483),
    (3.125, 1.33, 25, 0.04737), (-5.208, 1.67, 24, 0.04082), (-1.042, 2.0, 23, 0.01858),
    (3.125, 2.33, 22, 0.01007), (-5.208, 2.67, 21, 0.00286), (-1.042, 3.0, 20, 0.03167),
    (3.125, 3.33, 19, 0.02382), (-5.208, 3.67, 18, 0.01661), (-1.042, 4.0, 17, 0.04409),
    (3.125, 4.33, 16, 0.03691), (-5.208, 4.67, 15, 0.03037), (-1.042, 5.0, 14, 0.0068),
    (3.125, 5.33, 13, 0.0493), (-5.208, 5.67, 12, 0.04279), (-1.042, 6.0, 11, 0.02055),
    (-1.042, 7.0, 10, 0.03364), (-1.042, 8.0, 9, 0.04606), (-1.042, 9.0, 8, 0.01204),
    (-1.042, 10.0, 7, 0.02579), (-1.042, 11.0, 6, 0.03821), (-1.042, 12.0, 5, 0.05064),
    (-1.042, 13.0, 4, 0.01334), (-1.042, 14.0, 3, 0.02709), (-1.042, 19.0, 2, 0.03952),
    (-1.042, 25.0, 1, 0.05194),
]

# nebula PointXYZIRCAEDT (autoware_point_types) 와 동일한 바이트 배치, point_step 32
NEBULA_DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', 'u1'), ('return_type', 'u1'),
                         ('channel', '<u2'), ('azimuth', '<f4'), ('elevation', '<f4'), ('distance', '<f4'),
                         ('time_stamp', '<u4')])
GT_DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'), ('label', '<u4'),
                     ('instance', '<u4')])
NEBULA_RETURN_STRONGEST = 3  # nebula_common ReturnType::STRONGEST (실차 드라이버 출력과 대조 필요)


class Pandar40P:
    def __init__(self, cfg, seed=1):
        self.cfg = cfg
        self.rng = np.random.default_rng(seed)
        L = np.array(_PANDAR40P_LASERS)
        hoff, voff, ring, toff_ms = L[:, 0], L[:, 1], L[:, 2].astype(int), L[:, 3]
        elev = np.radians(-voff)                     # Unity(+아래) → ROS(+위)
        self.channel_of_laser = 40 - ring            # channel 0 = +15° (nebula/Hesai laser 1)
        res = cfg.get('azimuth_resolution_deg', 0.2)
        self.n_blocks = int(round(360.0 / res))
        self.n_lasers = len(L)
        block_cw = np.arange(self.n_blocks) * res    # 시계방향 회전 각
        az_cw = block_cw[:, None] + hoff[None, :]
        az = np.radians(-az_cw).ravel()              # ROS 반시계
        el = np.tile(elev, self.n_blocks)
        self.block_period_ns = 1e9 / cfg['rate_hz'] / self.n_blocks
        self.ray_time_ns = (np.arange(self.n_blocks)[:, None] * self.block_period_ns
                            + toff_ms[None, :] * 1e6).ravel()
        self.ray_channel = np.tile(self.channel_of_laser, self.n_blocks).astype(np.uint16)
        self.ray_dir = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], -1)
        self.env = None
        rays = rgl.ray_poses_from_angles(az, el)
        self.graph = rgl.RaycastGraph(
            rays, [cfg['min_range'], cfg['max_range']],
            [rgl.XYZ_VEC3_F32, rgl.DISTANCE_F32, rgl.RAY_IDX_U32, rgl.LASER_RETRO_F32,
             rgl.INCIDENT_ANGLE_F32, rgl.ENTITY_ID_I32],
            cfg['sensor_id'])

    def run(self, T_world_sensor, lin_vel=(0, 0, 0), ang_vel=(0, 0, 0)):
        o = self.graph.run(T_world_sensor, lin_vel, ang_vel)
        xyz, d, idx = o[rgl.XYZ_VEC3_F32].astype(np.float64), o[rgl.DISTANCE_F32].astype(np.float64), o[rgl.RAY_IDX_U32]
        eid = o[rgl.ENTITY_ID_I32]
        inten, keep = lidar_intensity_and_drop(o[rgl.LASER_RETRO_F32], o[rgl.INCIDENT_ANGLE_F32], d,
                                               self.cfg['range_at_10pct'], self.rng, self.env, eid,
                                               self.cfg.get('intensity_model'))
        if self.cfg.get('intensity_model') == 'hesai_v1':          # 실기처럼 채널(레이저)마다 밝기 이득이 조금씩 다름
            if not hasattr(self, 'ring_gain'):
                self.ring_gain = np.exp(self.rng.normal(0, 0.08, self.n_lasers))
            inten = inten * self.ring_gain[idx % self.n_lasers]
            if not hasattr(self, 'paint_gain'):                        # 도로마다 도색 상태가 다름 (오래된 ~ 새 도색)
                self.paint_gain = float(np.exp(self.rng.normal(0.15, 0.3)))
            lane = np.isin(eid // ID_SCALE, [CLASS['lane_white'], CLASS['lane_yellow']])
            inten = np.clip(np.where(lane, inten * self.paint_gain, inten), 0, 255)
        dn = d + self.rng.normal(0, self.cfg['range_noise_std'], len(d))
        xyz = xyz * (dn / np.maximum(d, 1e-6))[:, None]
        keep &= dn > self.cfg['min_range']
        xyz, dn, idx, inten, eid = xyz[keep], dn[keep], idx[keep], inten[keep], eid[keep]
        fi, fd, fin = false_returns(self.env, len(self.ray_dir), self.rng)
        if len(fi):                                   # 입자 허위점 (실제 히트보다 가까울 때만 의미 있지만 단순 추가)
            xyz = np.r_[xyz, self.ray_dir[fi] * fd[:, None]]
            dn, idx, inten = np.r_[dn, fd], np.r_[idx, fi].astype(idx.dtype), np.r_[inten, fin]
            eid = np.r_[eid, np.zeros(len(fi), eid.dtype)]
        pts = np.zeros(len(xyz), NEBULA_DTYPE)
        pts['x'], pts['y'], pts['z'] = xyz.T
        pts['intensity'] = np.round(inten).astype(np.uint8)
        pts['return_type'] = NEBULA_RETURN_STRONGEST
        pts['channel'] = self.ray_channel[idx]
        pts['azimuth'] = np.mod(np.arctan2(xyz[:, 1], xyz[:, 0]), 2 * np.pi)
        pts['elevation'] = np.arcsin(np.clip(xyz[:, 2] / np.maximum(dn, 1e-6), -1, 1))
        pts['distance'] = dn
        pts['time_stamp'] = self.ray_time_ns[idx].astype(np.uint32)
        gt = np.zeros(len(xyz), GT_DTYPE)
        gt['x'], gt['y'], gt['z'] = xyz.T
        gt['intensity'] = inten
        gt['label'] = eid // ID_SCALE
        gt['instance'] = eid % ID_SCALE
        return pts, gt


# ---------------------------------------------------------------- Tele-15

# livox_ros_driver2 PointCloud2 (xfer_format=0) 배치: x y z intensity(f32) tag(u8) line(u8) timestamp(f64), 26바이트
LIVOX_DTYPE = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'tag', 'line', 'timestamp'],
                        'formats': ['<f4', '<f4', '<f4', '<f4', 'u1', 'u1', '<f8'],
                        'offsets': [0, 4, 8, 12, 16, 17, 18], 'itemsize': 26})


def load_livox_pattern(csv_path):
    """livox_laser_simulation scan_mode CSV (Time/s, Azimuth/deg, Zenith/deg) → (t, az, el) rad. npy 캐시(사용자 캐시 폴더)."""
    import hashlib
    from .common import CACHE_DIR
    os.makedirs(CACHE_DIR, exist_ok=True)
    st = os.stat(csv_path)
    tag = hashlib.md5(f'{st.st_size}:{int(st.st_mtime)}'.encode()).hexdigest()[:8]
    cache = os.path.join(CACHE_DIR, f'{os.path.basename(csv_path)}.{tag}.npy')
    import glob
    for old in glob.glob(os.path.join(CACHE_DIR, f'{os.path.basename(csv_path)}.*.npy')):
        if old != cache:                                  # 예전 방식(경로 기준) 캐시 사본 정리
            try:
                os.remove(old)
            except OSError:
                pass
    if os.path.exists(cache):
        a = np.load(cache)
    else:
        a = np.loadtxt(csv_path, delimiter=',', skiprows=1, encoding='utf-8-sig')
        np.save(cache, a)
    t = a[:, 0]
    az = np.radians(a[:, 1])                  # gazebo yaw: +가 좌측(반시계)
    el = -np.radians(a[:, 2] - 90.0)          # zenith 90°가 수평, gazebo pitch +는 아래
    return t, az, el


def rosette_pattern(hfov_deg=14.5, vfov_deg=16.2, rate=240000.0, duration=4.0, f1=1103.9, f2=637.3):
    """Livox식 로제트(무아레) 비반복 스캔 근사: 두 회전 프리즘의 합성 방향.
    (x, y) = ½(cos 2πf1 t + cos 2πf2 t, sin 2πf1 t − sin 2πf2 t) → 단위 원판 → FOV로 스케일.
    실제 Tele-15 패턴과는 다르다 (tele.csv가 있으면 그쪽을 쓴다)."""
    t = np.arange(int(rate * duration)) / rate
    a1, a2 = 2 * np.pi * f1 * t, 2 * np.pi * f2 * t
    x = 0.5 * (np.cos(a1) + np.cos(a2))
    y = 0.5 * (np.sin(a1) - np.sin(a2))
    return t, np.radians(x * hfov_deg / 2), np.radians(y * vfov_deg / 2)


def find_tele_pattern(cfg_path):
    """tele.csv 위치: 설정 경로(사용자 폴더 우선) → ~/.config/beaconsim/tele.csv. 없으면 None."""
    from .common import USER_DIR, resource
    for p in (resource(cfg_path) if cfg_path else None, os.path.join(USER_DIR, 'tele.csv')):
        if p and os.path.exists(p):
            return p
    return None


class Tele15:
    def __init__(self, cfg, seed=2):
        self.cfg = cfg
        self.rng = np.random.default_rng(seed)
        path = find_tele_pattern(cfg.get('pattern_csv'))
        if path:
            t, self.az, self.el = load_livox_pattern(path)
            self.pattern = f'tele.csv ({path})'
        else:
            t, self.az, self.el = rosette_pattern()
            self.pattern = '자체 로제트 패턴 (tele.csv 없음)'
        self.dt = float(np.median(np.diff(t)))     # 4.1667us (240k pts/s)
        self.poses34 = np.empty((len(self.az), 3, 4), np.float32)          # 나눠서 계산 (임시 메모리 절약)
        for k in range(0, len(self.az), 100000):
            self.poses34[k:k + 100000] = rgl.ray_poses_from_angles(self.az[k:k + 100000], self.el[k:k + 100000])[:, :3, :4]
        self.dirs = self.poses34[:, :, 2].astype(np.float64)
        self.n = int(round(1.0 / cfg['rate_hz'] / self.dt))
        self.cursor = 0
        self.graph = rgl.RaycastGraph(
            self._rays(), [cfg['min_range'], cfg['max_range']],
            [rgl.XYZ_VEC3_F32, rgl.DISTANCE_F32, rgl.RAY_IDX_U32, rgl.LASER_RETRO_F32, rgl.INCIDENT_ANGLE_F32,
             rgl.ENTITY_ID_I32],
            cfg['sensor_id'])
        self.cursor_used = 0
        self.env = None

    def _rays(self):
        i = (self.cursor + np.arange(self.n)) % len(self.az)
        self.cursor_used = self.cursor
        self.cursor = (self.cursor + self.n) % len(self.az)
        self.ray_dir = self.dirs[i]
        return self.poses34[i]

    def run(self, T_world_sensor, frame_start_ns, lin_vel=(0, 0, 0), ang_vel=(0, 0, 0)):
        self.graph.set_rays(self._rays(), [self.cfg['min_range'], self.cfg['max_range']])
        o = self.graph.run(T_world_sensor, lin_vel, ang_vel)
        xyz, d, idx = o[rgl.XYZ_VEC3_F32].astype(np.float64), o[rgl.DISTANCE_F32].astype(np.float64), o[rgl.RAY_IDX_U32]
        inten, keep = lidar_intensity_and_drop(o[rgl.LASER_RETRO_F32], o[rgl.INCIDENT_ANGLE_F32], d,
                                               self.cfg['range_at_10pct'], self.rng, self.env, o[rgl.ENTITY_ID_I32])
        dn = d + self.rng.normal(0, self.cfg['range_noise_std'], len(d))
        xyz = xyz * (dn / np.maximum(d, 1e-6))[:, None]
        keep &= dn > self.cfg['min_range']
        xyz, inten, idx = xyz[keep], inten[keep], idx[keep]
        fi, fd, fin = false_returns(self.env, self.n, self.rng)
        if len(fi):
            xyz = np.r_[xyz, self.ray_dir[fi] * fd[:, None]]
            inten, idx = np.r_[inten, fin], np.r_[idx, fi].astype(idx.dtype)
        pts = np.zeros(len(xyz), LIVOX_DTYPE)
        pts['x'], pts['y'], pts['z'] = xyz.T
        pts['intensity'] = inten
        pts['timestamp'] = frame_start_ns + idx * self.dt * 1e9   # 점별 절대시각(ns), 모션 보정 검증용
        return pts


# ---------------------------------------------------------------- ARS408

CLUSTER_DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('distance', '<f4'), ('azimuth', '<f4'),
                          ('radial_speed', '<f4'), ('rcs', '<f4'), ('snr', '<f4'), ('far', 'u1'),
                          ('label', 'u1'), ('instance', '<u2')])


class ARS408:
    """레이 기반 클러스터 + 정답 기반 객체. 다중경로/고스트는 아직 없음(실로그 통계로 보강 예정)."""

    SNR_CONST = 96.0      # SNR[dB] = RCS - 40log10(R) + C, 승용차(10dBsm)가 250m에서 겨우 잡히도록

    def __init__(self, cfg, seed=3):
        self.cfg = cfg
        self.rng = np.random.default_rng(seed)
        nf, ff = cfg['near'], cfg['far']
        az = np.radians(np.arange(-nf['fov_deg'], nf['fov_deg'] + 1e-6, cfg['ray_az_step_deg']))
        half_el = cfg['elevation_fov_deg'] / 2
        el = np.radians(np.arange(-half_el, half_el + 1e-6, cfg['ray_el_step_deg']))
        A, E = np.meshgrid(az, el, indexing='ij')
        A, E = A.ravel(), E.ravel()
        absdeg = np.degrees(np.abs(A))
        rmax = np.where(absdeg <= ff['fov_deg'], ff['max_range'],
                        np.where(absdeg <= nf['wide_fov_deg'], nf['max_range'], nf['wide_max_range']))
        ranges = np.c_[np.full(len(A), 0.2), rmax]
        self.graph = rgl.RaycastGraph(
            rgl.ray_poses_from_angles(A, E), ranges,
            [rgl.XYZ_VEC3_F32, rgl.DISTANCE_F32, rgl.ENTITY_ID_I32, rgl.RADIAL_SPEED_F32, rgl.INCIDENT_ANGLE_F32],
            cfg['sensor_id'])
        self.env = None
        # 스캔 면 시각화용: 방위각별 최대 거리와 '처음 닿은 물체(노면 제외)' 거리
        self.scan_az = az
        self.scan_rmax = np.where(np.degrees(np.abs(az)) <= ff['fov_deg'], ff['max_range'],
                                  np.where(np.degrees(np.abs(az)) <= nf['wide_fov_deg'], nf['max_range'],
                                           nf['wide_max_range']))
        self.scan = (az, self.scan_rmax.copy(), np.zeros(len(az), bool))
        self._ground = np.array([CLASS['road'], CLASS['lane_white'], CLASS['lane_yellow'], CLASS['terrain']])

    def run(self, T_world_sensor, lin_vel=(0, 0, 0), ang_vel=(0, 0, 0), npc_kind=None):
        o = self.graph.run(T_world_sensor, lin_vel, ang_vel)
        # 스캔 면: 방위각 칸마다 노면이 아닌 첫 히트 거리
        xyz_all, d_all, eid_all = o[rgl.XYZ_VEC3_F32], o[rgl.DISTANCE_F32], o[rgl.ENTITY_ID_I32]
        r = self.scan_rmax.copy()
        hit = np.zeros(len(r), bool)
        solid = ~np.isin(eid_all // ID_SCALE, self._ground)
        if solid.any():
            azh = np.arctan2(xyz_all[solid, 1], xyz_all[solid, 0])
            k = np.clip(np.round((azh - self.scan_az[0]) / (self.scan_az[1] - self.scan_az[0])).astype(int), 0, len(r) - 1)
            dmin = np.full(len(r), np.inf)
            np.minimum.at(dmin, k, d_all[solid])
            hit = dmin < r
            r = np.where(hit, dmin, r)
        self.scan = (self.scan_az, r, hit)
        xyz = o[rgl.XYZ_VEC3_F32].astype(np.float64)
        d = o[rgl.DISTANCE_F32].astype(np.float64)
        eid = o[rgl.ENTITY_ID_I32]
        vr = o[rgl.RADIAL_SPEED_F32].astype(np.float64)
        inc = o[rgl.INCIDENT_ANGLE_F32].astype(np.float64)
        ok = np.isfinite(vr)
        xyz, d, eid, vr, inc = xyz[ok], d[ok], eid[ok], vr[ok], inc[ok]
        if len(d) == 0:
            return np.zeros(0, CLUSTER_DTYPE)
        az = np.arctan2(xyz[:, 1], xyz[:, 0])
        cls = eid // ID_SCALE
        inst = eid % ID_SCALE
        rcs = self._rcs_lut[np.clip(cls, 0, len(self._rcs_lut) - 1)]
        veh = cls == CLASS['vehicle']
        if veh.any() and npc_kind:
            truck = np.array([npc_kind.get(int(i)) in ('truck', 'bus') for i in inst[veh]])
            rcs[veh] = np.where(truck, RCS['vehicle_truck'], RCS['vehicle_car'])
        rcs_eff = rcs + 40.0 * np.log10(np.maximum(np.cos(inc), 1e-3))   # 비스듬한 면(노면 등)은 약함
        snr = rcs_eff - 40.0 * np.log10(np.maximum(d, 0.2)) + self.SNR_CONST
        if self.env is not None:
            snr = snr - self.env.radar_snr_loss
        nf, ff = self.cfg['near'], self.cfg['far']
        absdeg = np.degrees(np.abs(az))
        near = (d <= nf['max_range']) & ((absdeg <= nf['wide_fov_deg']) | (d <= nf['wide_max_range']))
        far = ~near & (absdeg <= ff['fov_deg']) & (d <= ff['max_range'])
        valid = (near | far) & (snr > 0)
        out = []
        for scope_mask, sc, is_far in [(near & valid, nf, 0), (far & valid, ff, 1)]:
            if not scope_mask.any():
                continue
            ii = np.nonzero(scope_mask)[0]
            rb = np.floor(d[ii] / sc['range_res']).astype(np.int64)
            ab = np.floor(np.degrees(az[ii]) / sc['az_res_deg']).astype(np.int64)
            key = rb * 100000 + ab
            order = np.lexsort((-snr[ii], key))           # 셀마다 SNR 최대 점 하나
            ks = key[order]
            first = np.r_[True, ks[1:] != ks[:-1]]
            sel = ii[order[first]]
            n = len(sel)
            dd = d[sel] + self.rng.normal(0, sc['range_std'], n)
            aa = az[sel] + np.radians(self.rng.normal(0, sc['az_std_deg'], n))
            c = np.zeros(n, CLUSTER_DTYPE)
            c['x'], c['y'], c['z'] = dd * np.cos(aa), dd * np.sin(aa), 0.0   # ARS408은 2D 출력
            c['distance'], c['azimuth'] = dd, aa
            c['radial_speed'] = vr[sel] + self.rng.normal(0, self.cfg['velocity_std'], n)
            c['rcs'] = rcs[sel] + self.rng.normal(0, 1.0, n)
            c['snr'] = snr[sel]
            c['far'] = is_far
            c['label'] = cls[sel]
            c['instance'] = inst[sel]
            out.append(c)
        if not out:
            return np.zeros(0, CLUSTER_DTYPE)
        c = np.concatenate(out)
        if len(c) > self.cfg['max_clusters']:
            c = c[np.argsort(-c['snr'])[:self.cfg['max_clusters']]]
        return c

    _rcs_lut = np.array([RCS.get(ID_TO_CLASS.get(c, ''), RCS['vehicle_car'] if ID_TO_CLASS.get(c) == 'vehicle'
                                 else -10.0) for c in range(max(CLASS.values()) + 1)])
