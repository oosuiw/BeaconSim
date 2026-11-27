"""실환경 조건: 시간대(낮/황혼/밤) + 날씨(맑음/비/안개/눈). 랜덤 또는 지정.

센서 영향 (간이 모델, 실측으로 보정할 값)
  카메라 : 하늘색·태양/주변광·가시거리(안개)·야간 가로등/차량 등화·비/눈 입자·노이즈
  LiDAR  : 대기 감쇠 exp(-2αd) → 유효거리 감소, 비/눈 근거리 허위점, 젖은 노면·차선 반사율 저하
  레이더 : 강우 시 SNR 저하
  GNSS   : 영향 없음
"""
from dataclasses import asdict, dataclass

import numpy as np

TIMES = ('day', 'dusk', 'night')
WEATHERS = ('clear', 'rain', 'fog', 'snow')
TIME_KO = {'day': '낮', 'dusk': '황혼', 'night': '밤'}
WEATHER_KO = {'clear': '맑음', 'rain': '비', 'fog': '안개', 'snow': '눈'}


@dataclass
class EnvParams:
    seed: int = -1                 # -1 = 실행마다 랜덤
    time_of_day: str = 'random'    # random | day | dusk | night
    weather: str = 'random'        # random | clear | rain | fog | snow
    intensity: float = -1.0        # 날씨 강도 0~1, -1 랜덤
    p_night: float = 0.3           # 랜덤일 때 확률
    p_dusk: float = 0.15
    p_rain: float = 0.25
    p_fog: float = 0.1
    p_snow: float = 0.1
    # 야간 조도 (황혼·밤). -1 / 'random' 이면 실행마다 랜덤
    moon: float = -1.0             # 달빛 0(그믐) ~ 1(보름)
    sky_glow: float = -1.0         # 하늘 밝음(도시 광공해) 0(산간) ~ 1(도심 근처)
    street_lights: str = 'random'  # random | all(전 구간) | partial(일부 구간) | none(없음)
    street_coverage: float = -1.0  # partial 일 때 가로등 있는 구간 비율 0~1
    lamp_fail: float = -1.0        # 꺼진(고장) 가로등 비율 0~1

    def to_dict(self):
        return asdict(self)


@dataclass
class Env:
    time_of_day: str
    weather: str
    intensity: float
    # 카메라/렌더
    sky: tuple
    sun: float                     # 태양 직사광 세기
    ambient: float                 # 주변광
    fog_dist: float                # 대기 산란 거리 [m] (작을수록 짙음)
    fog_color: tuple
    street_lights: bool
    lamps: bool                    # 차량 등화 켜짐
    noise: float                   # 영상 노이즈 표준편차 (0~255)
    # LiDAR
    alpha: float                   # 대기 감쇠 계수 [1/m]
    false_rate: float              # 프레임당 근거리 허위점 비율 (레이 대비)
    wet: float                     # 노면 젖음 0~1
    # 레이더
    radar_snr_loss: float          # [dB]
    # 운전자 시인성
    visibility: float = 10000.0    # 기상학적 가시거리 [m] (≈ 3 × 대기 산란 거리)
    seed: int = -1                 # 실제로 쓴 환경 시드 (재현용)
    # 야간 조도 (실제로 쓴 값)
    moon: float = 0.0
    sky_glow: float = 0.0
    street_mode: str = 'all'       # all | partial | none
    street_coverage: float = 1.0
    lamp_fail: float = 0.0

    def label(self):
        s = f'{TIME_KO[self.time_of_day]} / {WEATHER_KO[self.weather]}'
        if self.weather != 'clear':
            s += f' ({self.intensity:.0%})'
        return s

    def night_label(self):
        if self.time_of_day == 'day':
            return ''
        st = {'all': '가로등 전 구간', 'partial': f'가로등 일부 구간 ({self.street_coverage:.0%})', 'none': '가로등 없음'}
        s = f'달빛 {self.moon:.0%} · 하늘 밝음 {self.sky_glow:.0%} · {st.get(self.street_mode, "")}'
        if self.street_mode != 'none' and self.lamp_fail > 0.005:
            s += f' · 고장 {self.lamp_fail:.0%}'
        return s

    def to_dict(self):
        return asdict(self)


def resolve(ep: EnvParams, fallback_seed=0) -> Env:
    seed = ep.seed if ep.seed >= 0 else int(np.random.default_rng().integers(0, 2 ** 31 - 1))
    rng = np.random.default_rng(seed * 2654435761 % (2 ** 32) + fallback_seed)
    tod = ep.time_of_day
    if tod not in TIMES:
        r = rng.random()
        tod = 'night' if r < ep.p_night else ('dusk' if r < ep.p_night + ep.p_dusk else 'day')
    wx = ep.weather
    if wx not in WEATHERS:
        r = rng.random()
        c = np.cumsum([ep.p_rain, ep.p_fog, ep.p_snow])
        wx = 'rain' if r < c[0] else 'fog' if r < c[1] else 'snow' if r < c[2] else 'clear'
    k = float(ep.intensity) if ep.intensity >= 0 else float(rng.uniform(0.2, 1.0))
    if wx == 'clear':
        k = 0.0

    sky = {'day': (0.62, 0.74, 0.88), 'dusk': (0.85, 0.55, 0.40), 'night': (0.02, 0.03, 0.06)}[tod]
    sun = {'day': 0.65, 'dusk': 0.35, 'night': 0.0}[tod]
    amb = {'day': 0.45, 'dusk': 0.30, 'night': 0.06}[tod]
    fog = 900.0
    fog_col = sky
    alpha, false_rate, wet, snr = 0.0, 0.0, 0.0, 0.0
    if wx == 'rain':
        fog = 900.0 - 650.0 * k
        alpha = 0.0005 + 0.004 * k
        false_rate = 0.0005 + 0.003 * k
        wet = 0.5 + 0.5 * k
        snr = 1.0 + 4.0 * k
        sun *= 1 - 0.7 * k
        amb *= 1 - 0.3 * k
        fog_col = tuple(np.array(sky) * 0.6 + 0.25)
    elif wx == 'fog':
        fog = 250.0 - 210.0 * k                    # 가시거리 약 40~250 m
        alpha = 0.004 + 0.03 * k
        false_rate = 0.0002
        wet = 0.2
        snr = 0.5 * k
        sun *= 1 - 0.8 * k
        fog_col = (0.72, 0.74, 0.76) if tod != 'night' else (0.08, 0.08, 0.09)
    elif wx == 'snow':
        fog = 600.0 - 450.0 * k
        alpha = 0.002 + 0.01 * k
        false_rate = 0.002 + 0.008 * k
        wet = 0.4 + 0.3 * k
        snr = 1.0 + 2.0 * k
        sun *= 1 - 0.6 * k
        fog_col = (0.85, 0.86, 0.88) if tod != 'night' else (0.10, 0.10, 0.12)
    if tod != 'day' and wx == 'clear':
        fog_col = sky
    if wx != 'clear':
        sky = tuple(np.array(fog_col))
    noise = {'day': 2.0, 'dusk': 4.0, 'night': 9.0}[tod]
    vis = 10000.0 if wx == 'clear' else float(min(10000.0, 3.0 * fog))
    # 야간 조도: 달빛·광공해·가로등 (값을 정했으면 그대로, 아니면 랜덤). 난수는 항상 같은 순서로 뽑아 재현성 유지
    r_moon, r_glow, r_mode, r_cov, r_fail = rng.random(5)
    moon = float(ep.moon) if ep.moon >= 0 else float(r_moon)
    glow = float(ep.sky_glow) if ep.sky_glow >= 0 else float(r_glow ** 1.5)          # 산간·교외 쪽이 흔함
    mode = ep.street_lights if ep.street_lights in ('all', 'partial', 'none') else \
        ('none' if r_mode < 0.4 else 'partial' if r_mode < 0.75 else 'all')
    cov = float(ep.street_coverage) if ep.street_coverage >= 0 else float(0.2 + 0.6 * r_cov)
    fail = float(ep.lamp_fail) if ep.lamp_fail >= 0 else float(0.15 * r_fail ** 2)
    if mode == 'all':
        cov = 1.0
    elif mode == 'none':
        cov = 0.0
    if wx != 'clear':                       # 구름·강수: 달빛 가림
        moon *= 1.0 - 0.8 * k
    if tod == 'night':                      # 주변광: 달빛 + 광공해
        amb = 0.02 + 0.05 * moon + 0.06 * glow
        noise = 11.0 - 4.0 * min(1.0, moon + glow)
    return Env(tod, wx, k, tuple(map(float, sky)), float(sun), float(amb), float(fog), tuple(map(float, fog_col)),
               tod != 'day' and mode != 'none', tod != 'day' or wx in ('rain', 'fog', 'snow'), float(noise),
               float(alpha), float(false_rate), float(wet), float(snr), vis, int(seed),
               moon, glow, mode, cov, fail)


def notice_scale(env, beacon_light=True):
    """운전자 비콘 인지 거리 배율 (시간대·날씨·경광등). 가시거리 상한은 별도 적용."""
    if env is None:
        return 1.0
    f = {'day': 1.0, 'dusk': 0.85, 'night': 0.9 if beacon_light else 0.35}[env.time_of_day]
    if env.time_of_day == 'day' and not beacon_light:
        f *= 0.85                                   # 낮에도 경광등이 있으면 더 잘 보임
    if env.weather in ('rain', 'snow'):
        f *= 1.0 - 0.35 * env.intensity
    elif env.weather == 'fog':
        f *= 1.0 - 0.2 * env.intensity
    return f
