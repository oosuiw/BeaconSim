"""모드·조건 공용 로직 (화면과 무관): 실행 조건 묶음, 변경 감지·되돌리기, 프리셋·시나리오 YAML 적용, 요약 문구.
ImGui 앱(app_start.StartMixin)과 Godot 백엔드(server.Backend)가 함께 쓴다.

필요한 속성: params, sparams, tparams, eparams (생성 파라미터), test (시험 설정 dict), applied (실행 중 조건),
  mode, base_cfg, rand_map, rand_scn, gnss_override, log(msg)
"""
import copy
from dataclasses import fields

import yaml

from .environment import EnvParams
from .evaluate import DEFAULT_CRITERIA
from .roadgen import RoadParams
from .session import PRESETS
from .traffic import ScenarioParams, TrafficParams

MODE_KO = {'dev': '개발', 'test': '시험', 'regress': '회귀'}
GROUP_KO = {'map': '맵', 'scenario': '시나리오', 'traffic': '교통', 'environment': '환경', 'test': '시험'}
KEY_KO = {'seed': '시드', 'length': '길이', 'lanes': '차로 수', 'design_speed': '설계속도', 'tunnels': '터널',
          'gantries': '갠트리', 'wear': '차선 불량', 'wear_faded': '희미함', 'wear_broken': '끊김', 'wear_sparse': '희소',
          'side': '방향', 'lane': '비콘 차로', 'accident_s': '사고 위치(s)', 'accident_where': '사고 위치',
          'accident_vehicles': '사고 차량', 'accident_type': '사고 유형', 'pedestrians': '보행자',
          'p_ped_in_lane': '차로 위 보행자 확률', 'beacon_light': '경광등', 'warning_pattern': '경광등 점멸', 'flash_hz': '경광등 점멸 주기', 'searchlight': '서치라이트',
          'moon': '달빛', 'sky_glow': '하늘 밝음', 'street_lights': '가로등', 'street_coverage': '가로등 구간 비율',
          'lamp_fail': '고장 가로등', 'tower_auto': '센서타워 자동 펴기',
          'tower_deploy_at': '센서타워 펴기 시각', 'start_gap': '비콘 간격', 'enabled': '교통 켜기',
          'flow': '교통량', 'flow_opposite': '반대 교통량', 'speed_mean': '평균 속도', 'truck_ratio': '대형차',
          'p_lane_change': '차로변경', 'p_brake': '감속', 'p_crash': '무반응', 'p_scc': 'SCC 비율',
          'time_of_day': '시간대', 'weather': '날씨', 'intensity': '강도', 'duration': '시험 시간',
          'criteria': '판정 기준', 'faults': '고장 주입', 'topics': '평가 토픽'}
ATYPE_KO = {'basic': '단독 사고', 'chain': '연쇄 추돌', 'truck': '대형차 차로 막음', 'random': '유형 랜덤'}
INT_CRITERIA = {'max_ped_hits', 'max_beacon_hits', 'max_beacon_contacts', 'max_crashes', 'min_scc_locks'}
TIME_KO = {'random': '랜덤', 'day': '낮', 'dusk': '황혼', 'night': '밤'}
WX_KO = {'random': '랜덤', 'clear': '맑음', 'rain': '비', 'fog': '안개', 'snow': '눈'}
WHERE_KO = {'random': '랜덤', 'open': '개활지', 'tunnel_entry': '터널 입구', 'tunnel': '터널 안', 'tunnel_exit': '터널 출구'}
SIDE_KO = {'random': '랜덤', 'down': '하행', 'up': '상행'}


SETTINGS_VER = 2       # app.json 형식: 2 = 센서타워 자동 펴기 기본 켬


def migrate_settings(d):
    """예전 app.json → 지금 기본값 (한 번만). 반환: 같은 dict"""
    sc = d.get('scenario')
    if isinstance(sc, dict):                         # 예전 키 이름 (마스트 → 센서타워)
        for old, new in (('mast_auto', 'tower_auto'), ('mast_deploy_at', 'tower_deploy_at')):
            if old in sc:
                sc.setdefault(new, sc.pop(old))
    if int(d.get('settings_ver', 0)) < 2 and isinstance(d.get('scenario'), dict):
        d['scenario']['tower_auto'] = True          # 자동 펴기를 기본으로 바꿈 (v0.3.x)
    d['settings_ver'] = SETTINGS_VER
    return d


def default_test():
    return {'name': '시험', 'description': '', 'duration': 120.0, 'criteria': dict(DEFAULT_CRITERIA), 'faults': [],
            'topics': {}}


def same(a, b):
    """ImGui 슬라이더는 float32 로 돌려주므로 (0.3 → 0.30000001) 실수는 허용오차로 비교."""
    if isinstance(a, float) or isinstance(b, float):
        try:
            return abs(float(a) - float(b)) <= 1e-5 * max(1.0, abs(float(a)))
        except (TypeError, ValueError):
            return False
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(same(a[k], b[k]) for k in a)
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        return len(a) == len(b) and all(same(x, y) for x, y in zip(a, b))
    return a == b


def pick(cls, d):
    names = {f.name for f in fields(cls)}
    return cls(**{k: v for k, v in d.items() if k in names})


class CondMixin:
    # ------------------------------------------------------------ 조건 (변경 감지 / 되돌리기 / 프리셋 / YAML)
    def _cond(self, mode=None):
        d = {'map': self.params.to_dict(), 'scenario': self.sparams.to_dict(), 'traffic': self.tparams.to_dict(),
             'environment': self.eparams.to_dict()}
        if (mode or self.mode) == 'test':
            d['test'] = {k: self.test.get(k) for k in ('duration', 'criteria', 'faults', 'topics')}
        return copy.deepcopy(d)

    def _changes(self):
        if self.applied is None or self.mode not in ('dev', 'test'):
            return []
        cur, out = self._cond(), []
        for g, b in cur.items():
            a = self.applied.get(g, {})
            out += [(g, k) for k in b if not same(a.get(k), b.get(k))]
        return out

    def _revert(self):
        a = self.applied
        self.params, self.sparams = pick(RoadParams, a['map']), pick(ScenarioParams, a['scenario'])
        self.tparams, self.eparams = pick(TrafficParams, a['traffic']), pick(EnvParams, a['environment'])
        if 'test' in a:
            self.test.update(copy.deepcopy(a['test']))

    def _apply_overrides(self, ov, reset=False):
        """시나리오 YAML 형식 덮어쓰기 (프리셋, 불러오기). reset=True 면 sim.yaml 기본값에서 시작."""
        base = self.base_cfg
        for key, cls, attr in [('map', RoadParams, 'params'), ('scenario', ScenarioParams, 'sparams'),
                               ('traffic', TrafficParams, 'tparams'), ('environment', EnvParams, 'eparams')]:
            cur = dict(base.get(key, {})) if reset else getattr(self, attr).to_dict()
            cur.update(ov.get(key, {}) or {})
            setattr(self, attr, pick(cls, cur))
        if self.params.seed < 0:
            self.params.seed = 1
        if ov.get('tunnels_min'):
            self.params.tunnels = max(self.params.tunnels, ov['tunnels_min'])
        for k in ('rand_map', 'rand_scn'):
            if k in ov:
                setattr(self, k, ov[k])

    def _apply_preset(self, k):
        name, _, ov = PRESETS[k]
        self._apply_overrides(ov)
        self.log(f'프리셋: {name}')

    def load_scenario_yaml(self, path):
        with open(path) as f:
            sc = yaml.safe_load(f) or {}
        self._apply_overrides(sc, reset=True)
        if 'seed' in (sc.get('map') or {}):
            self.rand_map = False
        if 'seed' in (sc.get('scenario') or {}):
            self.rand_scn = False
        self.gnss_override = dict(sc.get('gnss') or {})
        t = default_test()
        t.update({k: sc[k] for k in ('name', 'description', 'duration', 'criteria', 'faults', 'topics') if k in sc})
        t['criteria'] = dict(t.get('criteria') or {})
        self.test = t
        if sc.get('beacon_script'):
            self.log('beacon_script 는 GUI 에서 쓰지 않습니다 (비콘은 /cmd_vel 또는 키보드). 회귀 모드에서는 사용됩니다.')
        self.log(f'시나리오 불러옴: {path}')

    def summary_rows(self, mode=None):
        """시작 화면 요약: [(아이콘 키, 항목, 내용)]."""
        p, sp, tp, e = self.params, self.sparams, self.tparams, self.eparams
        dice = ' (랜덤)'
        rows = [
            ('road', '맵', f'시드 {p.seed}{dice if self.rand_map else ""} · {p.length:.0f} m · {p.lanes}차로 · '
                          f'{p.design_speed} km/h · 터널 ≤{p.tunnels} · 차선 불량 {p.wear:.0%}'),
            ('burst', '사고', f'{SIDE_KO.get(sp.side, sp.side)} · '
                              f'{"랜덤" if sp.lane < 0 else f"{sp.lane + 1}차로"} · '
                              f'{WHERE_KO.get(sp.accident_where, sp.accident_where)} · '
                              f'{ATYPE_KO.get(sp.accident_type, sp.accident_type)} · 사고 차량 {sp.accident_vehicles}대'
                              + (f' · 보행자 {"랜덤" if sp.pedestrians < 0 else f"{sp.pedestrians}명"}' if sp.pedestrians else '')
                              + (' · 시나리오 시드 랜덤' if self.rand_scn or sp.seed < 0 else f' · 시드 {sp.seed}')),
            ('car', '교통', (f'켬 · {tp.flow:.0f} 대/h/차로 · SCC {tp.p_scc:.0%}' if tp.enabled else '끔 (차량 없음)')),
            ('cloud', '환경', f'{TIME_KO.get(e.time_of_day)} · {WX_KO.get(e.weather)} · '
                              f'강도 {"랜덤" if e.intensity < 0 else f"{e.intensity:.0%}"}'),
        ]
        if mode == 'test':
            t = self.test
            dur = t.get('duration') or 0
            rows.append(('stopwatch', '시험', f'{"제한 없음" if not dur else f"{dur:.0f} s"} · 기준 '
                                             f'{len(t.get("criteria") or {})}개 · 고장 {len(t.get("faults") or [])}건'))
        return rows

    def change_labels(self):
        return [f'{GROUP_KO[g]} {KEY_KO.get(k, k)}' for g, k in self._changes()]
