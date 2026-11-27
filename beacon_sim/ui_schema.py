"""Godot 사이드바 정의 (파이썬이 항목을 정하고 Godot 이 그린다).

탭 = 카드 목록, 카드 = {title, icon, color, items}. 항목 종류(t):
  int / float(min,max,step,fmt) / bool / choice(opts) / seg(opts) / dir / note / info(id) / button(action) /
  group(label, items: 접히는 묶음) / presets(시점 버튼)
공통: k = 값 키 ('map.seed' 처럼 묶음.이름), label, tip, enable = 'key' 또는 '!key' (그 값이 참일 때만 조작 가능).
값 키 묶음: map/scn/traffic/env = 생성 파라미터, app = 랜덤 시드 등, key = 방향키, export = 저장,
  view = 화면 표시 (Godot 이 바로 적용, 백엔드는 저장만).
"""
import os
import time
from dataclasses import fields

from .evaluate import CRITERIA_DEFAULT_VALUE, CRITERIA_KO
from .modes import INT_CRITERIA, MODE_KO
from .session import PRESETS, SENSORS_KO

# Font Awesome 6 Solid 코드
I = dict(road=0xf018, burst=0xf5e1, car=0xf5e4, cloud=0xf743, eye=0xf06e, floppy=0xf0c7, grip=0xf7a5, info=0xf05a,
         keyboard=0xf11c, user=0xf007, gauge=0xf625, dish=0xf7c0, layer=0xf5fd, sliders=0xf1de, video=0xf03d,
         terminal=0xf120, download=0xf019, file_export=0xf56e, folder=0xf07c, map=0xf279, rotate=0xf2f9,
         clipboard=0xf46c, check=0xf058, bolt=0xf0e7, plug=0xf1e6, trash=0xf1f8, plus=0xf067, stopwatch=0xf2f2)

# 화면 표시 기본값 (Godot 쪽 show 와 같은 키)
VIEW_DEFAULT = dict(pandar=True, tele=True, radar=True, radar_fov=False, gnss=True, camera=True, labels=True,
                    vehicles=True, scc=True, robot=True, markers=True, lanelet=False, terrain=True, tunnel=True,
                    auto_roof=True, env_view=True, legend=True, pandar_color=0, ui_zoom=0.0, log_filter=0)

GROUPS = {'map': 'params', 'scn': 'sparams', 'traffic': 'tparams', 'env': 'eparams'}


def _f(k, label, lo, hi, fmt='%.2f', step=0.01, **kw):
    return dict(t='float', k=k, label=label, min=lo, max=hi, fmt=fmt, step=step, **kw)


def _i(k, label, lo, hi, **kw):
    return dict(t='int', k=k, label=label, min=lo, max=hi, **kw)


def _b(k, label, **kw):
    return dict(t='bool', k=k, label=label, **kw)


def _card(title, icon, items, color='text'):
    return dict(title=title, icon=I[icon], color=color, items=items)


def tabs(be, mode=None, start=False):
    """mode: 'test' 이면 시험 설정 탭 추가. start=True 면 시작 화면용 (맵·시나리오·교통·환경 [+시험], 버튼·정보 없음)."""
    lanes = be.params.lanes
    tab_map = [
        _card('도로 생성', 'road', [
            _i('map.seed', '맵 시드', 0, 2 ** 31 - 1),
            _b('app.rand_map', '재생성할 때마다 맵 시드 랜덤'),
            _f('map.length', '길이', 1000, 10000, '%.0f m', 100),
            _i('map.lanes', '방향별 차로', 2, 4, slider=True),
            dict(t='choice', k='map.design_speed', label='설계속도', opts=[[100, '100 km/h'], [120, '120 km/h']],
                 tip='최소 곡선반경: 100 km/h → 460 m, 120 km/h → 710 m'),
            _i('map.tunnels', '터널 (최대)', 0, 4, slider=True),
            _i('map.gantries', '갠트리', 0, 5, slider=True)]),
        _card('차선 상태', 'grip', [
            _f('map.wear', '불량 구간 비율', 0, 1, tip='차선 도색 상태가 나쁜 구간의 비율 (Lanelet 맵에는 선이 그대로 남음)'),
            _f('map.wear_faded', '희미함', 0, 1),
            _f('map.wear_broken', '끊김', 0, 1),
            _f('map.wear_sparse', '희소', 0, 1),
            dict(t='group', label='고급', items=[
                _f('map.tunnel_len_min', '터널 최소', 100, 2000, '%.0f m', 10),
                _f('map.tunnel_len_max', '터널 최대', 100, 3000, '%.0f m', 10),
                _f('map.lane_width', '차로 폭', 3.25, 3.75, '%.2f m', 0.05),
                _f('map.dash_len', '점선 도색', 3, 15, '%.1f m', 0.5),
                _f('map.dash_gap', '점선 빈칸', 3, 20, '%.1f m', 0.5),
                _f('map.region_lat', '지역 위도', 33, 39, '%.4f', 0.0001),
                _f('map.region_lon', '지역 경도', 124, 132, '%.4f', 0.0001)])]),
        dict(t='button', label='맵 재생성  (G)', icon=I['map'], action='regen_map', primary=True),
        _card('현재 맵', 'info', [dict(t='info', id='map')], 'muted'),
    ]
    tab_scn = [
        _card('사고 현장', 'burst', [
            _i('scn.seed', '시나리오 시드', 0, 2 ** 31 - 1),
            _b('app.rand_scn', '재시작할 때마다 시나리오 시드 랜덤'),
            dict(t='choice', k='scn.side', label='방향', opts=[['random', '랜덤'], ['down', '하행'], ['up', '상행']]),
            dict(t='choice', k='scn.lane', label='비콘 차로', opts=[[-1, '랜덤']] + [[k, f'{k + 1}차로'] for k in range(lanes)]),
            _i('scn.accident_vehicles', '사고 차량', 1, 3, slider=True),
            dict(t='choice', k='scn.accident_where', label='사고 위치',
                 opts=[['random', '랜덤'], ['open', '개활지'], ['tunnel_entry', '터널 입구'], ['tunnel', '터널 안'],
                       ['tunnel_exit', '터널 출구']]),
            _b('scn.beacon_light', '비콘 경광등 켜기', tip='끄면 야간에 운전자가 비콘을 훨씬 늦게 인지합니다 (인지 거리 0.9배 → 0.35배)')],
            'amber'),
        _card('마스트 (센서함)', 'layer', [
            dict(t='note', text='비콘은 접힌 채로 배치됩니다. 재생 바의 "펴기" 버튼이나 U 키로 펴고 접습니다 '
                              '(상위 제어기는 /beacon/deploy, std_msgs/Bool)'),
            _b('scn.mast_auto', '시나리오 시작 후 자동으로 펴기 (회귀 시험용)'),
            _f('scn.mast_deploy_at', '자동 펴기 시각', 0, 60, '%.0f s', 1, enable='scn.mast_auto')]),
        _card('비콘 키보드 조작', 'keyboard', [
            dict(t='note', text='ROS /cmd_vel 이 들어오면 그쪽이 우선'),
            _f('key.speed', '방향키 속도', 0.2, 3.0, '%.1f m/s', 0.1),
            _f('key.yaw', '방향키 회전', 0.1, 1.5, '%.2f rad/s', 0.05)]),
        dict(t='button', label='시나리오 재시작  (R)', icon=I['rotate'], action='restart', primary=True),
        _card('현재 시나리오', 'info', [dict(t='info', id='scn')], 'muted'),
    ]
    tab_traffic = [
        _card('교통 흐름', 'car', [
            _b('traffic.enabled', '접근 차량 켜기', tip='끄면 기능 검증 모드: 차량 없이 비콘·사고 현장만'),
            _f('traffic.flow', '교통량', 0, 2000, '%.0f 대/h/차로', 10, enable='traffic.enabled'),
            _f('traffic.flow_opposite', '반대 방향', 0, 2000, '%.0f 대/h/차로', 10, enable='traffic.enabled'),
            _f('traffic.speed_mean', '평균 속도', 10, 40, '%.1f m/s', 0.5, enable='traffic.enabled'),
            _f('traffic.truck_ratio', '대형차 비율', 0, 0.6, enable='traffic.enabled')]),
        _card('운전자 반응 (비콘 인지 후)', 'user', [
            _f('traffic.p_lane_change', '차로변경', 0, 1),
            _f('traffic.p_brake', '감속', 0, 1),
            _f('traffic.p_crash', '무반응', 0, 1, tip='비콘을 보지 못하고 그대로 진행 → 추돌 가능'),
            _f('traffic.notice_min', '인지 거리 최소', 10, 300, '%.0f m', 5),
            _f('traffic.notice_max', '인지 거리 최대', 10, 500, '%.0f m', 5),
            dict(t='note', text='비율은 합으로 정규화')], 'warn'),
        _card('SCC (적응형 순항)', 'gauge', [
            _f('traffic.p_scc', '작동 비율', 0, 1),
            _f('traffic.scc_gap_min', '차간 시간 최소', 0.8, 3.0, '%.1f s', 0.1),
            _f('traffic.scc_gap_max', '차간 시간 최대', 0.8, 3.0, '%.1f s', 0.1),
            _f('traffic.scc_decel_min', '최대 감속 최소', 1, 8, '%.1f m/s²', 0.1),
            _f('traffic.scc_decel_max', '최대 감속 최대', 1, 8, '%.1f m/s²', 0.1),
            _f('traffic.scc_range', '레이더 거리', 50, 250, '%.0f m', 5),
            _f('traffic.scc_moving_min', '이동 판정 속도', 0.1, 3.0, '%.1f m/s', 0.1),
            dict(t='note', text='정지한 비콘은 무시, 사고 현장 쪽으로 후진하면 앞차로 인식')], 'scc'),
        dict(t='note', text='변경은 재시작(R) 시 반영'),
    ]
    tab_env = [
        _card('시간대 · 날씨', 'cloud', [
            dict(t='seg', k='env.time_of_day', opts=[['random', '랜덤'], ['day', '낮'], ['dusk', '황혼'], ['night', '밤']]),
            dict(t='seg', k='env.weather', opts=[['random', '랜덤'], ['clear', '맑음'], ['rain', '비'], ['fog', '안개'],
                                                 ['snow', '눈']]),
            _b('env.intensity_rand', '강도 랜덤'),
            _f('env.intensity', '강도', 0, 1, enable='!env.intensity_rand'),
            dict(t='group', label='랜덤 확률', items=[
                _f('env.p_night', '밤', 0, 1), _f('env.p_dusk', '황혼', 0, 1), _f('env.p_rain', '비', 0, 1),
                _f('env.p_fog', '안개', 0, 1), _f('env.p_snow', '눈', 0, 1)])], 'teal'),
        dict(t='note', text='변경은 재시작(R) 시 반영'),
        _card('현재 환경', 'info', [dict(t='info', id='env')], 'muted'),
    ]
    tab_view = [
        _card('센서', 'dish', [
            _b('view.pandar', 'Pandar40P'), _b('view.tele', 'Tele-15'), _b('view.radar', 'ARS408 클러스터·객체'),
            _b('view.radar_fov', 'ARS408 감지 범위 (면)'), _b('view.gnss', 'GNSS 위치·오차원·궤적'),
            _b('view.camera', '카메라 창'),
            dict(t='seg', k='view.pandar_color', label='Pandar 색상', opts=[[0, '세기'], [1, '정답 라벨']])]),
        _card('장면', 'layer', [
            _b('view.labels', '3D 이름표'), _b('view.vehicles', '차량'), _b('view.scc', 'SCC·운전자 상태 표시'),
            _b('view.robot', '비콘'), _b('view.markers', '사고 현장 표시'), _b('view.lanelet', 'Lanelet 경계선'),
            _b('view.terrain', '지형'), _b('view.tunnel', '터널'), _b('view.auto_roof', '비콘이 터널 안이면 천장 숨김'),
            _b('view.env_view', '3D 뷰에도 밤·날씨 적용'), _b('view.legend', '상황판 범례')]),
        _card('UI 크기', 'sliders', [
            _b('view.ui_zoom_auto', '창 크기에 맞춤 (자동)'),
            _f('view.ui_zoom', '배율', 0.75, 2.5, '%.2fx', 0.05, enable='!view.ui_zoom_auto')]),
        _card('시점', 'video', [_b('view.follow', '비콘 따라가기 (F)'), dict(t='presets')]),
    ]
    tab_save = [
        _card('Autoware 맵 세트', 'floppy', [
            dict(t='dir', k='export.dir', label='저장 위치'),
            _b('export.pcd', 'PCD 포함'),
            dict(t='note', text='lanelet2_map.osm · map_projector_info.yaml · pointcloud_map.pcd · scenario.yaml'),
            dict(t='button', label='저장 + 맵 검증', icon=I['download'], action='export', primary=True,
                 busy='export.busy', busy_label='저장 중...')]),
        _card('헤드리스 심', 'terminal', [
            dict(t='note', text='현재 맵·시나리오·교통·환경 설정을 config/sim.yaml 에 기록'),
            dict(t='button', label='sim.yaml 에 적용', icon=I['file_export'], action='apply_sim_yaml')]),
    ]
    out = [dict(id='map', label='맵', icon=I['road'], cards=tab_map),
           dict(id='scn', label='시나리오', icon=I['burst'], cards=tab_scn),
           dict(id='traffic', label='교통', icon=I['car'], cards=tab_traffic),
           dict(id='env', label='환경', icon=I['cloud'], cards=tab_env)]
    if start:
        for t in out:                         # 시작 화면: 재생성 버튼·현재 정보 카드 빼기
            t['cards'] = [c for c in t['cards'] if c.get('t') != 'button' and c.get('color') != 'muted'
                          and not (c.get('t') == 'note' and '재시작' in c.get('text', ''))]
        if mode == 'test':
            out.append(dict(id='test', label='시험 설정', icon=I['clipboard'], cards=tab_test(be)))
        return out
    out.append(dict(id='view', label='보기', icon=I['eye'], cards=tab_view))
    if mode == 'test':
        out.append(dict(id='test', label='시험', icon=I['clipboard'], cards=tab_test(be)))
    else:
        out.append(dict(id='save', label='저장', icon=I['floppy'], cards=tab_save))
    return out


def tab_test(be):
    crit = []
    for k, label in CRITERIA_KO.items():
        if k == 'beacon_not_damaged':
            crit.append(_b(f'test.crit_on.{k}', label))
        else:
            num = dict(t='num', k=f'test.crit.{k}', min=0, max=10000, step=1 if k in INT_CRITERIA else 0.01,
                       enable=f'test.crit_on.{k}', w=110)
            crit.append(dict(t='row', items=[_b(f'test.crit_on.{k}', label, expand=True), num]))
    faults = []
    sens = [[k, v] for k, v in SENSORS_KO.items()]
    for i, _ in enumerate(be.test.get('faults') or []):
        faults.append(dict(t='row', items=[
            dict(t='choice', k=f'test.faults.{i}.sensor', opts=sens, w=130),
            dict(t='num', k=f'test.faults.{i}.t0', min=0, max=3600, step=1, suffix=' s', w=90),
            dict(t='note', text='~'),
            dict(t='num', k=f'test.faults.{i}.t1', min=0, max=3600, step=1, suffix=' s', w=90),
            dict(t='button', label='', icon=I['trash'], action='fault_del', arg=i, w=40)]))
    return [
        _card('시험 정보', 'clipboard', [
            dict(t='text', k='test.name', label='이름'),
            dict(t='text', k='test.description', label='설명'),
            _b('test.dur_on', '시험 시간 제한'),
            _f('test.duration', '시험 시간', 10, 900, '%.0f s', 5, enable='test.dur_on'),
            dict(t='note', text='제한이 없으면 "시험 종료"를 누를 때까지 진행')]),
        _card('판정 기준', 'check', [dict(t='note', text='값은 예시입니다. 실차 요구사항에 맞춰 조정하세요.')] + crit +
              [dict(t='note', text='측위·차선검출 기준은 아래 평가 토픽이 연결될 때만 판정 (없으면 N/A)')], 'ok'),
        _card('센서 고장 주입', 'bolt', faults + [
            dict(t='button', label='고장 추가', icon=I['plus'], action='fault_add'),
            dict(t='note', text='지정 구간 동안 해당 센서 출력 없음 (카메라는 GUI 미적용)')], 'warn'),
        _card('ROS 평가 토픽 (선택)', 'plug', [
            dict(t='text', k='test.topics.localization', label='측위', hint='/localization/kinematic_state'),
            dict(t='choice', k='test.topics.localization_type', label='측위 형식',
                 opts=[['odom', 'Odometry'], ['pose', 'PoseStamped'], ['pose_cov', 'PoseWithCovarianceStamped']]),
            dict(t='text', k='test.topics.lane_detection', label='차선검출', hint='PointCloud2 토픽 (Pandar 좌표)')]),
    ]


def values(be):
    out = {}
    for g, attr in GROUPS.items():
        obj = getattr(be, attr)
        for f in fields(obj):
            out[f'{g}.{f.name}'] = getattr(obj, f.name)
    out['env.intensity_rand'] = be.eparams.intensity < 0
    if be.eparams.intensity < 0:
        out['env.intensity'] = 0.5
    out.update({'app.rand_map': be.rand_map, 'app.rand_scn': be.rand_scn, 'key.speed': be.key_speed,
                'key.yaw': be.key_yaw, 'export.dir': be.export_dir, 'export.pcd': be.export_pcd,
                'export.busy': be.export_busy()})
    for k, v in be.view.items():
        out[f'view.{k}'] = v
    t = be.test
    out['test.name'] = t.get('name', '')
    out['test.description'] = t.get('description', '')
    out['test.dur_on'] = bool(t.get('duration'))
    out['test.duration'] = float(t.get('duration') or 120.0)
    crit = t.get('criteria') or {}
    for k in CRITERIA_KO:
        out[f'test.crit_on.{k}'] = k in crit
        if k != 'beacon_not_damaged':
            out[f'test.crit.{k}'] = crit.get(k, CRITERIA_DEFAULT_VALUE[k])
    for i, f in enumerate(t.get('faults') or []):
        out[f'test.faults.{i}.sensor'] = f.get('sensor', 'gnss')
        out[f'test.faults.{i}.t0'] = float(f.get('t0', 0))
        out[f'test.faults.{i}.t1'] = float(f.get('t1', 0))
    tp = t.get('topics') or {}
    out['test.topics.localization'] = tp.get('localization', '')
    out['test.topics.localization_type'] = tp.get('localization_type', 'odom')
    out['test.topics.lane_detection'] = tp.get('lane_detection', '')
    out['regress.ros'] = be.regress_ros
    out['view.ui_zoom_auto'] = be.view.get('ui_zoom', 0.0) <= 0
    return out


def _coerce(cur, v):
    if isinstance(cur, bool):
        return bool(v)
    if isinstance(cur, int):
        return int(round(float(v)))
    if isinstance(cur, float):
        return float(v)
    return v if isinstance(v, str) else str(v)


def set_value(be, key, v):
    """값 하나 바꾸기. 반환: 탭 정의를 다시 보내야 하면 True (차로 수 → 비콘 차로 목록)."""
    grp, _, name = key.partition('.')
    if grp in GROUPS:
        obj = getattr(be, GROUPS[grp])
        if grp == 'env' and name == 'intensity_rand':
            obj.intensity = -1.0 if v else 0.5
            return False
        if not hasattr(obj, name):
            return False
        setattr(obj, name, _coerce(getattr(obj, name), v))
        p, tp = be.params, be.tparams
        p.tunnel_len_max = max(p.tunnel_len_max, p.tunnel_len_min)          # 최대 ≥ 최소
        tp.notice_max = max(tp.notice_max, tp.notice_min)
        tp.scc_gap_max = max(tp.scc_gap_max, tp.scc_gap_min)
        tp.scc_decel_max = max(tp.scc_decel_max, tp.scc_decel_min)
        if grp == 'map' and name == 'lanes':
            be.sparams.lane = min(be.sparams.lane, p.lanes - 1)
            return True
        if grp == 'scn' and name == 'seed':
            be.rand_scn = False
        return False
    if grp == 'test':
        _set_test(be, name, v)
        return False
    if key == 'regress.ros':
        be.regress_ros = bool(v)
        return False
    if key == 'app.rand_map':
        be.rand_map = bool(v)
    elif key == 'app.rand_scn':
        be.rand_scn = bool(v)
    elif key == 'key.speed':
        be.key_speed = float(v)
    elif key == 'key.yaw':
        be.key_yaw = float(v)
    elif key == 'export.dir':
        be.export_dir = str(v)
    elif key == 'export.pcd':
        be.export_pcd = bool(v)
    elif key == 'view.ui_zoom_auto':
        be.view['ui_zoom'] = 0.0 if v else 1.0
    elif grp == 'view' and name in VIEW_DEFAULT:
        be.view[name] = _coerce(VIEW_DEFAULT[name], v)
    elif grp == 'view' and name == 'follow':
        pass                                    # 시점 따라가기는 저장하지 않음 (Godot 상태)
    return False


def _set_test(be, name, v):
    t = be.test
    if name in ('name', 'description'):
        t[name] = str(v)
    elif name == 'dur_on':
        t['duration'] = 120.0 if v else 0.0
    elif name == 'duration':
        t['duration'] = float(v)
    elif name.startswith('crit_on.'):
        k = name.split('.', 1)[1]
        crit = t.setdefault('criteria', {})
        if v:
            crit[k] = CRITERIA_DEFAULT_VALUE[k]
        else:
            crit.pop(k, None)
    elif name.startswith('crit.'):
        k = name.split('.', 1)[1]
        crit = t.setdefault('criteria', {})
        if k in crit:
            crit[k] = max(0, int(round(float(v)))) if k in INT_CRITERIA else float(v)
    elif name.startswith('faults.'):
        _, i, field = name.split('.')
        faults = t.setdefault('faults', [])
        if int(i) < len(faults):
            faults[int(i)][field] = str(v) if field == 'sensor' else float(v)
    elif name.startswith('topics.'):
        k = name.split('.', 1)[1]
        tp = dict(t.get('topics') or {})
        v = str(v).strip()
        if v and not (k == 'localization_type' and v == 'odom'):
            tp[k] = v
        else:
            tp.pop(k, None)
        t['topics'] = tp


_SC_CACHE = {}


def _scenario_info(f):
    import yaml
    mt = os.path.getmtime(f)
    if f not in _SC_CACHE or _SC_CACHE[f][0] != mt:
        try:
            with open(f) as fh:
                d = yaml.safe_load(fh) or {}
        except Exception:           # noqa: BLE001
            d = {}
        _SC_CACHE[f] = (mt, {k: d.get(k) for k in ('name', 'description', 'duration')})
    return _SC_CACHE[f][1]


def mode_state(be):
    """시작 화면·시험 카드·변경 배너·결과 창·회귀 패널에 필요한 상태 한 묶음."""
    from .common import ROOT
    from .session import list_scenarios
    building = be.build_future is not None and not be.build_future.done()
    out = {'type': 'mode', 'mode': be.mode, 'mode_sel': be.mode_sel, 'prev_mode': be.prev_mode,
           'mode_ko': MODE_KO, 'robot_ready': be.robot_cfg is not None,
           'robot_busy': be.robot_future is not None and not be.robot_future.done(), 'building': building,
           'ros': bool(be.ros is not None and be.ros.ok), 'presets': [[n, d] for n, d, _ in PRESETS],
           'summary': be.summary_rows(be.mode_sel), 'changes': be.change_labels(),
           'test_name': be.test.get('name', '')}
    s = be.session
    if s is not None:
        out['session'] = {'name': s.test.get('name', ''), 'duration': s.duration, 'done': s.done,
                          'ok': bool(s.result['ok']) if s.result else None,
                          'n_iv': len(s.interventions) + (1 if s._iv else 0)}
    r = be.test_result
    if r is not None:
        m = r['metrics']
        out['result'] = {
            'ok': bool(r['ok']), 'name': r['scenario']['name'], 'reason': r['reason'], 'complete': bool(r['complete']),
            'dir': r['dir'], 'sim_time': m.get('sim_time_s'),
            'rows': [[CRITERIA_KO.get(k, k), str(w), '-' if g is None else str(g), st] for k, w, g, st in r['rows']],
            'metrics': (f'추돌 {m["crashes"]} · 비콘 추돌 {m["beacon_hits"]} · 최소 TTC {m["min_ttc_s"]} s · '
                        f'인지 {m["noticed"]} · SCC 인식 {m["scc_locks"]} · GNSS FIX {m["gnss_fix_ratio"]:.0%} · '
                        f'수동 개입 {m["interventions"]}회'),
            'interventions': int(m.get('interventions') or 0)}
        out['result_new'] = be.result_new
        be.result_new = False
    if be.mode_sel == 'regress' or be.mode == 'regress' or be.regress is not None:
        bundled = os.path.join(ROOT, 'scenarios')
        sc = []
        for f in list_scenarios():
            info_ = _scenario_info(f)
            sc.append({'path': f, 'name': info_.get('name') or os.path.basename(f),
                       'desc': info_.get('description') or '', 'duration': info_.get('duration'),
                       'user': not f.startswith(bundled), 'sel': be.regress_sel.get(f, True)})
        out['scenarios'] = sc
        rr = be.regress
        if rr is not None:
            done = sum(r_[1] in ('PASS', 'FAIL') for r_ in rr.rows)
            errs = [ln for ln in rr.lines if 'Error' in ln or 'Traceback' in ln]
            out['regress'] = {'running': rr.running, 'done': done, 'total': len(rr.files),
                              'elapsed': round(time.time() - rr.t0), 'rows': [list(r_) for r_ in rr.rows],
                              'npass': sum(r_[1] == 'PASS' for r_ in rr.rows), 'code': rr.code,
                              'report': rr.report, 'out': rr.out, 'err': errs[-1][:200] if errs else ''}
    return out


def info(be):
    """정보 카드 내용 (줄 목록, 첫 줄은 본문 색, 나머지는 흐린 색)."""
    out = {}
    if be.model is not None:
        sm = be.model.summary()
        tun = ', '.join(f'{a}~{b}' for a, b in sm['tunnels']) or '없음'
        out['map'] = [f'시드 {sm["seed"]} · {sm["length_m"]:.0f} m · {sm["lanes"]}차로×2 · MGRS {sm["mgrs_grid"]}',
                      f'최소반경 {sm["min_radius_m"]:.0f} m · 종단 {sm["max_grade_pct"]:.1f}% · 편경사 {sm["max_superelevation_pct"]:.1f}%',
                      f'터널 {tun} · 차선 불량 {sm["lane_wear_pct"]}%']
    if be.engine is not None:
        sm = be.engine.scn.summary()
        out['scn'] = [f'{sm["side"]} {sm["lane"]}차로 · 사고 s = {sm["accident_s"]:.0f} m · 사고 차량 {sm["accident_vehicles"]}대',
                      f'시드 {be.engine.scn.sp.seed}']
        env = be.engine.env
        out['env'] = [env.label(), f'카메라 대기 산란 {env.fog_dist:.0f} m · LiDAR 감쇠 α {env.alpha:.4f}/m',
                      f'허위점 {env.false_rate:.4f} · 노면 젖음 {env.wet:.2f} · 레이더 SNR −{env.radar_snr_loss:.1f} dB']
    return out
