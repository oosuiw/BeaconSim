"""GUI 시험 모드 세션 + 회귀 실행기.

시험 1회 = 조건 1세트. 시작할 때 랜덤 항목은 모두 실제 값(시드)으로 정해져 시나리오 YAML로 남고,
같은 YAML을 회귀 테스트(tools/run_scenarios.py)나 GUI '불러오기'로 그대로 재현할 수 있다.
"""
import copy
import csv
import json
import os
import re
import subprocess
import sys
import threading
import time

import yaml

from .common import ROOT, USER_DIR, results_root, writable_root
from .evaluate import COLS, Metrics, judge

# 프리셋: 시나리오 YAML과 같은 형식의 덮어쓰기. 'tunnels_min' 은 맵 터널 수 최소값
PRESETS = [
    ('기능 검증 기본', '차량 없음 · 낮 맑음 · 개활지',
     {'traffic': {'enabled': False}, 'scenario': {'accident_where': 'open'},
      'environment': {'time_of_day': 'day', 'weather': 'clear', 'intensity': 0.0}}),
    ('맑은 낮 · 개활지', '가장 기본적인 접근 차량 시험',
     {'traffic': {'enabled': True}, 'scenario': {'accident_where': 'open'},
      'environment': {'time_of_day': 'day', 'weather': 'clear', 'intensity': 0.0}}),
    ('야간 우천 · 터널 입구', '인지 거리 짧음 + GNSS 단절 전환',
     {'traffic': {'enabled': True}, 'scenario': {'accident_where': 'tunnel_entry'}, 'tunnels_min': 1,
      'environment': {'time_of_day': 'night', 'weather': 'rain', 'intensity': 0.7}}),
    ('안개 · 터널 출구', '시야 불량 + 터널 출구 GNSS 재수렴',
     {'traffic': {'enabled': True}, 'scenario': {'accident_where': 'tunnel_exit'}, 'tunnels_min': 1,
      'environment': {'time_of_day': 'day', 'weather': 'fog', 'intensity': 0.8}}),
    ('황혼 눈 · 차선 불량', '차선 희미함/끊김 많음',
     {'traffic': {'enabled': True}, 'scenario': {'accident_where': 'open'}, 'map': {'wear': 0.8},
      'environment': {'time_of_day': 'dusk', 'weather': 'snow', 'intensity': 0.6}}),
    ('전부 랜덤', '맵·시나리오·환경 모두 시작할 때마다 랜덤',
     {'traffic': {'enabled': True}, 'rand_map': True, 'rand_scn': True,
      'scenario': {'side': 'random', 'lane': -1, 'accident_where': 'random', 'accident_s': -1.0},
      'environment': {'seed': -1, 'time_of_day': 'random', 'weather': 'random', 'intensity': -1.0}}),
]

SENSORS_KO = {'pandar40p': 'Pandar40P', 'tele15': 'Tele-15', 'ars408': 'ARS408', 'gnss': 'GNSS', 'camera': '카메라'}


def scenario_dirs():
    """회귀 시나리오 폴더: 동봉(ROOT/scenarios) + 사용자(쓰기 가능한 곳/scenarios)."""
    out = []
    for d in (os.path.join(ROOT, 'scenarios'), os.path.join(writable_root(), 'scenarios'),
              os.path.join(USER_DIR, 'scenarios')):
        if os.path.isdir(d) and d not in out:
            out.append(d)
    return out


def list_scenarios():
    files = []
    for d in scenario_dirs():
        files += sorted(os.path.join(d, f) for f in os.listdir(d) if f.endswith('.yaml'))
    seen, out = set(), []
    for f in files:
        k = os.path.realpath(f)
        if k not in seen:
            seen.add(k)
            out.append(f)
    return out


def safe_name(s):
    s = re.sub(r'[^\w가-힣.-]+', '_', s.strip()) or 'test'
    return s[:60]


class TestSession:
    """시험 1회: 지표 수집(엔진 스레드에서 on_step), 수동 개입·배속 변경 기록, 종료 시 결과 저장."""

    def __init__(self, engine, test, log=print):
        self.e, self.test, self.log = engine, copy.deepcopy(test), log
        self.met = Metrics(engine)
        self.pending = {}
        self.lock = threading.Lock()
        self.interventions = []         # [t0, t1]
        self._iv = None
        self.rtf_log = []               # (t, rtf)
        self.wall0 = time.time()
        self.started = time.strftime('%Y-%m-%d %H:%M:%S')
        self.done = False
        self.result = None              # finish() 결과
        self.subs = []

    @property
    def duration(self):
        return float(self.test.get('duration', 0) or 0)

    # 엔진 스레드
    def on_step(self, e, fired, ev):
        if self.done:
            return
        with self.lock:
            self.pending.update(fired)
            if e.tick % 5 == 0 or ev:
                self.met.update(self.pending, ev)
                self.pending = {}

    # GUI 스레드
    def keyboard(self, active):
        t = round(self.e.t, 2)
        if active and self._iv is None:
            self._iv = [t, None]
            self.log(f'[{t:.1f}s] 수동 개입 시작 (키보드)')
        elif not active and self._iv is not None:
            self._iv[1] = t
            self.interventions.append(self._iv)
            self.log(f'[{t:.1f}s] 수동 개입 끝 ({t - self._iv[0]:.1f} s)')
            self._iv = None

    def rtf_changed(self, rtf):
        self.rtf_log.append((round(self.e.t, 2), rtf))
        self.log(f'[{self.e.t:.1f}s] 배속 {rtf:g}x (리포트에 기록)')

    def resolved_scenario(self):
        """랜덤 항목이 모두 정해진 시나리오 YAML (회귀 테스트 형식)."""
        e = self.e
        env = dict(e.cfg.get('environment', {}))
        env['seed'] = int(e.env.seed)
        sc = {'name': self.test.get('name', 'gui_test'),
              'description': self.test.get('description', '') or f'GUI 시험 ({self.started})',
              'duration': self.duration or round(e.t, 1),
              'map': e.model.params.to_dict(), 'scenario': e.scn.sp.to_dict(),
              'traffic': dict(e.cfg['traffic']), 'environment': env}
        if e.cfg.get('gnss_override'):
            sc['gnss'] = e.cfg['gnss_override']
        if self.test.get('faults'):
            sc['faults'] = self.test['faults']
        if self.test.get('topics'):
            sc['topics'] = self.test['topics']
        sc['criteria'] = self.test.get('criteria', {})
        return sc

    def finish(self, reason, out_root=None):
        """결과 저장 → dict(ok, rows, dir, metrics, ...). 여러 번 불러도 한 번만 저장."""
        if self.done:
            return self.result
        self.keyboard(False)
        with self.lock:
            self.done = True
            res = self.met.result()
            samples = list(self.met.samples)
        e = self.e
        res['sim_time_s'] = round(e.t, 1)
        res['wall_s'] = round(time.time() - self.wall0, 1)
        res['interventions'] = len(self.interventions)
        res['intervention_s'] = round(sum(b - a for a, b in self.interventions), 1)
        rows, ok = judge(res, self.test.get('criteria'))
        complete = not self.duration or e.t >= self.duration - 0.05
        name = safe_name(self.test.get('name', 'gui_test'))
        d = os.path.join(out_root or os.path.join(results_root(), 'gui'), f'{time.strftime("%Y%m%d_%H%M%S")}_{name}')
        os.makedirs(d, exist_ok=True)
        sc = self.resolved_scenario()
        with open(os.path.join(d, 'scenario.yaml'), 'w') as f:
            f.write('# GUI 시험에서 저장된 조건 (랜덤 항목은 실제 값으로 고정됨). 회귀 테스트에 그대로 사용 가능\n')
            yaml.safe_dump(sc, f, allow_unicode=True, sort_keys=False)
        with open(os.path.join(d, 'timeseries.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(COLS)
            w.writerows(samples)
        entry = {'name': sc['name'], 'pass': ok, 'complete': complete, 'end_reason': reason, 'started': self.started,
                 'environment': e.env.label(), 'scenario': e.scn.summary(), 'metrics': res,
                 'interventions': self.interventions, 'rtf_changes': self.rtf_log,
                 'criteria': [dict(zip(['criterion', 'expected', 'actual', 'status'], r)) for r in rows]}
        with open(os.path.join(d, 'result.json'), 'w') as f:
            json.dump(entry, f, ensure_ascii=False, indent=1, default=str)
        md = [f'# 시험 결과 — {sc["name"]}', '',
              f'- 판정: **{"PASS" if ok else "FAIL"}**' + ('' if complete else '  (시험 시간 전에 종료 — 참고용)'),
              f'- 종료: {reason} · 시뮬레이션 {res["sim_time_s"]} s / 실제 {res["wall_s"]} s · 시작 {self.started}',
              f'- 환경: {e.env.label()} · 맵 시드 {sc["map"]["seed"]} · 시나리오 시드 {sc["scenario"]["seed"]} · '
              f'환경 시드 {sc["environment"]["seed"]}',
              f'- 수동 개입: {len(self.interventions)}회 ({res["intervention_s"]} s)'
              + (''.join(f' [{a:.1f}~{b:.1f}s]' for a, b in self.interventions)),
              f'- 배속 변경: ' + (', '.join(f'{t:.1f}s→{r:g}x' for t, r in self.rtf_log) or '없음'), '',
              '| 기준 | 기대 | 실제 | 판정 |', '|---|---|---|---|']
        md += [f'| {k} | {w_} | {"-" if g is None else g} | {s} |' for k, w_, g, s in rows]
        md += ['', '## 지표', '', '| 지표 | 값 |', '|---|---|']
        md += [f'| {k} | {"-" if v is None else v} |' for k, v in res.items()]
        with open(os.path.join(d, 'report.md'), 'w') as f:
            f.write('\n'.join(md) + '\n')
        self.result = dict(ok=ok, rows=rows, dir=d, metrics=res, complete=complete, reason=reason, scenario=sc)
        self.log(f'시험 종료 ({reason}): {"PASS" if ok else "FAIL"} → {d}')
        return self.result


def save_regression(sc, name=None):
    """시험 조건을 회귀 시나리오 폴더에 저장 → 경로."""
    sc = copy.deepcopy(sc)
    if name:
        sc['name'] = name
    d = os.path.join(writable_root(), 'scenarios')
    os.makedirs(d, exist_ok=True)
    p = os.path.join(d, safe_name(sc['name']) + '.yaml')
    k = 2
    while os.path.exists(p):
        p = os.path.join(d, f'{safe_name(sc["name"])}_{k}.yaml')
        k += 1
    with open(p, 'w') as f:
        f.write('# GUI 시험에서 저장한 회귀 시나리오. criteria 값은 팀 기준에 맞게 수정\n')
        yaml.safe_dump(sc, f, allow_unicode=True, sort_keys=False)
    return p


class RegressRun:
    """tools/run_scenarios.py 를 하위 프로세스로 실행하고 진행 상황을 모은다."""

    def __init__(self, files, ros=False, log=print):
        self.files, self.ros, self.log = list(files), ros, log
        self.out = os.path.join(results_root(), time.strftime('%Y%m%d_%H%M%S'))
        self.lines, self.rows = [], []      # rows: [name, 'RUN'|'PASS'|'FAIL', 요약]
        self.report = None
        self.code = None
        cmd = [sys.executable, '-u', os.path.join(ROOT, 'tools', 'run_scenarios.py'), *self.files, '--out', self.out]
        if ros:
            cmd.append('--ros')
        env = dict(os.environ, PYTHONUNBUFFERED='1')
        env['PYTHONPATH'] = ROOT + (os.pathsep + env['PYTHONPATH'] if env.get('PYTHONPATH') else '')
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env,
                                     cwd=ROOT, bufsize=1)
        threading.Thread(target=self._read, daemon=True).start()
        self.t0 = time.time()

    def _read(self):
        for ln in self.proc.stdout:
            ln = ln.rstrip()
            self.lines.append(ln)
            if ln.startswith('▶ '):
                self.rows.append([ln[2:].split(':')[0], 'RUN', ''])
            elif ln.strip().startswith(('PASS', 'FAIL')) and self.rows:
                st, _, rest = ln.strip().partition(' ')
                self.rows[-1][1:] = [st, rest.strip()]
            elif ln.startswith('리포트:'):
                self.report = ln.split(':', 1)[1].split('→')[0].strip()
        self.code = self.proc.wait()
        self.log(f'회귀 테스트 종료 (코드 {self.code}): {self.report or self.out}')

    @property
    def running(self):
        return self.code is None

    def stop(self):
        if self.running:
            self.proc.terminate()
