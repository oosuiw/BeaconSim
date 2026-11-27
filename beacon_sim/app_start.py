"""시작 화면(모드 선택 · 조건 설정 · 회귀 실행)과 시험 모드 UI. App 에 섞어 쓰는 mixin."""
import os
import subprocess
import time

import yaml
from imgui_bundle import imgui

from . import ui_style as S
from .evaluate import CRITERIA_DEFAULT_VALUE, CRITERIA_KO
from . import __version__
from .session import PRESETS, SENSORS_KO, RegressRun, list_scenarios, save_regression, scenario_dirs
from .ui_style import FA
from .modes import (GROUP_KO, INT_CRITERIA, KEY_KO, MODE_KO, SIDE_KO, TIME_KO, WHERE_KO, WX_KO,  # noqa: F401
                    CondMixin, default_test)

MODES = [
    ('dev', FA.ICON_FA_SCREWDRIVER_WRENCH, '개발', '기능 검증',
     '센서 · TF · 상위 제어기 연동 디버깅. 실행 중에도 조건을 바꿔 바로 적용할 수 있습니다.',
     ['교통 끔 (기본)', '실행 중 변경', '평가 없음']),
    ('test', FA.ICON_FA_CLIPBOARD_CHECK, '시험', '시나리오 평가',
     '조건 1세트로 시험하고 PASS/FAIL 리포트를 남깁니다. 시험 중 조건은 잠기고, 바꾸면 재시작합니다.',
     ['교통 켬 (기본)', '조건 잠금', '리포트 · YAML']),
    ('regress', FA.ICON_FA_LIST_CHECK, '회귀', '일괄 실행',
     '저장된 시나리오 YAML 을 차례로 자동 실행해 기준 통과 여부를 판정합니다.',
     ['화면 없이 고속', 'ROS 연동 선택', 'report.md']),
]
class StartMixin(CondMixin):
    # ------------------------------------------------------------ 모드 전환
    def start_mode(self, mode):
        if self.session is not None and not self.session.done:
            self._finish_test('모드 변경', show=False)
        prev = self.mode or self.prev_mode
        self.mode = self.mode_sel = mode
        self.prev_mode = None
        self.result_open = False
        self._save_settings()
        if mode == 'regress':
            return
        changed = prev != mode or self.applied is None or bool(self._changes()) or self.engine is None
        if changed:
            self.paused = False
            map_too = self.model is None or self.applied is None or self.applied['map'] != self._cond()['map']
            self.rebuild(map_too=map_too)
        else:
            self._resume()

    def go_home(self):
        self.prev_mode = self.mode
        self.was_paused = self.paused
        self.mode = None
        if self.eth is not None:
            self.eth.paused = True

    def _resume(self):
        self.paused = getattr(self, 'was_paused', False)
        if self.eth is not None:
            self.eth.paused = self.paused

    def _finish_test(self, reason, show=True):
        s = self.session
        if s is None or s.done:
            return
        res = s.finish(reason)
        if self.ros is not None and self.ros.ok:
            for sub in s.subs:
                try:
                    self.ros.node.destroy_subscription(sub)
                except Exception:   # noqa: BLE001
                    pass
        s.subs = []
        self.test_result = res
        if show:
            self.result_open = True

    # ------------------------------------------------------------ 시작 화면
    def _start_screen(self, W, H):
        sc = self.scale
        imgui.set_next_window_pos(imgui.ImVec2(0, 0))
        imgui.set_next_window_size(imgui.ImVec2(W, H))
        imgui.push_style_color(imgui.Col_.window_bg, S.v4(S.BG0, 1.0))
        imgui.begin('start', None, imgui.WindowFlags_.no_decoration | imgui.WindowFlags_.no_move |
                    imgui.WindowFlags_.no_saved_settings | imgui.WindowFlags_.no_bring_to_front_on_focus)
        imgui.pop_style_color()
        cw = min(1320 * sc, W - 48 * sc)
        x0 = (W - cw) / 2
        imgui.set_cursor_pos(imgui.ImVec2(x0, 28 * sc))
        imgui.begin_group()
        # 헤더
        imgui.push_font(S.Fonts.bold, 30)
        imgui.text_colored(S.v4(S.ACCENT), FA.ICON_FA_TOWER_BROADCAST)
        imgui.same_line()
        imgui.text('BeaconSim')
        imgui.pop_font()
        imgui.same_line()
        imgui.set_cursor_pos_y(imgui.get_cursor_pos_y() + 12 * sc)
        S.muted(f'v{__version__}  ·  세이프티 비콘 시뮬레이터  ·  모드를 고르고 조건을 정한 뒤 시작하세요')
        if self.prev_mode is not None:
            imgui.same_line(cw - 190 * sc)
            if imgui.button(f'{FA.ICON_FA_ARROW_LEFT}  {MODE_KO[self.prev_mode]} 화면으로 돌아가기'):
                self.mode, self.prev_mode = self.prev_mode, None
                self._resume()
        imgui.dummy(imgui.ImVec2(0, 10 * sc))
        # 모드 카드
        gap = 14 * sc
        card_w = (cw - 2 * gap) / 3
        for k, (key, ic, title, sub, desc, chips) in enumerate(MODES):
            if k:
                imgui.same_line(spacing=gap)
            self._mode_card(key, ic, title, sub, desc, chips, card_w, 150 * sc)
        imgui.dummy(imgui.ImVec2(0, 8 * sc))
        # 본문
        body_h = H - imgui.get_cursor_pos_y() - 24 * sc
        lw = cw * 0.64
        imgui.begin_child('st_left', imgui.ImVec2(lw, body_h))
        if self.mode_sel == 'regress':
            self._regress_list()
        else:
            self._cond_editor()
        imgui.end_child()
        imgui.same_line(spacing=gap)
        imgui.begin_child('st_right', imgui.ImVec2(cw - lw - gap, body_h))
        if self.mode_sel == 'regress':
            self._regress_panel()
        else:
            self._start_summary()
        imgui.end_child()
        imgui.end_group()
        imgui.end()

    def _mode_card(self, key, ic, title, sub, desc, chips, w, h):
        sel = self.mode_sel == key
        imgui.push_style_color(imgui.Col_.child_bg, S.v4(S.ACCENT, 0.10) if sel else S.v4(S.BG1))
        imgui.push_style_color(imgui.Col_.border, S.v4(S.ACCENT, 0.9) if sel else S.v4(S.BORDER))
        imgui.push_style_var(imgui.StyleVar_.child_border_size, 2.0 if sel else 1.0)
        sc = self.scale
        imgui.push_style_var(imgui.StyleVar_.window_padding, imgui.ImVec2(16 * sc, 14 * sc))
        imgui.begin_child(f'mode_{key}', imgui.ImVec2(w, h), imgui.ChildFlags_.borders |
                          imgui.ChildFlags_.always_use_window_padding, imgui.WindowFlags_.no_scrollbar)
        hovered = imgui.is_window_hovered()
        imgui.push_font(S.Fonts.bold, 22)
        imgui.text_colored(S.v4(S.ACCENT if sel else (S.TEXT if hovered else S.MUTED)), f'{ic}  {title}')
        imgui.pop_font()
        imgui.same_line()
        imgui.set_cursor_pos_y(imgui.get_cursor_pos_y() + 5 * sc)
        S.muted(sub)
        imgui.push_text_wrap_pos(w - 16 * sc)
        imgui.text_wrapped(desc)
        imgui.pop_text_wrap_pos()
        imgui.set_cursor_pos_y(h - 40 * sc)
        for i, c in enumerate(chips):
            if i:
                imgui.same_line(spacing=4 * sc)
            S.chip(c, S.ACCENT if sel else S.MUTED)
        imgui.set_cursor_pos(imgui.ImVec2(0, 0))
        if imgui.invisible_button('hit', imgui.ImVec2(w, h)) and not sel:
            self.mode_sel = key
            if key in ('dev', 'test'):
                self.tparams.enabled = key == 'test'
                if self.start_tab == 4 and key == 'dev':
                    self.start_tab = 0
        imgui.end_child()
        imgui.pop_style_var(2)
        imgui.pop_style_color(2)

    def _cond_editor(self):
        sc = self.scale
        S.begin_card('st_preset')
        S.section(FA.ICON_FA_SLIDERS, '시험 조건' if self.mode_sel == 'test' else '실행 조건')
        names = ['프리셋 선택...'] + [f'{n}  —  {d}' for n, d, _ in PRESETS]
        imgui.set_next_item_width(imgui.get_content_region_avail().x - 250 * sc)
        ch, i = imgui.combo('##preset', 0, names)
        if ch and i > 0:
            self._apply_preset(i - 1)
        imgui.same_line()
        if imgui.button(f'{FA.ICON_FA_FILE_IMPORT}  YAML 불러오기'):
            try:
                from imgui_bundle import portable_file_dialogs as pfd
                start = scenario_dirs()[-1] if scenario_dirs() else os.path.expanduser('~')
                r = pfd.open_file('시나리오 YAML 선택', start, ['YAML', '*.yaml *.yml']).result()
                if r:
                    self.load_scenario_yaml(r[0])
            except Exception as e:      # noqa: BLE001
                self.log(f'불러오기 실패: {e}')
        S.tip('회귀 시나리오나 시험 결과 폴더의 scenario.yaml → 같은 조건 재현')
        imgui.same_line()
        if imgui.button(f'{FA.ICON_FA_ROTATE_LEFT}  기본값'):
            self._apply_overrides({}, reset=True)
            self.test = default_test()
            self.gnss_override = {}
            self.tparams.enabled = self.mode_sel == 'test'
        S.tip('config/sim.yaml 값으로 되돌리기')
        S.end_card()
        tabs = ['맵', '시나리오', '교통', '환경'] + (['시험 설정'] if self.mode_sel == 'test' else [])
        self.start_tab = min(self.start_tab, len(tabs) - 1)
        self.start_tab = S.segmented(tabs, self.start_tab, (imgui.get_content_region_avail().x - 8) / len(tabs))
        imgui.spacing()
        imgui.begin_child('st_tab', imgui.ImVec2(0, 0))
        [self._tab_map, lambda: self._tab_scenario(None), self._tab_traffic, self._tab_env, self._tab_test][self.start_tab]()
        imgui.end_child()

    def _summary_rows(self):
        p, sp, tp, e = self.params, self.sparams, self.tparams, self.eparams
        dice = f' {FA.ICON_FA_DICE}'
        rows = [
            (FA.ICON_FA_ROAD, '맵', f'시드 {p.seed}{dice if self.rand_map else ""} · {p.length:.0f} m · {p.lanes}차로 · '
                                   f'{p.design_speed} km/h · 터널 ≤{p.tunnels} · 차선 불량 {p.wear:.0%}'),
            (FA.ICON_FA_CAR_BURST, '사고', f'{SIDE_KO.get(sp.side, sp.side)} · '
                                         f'{"랜덤" if sp.lane < 0 else f"{sp.lane + 1}차로"} · '
                                         f'{WHERE_KO.get(sp.accident_where, sp.accident_where)} · 사고 차량 {sp.accident_vehicles}대'
                                         + (' · 시나리오 시드 랜덤' if self.rand_scn or sp.seed < 0 else f' · 시드 {sp.seed}')),
            (FA.ICON_FA_CAR_SIDE, '교통', (f'켬 · {tp.flow:.0f} 대/h/차로 · SCC {tp.p_scc:.0%}' if tp.enabled
                                          else '끔 (차량 없음)')),
            (FA.ICON_FA_CLOUD_SUN_RAIN, '환경', f'{TIME_KO.get(e.time_of_day)} · {WX_KO.get(e.weather)} · '
                                               f'강도 {"랜덤" if e.intensity < 0 else f"{e.intensity:.0%}"}'),
        ]
        if self.mode_sel == 'test':
            t = self.test
            dur = t.get('duration') or 0
            rows.append((FA.ICON_FA_STOPWATCH, '시험', f'{"제한 없음" if not dur else f"{dur:.0f} s"} · 기준 '
                                                     f'{len(t.get("criteria") or {})}개 · 고장 {len(t.get("faults") or [])}건'))
        return rows

    def _start_summary(self):
        sc = self.scale
        S.begin_card('st_sum')
        S.section(FA.ICON_FA_CIRCLE_INFO, '요약', S.MUTED)
        for ic, k, v in self._summary_rows():
            imgui.text_colored(S.v4(S.ACCENT), ic)
            imgui.same_line(36 * sc)
            imgui.text_colored(S.v4(S.MUTED), k)
            imgui.same_line(80 * sc)
            imgui.push_text_wrap_pos(0)
            imgui.text_wrapped(v)
            imgui.pop_text_wrap_pos()
        S.end_card()
        S.begin_card('st_ros')
        ros = self.ros is not None and self.ros.ok
        S.chip('ROS2 연동' if ros else 'ROS2 없음 (단독 실행)', S.ACCENT if ros else S.MUTED, FA.ICON_FA_LINK)
        imgui.same_line()
        S.chip('비콘 조작: /cmd_vel → 키보드', S.MUTED, FA.ICON_FA_GAMEPAD)
        if self.mode_sel == 'test':
            S.muted('시험 중 조건은 잠깁니다. 바꾸면 "이 조건으로 재시작".')
            S.muted('키보드 조작은 "수동 개입"으로 리포트에 기록됩니다.')
        else:
            S.muted('개발 모드: 조건을 바꾸고 "적용"을 누르면 바로 반영됩니다.')
        S.end_card()
        imgui.spacing()
        busy = self.build_future is not None and not self.build_future.done()
        imgui.begin_disabled(busy or self.robot_parts is None)
        label = {'dev': f'{FA.ICON_FA_PLAY}  개발 시작', 'test': f'{FA.ICON_FA_FLAG_CHECKERED}  시험 시작'}[self.mode_sel]
        if S.primary_button(('준비 중...' if busy else label), imgui.ImVec2(-1, 48 * sc)):
            self.start_mode(self.mode_sel)
        imgui.end_disabled()
        if self.test_result is not None:
            imgui.spacing()
            self._result_card(self.test_result, 'st_last')

    # ------------------------------------------------------------ 시험 설정 탭
    def _tab_test(self):
        t = self.test
        sc = self.scale
        S.begin_card('t_info')
        S.section(FA.ICON_FA_CLIPBOARD_CHECK, '시험 정보')
        _, t['name'] = imgui.input_text('이름', t.get('name', ''))
        _, t['description'] = imgui.input_text('설명', t.get('description', ''))
        lim = bool(t.get('duration'))
        ch, lim = imgui.checkbox('시험 시간 제한', lim)
        if ch:
            t['duration'] = 120.0 if lim else 0.0
        if lim:
            imgui.same_line()
            imgui.set_next_item_width(-1)
            _, t['duration'] = imgui.slider_float('##dur', float(t['duration']), 10.0, 900.0, '%.0f s')
        else:
            S.muted('제한 없음: "시험 종료"를 누를 때까지')
        S.end_card()
        S.begin_card('t_crit')
        S.section(FA.ICON_FA_CIRCLE_CHECK, '판정 기준', S.OK)
        S.muted('값은 예시입니다. 실차 요구사항에 맞춰 조정하세요.')
        crit = t.setdefault('criteria', {})
        for k, label in CRITERIA_KO.items():
            imgui.push_id(k)
            on = k in crit
            ch, on = imgui.checkbox(label, on)
            if ch:
                if on:
                    crit[k] = CRITERIA_DEFAULT_VALUE[k]
                else:
                    crit.pop(k, None)
            if on and k != 'beacon_not_damaged':
                imgui.same_line(imgui.get_content_region_avail().x + imgui.get_cursor_pos_x() - 110 * sc)
                imgui.set_next_item_width(110 * sc)
                if k in INT_CRITERIA:
                    _, v = imgui.input_int('##v', int(crit[k]))
                    crit[k] = max(0, v)
                else:
                    _, crit[k] = imgui.input_float('##v', float(crit[k]), 0.0, 0.0, '%.2f')
            imgui.pop_id()
        S.muted('측위·차선검출 기준은 아래 평가 토픽이 연결될 때만 판정 (없으면 N/A)')
        S.end_card()
        S.begin_card('t_fault')
        S.section(FA.ICON_FA_BOLT, '센서 고장 주입', S.WARN)
        faults = t.setdefault('faults', [])
        names = list(SENSORS_KO)
        drop = None
        for i, f in enumerate(faults):
            imgui.push_id(f'f{i}')
            imgui.set_next_item_width(120 * sc)
            ch, k = imgui.combo('##s', names.index(f['sensor']) if f.get('sensor') in names else 0,
                                [SENSORS_KO[n] for n in names])
            if ch:
                f['sensor'] = names[k]
            imgui.same_line()
            imgui.set_next_item_width(90 * sc)
            _, f['t0'] = imgui.input_float('##t0', float(f.get('t0', 0)), 0, 0, '%.0f s')
            imgui.same_line()
            imgui.text('~')
            imgui.same_line()
            imgui.set_next_item_width(90 * sc)
            _, f['t1'] = imgui.input_float('##t1', float(f.get('t1', 0)), 0, 0, '%.0f s')
            imgui.same_line()
            if imgui.button(FA.ICON_FA_TRASH):
                drop = i
            imgui.pop_id()
        if drop is not None:
            faults.pop(drop)
        if imgui.button(f'{FA.ICON_FA_PLUS}  고장 추가'):
            faults.append({'sensor': 'gnss', 't0': 30.0, 't1': 40.0})
        S.muted('지정 구간 동안 해당 센서 출력 없음 (카메라는 GUI 미적용)')
        S.end_card()
        S.begin_card('t_topic')
        S.section(FA.ICON_FA_PLUG, 'ROS 평가 토픽 (선택)')
        tp = t.setdefault('topics', {})
        _, loc = imgui.input_text_with_hint('측위', '/localization/kinematic_state', tp.get('localization', ''))
        types = ['odom', 'pose', 'pose_cov']
        ch, k = imgui.combo('측위 형식', types.index(tp.get('localization_type', 'odom')),
                            ['Odometry', 'PoseStamped', 'PoseWithCovarianceStamped'])
        _, lane = imgui.input_text_with_hint('차선검출', 'PointCloud2 토픽 (Pandar 좌표)', tp.get('lane_detection', ''))
        new = {}
        if loc.strip():
            new['localization'] = loc.strip()
            if types[k] != 'odom':
                new['localization_type'] = types[k]
        if lane.strip():
            new['lane_detection'] = lane.strip()
        t['topics'] = new
        S.end_card()

    # ------------------------------------------------------------ 시험 진행 (사이드바) / 결과
    def _test_card(self, snap):
        s = self.session
        sc = self.scale
        S.begin_card('test_now')
        if s is None:
            S.section(FA.ICON_FA_CLIPBOARD_CHECK, '시험 준비 중', S.MUTED)
            S.end_card()
            return
        name = s.test.get('name', '')
        if s.done:
            r = s.result
            S.section(FA.ICON_FA_FLAG_CHECKERED, f'{name} — 종료', S.OK if r['ok'] else S.DANGER)
            S.chip('PASS' if r['ok'] else 'FAIL', S.OK if r['ok'] else S.DANGER, filled=True)
            imgui.same_line()
            if imgui.button(f'{FA.ICON_FA_FILE_LINES}  결과'):
                self.result_open = True
            imgui.same_line()
            if S.primary_button(f'{FA.ICON_FA_ROTATE_RIGHT}  다시 시험'):
                self.restart()
        else:
            S.section(FA.ICON_FA_CLIPBOARD_CHECK, f'시험 중 · {name}', S.WARN)
            t = snap['t'] if snap else 0.0
            if s.duration:
                imgui.progress_bar(min(t / s.duration, 1.0), imgui.ImVec2(-1, 6 * sc), '')
                S.muted(f'{t:.0f} / {s.duration:.0f} s')
            else:
                S.muted(f'{t:.0f} s (제한 없음)')
            imgui.same_line()
            n_iv = len(s.interventions) + (1 if s._iv else 0)
            S.chip(f'수동 개입 {n_iv}', S.WARN if n_iv else S.MUTED, FA.ICON_FA_HAND)
            if imgui.button(f'{FA.ICON_FA_STOP}  시험 종료', imgui.ImVec2(-1, 0)):
                self._finish_test('사용자 종료')
                self.paused = True
                if self.eth is not None:
                    self.eth.paused = True
        S.end_card()

    def _change_banner(self):
        ch = self._changes()
        if not ch:
            return
        test = self.mode == 'test'
        S.begin_card('pending')
        if test:
            S.section(FA.ICON_FA_LOCK, f'시험 조건 잠김 · 변경 {len(ch)}개', S.WARN)
            S.muted('지금 시험에는 반영되지 않습니다')
        else:
            S.section(FA.ICON_FA_PEN, f'변경 사항 {len(ch)}개', S.ACCENT)
        labels = [f'{GROUP_KO[g]} {KEY_KO.get(k, k)}' for g, k in ch]
        imgui.push_text_wrap_pos(0)
        S.muted(', '.join(labels[:5]) + (f' 외 {len(labels) - 5}개' if len(labels) > 5 else ''))
        imgui.pop_text_wrap_pos()
        w = imgui.get_content_region_avail().x
        if S.primary_button(f'{FA.ICON_FA_ROTATE_RIGHT}  ' + ('이 조건으로 재시작' if test else '적용 (재시작)'),
                            imgui.ImVec2(w * 0.64, 0), S.WARN if test else S.ACCENT):
            self.restart()
        imgui.same_line()
        if imgui.button(f'{FA.ICON_FA_ROTATE_LEFT}  되돌리기', imgui.ImVec2(-1, 0)):
            self._revert()
        S.end_card()

    def _result_card(self, r, cid):
        S.begin_card(cid)
        S.section(FA.ICON_FA_FLAG_CHECKERED, f'최근 시험 — {r["scenario"]["name"]}', S.OK if r['ok'] else S.DANGER)
        S.chip('PASS' if r['ok'] else 'FAIL', S.OK if r['ok'] else S.DANGER, filled=True)
        if not r['complete']:
            imgui.same_line()
            S.chip('시험 시간 전 종료', S.WARN)
        imgui.same_line()
        S.muted(r['reason'])
        if imgui.button(f'{FA.ICON_FA_FOLDER_OPEN}  결과 폴더'):
            self._open_path(r['dir'])
        imgui.same_line()
        if imgui.button(f'{FA.ICON_FA_FILE_LINES}  자세히'):
            self.result_open = True
        S.end_card()

    def _result_modal(self):
        if self.result_open and self.test_result is not None:
            imgui.open_popup('시험 결과')
            self.result_open = False
        imgui.set_next_window_size(imgui.ImVec2(620 * self.scale, 0))
        if not imgui.begin_popup_modal('시험 결과', None, imgui.WindowFlags_.no_resize)[0]:
            return
        r = self.test_result
        m = r['metrics']
        imgui.push_font(S.Fonts.bold, 24)
        imgui.text_colored(S.v4(S.OK if r['ok'] else S.DANGER),
                           f'{FA.ICON_FA_CIRCLE_CHECK if r["ok"] else FA.ICON_FA_CIRCLE_XMARK}  '
                           f'{"PASS" if r["ok"] else "FAIL"}')
        imgui.pop_font()
        imgui.same_line()
        imgui.set_cursor_pos_y(imgui.get_cursor_pos_y() + 8 * self.scale)
        S.muted(f'{r["scenario"]["name"]} · {r["reason"]} · {m["sim_time_s"]} s'
                + ('' if r['complete'] else ' · 시험 시간 전 종료 (참고용)'))
        imgui.spacing()
        if imgui.begin_table('crit', 4, imgui.TableFlags_.row_bg | imgui.TableFlags_.borders_inner_h):
            for h in ('기준', '기대', '실제', '판정'):
                imgui.table_setup_column(h)
            imgui.table_headers_row()
            for k, want, got, st in r['rows']:
                imgui.table_next_row()
                imgui.table_next_column()
                imgui.text(CRITERIA_KO.get(k, k))
                imgui.table_next_column()
                imgui.text(str(want))
                imgui.table_next_column()
                imgui.text('-' if got is None else str(got))
                imgui.table_next_column()
                col = {'PASS': S.OK, 'FAIL': S.DANGER}.get(st, S.MUTED)
                imgui.text_colored(S.v4(col), st)
            imgui.end_table()
        imgui.spacing()
        S.muted(f'추돌 {m["crashes"]} · 비콘 추돌 {m["beacon_hits"]} · 최소 TTC {m["min_ttc_s"]} s · 인지 {m["noticed"]} · '
                f'SCC 인식 {m["scc_locks"]} · GNSS FIX {m["gnss_fix_ratio"]:.0%} · 수동 개입 {m["interventions"]}회')
        S.muted(f'저장: {r["dir"]}')
        imgui.spacing()
        if imgui.button(f'{FA.ICON_FA_FOLDER_OPEN}  결과 폴더'):
            self._open_path(r['dir'])
        imgui.same_line()
        if imgui.button(f'{FA.ICON_FA_LIST_CHECK}  회귀 시나리오로 저장'):
            try:
                p = save_regression(r['scenario'])
                self.log(f'회귀 시나리오 저장: {p}')
                if m['interventions']:
                    self.log('주의: 수동 개입(키보드)은 회귀 실행에서 재현되지 않습니다 (/cmd_vel 상위 제어기 필요)')
            except Exception as e:      # noqa: BLE001
                self.log(f'저장 실패: {e}')
        S.tip('같은 조건(시드 고정)을 scenarios/ 에 저장 → 회귀 모드에서 반복 실행')
        imgui.same_line()
        if S.primary_button(f'{FA.ICON_FA_ROTATE_RIGHT}  다시 시험'):
            imgui.close_current_popup()
            self.restart()
        imgui.same_line()
        if imgui.button('닫기'):
            imgui.close_current_popup()
        imgui.end_popup()

    def _open_path(self, p):
        try:
            subprocess.Popen(['xdg-open', p], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:          # noqa: BLE001
            self.log(f'열기 실패: {e} ({p})')

    # ------------------------------------------------------------ 회귀 모드
    def _regress_list(self):
        files = list_scenarios()
        for f in files:
            self.regress_sel.setdefault(f, True)
        S.begin_card('rg_list')
        S.section(FA.ICON_FA_LIST_CHECK, f'시나리오 ({sum(self.regress_sel.get(f, False) for f in files)}/{len(files)})')
        if imgui.button('전체 선택'):
            for f in files:
                self.regress_sel[f] = True
        imgui.same_line()
        if imgui.button('전체 해제'):
            for f in files:
                self.regress_sel[f] = False
        imgui.same_line()
        if imgui.button(f'{FA.ICON_FA_FOLDER_OPEN}  폴더'):
            self._open_path(scenario_dirs()[-1])
        S.end_card()
        for f in files:
            info = self._scenario_info(f)
            imgui.push_id(f)
            _, self.regress_sel[f] = imgui.checkbox('##on', self.regress_sel[f])
            imgui.same_line()
            imgui.text(info.get('name', os.path.basename(f)))
            imgui.same_line()
            S.muted(f'{info.get("duration", "-")} s')
            if not f.startswith(os.path.join(os.path.dirname(os.path.dirname(__file__)), 'scenarios')):
                imgui.same_line()
                S.chip('사용자', S.TEAL)
            imgui.set_cursor_pos_x(imgui.get_cursor_pos_x() + 30 * self.scale)
            imgui.push_text_wrap_pos(0)
            S.muted(info.get('description', ''))
            imgui.pop_text_wrap_pos()
            imgui.pop_id()

    def _scenario_info(self, f):
        cache = self.__dict__.setdefault('_sc_info', {})
        mt = os.path.getmtime(f)
        if f not in cache or cache[f][0] != mt:
            try:
                with open(f) as fh:
                    d = yaml.safe_load(fh) or {}
            except Exception:           # noqa: BLE001
                d = {}
            cache[f] = (mt, {k: d.get(k) for k in ('name', 'description', 'duration')})
        return cache[f][1]

    def _regress_panel(self):
        sc = self.scale
        rr = self.regress
        S.begin_card('rg_opt')
        S.section(FA.ICON_FA_SLIDERS, '실행 방식')
        imgui.begin_disabled(rr is not None and rr.running)
        ros = self.ros is not None and self.ros.ok
        imgui.begin_disabled(not ros)
        _, self.regress_ros = imgui.checkbox('ROS 연동 (실시간, /cmd_vel · 평가 토픽)', self.regress_ros and ros)
        imgui.end_disabled()
        S.muted('끄면 단독 고속 실행 (비콘은 시나리오의 beacon_script)')
        imgui.end_disabled()
        S.end_card()
        sel = [f for f in list_scenarios() if self.regress_sel.get(f)]
        if rr is None or not rr.running:
            imgui.begin_disabled(not sel)
            if S.primary_button(f'{FA.ICON_FA_PLAY}  {len(sel)}개 실행', imgui.ImVec2(-1, 48 * sc)):
                if self.eth is not None:          # GUI 엔진 정지 (GPU·토픽 충돌 방지)
                    self.paused = True
                    self.eth.paused = True
                self.mode_sel = 'regress'
                self.regress = RegressRun(sel, ros=self.regress_ros, log=self.log)
                self.log(f'회귀 테스트 시작: {len(sel)}개 → {self.regress.out}')
            imgui.end_disabled()
        else:
            if imgui.button(f'{FA.ICON_FA_STOP}  중지', imgui.ImVec2(-1, 40 * sc)):
                rr.stop()
        if rr is None:
            return
        done = sum(r[1] in ('PASS', 'FAIL') for r in rr.rows)
        imgui.spacing()
        S.begin_card('rg_prog')
        if rr.running:
            S.section(FA.ICON_FA_SPINNER, f'실행 중 {done}/{len(rr.files)} · {time.time() - rr.t0:.0f} s', S.WARN)
        else:
            npass = sum(r[1] == 'PASS' for r in rr.rows)
            ok = rr.code == 0
            S.section(FA.ICON_FA_FLAG_CHECKERED, f'완료 · 통과 {npass}/{len(rr.rows)}', S.OK if ok else S.DANGER)
        imgui.progress_bar(done / max(len(rr.files), 1), imgui.ImVec2(-1, 6 * sc), '')
        for name, st, info in rr.rows:
            col = {'PASS': S.OK, 'FAIL': S.DANGER}.get(st, S.WARN)
            S.chip(st, col, filled=st != 'RUN')
            imgui.same_line()
            imgui.text(name)
            if info:
                imgui.push_text_wrap_pos(0)
                imgui.push_font(S.Fonts.body, S.Fonts.SMALL)
                S.muted(info)
                imgui.pop_font()
                imgui.pop_text_wrap_pos()
        if not rr.running:
            if rr.report:
                if imgui.button(f'{FA.ICON_FA_FILE_LINES}  report.md'):
                    self._open_path(rr.report)
                imgui.same_line()
            if imgui.button(f'{FA.ICON_FA_FOLDER_OPEN}  결과 폴더'):
                self._open_path(rr.out)
            errs = [ln for ln in rr.lines if 'Error' in ln or 'Traceback' in ln]
            if errs:
                S.muted('오류: ' + errs[-1][:200])
        S.end_card()
