"""BeaconSim 통합 시뮬레이터 GUI (Dear ImGui + moderngl + GLFW).

- 맵: 시드/파라미터 → 고속도로 생성, Lanelet2 OSM/PCD 저장
- 시나리오: 사고 현장 + 역방향 비콘. 비콘 조작은 ROS /cmd_vel(상위 제어기) 또는 키보드(방향키)
- 교통: 접근 차량 반응(차로변경/감속/추돌) + SCC 추종, 기능 검증 모드(차량 없음)
- 환경: 낮/황혼/밤, 맑음/비/안개/눈 (랜덤 또는 지정)
- 센서 시각화: Pandar40P, Tele-15, ARS408, GNSS(ZED-F9P), 카메라 창
- ROS2가 설치돼 있으면 같은 토픽으로 발행 (RViz 불필요)

조작: 3D 뷰 좌클릭 드래그 회전 / 우클릭 드래그 이동 / 휠 확대
      방향키 = 비콘 주행(↑ 전진 ↓ 후진 ←→ 회전), Space 일시정지, F 비콘 따라가기
      G 맵 생성, R 시나리오 재시작(랜덤), 1 비콘 시점 2 전체 3 터널
"""
import argparse
import array
import collections
import copy
import json
import os
import re
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import fields

import glfw
import moderngl
import numpy as np
from imgui_bundle import imgui

from . import scene
from .common import (ROOT, T_CAMLINK_OPTICAL, has_robot_model, load_robot_parts, load_yaml, resource,
                     sensor_mounts, writable_root)
from .engine import EngineThread, SimEngine
from .environment import TIME_KO, WEATHER_KO, EnvParams
from .export import export_all
from .gnss import FIX, FLOAT, NONE, SINGLE
from . import ui_style as S
from .imgui_mgl import GlfwInput, ImguiRenderer
from .ui_style import FA
from .render import CameraRenderer, OverlayRenderer, SceneRenderer
from .roadgen import GENERATOR_VERSION, RoadParams, generate
from .traffic import B_CRASH, ScenarioParams, TrafficParams

CONFIG_DIR = os.path.join(os.path.expanduser('~'), '.config', 'beaconsim')
FONT_CANDIDATES = [
    os.path.join(ROOT, 'assets', 'fonts', 'NanumGothic.ttf'),
    os.path.join(ROOT, 'assets', 'fonts', 'NotoSansKR-Regular.ttf'),
    '/usr/share/fonts/truetype/nanum/NanumGothic.ttf',
    '/usr/share/fonts/truetype/unfonts-core/UnDotum.ttf',
    ('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', 1),
]
LINE_COLORS = {
    'yellow_solid': (1.0, 0.82, 0.10), 'white_solid': (1.0, 1.0, 1.0),
    'white_dashed': (0.55, 0.85, 1.0), 'road_border': (1.0, 0.30, 0.30), 'center': (0.2, 1.0, 0.5),
}
GNSS_COLOR = {FIX: (0.2, 1.0, 0.3, 1), FLOAT: (1.0, 0.85, 0.1, 1), SINGLE: (1.0, 0.45, 0.1, 1), NONE: (1.0, 0.1, 0.1, 1)}
LABEL_COLORS = np.array([[0.5, 0.5, 0.5], [0.35, 0.35, 0.38], [1, 1, 1], [1, 0.8, 0.1], [0.7, 0.75, 0.8],
                         [0.6, 0.6, 0.55], [0.3, 0.6, 0.25], [0.6, 0.5, 0.4], [0.2, 0.6, 1.0], [1, 0.5, 0],
                         [0.1, 0.8, 0.4], [0.7, 0.7, 0.7], [1.0, 0.2, 0.2]], np.float32)
SCC_ON, SCC_BEACON, NOTICED, CRASHED = (0.169, 0.831, 0.773, 1), (0.780, 0.490, 1.0, 1), (1.0, 0.722, 0.302, 1), (1.0, 0.353, 0.373, 1)


def perspective(fovy_deg, aspect, near, far):
    f = 1 / np.tan(np.radians(fovy_deg) / 2)
    return np.array([[f / aspect, 0, 0, 0], [0, f, 0, 0],
                     [0, 0, (far + near) / (near - far), 2 * far * near / (near - far)], [0, 0, -1, 0]])


def look_at(eye, target, up=(0, 0, 1)):
    eye, target, up = map(lambda a: np.asarray(a, float), (eye, target, up))
    f = target - eye
    f /= np.linalg.norm(f)
    s = np.cross(f, up)
    if np.linalg.norm(s) < 1e-6:
        s = np.cross(f, [0, 1, 0])
    s /= np.linalg.norm(s)
    u = np.cross(s, f)
    V = np.eye(4)
    V[0, :3], V[1, :3], V[2, :3] = s, u, -f
    V[:3, 3] = -V[:3, :3] @ eye
    return V


def lanelet_overlay(m):
    """Lanelet 경계선 + 차로 중심선 → 선분 정점 (pos3, col4)."""
    s = np.r_[np.arange(0, m.length, 2.0), m.length]
    P, C = [], []
    for side in (-1, 1):
        for ln in m.lines():
            Q = m.point(s, side * ln.offset, 0.05)
            seg = np.stack([Q[:-1], Q[1:]], 1).reshape(-1, 3)
            P.append(seg)
            C.append(np.tile((*LINE_COLORS[ln.kind], 1.0), (len(seg), 1)))
        for i in range(m.params.lanes):
            Q = m.point(s, m.lane_center_t(i, side), 0.05)
            keep = (np.arange(len(Q) - 1) // 3) % 2 == 0
            seg = np.stack([Q[:-1][keep], Q[1:][keep]], 1).reshape(-1, 3)
            P.append(seg)
            C.append(np.tile((*LINE_COLORS['center'], 1.0), (len(seg), 1)))
    return np.concatenate(P).astype('f4'), np.concatenate(C).astype('f4')


def apply_to_sim_config(blocks, path=None):
    """config/sim.yaml 의 블록(map/scenario/traffic/environment)을 교체 (주석·다른 설정 유지).
    저장소에서 실행하면 저장소 파일, AppImage면 ~/.config/beaconsim/config/sim.yaml."""
    path = path or os.path.join(writable_root(), 'config', 'sim.yaml')
    src = path if os.path.exists(path) else resource(os.path.join('config', 'sim.yaml'))
    with open(src) as f:
        text = f.read()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    for name, vals in blocks.items():
        block = f'{name}:\n' + ''.join(f'  {k}: {v}\n' for k, v in vals.items())
        text, n = re.subn(rf'^{name}:\n(?:[ \t]+.*\n)*', block, text, count=1, flags=re.M)
        if n == 0:
            text += '\n' + block
    with open(path, 'w') as f:
        f.write(text)
    return path


def intensity_colors(x, vmax=60.0):
    """세기 → 색 (어두운 파랑 → 청록 → 노랑)."""
    t = np.clip(np.asarray(x, np.float32) / vmax, 0, 1)
    r = np.clip(2 * t - 0.6, 0, 1)
    g = np.clip(1.6 * t, 0, 1)
    b = np.clip(1.0 - 1.2 * t, 0.15, 1)
    return np.stack([r, g, b, np.ones_like(t)], 1)


class RosLink:
    """ROS2가 있으면 RosBridge + 카메라 발행. 없으면 비활성."""

    def __init__(self, log):
        self.ok, self.bridge, self.node = False, None, None
        self.log = log
        try:
            import rclpy
            from rclpy.node import Node
            self.rclpy = rclpy
            rclpy.init()
            self.node = Node('beacon_sim_app')
            self.ok = True
        except Exception as e:        # noqa: BLE001
            log(f'ROS2 연동 없음 (단독 실행): {e}')

    def attach(self, engine, cfg):
        if not self.ok:
            return None
        from .ros_bridge import RosBridge
        if self.bridge is None:
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import CameraInfo, Image
            self.bridge = RosBridge(self.node, engine, cfg, markers=False)
            qs = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.pub_img = self.node.create_publisher(Image, cfg['camera']['image_topic'], qs)
            self.pub_info = self.node.create_publisher(CameraInfo, cfg['camera']['info_topic'], qs)
            self.Image, self.CameraInfo = Image, CameraInfo
            threading.Thread(target=self._spin, daemon=True).start()
            self.log('ROS2 연동: 토픽 발행 중, /cmd_vel 수신 대기')
        else:
            self.bridge.e = engine
            self.bridge.publish_static()
        return self.bridge.on_step

    def _spin(self):
        try:
            self.rclpy.spin(self.node)
        except Exception:             # noqa: BLE001
            pass

    def publish_camera(self, img, t, cam, frame):
        if not self.ok or self.bridge is None:
            return
        from .ros_bridge import t_to_msg
        st = t_to_msg(t)
        msg = self.Image()
        msg.header.stamp, msg.header.frame_id = st, frame
        msg.height, msg.width, msg.encoding, msg.is_bigendian = img.shape[0], img.shape[1], 'rgb8', 0
        msg.step = img.shape[1] * 3
        msg.data = array.array('B', img.tobytes())
        self.pub_img.publish(msg)
        info = self.CameraInfo()
        info.header = msg.header
        info.height, info.width = img.shape[0], img.shape[1]
        info.distortion_model = 'plumb_bob'
        info.d = [float(x) for x in cam.D]
        info.k = [float(x) for x in cam.K.ravel()]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [float(cam.K[0, 0]), 0.0, float(cam.K[0, 2]), 0.0, 0.0, float(cam.K[1, 1]), float(cam.K[1, 2]),
                  0.0, 0.0, 0.0, 1.0, 0.0]
        self.pub_info.publish(info)

    def shutdown(self):
        if self.ok:
            try:
                self.node.destroy_node()
                self.rclpy.shutdown()
            except Exception:         # noqa: BLE001
                pass


class App:
    def __init__(self, width=1700, height=1000, hidden=False, ros=True):
        if not glfw.init():
            raise RuntimeError('GLFW 초기화 실패 (디스플레이가 없나요?)')
        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
        glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
        glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, True)
        glfw.window_hint(glfw.SAMPLES, 4)
        glfw.window_hint(glfw.VISIBLE, not hidden)
        self.win = glfw.create_window(width, height, f'BeaconSim 세이프티 비콘 시뮬레이터 v{GENERATOR_VERSION}', None, None)
        glfw.make_context_current(self.win)
        glfw.swap_interval(1)
        self.ctx = moderngl.create_context()
        self.ctx.enable(moderngl.PROGRAM_POINT_SIZE)

        imgui.create_context()
        io = imgui.get_io()
        os.makedirs(CONFIG_DIR, exist_ok=True)
        io.set_ini_filename(os.path.join(CONFIG_DIR, 'imgui.ini'))
        self.scale = max(glfw.get_window_content_scale(self.win)[0], 1.0)
        self.font_path = S.load_fonts(self.scale)
        S.apply_theme(self.scale)
        self.gui = ImguiRenderer(self.ctx)
        self.input = GlfwInput(self.win, self._on_drag, self._on_scroll, self._on_key)
        self.ov = OverlayRenderer(self.ctx)

        self.log_lines = collections.deque(maxlen=400)
        self.log_lock = threading.Lock()
        self.pool = ThreadPoolExecutor(max_workers=2)
        self.build_future = None
        self.export_future = None

        self.base_cfg = load_yaml('config/sim.yaml')
        self.robot_cfg = None
        st_ = self._load_settings()
        self.params, self.sparams, self.tparams, self.eparams = st_['params'], st_['scenario'], st_['traffic'], st_['env']
        self.export_dir = st_['export_dir']
        self.rand_map, self.rand_scn = st_.get('rand_map', False), st_.get('rand_scn', True)
        self.export_pcd = True
        self.show = dict(terrain=True, tunnel=True, lanelet=False, vehicles=True, robot=True, markers=True,
                         pandar=True, tele=True, radar=True, gnss=True, scc=True, camera=True, env_view=True)
        self.pandar_color = 0              # 0 세기, 1 정답 라벨
        self.show.update(labels=True, radar_fov=False, legend=True, auto_roof=True)
        self.log_filter = 0                # 0 전체, 1 이벤트만
        self.help_open = False
        self.tab = 0
        self.rtf = 1.0
        self.paused = False
        self.follow = True
        self.key_speed, self.key_yaw = 1.5, 0.6
        self.cam = {'target': np.zeros(3), 'dist': 30.0, 'yaw': 0.0, 'pitch': np.radians(20)}
        self.gnss_trail = collections.deque(maxlen=600)
        self.truth_trail = collections.deque(maxlen=600)
        self.last_gnss_t = -1.0
        self.cam_next_t = 0.0
        self.cam_tex = None
        self.target_fbo = None

        self.model = self.parts = self.engine = self.eth = None
        self.renderer = None
        self.robot_parts = None
        self.robot_future = None
        self.camera = CameraRenderer(self.ctx, self.base_cfg['camera'])
        self.ros = RosLink(self.log) if ros else None
        self.log(f'폰트: {self.font_path}')
        if has_robot_model():
            self._load_robot()
            self.rebuild(map_too=True, sync=True)
        else:
            self.log('로봇 모델이 없습니다. 비콘 CAD(STL)를 선택하세요.')

    # ------------------------------------------------------------ 로봇 모델 (첫 실행)
    def _load_robot(self):
        self.robot_cfg = load_yaml('config/robot.yaml')
        self.robot_parts = load_robot_parts()
        _, T_bc = sensor_mounts(self.robot_cfg)['camera']
        self.T_b_opt = T_bc @ T_CAMLINK_OPTICAL

    def build_robot_async(self, stl_path):
        from .robot_build import build
        self.log(f'로봇 모델 생성 중: {stl_path}')
        self.robot_future = self.pool.submit(build, stl_path, writable_root(), True, self.log)

    def _robot_setup_modal(self, W, H):
        imgui.set_next_window_pos(imgui.ImVec2(W / 2, H / 2), imgui.Cond_.always, imgui.ImVec2(0.5, 0.5))
        imgui.set_next_window_size(imgui.ImVec2(560 * self.scale, 0))
        imgui.begin('robot_setup', None, imgui.WindowFlags_.no_decoration | imgui.WindowFlags_.no_move)
        S.section(FA.ICON_FA_TOWER_BROADCAST, '비콘 로봇 모델이 필요합니다', S.WARN)
        imgui.text_wrapped('비콘 CAD(STL, 단위 mm)를 선택하면 로봇 메시와 센서 장착 포즈를 만들어 '
                           f'{writable_root()} 에 저장합니다. CAD는 공개 배포하지 않으므로 팀 내부에서 받으세요.')
        imgui.spacing()
        busy = self.robot_future is not None and not self.robot_future.done()
        imgui.begin_disabled(busy)
        if S.primary_button(f'{FA.ICON_FA_FOLDER_OPEN}  ' + ('생성 중...' if busy else 'STL 파일 선택'),
                            imgui.ImVec2(-1, 0)):
            try:
                from imgui_bundle import portable_file_dialogs as pfd
                r = pfd.open_file('비콘 CAD (STL) 선택', os.path.expanduser('~'), ['STL', '*.stl *.STL']).result()
                if r:
                    self.build_robot_async(r[0])
            except Exception as e:      # noqa: BLE001
                self.log(f'파일 선택 창을 열 수 없음: {e} — 명령행: BeaconSim.AppImage --robot-stl <경로>')
        imgui.end_disabled()
        with self.log_lock:
            tail = list(self.log_lines)[-6:]
        for ln in tail:
            S.muted(ln)
        imgui.end()

    # ------------------------------------------------------------ 설정/로그
    def log(self, msg):
        with self.log_lock:
            self.log_lines.append(time.strftime('%H:%M:%S ') + str(msg))

    def _load_settings(self):
        p = os.path.join(CONFIG_DIR, 'app.json')
        c = self.base_cfg

        def pick(cls, d):
            names = {fl.name for fl in fields(cls)}
            return cls(**{k: v for k, v in d.items() if k in names})
        out = {'params': pick(RoadParams, c.get('map', {})), 'scenario': pick(ScenarioParams, c.get('scenario', {})),
               'traffic': pick(TrafficParams, c.get('traffic', {})), 'env': pick(EnvParams, c.get('environment', {})),
               'export_dir': os.path.join(os.path.expanduser('~'), 'BeaconSim_maps')}
        if out['params'].seed < 0:
            out['params'].seed = 1
        try:
            with open(p) as f:
                d = json.load(f)
            for key, cls in [('params', RoadParams), ('scenario', ScenarioParams), ('traffic', TrafficParams),
                             ('env', EnvParams)]:
                if key in d:
                    out[key] = pick(cls, d[key])
            for k in ('export_dir', 'rand_map', 'rand_scn'):
                if k in d:
                    out[k] = d[k]
        except Exception:           # noqa: BLE001
            pass
        return out

    def _save_settings(self):
        with open(os.path.join(CONFIG_DIR, 'app.json'), 'w') as f:
            json.dump({'params': self.params.to_dict(), 'scenario': self.sparams.to_dict(),
                       'traffic': self.tparams.to_dict(), 'env': self.eparams.to_dict(),
                       'export_dir': self.export_dir, 'rand_map': self.rand_map, 'rand_scn': self.rand_scn},
                      f, indent=1)

    # ------------------------------------------------------------ 빌드 (맵/시나리오/엔진)
    def _make_cfg(self):
        cfg = copy.deepcopy(self.base_cfg)
        cfg['map'] = self.params.to_dict()
        cfg['scenario'] = self.sparams.to_dict()
        cfg['traffic'] = self.tparams.to_dict()
        cfg['environment'] = self.eparams.to_dict()
        return cfg

    def _build_job(self, cfg, map_too, model, parts):
        overlay = None
        if map_too:
            model = generate(RoadParams(**cfg['map']))
            parts = scene.build_static(model)
            overlay = lanelet_overlay(model)
        eng = SimEngine(cfg, self.robot_cfg, model=model, parts=parts, log=self.log)
        return model, parts, overlay, eng

    def rebuild(self, map_too=False, sync=False):
        if self.build_future is not None and not self.build_future.done():
            return
        rng = np.random.default_rng()
        if map_too and self.rand_map:
            self.params.seed = int(rng.integers(0, 2 ** 31 - 1))
        if self.rand_scn:
            self.sparams.seed = int(rng.integers(0, 2 ** 31 - 1))
        self._save_settings()
        if self.eth is not None:
            self.eth.stop()
            self.eth = None
        cfg = self._make_cfg()
        self.log(('맵·' if map_too else '') + f'시나리오 생성 중... (맵 시드 {self.params.seed}, 시나리오 시드 {self.sparams.seed})')
        args = (cfg, map_too or self.model is None, self.model, self.parts)
        if sync:
            self._apply(*self._build_job(*args), cfg=cfg)
        else:
            self.build_cfg = cfg
            self.build_future = self.pool.submit(self._build_job, *args)

    def _apply(self, model, parts, overlay, eng, cfg):
        old = self.engine
        if old is not None and old is not eng:
            old.close()
        map_changed = model is not self.model
        self.model, self.parts, self.engine = model, parts, eng
        if self.renderer is None:
            self.renderer = SceneRenderer(self.ctx, model, self.robot_parts, exclude_robot=(), fog_dist=4000.0,
                                          parts=parts)
            self.cam_scene = SceneRenderer(self.ctx, model, self.robot_parts, exclude_robot=('camera',), parts=parts)
        elif map_changed:
            self.renderer.set_model(model, parts)
            self.cam_scene.set_model(model, parts)
        self.renderer.set_env(eng.env if self.show['env_view'] else None)
        self.cam_scene.set_env(eng.env)
        if overlay is not None:
            self.lane_pts, self.lane_cols = overlay
        scn = eng.scn
        mk = []
        for s_, col in [(scn.accident_s, (1.0, 0.5, 0.0, 1))]:
            p0 = model.point(s_, scn.lane_t)
            mk += [(p0, col), (p0 + [0, 0, 6.0], col)]
        self.marker_pts = np.array([p for p, _ in mk], 'f4')
        self.marker_cols = np.array([c for _, c in mk], 'f4')
        self.gnss_trail.clear()
        self.truth_trail.clear()
        self.cam_next_t = 0.0
        on_step = self.ros.attach(eng, cfg) if self.ros is not None else None
        self.eth = EngineThread(eng, self.rtf, on_step)
        self.eth.paused = self.paused
        self.eth.start()
        sm, ms = scn.summary(), model.summary()
        self.log(f'맵 시드 {ms["seed"]} ({ms["length_m"]:.0f} m, {ms["lanes"]}차로, 터널 {len(ms["tunnels"])}, '
                 f'차선 불량 {ms["lane_wear_pct"]}%) | 시나리오 시드 {scn.sp.seed}: {sm["side"]} {sm["lane"]}차로, '
                 f'사고 s={sm["accident_s"]:.0f} m | 환경 {eng.env.label()} | '
                 f'교통 {"켜짐" if eng.traffic.tp.enabled else "꺼짐(검증 모드)"}')
        if 'tele15' in eng.sensors:
            self.log(f'Tele-15 스캔 패턴: {eng.sensors["tele15"].pattern}')
        self._preset('beacon')

    def export_async(self):
        if self.export_future and not self.export_future.done():
            return
        p = self.model.params
        out = os.path.join(self.export_dir, f'hwy_s{p.seed}_{int(p.length)}m_{p.lanes}L_{p.design_speed}kph')
        self._save_settings()
        self.log(f'저장 시작 → {out}')
        self.export_future = self.pool.submit(self._export_job, out)

    def _export_job(self, out):
        import yaml
        meta = export_all(self.model, out, self.log, self.export_pcd, self.parts)
        with open(os.path.join(out, 'scenario.yaml'), 'w') as f:
            yaml.safe_dump({'map': self.model.params.to_dict(), 'scenario': self.engine.scn.sp.to_dict(),
                            'traffic': self.tparams.to_dict(), 'environment': self.eparams.to_dict(),
                            'resolved_environment': self.engine.env.to_dict(),
                            'scenario_summary': self.engine.scn.summary()}, f, allow_unicode=True, sort_keys=False)
        self.log('scenario.yaml 저장 (맵·시나리오·교통·환경)')
        return meta

    # ------------------------------------------------------------ 카메라 조작
    def _on_drag(self, dx, dy, buttons):
        c = self.cam
        if buttons[0]:
            c['yaw'] -= dx * 0.005
            c['pitch'] = float(np.clip(c['pitch'] + dy * 0.005, np.radians(2), np.radians(89)))
        elif buttons[1] or buttons[2]:
            self.follow = False
            k = c['dist'] * 0.0015
            fwd = np.array([np.cos(c['yaw']), np.sin(c['yaw']), 0.0])
            right = np.array([np.sin(c['yaw']), -np.cos(c['yaw']), 0.0])
            c['target'] = c['target'] + (right * dx - fwd * dy) * k
            s, t = self.model.project(c['target'][0], c['target'][1])
            c['target'][2] = float(self.model.surface_z(s, np.clip(t, -self.model.paved_t, self.model.paved_t)))

    def _on_scroll(self, dy):
        self.cam['dist'] = float(np.clip(self.cam['dist'] * (0.88 ** dy), 2.0, 8000.0))

    def _on_key(self, key):
        acts = {glfw.KEY_G: lambda: self.rebuild(map_too=True), glfw.KEY_R: lambda: self.rebuild(),
                glfw.KEY_SPACE: self._toggle_pause, glfw.KEY_F: lambda: setattr(self, 'follow', not self.follow),
                glfw.KEY_1: lambda: self._preset('beacon'), glfw.KEY_2: lambda: self._preset('overview'),
                glfw.KEY_3: lambda: self._preset('tunnel'),
                glfw.KEY_F1: lambda: setattr(self, 'help_open', True)}
        if key in acts:
            acts[key]()

    def _toggle_pause(self):
        self.paused = not self.paused
        if self.eth is not None:
            self.eth.paused = self.paused

    def _preset(self, name):
        m, c = self.model, self.cam
        if name == 'beacon':
            self.follow = True
            T = self.engine.T_wb
            c['target'] = T[:3, 3] + np.array([0, 0, 0.5])
            c['dist'], c['pitch'] = 22.0, np.radians(18)
            c['yaw'] = float(np.arctan2(-T[1, 0], -T[0, 0]) + 0.35)
        elif name == 'overview':
            self.follow = False
            P = np.c_[m.x, m.y, m.z]
            c['target'] = (P.min(0) + P.max(0)) / 2
            c['dist'], c['pitch'], c['yaw'] = float(np.ptp(P[:, :2], axis=0).max() * 0.9), np.radians(70), np.radians(-90)
        elif name == 'tunnel':
            if not m.tunnels:
                self.log('터널 없음')
                return
            self.follow = False
            x, y, z, th = m.frame(m.tunnels[0][0] - 60)
            c['target'] = np.array([x, y, z])
            c['dist'], c['pitch'], c['yaw'] = 70.0, np.radians(20), float(th + np.pi + 0.5)

    def _keyboard_drive(self):
        if imgui.get_io().want_capture_keyboard or self.engine is None:
            return

        def k(key):
            return glfw.get_key(self.win, key) == glfw.PRESS
        v = (self.key_speed if k(glfw.KEY_UP) else 0.0) - (self.key_speed if k(glfw.KEY_DOWN) else 0.0)
        w = (self.key_yaw if k(glfw.KEY_LEFT) else 0.0) - (self.key_yaw if k(glfw.KEY_RIGHT) else 0.0)
        if v or w:
            self.engine.set_cmd('key', v, w)

    # ------------------------------------------------------------ UI
    # ------------------------------------------------------------ UI (디자인: ui_style.py)
    def _ui(self, snap):
        io = imgui.get_io()
        W, H = io.display_size
        self.sidebar_w = pw = 380 * self.scale
        self._sidebar(snap, pw, H)
        self._playbar(snap, pw, W)
        self._dashboard(snap, W)
        if self.show['camera'] and self.cam_tex is not None:
            self._camera_window(W, H)
        self._help_popup()

    # ---- 왼쪽 사이드바
    def _sidebar(self, snap, pw, H):
        imgui.set_next_window_pos(imgui.ImVec2(0, 0))
        imgui.set_next_window_size(imgui.ImVec2(pw, H))
        imgui.begin('sidebar', None, imgui.WindowFlags_.no_decoration | imgui.WindowFlags_.no_move |
                    imgui.WindowFlags_.no_bring_to_front_on_focus)
        # 헤더
        imgui.push_font(S.Fonts.bold, 22 * self.scale / max(self.scale, 1))
        imgui.text_colored(S.v4(S.ACCENT), FA.ICON_FA_TOWER_BROADCAST)
        imgui.same_line()
        imgui.text('BeaconSim')
        imgui.pop_font()
        imgui.same_line()
        imgui.set_cursor_pos_y(imgui.get_cursor_pos_y() + 6)
        S.muted(f'v{GENERATOR_VERSION}  ·  세이프티 비콘 시뮬레이터')
        imgui.spacing()

        tabs = [(FA.ICON_FA_ROAD, '맵', self._tab_map), (FA.ICON_FA_CAR_BURST, '시나리오', lambda: self._tab_scenario(snap)),
                (FA.ICON_FA_CAR_SIDE, '교통', self._tab_traffic), (FA.ICON_FA_CLOUD_SUN_RAIN, '환경', self._tab_env),
                (FA.ICON_FA_EYE, '보기', self._tab_view), (FA.ICON_FA_FLOPPY_DISK, '저장', self._tab_export)]
        # 아이콘 탭 (2줄 x 3)
        bw = (pw - 28 * self.scale - 2 * 6) / 3
        for k, (ic, label, _) in enumerate(tabs):
            if k % 3:
                imgui.same_line(spacing=6)
            sel = self.tab == k
            imgui.push_style_color(imgui.Col_.button, S.v4(S.ACCENT, 0.18) if sel else S.v4(S.BG1))
            imgui.push_style_color(imgui.Col_.button_hovered, S.v4(S.ACCENT, 0.28) if sel else S.v4(S.BG3))
            imgui.push_style_color(imgui.Col_.text, S.v4(S.ACCENT) if sel else S.v4(S.MUTED))
            if imgui.button(f'{ic}  {label}', imgui.ImVec2(bw, 34 * self.scale / max(self.scale, 1))):
                self.tab = k
            imgui.pop_style_color(3)
        imgui.spacing()
        feed_h = 230 * self.scale
        imgui.begin_child('tabbody', imgui.ImVec2(0, imgui.get_content_region_avail().y - feed_h), imgui.ChildFlags_.none)
        tabs[self.tab][2]()
        imgui.end_child()
        self._event_feed(feed_h)
        imgui.end()

    def _event_feed(self, h):
        S.begin_card('feed', h - 8)
        S.section(FA.ICON_FA_LIST, '이벤트')
        imgui.same_line(imgui.get_content_region_avail().x - 110 * self.scale)
        self.log_filter = S.segmented(['전체', '차량'], self.log_filter)
        S.tip('전체 로그 / 차량 이벤트(추돌, SCC 인식)만')
        imgui.begin_child('feedlist', imgui.ImVec2(0, 0))
        with self.log_lock:
            lines = list(self.log_lines)
        for ln in lines:
            ev = '차량 #' in ln
            if self.log_filter == 1 and not ev:
                continue
            ts, _, msg = ln.partition(' ')
            if '추돌' in msg:
                ic, col = FA.ICON_FA_CAR_BURST, S.DANGER
            elif 'SCC' in msg:
                ic, col = FA.ICON_FA_GAUGE_HIGH, S.SCC
            elif '실패' in msg or '없음' in msg:
                ic, col = FA.ICON_FA_TRIANGLE_EXCLAMATION, S.WARN
            else:
                ic, col = FA.ICON_FA_CIRCLE_INFO, S.MUTED
            imgui.text_colored(S.v4(col), ic)
            imgui.same_line()
            imgui.push_font(S.Fonts.body, S.Fonts.SMALL)
            S.muted(ts)
            imgui.same_line()
            imgui.push_style_color(imgui.Col_.text, S.v4(col if col != S.MUTED else S.TEXT))
            imgui.text_wrapped(msg)
            imgui.pop_style_color()
            imgui.pop_font()
        if imgui.get_scroll_y() >= imgui.get_scroll_max_y() - 5:
            imgui.set_scroll_here_y(1.0)
        imgui.end_child()
        S.end_card()

    # ---- 위쪽 재생 바 (3D 뷰 위에 떠 있음)
    def _playbar(self, snap, pw, W):
        cx = pw + (W - pw) / 2
        imgui.set_next_window_pos(imgui.ImVec2(cx, 12), imgui.Cond_.always, imgui.ImVec2(0.5, 0))
        imgui.set_next_window_bg_alpha(0.88)
        imgui.push_style_var(imgui.StyleVar_.window_rounding, 14.0 * self.scale)
        imgui.push_style_var(imgui.StyleVar_.window_padding, imgui.ImVec2(14, 8))
        imgui.begin('playbar', None, imgui.WindowFlags_.no_decoration | imgui.WindowFlags_.always_auto_resize |
                    imgui.WindowFlags_.no_move | imgui.WindowFlags_.no_saved_settings)
        busy = self.build_future is not None and not self.build_future.done()
        if S.primary_button(FA.ICON_FA_PLAY if self.paused else FA.ICON_FA_PAUSE, imgui.ImVec2(36, 0)):
            self._toggle_pause()
        S.tip('재생 / 일시정지 (Space)')
        imgui.same_line()
        speeds = [0.5, 1.0, 2.0, 4.0]
        cur = min(range(4), key=lambda i: abs(speeds[i] - self.rtf))
        new = S.segmented(['0.5x', '1x', '2x', '4x'], cur)
        if new != cur:
            self.rtf = speeds[new]
            if self.eth is not None:
                self.eth.set_rtf(self.rtf)
        S.tip('시뮬레이션 배속')
        imgui.same_line()
        imgui.begin_disabled(busy)
        if imgui.button(FA.ICON_FA_ROTATE_RIGHT):
            self.rebuild()
        S.tip('시나리오 재시작 (R) — 시나리오·교통·환경 다시 생성')
        imgui.same_line()
        if imgui.button(FA.ICON_FA_MAP):
            self.rebuild(map_too=True)
        S.tip('맵 재생성 (G)')
        imgui.end_disabled()
        if snap is not None:
            imgui.same_line()
            rtf = self.eth.measured_rtf if self.eth else 0
            S.chip(f'{snap["t"]:6.1f} s · RTF {rtf:.2f}', S.TEXT, FA.ICON_FA_CLOCK)
            imgui.same_line()
            env = self.engine.env
            eic = {'night': FA.ICON_FA_MOON, 'dusk': FA.ICON_FA_CLOUD_SUN}.get(env.time_of_day, FA.ICON_FA_SUN)
            if env.weather == 'rain':
                eic = FA.ICON_FA_CLOUD_RAIN
            elif env.weather == 'snow':
                eic = FA.ICON_FA_SNOWFLAKE
            elif env.weather == 'fog':
                eic = FA.ICON_FA_SMOG
            S.chip(env.label(), S.TEAL, eic)
            imgui.same_line()
            src = snap['cmd_src']
            S.chip(f'명령 {src}', S.OK if src != '-' else S.MUTED, FA.ICON_FA_GAMEPAD)
            S.tip('비콘 이동 명령 출처: /cmd_vel(상위 제어기) 우선, 없으면 방향키')
            imgui.same_line()
            ros = self.ros is not None and self.ros.ok
            S.chip('ROS2' if ros else 'ROS2 없음', S.ACCENT if ros else S.MUTED, FA.ICON_FA_LINK)
        imgui.same_line()
        imgui.text_colored(S.v4(S.BORDER), '|')
        for k, ic, label in [('pandar', FA.ICON_FA_CIRCLE_DOT, 'P40'), ('tele', FA.ICON_FA_BULLSEYE, 'Tele'),
                             ('radar', FA.ICON_FA_WIFI, '레이더'), ('gnss', FA.ICON_FA_SATELLITE, 'GNSS'),
                             ('camera', FA.ICON_FA_CAMERA, '')]:
            imgui.same_line()
            self.show[k] = S.toggle_button(ic, label, self.show[k])
        imgui.same_line()
        if imgui.button(FA.ICON_FA_CIRCLE_QUESTION):
            self.help_open = True
        S.tip('조작 도움말 (F1)')
        imgui.end()
        imgui.pop_style_var(2)

    def _help_popup(self):
        if self.help_open:
            imgui.open_popup('조작 도움말')
            self.help_open = False
        imgui.set_next_window_size(imgui.ImVec2(520 * self.scale, 0))
        if imgui.begin_popup_modal('조작 도움말', None, imgui.WindowFlags_.no_resize)[0]:
            for k, v in [('↑ / ↓', '비콘 전진 / 후진 (ROS /cmd_vel이 오면 그쪽 우선)'), ('← / →', '비콘 회전'),
                         ('좌클릭 드래그', '시점 회전'), ('우클릭 드래그', '시점 이동 (따라가기 해제)'),
                         ('휠', '확대 / 축소'), ('F', '비콘 따라가기'), ('1  2  3', '비콘 / 전체 / 터널 시점'),
                         ('Space', '일시정지 / 재생'), ('R', '시나리오 재시작'), ('G', '맵 재생성'), ('F1', '도움말')]:
                S.chip(k, S.ACCENT)
                imgui.same_line(150 * self.scale)
                imgui.set_cursor_pos_y(imgui.get_cursor_pos_y() + 3)
                imgui.text(v)
            imgui.spacing()
            S.muted('주황 기둥 = 사고 현장,  비콘 위 짧은 선 = GNSS 위치 (색 = RTK 상태)')
            imgui.spacing()
            if S.primary_button('닫기', imgui.ImVec2(-1, 0)):
                imgui.close_current_popup()
            imgui.end_popup()

    # ---- 사이드바 탭 내용
    def _tab_map(self):
        p = self.params
        S.begin_card('map_gen')
        S.section(FA.ICON_FA_ROAD, '도로 생성')
        _, p.seed = imgui.input_int('맵 시드', p.seed)
        p.seed = max(0, p.seed)
        _, self.rand_map = imgui.checkbox('재생성할 때마다 맵 시드 랜덤', self.rand_map)
        _, p.length = imgui.slider_float('길이', p.length, 1000.0, 10000.0, '%.0f m')
        _, p.lanes = imgui.slider_int('방향별 차로', p.lanes, 2, 4)
        speeds = [100, 120]
        ch, i = imgui.combo('설계속도', speeds.index(p.design_speed) if p.design_speed in speeds else 0,
                            ['100 km/h', '120 km/h'])
        if ch:
            p.design_speed = speeds[i]
        S.tip('최소 곡선반경: 100 km/h → 460 m, 120 km/h → 710 m')
        _, p.tunnels = imgui.slider_int('터널 (최대)', p.tunnels, 0, 4)
        _, p.gantries = imgui.slider_int('갠트리', p.gantries, 0, 5)
        S.end_card()
        S.begin_card('map_lane')
        S.section(FA.ICON_FA_GRIP_LINES_VERTICAL, '차선 상태')
        _, p.wear = imgui.slider_float('불량 구간 비율', p.wear, 0.0, 1.0, '%.2f')
        S.tip('차선 도색 상태가 나쁜 구간의 비율 (Lanelet 맵에는 선이 그대로 남음)')
        _, p.wear_faded = imgui.slider_float('희미함', p.wear_faded, 0.0, 1.0, '%.2f')
        _, p.wear_broken = imgui.slider_float('끊김', p.wear_broken, 0.0, 1.0, '%.2f')
        _, p.wear_sparse = imgui.slider_float('희소', p.wear_sparse, 0.0, 1.0, '%.2f')
        if imgui.tree_node('고급'):
            _, p.tunnel_len_min = imgui.slider_float('터널 최소', p.tunnel_len_min, 100.0, 2000.0, '%.0f m')
            _, p.tunnel_len_max = imgui.slider_float('터널 최대', p.tunnel_len_max, p.tunnel_len_min, 3000.0, '%.0f m')
            _, p.lane_width = imgui.slider_float('차로 폭', p.lane_width, 3.25, 3.75, '%.2f m')
            _, p.dash_len = imgui.slider_float('점선 도색', p.dash_len, 3.0, 15.0, '%.1f m')
            _, p.dash_gap = imgui.slider_float('점선 빈칸', p.dash_gap, 3.0, 20.0, '%.1f m')
            _, p.region_lat = imgui.input_float('지역 위도', p.region_lat, 0.1, 1.0, '%.4f')
            _, p.region_lon = imgui.input_float('지역 경도', p.region_lon, 0.1, 1.0, '%.4f')
            imgui.tree_pop()
        S.end_card()
        if S.primary_button(f'{FA.ICON_FA_MAP}  맵 재생성  (G)', imgui.ImVec2(-1, 0)):
            self.rebuild(map_too=True)
        if self.model is not None:
            sm = self.model.summary()
            S.begin_card('map_info')
            S.section(FA.ICON_FA_CIRCLE_INFO, '현재 맵', S.MUTED)
            imgui.text(f'시드 {sm["seed"]} · {sm["length_m"]:.0f} m · {sm["lanes"]}차로×2 · MGRS {sm["mgrs_grid"]}')
            S.muted(f'최소반경 {sm["min_radius_m"]:.0f} m · 종단 {sm["max_grade_pct"]:.1f}% · 편경사 {sm["max_superelevation_pct"]:.1f}%')
            tun = ', '.join(f'{a}~{b}' for a, b in sm['tunnels']) or '없음'
            S.muted(f'터널 {tun} · 차선 불량 {sm["lane_wear_pct"]}%')
            S.end_card()

    def _tab_scenario(self, snap):
        sp = self.sparams
        S.begin_card('scn')
        S.section(FA.ICON_FA_CAR_BURST, '사고 현장', S.AMBER)
        _, sp.seed = imgui.input_int('시나리오 시드', sp.seed)
        _, self.rand_scn = imgui.checkbox('재시작할 때마다 시나리오 시드 랜덤', self.rand_scn)
        sides = ['random', 'down', 'up']
        ch, i = imgui.combo('방향', sides.index(sp.side) if sp.side in sides else 0, ['랜덤', '하행', '상행'])
        if ch:
            sp.side = sides[i]
        items = ['랜덤'] + [f'{k + 1}차로' for k in range(self.params.lanes)]
        ch, i = imgui.combo('비콘 차로', min(sp.lane + 1, len(items) - 1), items)
        if ch:
            sp.lane = i - 1
        _, sp.accident_vehicles = imgui.slider_int('사고 차량', sp.accident_vehicles, 1, 3)
        wh = ['random', 'open', 'tunnel_entry', 'tunnel', 'tunnel_exit']
        ch, i = imgui.combo('사고 위치', wh.index(sp.accident_where) if sp.accident_where in wh else 0,
                            ['랜덤', '개활지', '터널 입구', '터널 안', '터널 출구'])
        if ch:
            sp.accident_where = wh[i]
        _, sp.beacon_light = imgui.checkbox('비콘 경광등 켜기', sp.beacon_light)
        S.tip('끄면 야간에 운전자가 비콘을 훨씬 늦게 인지합니다 (인지 거리 0.9배 → 0.35배)')
        S.end_card()
        S.begin_card('scn_key')
        S.section(FA.ICON_FA_KEYBOARD, '비콘 키보드 조작')
        S.muted('ROS /cmd_vel 이 들어오면 그쪽이 우선')
        _, self.key_speed = imgui.slider_float('방향키 속도', self.key_speed, 0.2, 3.0, '%.1f m/s')
        _, self.key_yaw = imgui.slider_float('방향키 회전', self.key_yaw, 0.1, 1.5, '%.2f rad/s')
        S.end_card()
        if S.primary_button(f'{FA.ICON_FA_ROTATE_RIGHT}  시나리오 재시작  (R)', imgui.ImVec2(-1, 0)):
            self.rebuild()
        if self.engine is not None:
            sm = self.engine.scn.summary()
            S.begin_card('scn_info')
            S.section(FA.ICON_FA_CIRCLE_INFO, '현재 시나리오', S.MUTED)
            imgui.text(f'{sm["side"]} {sm["lane"]}차로 · 사고 s = {sm["accident_s"]:.0f} m · 사고 차량 {sm["accident_vehicles"]}대')
            S.muted(f'시드 {self.engine.scn.sp.seed}')
            S.end_card()

    def _tab_traffic(self):
        tp = self.tparams
        S.begin_card('tr_flow')
        S.section(FA.ICON_FA_CAR_SIDE, '교통 흐름')
        _, tp.enabled = imgui.checkbox('접근 차량 켜기', tp.enabled)
        S.tip('끄면 기능 검증 모드: 차량 없이 비콘·사고 현장만')
        imgui.begin_disabled(not tp.enabled)
        _, tp.flow = imgui.slider_float('교통량', tp.flow, 0.0, 2000.0, '%.0f 대/h/차로')
        _, tp.flow_opposite = imgui.slider_float('반대 방향', tp.flow_opposite, 0.0, 2000.0, '%.0f 대/h/차로')
        _, tp.speed_mean = imgui.slider_float('평균 속도', tp.speed_mean, 10.0, 40.0, '%.1f m/s')
        _, tp.truck_ratio = imgui.slider_float('대형차 비율', tp.truck_ratio, 0.0, 0.6, '%.2f')
        imgui.end_disabled()
        S.end_card()
        S.begin_card('tr_drv')
        S.section(FA.ICON_FA_USER, '운전자 반응 (비콘 인지 후)', S.WARN)
        _, tp.p_lane_change = imgui.slider_float('차로변경', tp.p_lane_change, 0.0, 1.0, '%.2f')
        _, tp.p_brake = imgui.slider_float('감속', tp.p_brake, 0.0, 1.0, '%.2f')
        _, tp.p_crash = imgui.slider_float('무반응', tp.p_crash, 0.0, 1.0, '%.2f')
        S.tip('비콘을 보지 못하고 그대로 진행 → 추돌 가능')
        _, tp.notice_min = imgui.slider_float('인지 거리 최소', tp.notice_min, 10.0, 300.0, '%.0f m')
        _, tp.notice_max = imgui.slider_float('인지 거리 최대', tp.notice_max, tp.notice_min, 500.0, '%.0f m')
        S.muted('비율은 합으로 정규화')
        S.end_card()
        S.begin_card('tr_scc')
        S.section(FA.ICON_FA_GAUGE_HIGH, 'SCC (적응형 순항)', S.SCC)
        _, tp.p_scc = imgui.slider_float('작동 비율', tp.p_scc, 0.0, 1.0, '%.2f')
        _, tp.scc_gap_min = imgui.slider_float('차간 시간 최소', tp.scc_gap_min, 0.8, 3.0, '%.1f s')
        _, tp.scc_gap_max = imgui.slider_float('차간 시간 최대', tp.scc_gap_max, tp.scc_gap_min, 3.0, '%.1f s')
        _, tp.scc_decel_min = imgui.slider_float('최대 감속 최소', tp.scc_decel_min, 1.0, 8.0, '%.1f m/s²')
        _, tp.scc_decel_max = imgui.slider_float('최대 감속 최대', tp.scc_decel_max, tp.scc_decel_min, 8.0, '%.1f m/s²')
        _, tp.scc_range = imgui.slider_float('레이더 거리', tp.scc_range, 50.0, 250.0, '%.0f m')
        _, tp.scc_moving_min = imgui.slider_float('이동 판정 속도', tp.scc_moving_min, 0.1, 3.0, '%.1f m/s')
        S.muted('정지한 비콘은 무시, 사고 현장 쪽으로 후진하면 앞차로 인식')
        S.end_card()
        S.muted(f'{FA.ICON_FA_CIRCLE_INFO}  변경은 시나리오 재시작(R) 시 반영')

    def _tab_env(self):
        e = self.eparams
        S.begin_card('env')
        S.section(FA.ICON_FA_CLOUD_SUN_RAIN, '시간대 · 날씨', S.TEAL)
        times = ['random', 'day', 'dusk', 'night']
        k = times.index(e.time_of_day) if e.time_of_day in times else 0
        k = S.segmented(['랜덤', '낮', '황혼', '밤'], k, 72 * self.scale / max(self.scale, 1))
        e.time_of_day = times[k]
        wxs = ['random', 'clear', 'rain', 'fog', 'snow']
        k = wxs.index(e.weather) if e.weather in wxs else 0
        k = S.segmented(['랜덤', '맑음', '비', '안개', '눈'], k, 57 * self.scale / max(self.scale, 1))
        e.weather = wxs[k]
        rnd = e.intensity < 0
        ch, rnd = imgui.checkbox('강도 랜덤', rnd)
        if ch:
            e.intensity = -1.0 if rnd else 0.5
        if not rnd:
            _, e.intensity = imgui.slider_float('강도', e.intensity, 0.0, 1.0, '%.2f')
        if imgui.tree_node('랜덤 확률'):
            _, e.p_night = imgui.slider_float('밤', e.p_night, 0.0, 1.0, '%.2f')
            _, e.p_dusk = imgui.slider_float('황혼', e.p_dusk, 0.0, 1.0, '%.2f')
            _, e.p_rain = imgui.slider_float('비', e.p_rain, 0.0, 1.0, '%.2f')
            _, e.p_fog = imgui.slider_float('안개', e.p_fog, 0.0, 1.0, '%.2f')
            _, e.p_snow = imgui.slider_float('눈', e.p_snow, 0.0, 1.0, '%.2f')
            imgui.tree_pop()
        S.end_card()
        if self.engine is not None:
            env = self.engine.env
            S.begin_card('env_now')
            S.section(FA.ICON_FA_CIRCLE_INFO, f'현재  {env.label()}', S.MUTED)
            S.muted(f'카메라 대기 산란 {env.fog_dist:.0f} m · LiDAR 감쇠 α {env.alpha:.4f}/m')
            S.muted(f'허위점 {env.false_rate:.4f} · 노면 젖음 {env.wet:.2f} · 레이더 SNR −{env.radar_snr_loss:.1f} dB')
            S.end_card()
        S.muted(f'{FA.ICON_FA_CIRCLE_INFO}  변경은 시나리오 재시작(R) 시 반영')

    def _tab_view(self):
        S.begin_card('v_sensor')
        S.section(FA.ICON_FA_SATELLITE_DISH, '센서')
        for k, label in [('pandar', 'Pandar40P'), ('tele', 'Tele-15'), ('radar', 'ARS408 클러스터·객체'),
                         ('radar_fov', 'ARS408 감지 범위 (면)'), ('gnss', 'GNSS 위치·오차원·궤적'), ('camera', '카메라 창')]:
            _, self.show[k] = imgui.checkbox(label, self.show[k])
        S.muted('Pandar 색상')
        self.pandar_color = S.segmented(['세기', '정답 라벨'], self.pandar_color)
        S.end_card()
        S.begin_card('v_scene')
        S.section(FA.ICON_FA_LAYER_GROUP, '장면')
        for k, label in [('labels', '3D 이름표'), ('vehicles', '차량'), ('scc', 'SCC·운전자 상태 표시'), ('robot', '비콘'),
                         ('markers', '사고 현장 표시'), ('lanelet', 'Lanelet 경계선'), ('terrain', '지형'),
                         ('tunnel', '터널'), ('auto_roof', '비콘이 터널 안이면 천장 숨김')]:
            _, self.show[k] = imgui.checkbox(label, self.show[k])
        ch, self.show['env_view'] = imgui.checkbox('3D 뷰에도 밤·날씨 적용', self.show['env_view'])
        if ch and self.engine is not None:
            self.renderer.set_env(self.engine.env if self.show['env_view'] else None)
        S.end_card()
        S.begin_card('v_cam')
        S.section(FA.ICON_FA_VIDEO, '시점')
        _, self.follow = imgui.checkbox('비콘 따라가기 (F)', self.follow)
        for i, (key, label) in enumerate([('beacon', '비콘'), ('overview', '전체'), ('tunnel', '터널')]):
            if i:
                imgui.same_line()
            if imgui.button(f'{label}  {i + 1}'):
                self._preset(key)
        S.end_card()

    def _tab_export(self):
        S.begin_card('ex')
        S.section(FA.ICON_FA_FLOPPY_DISK, 'Autoware 맵 세트')
        _, self.export_dir = imgui.input_text('##dir', self.export_dir)
        imgui.same_line()
        if imgui.button(FA.ICON_FA_FOLDER_OPEN):
            try:
                from imgui_bundle import portable_file_dialogs as pfd
                r = pfd.select_folder('저장 위치 선택', self.export_dir).result()
                if r:
                    self.export_dir = r
            except Exception as e:      # noqa: BLE001
                self.log(f'폴더 선택 창을 열 수 없음: {e}')
        _, self.export_pcd = imgui.checkbox('PCD 포함', self.export_pcd)
        S.muted('lanelet2_map.osm · map_projector_info.yaml · pointcloud_map.pcd · scenario.yaml')
        busy = self.export_future is not None and not self.export_future.done()
        imgui.begin_disabled(busy)
        if S.primary_button(f'{FA.ICON_FA_DOWNLOAD}  ' + ('저장 중...' if busy else '저장 + 맵 검증'), imgui.ImVec2(-1, 0)):
            self.export_async()
        imgui.end_disabled()
        S.end_card()
        S.begin_card('ex2')
        S.section(FA.ICON_FA_TERMINAL, '헤드리스 심')
        S.muted('현재 맵·시나리오·교통·환경 설정을 config/sim.yaml 에 기록')
        if imgui.button(f'{FA.ICON_FA_FILE_EXPORT}  sim.yaml 에 적용', imgui.ImVec2(-1, 0)):
            try:
                keys = ['seed', 'length', 'lanes', 'design_speed', 'tunnels', 'gantries', 'wear', 'wear_faded',
                        'wear_broken', 'wear_sparse', 'tunnel_len_min', 'tunnel_len_max', 'dash_len', 'dash_gap',
                        'region_lat', 'region_lon']
                d = self.params.to_dict()
                path = apply_to_sim_config({'map': {k: d[k] for k in keys}, 'scenario': self.engine.scn.sp.to_dict(),
                                            'traffic': self.tparams.to_dict(), 'environment': self.eparams.to_dict()})
                self.log(f'적용: {path}')
            except Exception as e:      # noqa: BLE001
                self.log(f'적용 실패: {e}')
        S.end_card()

    # ---- 오른쪽 상황판
    def _dashboard(self, snap, W):
        dw = 300 * self.scale
        imgui.set_next_window_pos(imgui.ImVec2(W - dw - 12, 64 * self.scale))
        imgui.set_next_window_size(imgui.ImVec2(dw, 0))
        imgui.set_next_window_bg_alpha(0.0)
        imgui.begin('dash', None, imgui.WindowFlags_.no_decoration | imgui.WindowFlags_.no_move |
                    imgui.WindowFlags_.no_saved_settings | imgui.WindowFlags_.no_focus_on_appearing |
                    imgui.WindowFlags_.no_nav | imgui.WindowFlags_.always_auto_resize | imgui.WindowFlags_.no_inputs)
        if snap is None:
            S.chip('준비 중...', S.MUTED, FA.ICON_FA_HOURGLASS_HALF)
            imgui.end()
            return
        scn = self.engine.scn
        S.begin_card('d_beacon')
        S.section(FA.ICON_FA_TOWER_BROADCAST, '비콘', S.WARN)
        half = dw / 2 - 8
        S.kpi('속도', f'{snap["v"]:+.1f} m/s', S.TEXT, half)
        S.kpi('사고 현장까지', f'{abs(snap["s_b"] - scn.accident_s):.0f} m', S.AMBER)
        if snap.get('damaged'):
            S.chip('비콘 파손 — 명령 무시', S.DANGER, FA.ICON_FA_CAR_BURST, filled=True)
        elif snap.get('contact'):
            S.chip(f'접촉: {snap["contact"]}', S.WARN, FA.ICON_FA_HAND)
        g = snap['sensors'].get('gnss')
        if g is not None:
            gc = {4: S.OK, 5: S.WARN, 1: S.AMBER, 0: S.DANGER}[g['status']]
            S.chip(f'GNSS {g["status_name"]} · σ {g["hacc"]:.2f} m', gc, FA.ICON_FA_SATELLITE)
        S.end_card()
        S.begin_card('d_traffic')
        S.section(FA.ICON_FA_CAR_SIDE, '접근 차량')
        c = snap['counts']
        third = (dw - 24) / 3
        imgui.columns(3, 'kc', False)
        for label, val, col in [('주행', int(snap['active'].sum()), S.TEXT), ('비콘 인지', c['noticed'], S.WARN),
                                ('차로변경', c['lane_change'], S.ACCENT), ('감속', c['brake'], S.TEAL),
                                ('SCC 인식', c['scc_lock'], S.SCC), ('SCC 추종', int((snap['scc_target'] == -2).sum()), S.SCC)]:
            S.kpi(label, val, col)
            imgui.next_column()
        imgui.columns(1)
        imgui.spacing()
        crash = c['crash']
        S.chip(f'추돌 {crash}  ·  비콘 추돌 {c["beacon_hit"]}', S.DANGER if crash else S.OK,
               FA.ICON_FA_CAR_BURST if crash else FA.ICON_FA_CIRCLE_CHECK, filled=bool(crash))
        S.end_card()
        if self.show['legend']:
            S.begin_card('d_legend')
            if imgui.tree_node_ex('범례', imgui.TreeNodeFlags_.none):
                for col, label in [(S.TEAL, 'SCC 작동 (접근 차량)'), (S.SCC, 'SCC가 비콘 추종 (비콘까지 선)'),
                                   (S.WARN, '운전자가 비콘 인지'), (S.DANGER, '추돌'),
                                   ((1, 0.3, 0.3, 1), '레이더 감지 범위(면) · 클러스터'), (S.AMBER, '레이더 감지 객체 (앞면 사각 판) · 사고 현장'),
                                   ((1, 0.62, 0.25, 1), 'Tele-15'), (S.ACCENT, 'Pandar40P (세기)'),
                                   ((1, 1, 1, 1), 'GNSS 정답 궤적 / 측정(상태색)')]:
                    imgui.text_colored(S.v4(col), FA.ICON_FA_CIRCLE)
                    imgui.same_line()
                    imgui.push_font(S.Fonts.body, S.Fonts.SMALL)
                    imgui.text(label)
                    imgui.pop_font()
                imgui.tree_pop()
            S.end_card()
        imgui.end()

    def _world_labels(self, snap, VP):
        """3D 위치 이름표 (알약 모양)."""
        if not self.show['labels'] or snap is None:
            return
        io = imgui.get_io()
        W, H = io.display_size
        pw = getattr(self, 'sidebar_w', 0)
        dl = imgui.get_background_draw_list()
        scn, m = self.engine.scn, self.model

        def put(p, text, rgba):
            q = VP @ np.r_[p, 1.0]
            if q[3] <= 0.1:
                return
            x = (q[0] / q[3] * 0.5 + 0.5) * W
            y = (1 - (q[1] / q[3] * 0.5 + 0.5)) * H
            if pw + 40 < x < W - 40 and 50 < y < H:
                S.pill_label(dl, x, y, text, rgba)

        bpos = snap['T_wb'][:3, 3]
        put(bpos + [0, 0, 1.6], '비콘', S.WARN)
        if self.show['markers']:
            put(m.point(scn.accident_s, scn.lane_t, 6.5), '사고 현장', S.AMBER)
        if self.show['vehicles']:
            Ts = snap['traffic_T']
            for i in np.nonzero(snap['active'])[0]:
                if np.linalg.norm(Ts[i][:3, 3] - bpos) > 300:
                    continue
                top = Ts[i][:3, 3] + [0, 0, self.engine.traffic.specs[i].height + 1.4]
                if snap['crashed'][i]:
                    put(top, '추돌', S.DANGER)
                elif snap['scc_target'][i] == -2:
                    v = np.linalg.norm(snap['traffic_vel'][i]) * 3.6
                    put(top, f'SCC 추종 {v:.0f} km/h', S.SCC)

    def _radar_fan_local(self):
        """ARS408 감지 범위 면 (센서 좌표, 한 번만 생성): 근거리 ±60°/70m(45° 밖 40m) + 원거리 ±9°/250m."""
        if getattr(self, '_fan_local', None) is not None:
            return self._fan_local
        rc = self.base_cfg['ars408']
        nf, ff = rc['near'], rc['far']
        tris, cols, edges = [], [], []

        def fan(az0, az1, r_of, rgba, n):
            az = np.radians(np.linspace(az0, az1, n))
            r = np.array([r_of(a) for a in np.degrees(np.abs(az))])
            arc = np.c_[r * np.cos(az), r * np.sin(az), np.zeros(n)]
            for k in range(n - 1):
                tris.extend([[0, 0, 0], arc[k], arc[k + 1]])
                cols.extend([rgba] * 3)
            outline = np.vstack([[0, 0, 0], arc, [0, 0, 0]])
            for a, b in zip(outline[:-1], outline[1:]):
                edges.extend([a, b])

        fan(-nf['fov_deg'], nf['fov_deg'],
            lambda a: nf['max_range'] if a <= nf['wide_fov_deg'] + 1e-6 else nf['wide_max_range'],
            (1.0, 0.30, 0.30, 0.16), 121)
        fan(-ff['fov_deg'], ff['fov_deg'], lambda a: ff['max_range'], (1.0, 0.30, 0.30, 0.12), 19)
        self._fan_local = (np.array(tris, 'f4'), np.array(cols, 'f4'), np.array(edges, 'f4'))
        return self._fan_local

    def _camera_window(self, W, H):
        cw = 520 * self.scale
        chh = cw * self.camera.H / self.camera.W
        imgui.set_next_window_pos(imgui.ImVec2(W - cw - 24, H - chh - 70), imgui.Cond_.first_use_ever)
        imgui.set_next_window_size(imgui.ImVec2(cw + 20, chh + 58), imgui.Cond_.first_use_ever)
        imgui.push_style_var(imgui.StyleVar_.window_padding, imgui.ImVec2(10, 8))
        imgui.push_style_color(imgui.Col_.window_bg, S.v4(S.BG1, 0.95))
        opened, self.show['camera'] = imgui.begin(f'{FA.ICON_FA_CAMERA}  카메라###cam', self.show['camera'],
                                                  imgui.WindowFlags_.no_collapse | imgui.WindowFlags_.no_title_bar)
        if opened:
            S.chip('Arducam B0495', S.ACCENT, FA.ICON_FA_CAMERA)
            imgui.same_line()
            S.chip(f'{self.camera.W}×{self.camera.H} · {self.base_cfg["camera"]["rate_hz"]:.0f} Hz · '
                   f'HFOV {self.camera.hfov:.0f}°', S.MUTED)
            imgui.same_line(imgui.get_window_width() - 34)
            if imgui.small_button(FA.ICON_FA_XMARK):
                self.show['camera'] = False
            w = imgui.get_content_region_avail().x
            p = imgui.get_cursor_screen_pos()
            imgui.image(imgui.ImTextureRef(self.cam_tex.glo), imgui.ImVec2(w, w * self.camera.H / self.camera.W))
            imgui.get_window_draw_list().add_rect(p, imgui.ImVec2(p.x + w, p.y + w * self.camera.H / self.camera.W),
                                                  S.u32(S.BORDER), 6.0, 1.0)
        imgui.end()
        imgui.pop_style_color()
        imgui.pop_style_var()

    # ------------------------------------------------------------ 3D
    def _view(self, aspect):
        P = perspective(50, aspect, 0.3, 12000)
        c = self.cam
        d = np.array([np.cos(c['pitch']) * np.cos(c['yaw']), np.cos(c['pitch']) * np.sin(c['yaw']), np.sin(c['pitch'])])
        eye = c['target'] + d * c['dist']
        return P, look_at(eye, c['target']), eye

    def _update_camera_sensor(self, snap):
        """sim 시각 기준 카메라 주기마다 카메라 영상 렌더 (창 표시 또는 ROS 발행 시)."""
        need_ros = self.ros is not None and self.ros.ok
        if not (self.show['camera'] or need_ros) or snap['t'] < self.cam_next_t:
            return
        self.cam_next_t = snap['t'] + 1.0 / self.base_cfg['camera']['rate_hz']
        T_wb = snap['T_wb']
        veh = list(self.engine.accident) + list(zip(self.engine.traffic.specs, snap['traffic_T']))
        img = self.camera.render(self.cam_scene, T_wb @ self.T_b_opt, T_wb, veh, self.engine.env)
        if need_ros:
            self.ros.publish_camera(img, snap['t'], self.camera, self.base_cfg['camera']['optical_frame'])
        if self.show['camera']:
            small = np.ascontiguousarray(img[::2, ::2])
            if self.cam_tex is None or self.cam_tex.size != (small.shape[1], small.shape[0]):
                if self.cam_tex is not None:
                    self.gui.textures.pop(self.cam_tex.glo, None)
                    self.cam_tex.release()
                self.cam_tex = self.ctx.texture((small.shape[1], small.shape[0]), 3)
                self.cam_tex.filter = (moderngl.LINEAR, moderngl.LINEAR)
                self.gui.textures[self.cam_tex.glo] = self.cam_tex
            self.cam_tex.write(small.tobytes())

    def _world_pts(self, key, o, cols_fn):
        """센서 점군 → 월드 좌표·색 (센서 프레임이 바뀔 때만 다시 계산)."""
        c = getattr(self, '_pt_cache', {})
        self._pt_cache = c
        tag = (o['t'], self.pandar_color)
        if key not in c or c[key][0] != tag:
            p = o['points']
            xyz = (np.c_[p['x'], p['y'], p['z']] @ o['T_ws'][:3, :3].T + o['T_ws'][:3, 3]).astype('f4')
            c[key] = (tag, xyz, cols_fn(o).astype('f4'))
        return c[key][1], c[key][2]

    def _draw_overlays(self, snap, VP):
        e = self.engine
        ov = self.ov
        if self.show['lanelet']:
            ov.draw('lanes', self.lane_pts, self.lane_cols, VP, moderngl.LINES)
        if self.show['markers']:
            ov.draw('markers', self.marker_pts, self.marker_cols, VP, moderngl.LINES)
        S = snap['sensors']
        if self.show['pandar'] and 'pandar40p' in S:
            def pcol(o):
                if self.pandar_color == 1:
                    lab = np.clip(o['gt']['label'], 0, len(LABEL_COLORS) - 1)
                    return np.c_[LABEL_COLORS[lab], np.ones(len(lab), np.float32)]
                return intensity_colors(o['points']['intensity'])
            xyz, cols = self._world_pts('pandar', S['pandar40p'], pcol)
            ov.draw('pandar', xyz, cols, VP, size=2.0 * self.scale)
        if self.show['tele'] and 'tele15' in S:
            def tcol(o):
                t = np.clip(o['points']['intensity'] / 60.0, 0, 1)[:, None]
                return np.c_[np.full_like(t, 1.0), 0.45 + 0.5 * t, 0.1 + 0.3 * t, np.ones_like(t)]
            xyz, cols = self._world_pts('tele', S['tele15'], tcol)
            ov.draw('tele', xyz, cols, VP, size=2.0 * self.scale)
        if self.show['radar'] and 'ars408' in S:
            o = S['ars408']
            c, T = o['clusters'], o['T_ws']
            if self.show['radar_fov']:
                # 센서에 고정된 감지 범위 면 (현재 비콘 자세 기준 → 흔들리지 않음)
                tri, fill_c, edge = self._radar_fan_local()
                Tf = snap['T_wb'] @ self.engine.mounts['ars408'][1]
                ov.draw('radar_fov', tri @ Tf[:3, :3].T + Tf[:3, 3], fill_c, VP, moderngl.TRIANGLES, blend=True)
                ov.draw('radar_edge', edge @ Tf[:3, :3].T + Tf[:3, 3], (1.0, 0.40, 0.40, 0.8), VP, moderngl.LINES)
            if len(c):
                xyz = np.c_[c['x'], c['y'], np.zeros(len(c))] @ T[:3, :3].T + T[:3, 3]
                ov.draw('radar', xyz, (1.0, 0.15, 0.15, 1.0), VP, size=7.0 * self.scale)
            # 감지 객체: 레이더를 마주 보는 사각 판 (두께 0.6 m × 차폭 × 차고, RViz 마커와 동일)
            tris, edges = [], []
            ex, ey = T[:3, 0].copy(), T[:3, 1].copy()
            ex[2] = ey[2] = 0.0
            ex /= max(np.linalg.norm(ex), 1e-9)
            ey /= max(np.linalg.norm(ey), 1e-9)
            for tk in o['tracks']:
                sp = tk['spec']
                c = T[:3, :3] @ np.array([tk['x'], tk['y'], 0.0]) + T[:3, 3]
                s_, t_ = self.model.project(c[0], c[1])
                z0 = float(self.model.surface_z(s_, np.clip(t_, -self.model.paved_t, self.model.paved_t)))
                hw, d = sp.width / 2, 0.3
                corners = []
                for dx in (-d, d):
                    for dy in (-hw, hw):
                        for z in (z0, z0 + sp.height):
                            q = c + ex * dx + ey * dy
                            corners.append([q[0], q[1], z])
                P8 = np.array(corners)            # 인덱스: (dx, dy, z) 비트
                for f in [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]:
                    a, b, cc, dd = (P8[k] for k in f)
                    tris += [a, b, cc, a, cc, dd]
                for e0, e1 in [(0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7), (0, 4), (1, 5), (2, 6), (3, 7)]:
                    edges += [P8[e0], P8[e1]]
            if tris:
                ov.draw('radar_obj', np.array(tris), (1.0, 0.45, 0.0, 0.45), VP, moderngl.TRIANGLES, blend=True)
                ov.draw('radar_obj_edge', np.array(edges), (1.0, 0.55, 0.1, 1.0), VP, moderngl.LINES)
        if self.show['gnss'] and 'gnss' in S:
            g = S['gnss']
            if g['t'] != self.last_gnss_t:
                self.last_gnss_t = g['t']
                if g['valid']:
                    self.gnss_trail.append((g['xyz'] + [0, 0, 0.1], GNSS_COLOR[g['status']]))
                self.truth_trail.append(g['truth'] + [0, 0, 0.1])
            if self.truth_trail:
                ov.draw('truth', np.array(self.truth_trail), (1, 1, 1, 0.6), VP, size=2.0 * self.scale)
            if self.gnss_trail:
                ov.draw('gnss_trail', np.array([p for p, _ in self.gnss_trail]),
                        np.array([c for _, c in self.gnss_trail]), VP, size=4.0 * self.scale)
            if g['valid']:
                r = max(g['hacc'] * 3, 0.3)
                a = np.linspace(0, 2 * np.pi, 33)
                ring = g['xyz'] + np.c_[r * np.cos(a), r * np.sin(a), np.full_like(a, 0.15)]
                pts = np.r_[np.stack([ring[:-1], ring[1:]], 1).reshape(-1, 3), [g['xyz'], g['xyz'] + [0, 0, 3.0]]]
                ov.draw('gnss', pts, GNSS_COLOR[g['status']], VP, moderngl.LINES)
        if self.show['scc'] and self.show['vehicles']:
            Ts, act = snap['traffic_T'], snap['active']
            side_ok = e.traffic.side == e.scn.side                  # 비콘 쪽(접근) 차량만
            idx = np.nonzero(act & side_ok)[0]
            pts, cols, segs = [], [], []
            bpos = snap['T_wb'][:3, 3] + [0, 0, 0.5]
            for i in idx:
                if snap['crashed'][i]:
                    col = CRASHED
                elif snap['scc_target'][i] == -2:
                    col = SCC_BEACON
                elif snap['noticed'][i] and snap['beh'][i] != B_CRASH:
                    col = NOTICED
                elif snap['scc'][i]:
                    col = SCC_ON
                else:
                    continue
                sp = e.traffic.specs[i]
                pts.append(Ts[i][:3, 3] + Ts[i][:3, 2] * (sp.height + 0.6))
                cols.append(col)
                if snap['scc_target'][i] == -2:                     # 비콘 추종: 앞범퍼 → 비콘
                    front = Ts[i][:3, 3] + Ts[i][:3, 0] * (sp.length / 2) + Ts[i][:3, 2] * 0.8
                    segs += [front, bpos]
            if pts:
                ov.draw('scc_pts', np.array(pts), np.array(cols), VP, size=9.0 * self.scale)
            if segs:
                ov.draw('scc_lines', np.array(segs), SCC_BEACON, VP, moderngl.LINES)

    def frame(self, dt):
        if self.robot_future is not None and self.robot_future.done():
            fut, self.robot_future = self.robot_future, None
            try:
                fut.result()
                self._load_robot()
                self.log('로봇 모델 준비 완료 → 시뮬레이션 시작')
                self.rebuild(map_too=True)
            except Exception as ex:     # noqa: BLE001
                self.log(f'로봇 모델 생성 실패: {ex}')
        if self.build_future is not None and self.build_future.done():
            fut, self.build_future = self.build_future, None
            try:
                self._apply(*fut.result(), cfg=self.build_cfg)
            except Exception as ex:     # noqa: BLE001
                import traceback
                self.log(f'생성 실패: {ex}')
                traceback.print_exc()
        if self.export_future is not None and self.export_future.done():
            fut, self.export_future = self.export_future, None
            try:
                v = fut.result()['validation']
                self.log('맵 검증 ' + ('통과' if v['ok'] else '실패') + f' (경로 하행 {v.get("route_down")}, 상행 {v.get("route_up")})')
            except Exception as ex:     # noqa: BLE001
                self.log(f'저장 실패: {ex}')
        self._keyboard_drive()
        snap = self.engine.snapshot() if self.engine is not None and self.build_future is None else None
        if snap is not None:
            for ev in snap['events']:
                if ev[1] in ('crash', 'scc_lock'):
                    self.log(f'[{ev[0]:.1f}s] 차량 #{ev[2]} {ev[3]}')
                elif ev[1] in ('beacon_contact', 'beacon_damaged'):
                    self.log(f'[{ev[0]:.1f}s] 차량 #{ev[2] if ev[2] >= 0 else "-"} {ev[3]}')
            if self.follow:
                self.cam['target'] = snap['T_wb'][:3, 3] + np.array([0, 0, 0.5])
            self._update_camera_sensor(snap)

        self.input.new_frame(dt)
        imgui.new_frame()
        fw, fh = glfw.get_framebuffer_size(self.win)
        P_, V_, _ = self._view(fw / max(fh, 1))
        if self.robot_parts is None:
            self._robot_setup_modal(*imgui.get_io().display_size)
        else:
            self._ui(snap)
            self._world_labels(snap, P_ @ V_)
        imgui.render()

        (self.target_fbo or self.ctx.screen).use()
        self.ctx.viewport = (0, 0, fw, fh)
        self.ctx.clear(*(self.renderer.sky if self.renderer else (0.06, 0.07, 0.09)), 1.0)
        self.ctx.enable(moderngl.DEPTH_TEST)
        P, V, eye = self._view(fw / max(fh, 1))
        VP = P @ V
        if snap is not None and self.renderer is not None:
            hidden = set()
            in_tun = bool(self.model.in_tunnel(snap['s_b'], margin=30)) and self.show['auto_roof']
            if not self.show['terrain'] or in_tun:
                hidden.add('terrain')
            if not self.show['tunnel'] or in_tun:
                hidden |= {'tunnel', 'portal'}
            veh = list(self.engine.accident)
            if self.show['vehicles']:
                veh += list(zip(self.engine.traffic.specs, snap['traffic_T']))
            self.renderer.draw(VP, eye, snap['T_wb'] if self.show['robot'] else None, hidden=hidden, vehicles=veh)
            self._draw_overlays(snap, VP)
        self.ctx.disable(moderngl.DEPTH_TEST)
        self.gui.render(imgui.get_draw_data())

    def run(self, frames=None, screenshot=None, warmup=0.0):
        last = time.perf_counter()
        n = 0
        if screenshot:
            fw, fh = glfw.get_framebuffer_size(self.win)
            self.target_fbo = self.ctx.framebuffer(color_attachments=[self.ctx.renderbuffer((fw, fh), 3, samples=4)],
                                                   depth_attachment=self.ctx.depth_renderbuffer((fw, fh), samples=4))
            resolve = self.ctx.framebuffer(color_attachments=[self.ctx.renderbuffer((fw, fh), 3)])
        t_start = time.perf_counter()
        while not glfw.window_should_close(self.win):
            glfw.poll_events()
            now = time.perf_counter()
            self.frame(now - last)
            last = now
            n += 1
            if screenshot and frames and n >= frames and now - t_start >= warmup:
                self.ctx.copy_framebuffer(resolve, self.target_fbo)
                img = np.frombuffer(resolve.read(components=3, alignment=1), np.uint8).reshape(fh, fw, 3)
                import cv2
                cv2.imwrite(screenshot, img[::-1, :, ::-1])
                break
            glfw.swap_buffers(self.win)
            if frames and not screenshot and n >= frames:
                break
        self.close()

    def close(self):
        self._save_settings()
        if self.eth is not None:
            self.eth.stop()
        if self.engine is not None:
            self.engine.close()
        if self.ros is not None:
            self.ros.shutdown()
        self.pool.shutdown(wait=False)
        glfw.terminate()


def main():
    ap = argparse.ArgumentParser(description='BeaconSim 세이프티 비콘 시뮬레이터')
    ap.add_argument('--hidden', action='store_true', help='창을 띄우지 않음 (테스트용)')
    ap.add_argument('--no-ros', action='store_true', help='ROS2 연동 끄기')
    ap.add_argument('--frames', type=int, default=None)
    ap.add_argument('--warmup', type=float, default=0.0, help='스크린샷 전 대기 [s]')
    ap.add_argument('--screenshot', default=None)
    ap.add_argument('--set', action='append', default=[], help='테스트용 파라미터 (예: env.weather=rain)')
    ap.add_argument('--robot-stl', default=None, help='비콘 CAD(STL)로 로봇 모델 생성 후 시작')
    a = ap.parse_args()
    if a.robot_stl:
        from .robot_build import build
        build(a.robot_stl, writable_root(), True)
    app = App(hidden=a.hidden, ros=not a.no_ros)
    if a.set:
        if any(kv.startswith('scn.seed=') for kv in a.set):
            app.rand_scn = False
        for kv in a.set:
            k, v = kv.split('=', 1)
            grp, name = k.split('.')
            obj = {'map': app.params, 'scn': app.sparams, 'traffic': app.tparams, 'env': app.eparams}[grp]
            cur = getattr(obj, name)
            setattr(obj, name, (v.lower() in ('1', 'true', 'yes')) if isinstance(cur, bool) else type(cur)(v))
        app.rebuild(map_too=True, sync=True)
    app.run(a.frames, a.screenshot, a.warmup)


if __name__ == '__main__':
    main()
