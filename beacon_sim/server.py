"""BeaconSim 백엔드 + Godot 프런트엔드 (B안).

  python -m beacon_sim.server            → 백엔드 실행 + Godot 클라이언트 창
백엔드(이 프로세스): 도로·시나리오·교통·RGL 센서·GNSS·ROS·평가. 화면과 UI 는 Godot 이 맡는다.

프로토콜 (TCP, 127.0.0.1): 프레임 = <u32 json 길이><u32 바이너리 길이><json><바이너리>
  json['arrays'] = {이름: [오프셋, 바이트수, dtype, shape]} → 바이너리 안의 numpy 배열 (리틀 엔디언)
  백엔드 → Godot: hello, world(장면 다시 만들기), state(60 Hz), pc(센서 점군), log, cam_req
  Godot → 백엔드: cmd(pause/rtf/restart/regen_map/drive/quit), cam_done
좌표: 모두 Godot 좌표(x=X, y=Z, z=-Y, 맵 시작점 원점)로 바꿔서 보낸다.
"""
import argparse
import collections
import copy
import json
import os
import select
import signal
import socket
import struct
import subprocess
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import fields

import numpy as np

from . import __version__, scene
from .common import (CACHE_DIR, ROOT, T_CAMLINK_OPTICAL, has_robot_model, load_yaml, sensor_mounts)
from .engine import EngineThread, SimEngine
from .environment import EnvParams
from .godot_link import C, CameraModel, export_scene, godot_bin, project_dir, to_g, xform_g
from .roadgen import RoadParams, generate
from .ros_link import RosLink
from .traffic import B_CRASH, ScenarioParams, TrafficParams
from .viz import GNSS_COLOR, LABEL_COLORS, intensity_colors, lanelet_overlay

SETTINGS = os.path.join(os.path.expanduser('~'), '.config', 'beaconsim', 'app.json')


# ---------------------------------------------------------------- 프레이밍
def pack(msg, arrays=None):
    bin_ = bytearray()
    if arrays:
        desc = {}
        for k, a in arrays.items():
            a = np.ascontiguousarray(a)
            while len(bin_) % 4:
                bin_.append(0)
            desc[k] = [len(bin_), a.nbytes, a.dtype.str, list(a.shape)]
            bin_ += a.tobytes()
        msg = dict(msg, arrays=desc)
    js = json.dumps(msg, separators=(',', ':'), default=float).encode()
    return struct.pack('<II', len(js), len(bin_)) + js + bytes(bin_)


def xforms_g(Ts, origin):
    """(N,4,4) map 변환 → (N,12) Godot 변환 (행 우선 3x4)."""
    R = np.einsum('ij,njk,kl->nil', C, Ts[:, :3, :3], C.T)
    o = (Ts[:, :3, 3] - origin) @ C.T
    return np.concatenate([R, o[:, :, None]], 2).reshape(len(Ts), 12).astype('f4')


def _pick(cls, d):
    names = {f.name for f in fields(cls)}
    return cls(**{k: v for k, v in d.items() if k in names})


class Backend:
    """UI 와 무관한 시뮬레이션 제어 (설정, 맵·시나리오 생성, 엔진 스레드, ROS)."""

    def __init__(self, ros=True, log=None):
        self.log_lines = collections.deque(maxlen=400)
        self.log_lock = threading.Lock()
        self.base_cfg = load_yaml('config/sim.yaml')
        self.robot_cfg = load_yaml('config/robot.yaml') if has_robot_model() else None
        c = self.base_cfg
        self.params = _pick(RoadParams, c.get('map', {}))
        self.sparams = _pick(ScenarioParams, c.get('scenario', {}))
        self.tparams = _pick(TrafficParams, c.get('traffic', {}))
        self.eparams = _pick(EnvParams, c.get('environment', {}))
        self.rand_map, self.rand_scn = False, True
        self._load_settings()
        if self.params.seed < 0:
            self.params.seed = 1
        self.rtf, self.paused = 1.0, False
        self.model = self.parts = self.engine = self.eth = None
        self.pool = ThreadPoolExecutor(max_workers=2)
        self.build_future = None
        self.world_seq = 0
        self.cam = CameraModel(self.base_cfg['camera'])
        self.ros = RosLink(self.log) if ros else None
        if self.robot_cfg is not None:
            _, T_bc = sensor_mounts(self.robot_cfg)['camera']
            self.T_b_opt = T_bc @ T_CAMLINK_OPTICAL

    # ---- 설정 (ImGui 앱과 같은 파일)
    def _load_settings(self):
        try:
            with open(SETTINGS) as f:
                d = json.load(f)
        except Exception:     # noqa: BLE001
            return
        for key, cls, attr in [('params', RoadParams, 'params'), ('scenario', ScenarioParams, 'sparams'),
                               ('traffic', TrafficParams, 'tparams'), ('env', EnvParams, 'eparams')]:
            if key in d:
                setattr(self, attr, _pick(cls, d[key]))
        self.rand_map, self.rand_scn = d.get('rand_map', False), d.get('rand_scn', True)

    def save_settings(self):
        try:
            with open(SETTINGS) as f:
                d = json.load(f)
        except Exception:     # noqa: BLE001
            d = {}
        d.update(params=self.params.to_dict(), scenario=self.sparams.to_dict(), traffic=self.tparams.to_dict(),
                 env=self.eparams.to_dict(), rand_map=self.rand_map, rand_scn=self.rand_scn)
        os.makedirs(os.path.dirname(SETTINGS), exist_ok=True)
        with open(SETTINGS, 'w') as f:
            json.dump(d, f, indent=1)

    def log(self, msg):
        with self.log_lock:
            self.log_lines.append(time.strftime('%H:%M:%S ') + str(msg))

    # ---- 생성
    def _make_cfg(self):
        cfg = copy.deepcopy(self.base_cfg)
        cfg.update(map=self.params.to_dict(), scenario=self.sparams.to_dict(), traffic=self.tparams.to_dict(),
                   environment=self.eparams.to_dict())
        return cfg

    def _build_job(self, cfg, map_too, model, parts):
        if map_too:
            model = generate(RoadParams(**cfg['map']))
            parts = scene.build_static(model)
        eng = SimEngine(cfg, self.robot_cfg, model=model, parts=parts, log=self.log)
        out = os.path.join(CACHE_DIR, 'godot_scene', f'{os.getpid()}_{self.world_seq + 1}')
        _, js, origin = export_scene(eng, out, cfg['camera'], self.cam.Wr, self.cam.Hr, self.cam.K[1, 1])
        lane = lanelet_overlay(model) if map_too or self.model is None else None
        return model, parts, eng, cfg, js, origin, lane

    def rebuild(self, map_too=False):
        if self.build_future is not None and not self.build_future.done():
            return
        rng = np.random.default_rng()
        if map_too and self.rand_map:
            self.params.seed = int(rng.integers(0, 2 ** 31 - 1))
        if self.rand_scn:
            self.sparams.seed = int(rng.integers(0, 2 ** 31 - 1))
        self.save_settings()
        if self.eth is not None:
            self.eth.stop()
            self.eth = None
        cfg = self._make_cfg()
        self.log(('맵·' if map_too else '') + f'시나리오 생성 중... (맵 시드 {self.params.seed}, 시나리오 시드 {self.sparams.seed})')
        self.build_future = self.pool.submit(self._build_job, cfg, map_too or self.model is None, self.model, self.parts)

    def poll_build(self):
        """생성 완료 → 엔진 시작, (js, origin, lane, 맵 정보) 반환."""
        f = self.build_future
        if f is None or not f.done():
            return None
        self.build_future = None
        try:
            model, parts, eng, cfg, js, origin, lane = f.result()
        except Exception as ex:   # noqa: BLE001
            import traceback
            traceback.print_exc()
            self.log(f'생성 실패: {ex}')
            return None
        if self.engine is not None and self.engine is not eng:
            self.engine.close()
        self.model, self.parts, self.engine, self.origin = model, parts, eng, origin
        self.world_seq += 1
        on_step = self.ros.attach(eng, cfg) if self.ros is not None else None
        self.eth = EngineThread(eng, self.rtf, on_step)
        self.eth.paused = self.paused
        self.eth.start()
        sm, ms = eng.scn.summary(), model.summary()
        self.log(f'맵 시드 {ms["seed"]} ({ms["length_m"]:.0f} m, {ms["lanes"]}차로, 터널 {len(ms["tunnels"])}) | '
                 f'시나리오 시드 {eng.scn.sp.seed}: {sm["side"]} {sm["lane"]}차로, 사고 s={sm["accident_s"]:.0f} m | '
                 f'환경 {eng.env.label()} | 교통 {"켜짐" if eng.traffic.tp.enabled else "꺼짐(검증 모드)"}')
        return js, origin, lane

    def set_pause(self, p):
        self.paused = bool(p)
        if self.eth is not None:
            self.eth.paused = self.paused

    def set_rtf(self, r):
        self.rtf = float(r)
        if self.eth is not None:
            self.eth.set_rtf(self.rtf)

    def close(self):
        if self.eth is not None:
            self.eth.stop()
        if self.engine is not None:
            self.engine.close()
        if self.ros is not None:
            self.ros.shutdown()
        self.pool.shutdown(wait=False)


class Server:
    """Godot 클라이언트와의 연결: 상태·센서 송신, 명령 수신."""

    STATE_HZ = 60.0

    def __init__(self, be: Backend, window=(1700, 1000), extra=()):
        self.be = be
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('127.0.0.1', 0))
        self.sock.listen(1)
        self.port = self.sock.getsockname()[1]
        self.conn = None
        self.rx = bytearray()
        self.running = True
        self.sent_log = 0
        self.last_pc_t = {}
        self.cam_seq, self.cam_next_t, self.cam_pending = 0, 0.0, None
        shm = '/dev/shm' if os.path.isdir('/dev/shm') else CACHE_DIR
        self.cam_path = os.path.join(shm, f'beaconsim_view_{os.getpid()}.rgb')
        cmd = [godot_bin(), '--path', project_dir(), '--resolution', f'{window[0]}x{window[1]}', '--',
               '--client', f'--port={self.port}', *extra]
        self.proc = subprocess.Popen(cmd)

    # ---- 송수신
    def send(self, msg, arrays=None):
        if self.conn is None:
            return
        try:
            self.conn.sendall(pack(msg, arrays))
        except (BrokenPipeError, ConnectionResetError, OSError):
            self.running = False

    def _recv(self):
        while True:
            r, _, _ = select.select([self.conn], [], [], 0)
            if not r:
                break
            try:
                data = self.conn.recv(1 << 20)
            except (ConnectionResetError, OSError):
                data = b''
            if not data:
                self.running = False
                return
            self.rx += data
        while len(self.rx) >= 8:
            jl, bl = struct.unpack_from('<II', self.rx)
            if len(self.rx) < 8 + jl + bl:
                break
            msg = json.loads(self.rx[8:8 + jl].decode())
            del self.rx[:8 + jl + bl]
            self.handle(msg)

    def handle(self, m):
        be, t = self.be, m.get('type')
        if t == 'quit':
            self.running = False
        elif t == 'pause':
            be.set_pause(m['value'])
        elif t == 'rtf':
            be.set_rtf(m['value'])
        elif t == 'restart':
            be.rebuild(map_too=bool(m.get('map', False)))
        elif t == 'drive' and be.engine is not None:
            be.engine.set_cmd('key', float(m['v']), float(m['w']))
        elif t == 'cam_done':
            self._on_cam_done(m)

    # ---- 월드 / 상태
    def send_world(self, js, origin, lane):
        be = self.be
        m, scn = be.model, be.engine.scn
        P = np.c_[m.x, m.y, m.z]
        lo, hi = to_g(P.min(0), origin), to_g(P.max(0), origin)
        acc = to_g(m.point(scn.accident_s, scn.lane_t), origin)
        tun = [to_g(m.point(a - 60, 0.0), origin).tolist() for a, _ in m.tunnels]
        arrays = {}
        if lane is not None:
            arrays['lane_pts'] = (to_g(lane[0], origin)).astype('f4')
            arrays['lane_cols'] = lane[1].astype('f4')
        specs = be.engine.traffic.specs
        self.send({'type': 'world', 'seq': be.world_seq, 'scene': js, 'bounds': [np.minimum(lo, hi).tolist(),
                   np.maximum(lo, hi).tolist()], 'accident': acc.tolist(), 'tunnels': tun,
                   'veh_dims': [[sp.length, sp.width, sp.height] for sp in specs],
                   'env_label': be.engine.env.label(), 'map': m.summary(), 'scenario': scn.summary()}, arrays)

    def send_state(self):
        be = self.be
        e = be.engine
        snap = e.snapshot()
        o = be.origin
        act = np.nonzero(snap['active'])[0]
        tr = e.traffic
        flags = (snap['crashed'][act].astype(np.uint8) |
                 ((snap['scc_target'][act] == -2).astype(np.uint8) << 1) |
                 ((snap['noticed'][act] & (snap['beh'][act] != B_CRASH)).astype(np.uint8) << 2) |
                 (snap['scc'][act].astype(np.uint8) << 3) |
                 ((tr.side[act] == e.scn.side).astype(np.uint8) << 4))
        g = snap['sensors'].get('gnss')
        gj = None
        if g is not None:
            gj = {'t': g['t'], 'status': int(g['status']), 'name': g['status_name'], 'hacc': float(g['hacc']),
                  'valid': bool(g['valid']), 'xyz': to_g(g['xyz'], o).tolist(), 'truth': to_g(g['truth'], o).tolist(),
                  'color': list(GNSS_COLOR[g['status']])}
        T_wb = snap['T_wb']
        msg = {'type': 'state', 'world': be.world_seq, 't': snap['t'], 'paused': be.paused, 'rtf': be.rtf,
               'rtf_meas': be.eth.measured_rtf if be.eth else 0.0, 'v': snap['v'], 'cmd_src': snap['cmd_src'],
               'damaged': bool(snap['damaged']), 'contact': snap['contact'],
               'acc_dist': abs(snap['s_b'] - e.scn.accident_s), 'counts': snap['counts'],
               'n_active': int(len(act)), 'scc_follow': int((snap['scc_target'] == -2).sum()),
               'in_tunnel': bool(e.model.in_tunnel(snap['s_b'], margin=30)),
               'robot': xform_g(T_wb, o), 'cam': xform_g(T_wb @ be.T_b_opt, o, cam=True), 'gnss': gj,
               'ros': bool(be.ros is not None and be.ros.ok)}
        self.send(msg, {'veh_idx': act.astype('<i4'), 'veh_T': xforms_g(snap['traffic_T'][act], o),
                        'veh_flags': flags.astype(np.uint8)})
        for ev in snap['events']:
            if ev[1] in ('crash', 'scc_lock', 'beacon_contact', 'beacon_damaged'):
                be.log(f'[{ev[0]:.1f}s] 차량 #{ev[2] if ev[2] >= 0 else "-"} {ev[3]}')
        self._send_sensors(snap)
        self._camera(snap)

    def _send_sensors(self, snap):
        o, S = self.be.origin, snap['sensors']
        for key in ('pandar40p', 'tele15'):
            s = S.get(key)
            if s is None or self.last_pc_t.get(key) == s['t']:
                continue
            self.last_pc_t[key] = s['t']
            p = s['points']
            xyz = np.c_[p['x'], p['y'], p['z']] @ s['T_ws'][:3, :3].T + s['T_ws'][:3, 3]
            if key == 'pandar40p':
                col = intensity_colors(p['intensity'])
                lab = np.clip(s['gt']['label'], 0, len(LABEL_COLORS) - 1)
                col2 = np.c_[LABEL_COLORS[lab], np.ones(len(lab), np.float32)]
                self.send({'type': 'pc', 'name': 'pandar'}, {'xyz': to_g(xyz, o).astype('f4'),
                                                             'col': col.astype('f4'), 'col2': col2.astype('f4')})
            else:
                t = np.clip(p['intensity'] / 60.0, 0, 1)[:, None]
                col = np.c_[np.full_like(t, 1.0), 0.45 + 0.5 * t, 0.1 + 0.3 * t, np.ones_like(t)]
                self.send({'type': 'pc', 'name': 'tele'}, {'xyz': to_g(xyz, o).astype('f4'), 'col': col.astype('f4')})
        r = S.get('ars408')
        if r is not None and self.last_pc_t.get('ars408') != r['t']:
            self.last_pc_t['ars408'] = r['t']
            self.send({'type': 'radar'}, self._radar_arrays(r))

    def _radar_arrays(self, r):
        """클러스터 점 + 감지 객체 앞면 판 (두께 0.6 m × 차폭 × 차고) → Godot 상자 변환."""
        be = self.be
        m, o = be.model, be.origin
        c, T = r['clusters'], r['T_ws']
        xyz = np.c_[c['x'], c['y'], np.zeros(len(c))] @ T[:3, :3].T + T[:3, 3] if len(c) else np.zeros((0, 3))
        ex, ey = T[:3, 0].copy(), T[:3, 1].copy()
        ex[2] = ey[2] = 0.0
        ex /= max(np.linalg.norm(ex), 1e-9)
        ey /= max(np.linalg.norm(ey), 1e-9)
        boxes = []
        for tk in r['tracks']:
            sp = tk['spec']
            cen = T[:3, :3] @ np.array([tk['x'], tk['y'], 0.0]) + T[:3, 3]
            s_, t_ = m.project(cen[0], cen[1])
            z0 = float(m.surface_z(s_, np.clip(t_, -m.paved_t, m.paved_t)))
            Tb = np.eye(4)
            Tb[:3, 0], Tb[:3, 1], Tb[:3, 2] = ex * 0.6, ey * sp.width, [0, 0, sp.height]
            Tb[:3, 3] = [cen[0], cen[1], z0 + sp.height / 2]
            boxes.append(Tb)
        bx = xforms_g(np.array(boxes), o) if boxes else np.zeros((0, 12), 'f4')
        return {'xyz': to_g(xyz, o).astype('f4'), 'boxes': bx}

    # ---- ROS 카메라: Godot 이 같은 월드에서 그린 영상을 받아 렌즈 왜곡 후 발행
    def _camera(self, snap):
        ros = self.be.ros
        if ros is None or not ros.ok:
            return
        if self.cam_pending is not None and time.time() - self.cam_pending[1] > 2.0:
            self.cam_pending = None
        if self.cam_pending is None and snap['t'] >= self.cam_next_t:
            self.cam_seq += 1
            self.cam_pending = (self.cam_seq, time.time(), snap['t'])
            self.cam_next_t = snap['t'] + 1.0 / self.be.base_cfg['camera']['rate_hz']
            self.send({'type': 'cam_req', 'seq': self.cam_seq, 'path': self.cam_path})

    def _on_cam_done(self, m):
        if self.cam_pending is None or m.get('seq') != self.cam_pending[0]:
            return
        t = self.cam_pending[2]
        self.cam_pending = None
        w, h = int(m['w']), int(m['h'])
        raw = np.fromfile(self.cam_path, np.uint8)
        if raw.size != w * h * 3 or (h, w) != (self.be.cam.Hr, self.be.cam.Wr):
            return
        img = self.be.cam.distort(raw.reshape(h, w, 3))
        self.be.ros.publish_camera(img, t, self.be.cam, self.be.base_cfg['camera']['optical_frame'])

    def send_logs(self):
        with self.be.log_lock:
            lines = list(self.be.log_lines)
        new = lines[self.sent_log:] if self.sent_log <= len(lines) else lines
        if new:
            self.send({'type': 'log', 'lines': new})
        self.sent_log = len(lines)

    # ---- 메인 루프
    def run(self):
        self.sock.settimeout(60)
        try:
            self.conn, _ = self.sock.accept()
        except socket.timeout:
            raise RuntimeError('Godot 클라이언트가 연결하지 않음')
        self.conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        fonts = {w: os.path.join(ROOT, 'assets', 'fonts', f'Pretendard-{w}.otf') for w in ('Medium', 'SemiBold', 'Bold')}
        try:
            from .ui_style import _ui_font
            fonts = {w: _ui_font(w) for w in fonts}               # Pretendard PUA 영역 제거본 (아이콘 충돌 방지)
        except Exception:     # noqa: BLE001
            pass
        self.send({'type': 'hello', 'version': __version__, 'fonts': fonts,
                   'icons': os.path.join(ROOT, 'assets', 'fonts', 'Font_Awesome_6_Free-Solid-900.otf'),
                   'camera': {'width': self.be.cam.W, 'height': self.be.cam.H, 'hfov': self.be.cam.hfov,
                              'wr': self.be.cam.Wr, 'hr': self.be.cam.Hr,
                              'vfov': float(np.degrees(2 * np.arctan(self.be.cam.Hr / 2.0 / self.be.cam.K[1, 1]))),
                              'rate_hz': self.be.base_cfg['camera']['rate_hz']}})
        self.be.rebuild(map_too=True)
        period = 1.0 / self.STATE_HZ
        nxt = time.perf_counter()
        while self.running and self.proc.poll() is None:
            self._recv()
            w = self.be.poll_build()
            if w is not None:
                self.last_pc_t = {}
                self.send_world(*w)
            if self.be.engine is not None and self.be.build_future is None:
                self.send_state()
            self.send_logs()
            nxt += period
            time.sleep(max(0.0, nxt - time.perf_counter()))
            if time.perf_counter() - nxt > 0.5:
                nxt = time.perf_counter()

    def close(self):
        self.running = False
        try:
            if self.conn is not None:
                self.conn.sendall(pack({'type': 'bye'}))
        except OSError:
            pass
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.proc.kill()
        for s in (self.conn, self.sock):
            try:
                s.close()
            except Exception:  # noqa: BLE001
                pass
        try:
            os.remove(self.cam_path)
        except OSError:
            pass


def main():
    ap = argparse.ArgumentParser(description='BeaconSim (Godot 프런트엔드)')
    ap.add_argument('--no-ros', action='store_true')
    ap.add_argument('--size', default='1700x1000')
    ap.add_argument('--set', action='append', default=[], help='예: env.weather=rain')
    ap.add_argument('--shot', default=None, help='테스트용: 장면 로드 후 --shot-delay 초 뒤 화면 저장하고 종료')
    ap.add_argument('--shot-delay', type=float, default=8.0)
    a = ap.parse_args()
    be = Backend(ros=not a.no_ros)
    if be.robot_cfg is None:
        raise SystemExit('로봇 모델 없음: 먼저 기존 앱(run_app.sh)이나 build-robot 으로 생성하세요 (Godot UI 의 STL 선택은 3단계)')
    for kv in a.set:
        k, v = kv.split('=', 1)
        grp, name = k.split('.')
        obj = {'map': be.params, 'scn': be.sparams, 'traffic': be.tparams, 'env': be.eparams}[grp]
        cur = getattr(obj, name)
        setattr(obj, name, (v.lower() in ('1', 'true', 'yes')) if isinstance(cur, bool) else type(cur)(v))
        if grp == 'scn' and name == 'seed':
            be.rand_scn = False
    w, h = (int(x) for x in a.size.lower().split('x'))
    extra = [f'--shot={os.path.abspath(a.shot)}', f'--shot-delay={a.shot_delay}'] if a.shot else []
    srv = Server(be, (w, h), extra)

    def stop(*_):
        srv.running = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        srv.run()
    finally:
        srv.close()
        be.close()


if __name__ == '__main__':
    main()
