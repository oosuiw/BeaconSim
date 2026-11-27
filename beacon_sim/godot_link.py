"""Godot 렌더러 연동 (카메라 시제품).

파이썬 엔진(도로·교통·시나리오)은 그대로 두고, 카메라 영상만 Godot 4 (Forward+)로 그린다.
  1) export_scene(): 정적 장면 + 차량 원형 + 비콘 부품 → scene.glb, 광원·카메라·환경 → scene.json
  2) GodotLink: Godot 프로세스 실행, 프레임마다 상태(카메라·비콘·차량 포즈)를 UDP JSON 으로 보내고
     캡처 요청 시 Godot 이 PNG 를 쓰면 응답을 받는다.
좌표: map(X 동, Y 북, Z 위) → glTF/Godot(x=X, y=Z, z=-Y), 원점은 맵 시작점으로 옮겨 float 정밀도 확보.
"""
import json
import os
import socket
import struct
import subprocess
import time

import numpy as np

from . import scene
from .common import ROBOT_PART_COLORS, ROOT, T_CAMLINK_OPTICAL, load_robot_parts, sensor_mounts

C = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float)     # map → godot 기저
D_CAM = np.diag([1.0, -1.0, -1.0])                           # Godot 카메라 로컬 → 광학 프레임
GODOT_DIR = os.path.join(ROOT, 'godot')
SMOOTH = {'road', 'terrain', 'lane_white', 'lane_yellow'}      # 위를 향하는 면: 방향 통일 + 부드러운 법선


def godot_bin():
    from .common import USER_DIR
    for p in (os.environ.get('BEACONSIM_GODOT', ''), os.path.join(ROOT, 'third_party', 'godot'),
              os.path.join(USER_DIR, 'godot')):
        if os.path.isfile(p) and os.access(p, os.X_OK):
            return p
        if os.path.isdir(p):
            for f in sorted(os.listdir(p), reverse=True):
                if f.startswith('Godot_v4') and f.endswith('x86_64'):
                    return os.path.join(p, f)
    raise FileNotFoundError('Godot 4 실행 파일 없음: third_party/godot/ 또는 ~/.config/beaconsim/godot/ 에 '
                            'Godot_v4.x_linux.x86_64 를 두거나 BEACONSIM_GODOT 환경변수로 지정')


# ---------------------------------------------------------------- 좌표 변환
def to_g(p, origin):
    return (np.asarray(p, float) - origin) @ C.T


def xform_g(T, origin, cam=False):
    """map 4x4 → Godot Transform3D (행 우선 3x4 → 12개 실수)."""
    R = C @ T[:3, :3] @ (D_CAM if cam else np.eye(3)) @ (np.eye(3) if cam else C.T)
    o = C @ (T[:3, 3] - origin)
    return [round(float(x), 5) for x in np.c_[R, o].ravel()]


# ---------------------------------------------------------------- glb 쓰기 (외부 라이브러리 없이)
def _normals(v, f, smooth):
    if not smooth:                              # 면마다 정점 복제 → 평면 셰이딩
        tri = v[f]
        n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
        return tri.reshape(-1, 3), np.repeat(n, 3, 0), np.arange(len(f) * 3, dtype=np.uint32).reshape(-1, 3)
    f = f.copy()
    fn = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    down = fn[:, 1] < 0                          # Godot y = 위. 아래를 향한 면은 감는 방향 뒤집기
    f[down] = f[down][:, ::-1]
    fn[down] *= -1
    n = np.zeros_like(v)
    for k in range(3):
        np.add.at(n, f[:, k], fn)
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    return v, n, f.astype(np.uint32)


class Glb:
    def __init__(self):
        self.bin = bytearray()
        self.views, self.acc, self.meshes, self.nodes, self.mats = [], [], [], [], []
        self.mat_idx = {}

    def _view(self, arr, target):
        while len(self.bin) % 4:
            self.bin.append(0)
        off = len(self.bin)
        b = arr.tobytes()
        self.bin += b
        self.views.append({'buffer': 0, 'byteOffset': off, 'byteLength': len(b), 'target': target})
        return len(self.views) - 1

    def material(self, name, color):
        if name not in self.mat_idx:
            self.mat_idx[name] = len(self.mats)
            self.mats.append({'name': name, 'pbrMetallicRoughness': {
                'baseColorFactor': [float(c) ** 2.2 for c in color[:3]] + [1.0],    # glTF 는 선형 색공간
                'metallicFactor': 0.0, 'roughnessFactor': 0.8}, 'doubleSided': True})
        return self.mat_idx[name]

    def mesh(self, name, v, f, mat_name, color, smooth=False):
        """v: Godot 좌표 정점. → mesh index"""
        v, n, idx = _normals(np.asarray(v, np.float32), np.asarray(f, np.int64), smooth)
        v, n = v.astype(np.float32), n.astype(np.float32)
        a0 = len(self.acc)
        self.acc.append({'bufferView': self._view(v, 34962), 'componentType': 5126, 'count': len(v), 'type': 'VEC3',
                         'min': v.min(0).tolist(), 'max': v.max(0).tolist()})
        self.acc.append({'bufferView': self._view(n, 34962), 'componentType': 5126, 'count': len(n), 'type': 'VEC3'})
        self.acc.append({'bufferView': self._view(idx.ravel().astype(np.uint32), 34963), 'componentType': 5125,
                         'count': int(idx.size), 'type': 'SCALAR'})
        self.meshes.append({'name': name, 'primitives': [{'attributes': {'POSITION': a0, 'NORMAL': a0 + 1},
                                                           'indices': a0 + 2, 'material': self.material(mat_name, color)}]})
        return len(self.meshes) - 1

    def node(self, name, mesh=None, children=None):
        nd = {'name': name}
        if mesh is not None:
            nd['mesh'] = mesh
        if children:
            nd['children'] = children
        self.nodes.append(nd)
        return len(self.nodes) - 1

    def write(self, path, roots):
        while len(self.bin) % 4:
            self.bin.append(0)
        doc = {'asset': {'version': '2.0', 'generator': 'BeaconSim'}, 'scene': 0, 'scenes': [{'nodes': roots}],
               'nodes': self.nodes, 'meshes': self.meshes, 'materials': self.mats, 'accessors': self.acc,
               'bufferViews': self.views, 'buffers': [{'byteLength': len(self.bin)}]}
        js = json.dumps(doc, separators=(',', ':')).encode()
        js += b' ' * ((4 - len(js) % 4) % 4)
        with open(path, 'wb') as fh:
            fh.write(struct.pack('<III', 0x46546C67, 2, 12 + 8 + len(js) + 8 + len(self.bin)))
            fh.write(struct.pack('<II', len(js), 0x4E4F534A) + js)
            fh.write(struct.pack('<II', len(self.bin), 0x004E4942) + bytes(self.bin))


def _vkey(spec):
    return f'{spec.kind}_{spec.length:.2f}_{spec.width:.2f}_{spec.height:.2f}'


def export_scene(eng, out_dir, cam_cfg, Wr, Hr, fy):
    """엔진의 현재 맵·시나리오 → out_dir/scene.glb, scene.json. 반환: (glb, json 경로, origin)."""
    os.makedirs(out_dir, exist_ok=True)
    m = eng.model
    origin = np.array(m.point(0.0, 0.0), float)
    g = Glb()
    roots = []
    # 정적 장면: 클래스별 합치기
    by_cls = {}
    for p in eng.parts:
        by_cls.setdefault(p.cls, []).append(p)
    for cls, ps in by_cls.items():
        vs, fs, n0 = [], [], 0
        for p in ps:
            vs.append(to_g(p.v, origin))
            fs.append(p.f + n0)
            n0 += len(p.v)
        mi = g.mesh(cls, np.concatenate(vs), np.concatenate(fs), cls, ps[0].color, smooth=cls in SMOOTH)
        roots.append(g.node(f'static_{cls}', mi))
    # 차량 원형 (차체 + 전조등 + 후미등), 로컬 좌표 → Godot 기저
    protos, proto_idx = [], {}
    specs = list(eng.traffic.specs) + [sp for sp, _ in eng.accident]
    for sp in specs:
        k = _vkey(sp)
        if k in proto_idx:
            continue
        proto_idx[k] = len(protos)
        v, f = scene.vehicle_mesh(sp.kind, sp.length, sp.width, sp.height)
        body = g.mesh(f'veh_{proto_idx[k]}', v @ C.T, f, 'vehicle', (0.6, 0.6, 0.6))
        kids = []
        for j, (lv, lf, col) in enumerate(scene.vehicle_lamps(sp.kind, sp.length, sp.width, sp.height)):
            kind = 'head' if col[2] > 0.5 else 'tail'
            kids.append(g.node(f'lamp_{kind}_{j}', g.mesh(f'lamp_{proto_idx[k]}_{j}', lv @ C.T, lf, f'lamp_{kind}', col)))
        roots.append(g.node(f'proto_{proto_idx[k]}', body, kids))
        protos.append({'key': k, 'kind': sp.kind, 'length': sp.length, 'width': sp.width, 'height': sp.height})
    # 비콘 부품 (base_link 기준, 카메라 하우징 제외)
    rk = []
    for name, (v, f) in load_robot_parts().items():
        if name == 'camera':
            continue
        rk.append(g.node(f'robot_{name}', g.mesh(f'robot_{name}', np.asarray(v) @ C.T, f, f'robot_{name}',
                                                 ROBOT_PART_COLORS.get(name, (0.6, 0.6, 0.6)))))
    roots.append(g.node('robot', None, rk))
    glb = os.path.join(out_dir, 'scene.glb')
    g.write(glb, roots)
    # 광원: 가로등(야간), 터널등(항상)
    street = [to_g(p, origin).round(3).tolist() for p in scene.street_lights(m)]
    tunnel = []
    for a, b in m.tunnels:
        for s in np.arange(a + 6.0, b - 3.0, 12.0):
            for t in (-m.edge_t * 0.55, m.edge_t * 0.55):
                tunnel.append(to_g(m.point(s, t, scene.TUNNEL_WALL_H + 1.5), origin).round(3).tolist())
    env = eng.env
    info = {
        'origin': origin.tolist(),
        'camera': {'width': Wr, 'height': Hr, 'vfov_deg': float(np.degrees(2 * np.arctan(Hr / 2.0 / fy))),
                   'near': cam_cfg['near'], 'far': cam_cfg['far']},
        'protos': protos,
        'slots': [proto_idx[_vkey(sp)] for sp in eng.traffic.specs],
        'slot_colors': [list(map(float, sp.color)) for sp in eng.traffic.specs],
        'accident': [{'proto': proto_idx[_vkey(sp)], 'color': list(map(float, sp.color)), 'T': xform_g(T, origin)}
                     for sp, T in eng.accident],
        'street_lights': street, 'tunnel_lights': tunnel,
        'env': {'time_of_day': env.time_of_day, 'weather': env.weather, 'intensity': env.intensity,
                'visibility': env.visibility, 'fog_dist': env.fog_dist, 'wet': env.wet,
                'street_lights': env.street_lights, 'lamps': env.lamps},
    }
    js = os.path.join(out_dir, 'scene.json')
    with open(js, 'w') as fh:
        json.dump(info, fh)
    return glb, js, origin


def available():
    try:
        godot_bin()
        return True
    except FileNotFoundError:
        return False


def project_dir():
    """Godot 은 프로젝트 폴더에 .godot/ 캐시를 쓰므로, 읽기 전용(AppImage)이면 캐시 폴더로 복사해서 쓴다."""
    if os.access(GODOT_DIR, os.W_OK):
        return GODOT_DIR
    import shutil
    from .common import CACHE_DIR
    dst = os.path.join(CACHE_DIR, 'godot_project')
    os.makedirs(dst, exist_ok=True)
    for f in os.listdir(GODOT_DIR):
        if f.endswith(('.godot', '.tscn', '.gd')):
            shutil.copy2(os.path.join(GODOT_DIR, f), os.path.join(dst, f))
    return dst


def _free_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('127.0.0.1', 0))
    p = s.getsockname()[1]
    s.close()
    return p


class GodotLink:
    """Godot 렌더러 프로세스 + UDP 상태 송신 / 캡처 응답 수신.

    hidden=True: GUI 연동용 (창 최소화). request()/poll() 은 기다리지 않는 스트리밍 캡처 (공유 메모리 원시 RGB).
    """

    def __init__(self, scene_json, origin, T_b_opt, port=None, window=(1280, 820), hidden=False, log=print):
        self.origin, self.T_b_opt, self.log = origin, T_b_opt, log
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.bind(('127.0.0.1', 0))
        port = port or _free_port()
        self.addr = ('127.0.0.1', port)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0
        self.pending = None
        shm = '/dev/shm' if os.path.isdir('/dev/shm') else os.path.dirname(scene_json)
        self.raw_path = os.path.join(shm, f'beaconsim_cam_{os.getpid()}_{port}.rgb')
        cmd = [godot_bin(), '--path', project_dir(), '--resolution', f'{window[0]}x{window[1]}', '--',
               f'--scene={scene_json}', f'--port={port}', f'--reply={self.rx.getsockname()[1]}']
        if hidden:
            cmd.append('--hidden')
        self.proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL if hidden else subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True)
        self._wait('ready', 60)

    def _wait(self, what, timeout):
        self.rx.settimeout(timeout)
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.proc.poll() is not None:
                out = self.proc.stdout.read()[-3000:] if self.proc.stdout else ''
                raise RuntimeError('Godot 종료됨:\n' + out)
            try:
                msg = json.loads(self.rx.recv(65536).decode())
            except socket.timeout:
                break
            if msg.get('type') == what:
                return msg
        raise TimeoutError(f'Godot 응답 없음 ({what})')

    def _state(self, eng_t, T_wb, traffic_T, active):
        self.seq += 1
        veh = [[int(i)] + xform_g(traffic_T[i], self.origin) for i in np.nonzero(active)[0]]
        return {'type': 'state', 'seq': self.seq, 't': round(float(eng_t), 3),
                'cam': xform_g(T_wb @ self.T_b_opt, self.origin, cam=True),
                'robot': xform_g(T_wb, self.origin), 'veh': veh}

    def _send(self, msg):
        data = json.dumps(msg, separators=(',', ':')).encode()
        if len(data) > 65000:
            raise ValueError('상태 패킷이 UDP 한도 초과')
        self.tx.sendto(data, self.addr)

    def send_state(self, eng, T_wb, traffic_T, active, capture=None):
        """비교 도구용: capture 경로가 있으면 PNG 저장까지 기다린다."""
        msg = self._state(eng.t, T_wb, traffic_T, active)
        if capture:
            msg['capture'] = capture
        self._send(msg)
        if capture:
            return self._wait('captured', 30)

    # ---- 스트리밍 (GUI·ROS 카메라)
    def request(self, t, T_wb, traffic_T, active, settle=2):
        """캡처 요청 (응답을 기다리지 않음). 이전 요청이 아직이면 False."""
        if self.pending is not None and time.time() - self.pending[1] < 2.0:
            return False
        msg = self._state(t, T_wb, traffic_T, active)
        msg.update(capture=self.raw_path, raw=True, settle=settle)
        self._send(msg)
        self.pending = (self.seq, time.time(), t)
        return True

    def poll(self):
        """완료된 캡처 → (sim 시각, RGB uint8 (H,W,3)) 또는 None."""
        if self.pending is None:
            return None
        self.rx.settimeout(0.0)
        while True:
            try:
                msg = json.loads(self.rx.recv(65536).decode())
            except (BlockingIOError, socket.timeout):
                return None
            if msg.get('type') == 'captured' and msg.get('seq') == self.pending[0]:
                t = self.pending[2]
                self.pending = None
                self.fps = msg.get('fps')
                w, h = int(msg['w']), int(msg['h'])
                img = np.fromfile(self.raw_path, np.uint8)
                if img.size != w * h * 3:
                    return None
                return t, img.reshape(h, w, 3)

    @property
    def alive(self):
        return self.proc.poll() is None

    def close(self):
        try:
            self.tx.sendto(b'{"type":"quit"}', self.addr)
            self.proc.wait(timeout=5)
        except Exception:     # noqa: BLE001
            self.proc.kill()
        self.rx.close()
        self.tx.close()
        try:
            os.remove(self.raw_path)
        except OSError:
            pass


def camera_T_b_opt(robot_cfg):
    _, T_bc = sensor_mounts(robot_cfg)['camera']
    return T_bc @ T_CAMLINK_OPTICAL


class CameraModel:
    """Arducam 모델의 오버스캔 핀홀 크기와 plumb_bob 왜곡 remap (render.CameraRenderer 와 같은 계산, GL 불필요)."""

    def __init__(self, cam_cfg):
        import cv2
        self.W, self.H = cam_cfg['width'], cam_cfg['height']
        self.K = np.array(cam_cfg['K'], float).reshape(3, 3)
        self.D = np.array(cam_cfg['D'], float)
        fx, fy = self.K[0, 0], self.K[1, 1]
        uu, vv = np.meshgrid(np.arange(self.W, dtype=np.float64), np.arange(self.H, dtype=np.float64))
        pts = np.stack([uu.ravel(), vv.ravel()], -1)[:, None, :]
        und = cv2.undistortPoints(pts, self.K, self.D).reshape(self.H, self.W, 2)
        mx, my = np.abs(und[..., 0]).max(), np.abs(und[..., 1]).max()
        self.Wr, self.Hr = int(2 * np.ceil(fx * mx + 4)), int(2 * np.ceil(fy * my + 4))
        self.map_x = (fx * und[..., 0] + self.Wr / 2.0).astype(np.float32)
        self.map_y = (fy * und[..., 1] + self.Hr / 2.0).astype(np.float32)
        self.hfov = 2 * np.degrees(np.arctan(und[self.H // 2, -1, 0]))

    def distort(self, raw):
        import cv2
        return cv2.remap(raw, self.map_x, self.map_y, cv2.INTER_LINEAR)
