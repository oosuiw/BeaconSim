"""설정 로드, 좌표 변환, 로봇 모델/URDF 공용 함수."""
import os
import re

import numpy as np
import yaml

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))      # 배포 리소스 (AppImage 안은 읽기 전용)
USER_DIR = os.environ.get('BEACONSIM_HOME', os.path.join(os.path.expanduser('~'), '.config', 'beaconsim'))
CACHE_DIR = os.path.join(os.path.expanduser('~'), '.cache', 'beaconsim')


def resource(rel):
    """사용자 폴더(~/.config/beaconsim)에 있으면 그것, 없으면 배포 리소스."""
    for base in (USER_DIR, ROOT):
        p = os.path.join(base, rel)
        if os.path.exists(p):
            return p
    return os.path.join(ROOT, rel)


def writable_root():
    """생성물(로봇 모델, sim.yaml 적용 등)을 쓸 위치: 저장소에서 실행하면 저장소, AppImage면 사용자 폴더."""
    return ROOT if os.access(ROOT, os.W_OK) else USER_DIR


def results_root():
    return os.path.join(ROOT, 'results') if os.access(ROOT, os.W_OK) else \
        os.path.join(os.path.expanduser('~'), 'BeaconSim_results')


def load_yaml(path):
    if not os.path.isabs(path):
        path = resource(path)
    with open(path) as f:
        return yaml.safe_load(f)


def robot_mesh_dir():
    return os.path.dirname(resource(os.path.join('robot', 'meshes', 'robot_parts.npz')))


def has_robot_model():
    return os.path.exists(resource(os.path.join('robot', 'meshes', 'robot_parts.npz'))) and \
        os.path.exists(resource(os.path.join('config', 'robot.yaml')))


def rpy_to_R(r, p, y):
    """URDF 규약: 고정축 X(roll) → Y(pitch) → Z(yaw). R = Rz(y) Ry(p) Rx(r)."""
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def make_T(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    T = np.eye(4)
    T[:3, :3] = rpy_to_R(*rpy)
    T[:3, 3] = xyz
    return T


def planar_T(x, y, yaw, z=0.0):
    return make_T((x, y, z), (0, 0, yaw))


def R_to_quat(R):
    """회전행렬 → (x, y, z, w)."""
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return np.array([x, y, z, w])


# 카메라 링크(x 전방, z 위) → 광학 프레임(z 전방, x 우, y 하)
T_CAMLINK_OPTICAL = make_T((0, 0, 0), (-np.pi / 2, 0, -np.pi / 2))


def sensor_mounts(robot_cfg):
    """robot.yaml → {name: (frame_id, T_base_sensor)}"""
    out = {}
    for name, s in robot_cfg['sensors'].items():
        out[name] = (s['frame_id'], make_T(s['xyz'], s['rpy']))
    return out


def load_robot_parts():
    d = np.load(os.path.join(robot_mesh_dir(), 'robot_parts.npz'))
    names = sorted({k[:-2] for k in d.files})
    return {n: (d[f'{n}_v'], d[f'{n}_f']) for n in names}


ROBOT_PART_COLORS = {
    'body': (0.85, 0.55, 0.10), 'pandar40p': (0.05, 0.15, 0.85), 'tele15': (0.85, 0.08, 0.05),
    'ars408': (0.12, 0.12, 0.12), 'camera': (0.10, 0.45, 0.15),
}


def build_urdf(robot_cfg):
    """robot.yaml 하나로 URDF 생성 → TF와 심 센서 포즈가 같은 값을 쓴다."""
    mesh_dir = robot_mesh_dir()
    lines = ['<?xml version="1.0"?>', '<robot name="safety_beacon">']
    for part, c in ROBOT_PART_COLORS.items():
        lines.append(f'  <material name="{part}_mat"><color rgba="{c[0]} {c[1]} {c[2]} 1"/></material>')
    lines.append('  <link name="base_link">')
    for part in ROBOT_PART_COLORS:
        lines.append(f'    <visual name="{part}"><geometry><mesh filename="file://{mesh_dir}/{part}.stl"/></geometry>'
                     f'<material name="{part}_mat"/></visual>')
    lines.append('  </link>')

    def joint(parent, child, xyz, rpy):
        lines.append(f'  <link name="{child}"/>')
        lines.append(f'  <joint name="{parent}_to_{child}" type="fixed"><parent link="{parent}"/><child link="{child}"/>'
                     f'<origin xyz="{xyz[0]:.5f} {xyz[1]:.5f} {xyz[2]:.5f}" rpy="{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}"/></joint>')

    for name, s in robot_cfg['sensors'].items():
        joint('base_link', s['frame_id'], s['xyz'], s['rpy'])
        if name == 'camera':
            joint(s['frame_id'], 'camera_optical_link', (0, 0, 0), (-np.pi / 2, 0, -np.pi / 2))
    lines.append('</robot>')
    return '\n'.join(lines)

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
