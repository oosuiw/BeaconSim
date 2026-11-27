"""비콘 로봇 기구학: 차체(base_link) + 접이식 센서타워(sensor_tower_link, 회전 관절 1개) + 화면용 링크(크랭크·연결 링크).

robot.yaml
  sensor_tower:    (없으면 센서타워 없는 예전 로봇: 모든 센서가 base_link 에 고정)
    pivot, axis    base_link 기준 접이 축
    deployed       펴짐 각도 [rad] (0 = 접힘)
    deploy_time    접힘 → 펴짐 시간 [s]
  sensors.<이름>.parent: sensor_tower_link | base_link (기본 base_link)
관절 각도 q: 0 = 접힘, deployed = 펴짐.  T_base_mast(q) = 이동(pivot) · 회전(axis, q - deployed)
→ 펴짐 상태에서 sensor_tower_link 축은 base_link 축과 같다 (센서 rpy 는 펴짐 기준).
"""
import json
import os

import numpy as np

from .common import ROBOT_PART_COLORS, make_T, robot_mesh_dir


def tower_cfg(robot_cfg):
    """robot.yaml 의 센서타워 설정 (예전 이름 'mast' 도 읽음). 없으면 None."""
    return robot_cfg.get('sensor_tower') or robot_cfg.get('mast') or None


def rot(axis, ang):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K


class RobotKin:
    def __init__(self, robot_cfg, links_meta=None):
        m = tower_cfg(robot_cfg)
        self.has_mast = m is not None
        self.mast_frame = (m or {}).get('frame_id', 'sensor_tower_link')
        self.joint = (m or {}).get('joint', 'sensor_tower_joint')
        self.pivot = np.asarray((m or {}).get('pivot', [0, 0, 0]), float)
        self.axis = np.asarray((m or {}).get('axis', [0, 1, 0]), float)
        self.q_dep = float((m or {}).get('deployed', 0.0))
        self.deploy_time = float((m or {}).get('deploy_time', 4.0))
        self.rate = self.q_dep / max(self.deploy_time, 1e-3)       # rad/s
        self.sensors = {}
        for name, s in robot_cfg['sensors'].items():
            parent = s.get('parent', 'base_link')
            self.sensors[name] = (s['frame_id'], parent, make_T(s['xyz'], s['rpy']))
        self.links = links_meta or {}

    def T_base_mast(self, q):
        T = np.eye(4)
        if self.has_mast:
            T[:3, :3] = rot(self.axis, q - self.q_dep)
            T[:3, 3] = self.pivot
        return T

    def mount(self, name, q):
        """센서 → (frame_id, T_base_sensor) (관절 각도 q 에서)"""
        frame, parent, T = self.sensors[name]
        if self.has_mast and parent == self.mast_frame:
            return frame, self.T_base_mast(q) @ T
        return frame, T

    def mounts(self, q=None):
        q = self.q_dep if q is None else q
        return {n: self.mount(n, q) for n in self.sensors}

    def link_T(self, link, q):
        """부품 메시(링크 좌표) → base_link 변환. 화면용 링크는 접힘·펴짐 사이를 축 회전으로 보간 (양 끝은 CAD 와 같음)."""
        if link == 'base' or not self.has_mast:
            return np.eye(4)
        if link == 'mast':
            return self.T_base_mast(q)
        L = self.links.get(link)
        if L is None:
            return np.eye(4)
        s = 1.0 if self.q_dep <= 0 else float(np.clip(q / self.q_dep, 0.0, 1.0))
        R = rot(L['axis'], -(1.0 - s) * L['angle'])
        p = np.asarray(L['point'], float)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = p - R @ p
        return T


def load_robot_links():
    """robot_parts.npz → (부품 [(이름, 링크, v, f, 색)], 링크 정보). 예전 형식(meta 없음)은 모두 base."""
    d = np.load(os.path.join(robot_mesh_dir(), 'robot_parts.npz'))
    meta = json.loads(bytes(d['meta']).decode()) if 'meta' in d.files else {}
    names = sorted({k[:-2] for k in d.files if k.endswith(('_v', '_f'))})
    parts = []
    for n in names:
        pm = meta.get('parts', {}).get(n, {})
        parts.append((n, pm.get('link', 'base'), d[f'{n}_v'], d[f'{n}_f'],
                      tuple(pm.get('color', ROBOT_PART_COLORS.get(n, (0.6, 0.6, 0.6))))))
    return parts, meta.get('links', {}), meta


def robot_kin(robot_cfg):
    _, links, _ = load_robot_links()
    return RobotKin(robot_cfg, links)
