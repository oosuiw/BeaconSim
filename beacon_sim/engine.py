"""시뮬레이션 엔진 (ROS 비의존). GUI 앱과 헤드리스 ROS 노드가 함께 쓴다.

- 도로/시나리오/교통/환경 생성, RGL 씬(정적 + 교통 슬롯 + 사고 차량 + 로봇 메시)
- 비콘 기구학: 명령(/cmd_vel 또는 키보드) → 스키드 스티어, 노면 추종
- 센서: Pandar40P, Tele-15, ARS408 (RGL), ZED-F9P (GNSS 모델)
- step(dt) 고정 스텝. 최신 결과는 snapshot()으로 (스레드 안전) 가져간다.
"""
import threading
import time

import numpy as np

from . import rgl, scene
from .common import load_robot_parts, sensor_mounts
from .environment import EnvParams, notice_scale, resolve as resolve_env
from .gnss import GnssF9P, GnssParams
from .roadgen import RoadParams, generate
from .sensors import ARS408, Pandar40P, Tele15, entity_id
from .traffic import Scenario, ScenarioParams, TrafficParams, TrafficSim, resolve_seed

_RGL_LOCK = threading.Lock()


class SimEngine:
    DT = 0.01

    def __init__(self, cfg, robot_cfg, model=None, parts=None, log=print):
        self.cfg, self.robot_cfg, self.log = cfg, robot_cfg, log
        cfg['map']['seed'] = resolve_seed(cfg['map'].get('seed', 1))
        cfg['scenario']['seed'] = resolve_seed(cfg['scenario'].get('seed', 1))
        self.model = model if model is not None else generate(RoadParams(**cfg['map']))
        self.parts = parts if parts is not None else scene.build_static(self.model)
        self.scn = Scenario(self.model, ScenarioParams(**cfg['scenario']))
        self.traffic = TrafficSim(self.model, self.scn, TrafficParams(**cfg['traffic']), cfg['scenario']['seed'])
        self.env = resolve_env(EnvParams(**cfg.get('environment', {})), cfg['scenario']['seed'])
        self.gnss = GnssF9P(self.model, GnssParams(**cfg.get('gnss', {})), cfg['scenario']['seed'])
        self.mounts = sensor_mounts(robot_cfg)
        sc = cfg['sim']
        self.max_v, self.max_w, self.cmd_timeout = sc['max_linear'], sc['max_angular'], sc['cmd_vel_timeout']

        T0 = self.scn.beacon_pose(self.scn.beacon_s0)
        self.x, self.y = float(T0[0, 3]), float(T0[1, 3])
        self.yaw = float(np.arctan2(T0[1, 0], T0[0, 0]))
        self.v = self.w = 0.0
        self.t = 0.0
        self.tick = 0
        self.cmd = {'ros': (0.0, 0.0, -1e9), 'key': (0.0, 0.0, -1e9)}
        self.cmd_src = '-'
        self.lock = threading.Lock()
        self.latest = {}
        self.new_events = []
        self.n_events = 0
        self.stats = {}

        with _RGL_LOCK:
            rgl.configure_logging(rgl.LOG_WARN)
            self._build_rgl()
            self.sensors = {}
            for name, cls in [('pandar40p', Pandar40P), ('tele15', Tele15), ('ars408', ARS408)]:
                if cfg[name]['enabled']:
                    self.sensors[name] = cls(cfg[name])
                    self.sensors[name].env = self.env
        phase = {'pandar40p': 0.0, 'tele15': 0.03, 'ars408': 0.06}
        self.next_fire = {k: phase[k] for k in self.sensors}
        self.period = {k: 1.0 / cfg[k]['rate_hz'] for k in self.sensors}
        self.gnss_next = 0.0
        self.T_wb, self.s_b, self.t_b = self._ground_pose()
        # 운전자 인지: 환경·경광등 배율, 가시거리, 시야 가림(RGL 레이)
        self.traffic.notice_scale = notice_scale(self.env, self.scn.sp.beacon_light)
        self.traffic.visibility = self.env.visibility * 0.8
        self.traffic.los_fn = self._line_of_sight
        self._los_graph = None
        # 비콘 외형 (CAD 본체 bbox) — 충돌 판정
        v = np.concatenate([pv for pv, _ in load_robot_parts().values()])
        self.half_len = float(max(abs(v[:, 0].min()), v[:, 0].max()))
        self.half_wid = float(max(abs(v[:, 1].min()), v[:, 1].max()))
        self.damaged = False
        self.contact = None                     # 현재 접촉 중인 물체 이름
        self.events = []                        # 엔진 이벤트 (traffic 이벤트와 같은 형식)
        self.faults = cfg.get('faults', []) or []      # [{'sensor': 'pandar40p', 't0': 20, 't1': 30}]

    # ------------------------------------------------------------ RGL 씬
    def _build_rgl(self):
        self._ents, self._meshes = [], []
        for k, p in enumerate(self.parts):
            mesh = rgl.Mesh(p.v, p.f)
            self._meshes.append(mesh)
            self._ents.append(rgl.Entity(mesh, entity_id=entity_id(p.cls, k), laser_retro=p.retro))
        meshes = {}
        self.npc_entities = []
        for i, sp in enumerate(self.traffic.specs):
            if sp.key not in meshes:
                meshes[sp.key] = rgl.Mesh(*scene.vehicle_mesh(sp.kind, sp.length, sp.width, sp.height))
                self._meshes.append(meshes[sp.key])
            self.npc_entities.append(rgl.Entity(meshes[sp.key], entity_id=entity_id('vehicle', i),
                                                laser_retro=scene.RETRO['vehicle']))
        self.npc_kind = {i: sp.kind for i, sp in enumerate(self.traffic.specs)}
        self.accident = self.scn.accident_poses()
        for k, (sp, T) in enumerate(self.accident):
            mesh = rgl.Mesh(*scene.vehicle_mesh(sp.kind, sp.length, sp.width, sp.height))
            self._meshes.append(mesh)
            e = rgl.Entity(mesh, entity_id=entity_id('accident', k), laser_retro=scene.RETRO['accident'])
            e.set_transform(T)
            self._ents.append(e)
        ign = {n: self.cfg[n]['sensor_id'] for n in ('pandar40p', 'tele15', 'ars408')}
        self.robot_entities = []
        for k, (name, (v, f)) in enumerate(load_robot_parts().items()):
            if name == 'pandar40p':
                # Pandar 하우징은 레이 원점을 감싸는 원통: '자기 무시' 엔티티가 움직이면 RGL v0.21 커널이
                # illegal memory access로 죽는 경우가 있어 제외 (다른 센서 시야를 가리지 않으므로 영향 없음)
                continue
            mesh = rgl.Mesh(v, f)
            self._meshes.append(mesh)
            self.robot_entities.append(rgl.Entity(mesh, entity_id=entity_id('robot', k),
                                                  laser_retro=scene.RETRO['robot'], ignored_by_sensor=ign.get(name)))

    def close(self):
        """RGL 객체 해제 (맵 재생성 시)."""
        with _RGL_LOCK:
            for s in self.sensors.values():
                s.graph.destroy()
            if self._los_graph is not None:
                self._los_graph.destroy()
            for e in self._ents + self.npc_entities + self.robot_entities:
                e.destroy()
            for m in self._meshes:
                m.destroy()
        self.sensors = {}

    # ------------------------------------------------------------ 비콘
    def set_cmd(self, source, v, w):
        """source: 'ros' | 'key'. ROS 명령이 최근에 있으면 우선."""
        self.cmd[source] = (float(v), float(w), self.t)

    # ------------------------------------------------------------ 충돌 / 파손 / 고장 / 시야
    def _faulted(self, name):
        return any(f.get('sensor') == name and f.get('t0', 0) <= self.t < f.get('t1', 1e9) for f in self.faults)

    def _footprint(self, x, y, yaw):
        c, s = np.cos(yaw), np.sin(yaw)
        hl, hw = self.half_len, self.half_wid
        return np.array([[x + c * dx - s * dy, y + s * dx + c * dy] for dx, dy in
                         [(hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)]])

    @staticmethod
    def _obb_overlap(A, B):
        """2D 볼록 사각형 SAT 겹침 판정."""
        for P in (A, B):
            for k in range(4):
                e = P[(k + 1) % 4] - P[k]
                n = np.array([-e[1], e[0]])
                pa, pb = A @ n, B @ n
                if pa.max() < pb.min() or pb.max() < pa.min():
                    return False
        return True

    def _veh_rect(self, sp, T):
        ex, ey = T[:2, 0] / max(np.linalg.norm(T[:2, 0]), 1e-9), T[:2, 1] / max(np.linalg.norm(T[:2, 1]), 1e-9)
        c = T[:2, 3]
        hl, hw = sp.length / 2, sp.width / 2
        return np.array([c + ex * dx + ey * dy for dx, dy in [(hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)]])

    def _collides(self, x, y, yaw):
        """비콘 외형이 노변 구조물/차량과 겹치면 그 이름, 아니면 None."""
        m = self.model
        R = self._footprint(x, y, yaw)
        ts = np.array([m.project(px, py)[1] for px, py in R])
        s_c, _ = m.project(x, y)
        bh = m.params.barrier_half
        if (np.sign(ts) != np.sign(ts[0])).any() or np.abs(ts).min() < bh:
            return '터널 벽' if m.in_tunnel(s_c) else '중앙분리대'
        outer = m.edge_t + 0.3 if m.in_tunnel(s_c) else m.guardrail_t - 0.05
        if np.abs(ts).max() > outer:
            return '터널 벽' if m.in_tunnel(s_c) else '가드레일'
        for sp, T in self.accident:
            if np.linalg.norm(T[:2, 3] - [x, y]) < 8 and self._obb_overlap(R, self._veh_rect(sp, T)):
                return '사고 차량'
        Ts, _ = self.traffic.poses_at(self.t)
        near = np.nonzero(self.traffic.active & (np.linalg.norm(Ts[:, :2, 3] - [x, y], axis=1) < 12))[0]
        for i in near:
            if self._obb_overlap(R, self._veh_rect(self.traffic.specs[i], Ts[i])):
                return '차량'
        return None

    def _on_beacon_hit(self, t, i, v_hit):
        """차량이 비콘을 추돌 → 비콘이 차량 진행 방향으로 밀려나고 파손(이후 명령 무시)."""
        self.damaged = True
        T = self.traffic.poses_at(self.t)[0][i]
        d = float(min(0.45 * v_hit, 25.0))
        dirv = T[:2, 0] / max(np.linalg.norm(T[:2, 0]), 1e-9)
        x, y = self.x + dirv[0] * d, self.y + dirv[1] * d
        s_, t_ = self.model.project(x, y)
        lim = self.model.edge_t - 0.2
        t_ = float(np.clip(abs(t_), self.model.params.barrier_half + 0.5, lim)) * np.sign(t_ or 1)
        p = self.model.point(s_, t_)
        self.x, self.y = float(p[0]), float(p[1])
        self.yaw += float(np.random.default_rng(int(t * 1000)).uniform(-1.6, 1.6))
        self.v = self.w = 0.0
        self.T_wb, self.s_b, self.t_b = self._ground_pose()
        self.events.append((self.t, 'beacon_damaged', int(i),
                            f'비콘 파손: 차량 {v_hit * 3.6:.0f} km/h 추돌, {d:.0f} m 밀려남 (이후 명령 무시)'))

    def _line_of_sight(self, eyes, target):
        """RGL 레이로 눈 → 비콘 사이 가림 판정. 비콘 자체 히트(목표 0.8 m 이내)는 보이는 것으로."""
        d = target[None, :] - eyes
        dist = np.linalg.norm(d, axis=1)
        az = np.arctan2(d[:, 1], d[:, 0])
        el = np.arcsin(np.clip(d[:, 2] / np.maximum(dist, 1e-6), -1, 1))
        rays = rgl.ray_poses_from_angles(az, el)
        rays[:, :3, 3] = eyes
        ranges = np.c_[np.full(len(dist), 0.05), np.maximum(dist - 0.8, 0.1)]
        with _RGL_LOCK:
            if self._los_graph is None:
                self._los_graph = rgl.RaycastGraph(rays, ranges, [rgl.RAY_IDX_U32, rgl.DISTANCE_F32], 1004)
            else:
                self._los_graph.set_rays(rays, ranges)
            o = self._los_graph.run(np.eye(4))
        vis = np.ones(len(eyes), bool)
        vis[o[rgl.RAY_IDX_U32]] = False
        return vis

    def _ground_pose(self):
        s, t = self.model.project(self.x, self.y)
        Ts = self.model.pose(s, t)
        fz = Ts[:3, 2]
        h = np.array([np.cos(self.yaw), np.sin(self.yaw), 0.0])
        fx = h - fz * (h @ fz)
        fx /= np.linalg.norm(fx)
        T = np.eye(4)
        T[:3, 0], T[:3, 1], T[:3, 2] = fx, np.cross(fz, fx), fz
        T[:3, 3] = [self.x, self.y, Ts[2, 3]]
        return T, s, t

    def _sensor_velocity(self, T_bs):
        r = T_bs[:3, 3]
        v_b = np.array([self.v, 0, 0]) + np.cross([0, 0, self.w], r)
        R = T_bs[:3, :3].T
        return R @ v_b, R @ np.array([0, 0, self.w])

    # ------------------------------------------------------------ 스텝
    def step(self):
        dt = self.DT
        self.t += dt
        self.tick += 1
        rv, rw, rt = self.cmd['ros']
        kv, kw, kt = self.cmd['key']
        if self.t - rt <= self.cmd_timeout:
            v, w, self.cmd_src = rv, rw, '/cmd_vel'
        elif self.t - kt <= self.cmd_timeout:
            v, w, self.cmd_src = kv, kw, '키보드'
        else:
            v, w, self.cmd_src = 0.0, 0.0, '-'
        if self.damaged:
            v, w, self.cmd_src = 0.0, 0.0, '파손'
        with self.lock:
            self.v = float(np.clip(v, -self.max_v, self.max_v))
            self.w = float(np.clip(w, -self.max_w, self.max_w))
            ny = self.yaw + self.w * dt
            nx = self.x + self.v * np.cos(ny) * dt
            ny_ = self.y + self.v * np.sin(ny) * dt
            hit = self._collides(nx, ny_, ny) if (self.v or self.w) else None
            if hit is None:
                self.x, self.y, self.yaw = nx, ny_, ny
                self.contact = None
            else:                                # 막힘: 이동하지 않음
                if hit != self.contact:
                    self.events.append((self.t, 'beacon_contact', -1, f'비콘이 {hit}에 닿아 정지'))
                self.contact = hit
                self.v = self.w = 0.0
            self.T_wb, self.s_b, self.t_b = self._ground_pose()
            if self.tick % int(round(TrafficSim.DT / dt)) == 0:
                self.traffic.step(self.s_b, self.t_b)
                if self.traffic.beacon_hits and not self.damaged:
                    self._on_beacon_hit(*self.traffic.beacon_hits[0])
        fired = {}
        due = [k for k in self.sensors if self.t >= self.next_fire[k] - 1e-9]
        for k in [k for k in due if self._faulted(k)]:          # 고장 주입: 출력 없음
            self.next_fire[k] += self.period[k]
            due.remove(k)
        if due:
            with _RGL_LOCK:
                rgl.set_scene_time(int(self.t * 1e9) + 1_000_000_000)
                for e in self.robot_entities:
                    e.set_transform(self.T_wb)
                with self.lock:
                    Ts, _ = self.traffic.poses_at(self.t)
                # 움직인 차량만 RGL 에 보낸다 (비활성 차량은 지하에 그대로 → 매번 200대 호출하던 것 제거)
                M = np.ascontiguousarray(Ts[:, :3, :4], dtype=np.float32)
                last = getattr(self, '_npc_last', None)
                changed = range(len(M)) if last is None or last.shape != M.shape else \
                    np.nonzero((M != last).any(axis=(1, 2)))[0]
                for i in changed:
                    self.npc_entities[i].set_mat(M[i])
                self._npc_last = M
                for name in due:
                    self.next_fire[name] += self.period[name]
                    t0 = time.perf_counter()
                    fired[name] = self._fire(name)
                    self.stats.setdefault(name, []).append(time.perf_counter() - t0)
        if self.gnss.gp.enabled and self.t >= self.gnss_next - 1e-9:
            self.gnss_next += 1.0 / self.gnss.gp.rate_hz
            if not self._faulted('gnss'):
                T_ant = self.T_wb.copy()                    # 안테나 위치 (base_link + antenna_xyz)
                T_ant[:3, 3] = self.T_wb[:3, :3] @ np.asarray(self.gnss.gp.antenna_xyz, float) + self.T_wb[:3, 3]
                fired['gnss'] = self.gnss.step(self.t, T_ant, self.s_b)
        ev = self.traffic.events[self.n_events:] + self.events
        self.n_events = len(self.traffic.events)
        self.events = []
        with self.lock:
            self.latest.update(fired)
            self.new_events += ev
        return fired, ev

    def _fire(self, name):
        frame, T_bs = self.mounts[name]
        T_ws = self.T_wb @ T_bs
        lv, av = self._sensor_velocity(T_bs)
        out = {'t': self.t, 'frame': frame, 'T_ws': T_ws, 'T_wb': self.T_wb.copy()}
        s = self.sensors[name]
        if name == 'pandar40p':
            out['points'], out['gt'] = s.run(T_ws)        # 속도 필드 미사용 → 센서 속도 전달 안 함 (RGL 오류 회피)
        elif name == 'tele15':
            out['points'] = s.run(T_ws, int(self.t * 1e9) + 1_000_000_000)
        elif name == 'ars408':
            c = s.run(T_ws, lv, av, self.npc_kind)
            out['clusters'] = c
            out['scan'] = s.scan
            out['tracks'] = self._radar_tracks(c, T_ws, lv)
        return out

    def _radar_tracks(self, c, T_ws, sensor_lv):
        """클러스터가 잡힌 차량/사고 차량 → 객체 목록 (센서 좌표, 상대속도는 정답)."""
        R_sw = T_ws[:3, :3].T
        _, vel = self.traffic.poses_at(self.t)
        tracks = []
        for cls_name in ('vehicle', 'accident'):
            veh = c[c['label'] == scene.CLASS[cls_name]]
            for inst in np.unique(veh['instance']):
                cl = veh[veh['instance'] == inst]
                p = cl[np.argmin(cl['distance'])]
                if cls_name == 'vehicle':
                    sp, v_w, oid = self.traffic.specs[int(inst)], vel[int(inst)], int(inst)
                else:
                    sp, v_w, oid = self.accident[int(inst)][0], np.zeros(3), 1000 + int(inst)
                v_rel = R_sw @ v_w - sensor_lv
                tracks.append({'id': oid, 'x': float(p['x']), 'y': float(p['y']), 'vx': float(v_rel[0]),
                               'vy': float(v_rel[1]), 'dist': float(p['distance']), 'spec': sp})
        return tracks

    # ------------------------------------------------------------ 조회
    def snapshot(self):
        """GUI용 상태 묶음 (복사본)."""
        tr = self.traffic
        with self.lock:
            latest = dict(self.latest)
            ev, self.new_events = self.new_events, []
            Ts, vel = tr.poses_at(self.t)
            snap = {
            't': self.t, 'T_wb': self.T_wb.copy(), 'v': self.v, 'w': self.w, 'cmd_src': self.cmd_src,
            'damaged': self.damaged, 'contact': self.contact,
            's_b': self.s_b, 't_b': self.t_b,
            'traffic_T': Ts, 'traffic_vel': vel, 'active': tr.active.copy(), 'crashed': tr.crashed.copy(),
            'noticed': tr.noticed.copy(), 'scc': tr.scc.copy(), 'scc_target': tr.scc_target.copy(),
            'scc_lock': tr.scc_lock.copy(), 'beh': tr.beh.copy(), 'counts': dict(tr.counts),
            'sensors': latest, 'events': ev,
            }
        return snap

    def take_stats(self):
        s = {k: 1e3 * float(np.mean(v)) for k, v in self.stats.items() if v}
        self.stats = {}
        return s


class EngineThread(threading.Thread):
    """실시간(×rtf) 페이싱으로 엔진을 돌리는 스레드. on_step(engine, fired, events) 콜백(ROS 발행 등)."""

    def __init__(self, engine, rtf=1.0, on_step=None):
        super().__init__(daemon=True)
        self.engine, self.rtf, self.on_step = engine, rtf, on_step
        self.running = True
        self.paused = False
        self.wall_t0 = None
        self.sim_t0 = 0.0
        self.measured_rtf = 0.0

    def run(self):
        e = self.engine
        self.wall_t0, self.sim_t0 = time.perf_counter(), e.t
        last_meas = (self.wall_t0, e.t)
        while self.running:
            if self.paused:
                time.sleep(0.02)
                self.wall_t0, self.sim_t0 = time.perf_counter(), e.t
                continue
            target = self.wall_t0 + (e.t + e.DT - self.sim_t0) / max(self.rtf, 1e-3)
            now = time.perf_counter()
            if target > now:
                time.sleep(min(target - now, 0.01))
                continue
            if now - target > 0.5:                     # 너무 밀리면 기준 재설정 (실시간 불가)
                self.wall_t0, self.sim_t0 = now, e.t
            fired, ev = e.step()
            if self.on_step is not None:
                try:
                    self.on_step(e, fired, ev)
                except Exception as ex:    # noqa: BLE001  종료 중 ROS 발행 실패 등 → 스레드는 계속/정상 종료
                    if not self.running:
                        break
                    if not getattr(self, '_warned', False):
                        self._warned = True
                        print(f'[EngineThread] on_step 오류: {ex}', flush=True)
            if now - last_meas[0] > 1.0:
                self.measured_rtf = (e.t - last_meas[1]) / (now - last_meas[0])
                last_meas = (now, e.t)

    def set_rtf(self, rtf):
        self.wall_t0, self.sim_t0 = time.perf_counter(), self.engine.t
        self.rtf = rtf

    def stop(self):
        self.running = False
        self.join(timeout=2.0)
