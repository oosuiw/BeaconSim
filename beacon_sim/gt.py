"""정답(ground truth): 차선 3D 중심선, 주행 차량 3D 박스. ROS 와 무관한 계산만 (발행은 ros_bridge).

차선: 도로 모델의 차선(중앙 황색 실선, 백색 점선, 바깥 백색 실선)마다 도색 중심선을 step 간격 3D 점으로.
  점마다: line_id, kind, painted(점선 빈칸이면 0), wear(0 양호 1 희미 2 벗겨짐 3 드문드문)
  — 실제 노면 도색 메시와 같은 모델·같은 점선 주기. '벗겨짐/드문드문' 의 실제 구멍 위치는 무작위라 점 단위로는 wear 코드만 준다.
차량: 활성 슬롯(주행 차량)만, 박스 중심·크기·방향·속도. 세미 트레일러는 트랙터·트레일러 따로.
"""
import numpy as np

from .scene import PAINT_Z

KIND_ID = {'yellow_solid': 1, 'white_solid': 2, 'white_dashed': 3}
WEAR_ID = {'good': 0, 'faded': 1, 'broken': 2, 'sparse': 3}

LANE_DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('line_id', '<u2'), ('kind', 'u1'),
                       ('side', 'i1'), ('painted', 'u1'), ('wear', 'u1')])


class LaneGT:
    """맵 전체 차선 중심선을 미리 샘플 (map 좌표). window() 로 비콘 주변만 잘라 씀."""

    def __init__(self, model, step=1.0):
        p = model.params
        self.step = step
        s = np.arange(0.0, model.length + 1e-6, step)
        self.lines = []            # [(line_id, kind, side, key, s, P (N,3), painted, wear)]
        period = p.dash_len + p.dash_gap
        for side in (-1, 1):
            for k, ln in enumerate(model.lines()):
                if ln.kind == 'road_border':
                    continue
                t = side * ln.offset
                P = model.point(s, np.full_like(s, t), PAINT_Z)
                P = np.stack([P[..., 0], P[..., 1], P[..., 2]], -1) if P.shape[0] != len(s) else P
                painted = np.ones(len(s), np.uint8)
                if ln.kind == 'white_dashed':
                    painted = ((s % period) < p.dash_len).astype(np.uint8)
                wear = np.zeros(len(s), np.uint8)
                for sd, key, a, b, kind, _st in model.wear:
                    if sd == side and key == ln.key:
                        wear[(s >= a) & (s < b)] = WEAR_ID.get(kind, 0)
                lid = (0 if side < 0 else 100) + k
                self.lines.append((lid, KIND_ID[ln.kind], side, ln.key, s, np.asarray(P, float), painted, wear))

    def window(self, s_center, behind, ahead, T_wb=None):
        """비콘 주변 s 구간의 점들 → LANE_DTYPE 배열. T_wb 를 주면 base_link 좌표로."""
        chunks = []
        inv = None
        if T_wb is not None:
            inv = np.linalg.inv(T_wb)
        for lid, kind, side, _key, s, P, painted, wear in self.lines:
            i0 = int(np.searchsorted(s, s_center - behind))
            i1 = int(np.searchsorted(s, s_center + ahead))
            if i1 <= i0:
                continue
            Q = P[i0:i1]
            if inv is not None:
                Q = Q @ inv[:3, :3].T + inv[:3, 3]
            a = np.zeros(i1 - i0, LANE_DTYPE)
            a['x'], a['y'], a['z'] = Q[:, 0], Q[:, 1], Q[:, 2]
            a['line_id'], a['kind'], a['side'] = lid, kind, side
            a['painted'], a['wear'] = painted[i0:i1], wear[i0:i1]
            chunks.append(a)
        return np.concatenate(chunks) if chunks else np.zeros(0, LANE_DTYPE)


TRAILER_ID0 = 100000       # 트레일러 정답 객체 번호 = 이 값 + 슬롯 (트랙터는 슬롯 번호)


def vehicle_objects(traffic, t, T_wb=None, max_range=200.0, in_map=False):
    """활성 주행 차량 → [{'id', 'kind', 'T' (박스 중심 4x4), 'size' (L,W,H), 'vel' (박스 좌표계 vx,vy,vz)}].
    세미 트레일러는 트랙터(truck)와 트레일러(trailer, 꺾임 반영) 두 객체.
    T_wb 를 주면 비콘에서 max_range 안쪽만, in_map 이 아니면 base_link 좌표로."""
    from . import vehicles
    Ts, vel = traffic.poses_at(t)
    art = traffic.trailer_angle()
    inv = np.linalg.inv(T_wb) if T_wb is not None else None
    out = []

    def add(oid, kind, T, size, R_w, v):
        if inv is not None:
            if np.linalg.norm(T[:3, 3] - T_wb[:3, 3]) > max_range:
                return
            if not in_map:
                T = inv @ T
        out.append({'id': int(oid), 'kind': kind, 'T': T, 'size': size, 'vel': R_w.T @ v})

    for i in np.nonzero(traffic.active)[0]:
        sp = traffic.specs[i]
        tr = vehicles.trailer(sp.model) if sp.model else None
        if tr is None:
            T = Ts[i].copy()
            T[:3, 3] = T[:3, 3] + T[:3, 2] * sp.height / 2          # 바닥 기준 → 박스 중심
            add(i, sp.kind, T, (sp.length, sp.width, sp.height), Ts[i][:3, :3], vel[i])
            continue
        cx, ln, h = vehicles.tractor_box(sp.model)
        T = Ts[i] @ _shift(cx, h / 2)
        add(i, 'truck', T, (ln, sp.width, h), Ts[i][:3, :3], vel[i])
        L2, W2, H2 = tr['size']
        T2 = Ts[i] @ vehicles.hitch_T(tr['hitch_x'], art[i]) @ _shift(tr['center'][0], H2 / 2)
        add(TRAILER_ID0 + i, 'trailer', T2, (L2, W2, H2), T2[:3, :3], vel[i])
    return out


def _shift(x, z):
    T = np.eye(4)
    T[0, 3], T[2, 3] = x, z
    return T
