"""센서·오버레이 표시용 공용 정의 (ImGui 앱과 Godot 백엔드가 함께 쓴다)."""
import numpy as np

from .gnss import FIX, FLOAT, NONE, SINGLE

LINE_COLORS = {
    'yellow_solid': (1.0, 0.82, 0.10), 'white_solid': (1.0, 1.0, 1.0),
    'white_dashed': (0.55, 0.85, 1.0), 'road_border': (1.0, 0.30, 0.30), 'center': (0.2, 1.0, 0.5),
}
GNSS_COLOR = {FIX: (0.2, 1.0, 0.3, 1), FLOAT: (1.0, 0.85, 0.1, 1), SINGLE: (1.0, 0.45, 0.1, 1), NONE: (1.0, 0.1, 0.1, 1)}
LABEL_COLORS = np.array([[0.5, 0.5, 0.5], [0.35, 0.35, 0.38], [1, 1, 1], [1, 0.8, 0.1], [0.7, 0.75, 0.8],
                         [0.6, 0.6, 0.55], [0.3, 0.6, 0.25], [0.6, 0.5, 0.4], [0.2, 0.6, 1.0], [1, 0.5, 0],
                         [0.1, 0.8, 0.4], [0.7, 0.7, 0.7], [1.0, 0.2, 0.2], [1.0, 0.2, 0.8],
                         [0.15, 0.35, 0.95], [0.6, 0.3, 1.0], [0.3, 0.85, 0.9]], np.float32)   # 13 보행자, 14 트럭, 15 버스, 16 트레일러
SCC_ON, SCC_BEACON, NOTICED, CRASHED = (0.169, 0.831, 0.773, 1), (0.780, 0.490, 1.0, 1), (1.0, 0.722, 0.302, 1), (1.0, 0.353, 0.373, 1)


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


def intensity_colors(x, vmax=60.0):
    """세기 → 색 (어두운 파랑 → 청록 → 노랑)."""
    t = np.clip(np.asarray(x, np.float32) / vmax, 0, 1)
    r = np.clip(2 * t - 0.6, 0, 1)
    g = np.clip(1.6 * t, 0, 1)
    b = np.clip(1.0 - 1.2 * t, 0.15, 1)
    return np.stack([r, g, b, np.ones_like(t)], 1)


def radar_fan(rc):
    """ARS408 감지 범위 면 (센서 좌표): 근거리 ±60°/70m(45° 밖 40m) + 원거리 ±9°/250m → (삼각형 정점, 색, 모서리 선분)."""
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
    return np.array(tris, 'f4'), np.array(cols, 'f4'), np.array(edges, 'f4')
