"""설계 STL → base_link 기준 부품 메시 + CAD 측정 센서 포즈.

입력: '절곡 프레임 설계 v2.stl' (mm, 위쪽이 -Z, 전방이 -X)
출력:
  robot/meshes/{body,pandar40p,tele15,ars408,camera}.stl  (m, base_link 좌표, RViz용)
  robot/meshes/robot_parts.npz                            (심/카메라 렌더용)
  config/robot.yaml                                       (센서 포즈, 없을 때만 생성. --force로 덮어씀)

base_link: 바퀴 4개 중심의 지면 투영점, x 전방 / y 좌측 / z 위 (ROS REP-103).
"""
import argparse
import os

import numpy as np

from .common import ROOT, writable_root
from .stl_components import components, load_binary_stl

# STL 좌표(mm)에서 센서 본체를 고르는 상자. 연결 성분의 bbox가 상자 안에 들어오면 그 센서 부품.
SENSOR_BOXES_STL = {
    'pandar40p': ([5, -60, -700], [135, 70, -580]),
    'tele15': ([-90, -52, -568], [38, 66, -466]),
    'ars408': ([-255, -42, -405], [-217, 54, -264]),
    'camera': ([-119, -10, -470], [-77, 28, -431]),
}


def stl_to_base(p, origin_stl):
    """STL(mm, -X 전방, -Z 위) → base_link(m). Y축 기준 180° 회전 + 원점 이동."""
    p = np.asarray(p, dtype=np.float64)
    q = np.empty_like(p)
    q[..., 0] = -(p[..., 0] - origin_stl[0])
    q[..., 1] = p[..., 1] - origin_stl[1]
    q[..., 2] = -(p[..., 2] - origin_stl[2])
    return q / 1000.0


def write_binary_stl(path, tris):
    tris = np.asarray(tris, dtype=np.float32)
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(tris), dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    rec['n'] = n
    rec['v'] = tris.reshape(-1, 9)
    with open(path, 'wb') as f:
        f.write(b'beacon_sim'.ljust(80, b' '))
        f.write(np.uint32(len(tris)).tobytes())
        f.write(rec.tobytes())


def dedup(tris):
    v, inv = np.unique(np.round(tris.reshape(-1, 3), 6), axis=0, return_inverse=True)
    return v.astype(np.float32), inv.reshape(-1, 3).astype(np.int32)


def build(stl_path, out_root=None, force=False, log=print):
    """STL → out_root/robot/meshes/*, out_root/config/robot.yaml. 반환: robot.yaml 경로."""
    out_root = out_root or writable_root()
    print = log  # noqa: A001
    tris = load_binary_stl(stl_path)
    lab = components(tris)
    ncomp = lab.max() + 1
    cmin = np.full((ncomp, 3), np.inf)
    cmax = np.full((ncomp, 3), -np.inf)
    P = tris.reshape(-1, 3)
    L = np.repeat(lab, 3)
    np.minimum.at(cmin, L, P)
    np.maximum.at(cmax, L, P)
    cnt = np.bincount(lab)

    # 타이어: 같은 삼각형 수를 가진 큰 원통형 부품 4개 (지름 ~195mm)
    ext = cmax - cmin
    tire = [c for c in range(ncomp) if abs(ext[c, 0] - 195) < 3 and abs(ext[c, 2] - 195) < 3 and ext[c, 1] > 30]
    assert len(tire) == 4, f'타이어 4개를 찾지 못함: {tire}'
    wc = (cmin[tire] + cmax[tire]) / 2
    origin = np.array([wc[:, 0].mean(), wc[:, 1].mean(), cmax[tire, 2].max()])  # 지면 = 타이어 최하단(+Z)
    wheel_radius = ext[tire, 0].mean() / 2000.0
    wheelbase = (wc[:, 0].max() - wc[:, 0].min()) / 1000.0
    track = (wc[:, 1].max() - wc[:, 1].min()) / 1000.0

    part_of = np.array(['body'] * ncomp, dtype=object)
    for name, (lo, hi) in SENSOR_BOXES_STL.items():
        inside = np.all(cmin >= lo, axis=1) & np.all(cmax <= hi, axis=1)
        part_of[inside] = name
        print(f'{name}: 부품 {inside.sum()}개, 삼각형 {cnt[inside].sum()}개')

    mesh_dir = os.path.join(out_root, 'robot', 'meshes')
    os.makedirs(mesh_dir, exist_ok=True)
    tri_base = stl_to_base(tris, origin)
    # 축 반전(x,z 부호 반전)은 회전이므로 삼각형 감김 방향은 유지된다.
    npz = {}
    tri_part = part_of[lab]
    for name in ['body'] + list(SENSOR_BOXES_STL):
        t = tri_base[tri_part == name]
        write_binary_stl(os.path.join(mesh_dir, f'{name}.stl'), t)
        v, f = dedup(t)
        npz[f'{name}_v'] = v
        npz[f'{name}_f'] = f
    np.savez_compressed(os.path.join(mesh_dir, 'robot_parts.npz'), **npz)

    # --- 센서 포즈 측정 (STL mm 좌표) ---
    def bbox(name):
        m = part_of == name
        return cmin[m].min(0), cmax[m].max(0)

    def facet_tilt(name, axis):
        """해당 부품에서 주어진 축에 거의 수직인 면들의 평균 법선 → 피치 기울기(rad)."""
        t = tris[tri_part == name]
        n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
        a = np.linalg.norm(n, axis=1)
        nn = n / np.maximum(a, 1e-12)[:, None]
        m = nn[:, axis] > 0.99
        v = (nn[m] * a[m, None]).sum(0)
        v /= np.linalg.norm(v)
        return v

    pitch = {}
    for name, axis in [('pandar40p', 2), ('tele15', 2), ('ars408', 0), ('camera', 0)]:
        v = facet_tilt(name, axis)
        # STL에서 +Z 면 법선 (sx,0,sz) → base_link에서 (-sx,0,-sz). 아래 방향 법선이 앞(+x)으로 기울면 기수 하향(+pitch).
        if axis == 2:
            pitch[name] = float(np.arctan2(v[0], v[2]))
        else:
            pitch[name] = float(np.arctan2(-v[2], v[0]))

    lo, hi = bbox('pandar40p')
    c = (lo + hi) / 2
    # Pandar40P 측정 중심: 하단면에서 47.7mm 위 (AWSIM LaserArrayLibrary centerOfMeasurementLinearOffsetMm)
    pandar_stl = [c[0], c[1], hi[2] - 47.7]
    lo, hi = bbox('tele15')
    c = (lo + hi) / 2
    tele_stl = [lo[0], c[1], c[2]]          # 전면(창) 중심. Livox 원점 정의는 매뉴얼 확인 필요
    lo, hi = bbox('ars408')
    c = (lo + hi) / 2
    radar_stl = [lo[0], c[1], c[2]]         # 전면(레이돔) 중심
    m = (part_of == 'camera') & (ext[:, 0] > 15) & (ext[:, 1] > 30)  # 렌즈 홀더(22x34x34)
    lo, hi = cmin[m].min(0), cmax[m].max(0)
    c = (lo + hi) / 2
    cam_stl = [c[0], c[1], c[2]]            # 렌즈 홀더 중심 ≈ 광학 중심

    poses = {
        'pandar40p': stl_to_base(pandar_stl, origin),
        'tele15': stl_to_base(tele_stl, origin),
        'ars408': stl_to_base(radar_stl, origin),
        'camera': stl_to_base(cam_stl, origin),
    }

    print(f'base_link 원점(STL mm): {origin.round(2)}  바퀴 반지름 {wheel_radius:.4f} m, '
          f'휠베이스 {wheelbase:.3f} m, 윤거 {track:.3f} m')
    for k, p in poses.items():
        print(f'  {k:10s} xyz={np.round(p, 4)}  pitch={np.degrees(pitch[k]):.3f} deg')

    cfg = os.path.join(out_root, 'config', 'robot.yaml')
    if os.path.exists(cfg) and not force:
        print(f'{cfg} 이미 있음 (덮어쓰려면 --force)')
        return cfg
    os.makedirs(os.path.dirname(cfg), exist_ok=True)
    frames = {'pandar40p': 'pandar_40p', 'tele15': 'livox_tele15', 'ars408': 'ars408', 'camera': 'camera_link'}
    with open(cfg, 'w') as f:
        f.write('# tools/build_robot.py가 CAD(STL)에서 측정해 생성. 수동 수정 가능.\n')
        f.write('# 좌표: base_link (바퀴 4개 중심의 지면 투영점, x 전방, y 좌, z 위), 단위 m / rad\n')
        f.write('# 회전: rpy = 고정축 roll, pitch, yaw (URDF와 동일). 마스트 전체가 CAD상 약 0.70도 앞으로 숙여져 있음.\n')
        f.write(f'wheel_radius: {wheel_radius:.4f}\nwheelbase: {wheelbase:.4f}\ntrack: {track:.4f}\n')
        f.write('sensors:\n')
        notes = {
            'pandar40p': '원통 축 중심, 하단면 +47.7mm (측정 중심). yaw(커넥터 방향)는 CAD 형상으로 판별 불가 → 전방 가정',
            'tele15': '전면 창 중심. Livox 좌표 원점 정의(매뉴얼) 확인 필요',
            'ars408': '레이돔 전면 중심',
            'camera': '렌즈 홀더 중심. 광학 프레임은 camera_optical_link (z 전방)',
        }
        for k, p in poses.items():
            f.write(f'  {k}:\n    frame_id: {frames[k]}\n')
            f.write(f'    xyz: [{p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f}]\n')
            f.write(f'    rpy: [0.0, {pitch[k]:.5f}, 0.0]\n')
            f.write(f'    note: "{notes[k]}"\n')
    print(f'생성: {cfg}')
    return cfg


def main():
    ap = argparse.ArgumentParser(description='비콘 CAD(STL) → 로봇 메시 + 센서 장착 포즈')
    ap.add_argument('--stl', default=os.path.join(ROOT, '절곡 프레임 설계 v2.stl'))
    ap.add_argument('--out', default=None, help='출력 루트 (기본: 쓰기 가능하면 저장소, 아니면 ~/.config/beaconsim)')
    ap.add_argument('--force', action='store_true', help='config/robot.yaml 덮어쓰기')
    a = ap.parse_args()
    build(a.stl, a.out, a.force)


if __name__ == '__main__':
    main()
