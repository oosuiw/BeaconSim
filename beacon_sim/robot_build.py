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


# ================================================================== STEP (접이식 마스트, BeaconV6~)
# 차체 안에 가려 보이지 않는 부품 (메시 크기만 키움)
STEP_HIDDEN = ('모터(MD)', 'Bearing', '벨트', '풀리', '컨트롤러', 'mcu 컨트롤러', '파워 분대보드', 'XT60', 'PCB ASSY',
               'HEAT SINK', '모터 플레이트')
# 부품 이름(경로에 포함) → 출력 부품
STEP_PARTS = [
    ('pandar40p', ('Pandar40P',)), ('tele15', ('Tele-15',)), ('ars408', ('asr_408', 'ars_408', 'ARS408')),
    ('gnss', ('GNSS 안테나',)), ('camera', ('카메라 마운트',)), ('led_red', ('(RED)',)), ('led_blue', ('(BLUE)',)),
    ('body_dark', ('배터리', '하단바', 'ㄷ자함 바', 'AK45')),
]
PART_COLORS_V6 = {
    'body': (0.88, 0.88, 0.90), 'body_dark': (0.20, 0.20, 0.22), 'mast_body': (0.90, 0.90, 0.92),
    'pandar40p': (0.05, 0.15, 0.85), 'tele15': (0.16, 0.16, 0.18), 'ars408': (0.15, 0.15, 0.16),
    'camera': (0.10, 0.45, 0.15), 'gnss': (0.10, 0.10, 0.11), 'led_red': (0.90, 0.10, 0.10),
    'led_blue': (0.10, 0.25, 0.95), 'tire': (0.10, 0.10, 0.10), 'rim': (0.80, 0.80, 0.82),
    'crank': (0.60, 0.60, 0.62), 'coupler': (0.60, 0.60, 0.62),
}


def _kabsch(A, B):
    """B ≈ A R^T + t"""
    ca, cb = A.mean(0), B.mean(0)
    U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    return R, cb - R @ ca


def _axis_angle(R, t):
    """강체 변환 → (축 단위벡터, 각도 rad(≥0), 축 위의 점)"""
    ang = float(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
    w, v = np.linalg.eig(R)
    ax = np.real(v[:, np.argmin(np.abs(w - 1))])
    ax /= np.linalg.norm(ax)
    # 축 방향 부호: 각도를 양수로 (반시계, 오른손 법칙)
    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    if np.abs(np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K - R).max() > 1e-6:
        ax = -ax
    p = np.linalg.lstsq(np.eye(3) - R, t, rcond=None)[0]
    return ax, ang, p


def _cylinder(c, r, half_w, n=48):
    """y축 원통 (base 좌표, m): 중심 c, 반지름 r, 폭/2"""
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = np.c_[r * np.cos(a), np.zeros(n), r * np.sin(a)]
    v = np.r_[ring + [0, -half_w, 0], ring + [0, half_w, 0], [[0, -half_w, 0], [0, half_w, 0]]] + c
    f = []
    for i in range(n):
        j = (i + 1) % n
        f += [[i, j, n + j], [i, n + j, n + i], [2 * n, j, i], [2 * n + 1, n + i, n + j]]
    return v.astype(np.float32), np.array(f, np.int32)


def build_step(up_path, down_path, out_root=None, force=False, log=print, deploy_time=4.0):
    """접이식 비콘 STEP 두 개(펴짐·접힘) → 링크별 메시 + 마스트 관절 + 센서 포즈 (robot.yaml).
    두 파일에서 같은 부품의 위치를 비교해 움직이는 부품(센서함)과 관절 축·각도를 찾는다."""
    from .step_import import read_step
    out_root = out_root or writable_root()
    print = log  # noqa: A001
    print(f'STEP 읽는 중: {up_path}')
    up = read_step(up_path, deflection=1.5, angle=0.8)
    print(f'STEP 읽는 중: {down_path}')
    dn = read_step(down_path, deflection=1.5, angle=0.8)
    if [p['path'] for p in up] != [p['path'] for p in dn]:
        raise ValueError('두 STEP 의 부품 구성이 다릅니다 (같은 조립품의 펴짐/접힘 상태여야 함)')

    def key(p):
        return '/'.join(p['path'])

    # --- 바퀴 → 지면·base_link 원점 (STEP mm: -X 전방, -Z 위)
    tires = [p for p in up if 'Outer Wheel' in key(p)]
    if len(tires) != 4:
        raise ValueError(f'바퀴(Outer Wheel) 4개를 찾지 못함: {len(tires)}')
    tb = [(p['tris'].reshape(-1, 3).min(0), p['tris'].reshape(-1, 3).max(0)) for p in tires]
    wc = np.array([(lo + hi) / 2 for lo, hi in tb])
    origin = np.array([wc[:, 0].mean(), wc[:, 1].mean(), max(hi[2] for _, hi in tb)])
    wheel_radius = float(np.mean([(hi[2] - lo[2]) for lo, hi in tb]) / 2000.0)
    wheelbase = float(wc[:, 0].max() - wc[:, 0].min()) / 1000.0
    track = float(wc[:, 1].max() - wc[:, 1].min()) / 1000.0

    # --- 움직이는 부품 분류 (접힘 → 펴짐 강체 변환)
    groups = []                                  # [{R, t, idx: [...]}]
    moving = {}
    for i, (a, b) in enumerate(zip(dn, up)):
        A = a['tris'].reshape(-1, 3).astype(np.float64)
        B = b['tris'].reshape(-1, 3).astype(np.float64)
        if np.abs(A - B).max() < 1e-3:
            continue
        R, t = _kabsch(A, B)
        moving[i] = (R, t)
        for g in groups:
            if np.abs(g['R'] - R).max() < 2e-2 and np.abs(g['t'] - t).max() < 5.0:   # ~1° · 5 mm 이내 = 같은 강체
                g['idx'].append(i)
                break
        else:
            groups.append({'R': R, 't': t, 'idx': [i]})
    if not groups:
        raise ValueError('두 STEP 에서 움직이는 부품을 찾지 못함')
    mast_g = max(groups, key=lambda g: sum(len(up[i]['tris']) for i in g['idx']))
    for g in [g for g in groups if g is not mast_g]:
        # 센서함 변환을 적용했을 때 30 mm 안으로 맞으면 센서함에 붙은 부품 (접힘 STEP 에서 조립 구속이 빠진 부품)
        # → 펴짐 위치 기준으로 센서함에 포함
        err = max(np.abs(dn[i]['tris'].reshape(-1, 3) @ mast_g['R'].T + mast_g['t'] - up[i]['tris'].reshape(-1, 3)).max()
                  for i in g['idx'])
        if err < 30.0:
            log(f'  센서함에 포함 (접힘 위치 차 {err:.1f} mm): {", ".join(sorted({up[i]["name"] for i in g["idx"]}))}')
            mast_g['idx'] += g['idx']
            groups = [x for x in groups if x is not g]
    M = np.diag([-1.0, 1.0, -1.0])               # STEP → base 회전 (Y축 180°)

    def to_base_T(R, t):                         # STEP 강체변환(mm) → base(m)
        Rb = M @ R @ M
        tb_ = (M @ (t + R @ origin - origin)) / 1000.0
        return Rb, tb_

    Rm, tm = to_base_T(mast_g['R'], mast_g['t'])
    axis, A, pivot = _axis_angle(Rm, tm)
    print(f'마스트 관절: 축 {axis.round(4)}, 각도 {np.degrees(A):.2f}°, 피벗(base) {pivot.round(4)} m, 부품 {len(mast_g["idx"])}개')
    links = {}
    for g in groups:
        if g is mast_g:
            continue
        nm = 'crank' if any('지지대1' in key(up[i]) for i in g['idx']) else \
            'coupler' if any('지지대2' in key(up[i]) for i in g['idx']) else f'link{len(links)}'
        Rl, tl = to_base_T(g['R'], g['t'])
        ax_l, ang_l, p_l = _axis_angle(Rl, tl)
        links[nm] = {'idx': g['idx'], 'axis': ax_l, 'angle': ang_l, 'point': p_l}
        print(f'  링크 {nm}: {ang_l * 180 / np.pi:.1f}° ({", ".join(sorted({up[i]["name"] for i in g["idx"]}))[:60]})')

    # --- 출력 부품: 이름·링크·메시 (펴짐 상태 base 좌표 → 링크 좌표)
    link_of = {}
    for i in mast_g['idx']:
        link_of[i] = 'mast'
    for nm, L in links.items():
        for i in L['idx']:
            link_of[i] = nm
    out = {}                                     # 부품 → [tris(base m)]
    part_link = {}
    for i, p in enumerate(up):
        k = key(p)
        if 'Wheel' in k or '608-ZZ' in k or (any(h in k for h in STEP_HIDDEN) and i not in link_of):
            continue
        link = link_of.get(i, 'base')
        name = next((n for n, keys in STEP_PARTS if any(x in k for x in keys)), None)
        if name is None:
            name = 'mast_body' if link == 'mast' else link if link != 'base' else 'body'
        if name in ('crank', 'coupler') or link not in ('base', 'mast'):
            name = link
        part_link[name] = link
        out.setdefault(name, []).append(stl_to_base(p['tris'], origin))
    mesh_dir = os.path.join(out_root, 'robot', 'meshes')
    os.makedirs(mesh_dir, exist_ok=True)
    for f in os.listdir(mesh_dir):                # 예전 부품 파일 정리
        if f.endswith('.stl'):
            os.remove(os.path.join(mesh_dir, f))
    npz, meta_parts = {}, {}
    for name, ts in out.items():
        t = np.concatenate(ts).astype(np.float64)
        link = part_link[name]
        if link == 'mast':
            t = t - pivot                         # mast_link 좌표 (펴짐 상태에서 축 방향은 base 와 같음)
        v, f = dedup(t)
        npz[f'{name}_v'], npz[f'{name}_f'] = v, f
        meta_parts[name] = {'link': link, 'color': PART_COLORS_V6.get(name, PART_COLORS_V6['body'])}
        write_binary_stl(os.path.join(mesh_dir, f'{name}.stl'), t)
    for k, (lo, hi) in enumerate(tb):             # 바퀴: 원통 (타이어 + 휠)
        c = stl_to_base((lo + hi) / 2, origin)
        r, hw = (hi[2] - lo[2]) / 2000.0, (hi[1] - lo[1]) / 2000.0
        for nm, rr, ww in [('tire', r, hw), ('rim', r * 0.68, hw * 1.08)]:
            v, f = _cylinder(c, rr, ww)
            key_ = f'{nm}{k}'
            npz[f'{key_}_v'], npz[f'{key_}_f'] = v, f
            meta_parts[key_] = {'link': 'base', 'color': PART_COLORS_V6[nm]}
            write_binary_stl(os.path.join(mesh_dir, f'{key_}.stl'), v[f])
    import json
    meta = {'version': 2, 'parts': meta_parts, 'mast_pivot': pivot.tolist(),
            'links': {nm: {'axis': L['axis'].tolist(), 'angle': L['angle'], 'point': L['point'].tolist()}
                      for nm, L in links.items()}}
    npz['meta'] = np.frombuffer(json.dumps(meta, ensure_ascii=False).encode(), np.uint8)
    np.savez_compressed(os.path.join(mesh_dir, 'robot_parts.npz'), **npz)
    print('메시: ' + ', '.join(f'{k}({len(npz[k + "_f"])})' for k in meta_parts))

    # --- 센서 포즈 (펴짐 상태, STEP mm → base m → mast_link)
    def bb(keys):
        P = np.concatenate([p['tris'].reshape(-1, 3) for p in up if any(x in key(p) for x in keys)])
        return P.min(0), P.max(0)
    lo, hi = bb(('Pandar40P',))
    c = (lo + hi) / 2
    pandar = [c[0], c[1], hi[2] - 47.7]          # 하단면 +47.7mm (측정 중심), -Z 가 위
    lo, hi = bb(('Tele-15',))
    c = (lo + hi) / 2
    tele = [lo[0], c[1], c[2]]                   # 전면 창 중심
    lo, hi = bb(('asr_408', 'ars_408', 'ARS408'))
    c = (lo + hi) / 2
    radar = [lo[0], c[1], c[2]]                  # 레이돔 전면 중심
    # 카메라 (Arducam B0495, 34 x 34 x 40.2 mm, 본체는 STEP 에 없음): 카메라 마운트 판의 고정 구멍 4개 중심에
    # 렌즈가 전방(-X)을 보게 앉힌다 → camera_link = 렌즈 앞면 중심
    from .step_import import find_holes
    holes = [h for h in find_holes(up_path, '카메라 마운트', 1.0, 1.5) if abs(h[2][2]) > 0.9]   # 판에 수직인 M2.5 구멍
    lo, hi = bb(('카메라 마운트',))
    if len(holes) >= 4:
        hc = np.mean([h[1] for h in holes], 0)
        plate_top = min(h[1][2] for h in holes) - 0.6        # 판 윗면 (-Z 가 위, 판 두께 1.2 mm)
        cam = [hc[0] - 40.2 / 2, hc[1], plate_top - 34.0 / 2]
        cam_note = (f'Arducam B0495(34x34x40.2mm)를 카메라 마운트 고정 구멍 {len(holes)}개 중심에 렌즈 전방으로 놓았을 때 '
                    '렌즈 앞면 중심 (STEP 에 카메라 본체 없음)')
    else:
        c = (lo + hi) / 2
        cam = [lo[0], c[1], lo[2] - 17.0]
        cam_note = 'STEP 에 카메라 본체·고정 구멍 없음 → 카메라 마운트 앞끝 위로 추정. 실측으로 고칠 것'
    print(f'카메라: 고정 구멍 {len(holes)}개, 렌즈 앞면(STEP mm) {np.round(cam, 1)}, 마운트 앞끝 x {lo[0]:.1f}')
    lo, hi = bb(('GNSS 안테나',))
    c = (lo + hi) / 2
    gnss = [c[0], c[1], lo[2]]                   # 안테나 윗면 중심
    # 마스트 기울기 (펴짐): Pandar 윗면 법선
    P = np.concatenate([p['tris'] for p in up if 'Pandar40P' in key(p)])
    n = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0])
    a_ = np.linalg.norm(n, axis=1)
    nn = n / np.maximum(a_, 1e-12)[:, None]
    m = np.abs(nn[:, 2]) > 0.99
    v = (nn[m] * np.sign(nn[m, 2])[:, None] * a_[m, None]).sum(0)
    v /= np.linalg.norm(v)
    pitch = float(np.arctan2(v[0], v[2]))
    poses = {k: stl_to_base(p_, origin) - pivot for k, p_ in
             [('pandar40p', pandar), ('tele15', tele), ('ars408', radar), ('camera', cam), ('gnss', gnss)]}
    print(f'base_link 원점(STEP mm) {origin.round(2)}  바퀴 반지름 {wheel_radius:.4f} m, 휠베이스 {wheelbase:.3f} m, '
          f'윤거 {track:.3f} m, 마스트 기울기 {np.degrees(pitch):.3f}°')
    for k, p_ in poses.items():
        print(f'  {k:10s} mast_link 기준 xyz={np.round(p_, 4)}  (펴짐 base {np.round(p_ + pivot, 4)})')

    cfg = os.path.join(out_root, 'config', 'robot.yaml')
    if os.path.exists(cfg) and not force:
        print(f'{cfg} 이미 있음 (덮어쓰려면 --force)')
        return cfg
    os.makedirs(os.path.dirname(cfg), exist_ok=True)
    frames = {'pandar40p': 'pandar_40p', 'tele15': 'livox_tele15', 'ars408': 'ars408', 'camera': 'camera_link',
              'gnss': 'gnss_link'}
    notes = {
        'pandar40p': '원통 축 중심, 하단면 +47.7mm (측정 중심). yaw(커넥터 방향)는 CAD 로 판별 불가 → 전방 가정',
        'tele15': '전면 창 중심. Livox 좌표 원점 정의(매뉴얼) 확인 필요',
        'ars408': '레이돔 전면 중심',
        'camera': cam_note,
        'gnss': 'ANN-MB 안테나 윗면 중심',
    }
    with open(cfg, 'w') as f:
        f.write('# tools/build_robot.py 가 CAD(STEP 펴짐·접힘)에서 측정해 생성. 수동 수정 가능.\n')
        f.write('# 좌표: base_link (바퀴 4개 중심의 지면 투영점, x 전방, y 좌, z 위), 단위 m / rad\n')
        f.write('# 센서는 mast_link 기준 (펴짐 상태에서 mast_link 축 = base_link 축, 원점 = 접이 피벗)\n')
        f.write(f'wheel_radius: {wheel_radius:.4f}\nwheelbase: {wheelbase:.4f}\ntrack: {track:.4f}\n')
        f.write('mast:\n  joint: mast_joint\n  frame_id: mast_link\n')
        f.write(f'  pivot: [{pivot[0]:.4f}, {pivot[1]:.4f}, {pivot[2]:.4f}]   # base_link 기준 접이 축 위치\n')
        f.write(f'  axis: [{axis[0]:.4f}, {axis[1]:.4f}, {axis[2]:.4f}]\n')
        f.write(f'  deployed: {A:.5f}        # 펴짐 각도 [rad] ({np.degrees(A):.2f}°). 0 = 접힘\n')
        f.write(f'  deploy_time: {deploy_time:.1f}      # 접힘 → 펴짐 시간 [s] (AK45-36 구동, 실측으로 고칠 것)\n')
        f.write('sensors:\n')
        for k, p_ in poses.items():
            f.write(f'  {k}:\n    frame_id: {frames[k]}\n    parent: mast_link\n')
            f.write(f'    xyz: [{p_[0]:.4f}, {p_[1]:.4f}, {p_[2]:.4f}]\n')
            f.write(f'    rpy: [0.0, {pitch:.5f}, 0.0]\n')
            f.write(f'    note: "{notes[k]}"\n')
    print(f'생성: {cfg}')
    return cfg


def main():
    ap = argparse.ArgumentParser(description='비콘 CAD(STL) → 로봇 메시 + 센서 장착 포즈')
    ap.add_argument('--stl', default=os.path.join(ROOT, '절곡 프레임 설계 v2.stl'))
    ap.add_argument('--out', default=None, help='출력 루트 (기본: 쓰기 가능하면 저장소, 아니면 ~/.config/beaconsim)')
    ap.add_argument('--force', action='store_true', help='config/robot.yaml 덮어쓰기')
    ap.add_argument('--step-up', default=None, help='접이식 비콘 STEP (펴짐)')
    ap.add_argument('--step-down', default=None, help='접이식 비콘 STEP (접힘)')
    a = ap.parse_args()
    if a.step_up or a.step_down:
        build_step(a.step_up, a.step_down, a.out, a.force)
    else:
        build(a.stl, a.out, a.force)


if __name__ == '__main__':
    main()
