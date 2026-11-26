"""도로 모델 → Autoware 맵 세트 내보내기와 검증.

출력 디렉터리:
  lanelet2_map.osm         Lanelet2 (MGRS 로컬 좌표 local_x/local_y/ele + 위경도)
  map_projector_info.yaml  projector_type: MGRS
  pointcloud_map.pcd       씬 메시 표면 샘플링 점군 (x y z intensity, binary)
  map_meta.json            시드/파라미터/생성기 버전/요약 (재현용)
"""
import json
import os
import subprocess
import time
from xml.sax.saxutils import escape

import numpy as np

from . import scene
from .roadgen import GENERATOR_VERSION, RoadModel, local_to_latlon

MAP_DS = 5.0            # 맵 노드 간격 [m]
CHUNK = 100.0           # lanelet 분할 길이 [m]
VALIDATE_BIN = '/opt/ros/humble/lib/x86_64-linux-gnu/lanelet2_validation/lanelet2_validate'

LINE_TAGS = {
    'yellow_solid': {'type': 'line_thin', 'subtype': 'solid', 'color': 'yellow'},
    'white_solid': {'type': 'line_thin', 'subtype': 'solid', 'color': 'white'},
    'white_dashed': {'type': 'line_thin', 'subtype': 'dashed', 'color': 'white'},
    'road_border': {'type': 'road_border'},
}


class _Osm:
    def __init__(self):
        self.nodes, self.ways, self.rels = [], [], []
        self.next_id = 1

    def _id(self):
        i = self.next_id
        self.next_id += 1
        return i

    def add_nodes(self, xyz, lat, lon):
        ids = []
        for (x, y, z), la, lo in zip(xyz, lat, lon):
            i = self._id()
            self.nodes.append((i, la, lo, x, y, z))
            ids.append(i)
        return ids

    def add_way(self, refs, tags):
        i = self._id()
        self.ways.append((i, refs, tags))
        return i

    def add_rel(self, members, tags):
        i = self._id()
        self.rels.append((i, members, tags))
        return i

    def write(self, path):
        def tags_xml(tags, ind):
            return ''.join(f'{ind}<tag k="{escape(k)}" v="{escape(str(v))}"/>\n' for k, v in tags.items())
        with open(path, 'w') as f:
            f.write('<?xml version="1.0" encoding="UTF-8"?>\n<osm version="0.6" generator="BeaconSim">\n')
            for i, la, lo, x, y, z in self.nodes:
                f.write(f'  <node id="{i}" lat="{la:.11f}" lon="{lo:.11f}">\n'
                        f'    <tag k="local_x" v="{x:.4f}"/>\n    <tag k="local_y" v="{y:.4f}"/>\n'
                        f'    <tag k="ele" v="{z:.4f}"/>\n  </node>\n')
            for i, refs, tags in self.ways:
                f.write(f'  <way id="{i}">\n')
                f.write(''.join(f'    <nd ref="{r}"/>\n' for r in refs))
                f.write(tags_xml(tags, '    '))
                f.write('  </way>\n')
            for i, members, tags in self.rels:
                f.write(f'  <relation id="{i}">\n')
                for typ, ref, role in members:
                    f.write(f'    <member type="{typ}" ref="{ref}" role="{role}"/>\n')
                f.write(tags_xml(tags, '    '))
                f.write('  </relation>\n')
            f.write('</osm>\n')


def write_lanelet2(m: RoadModel, path):
    """방향별로 경계선(노드 공유) → 100m 단위 linestring → lanelet. 반환: 통계 dict."""
    p = m.params
    L = m.length
    s = np.unique(np.r_[np.arange(0, L, MAP_DS), np.arange(0, L, CHUNK), L])
    breaks = np.unique(np.r_[np.arange(0, L, CHUNK), L])
    if breaks[-1] - breaks[-2] < 20:                      # 너무 짧은 마지막 조각은 합침
        breaks = np.r_[breaks[:-2], L]
    bidx = np.searchsorted(s, breaks)
    osm = _Osm()
    lines = m.lines()
    n_llt = 0
    for side in (-1, 1):
        order = slice(None) if side < 0 else slice(None, None, -1)      # 주행 방향 순서
        node_ids = {}
        for ln in lines:
            xyz = m.point(s, side * ln.offset)
            lat, lon = local_to_latlon(m, xyz[:, 0], xyz[:, 1])
            node_ids[ln.key] = np.array(osm.add_nodes(xyz, lat, lon))
        ways = {ln.key: [] for ln in lines}
        for k in range(len(breaks) - 1):
            a, b = bidx[k], bidx[k + 1]
            for ln in lines:
                refs = list(node_ids[ln.key][a:b + 1][order])
                ways[ln.key].append(osm.add_way(refs, LINE_TAGS[ln.kind]))
        if side > 0:                                       # 상행은 주행 순서대로 조각 순서도 뒤집기
            ways = {k: v[::-1] for k, v in ways.items()}
        keys = [ln.key for ln in lines]                    # 중앙 → 바깥 순서
        lanes = []                                         # (left_key, right_key, subtype)
        lanes.append((keys[0], keys[1], 'road_shoulder'))
        for i in range(p.lanes):
            lanes.append((keys[1 + i], keys[2 + i], 'road'))
        lanes.append((keys[-2], keys[-1], 'road_shoulder'))
        for k in range(len(breaks) - 1):
            for lk, rk, sub in lanes:
                osm.add_rel([('way', ways[lk][k], 'left'), ('way', ways[rk][k], 'right')],
                            {'type': 'lanelet', 'subtype': sub, 'location': 'nonurban', 'one_way': 'yes',
                             'speed_limit': str(p.design_speed), 'participant:vehicle': 'yes'})
                n_llt += 1
    osm.write(path)
    return {'nodes': len(osm.nodes), 'ways': len(osm.ways), 'lanelets': n_llt}


def write_projector_info(m: RoadModel, path):
    with open(path, 'w') as f:
        f.write(f'projector_type: MGRS\nvertical_datum: WGS84\nmgrs_grid: {m.mgrs_grid}\n')


def write_pcd(m: RoadModel, path, voxel=0.25, seed=0, parts=None):
    """정적 씬 표면을 면적 비례로 샘플링 → 복셀 다운샘플 → binary PCD (x y z intensity)."""
    rng = np.random.default_rng(seed)
    parts = parts if parts is not None else scene.build_static(m)
    pts, inten = [], []
    for part in parts:
        dens = 2.0 if part.cls == 'terrain' else 1.0 / (voxel * voxel) * 1.5
        tri = part.v[part.f].astype(np.float64)
        area = 0.5 * np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1)
        n = rng.poisson(area * dens)
        if n.sum() == 0:
            continue
        ti = np.repeat(np.arange(len(tri)), n)
        r1, r2 = rng.random(len(ti)), rng.random(len(ti))
        sq = np.sqrt(r1)
        w0, w1, w2 = 1 - sq, sq * (1 - r2), sq * r2
        P = w0[:, None] * tri[ti, 0] + w1[:, None] * tri[ti, 1] + w2[:, None] * tri[ti, 2]
        pts.append(P)
        inten.append(np.full(len(P), part.retro * 100.0))
    P = np.concatenate(pts)
    I = np.concatenate(inten)
    key = np.floor(P / voxel).astype(np.int64)
    _, keep = np.unique(key, axis=0, return_index=True)
    P, I = P[keep], I[keep]
    arr = np.zeros(len(P), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4')])
    arr['x'], arr['y'], arr['z'], arr['intensity'] = P[:, 0], P[:, 1], P[:, 2], I
    header = ('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z intensity\n'
              'SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n'
              f'WIDTH {len(arr)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(arr)}\nDATA binary\n')
    with open(path, 'wb') as f:
        f.write(header.encode())
        f.write(arr.tobytes())
    return len(arr)


def validate(m: RoadModel, osm_path):
    """1) lanelet2_validate (있으면)  2) Python lanelet2 로드 + 라우팅 그래프 점검."""
    report = {'lanelet2_validate': None, 'issues': [], 'ok': True}
    lat0, lon0 = local_to_latlon(m, m.x[0], m.y[0])
    if os.path.exists(VALIDATE_BIN):
        r = subprocess.run([VALIDATE_BIN, '--map_file', osm_path, '--lat', str(lat0), '--lon', str(lon0)],
                           capture_output=True, text=True, timeout=120)
        lines = (r.stdout + r.stderr).strip().splitlines()
        # Autoware에서 정상인 태그 (lanelet2 core가 모를 뿐): local_x/local_y, subtype=road_shoulder
        expected = [ln for ln in lines if 'local_x is not a known' in ln or 'local_y is not a known' in ln
                    or 'road_shoulder is not a known' in ln]
        other = [ln for ln in lines if ln not in expected and 'issues found' not in ln and ln.strip()]
        report['lanelet2_validate'] = {'expected_warnings': len(expected), 'other': other[:30]}
        if any('Error' in ln for ln in other):
            report['ok'] = False
        report['issues'] += [f'lanelet2_validate: {ln}' for ln in other[:10]]
    try:
        import lanelet2
        from lanelet2.io import Origin
        from lanelet2.projection import UtmProjector
        lmap, errs = lanelet2.io.loadRobust(osm_path, UtmProjector(Origin(float(lat0), float(lon0))))
        if errs:
            report['issues'] += [f'load: {e}' for e in errs[:10]]
            report['ok'] = False
        rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                              lanelet2.traffic_rules.Participants.Vehicle)
        graph = lanelet2.routing.RoutingGraph(lmap, rules)
        roads = [llt for llt in lmap.laneletLayer if llt.attributes['subtype'] == 'road']
        report['road_lanelets'] = len(roads)
        bad = graph.checkValidity()
        if bad:
            report['issues'].append(f'routing graph: {bad}')
            report['ok'] = False
        # 방향별 시작 → 끝 경로 존재, 점선 차로 간 차로변경 가능 여부
        n_lc = 0
        for llt in roads:
            if graph.left(llt) is not None or graph.right(llt) is not None:
                n_lc += 1
        report['lane_change_lanelets'] = n_lc
        if m.params.lanes > 1 and n_lc == 0:
            report['issues'].append('차로변경 가능한 lanelet이 없음')
            report['ok'] = False
        for side in (-1, 1):
            s_a, s_b = (5.0, m.length - 5.0) if side < 0 else (m.length - 5.0, 5.0)
            t = m.lane_center_t(0, side)
            pa, pb = m.point(s_a, t), m.point(s_b, t)
            la = _nearest_lanelet(lmap, roads, pa, lat0, lon0, m)
            lb = _nearest_lanelet(lmap, roads, pb, lat0, lon0, m)
            route = graph.getRoute(la, lb) if la is not None and lb is not None else None
            ok = route is not None
            report[f'route_{"down" if side < 0 else "up"}'] = ok
            if not ok:
                report['issues'].append(f'{"하행" if side < 0 else "상행"} 시작→끝 경로 없음')
                report['ok'] = False
    except Exception as e:      # noqa: BLE001
        report['issues'].append(f'python lanelet2 점검 실패: {e}')
    return report


def _nearest_lanelet(lmap, roads, p_local, lat0, lon0, m):
    """local 좌표 점 → UtmProjector(origin) 좌표로 바꿔 가장 가까운 road lanelet."""
    import lanelet2
    from lanelet2.io import Origin
    from lanelet2.projection import UtmProjector
    lat, lon = local_to_latlon(m, p_local[0], p_local[1])
    proj = UtmProjector(Origin(float(lat0), float(lon0)))
    q = proj.forward(lanelet2.core.GPSPoint(float(lat), float(lon), 0.0))
    best, bd = None, 1e9
    for llt in roads:
        c = llt.centerline
        d = min((pt.x - q.x) ** 2 + (pt.y - q.y) ** 2 for pt in c)
        if d < bd:
            best, bd = llt, d
    return best


def export_all(m: RoadModel, out_dir, log=print, pcd=True, parts=None):
    os.makedirs(out_dir, exist_ok=True)
    t0 = time.time()
    osm_path = os.path.join(out_dir, 'lanelet2_map.osm')
    st = write_lanelet2(m, osm_path)
    log(f'OSM 저장: 노드 {st["nodes"]}, way {st["ways"]}, lanelet {st["lanelets"]}')
    write_projector_info(m, os.path.join(out_dir, 'map_projector_info.yaml'))
    log(f'map_projector_info.yaml: MGRS {m.mgrs_grid}')
    n_pcd = None
    if pcd:
        n_pcd = write_pcd(m, os.path.join(out_dir, 'pointcloud_map.pcd'), parts=parts)
        log(f'PCD 저장: {n_pcd:,} 점')
    rep = validate(m, osm_path)
    log('검증: ' + ('통과' if rep['ok'] else '문제 있음') + (f' — {rep["issues"]}' if rep['issues'] else ''))
    meta = {'generator': 'BeaconSim roadgen', 'generator_version': GENERATOR_VERSION,
            'params': m.params.to_dict(), 'summary': m.summary(), 'osm': st, 'pcd_points': n_pcd,
            'validation': rep, 'created': time.strftime('%Y-%m-%d %H:%M:%S')}
    with open(os.path.join(out_dir, 'map_meta.json'), 'w') as f:
        json.dump(meta, f, ensure_ascii=False, indent=2, default=str)
    log(f'완료 ({time.time() - t0:.1f}s): {out_dir}')
    return meta


if __name__ == '__main__':
    import argparse
    from .roadgen import RoadParams, generate
    ap = argparse.ArgumentParser(description='시드로 고속도로 생성 후 Autoware 맵 세트 저장')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--length', type=float, default=3000.0)
    ap.add_argument('--lanes', type=int, default=2)
    ap.add_argument('--speed', type=int, default=100, choices=[100, 120])
    ap.add_argument('--tunnels', type=int, default=1)
    ap.add_argument('--no-pcd', action='store_true')
    ap.add_argument('out')
    a = ap.parse_args()
    model = generate(RoadParams(seed=a.seed, length=a.length, lanes=a.lanes, design_speed=a.speed, tunnels=a.tunnels))
    export_all(model, a.out, pcd=not a.no_pcd)
