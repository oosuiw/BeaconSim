"""시나리오 자동 평가: 지표 수집 + 합격 기준 판정.

수집 (항상)
  안전   : 추돌 수, 비콘 추돌, 비콘 쪽 접근 차량의 최소 TTC·최소 간격(비콘 차로), 비콘 통과 속도·횡간격,
           비콘 인지 거리, 차로변경/감속/SCC 인식 수, 비콘 앞 정지 차량 수
  비콘   : 파손 여부, 접촉 횟수, 이동 거리, 사고 지점까지 최종 거리
  GNSS   : 상태별 시간 비율, NO FIX 구간, 재수렴(NO FIX 해제 → RTK FIX) 시간
수집 (토픽 연결 시, --ros)
  측위   : 정답 대비 횡/종/헤딩 오차 (전체, 터널 구간)
  차선검출: 정답 라벨 대비 정밀도/재현율 (반경 0.2 m 매칭, 40 m 이내)
"""
import math

import numpy as np

from .gnss import FIX, NONE
from .scene import CLASS


class Metrics:
    def __init__(self, engine):
        self.e = engine
        self.min_ttc = math.inf
        self.min_gap_lane = math.inf
        self.pass_speed, self.pass_clear = [], []
        self.passed = set()
        self.prev_rel = {}
        self.stopped_behind = 0
        self.contacts = 0
        self.dist_travel = 0.0
        self.prev_xy = None
        self.gnss_time = {}
        self.gnss_last_t = None
        self.no_fix_segments = []
        self.reconv = []
        self._nf_start = None
        self._nf_end = None
        self.loc_err = []          # (t, lat, lon, yaw, in_tunnel)
        self.lane_pr = []          # (precision, recall)
        self.samples = []          # 시계열 (CSV)

    # 엔진 스레드/루프에서 주기 호출
    def update(self, fired=None, events=()):
        e = self.e
        tr, scn = e.traffic, e.scn
        for ev in events:
            if ev[1] == 'beacon_contact':
                self.contacts += 1
        xy = np.array([e.x, e.y])
        if self.prev_xy is not None:
            self.dist_travel += float(np.linalg.norm(xy - self.prev_xy))
        self.prev_xy = xy
        bu = scn.dir * e.s_b
        lane_w = e.model.params.lane_width
        idx = np.nonzero(tr.active & (tr.side == scn.side))[0]
        n_stop = 0
        for i in idx:
            front = tr.u[i] + tr.len[i] / 2
            rel = bu - front                             # 비콘까지 (양수 = 아직 앞에 있음)
            t_i = tr._lane_t(scn.side, tr.lane[i])
            same_lane = abs(t_i - e.t_b) < lane_w / 2
            if same_lane and rel > 0:
                self.min_gap_lane = min(self.min_gap_lane, rel)
                closing = tr.v[i] - tr.beacon_vu
                if closing > 0.1:
                    self.min_ttc = min(self.min_ttc, rel / closing)
                if tr.v[i] < 0.5 and rel < 30:
                    n_stop += 1
            prev = self.prev_rel.get(int(i))
            if prev is not None and prev > 0 >= rel and int(i) not in self.passed and not tr.crashed[i]:
                self.passed.add(int(i))
                self.pass_speed.append(float(tr.v[i]) * 3.6)
                self.pass_clear.append(float(abs(t_i - e.t_b) - tr.wid[i] / 2 - e.half_wid))
            self.prev_rel[int(i)] = rel
        self.stopped_behind = max(self.stopped_behind, n_stop)
        g = (fired or {}).get('gnss')
        if g is not None:
            if self.gnss_last_t is not None:
                dt = g['t'] - self.gnss_last_t
                self.gnss_time[g['status']] = self.gnss_time.get(g['status'], 0.0) + dt
            self.gnss_last_t = g['t']
            if g['status'] == NONE and self._nf_start is None:
                self._nf_start = g['t']
            elif g['status'] != NONE and self._nf_start is not None:
                self.no_fix_segments.append((self._nf_start, g['t']))
                self._nf_start, self._nf_end = None, g['t']
            if g['status'] == FIX and self._nf_end is not None:
                self.reconv.append(g['t'] - self._nf_end)
                self._nf_end = None
        c = tr.counts
        self.samples.append((round(e.t, 2), round(e.x, 3), round(e.y, 3), round(e.s_b, 2), round(e.t_b, 3),
                             round(e.v, 2), int(tr.active.sum()), c['noticed'], c['lane_change'], c['brake'],
                             c['scc_lock'], c['crash'], c['beacon_hit'], int(e.damaged),
                             g['status'] if g is not None else ''))

    # ---- 선택 지표 (ROS 토픽)
    def add_localization(self, t, x, y, yaw):
        """측위 결과 (map 좌표) → 정답과 비교 (도로 기준 횡/종 오차)."""
        e = self.e
        T = e.T_wb
        th = math.atan2(T[1, 0], T[0, 0])
        dx, dy = x - T[0, 3], y - T[1, 3]
        lon = dx * math.cos(th) + dy * math.sin(th)
        lat = -dx * math.sin(th) + dy * math.cos(th)
        dyaw = (yaw - th + math.pi) % (2 * math.pi) - math.pi
        self.loc_err.append((t, lat, lon, dyaw, bool(e.model.in_tunnel(e.s_b))))

    def add_lane_detection(self, det_xy, gt):
        """det_xy (N,2) 검출 차선 점 (Pandar 좌표), gt: 같은 프레임 정답 라벨 점군."""
        lane = np.isin(gt['label'], [CLASS['lane_white'], CLASS['lane_yellow']])
        g = np.c_[gt['x'][lane], gt['y'][lane]]
        g = g[np.hypot(g[:, 0], g[:, 1]) < 40]
        d = det_xy[np.hypot(det_xy[:, 0], det_xy[:, 1]) < 40] if len(det_xy) else det_xy
        if len(g) == 0 or len(d) == 0:
            return
        from scipy.spatial import cKDTree
        tp_d = (cKDTree(g).query(d, distance_upper_bound=0.2)[0] < 0.2).mean()
        tp_g = (cKDTree(d).query(g, distance_upper_bound=0.2)[0] < 0.2).mean()
        self.lane_pr.append((float(tp_d), float(tp_g)))

    # ---- 결과
    def result(self):
        e = self.e
        tr = e.traffic
        c = tr.counts
        total = sum(self.gnss_time.values()) or 1.0
        nd = list(tr.notice_at.values())
        r = {
            'crashes': c['crash'], 'beacon_hits': c['beacon_hit'], 'noticed': c['noticed'],
            'lane_changes': c['lane_change'], 'brakes': c['brake'], 'scc_locks': c['scc_lock'],
            'vehicles_passed': len(self.passed),
            'min_ttc_s': None if math.isinf(self.min_ttc) else round(self.min_ttc, 2),
            'min_gap_in_beacon_lane_m': None if math.isinf(self.min_gap_lane) else round(self.min_gap_lane, 1),
            'pass_speed_max_kph': round(max(self.pass_speed), 1) if self.pass_speed else None,
            'pass_speed_mean_kph': round(float(np.mean(self.pass_speed)), 1) if self.pass_speed else None,
            'pass_clearance_min_m': round(min(self.pass_clear), 2) if self.pass_clear else None,
            'notice_dist_mean_m': round(float(np.mean(nd)), 1) if nd else None,
            'notice_dist_min_m': round(float(np.min(nd)), 1) if nd else None,
            'max_stopped_behind_beacon': self.stopped_behind,
            'beacon_damaged': bool(e.damaged), 'beacon_contacts': self.contacts,
            'beacon_travel_m': round(self.dist_travel, 1),
            'beacon_to_accident_m': round(abs(e.s_b - e.scn.accident_s), 1),
            'gnss_fix_ratio': round(self.gnss_time.get(FIX, 0.0) / total, 3),
            'gnss_nofix_s': round(sum(b - a for a, b in self.no_fix_segments), 1),
            'gnss_reconverge_max_s': round(max(self.reconv), 1) if self.reconv else None,
        }
        if self.loc_err:
            a = np.array([[x[1], x[2], x[3]] for x in self.loc_err])
            tun = np.array([x[4] for x in self.loc_err])
            r.update({'loc_lat_rms_m': round(float(np.sqrt((a[:, 0] ** 2).mean())), 3),
                      'loc_lat_max_m': round(float(np.abs(a[:, 0]).max()), 3),
                      'loc_lon_rms_m': round(float(np.sqrt((a[:, 1] ** 2).mean())), 3),
                      'loc_yaw_max_deg': round(float(np.degrees(np.abs(a[:, 2]).max())), 2),
                      'loc_lat_max_tunnel_m': round(float(np.abs(a[tun, 0]).max()), 3) if tun.any() else None})
        if self.lane_pr:
            p = np.array(self.lane_pr)
            r.update({'lane_precision': round(float(p[:, 0].mean()), 3), 'lane_recall': round(float(p[:, 1].mean()), 3)})
        return r


# 시계열 CSV 열 (Metrics.samples)
COLS = ['t', 'x', 'y', 's', 't_lat', 'v', 'vehicles', 'noticed', 'lane_change', 'brake', 'scc_lock', 'crash',
        'beacon_hit', 'damaged', 'gnss_status']

# 기준 키 → (지표 키, 비교)
CRITERIA = {
    'max_crashes': ('crashes', '<='), 'max_beacon_hits': ('beacon_hits', '<='),
    'min_ttc_s': ('min_ttc_s', '>='), 'max_pass_speed_kph': ('pass_speed_max_kph', '<='),
    'min_pass_clearance_m': ('pass_clearance_min_m', '>='), 'min_notice_dist_m': ('notice_dist_min_m', '>='),
    'beacon_not_damaged': ('beacon_damaged', 'false'), 'max_beacon_contacts': ('beacon_contacts', '<='),
    'min_scc_locks': ('scc_locks', '>='), 'min_gnss_fix_ratio': ('gnss_fix_ratio', '>='),
    'max_gnss_reconverge_s': ('gnss_reconverge_max_s', '<='),
    'max_loc_lat_rms_m': ('loc_lat_rms_m', '<='), 'max_loc_lat_m': ('loc_lat_max_m', '<='),
    'max_loc_lat_tunnel_m': ('loc_lat_max_tunnel_m', '<='),
    'min_lane_precision': ('lane_precision', '>='), 'min_lane_recall': ('lane_recall', '>='),
}


def judge(result, criteria):
    """→ [(기준, 기대, 실제, 'PASS'|'FAIL'|'N/A')], 전체 통과 여부 (N/A는 통과로 보지 않고 별도 표기)."""
    rows, ok = [], True
    for k, want in (criteria or {}).items():
        if k not in CRITERIA:
            rows.append((k, want, None, '알 수 없는 기준'))
            ok = False
            continue
        key, op = CRITERIA[k]
        got = result.get(key)
        if got is None:
            rows.append((k, want, None, 'N/A'))
            continue
        if op == '<=':
            st = got <= want
        elif op == '>=':
            st = got >= want
        else:                                  # 'false': want=True 이면 값이 False여야 통과
            st = (not got) if want else True
            rows.append((k, '파손 없음' if want else '-', '파손' if got else '정상', 'PASS' if st else 'FAIL'))
            ok &= bool(st)
            continue
        rows.append((k, want, got, 'PASS' if st else 'FAIL'))
        ok &= bool(st)
    return rows, ok


# GUI 표시용 이름 (순서 = 편집 화면 순서)
CRITERIA_KO = {
    'max_beacon_hits': '비콘 추돌 수 ≤', 'beacon_not_damaged': '비콘 파손 없음', 'max_beacon_contacts': '비콘 접촉 수 ≤',
    'min_ttc_s': '비콘 차로 최소 TTC ≥ [s]', 'max_crashes': '전체 추돌 수 ≤', 'max_pass_speed_kph': '비콘 통과 속도 ≤ [km/h]',
    'min_pass_clearance_m': '통과 횡간격 ≥ [m]', 'min_notice_dist_m': '운전자 인지 거리 ≥ [m]',
    'min_scc_locks': 'SCC 인식 수 ≥', 'min_gnss_fix_ratio': 'GNSS FIX 비율 ≥', 'max_gnss_reconverge_s': 'GNSS 재수렴 ≤ [s]',
    'max_loc_lat_rms_m': '측위 횡오차 RMS ≤ [m]', 'max_loc_lat_m': '측위 횡오차 최대 ≤ [m]',
    'max_loc_lat_tunnel_m': '터널 측위 횡오차 최대 ≤ [m]', 'min_lane_precision': '차선검출 정밀도 ≥',
    'min_lane_recall': '차선검출 재현율 ≥',
}
# 예시값 (계획서 6장: 합격 기준 미정 → 실차 요구사항에서 역산해 수정)
DEFAULT_CRITERIA = {'max_beacon_hits': 0, 'beacon_not_damaged': True, 'max_beacon_contacts': 0, 'min_ttc_s': 2.0}
CRITERIA_DEFAULT_VALUE = {
    'max_beacon_hits': 0, 'beacon_not_damaged': True, 'max_beacon_contacts': 0, 'min_ttc_s': 2.0, 'max_crashes': 0,
    'max_pass_speed_kph': 80.0, 'min_pass_clearance_m': 1.0, 'min_notice_dist_m': 100.0, 'min_scc_locks': 1,
    'min_gnss_fix_ratio': 0.8, 'max_gnss_reconverge_s': 60.0, 'max_loc_lat_rms_m': 0.1, 'max_loc_lat_m': 0.3,
    'max_loc_lat_tunnel_m': 0.3, 'min_lane_precision': 0.8, 'min_lane_recall': 0.8,
}


def subscribe_eval(node, met, eng, topics):
    """측위(Odometry/PoseStamped/PoseWithCovarianceStamped)·차선검출(PointCloud2) 토픽 구독 → Metrics."""
    subs = []
    loc = (topics or {}).get('localization')
    if loc:
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
        from nav_msgs.msg import Odometry

        def pose_of(m):
            p = m.pose.pose if hasattr(m.pose, 'pose') else m.pose
            q = p.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            met.add_localization(eng.t, p.position.x, p.position.y, yaw)
        typ = {'odom': Odometry, 'pose': PoseStamped, 'pose_cov': PoseWithCovarianceStamped}[
            topics.get('localization_type', 'odom')]
        subs.append(node.create_subscription(typ, loc, pose_of, 10))
    lane = (topics or {}).get('lane_detection')
    if lane:
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs_py import point_cloud2

        def lane_cb(m):
            gt = eng.latest.get('pandar40p', {}).get('gt')
            if gt is None:
                return
            a = point_cloud2.read_points(m, field_names=['x', 'y'])
            met.add_lane_detection(np.c_[a['x'], a['y']], gt)
        subs.append(node.create_subscription(PointCloud2, lane, lane_cb, 5))
    return subs
