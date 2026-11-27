"""SimEngine ↔ ROS2 브리지 (rclpy 있을 때만). GUI 앱과 헤드리스 sim_core가 함께 쓴다.

발행: /clock, TF map→base_link, /ground_truth/odom, 센서(PointCloud2/Radar/NavSatFix),
      /sim/traffic, /sim/events, /sim/scenario, (옵션) RViz용 마커
구독: /cmd_vel → engine.set_cmd('ros', ...)
"""
import array
import json

import numpy as np
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Point, Pose, PoseArray, TransformStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from radar_msgs.msg import RadarReturn, RadarScan, RadarTrack, RadarTracks
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, NavSatFix, NavSatStatus, PointCloud2, PointField
from std_msgs.msg import Bool, ColorRGBA, Header, String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from unique_identifier_msgs.msg import UUID
from visualization_msgs.msg import Marker, MarkerArray

import numpy as _np

from .common import T_CAMLINK_OPTICAL, R_to_quat, make_T, quats_from_R
from .gnss import FIX, FLOAT, NONE, SINGLE

T0_NS = 1_000_000_000          # /clock 시작 오프셋 (0은 '미설정'으로 취급되는 노드가 있어 피함)

_NP2PF = {np.dtype('float32'): PointField.FLOAT32, np.dtype('float64'): PointField.FLOAT64,
          np.dtype('uint8'): PointField.UINT8, np.dtype('uint16'): PointField.UINT16,
          np.dtype('uint32'): PointField.UINT32, np.dtype('int32'): PointField.INT32}


def to_pc2(arr, frame_id, stamp):
    msg = PointCloud2()
    msg.header = Header(frame_id=frame_id, stamp=stamp)
    msg.height, msg.width = 1, len(arr)
    msg.fields = [PointField(name=n, offset=arr.dtype.fields[n][1], datatype=_NP2PF[arr.dtype.fields[n][0]], count=1)
                  for n in arr.dtype.names]
    msg.is_bigendian = False
    msg.point_step = arr.dtype.itemsize
    msg.row_step = msg.point_step * len(arr)
    msg.is_dense = True
    msg.data = array.array('B', arr.tobytes())   # bytes 대입은 rclpy가 원소별 검사해서 매우 느림
    return msg


class SimClock:
    """ROS 시각 = 시작값 + offset + 엔진 시각. 재시작(새 엔진, t=0)해도 /clock 이 뒤로 가지 않게 offset 을 올린다
    (뒤로 가면 RViz2 등이 'jump back in time' 으로 TF 버퍼를 지우고 한동안 메시지를 버린다)."""
    offset = 0.0
    last = 0.0          # 지금까지 발행한 가장 늦은 시각 (offset 포함)

    @classmethod
    def new_run(cls, engine_t=0.0):
        cls.offset = cls.last + 0.1 - engine_t


def t_to_msg(t):
    ns = int(round((t + SimClock.offset) * 1e9)) + T0_NS
    return TimeMsg(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def pose_to_tf(T, parent, child, stamp):
    m = TransformStamped()
    m.header.frame_id, m.header.stamp, m.child_frame_id = parent, stamp, child
    m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = map(float, T[:3, 3])
    q = R_to_quat(T[:3, :3])
    m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = map(float, q)
    return m


def mesh_marker(part, ns, mid, frame):
    mk = Marker()
    mk.header.frame_id = frame
    mk.ns, mk.id, mk.type, mk.action = ns, mid, Marker.TRIANGLE_LIST, Marker.ADD
    mk.pose.orientation.w = 1.0
    mk.scale.x = mk.scale.y = mk.scale.z = 1.0
    c = part.color
    mk.color = ColorRGBA(r=float(c[0]), g=float(c[1]), b=float(c[2]), a=1.0)
    tri = part.v[part.f].reshape(-1, 3)
    mk.points = [Point(x=float(a), y=float(b), z=float(z)) for a, b, z in tri]
    return mk


def box_marker(frame, ns, mid, sp, T, rgba, stamp=None):
    mk = Marker()
    mk.header.frame_id = frame
    if stamp is not None:
        mk.header.stamp = stamp
    mk.ns, mk.id, mk.type, mk.action = ns, mid, Marker.CUBE, Marker.ADD
    c = T[:3, 3] + T[:3, 2] * sp.height / 2
    mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = map(float, c)
    q = R_to_quat(T[:3, :3])
    mk.pose.orientation.x, mk.pose.orientation.y, mk.pose.orientation.z, mk.pose.orientation.w = map(float, q)
    mk.scale = Vector3(x=float(sp.length), y=float(sp.width), z=float(sp.height))
    mk.color = ColorRGBA(r=float(rgba[0]), g=float(rgba[1]), b=float(rgba[2]), a=float(rgba[3]))
    return mk


class RosBridge:
    def __init__(self, node, engine, cfg, markers=True, clock=True, static_tf=True):
        self.node, self.e, self.cfg = node, engine, cfg
        self.static_tf = static_tf
        self.markers, self.clock = markers, clock
        sc = cfg['sim']
        self.map_frame, self.base_frame = sc['map_frame'], sc['base_frame']
        qs = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cp = node.create_publisher
        self.pub = {
            'clock': cp(Clock, '/clock', 10), 'odom': cp(Odometry, sc['odom_topic'], 10),
            'traffic': cp(PoseArray, sc['traffic_topic'], 10), 'events': cp(String, sc['events_topic'], 50),
            'scenario': cp(String, sc['scenario_topic'], latched),
            'pandar': cp(PointCloud2, cfg['pandar40p']['topic'], qs),
            'pandar_gt': cp(PointCloud2, cfg['pandar40p']['gt_topic'], qs),
            'tele': cp(PointCloud2, cfg['tele15']['topic'], qs),
            'radar_pc': cp(PointCloud2, cfg['ars408']['cluster_topic'], qs),
            'radar_scan': cp(RadarScan, cfg['ars408']['scan_topic'], 10),
            'radar_tracks': cp(RadarTracks, cfg['ars408']['track_topic'], 10),
            'gnss': cp(NavSatFix, cfg.get('gnss', {}).get('topic', '/sensing/gnss/fix'), 10),
        }
        if markers:
            self.pub['scene'] = cp(MarkerArray, sc['scene_topic'], latched)
            self.pub['npc'] = cp(MarkerArray, sc['npc_topic'], 10)
            self.pub['radar_mk'] = cp(MarkerArray, cfg['ars408']['marker_topic'], 10)
        self.tfb = TransformBroadcaster(node)
        self.stfb = StaticTransformBroadcaster(node) if static_tf else None
        node.create_subscription(Twist, sc['cmd_vel_topic'],
                                 lambda m: self.e.set_cmd('ros', m.linear.x, m.angular.z), 10)
        # 접이식 마스트: 펴기/접기 명령, 관절 상태
        node.create_subscription(Bool, sc.get('deploy_topic', '/beacon/deploy'),
                                 lambda m: self.e.set_mast(bool(m.data), 'ros'), 10)
        self.pub['joint_states'] = cp(JointState, sc.get('joint_states_topic', '/joint_states'), 10)
        self.joint_every = max(1, int(round(0.02 / engine.DT)))          # 50 Hz
        cam_hz = cfg['camera']['rate_hz']
        self.traffic_every = max(1, int(round(1.0 / cam_hz / engine.DT)))
        self.marker_every = max(1, int(round(0.1 / engine.DT)))
        self.publish_static()

    def publish_static_tf(self):
        """base_link(또는 mast_link) → 센서 프레임 (robot.yaml), camera_link → camera_optical_link, gnss_link.
        base_link → mast_link 는 움직이므로 매 스텝 /tf 로 (on_step)."""
        if self.stfb is None:
            return
        stamp = t_to_msg(self.e.t)
        tfs = []
        kin = self.e.kin
        for name, (frame, parent, T) in kin.sensors.items():
            if parent != kin.mast_frame or not kin.has_mast:
                parent = self.base_frame
            tfs.append(pose_to_tf(T, parent, frame, stamp))
            if name == 'camera':
                tfs.append(pose_to_tf(T_CAMLINK_OPTICAL, frame, self.cfg['camera']['optical_frame'], stamp))
        if 'gnss' not in kin.sensors:
            g = self.cfg.get('gnss', {})
            tfs.append(pose_to_tf(make_T(g.get('antenna_xyz', [0.0, 0.0, 0.0])), self.base_frame,
                                  g.get('frame_id', 'gnss_link'), stamp))
        self.stfb.sendTransform(tfs)

    def publish_static(self):
        e = self.e
        self.publish_static_tf()
        d = {'scenario': e.scn.summary(), 'params': e.scn.sp.to_dict(), 'traffic': e.traffic.tp.to_dict(),
             'map': e.model.summary(), 'map_params': e.model.params.to_dict(), 'environment': e.env.to_dict()}
        self.pub['scenario'].publish(String(data=json.dumps(d, ensure_ascii=False, default=str)))
        if self.markers:
            ma = MarkerArray()
            for k, p in enumerate(e.parts):
                ma.markers.append(mesh_marker(p, 'scene', k, self.map_frame))
            for k, (sp, T) in enumerate(e.accident):
                ma.markers.append(box_marker(self.map_frame, 'accident', k, sp, T, (1.0, 0.45, 0.0, 0.95)))
            self.pub['scene'].publish(ma)

    # 엔진 스레드에서 매 스텝 호출
    def on_step(self, e, fired, events):
        SimClock.last = max(SimClock.last, e.t + SimClock.offset)
        stamp = t_to_msg(e.t)
        if self.clock:
            self.pub['clock'].publish(Clock(clock=stamp))
        T_wb = e.T_wb
        tfs = [pose_to_tf(T_wb, self.map_frame, self.base_frame, stamp)]
        kin = e.kin
        if kin.has_mast and self.stfb is not None:     # 런치(robot_state_publisher)에서는 joint_states 로 계산
            tfs.append(pose_to_tf(kin.T_base_mast(e.mast_q), self.base_frame, kin.mast_frame, stamp))
        self.tfb.sendTransform(tfs)
        if kin.has_mast and e.tick % self.joint_every == 0:
            js = JointState()
            js.header.stamp = stamp
            js.name = [kin.joint]
            js.position = [float(e.mast_q)]
            js.velocity = [float(np.sign(e.mast_target - e.mast_q) * kin.rate) if e.mast_q != e.mast_target else 0.0]
            self.pub['joint_states'].publish(js)
        od = Odometry()
        od.header.frame_id, od.header.stamp, od.child_frame_id = self.map_frame, stamp, self.base_frame
        od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = map(float, T_wb[:3, 3])
        q = R_to_quat(T_wb[:3, :3])
        o = od.pose.pose.orientation
        o.x, o.y, o.z, o.w = map(float, q)
        od.twist.twist.linear.x, od.twist.twist.angular.z = e.v, e.w
        self.pub['odom'].publish(od)
        if e.tick % self.traffic_every == 0:
            self._publish_traffic(e, stamp)
        if self.markers and e.tick % self.marker_every == 0:
            self._publish_npc_markers(e, stamp)
        for ev in events:
            self.pub['events'].publish(String(data=json.dumps(
                {'t': round(ev[0], 2), 'type': ev[1], 'vehicle': ev[2], 'info': ev[3]}, ensure_ascii=False)))
        for name, out in fired.items():
            st = t_to_msg(out['t'])
            if name == 'pandar40p':
                self.pub['pandar'].publish(to_pc2(out['points'], out['frame'], st))
                self.pub['pandar_gt'].publish(to_pc2(out['gt'], out['frame'], st))
            elif name == 'tele15':
                self.pub['tele'].publish(to_pc2(out['points'], out['frame'], st))
            elif name == 'ars408':
                self._publish_radar(out, st)
            elif name == 'gnss':
                self._publish_gnss(out, st)

    def _publish_traffic(self, e, stamp):
        with e.lock:
            Ts, _ = e.traffic.poses_at(e.t)
        pa = PoseArray()
        pa.header.frame_id, pa.header.stamp = self.map_frame, stamp
        P, Q = Ts[:, :3, 3].tolist(), quats_from_R(Ts[:, :3, :3]).tolist()
        for (x, y, z), (qx, qy, qz, qw) in zip(P, Q):
            p = Pose()
            p.position.x, p.position.y, p.position.z = x, y, z
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
            pa.poses.append(p)
        self.pub['traffic'].publish(pa)

    def _publish_npc_markers(self, e, stamp):
        tr = e.traffic
        with e.lock:
            Ts, _ = tr.poses_at(e.t)
            active, crashed, noticed, scc = tr.active.copy(), tr.crashed.copy(), tr.noticed.copy(), tr.scc.copy()
        ma = MarkerArray()
        for i, (sp, T) in enumerate(zip(tr.specs, Ts)):
            if not active[i]:
                mk = Marker(ns='npc', id=i, action=Marker.DELETE)
                mk.header.frame_id = self.map_frame
                ma.markers.append(mk)
                continue
            rgba = (1.0, 0.0, 0.0, 0.95) if crashed[i] else (1.0, 0.9, 0.2, 0.9) if noticed[i] else (*sp.color, 0.9)
            ma.markers.append(box_marker(self.map_frame, 'npc', i, sp, T, rgba, stamp))
        self.pub['npc'].publish(ma)

    def _publish_radar(self, out, st):
        hdr = Header(frame_id=out['frame'], stamp=st)
        c = out['clusters']
        self.pub['radar_pc'].publish(to_pc2(c, out['frame'], st))
        scan = RadarScan(header=hdr)
        for p in c:
            scan.returns.append(RadarReturn(range=float(p['distance']), azimuth=float(p['azimuth']), elevation=0.0,
                                            doppler_velocity=float(p['radial_speed']), amplitude=float(p['rcs'])))
        self.pub['radar_scan'].publish(scan)
        tracks = RadarTracks(header=hdr)
        ma = MarkerArray()
        ma.markers.append(Marker(header=hdr, ns='ars408_obj', action=Marker.DELETEALL))
        zg = -out['T_ws'][2, 3] + out['T_wb'][2, 3]
        for tk in out['tracks']:
            sp = tk['spec']
            tr = RadarTrack()
            tr.uuid = UUID(uuid=list(int(tk['id']).to_bytes(16, 'little')))
            tr.position = Point(x=tk['x'], y=tk['y'], z=0.0)
            tr.velocity = Vector3(x=tk['vx'], y=tk['vy'], z=0.0)
            tr.size = Vector3(x=float(sp.length), y=float(sp.width), z=float(sp.height))
            tr.classification = 1 if sp.kind == 'car' else 2
            tracks.tracks.append(tr)
            if self.markers:
                mk = Marker(header=hdr, ns='ars408_obj', id=tk['id'], type=Marker.CUBE, action=Marker.ADD)
                mk.pose.position = Point(x=tk['x'], y=tk['y'], z=float(zg + sp.height / 2))
                mk.pose.orientation.w = 1.0
                mk.scale = Vector3(x=0.6, y=float(sp.width), z=float(sp.height))
                mk.color = ColorRGBA(r=1.0, g=0.3, b=0.0, a=0.6)
                ma.markers.append(mk)
        self.pub['radar_tracks'].publish(tracks)
        if self.markers:
            self.pub['radar_mk'].publish(ma)

    def _publish_gnss(self, g, st):
        m = NavSatFix()
        m.header.stamp, m.header.frame_id = st, self.cfg.get('gnss', {}).get('frame_id', 'gnss_link')
        m.status.service = NavSatStatus.SERVICE_GPS
        m.status.status = {FIX: NavSatStatus.STATUS_GBAS_FIX, FLOAT: NavSatStatus.STATUS_SBAS_FIX,
                           SINGLE: NavSatStatus.STATUS_FIX, NONE: NavSatStatus.STATUS_NO_FIX}[g['status']]
        m.latitude, m.longitude, m.altitude = g['lat'], g['lon'], g['alt']
        h2, v2 = g['hacc'] ** 2, g['vacc'] ** 2
        m.position_covariance = [h2, 0.0, 0.0, 0.0, h2, 0.0, 0.0, 0.0, v2]
        m.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub['gnss'].publish(m)
