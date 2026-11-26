"""Arducam B0495 (AR0234) 카메라 모델 노드.

RGL은 레이캐스트 전용이라 카메라는 같은 씬 메시를 EGL 헤드리스 OpenGL(moderngl)로 래스터 렌더한다.
1) 핀홀 + 오버스캔으로 렌더 (4x MSAA)
2) 캘리브레이션 K, D(plumb_bob)로 왜곡 remap → 실제 해상도 이미지
3) sensor_msgs/Image + CameraInfo 발행 (광학 프레임)

로봇 포즈는 /ground_truth/odom, 차량은 같은 시각의 /sim/traffic(PoseArray, 슬롯 순서 고정)으로 그린다.
"""
import argparse
import array

import cv2
import moderngl
import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from .common import T_CAMLINK_OPTICAL, load_robot_parts, load_yaml, sensor_mounts
from .environment import Env
from .render import CameraRenderer, SceneRenderer
from .roadgen import RoadParams, generate
from .traffic import Scenario, ScenarioParams, TrafficParams, slot_specs


def odom_to_T(od):
    p, q = od.pose.pose.position, od.pose.pose.orientation
    x, y, z, w = q.x, q.y, q.z, q.w
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    T[:3, 3] = [p.x, p.y, p.z]
    return T


class CameraNode(Node):
    def __init__(self, cfg, robot_cfg):
        super().__init__('beacon_camera')
        self.cfg = cc = cfg['camera']
        self.ctx = moderngl.create_context(standalone=True, backend='egl')
        self.cam = CameraRenderer(self.ctx, cc)
        self.W, self.H, self.K, self.D = self.cam.W, self.cam.H, self.cam.K, self.cam.D
        self.get_logger().info(f'카메라 {self.W}x{self.H}, 오버스캔 렌더 {self.cam.Wr}x{self.cam.Hr}, '
                               f'HFOV {self.cam.hfov:.1f}°, VFOV {self.cam.vfov:.1f}°')
        # 시드가 랜덤(-1)일 수 있으므로 sim_core가 실제로 쓴 값을 /sim/scenario(latched)에서 받는다
        info = self._wait_scenario(cfg['sim']['scenario_topic'])
        model = generate(RoadParams(**info['map_params']))
        self.env = Env(**info['environment']) if 'environment' in info else None
        self.renderer = SceneRenderer(self.ctx, model, load_robot_parts(), exclude_robot=('camera',), env=self.env)
        self.specs = slot_specs(TrafficParams(**info['traffic']), info['params']['seed'])
        self.accident = Scenario(model, ScenarioParams(**info['params'])).accident_poses()
        self.get_logger().info(f'맵 시드 {model.params.seed}, 시나리오 시드 {info["params"]["seed"]}, '
                               f'환경 {self.env.label() if self.env else "-"}')

        self.odoms = {}                       # stamp(ns) → Odometry (최근 것만)
        _, T_bc = sensor_mounts(robot_cfg)['camera']
        self.T_b_opt = T_bc @ T_CAMLINK_OPTICAL
        self.frame = cc['optical_frame']
        self.odom = None
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub_img = self.create_publisher(Image, cc['image_topic'], qos)
        self.pub_info = self.create_publisher(CameraInfo, cc['info_topic'], qos)
        self.create_subscription(Odometry, cfg['sim']['odom_topic'], self._on_odom, 10)
        self.create_subscription(PoseArray, cfg['sim']['traffic_topic'], self._on_traffic, 20)
        self.last_stamp = None
        # 렌더는 /sim/traffic 수신 시 (sim_core가 카메라 주기로 발행)

    def _wait_scenario(self, topic):
        import json
        from rclpy.qos import DurabilityPolicy
        from std_msgs.msg import String
        box = {}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sub = self.create_subscription(String, topic, lambda m: box.setdefault('d', json.loads(m.data)), qos)
        while 'd' not in box and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
        self.destroy_subscription(sub)
        return box['d']

    def _on_odom(self, msg):
        ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.odoms[ns] = msg
        if len(self.odoms) > 60:
            for k in sorted(self.odoms)[:-60]:
                del self.odoms[k]

    def _on_traffic(self, msg):
        ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        T = np.tile(np.eye(4), (len(msg.poses), 1, 1))
        for i, p in enumerate(msg.poses):
            x, y, z, w = p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
            T[i, :3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
            T[i, :3, 3] = [p.position.x, p.position.y, p.position.z]
        od = self.odoms.get(ns)
        if od is None and self.odoms:
            od = self.odoms[min(self.odoms, key=lambda k: abs(k - ns))]
        if od is not None:
            self._render(od, T)

    def _render(self, od, Ts):
        vehicles = list(self.accident) + list(zip(self.specs, Ts))
        T_wb = odom_to_T(od)
        img = self.cam.render(self.renderer, T_wb @ self.T_b_opt, T_wb, vehicles, self.env)

        msg = Image()
        msg.header.stamp, msg.header.frame_id = od.header.stamp, self.frame
        msg.height, msg.width, msg.encoding, msg.is_bigendian = self.H, self.W, self.cfg['encoding'], 0
        msg.step = self.W * 3
        msg.data = array.array('B', img.tobytes())
        self.pub_img.publish(msg)
        info = CameraInfo()
        info.header = msg.header
        info.height, info.width = self.H, self.W
        info.distortion_model = 'plumb_bob'
        info.d = [float(x) for x in self.D]
        info.k = [float(x) for x in self.K.ravel()]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [float(self.K[0, 0]), 0.0, float(self.K[0, 2]), 0.0, 0.0, float(self.K[1, 1]), float(self.K[1, 2]),
                  0.0, 0.0, 0.0, 1.0, 0.0]
        self.pub_info.publish(info)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', default='config/sim.yaml')
    ap.add_argument('--robot', default='config/robot.yaml')
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = CameraNode(load_yaml(args.config), load_yaml(args.robot))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
