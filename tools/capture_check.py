"""실행 중인 심에서 한 프레임씩 받아 TF로 map 좌표에 올린 뒤 그림으로 저장한다 (RViz 없이 점검용).

사용: python3 tools/capture_check.py [출력 디렉터리]
"""
import os
import sys
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from rclpy.time import Time  # noqa: E402
from sensor_msgs.msg import Image, PointCloud2  # noqa: E402
from sensor_msgs_py import point_cloud2  # noqa: E402
from tf2_ros import Buffer, TransformListener  # noqa: E402

TOPICS = {
    'pandar': '/sensing/lidar/pandar/pointcloud_raw_ex',
    'gt': '/ground_truth/lidar/pandar/labels',
    'tele': '/sensing/lidar/livox/points',
    'radar': '/sensing/radar/ars408/clusters',
}


def tf_to_T(tf):
    t, q = tf.transform.translation, tf.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, [t.x, t.y, t.z]
    return T


class Cap(Node):
    def __init__(self):
        super().__init__('capture_check', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)
        self.msgs = {}
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        for k, t in TOPICS.items():
            self.create_subscription(PointCloud2, t, lambda m, k=k: self.msgs.setdefault(k, m), qos)
        self.create_subscription(Image, '/sensing/camera/arducam/image_raw', lambda m: self.msgs.setdefault('img', m), qos)


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else '.'
    os.makedirs(out, exist_ok=True)
    rclpy.init()
    n = Cap()
    t0 = time.time()
    while time.time() - t0 < 15 and len(n.msgs) < 5:
        rclpy.spin_once(n, timeout_sec=0.1)
    for _ in range(20):
        rclpy.spin_once(n, timeout_sec=0.05)
    print('받은 토픽:', sorted(n.msgs))
    clouds = {}
    for k in TOPICS:
        if k not in n.msgs:
            continue
        m = n.msgs[k]
        names = [f.name for f in m.fields]
        arr = point_cloud2.read_points(m, field_names=names)
        tf = n.buf.lookup_transform('map', m.header.frame_id, Time.from_msg(m.header.stamp),
                                    timeout=rclpy.duration.Duration(seconds=2.0))
        T = tf_to_T(tf)
        xyz = np.c_[arr['x'], arr['y'], arr['z']].astype(float)
        clouds[k] = (xyz @ T[:3, :3].T + T[:3, 3], arr)
        print(f'{k:6s} frame={m.header.frame_id:13s} 점 {len(arr):6d}  필드 {names}  point_step {m.point_step}')
    base = tf_to_T(n.buf.lookup_transform('map', 'base_link', Time()))
    bx, by = base[0, 3], base[1, 3]

    if 'img' in n.msgs:
        im = n.msgs['img']
        img = np.frombuffer(bytes(im.data), np.uint8).reshape(im.height, im.width, 3)
        plt.imsave(os.path.join(out, 'camera.png'), img)

    fig, ax = plt.subplots(2, 1, figsize=(16, 12))
    if 'pandar' in clouds:
        p, a = clouds['pandar']
        sc = ax[0].scatter(p[:, 0], p[:, 1], s=0.3, c=np.clip(a['intensity'], 0, 60), cmap='viridis')
        plt.colorbar(sc, ax=ax[0], label='Pandar intensity')
    if 'tele' in clouds:
        p, _ = clouds['tele']
        ax[0].scatter(p[:, 0], p[:, 1], s=0.3, c='orange', label='Tele-15')
    if 'radar' in clouds:
        p, a = clouds['radar']
        ax[0].scatter(p[:, 0], p[:, 1], s=25, facecolors='none', edgecolors='r', label='ARS408')
    ax[0].plot(bx, by, 'k*', ms=15)
    ax[0].set_xlim(bx - 40, bx + 120)
    ax[0].set_ylim(by - 20, by + 25)
    ax[0].set_aspect('equal')
    ax[0].legend(loc='upper right')
    ax[0].set_title('top view (map)')
    if 'pandar' in clouds:
        p, a = clouds['pandar']
        m = np.abs(p[:, 1] - by) < 1.0
        ax[1].scatter(p[m, 0], p[m, 2], s=1, c='b', label='Pandar |dy|<1m')
    if 'tele' in clouds:
        p, _ = clouds['tele']
        m = np.abs(p[:, 1] - by) < 1.0
        ax[1].scatter(p[m, 0], p[m, 2], s=1, c='orange', label='Tele-15 |dy|<1m')
    ax[1].axhline(0, color='k', lw=0.5)
    ax[1].set_xlim(bx - 10, bx + 120)
    ax[1].set_ylim(-1, 10)
    ax[1].legend()
    ax[1].set_title('side view (map x-z)')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'clouds.png'), dpi=110)
    if 'pandar' in clouds:
        p, _ = clouds['pandar']
        g = p[np.abs(p[:, 0] - bx) < 15]
        road = g[np.abs(g[:, 2] - base[2, 3]) < 0.5]
        print(f'Pandar 지면점 z - base_link z (로봇 ±15m): 평균 {(road[:, 2] - base[2, 3]).mean():+.4f} m, '
              f'표준편차 {road[:, 2].std():.4f}')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
