"""ROS 토픽 실제 수신 주기 측정 (메시지를 역직렬화하지 않아 큰 영상·점군도 측정 도구가 병목이 되지 않음).

사용: python tools/topic_rates.py [초=10] [토픽 ...]   (토픽을 안 주면 BeaconSim 센서 토픽 전부)
`ros2 topic hz` 는 파이썬에서 메시지를 풀어 보느라 8 MB 영상은 실제보다 훨씬 낮게 나온다.
"""
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosidl_runtime_py.utilities import get_message

DEFAULT = ['/sensing/camera/arducam/image_raw', '/sensing/camera/arducam/camera_info',
           '/sensing/lidar/pandar/pointcloud_raw_ex', '/sensing/lidar/livox/tele15/pointcloud',
           '/sensing/radar/ars408/clusters', '/sensing/gnss/fix', '/clock', '/sim/traffic']


def main():
    args = sys.argv[1:]
    dur = float(args.pop(0)) if args and args[0].replace('.', '').isdigit() else 10.0
    rclpy.init()
    node = rclpy.create_node('beaconsim_topic_rates')
    time.sleep(1.5)                                     # 토픽 탐색
    types = dict((n, t[0]) for n, t in node.get_topic_names_and_types())
    want = args or DEFAULT
    want = [w for w in want if w in types] + [n for n in types if not args and n.startswith('/sensing/') and n not in want]
    qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
    stamps = {w: [] for w in want}
    sizes = {w: 0 for w in want}
    for w in want:
        def cb(msg, w=w):
            stamps[w].append(time.perf_counter())
            sizes[w] = len(msg)
        node.create_subscription(get_message(types[w]), w, cb, qos, raw=True)
    t_end = time.perf_counter() + dur
    while time.perf_counter() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
    print(f'{"토픽":52s} {"Hz":>7s} {"최대 간격":>9s} {"크기":>9s}')
    for w in want:
        s = stamps[w]
        if len(s) < 2:
            print(f'{w:52s} {"-":>7s}')
            continue
        gaps = [b - a for a, b in zip(s, s[1:])]
        print(f'{w:52s} {(len(s) - 1) / (s[-1] - s[0]):7.2f} {max(gaps) * 1000:7.0f}ms {sizes[w] / 1e6:7.2f}MB')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
