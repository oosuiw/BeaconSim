"""ROS2 연동 (있으면): RosBridge(센서·TF·/cmd_vel) + 카메라 영상 발행. GUI 종류와 무관하게 쓴다."""
import array
import threading


class RosLink:
    """ROS2가 있으면 RosBridge + 카메라 발행. 없으면 비활성."""

    def __init__(self, log):
        self.ok, self.bridge, self.node = False, None, None
        self.log = log
        try:
            import rclpy
            from rclpy.node import Node
            self.rclpy = rclpy
            try:            # Ctrl+C 는 앱이 직접 처리 (rclpy 가 먼저 컨텍스트를 끄면 엔진 스레드 발행이 실패)
                from rclpy.signals import SignalHandlerOptions
                rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            except ImportError:
                rclpy.init()
            self.node = Node('beacon_sim_app')
            self.ok = True
        except Exception as e:        # noqa: BLE001
            log(f'ROS2 연동 없음 (단독 실행): {e}')

    def attach(self, engine, cfg):
        if not self.ok:
            return None
        from .ros_bridge import RosBridge
        if self.bridge is None:
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import CameraInfo, Image
            self.bridge = RosBridge(self.node, engine, cfg, markers=False)
            qs = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.pub_img = self.node.create_publisher(Image, cfg['camera']['image_topic'], qs)
            self.pub_info = self.node.create_publisher(CameraInfo, cfg['camera']['info_topic'], qs)
            self.Image, self.CameraInfo = Image, CameraInfo
            threading.Thread(target=self._spin, daemon=True).start()
            self.log('ROS2 연동: 토픽 발행 중, /cmd_vel 수신 대기')
        else:
            self.bridge.e = engine
            self.bridge.publish_static()
        return self.bridge.on_step

    def _spin(self):
        try:
            self.rclpy.spin(self.node)
        except Exception:             # noqa: BLE001
            pass

    def publish_camera(self, img, t, cam, frame):
        if not self.ok or self.bridge is None:
            return
        from .ros_bridge import t_to_msg
        st = t_to_msg(t)
        msg = self.Image()
        msg.header.stamp, msg.header.frame_id = st, frame
        msg.height, msg.width, msg.encoding, msg.is_bigendian = img.shape[0], img.shape[1], 'rgb8', 0
        msg.step = img.shape[1] * 3
        msg.data = array.array('B', img.tobytes())
        self.pub_img.publish(msg)
        info = self.CameraInfo()
        info.header = msg.header
        info.height, info.width = img.shape[0], img.shape[1]
        info.distortion_model = 'plumb_bob'
        info.d = [float(x) for x in cam.D]
        info.k = [float(x) for x in cam.K.ravel()]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [float(cam.K[0, 0]), 0.0, float(cam.K[0, 2]), 0.0, 0.0, float(cam.K[1, 1]), float(cam.K[1, 2]),
                  0.0, 0.0, 0.0, 1.0, 0.0]
        self.pub_info.publish(info)

    def shutdown(self):
        if self.ok:
            try:
                self.node.destroy_node()
                self.rclpy.shutdown()
            except Exception:         # noqa: BLE001
                pass
