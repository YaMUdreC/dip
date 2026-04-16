import math

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2DArray


class Target3DEstimator(Node):
    def __init__(self):
        super().__init__('target_3d_estimator')

        self.declare_parameter('target_class', 'bottle')
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('depth_topic', '/rgbd_camera/depth_image')
        self.declare_parameter('camera_info_topic', '/rgbd_camera/camera_info')
        self.declare_parameter('target_3d_topic', '/target_3d')
        self.declare_parameter('depth_window_size', 5)

        self.target_class = self.get_parameter('target_class').value
        detections_topic = self.get_parameter('detections_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        target_3d_topic = self.get_parameter('target_3d_topic').value
        self.depth_window_size = int(self.get_parameter('depth_window_size').value)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.bridge = CvBridge()
        self.latest_depth_image = None
        self.latest_camera_info = None

        self.create_subscription(
            Detection2DArray,
            detections_topic,
            self.detections_callback,
            10
        )

        self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            sensor_qos
        )

        self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            sensor_qos
        )

        self.target_3d_pub = self.create_publisher(Point, target_3d_topic, 10)

        self.get_logger().info(
            f'Target3DEstimator started. target_class={self.target_class}, '
            f'depth_topic={depth_topic}, camera_info_topic={camera_info_topic}'
        )

    def depth_callback(self, msg: Image):
        try:
            if msg.encoding == '32FC1':
                self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            elif msg.encoding == '16UC1':
                self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            else:
                self.get_logger().warning(f'Неподдерживаемый формат depth image: {msg.encoding}')
        except Exception as e:
            self.get_logger().error(f'Ошибка при чтении depth image: {e}')

    def camera_info_callback(self, msg: CameraInfo):
        self.latest_camera_info = msg

    def detections_callback(self, msg: Detection2DArray):
        if self.latest_depth_image is None or self.latest_camera_info is None:
            return

        best_detection = None
        best_score = -1.0

        for detection in msg.detections:
            if not detection.results:
                continue

            result = detection.results[0]
            class_name = result.hypothesis.class_id
            score = float(result.hypothesis.score)

            if class_name == self.target_class and score > best_score:
                best_score = score
                best_detection = detection

        if best_detection is None:
            return

        u = int(round(best_detection.bbox.center.position.x))
        v = int(round(best_detection.bbox.center.position.y))

        depth_value = self.get_median_depth(u, v)
        if depth_value is None:
            self.get_logger().warning(f'Не удалось получить глубину около пикселя ({u}, {v})')
            return

        fx = float(self.latest_camera_info.k[0])
        fy = float(self.latest_camera_info.k[4])
        cx = float(self.latest_camera_info.k[2])
        cy = float(self.latest_camera_info.k[5])

        x = (u - cx) * depth_value / fx
        y = (v - cy) * depth_value / fy
        z = depth_value

        point_msg = Point()
        point_msg.x = float(x)
        point_msg.y = float(y)
        point_msg.z = float(z)

        self.target_3d_pub.publish(point_msg)

        self.get_logger().info(
            f'Published target_3d: X={x:.3f}, Y={y:.3f}, Z={z:.3f} m'
        )

    def get_median_depth(self, u: int, v: int):
        img = self.latest_depth_image
        if img is None:
            return None

        h, w = img.shape[:2]
        r = max(1, self.depth_window_size // 2)

        x_min = max(0, u - r)
        x_max = min(w, u + r + 1)
        y_min = max(0, v - r)
        y_max = min(h, v + r + 1)

        patch = img[y_min:y_max, x_min:x_max]
        values = patch.flatten()

        if img.dtype == np.uint16:
            values = values[values > 0].astype(np.float32) / 1000.0
        else:
            values = values[np.isfinite(values)]
            values = values[values > 0.0].astype(np.float32)

        if values.size == 0:
            return None

        return float(np.median(values))


def main(args=None):
    rclpy.init(args=args)
    node = Target3DEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()