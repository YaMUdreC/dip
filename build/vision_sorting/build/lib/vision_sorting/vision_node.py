import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from ultralytics import YOLO


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.declare_parameter('image_topic', '/rgbd_camera/image')
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('model_path', 'yolov8l.pt')
        self.declare_parameter('conf_threshold', 0.1)
        self.declare_parameter('show_window', True)

        image_topic = self.get_parameter('image_topic').value
        detections_topic = self.get_parameter('detections_topic').value
        model_path = self.get_parameter('model_path').value
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.show_window = bool(self.get_parameter('show_window').value)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.bridge = CvBridge()
        self.model = YOLO(model_path)
        self.processing = False

        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            sensor_qos
        )

        self.publisher = self.create_publisher(
            Detection2DArray,
            detections_topic,
            10
        )

        self.get_logger().info(
            f'Vision node started. image_topic={image_topic}, detections_topic={detections_topic}'
        )

    def image_callback(self, msg: Image):
        if self.processing:
            return
        self.processing = True

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(frame, verbose=False)

            annotated_frame = frame.copy()
            detections_msg = Detection2DArray()
            detections_msg.header = msg.header

            for result in results:
                if result.boxes is None:
                    continue

                for box in result.boxes:
                    confidence = float(box.conf[0])
                    if confidence < self.conf_threshold:
                        continue

                    class_id = int(box.cls[0])
                    class_name = str(self.model.names[class_id])

                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                    center_x = float((x1 + x2) / 2.0)
                    center_y = float((y1 + y2) / 2.0)
                    width = float(x2 - x1)
                    height = float(y2 - y1)

                    detection = Detection2D()
                    detection.header = msg.header

                    detection.bbox.center.position.x = center_x
                    detection.bbox.center.position.y = center_y
                    detection.bbox.center.theta = 0.0
                    detection.bbox.size_x = width
                    detection.bbox.size_y = height

                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = class_name
                    hypothesis.hypothesis.score = float(confidence)
                    detection.results.append(hypothesis)

                    detections_msg.detections.append(detection)

                    label = f'{class_name} {confidence:.2f}'
                    cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(annotated_frame, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)
                    cv2.putText(
                        annotated_frame,
                        label,
                        (x1, max(y1 - 10, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2
                    )

            self.publisher.publish(detections_msg)

            if self.show_window:
                cv2.imshow('YOLO Vision Node', annotated_frame)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Ошибка в vision_node: {e}')
        finally:
            self.processing = False


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()