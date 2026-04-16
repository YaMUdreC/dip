import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray


class TargetSelector(Node):
    def __init__(self):
        super().__init__('target_selector')

        self.target_class = 'bottle'

        self.subscription = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detections_callback,
            10
        )

        self.target_publisher = self.create_publisher(
            Point,
            '/target_center',
            10
        )

        self.get_logger().info(
            f'Target selector started. Looking for class: {self.target_class}'
        )

    def detections_callback(self, msg: Detection2DArray):
        best_detection = None
        best_score = -1.0

        for detection in msg.detections:
            if not detection.results:
                continue

            hypothesis = detection.results[0]
            class_name = hypothesis.hypothesis.class_id
            score = float(hypothesis.hypothesis.score)

            if class_name != self.target_class:
                continue

            if score > best_score:
                best_score = score
                best_detection = detection

        if best_detection is None:
            self.get_logger().info(f'Объект класса "{self.target_class}" не найден')
            return

        center_x = float(best_detection.bbox.center.position.x)
        center_y = float(best_detection.bbox.center.position.y)

        target_msg = Point()
        target_msg.x = center_x
        target_msg.y = center_y
        target_msg.z = 0.0

        self.target_publisher.publish(target_msg)

        self.get_logger().info(
            f'Целевой объект найден: class={self.target_class}, '
            f'score={best_score:.3f}, center=({center_x:.1f}, {center_y:.1f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TargetSelector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()