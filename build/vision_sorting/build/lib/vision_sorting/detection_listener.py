import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray


class DetectionListener(Node):
    def __init__(self):
        super().__init__('detection_listener')

        self.subscription = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detections_callback,
            10
        )

        self.get_logger().info('Detection listener started. Waiting for Detection2DArray...')

    def detections_callback(self, msg: Detection2DArray):
        detections = msg.detections

        if not detections:
            self.get_logger().info('Детекции не найдены')
            return

        self.get_logger().info(f'Получено детекций: {len(detections)}')

        for i, detection in enumerate(detections, start=1):
            center_x = detection.bbox.center.position.x
            center_y = detection.bbox.center.position.y
            size_x = detection.bbox.size_x
            size_y = detection.bbox.size_y

            if detection.results:
                hypothesis = detection.results[0]
                class_name = hypothesis.hypothesis.class_id
                score = hypothesis.hypothesis.score
            else:
                class_name = 'unknown'
                score = 0.0

            self.get_logger().info(
                f'[{i}] class={class_name}, '
                f'score={score:.3f}, '
                f'center=({center_x:.1f}, {center_y:.1f}), '
                f'size=({size_x:.1f}, {size_y:.1f})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = DetectionListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()