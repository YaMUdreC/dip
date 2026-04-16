import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info('Image subscriber started. Waiting for images...')

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            height, width, channels = cv_image.shape
            self.get_logger().info(
                f'Получено изображение: width={width}, height={height}, channels={channels}'
            )

            cv2.imshow('ROS Image Subscriber', cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Ошибка при обработке изображения: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()

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