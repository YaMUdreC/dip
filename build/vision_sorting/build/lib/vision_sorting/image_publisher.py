import os

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        # Укажи путь к своему изображению
        self.image_path = os.path.expanduser('~/ros2_ws/test_data/test.jpeg')

        self.image = cv2.imread(self.image_path)

        if self.image is None:
            self.get_logger().error(f'Не удалось загрузить изображение: {self.image_path}')
            raise FileNotFoundError(f'Image not found: {self.image_path}')

        self.timer = self.create_timer(1.0, self.publish_image)
        self.get_logger().info(f'Image publisher started. Publishing: {self.image_path}')

    def publish_image(self):
        try:
            msg = self.bridge.cv2_to_imgmsg(self.image, encoding='bgr8')
            self.publisher_.publish(msg)
            self.get_logger().info('Изображение опубликовано в /camera/image_raw')
        except Exception as e:
            self.get_logger().error(f'Ошибка при публикации изображения: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()