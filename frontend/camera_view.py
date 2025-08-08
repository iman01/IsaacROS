import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np

from cv_bridge import CvBridge

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.bridge = CvBridge()
        # Subscribe to all three camera topics
        self.create_subscription(Image, '/camera_front_rgb', self.callback_front, 10)
        self.create_subscription(Image, '/camera_left_rgb', self.callback_left, 10)
        self.create_subscription(Image, '/camera_right_rgb', self.callback_right, 10)

    def callback_front(self, msg):
        self.display_image(msg, 'Front Camera')

    def callback_left(self, msg):
        self.display_image(msg, 'Left Camera')

    def callback_right(self, msg):
        self.display_image(msg, 'Right Camera')

    def display_image(self, msg, window_name):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow(window_name, cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()