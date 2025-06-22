import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from ament_index_python.packages import get_package_share_directory

class ImagePublisher(Node):
    def __init__(self):
    
        super().__init__('image_publisher_node')
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()
        
        pkg_share_dir = get_package_share_directory('yolo_ros2')
        image_path = os.path.join(pkg_share_dir, 'bus.jpg')

        self.get_logger().info(f'Reading image from: {image_path}')
        self.cv_image = cv2.imread(image_path)
        if self.cv_image is None:
            self.get_logger().error(f'Failed to load image from {image_path}.')
            self.destroy_node()

    def timer_callback(self):
        if hasattr(self, 'cv_image') and self.cv_image is not None:
            ros_image = self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8")
            self.publisher_.publish(ros_image)
            self.get_logger().info('Publishing an image')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    if image_publisher.cv_image is not None:
        try:
            rclpy.spin(image_publisher)
        except KeyboardInterrupt:
            pass
        finally:
            image_publisher.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
