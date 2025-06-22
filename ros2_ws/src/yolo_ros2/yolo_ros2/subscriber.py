# 文件路径: ~/ros2_ws/src/yolo_ros2/yolo_ros2/subscriber.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from collections import Counter

class YoloSubscriber(Node):
    def __init__(self):
        super().__init__('yolo_subscriber_node')
        self.subscription = self.create_subscription(
            Image, 'image_raw', self.listener_callback, 10)
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info('YOLO Subscriber node started and model loaded.')

    def listener_callback(self, msg):
        self.get_logger().info('Receiving image for detection...')
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # 使用YOLOv8进行目标检测
        results = self.model(cv_image, verbose=False) # verbose=False让它不在终端打印YOLO自己的日志
        
        # 从结果中获取检测到的物体类别
        names = self.model.names
        detected_objects = [names[int(c)] for c in results[0].boxes.cls]
        
        # 统计并打印结果
        if detected_objects:
            detection_summary = ", ".join([f"{count} {name}" for name, count in Counter(detected_objects).items()])
            self.get_logger().info(f'Detected: [ {detection_summary} ]')
        else:
            self.get_logger().info('Detected: [ No objects found ]')


def main(args=None):
    rclpy.init(args=args)
    yolo_subscriber = YoloSubscriber()
    try:
        rclpy.spin(yolo_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        yolo_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
