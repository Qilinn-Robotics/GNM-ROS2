#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(
            Image,
            '/camera/image',
            10
        )
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1Hz

        self.image = cv2.imread('/home/qilinn-dev/codes/GNM-ROS2/src/visualnav_transformer/deployment/topomaps/images/map1229_1/0.png')
        if self.image is None:
            self.get_logger().error('图片读取失败')

    def timer_callback(self):
        msg = self.bridge.cv2_to_imgmsg(self.image, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
