#!/usr/bin/env python3
import rclcpp
from rclcpp.node import Node
from sensor_msgs.msg import PointCloud2, Image
from tf2_ros import TransformListener, Buffer
from cv_bridge import CvBridge
import cv2

class SensorFusion(Node):
    def __init__(self):
        super().__init__('sensor_fusion')
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.radar_sub = self.create_subscription(PointCloud2, '/ti_mmwave/radar_scan_pcl', self.radar_callback, 10)
        self.image_pub = self.create_publisher(Image, '/fused_image', 10)
        self.points = []

    def radar_callback(self, msg):
        self.points = []  # Placeholder: Parse PointCloud2

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        for point in self.points:
            x, y = 320, 240  # Placeholder
            cv2.circle(cv_image, (int(x), int(y)), 5, (0, 0, 255), -1)
        fused_image = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
        self.image_pub.publish(fused_image)

def main(args=None):
    rclcpp.init(args=args)
    node = SensorFusion()
    rclcpp.spin(node)
    node.destroy_node()
    rclcpp.shutdown()

if __name__ == '__main__':
    main()