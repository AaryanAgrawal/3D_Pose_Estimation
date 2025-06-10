#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

class DopeTopicBridge(Node):
    def __init__(self):
        super().__init__('dope_topic_bridge')
        
        self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.info_callback, 10)
        
        self.image_pub = self.create_publisher(Image, '/image_rect', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera_info_rect', 10)
        
        self.get_logger().info('DOPE Topic Bridge Started')
    
    def image_callback(self, msg):
        self.image_pub.publish(msg)
    
    def info_callback(self, msg):
        self.info_pub.publish(msg)

def main():
    rclpy.init()
    bridge = DopeTopicBridge()
    try:
        rclpy.spin(bridge)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            bridge.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
