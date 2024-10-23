#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped

class GPSHandler(Node):
    def __init__(self):
        super().__init__('gps_handler')
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10)
        self.pose_pub = self.create_publisher(PoseStamped, '/gps/pose', 10)
        
    def gps_callback(self, msg):
        # Convert GPS coordinates to local coordinates (you'll need to implement the conversion)
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        # Add conversion logic here
        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GPSHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
