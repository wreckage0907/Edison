#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import math
import pyproj

class GPSSimulator(Node):
    def __init__(self):
        super().__init__('gps_simulator')
        
        # Publishers
        self.gps_publisher = self.create_publisher(NavSatFix, '/gps/fix', 10)
        
        # Subscribers
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',  # This should match your robot's odometry topic
            self.odom_callback,
            10)
            
        # Reference point (origin) in your Gazebo world
        self.ref_lat = 37.7749  # San Francisco latitude as reference
        self.ref_lon = -122.4194  # San Francisco longitude as reference
        self.ref_alt = 0.0
        
        # Set up the projection
        self.proj = pyproj.Proj(proj='utm', zone=10, ellps='WGS84')
        self.ref_x, self.ref_y = self.proj(self.ref_lon, self.ref_lat)
        
        self.get_logger().info('GPS Simulator started with reference point: '
                              f'Lat: {self.ref_lat}, Lon: {self.ref_lon}')

    def odom_callback(self, msg):
        # Get position from odometry
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        # Convert to GPS coordinates
        world_x = self.ref_x + x
        world_y = self.ref_y + y
        
        # Convert back to lat/lon
        lon, lat = self.proj(world_x, world_y, inverse=True)
        
        # Create GPS message
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = 'gps_link'
        
        gps_msg.latitude = lat
        gps_msg.longitude = lon
        gps_msg.altitude = z + self.ref_alt
        
        # Set accuracy and status
        gps_msg.position_covariance[0] = 0.0001  # ~10m accuracy
        gps_msg.position_covariance[4] = 0.0001
        gps_msg.position_covariance[8] = 0.01
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        gps_msg.status.status = 0  # STATUS_FIX
        gps_msg.status.service = 1  # SERVICE_GPS
        
        # Publish the GPS message
        self.gps_publisher.publish(gps_msg)
        self.get_logger().debug(
            f'Published GPS: Lat={lat:.6f}, Lon={lon:.6f}, Alt={z:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    gps_simulator = GPSSimulator()
    rclpy.spin(gps_simulator)
    gps_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()