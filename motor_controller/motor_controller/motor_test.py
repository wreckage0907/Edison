#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time

class MotorTester(Node):
    def __init__(self):
        super().__init__('motor_tester')
        self.publisher = self.create_publisher(Float32MultiArray, 'motor_speeds', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.test_sequence = 0
    
    def timer_callback(self):
        msg = Float32MultiArray()
        
        # Test sequence
        if self.test_sequence == 0:
            msg.data = [50.0, 50.0]  # Forward
        elif self.test_sequence == 1:
            msg.data = [-50.0, -50.0]  # Backward
        elif self.test_sequence == 2:
            msg.data = [50.0, -50.0]  # Turn right
        elif self.test_sequence == 3:
            msg.data = [-50.0, 50.0]  # Turn left
        else:
            msg.data = [0.0, 0.0]  # Stop
            
        self.publisher.publish(msg)
        self.test_sequence = (self.test_sequence + 1) % 5

def main(args=None):
    rclpy.init(args=args)
    motor_tester = MotorTester()
    
    try:
        rclpy.spin(motor_tester)
    except KeyboardInterrupt:
        pass
    finally:
        motor_tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
