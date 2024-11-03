#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import RPi.GPIO as GPIO
import time

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # GPIO Setup
        GPIO.setmode(GPIO.BCM)
        
        # Motor A pins
        self.IN1 = 17
        self.IN2 = 27
        self.ENA = 12
        
        # Motor B pins
        self.IN3 = 22
        self.IN4 = 23
        self.ENB = 13
        
        # Setup all pins
        self.pins = [self.IN1, self.IN2, self.IN3, self.IN4, self.ENA, self.ENB]
        for pin in self.pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        
        # Setup PWM
        self.pwm_a = GPIO.PWM(self.ENA, 1000)  # 1000 Hz frequency
        self.pwm_b = GPIO.PWM(self.ENB, 1000)
        self.pwm_a.start(0)
        self.pwm_b.start(0)
        
        # Create subscription
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'motor_speeds',
            self.motor_callback,
            10)
        
        self.get_logger().info('Motor controller node initialized')
    
    def motor_callback(self, msg):
        # Expect msg.data to be [left_speed, right_speed]
        # Speed range: -100 to 100
        if len(msg.data) != 2:
            self.get_logger().error('Invalid message length')
            return
            
        left_speed, right_speed = msg.data
        
        # Control Motor A (left)
        if left_speed >= 0:
            GPIO.output(self.IN1, GPIO.HIGH)
            GPIO.output(self.IN2, GPIO.LOW)
            self.pwm_a.ChangeDutyCycle(abs(left_speed))
        else:
            GPIO.output(self.IN1, GPIO.LOW)
            GPIO.output(self.IN2, GPIO.HIGH)
            self.pwm_a.ChangeDutyCycle(abs(left_speed))
        
        # Control Motor B (right)
        if right_speed >= 0:
            GPIO.output(self.IN3, GPIO.HIGH)
            GPIO.output(self.IN4, GPIO.LOW)
            self.pwm_b.ChangeDutyCycle(abs(right_speed))
        else:
            GPIO.output(self.IN3, GPIO.LOW)
            GPIO.output(self.IN4, GPIO.HIGH)
            self.pwm_b.ChangeDutyCycle(abs(right_speed))
    
    def cleanup(self):
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass
    finally:
        motor_controller.cleanup()
        motor_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
