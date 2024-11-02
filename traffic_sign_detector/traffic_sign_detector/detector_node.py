import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from tensorflow.keras.models import load_model

class TrafficSignDetectorNode(Node):
    def __init__(self):
        super().__init__('traffic_sign_detector')
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Create subscription to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/usb_cam/image_raw',
            self.image_callback,
            10)
        
        # Publisher for processed images
        self.publisher = self.create_publisher(
            Image,
            '/traffic_sign_detector/processed_image',
            10)
        
        # Initialize cascade classifier for traffic sign detection
        self.sign_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'  # Replace with traffic sign cascade
        )
        
        self.get_logger().info('Traffic Sign Detector Node has been started')

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Convert to grayscale for detection
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Detect traffic signs
        signs = self.sign_cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30)
        )
        
        # Draw rectangles around detected signs
        for (x, y, w, h) in signs:
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
            
            # Extract sign region for classification
            sign_roi = cv_image[y:y+h, x:x+w]
            # Add classification logic here
        
        # Publish processed image
        processed_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.publisher.publish(processed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
