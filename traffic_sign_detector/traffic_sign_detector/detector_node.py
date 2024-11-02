import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class SimpleCameraNode(Node):
    def __init__(self):
        super().__init__('simple_camera_node')
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Create subscription to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/usb_cam/image_raw',  # This is the default topic for usb_cam
            self.image_callback,
            10)
        
        # Create output directory for saved images
        self.output_dir = os.path.expanduser('~/camera_feed_images')
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Counter for received frames
        self.frame_count = 0
        
        # Save an image every N frames
        self.save_interval = 30  # Adjust this value to save more or less frequently
        
        self.get_logger().info('Simple Camera Node has been started')
        self.get_logger().info(f'Saving images to: {self.output_dir}')
        self.get_logger().info('Waiting for camera feed...')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Increment frame counter
            self.frame_count += 1
            
            # Log every 100 frames
            if self.frame_count % 100 == 0:
                self.get_logger().info(f'Received {self.frame_count} frames')
            
            # Save image every N frames
            if self.frame_count % self.save_interval == 0:
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                filename = os.path.join(self.output_dir, f'frame_{timestamp}.jpg')
                cv2.imwrite(filename, cv_image)
                self.get_logger().info(f'Saved image: {filename}')
            
            # Display basic image information
            height, width = cv_image.shape[:2]
            if self.frame_count == 1:
                self.get_logger().info(f'Image size: {width}x{height}')
                self.get_logger().info('Successfully receiving camera feed!')
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    try:
        node = SimpleCameraNode()
        self.get_logger().info('Press Ctrl+C to stop the node')
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
