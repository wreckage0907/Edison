import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

class TrafficLightViewer(Node):
    def __init__(self):
        super().__init__('traffic_light_viewer')
        
        # Create subscriptions
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        
        self.status_sub = self.create_subscription(
            String,
            'traffic_light/status',
            self.status_callback,
            10)
        
        self.cv_bridge = CvBridge()
        self.current_status = "No traffic light detected"
        self.get_logger().info('Traffic light viewer started')

    def status_callback(self, msg):
        self.current_status = msg.data
        self.get_logger().info(f'Traffic light status: {self.current_status}')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Add status text to the image
            cv2.putText(cv_image, f'Status: {self.current_status}',
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                       (255, 255, 255), 2)
            
            # Display the image
            cv2.imshow('Traffic Light Detection', cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    viewer = TrafficLightViewer()
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        viewer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

