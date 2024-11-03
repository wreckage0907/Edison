import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
from std_msgs.msg import String

class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')
        
        # Initialize publishers
        self.image_publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.status_publisher = self.create_publisher(String, 'traffic_light/status', 10)
        
        # Create timer for camera capture
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        self.cv_bridge = CvBridge()
        
        # Initialize camera
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera!')
            return
            
        # Set camera resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Load YOLO model
        self.get_logger().info('Loading YOLO model...')
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.TRAFFIC_LIGHT_CLASS_ID = 9  # COCO dataset class ID for traffic light
        
        # Define color ranges for HSV
        self.color_ranges = {
            'red': (np.array([0, 100, 100]), np.array([10, 255, 255])),
            'yellow': (np.array([20, 100, 100]), np.array([30, 255, 255])),
            'green': (np.array([40, 50, 50]), np.array([90, 255, 255]))
        }
        
        self.get_logger().info('Traffic light detector initialized')

    def detect_traffic_light(self, image):
        # Convert image to RGB for YOLO
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # Run YOLO inference
        results = self.model(image_rgb)
        
        # Process detections
        detections = results.xyxy[0]  # xyxy format: xmin, ymin, xmax, ymax, confidence, class
        
        for *box, confidence, cls in detections:
            if int(cls) == self.TRAFFIC_LIGHT_CLASS_ID:
                # Get bounding box coordinates
                x1, y1, x2, y2 = map(int, box)
                
                # Crop the detected traffic light area
                traffic_light_img = image[y1:y2, x1:x2]
                
                if traffic_light_img.size == 0:
                    continue
                
                # Convert to HSV
                hsv = cv2.cvtColor(traffic_light_img, cv2.COLOR_BGR2HSV)
                
                # Detect colors
                color_counts = {
                    'red': cv2.countNonZero(cv2.inRange(hsv, *self.color_ranges['red'])),
                    'yellow': cv2.countNonZero(cv2.inRange(hsv, *self.color_ranges['yellow'])),
                    'green': cv2.countNonZero(cv2.inRange(hsv, *self.color_ranges['green']))
                }
                
                # Determine dominant color
                max_color = max(color_counts.items(), key=lambda x: x[1])
                
                # Draw bounding box and label
                if max_color[1] > 0:
                    color_text = {
                        'red': ('GO', (0, 0, 255)),
                        'yellow': ('WAIT', (0, 255, 255)),
                        'green': ('STOP', (0, 255, 0))
                    }[max_color[0]]
                    
                    cv2.putText(image, color_text[0], (x1, y1 - 10),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.9, color_text[1], 2)
                    cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    
                    # Publish status
                    msg = String()
                    msg.data = color_text[0]
                    self.status_publisher.publish(msg)
                    
                    self.get_logger().info(f'Traffic light detected: {color_text[0]}')
        
        return image

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Prcess frame for traffic light detection
            processed_frame = self.detect_traffic_light(frame)
            
            # Convert processed frame to ROS message and publish
            msg = self.cv_bridge.cv2_to_imgmsg(processed_frame, encoding='bgr8')
            self.image_publisher.publish(msg)
        else:
            self.get_logger().warning('Failed to capture frame')

    def __del__(self):
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    detector = TrafficLightDetector()
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
