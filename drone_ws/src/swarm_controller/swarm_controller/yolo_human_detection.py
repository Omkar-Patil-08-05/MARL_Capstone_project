import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json
import os

class YoloHumanDetection(Node):
    def __init__(self):
        super().__init__('yolo_human_detection')
        
        self.declare_parameter('drone_id', 0)
        self.drone_id = self.get_parameter('drone_id').value
        
        self.declare_parameter('vision_mode', os.environ.get('VISION_MODE', 'mock'))
        self.vision_mode = self.get_parameter('vision_mode').value.lower()
        
        self.declare_parameter('model_path', '/home/capstone/capstone_project_antigravity/models/yolov8n.pt')
        self.model_path = self.get_parameter('model_path').value
        
        self.bridge = CvBridge()
        self.model = None
        
        if self.vision_mode == 'yolo':
            try:
                from ultralytics import YOLO
                self.model = YOLO(self.model_path)
                self.get_logger().info(f'Loaded REAL YOLO model from {self.model_path}')
            except ImportError:
                self.get_logger().warn('ultralytics package not found. Falling back to MOCK mode.')
                self.vision_mode = 'mock'
            except Exception as e:
                self.get_logger().warn(f'Failed to load YOLO model: {e}. Expected at {self.model_path}. Falling back to MOCK mode.')
                self.vision_mode = 'mock'
        
        if self.vision_mode == 'mock':
            self.get_logger().info('Running in MOCK YOLO mode (Executable without ultralytics).')

        topic_in = f'/drone_{self.drone_id}/camera/image_raw'
        topic_out_img = f'/drone_{self.drone_id}/camera/detection_image'
        topic_out_data = f'/drone_{self.drone_id}/camera/detection_data'
        
        self.subscription = self.create_subscription(
            Image,
            topic_in,
            self.image_callback,
            10)
            
        self.publisher_img = self.create_publisher(Image, topic_out_img, 10)
        self.publisher_data = self.create_publisher(String, topic_out_data, 10)
        
        self.get_logger().info(f'YOLO Human Detection [{self.vision_mode.upper()}] initialized for drone {self.drone_id} on {topic_in}')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            detections = []
            
            if self.vision_mode == 'yolo' and self.model is not None:
                results = self.model(cv_image, verbose=False)
                # Parse results for class 0 (person)
                for r in results:
                    boxes = r.boxes
                    for box in boxes:
                        if int(box.cls[0]) == 0:  # 0 is 'person' in COCO
                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            conf = float(box.conf[0])
                            if conf > 0.5:
                                detections.append({
                                    'bbox': [x1, y1, x2, y2],
                                    'confidence': conf,
                                    'class': 'person'
                                })
                                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                                cv2.putText(cv_image, f"Human {conf:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            else:
                # Mock inference
                # In mock mode, we assume a person is always in the center for testing
                h, w = cv_image.shape[:2]
                cx, cy = w // 2, h // 2
                x1, y1, x2, y2 = cx - 50, cy - 100, cx + 50, cy + 100
                conf = 0.95
                detections.append({
                    'bbox': [x1, y1, x2, y2],
                    'confidence': conf,
                    'class': 'person'
                })
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(cv_image, f"Human {conf:.2f} (MOCK)", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Create detection data message
            detection_data = {
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                'drone_id': self.drone_id,
                'source': self.vision_mode.upper(),
                'image_width': cv_image.shape[1],
                'image_height': cv_image.shape[0],
                'detections': detections
            }
            data_msg = String()
            data_msg.data = json.dumps(detection_data)
            self.publisher_data.publish(data_msg)
            
            # Publish annotated image
            out_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            out_msg.header = msg.header
            self.publisher_img.publish(out_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloHumanDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
