#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json
import time
from rclpy.qos import qos_profile_sensor_data

try:
    from ultralytics import YOLO
    ULTRA_AVAILABLE = True
except ImportError:
    ULTRA_AVAILABLE = False
    print("CRITICAL: ultralytics is not installed in the current environment!")

class SwarmPerceptionNode(Node):
    def __init__(self):
        super().__init__('swarm_perception_node')
        
        self.declare_parameter('model_path', '/home/capstone/capstone_project_antigravity/models/yolov8n.pt')
        self.declare_parameter('debug_images', False)
        
        self.model_path = self.get_parameter('model_path').value
        self.debug_images = self.get_parameter('debug_images').value
        
        self.bridge = CvBridge()
        self.model = None
        self.num_drones = 6
        
        if not ULTRA_AVAILABLE:
            self.get_logger().error('CRITICAL: ultralytics not available. YOLO perception DISABLED.')
        else:
            try:
                self.model = YOLO(self.model_path)
                self.get_logger().info(f'Loaded centralized YOLO model from {self.model_path}')
            except Exception as e:
                self.get_logger().error(f'CRITICAL: Failed to load YOLO model: {e}')
        
        self.latest_frames = {i: None for i in range(self.num_drones)}
        self.subscriptions_list = []
        self.publishers_data = {}
        self.publishers_img = {}
        
        for i in range(self.num_drones):
            # Subscriptions
            topic_in = f'/drone_{i}/camera/image'
            sub = self.create_subscription(
                Image,
                topic_in,
                lambda msg, drone_id=i: self.image_callback(msg, drone_id),
                qos_profile_sensor_data)
            self.subscriptions_list.append(sub)
            
            # Publishers
            topic_out_data = f'/drone_{i}/perception/detections'
            self.publishers_data[i] = self.create_publisher(String, topic_out_data, 10)
            
            topic_out_img = f'/drone_{i}/camera/detection_image'
            self.publishers_img[i] = self.create_publisher(Image, topic_out_img, qos_profile_sensor_data)
                
            self.get_logger().info(f'Subscribed to {topic_in} | Publishing to {topic_out_data}')
        
        # Scheduler
        self.current_drone_index = 0
        timer_period = 0.2  # 5 Hz — matches camera update rate, sufficient for visual demo
        self.timer = self.create_timer(timer_period, self.process_next_frame)
        
        # Throttled logging: only log profiling every N inferences to reduce GIL contention
        self.inference_count = 0
        self.LOG_EVERY_N = 30  # Log profiling once every 30 inferences (~every 2.4s)
        
        self.get_logger().info('Swarm Perception Node initialized with round-robin scheduling.')

    def image_callback(self, msg, drone_id):
        # Store only the latest frame, dropping older ones instantly
        self.latest_frames[drone_id] = msg

    def process_next_frame(self):
        start_idx = self.current_drone_index
        processed = False
        
        while not processed:
            drone_id = self.current_drone_index
            msg = self.latest_frames[drone_id]
            
            # Advance index for the next timer call
            self.current_drone_index = (self.current_drone_index + 1) % self.num_drones
            
            if msg is not None:
                # We have a frame, consume it
                self.latest_frames[drone_id] = None
                self.run_inference(msg, drone_id)
                processed = True
            else:
                # If we've looped through all drones and found nothing, exit timer
                if self.current_drone_index == start_idx:
                    break

    def run_inference(self, msg, drone_id):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            detections = []
            
            if self.model is not None:
                results = self.model(cv_image, verbose=False)
                
                for r in results:
                    boxes = r.boxes
                    for box in boxes:
                        if int(box.cls[0]) == 0:  # person
                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            conf = float(box.conf[0])
                            if conf > 0.25:
                                detections.append({
                                    'bbox': [x1, y1, x2, y2],
                                    'confidence': conf,
                                    'class': 'person'
                                })
            
            detection_data = {
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                'drone_id': drone_id,
                'image_width': cv_image.shape[1],
                'image_height': cv_image.shape[0],
                'detections': detections
            }
            
            data_msg = String()
            data_msg.data = json.dumps(detection_data)
            self.publishers_data[drone_id].publish(data_msg)
            
            self.inference_count += 1
            
            # Log detections always (important operational info)
            if detections:
                for det in detections:
                    self.get_logger().info(
                        f"[DETECTION Drone {drone_id}] Person conf={det['confidence']:.3f} "
                        f"bbox={det['bbox']} img={cv_image.shape[1]}x{cv_image.shape[0]}"
                    )
            
            # Log profiling only periodically to reduce GIL/CPU contention
            if self.inference_count % self.LOG_EVERY_N == 0:
                self.get_logger().info(
                    f"[Perception] Inference #{self.inference_count} "
                    f"Drone {drone_id} | "
                    f"img={cv_image.shape[1]}x{cv_image.shape[0]} | "
                    f"Device: {self.model.device.type if self.model else 'N/A'}"
                )
            
            # Draw bounding boxes
            for det in detections:
                x1, y1, x2, y2 = det['bbox']
                conf = det['confidence']
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(cv_image, f"Person: {conf:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                
            out_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            out_msg.header = msg.header
            self.publishers_img[drone_id].publish(out_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error processing image for drone {drone_id}: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SwarmPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
