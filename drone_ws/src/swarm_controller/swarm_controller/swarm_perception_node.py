#!/usr/bin/env python3
"""
Centralized Swarm Perception Node (Phase 2 Remediation - Stage 2)

Features:
  1. Batched GPU Inference: Collects latest ready camera frames from all active drones
     and executes batched Tensor inference on RTX 3050 Laptop GPU.
  2. Starvation-Free Fair Scheduling: Uses rotating round-robin start offsets to ensure
     fair servicing even under partial or constrained batch sizes.
  3. Single-Frame Guarantee: Tracks inference state per frame to ensure no camera frame
     is ever inferred twice.
  4. Non-Blocking Image Ingestion: Fast O(1) buffer updates in image callbacks prevent
     blocking ROS 2 transport while GPU inference executes.
  5. Simulation Timestamp Preservation: Strictly propagates original camera simulation
     clock timestamps through to perception detection messages.
  6. Comprehensive Profiling: Tracks sustained total FPS, per-drone FPS, batch latency,
     P95 latency, frame age, and dropped frames.
"""

import os
import sys
import time
import json
import threading
from typing import Dict, List, Optional, Any

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

yolo_site_packages = '/home/capstone/yolo_venv/lib/python3.12/site-packages'
if os.path.exists(yolo_site_packages) and yolo_site_packages not in sys.path:
    sys.path.insert(0, yolo_site_packages)

try:
    import torch
    from ultralytics import YOLO
    ULTRA_AVAILABLE = True
except ImportError:
    ULTRA_AVAILABLE = False


class SwarmPerceptionNode(Node):
    def __init__(self):
        super().__init__('swarm_perception_node')

        # Parameters
        self.declare_parameter('model_path', '/home/capstone/capstone_project_antigravity/models/yolov8n.pt')
        self.declare_parameter('debug_images', False)
        self.declare_parameter('max_batch_size', 6)
        self.declare_parameter('min_confidence', 0.25)
        self.declare_parameter('poll_rate_hz', 25.0)

        self.model_path = self.get_parameter('model_path').value
        self.debug_images = self.get_parameter('debug_images').value
        self.max_batch_size = int(self.get_parameter('max_batch_size').value)
        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.poll_rate_hz = float(self.get_parameter('poll_rate_hz').value)

        self.bridge = CvBridge()
        self.model = None
        self.num_drones = 6
        self.device = "cuda:0" if (ULTRA_AVAILABLE and torch.cuda.is_available()) else "cpu"

        # Load YOLO model
        if not ULTRA_AVAILABLE:
            self.get_logger().error('CRITICAL: ultralytics is not available in environment! Perception DISABLED.')
        else:
            try:
                self.model = YOLO(self.model_path)
                self.model.to(self.device)
                self.get_logger().info(
                    f"Centralized YOLO model loaded from {self.model_path} on {self.device} "
                    f"(Max Batch Size: {self.max_batch_size})"
                )
            except Exception as e:
                self.get_logger().error(f"CRITICAL: Failed to load YOLO model: {e}")

        # Strict Person-Only Class Resolution (dynamically query model class names)
        self.person_class_ids = set()
        if self.model is not None and hasattr(self.model, 'names') and isinstance(self.model.names, dict):
            for c_id, c_name in self.model.names.items():
                if str(c_name).strip().lower() == 'person':
                    self.person_class_ids.add(int(c_id))
        if not self.person_class_ids:
            self.person_class_ids = {0}  # Standard COCO fallback (class 0 = person)
        self.get_logger().info(f"YOLO Person Class Filter active: target class IDs = {sorted(list(self.person_class_ids))}")

        # Bounded frame buffer per drone
        # Holds: {'msg': Image, 'ts': float, 'seq': int, 'inferred': bool, 'recv_time': float}
        self.frame_buffers: Dict[int, Optional[Dict[str, Any]]] = {i: None for i in range(self.num_drones)}
        self.buffer_lock = threading.Lock()
        self.inference_lock = threading.Lock()

        # Profiling & Metrics Counters
        self.total_frames_inferred = 0
        self.frames_inferred_per_drone = {i: 0 for i in range(self.num_drones)}
        self.dropped_frames_per_drone = {i: 0 for i in range(self.num_drones)}
        self.batch_latencies_ms: List[float] = []
        self.frame_ages_sec: List[float] = []
        self.start_time = time.time()
        self.last_profile_log = time.time()
        self.PROFILE_INTERVAL_SEC = 5.0

        # Detailed Classification Metrics
        self.total_detections_evaluated = 0
        self.total_person_detections_accepted = 0
        self.total_non_person_rejected = 0
        self.total_low_conf_rejected = 0
        self.non_person_rejections_by_class: Dict[str, int] = {}

        # Fair scheduling offset
        self.next_drone_offset = 0

        # ROS 2 Subscriptions and Publishers
        self.subscriptions_list = []
        self.publishers_data: Dict[int, rclpy.publisher.Publisher] = {}
        self.publishers_img: Dict[int, rclpy.publisher.Publisher] = {}

        for i in range(self.num_drones):
            # Input camera image
            topic_in = f'/drone_{i}/camera/image'
            sub = self.create_subscription(
                Image,
                topic_in,
                lambda msg, drone_id=i: self.image_callback(msg, drone_id),
                qos_profile_sensor_data
            )
            self.subscriptions_list.append(sub)

            # Output detection data
            topic_out_data = f'/drone_{i}/perception/detections'
            self.publishers_data[i] = self.create_publisher(String, topic_out_data, 10)

            # Output annotated detection image
            topic_out_img = f'/drone_{i}/camera/detection_image'
            self.publishers_img[i] = self.create_publisher(Image, topic_out_img, qos_profile_sensor_data)

            self.get_logger().info(f"Drone {i}: Subscribed {topic_in} -> Publishing {topic_out_data}")

        # High-frequency batched inference polling timer (runs non-blocking)
        timer_period = 1.0 / self.poll_rate_hz
        self.timer = self.create_timer(timer_period, self.process_batch)

        self.get_logger().info(
            f"Swarm Perception Node initialized with Batched Multi-Camera Scheduler "
            f"(Poll Rate: {self.poll_rate_hz} Hz, Device: {self.device})"
        )

    def image_callback(self, msg: Image, drone_id: int):
        """
        Fast O(1) non-blocking frame buffer update.
        Replaces older frame while tracking drops if older frame was never inferred.
        """
        sim_ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        wall_recv = time.time()

        with self.buffer_lock:
            existing = self.frame_buffers[drone_id]
            if existing is not None and not existing['inferred']:
                # The previous frame was overwritten before GPU could infer it
                self.dropped_frames_per_drone[drone_id] += 1

            self.frame_buffers[drone_id] = {
                'msg': msg,
                'sim_ts': sim_ts,
                'wall_recv': wall_recv,
                'inferred': False
            }

    def process_batch(self):
        """
        Periodic scheduler collecting up to max_batch_size ready frames
        and executing single-pass GPU batched inference.
        """
        if self.model is None:
            return

        # Ensure only one inference batch runs at a time without blocking the timer loop
        if not self.inference_lock.acquire(blocking=False):
            return

        try:
            batch_drones: List[int] = []
            batch_cv_images: List[np.ndarray] = []
            batch_records: List[Dict[str, Any]] = []

            # 1. Collect available un-inferred frames with round-robin fair starting index
            with self.buffer_lock:
                start_offset = self.next_drone_offset
                for step in range(self.num_drones):
                    d_id = (start_offset + step) % self.num_drones
                    rec = self.frame_buffers[d_id]
                    if rec is not None and not rec['inferred']:
                        # Mark as inferred immediately so it cannot be collected twice
                        rec['inferred'] = True
                        batch_drones.append(d_id)
                        batch_records.append(rec)

                        # Check batch size limit
                        if len(batch_drones) >= self.max_batch_size:
                            # Advance starting offset to prevent starvation
                            self.next_drone_offset = (d_id + 1) % self.num_drones
                            break
                else:
                    # If we checked all drones, advance offset by 1
                    self.next_drone_offset = (self.next_drone_offset + 1) % self.num_drones

            # If no camera frames are ready, release and return
            if not batch_drones:
                return

            # 2. Convert ROS images to OpenCV BGR arrays
            for rec in batch_records:
                try:
                    cv_img = self.bridge.imgmsg_to_cv2(rec['msg'], 'bgr8')
                    batch_cv_images.append(cv_img)
                except Exception as e:
                    self.get_logger().error(f"Image cv2 conversion error for drone {rec['msg'].header.frame_id}: {e}")
                    batch_cv_images.append(None)

            # Filter valid images
            valid_indices = [idx for idx, img in enumerate(batch_cv_images) if img is not None]
            if not valid_indices:
                return

            run_imgs = [batch_cv_images[i] for i in valid_indices]
            run_drones = [batch_drones[i] for i in valid_indices]
            run_records = [batch_records[i] for i in valid_indices]

            # 3. Execute single-pass GPU batched inference
            t_infer_start = time.perf_counter()
            results = self.model(run_imgs, verbose=False)
            if torch.cuda.is_available():
                torch.cuda.synchronize()
            t_infer_end = time.perf_counter()

            batch_lat_ms = (t_infer_end - t_infer_start) * 1000.0
            self.batch_latencies_ms.append(batch_lat_ms)
            if len(self.batch_latencies_ms) > 300:
                self.batch_latencies_ms.pop(0)

            # 4. Dispatch detections per drone independently
            current_wall = time.time()
            for k, (drone_id, rec, cv_img, r) in enumerate(zip(run_drones, run_records, run_imgs, results)):
                msg = rec['msg']
                sim_ts = rec['sim_ts']

                # Measure frame age from arrival to inference completion
                frame_age = current_wall - rec['wall_recv']
                self.frame_ages_sec.append(frame_age)
                if len(self.frame_ages_sec) > 300:
                    self.frame_ages_sec.pop(0)

                detections = []
                rejected_non_person = []
                boxes = r.boxes
                for box in boxes:
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    cls_name = self.model.names.get(cls_id, f"class_{cls_id}") if (self.model and hasattr(self.model, 'names')) else f"class_{cls_id}"
                    
                    self.total_detections_evaluated += 1

                    # Strict Person-Only Filtering:
                    # Only accept candidates whose predicted class maps to 'person'
                    if cls_id in self.person_class_ids:
                        if conf >= self.min_confidence:
                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            detections.append({
                                'bbox': [x1, y1, x2, y2],
                                'confidence': conf,
                                'class': 'person',
                                'class_id': cls_id
                            })
                            self.total_person_detections_accepted += 1
                        else:
                            self.total_low_conf_rejected += 1
                    else:
                        # Strictly reject cars, bicycles, animals, buildings, rubble, etc.
                        rejected_non_person.append((cls_name, conf))
                        self.total_non_person_rejected += 1
                        self.non_person_rejections_by_class[cls_name] = self.non_person_rejections_by_class.get(cls_name, 0) + 1

                if rejected_non_person:
                    self.get_logger().debug(
                        f"Drone {drone_id}: Rejected {len(rejected_non_person)} non-person detections: "
                        f"{[f'{name}(conf={c:.2f})' for name, c in rejected_non_person]}"
                    )

                # Publish detection JSON with explicit detection verification metadata
                detection_data = {
                    'timestamp': sim_ts,
                    'drone_id': drone_id,
                    'image_width': cv_img.shape[1],
                    'image_height': cv_img.shape[0],
                    'detections': detections,
                    'non_person_rejected_count': len(rejected_non_person)
                }

                data_msg = String()
                data_msg.data = json.dumps(detection_data)
                self.publishers_data[drone_id].publish(data_msg)

                self.total_frames_inferred += 1
                self.frames_inferred_per_drone[drone_id] += 1

                # Log verified detections
                if detections:
                    for det in detections:
                        self.get_logger().info(
                            f"[DETECTION Drone {drone_id}] Person conf={det['confidence']:.3f} "
                            f"bbox={det['bbox']} at sim_ts={sim_ts:.3f}"
                        )

                # Optional debug image publishing
                if self.debug_images:
                    for det in detections:
                        x1, y1, x2, y2 = det['bbox']
                        conf = det['confidence']
                        cv2.rectangle(cv_img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        cv2.putText(
                            cv_img,
                            f"Person: {conf:.2f}",
                            (x1, max(15, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 0, 255),
                            2
                        )
                    out_msg = self.bridge.cv2_to_imgmsg(cv_img, 'bgr8')
                    out_msg.header = msg.header
                    self.publishers_img[drone_id].publish(out_msg)

            # 5. Periodic profiling statistics
            if current_wall - self.last_profile_log >= self.PROFILE_INTERVAL_SEC:
                self._log_profiling_summary(current_wall)

        except Exception as e:
            self.get_logger().error(f"Error during batched perception inference: {e}")
        finally:
            self.inference_lock.release()

    def _log_profiling_summary(self, current_wall: float):
        elapsed = current_wall - self.start_time
        if elapsed <= 0 or not self.batch_latencies_ms:
            return

        total_fps = self.total_frames_inferred / elapsed
        per_drone_fps_str = ", ".join(
            f"D{d}:{self.frames_inferred_per_drone[d] / elapsed:.1f}" for d in range(self.num_drones)
        )
        mean_lat = np.mean(self.batch_latencies_ms)
        p95_lat = np.percentile(self.batch_latencies_ms, 95)
        mean_age = np.mean(self.frame_ages_sec) * 1000.0 if self.frame_ages_sec else 0.0

        vram_mb = torch.cuda.memory_allocated() / (1024 * 1024) if torch.cuda.is_available() else 0.0

        self.get_logger().info(
            f"[SwarmPerception Batching] Total: {total_fps:.1f} FPS | "
            f"PerDrone [{per_drone_fps_str}] | "
            f"Batch Lat: mean={mean_lat:.1f}ms, p95={p95_lat:.1f}ms | "
            f"Frame Age: {mean_age:.1f}ms | VRAM: {vram_mb:.1f}MB"
        )
        self.last_profile_log = current_wall


def main(args=None):
    rclpy.init(args=args)
    node = SwarmPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
