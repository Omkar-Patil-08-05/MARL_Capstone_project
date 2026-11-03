import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import numpy as np
import json
import math
from collections import deque
import time
from rclpy.qos import qos_profile_sensor_data
from swarm_controller.grid_world_transform import GridWorldTransform
def quat2euler(q):
    w, x, y, z = q
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return [roll_x, pitch_y, yaw_z]

class RGBDGeometricLocalizer:
    def __init__(self, fx=539.9, fy=539.9, cx=320.0, cy=240.0):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy

    def update_intrinsics(self, fx, fy, cx, cy):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy

    def localize_with_depth(self, u, v, measured_depth, drone_x, drone_y, drone_z, drone_yaw):
        """
        Projects a pixel (u,v) with measured depth (Z_c) into the World frame.
        Assumes downward facing camera (pitch=-90 deg).
        """
        x_c = (u - self.cx) * measured_depth / self.fx
        y_c = (v - self.cy) * measured_depth / self.fy
        z_c = measured_depth

        x_d = -y_c
        y_d = -x_c
        z_d = -z_c

        cos_y = math.cos(drone_yaw)
        sin_y = math.sin(drone_yaw)
        
        world_dx = x_d * cos_y - y_d * sin_y
        world_dy = x_d * sin_y + y_d * cos_y
        
        world_x = drone_x + world_dx
        world_y = drone_y + world_dy
        
        return world_x, world_y

try:
    from swarm_controller.temporal_event_confirmer import TemporalEventConfirmer
except ImportError:
    try:
        from .temporal_event_confirmer import TemporalEventConfirmer
    except ImportError:
        from temporal_event_confirmer import TemporalEventConfirmer

# Canonical alias preserving existing localizer interface
EventConfirmer = TemporalEventConfirmer


class RGBDVictimLocalizerNode(Node):
    def __init__(self):
        super().__init__('rgbd_victim_localizer')
        self.bridge = CvBridge()
        self.localizer = RGBDGeometricLocalizer()

        # Initialize GridWorldTransform bounds from environment metadata if present
        meta_file = os.environ.get("WORLD_META_PATH", "/home/capstone/capstone_project_antigravity/worlds/generated_world_meta.json")
        if os.path.exists(meta_file):
            try:
                with open(meta_file, 'r') as f:
                    meta = json.load(f)
                w_cfg = meta.get('search_area') or meta.get('world') or {}
                width = int(w_cfg.get('grid_width', 25))
                height = int(w_cfg.get('grid_height', 25))
                meters_per_cell = float(w_cfg.get('meters_per_cell', 4.0))
                origin_x = float(w_cfg.get('origin_x', w_cfg.get('grid_origin_x', 0.0)))
                origin_y = float(w_cfg.get('origin_y', w_cfg.get('grid_origin_y', 0.0)))
                GridWorldTransform.set_bounds(
                    width=width,
                    height=height,
                    meters_per_cell=meters_per_cell,
                    origin_x=origin_x,
                    origin_y=origin_y
                )
                self.get_logger().info(f"Configured GridWorldTransform from {meta_file}: origin=({origin_x},{origin_y}) cell={meters_per_cell}m dims={width}x{height}")
            except Exception as e:
                self.get_logger().warn(f"Failed to load world bounds from {meta_file}: {e}")

        self.confirmer = EventConfirmer(
            window_size=int(os.environ.get("LOCALIZER_WINDOW_SIZE", "6")),
            consensus_needed=int(os.environ.get("LOCALIZER_CONSENSUS_NEEDED", "2")),
            proximity_threshold=float(os.environ.get("LOCALIZER_PROXIMITY_THRESHOLD", "3.0")),
            grid_cell_size=GridWorldTransform.METERS_PER_CELL,
            min_confidence=float(os.environ.get("LOCALIZER_MIN_CONFIDENCE", "0.28")),
            min_time_span=float(os.environ.get("LOCALIZER_MIN_TIME_SPAN", "0.15")),
            dedup_radius=float(os.environ.get("LOCALIZER_DEDUP_RADIUS", "6.0"))
        )
        
        self.depth_buffers = {i: [] for i in range(6)}
        self.drone_poses = {i: None for i in range(6)}
        
        self.patch_fraction = 0.30
        
        # Throttled logging counters
        self.detection_cb_count = 0
        self.LOG_EVERY_N = 50  # Log sync/processing stats every 50 callbacks
        
        for i in range(6):
            self.create_subscription(String, f'/drone_{i}/perception/detections', 
                                     lambda msg, d=i: self.detection_cb(msg, d), 10)
            self.create_subscription(Image, f'/drone_{i}/camera/depth_image', 
                                     lambda msg, d=i: self.depth_cb(msg, d), qos_profile_sensor_data)
            self.create_subscription(CameraInfo, f'/drone_{i}/camera/camera_info', 
                                     lambda msg, d=i: self.info_cb(msg, d), qos_profile_sensor_data)
            self.create_subscription(PoseStamped, f'/model/drone_{i}/pose',
                                     lambda msg, d=i: self.pose_cb(msg, d), qos_profile_sensor_data)
                                     
        self.discovery_pub = self.create_publisher(String, '/sar/victim_discovery', 10)
        self.get_logger().info('RGB-D Victim Localizer initialized with 6 drone pose streams.')

    def info_cb(self, msg, drone_id):
        fx = msg.k[0]
        cx = msg.k[2]
        fy = msg.k[4]
        cy = msg.k[5]
        self.localizer.update_intrinsics(fx, fy, cx, cy)
        
    def pose_cb(self, msg, drone_id):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        q = msg.pose.orientation
        euler = quat2euler([q.w, q.x, q.y, q.z])
        yaw = euler[2]
        self.drone_poses[drone_id] = (x, y, z, yaw)

    def depth_cb(self, msg, drone_id):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.depth_buffers[drone_id].append((ts, msg))
        if len(self.depth_buffers[drone_id]) > 30:
            self.depth_buffers[drone_id].pop(0)

    def detection_cb(self, msg, drone_id):
        data = json.loads(msg.data)
        det_ts = data['timestamp']
        detections = data['detections']
        
        if not detections:
            return
            
        pose = self.drone_poses[drone_id]
        if pose is None:
            self.get_logger().warn(f'Drone {drone_id}: detection received but no pose yet, skipping')
            return # Awaiting first pose message
            
        buffer = self.depth_buffers[drone_id]
        if not buffer:
            self.get_logger().warn(f'Drone {drone_id}: detection received but no depth buffer, skipping')
            return
            
        closest_depth_msg = min(buffer, key=lambda item: abs(item[0] - det_ts))[1]
        sync_diff = abs(det_ts - (closest_depth_msg.header.stamp.sec + closest_depth_msg.header.stamp.nanosec * 1e-9))
        
        # Accommodate both 5 Hz (0.2s) and 2 Hz (0.5s) camera rates
        max_sync_tolerance = float(os.environ.get("DEPTH_SYNC_TOLERANCE", "0.6"))
        if sync_diff > max_sync_tolerance:
            self.get_logger().warn(f'Drone {drone_id}: depth sync diff {sync_diff:.3f}s > {max_sync_tolerance:.1f}s, skipping')
            return
            
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(closest_depth_msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed for drone {drone_id}: {e}")
            return
            
        drone_x, drone_y, drone_z, drone_yaw = pose
            
        for det in detections:
            # Defensive strict check: only process candidates classified as 'person'
            det_cls = str(det.get('class', 'person')).strip().lower()
            if det_cls != 'person':
                self.get_logger().warn(f'Drone {drone_id}: Ignoring non-person detection class="{det_cls}"')
                continue

            x1, y1, x2, y2 = det['bbox']
            conf = det['confidence']
            w = x2 - x1
            h = y2 - y1
            
            pw = int(w * self.patch_fraction)
            ph = int(h * self.patch_fraction)
            
            cx = int(x1 + w/2)
            cy = int(y1 + h/2)
            
            px1 = max(0, cx - pw//2)
            px2 = min(cv_depth.shape[1], cx + pw//2)
            py1 = max(0, cy - ph//2)
            py2 = min(cv_depth.shape[0], cy + ph//2)
            
            patch = cv_depth[py1:py2, px1:px2]
            
            valid_mask = (patch > 0) & (~np.isnan(patch)) & (~np.isinf(patch))
            valid_samples = patch[valid_mask]
            
            if len(valid_samples) < 15:
                self.get_logger().debug(f'Drone {drone_id}: detection has only {len(valid_samples)} valid depth samples (<15), skipping')
                continue
                
            median_depth = float(np.median(valid_samples))
            depth_spread = float(np.std(valid_samples))
            valid_count = len(valid_samples)
            
            world_x, world_y = self.localizer.localize_with_depth(cx, cy, median_depth, drone_x, drone_y, drone_z, drone_yaw)
            
            # Filter out drone self-detections (YOLO detecting OTHER drones as victims)
            is_drone = False
            for d_id, d_pose in self.drone_poses.items():
                if d_id == drone_id:
                    continue  # Do not filter out targets directly beneath the detecting drone
                if d_pose is not None:
                    dx, dy, dz, _ = d_pose
                    if math.hypot(world_x - dx, world_y - dy) < 1.5:
                        is_drone = True
                        break
            if is_drone:
                continue
            
            # Always log candidate localizations (distinct from confirmed discoveries)
            self.get_logger().info(
                f"[Localizer Candidate Drone {drone_id}] Person conf={conf:.3f} | "
                f"depth={median_depth:.2f}m (n={valid_count}, spread={depth_spread:.2f}) | "
                f"world=({world_x:.1f}, {world_y:.1f}) | "
                f"pose=({drone_x:.1f}, {drone_y:.1f}, {drone_z:.1f})"
            )
            
            # Validate candidate detection against bounds, confidence, depth, and duplicate criteria
            is_valid, reason = self.confirmer.is_valid_observation(
                world_x, world_y, conf, median_depth, valid_count, depth_spread
            )
            if not is_valid:
                self.get_logger().info(
                    f"[Localizer Candidate Drone {drone_id}] REJECTED ({reason}): "
                    f"conf={conf:.3f}, depth={median_depth:.1f}m, world=({world_x:.1f}, {world_y:.1f})"
                )
                continue

            track_id, event = self.confirmer.add_observation(
                drone_id, det_ts, world_x, world_y, conf, median_depth, valid_count, depth_spread
            )
            if event:
                msg_out = String()
                msg_out.data = json.dumps(event)
                self.discovery_pub.publish(msg_out)
                self.get_logger().info(
                    f"*** VICTIM_DISCOVERED *** track={track_id} at ({event['world_x']}, {event['world_y']}) "
                    f"grid=({event['grid_x']}, {event['grid_y']}) conf={event['confidence']} "
                    f"depth={event['median_depth_estimate']}m consensus={event['confirmation_count']}/{event['confirmation_window']} "
                    f"span={event['consensus_span_sec']}s"
                )

def main(args=None):
    rclpy.init(args=args)
    node = RGBDVictimLocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


