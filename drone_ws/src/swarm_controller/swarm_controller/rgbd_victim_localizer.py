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
    def __init__(self, fx=539.9, fy=539.9, cx=640.0, cy=480.0):
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

class EventConfirmer:
    def __init__(self, window_size=3, consensus_needed=2, proximity_threshold=2.0, grid_cell_size=4.0):
        self.window_size = window_size
        self.consensus_needed = consensus_needed
        self.proximity_threshold = proximity_threshold
        self.grid_cell_size = grid_cell_size
        
        self.tracks = {}
        self.next_track_id = 1

    def add_observation(self, drone_id, timestamp, world_x, world_y, confidence, median_depth, valid_samples, depth_spread):
        best_track_id = None
        min_dist = self.proximity_threshold
        
        for track_id, track in self.tracks.items():
            if not track['history']: continue
            last_obs = track['history'][-1]
            dist = math.sqrt((world_x - last_obs['x'])**2 + (world_y - last_obs['y'])**2)
            if dist < min_dist:
                min_dist = dist
                best_track_id = track_id
                
        if best_track_id is None:
            best_track_id = self.next_track_id
            self.next_track_id += 1
            self.tracks[best_track_id] = {'history': deque(maxlen=self.window_size), 'discovered': False, 'last_discovered': 0}
            
        track = self.tracks[best_track_id]
        
        grid_x = int(world_x // self.grid_cell_size)
        grid_y = int(world_y // self.grid_cell_size)
        
        track['history'].append({
            'x': world_x, 'y': world_y, 'gx': grid_x, 'gy': grid_y, 'ts': timestamp,
            'conf': confidence, 'depth': median_depth, 'samples': valid_samples, 'spread': depth_spread,
            'drone': drone_id
        })
        
        discovery_event = None
        
        # Debouncing rule: we only emit if it wasn't discovered recently, or if we want to update the dashboard continuously
        # For this requirement, we only confirm it once initially.
        if not track['discovered'] and len(track['history']) >= self.consensus_needed:
            grid_counts = {}
            for obs in track['history']:
                cell = (obs['gx'], obs['gy'])
                grid_counts[cell] = grid_counts.get(cell, 0) + 1
                
            winning_cell, count = max(grid_counts.items(), key=lambda item: item[1])
            
            if count >= self.consensus_needed:
                track['discovered'] = True
                
                # Take the latest observation in the winning cell as the representative value
                winning_obs = next(obs for obs in reversed(track['history']) if (obs['gx'], obs['gy']) == winning_cell)
                
                discovery_event = {
                    "event_type": "VICTIM_DISCOVERED",
                    "timestamp": float(timestamp),
                    "victim_track_id": int(best_track_id),
                    "confirming_drone_id": int(drone_id),
                    "world_x": float(winning_obs['x']),
                    "world_y": float(winning_obs['y']),
                    "grid_x": int(winning_cell[0]),
                    "grid_y": int(winning_cell[1]),
                    "grid_cell_size_m": float(self.grid_cell_size),
                    "confidence": float(winning_obs['conf']),
                    "median_depth_estimate": float(winning_obs['depth']),
                    "confirmation_count": int(count),
                    "confirmation_window": int(self.window_size),
                    "depth_valid_samples": int(winning_obs['samples']),
                    "depth_spread": float(winning_obs['spread'])
                }
                
        return best_track_id, discovery_event

class RGBDVictimLocalizerNode(Node):
    def __init__(self):
        super().__init__('rgbd_victim_localizer')
        self.bridge = CvBridge()
        self.localizer = RGBDGeometricLocalizer()
        self.confirmer = EventConfirmer(window_size=3, consensus_needed=2, grid_cell_size=4.0)
        
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
        # Using transforms3d
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
        
        if sync_diff > 0.2:
            self.get_logger().warn(f'Drone {drone_id}: depth sync diff {sync_diff:.3f}s > 0.2s, skipping')
            return
            
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(closest_depth_msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed for drone {drone_id}: {e}")
            return
            
        drone_x, drone_y, drone_z, drone_yaw = pose
            
        for det in detections:
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
            
            if len(valid_samples) < 10:
                self.get_logger().warn(f'Drone {drone_id}: detection has only {len(valid_samples)} valid depth samples (<10), skipping')
                continue
                
            median_depth = float(np.median(valid_samples))
            depth_spread = float(np.std(valid_samples))
            valid_count = len(valid_samples)
            
            world_x, world_y = self.localizer.localize_with_depth(cx, cy, median_depth, drone_x, drone_y, drone_z, drone_yaw)
            
            # Filter out drone self-detections (YOLO detecting other drones as victims)
            is_drone = False
            for d_id, d_pose in self.drone_poses.items():
                if d_pose is not None:
                    dx, dy, dz, _ = d_pose
                    if math.hypot(world_x - dx, world_y - dy) < 1.5:
                        is_drone = True
                        break
            if is_drone:
                continue
            
            # Always log successful localizations (important operational info)
            self.get_logger().info(
                f"[Localizer Drone {drone_id}] Person conf={conf:.3f} | "
                f"depth={median_depth:.2f}m (n={valid_count}, spread={depth_spread:.2f}) | "
                f"world=({world_x:.1f}, {world_y:.1f}) | "
                f"pose=({drone_x:.1f}, {drone_y:.1f}, {drone_z:.1f})"
            )
            
            track_id, event = self.confirmer.add_observation(drone_id, det_ts, world_x, world_y, conf, median_depth, valid_count, depth_spread)
            if event:
                msg_out = String()
                msg_out.data = json.dumps(event)
                self.discovery_pub.publish(msg_out)
                self.get_logger().info(
                    f"*** VICTIM_DISCOVERED *** track={track_id} at ({world_x:.1f}, {world_y:.1f}) "
                    f"grid=({event['grid_x']}, {event['grid_y']}) conf={conf:.3f} "
                    f"depth={median_depth:.2f}m confirmations={event['confirmation_count']}"
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


