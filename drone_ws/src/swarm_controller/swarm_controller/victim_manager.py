import subprocess
from enum import Enum
import time
import random
import math
from .grid_world_transform import GridWorldTransform

class VictimState(Enum):
    UNDETECTED = "UNDETECTED"
    DETECTED = "DETECTED"
    RESCUED = "RESCUED"

class MovementState(Enum):
    RESTING = "RESTING"
    MOVING = "MOVING"

class Victim:
    def __init__(self, v_id, grid_x, grid_y, world_x, world_y, profile_config, random_gen):
        self.id = v_id
        self.grid_x = grid_x
        self.grid_y = grid_y
        self.world_x = world_x
        self.world_y = world_y
        
        self.state = VictimState.UNDETECTED
        self.movement_state = MovementState.RESTING
        
        self.profile = profile_config
        self.rng = random_gen
        
        self.speed = self.rng.uniform(self.profile['min_speed'], self.profile['max_speed'])
        self.rest_timer = self.rng.uniform(0.0, self.profile['max_rest_duration'])
        
        self.dest_world_x = self.world_x
        self.dest_world_y = self.world_y
        
        self.last_gz_update_time = time.time()
        
    def get_dict(self):
        return {
            "id": self.id,
            "x": self.grid_x,
            "y": self.grid_y,
            "world_x": self.world_x,
            "world_y": self.world_y,
            "detected": self.state == VictimState.DETECTED or self.state == VictimState.RESCUED,
            "state": self.state.value,
            "movement_state": self.movement_state.value
        }

class TrackedVictim:
    def __init__(self, t_id, world_x, world_y, source, confidence, drone_id):
        self.id = t_id
        self.world_x = world_x
        self.world_y = world_y
        self.grid_x, self.grid_y = GridWorldTransform.world_to_grid(world_x, world_y)
        self.source = source
        self.confidence = confidence
        self.detected_by = drone_id
        self.first_seen = time.time()
        self.last_seen = time.time()
        self.status = "DETECTED"

    def update(self, world_x, world_y, source, confidence, drone_id):
        # Exponential moving average for position smoothing
        alpha = 0.7
        self.world_x = self.world_x * (1 - alpha) + world_x * alpha
        self.world_y = self.world_y * (1 - alpha) + world_y * alpha
        self.grid_x, self.grid_y = GridWorldTransform.world_to_grid(self.world_x, self.world_y)
        self.source = source
        self.confidence = confidence
        self.detected_by = drone_id
        self.last_seen = time.time()
        self.status = "DETECTED"
        
    def check_status(self, current_time):
        age = current_time - self.last_seen
        if age > 15.0:
            self.status = "LOST"
        elif age > 5.0:
            self.status = "SEARCHING"
        else:
            self.status = "DETECTED"

    def get_dict(self):
        return {
            "id": self.id,
            "x": self.grid_x,
            "y": self.grid_y,
            "world_x": self.world_x,
            "world_y": self.world_y,
            "detected": True,
            "state": self.status,
            "source": self.source,
            "confidence": self.confidence,
            "detected_by": self.detected_by,
            "last_seen_sec_ago": round(time.time() - self.last_seen, 1)
        }

class VictimManager:
    def __init__(self, node, metadata, world_name, env, seed=42):
        self.node = node
        self.world_name = world_name
        self.env = env # SARGridEnv reference for obstacle checking
        self.victims = {} # Ground truth victims
        self.tracked_victims = {} # Camera-detected tracked victims
        self.next_track_id = 0
        self.association_radius = 12.0 # Meters
        
        self.rng = random.Random(seed)
        
        # Victim Profiles
        self.profiles = {
            "stationary": {"movement_enabled": False, "min_speed": 0.0, "max_speed": 0.0, "min_rest_duration": 9999, "max_rest_duration": 9999, "radius": 0},
            "slow": {"movement_enabled": True, "min_speed": 0.2, "max_speed": 0.4, "min_rest_duration": 5.0, "max_rest_duration": 15.0, "radius": 12.0},
            "intermittent": {"movement_enabled": True, "min_speed": 0.4, "max_speed": 0.8, "min_rest_duration": 10.0, "max_rest_duration": 30.0, "radius": 20.0},
            "moderate": {"movement_enabled": True, "min_speed": 0.6, "max_speed": 1.2, "min_rest_duration": 3.0, "max_rest_duration": 10.0, "radius": 30.0},
            "wanderer": {"movement_enabled": True, "min_speed": 0.3, "max_speed": 0.7, "min_rest_duration": 2.0, "max_rest_duration": 8.0, "radius": 40.0},
        }
        
        self.parse_metadata(metadata)
        
        self.waypoint_tolerance = 0.5 # meters
        self.gz_update_interval = 1.0 # Throttle Gazebo service calls to 1 Hz to prevent performance drop
        self.last_update_time = time.time()

    def parse_metadata(self, metadata):
        victims_meta = metadata.get("victims", [])
        profile_names = list(self.profiles.keys())
        
        for idx, v_meta in enumerate(victims_meta):
            v_id = v_meta.get("id", f"V{idx}")
            grid_x = v_meta["grid"]["x"]
            grid_y = v_meta["grid"]["y"]
            world_x = v_meta["world"]["x"]
            world_y = v_meta["world"]["y"]
            
            # Assign varied behavior profiles based on index for variety
            if idx == 0:
                p_name = "stationary"
            elif idx == 1:
                p_name = "slow"
            elif idx == 2:
                p_name = "intermittent"
            elif idx == 3:
                p_name = "moderate"
            else:
                p_name = "wanderer"
                
            self.victims[v_id] = Victim(v_id, grid_x, grid_y, world_x, world_y, self.profiles[p_name], self.rng)

    def spawn_all(self):
        """Spawns all victims into Gazebo using the rescue_randy_sitting model."""
        for v_id, victim in self.victims.items():
            self.node.get_logger().info(f"[VictimManager] Spawning {v_id} at world({victim.world_x:.1f}, {victim.world_y:.1f}) with profile {victim.profile['movement_enabled']}")
            
            # Using rescue_randy_sitting model
            model_path = "/home/capstone/capstone_project_antigravity/assets_real/victims/rescue_randy_sitting/model.sdf"
            cmd = [
                'gz', 'service', '-s', f'/world/{self.world_name}/create',
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--req', f'sdf_filename: "{model_path}", name: "{v_id}", pose: {{position: {{x: {victim.world_x}, y: {victim.world_y}, z: 0.1}}}}'
            ]
            subprocess.Popen(cmd)
            time.sleep(0.1)

    def process_visual_detection(self, world_x, world_y, source, confidence, drone_id):
        """Processes an incoming visual detection from YOLO/Mock and maintains stable tracks."""
        best_dist = float('inf')
        best_track_id = None
        
        # 1. Try to associate with an existing track
        for t_id, track in self.tracked_victims.items():
            dist = math.hypot(track.world_x - world_x, track.world_y - world_y)
            if dist < best_dist and dist < self.association_radius:
                best_dist = dist
                best_track_id = t_id
                
        if best_track_id is not None:
            # Update existing track
            self.tracked_victims[best_track_id].update(world_x, world_y, source, confidence, drone_id)
        else:
            # Create new track
            t_id = f"T{self.next_track_id}"
            self.next_track_id += 1
            self.tracked_victims[t_id] = TrackedVictim(t_id, world_x, world_y, source, confidence, drone_id)
            self.node.get_logger().info(f"[VictimTracker] New victim track {t_id} initialized at ({world_x:.1f}, {world_y:.1f}) by D{drone_id}")
            
        # 2. Also associate with ground truth to mark internal state DETECTED for the RL env
        # (This preserves the RL coverage calculation without exposing ground truth to perception)
        gt_best_dist = float('inf')
        gt_best_id = None
        for v_id, victim in self.victims.items():
            dist = math.hypot(victim.world_x - world_x, victim.world_y - world_y)
            if dist < gt_best_dist and dist < self.association_radius:
                gt_best_dist = dist
                gt_best_id = v_id
                
        if gt_best_id is not None:
            self.mark_detected(gt_best_id)
            # Part J: Calculate localization error metrics
            gt_v = self.victims[gt_best_id]
            err_m = gt_best_dist
            grid_err = math.hypot( (world_x / 4.0) - (gt_v.world_x / 4.0), (world_y / 4.0) - (gt_v.world_y / 4.0) )
            self.node.get_logger().info(f"[Metrics] LOCALIZATION ERROR -> Mean: {err_m:.2f}m, Max bounds (X:{abs(world_x - gt_v.world_x):.2f}m, Y:{abs(world_y - gt_v.world_y):.2f}m), Grid Cell: {grid_err:.2f}")

    def mark_detected(self, v_id):
        if v_id in self.victims and self.victims[v_id].state == VictimState.UNDETECTED:
            self.victims[v_id].state = VictimState.DETECTED
            self.node.get_logger().info(f"[VictimManager] Ground truth {v_id} marked as DETECTED internally!")
            return True
        return False
        
    def mark_detected_by_grid(self, grid_x, grid_y):
        for v_id, victim in self.victims.items():
            if victim.grid_x == grid_x and victim.grid_y == grid_y:
                return self.mark_detected(v_id)
        return False

    def is_valid_destination(self, wx, wy):
        """Checks if a continuous world coordinate is a valid non-obstacle location."""
        gx, gy = GridWorldTransform.world_to_grid(wx, wy)
        gx, gy, is_valid = GridWorldTransform.clamp_grid(gx, gy)
        
        if not is_valid:
            return False
            
        if self.env.grid[gx, gy] == -1: # -1 is obstacle in SARGridEnv
            return False
            
        return True

    def choose_new_destination(self, victim):
        """Chooses a valid random destination within the victim's movement radius."""
        if not victim.profile["movement_enabled"]:
            return False
            
        radius = victim.profile["radius"]
        max_attempts = 10
        
        for _ in range(max_attempts):
            angle = self.rng.uniform(0, 2 * math.pi)
            dist = self.rng.uniform(1.0, radius)
            
            target_wx = victim.world_x + math.cos(angle) * dist
            target_wy = victim.world_y + math.sin(angle) * dist
            
            if self.is_valid_destination(target_wx, target_wy):
                victim.dest_world_x = target_wx
                victim.dest_world_y = target_wy
                return True
                
        return False

    def update(self):
        """Main update loop. Should be called periodically from the mission controller."""
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # Update track statuses
        for t_id, track in self.tracked_victims.items():
            track.check_status(current_time)
        
        for v_id, victim in self.victims.items():
            if victim.state == VictimState.RESCUED:
                continue
                
            if not victim.profile["movement_enabled"]:
                continue
                
            if victim.movement_state == MovementState.RESTING:
                victim.rest_timer -= dt
                if victim.rest_timer <= 0:
                    if self.choose_new_destination(victim):
                        victim.movement_state = MovementState.MOVING
                        victim.speed = self.rng.uniform(victim.profile['min_speed'], victim.profile['max_speed'])
                    else:
                        # Failed to find valid destination, reset timer
                        victim.rest_timer = self.rng.uniform(victim.profile['min_rest_duration'], victim.profile['max_rest_duration'])
                        
            elif victim.movement_state == MovementState.MOVING:
                dx = victim.dest_world_x - victim.world_x
                dy = victim.dest_world_y - victim.world_y
                dist = math.hypot(dx, dy)
                
                if dist < self.waypoint_tolerance:
                    victim.movement_state = MovementState.RESTING
                    victim.rest_timer = self.rng.uniform(victim.profile['min_rest_duration'], victim.profile['max_rest_duration'])
                else:
                    # Move towards destination
                    move_dist = min(victim.speed * dt, dist)
                    victim.world_x += (dx / dist) * move_dist
                    victim.world_y += (dy / dist) * move_dist
                    
                    # Update grid coordinates
                    gx, gy = GridWorldTransform.world_to_grid(victim.world_x, victim.world_y)
                    victim.grid_x, victim.grid_y, _ = GridWorldTransform.clamp_grid(gx, gy)
                    
                    # Update Gazebo model at a throttled rate (1 Hz)
                    if current_time - victim.last_gz_update_time > self.gz_update_interval:
                        self.update_gazebo_pose(victim)
                        victim.last_gz_update_time = current_time

    def update_gazebo_pose(self, victim):
        """Updates the physical Gazebo model position using the set_pose service."""
        cmd = [
            'gz', 'service', '-s', f'/world/{self.world_name}/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--req', f'name: "{victim.id}", position: {{x: {victim.world_x}, y: {victim.world_y}, z: 0.1}}'
        ]
        # Run in background to avoid blocking Python loop
        subprocess.Popen(cmd)

    def get_dashboard_state(self):
        """Returns the serialized state for telemetry output (using camera tracked victims instead of GT)."""
        # The frontend expects a list of victim dicts.
        # We will supply the tracked victims so the dashboard shows visual detections accurately.
        return [t.get_dict() for t in self.tracked_victims.values()]

