import os
import csv
import time
import rclpy
from rclpy.node import Node
import sys
from std_msgs.msg import String
import json

from swarm_controller.drone_agent import DroneAgent, FlightState
from swarm_controller.grid_world_transform import GridWorldTransform
from swarm_controller.qmix_ros2_adapter import QMIXAdapter

sys.path.append('/home/capstone/capstone_project_antigravity')
from marl_drone_project.env.sar_env import SARGridEnv

class QMIXMissionController(Node):
    def __init__(self, mission_config, drone_configs, checkpoint_path, max_decisions=30):
        super().__init__('qmix_mission_controller')
        self.mission_config = mission_config
        self.drone_configs = drone_configs
        self.max_decisions = max_decisions
        
        # Instantiate drones
        self.agents = [DroneAgent(self, config, mission_config) for config in drone_configs]
        
        # Initialize internal SARGridEnv to track deployment grid state and observations
        self.env = SARGridEnv(num_drones=len(self.agents), max_steps=1000)
        self.env.reset()
        
        # Override initial grid with small_sar.sdf buildings
        self._initialize_grid_obstacles()
        
        self.unspawned_victims = list(self.env.victims.keys())
        
        # QMIX inference bridge
        self.qmix = QMIXAdapter(checkpoint_path, num_agents=len(self.agents))
        
        # Logging
        self.results_dir = os.path.join(os.getcwd(), "results", "h8_v4_physical_final")
        os.makedirs(self.results_dir, exist_ok=True)
        self.log_file = open(os.path.join(self.results_dir, "qmix_decisions.csv"), "w", newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow([
            "timestamp", "episode_step", "drone_id", 
            "grid_x", "grid_y", "current_world_x", "current_world_y", "world_z", 
            "action", "action_name", 
            "target_grid_x", "target_grid_y", 
            "target_world_x", "target_world_y", 
            "distance_to_target", "coverage", "victims_detected", "inference_latency_ms",
            "safety_override"
        ])
        
        self.traj0_file = open(os.path.join(self.results_dir, "trajectory_drone_0.csv"), "w", newline='')
        self.traj0_writer = csv.writer(self.traj0_file)
        self.traj0_writer.writerow(["timestamp", "world_x", "world_y", "world_z", "grid_x", "grid_y"])
        
        self.traj1_file = open(os.path.join(self.results_dir, "trajectory_drone_1.csv"), "w", newline='')
        self.traj1_writer = csv.writer(self.traj1_file)
        self.traj1_writer.writerow(["timestamp", "world_x", "world_y", "world_z", "grid_x", "grid_y"])
        
        self.victim_file = open(os.path.join(self.results_dir, "victim_detection.csv"), "w", newline='')
        self.victim_writer = csv.writer(self.victim_file)
        self.victim_writer.writerow(["timestamp", "decision_step", "victim_grid_x", "victim_grid_y", "detected_by"])
        
        # State tracking
        self.decisions_made = 0
        self.mission_active = False
        self.land_commanded = False
        self.detected_victims = set()
        self.last_decision_time = time.time()
        self.last_heartbeat_time = time.time()
        self.natural_hover_count = 0
        self.safety_forced_hover_count = 0
        self.safety_overrides_count = 0
        self.start_time = time.time()
        self.global_step_state = "INITIALIZING" 
        
        # Create control loop timer
        self.timer = self.create_timer(1.0 / self.mission_config.control_rate_hz, self.tick)
        
        # Telemetry
        self.telemetry_pub = self.create_publisher(String, '/swarm/telemetry', 10)
        self.telemetry_tick_counter = 0
        self.last_actions = {a.config.drone_id: "None" for a in self.agents}
        self.last_safety_overrides = {a.config.drone_id: False for a in self.agents}
        
        self.get_logger().info("QMIX Mission Controller initialized.")

    def _initialize_grid_obstacles(self):
        """Populates the grid with static buildings from generated_world_meta.json."""
        import json
        
        # Robust project-relative resolution
        project_root = os.path.expanduser('~/capstone_project_antigravity')
        meta_path = os.path.join(project_root, 'worlds', 'generated_world_meta.json')
        
        if not os.path.exists(meta_path):
            raise RuntimeError(f"CRITICAL: Metadata file not found at {meta_path}. Run generate_world.py first.")
            
        try:
            with open(meta_path, 'r') as f:
                meta = json.load(f)
        except json.JSONDecodeError:
            raise RuntimeError(f"CRITICAL: Metadata file {meta_path} is malformed.")
            
        obstacles = meta.get("obstacles", [])
        victims_meta = meta.get("victims", [])
        drone_spawns = meta.get("drone_base", {}).get("spawns", [])
        
        METERS_PER_CELL = GridWorldTransform.METERS_PER_CELL
        
        # 1. Clear training obstacles and project geometric AABBs
        self.env.grid.fill(0)
        for gx in range(self.env.x_size):
            for gy in range(self.env.y_size):
                cell_min_x = gx * METERS_PER_CELL
                cell_max_x = (gx + 1) * METERS_PER_CELL
                cell_min_y = gy * METERS_PER_CELL
                cell_max_y = (gy + 1) * METERS_PER_CELL
                
                for obs in obstacles:
                    aabb = obs["aabb"]
                    intersect_x = (aabb["max_x"] > cell_min_x) and (aabb["min_x"] < cell_max_x)
                    intersect_y = (aabb["max_y"] > cell_min_y) and (aabb["min_y"] < cell_max_y)
                    
                    if intersect_x and intersect_y:
                        self.env.grid[gx, gy] = -1
                        break
        
        # 2. Populate and strictly validate victims
        self.env.victims = {}
        for v in victims_meta:
            gx = v["grid"]["x"]
            gy = v["grid"]["y"]
            
            if not (0 <= gx < self.env.x_size and 0 <= gy < self.env.y_size):
                raise RuntimeError(f"CRITICAL: Victim {v.get('id')} is out of grid bounds at ({gx}, {gy})!")
                
            if self.env.grid[gx, gy] == -1:
                raise RuntimeError(f"CRITICAL: Victim {v.get('id')} occupies an obstacle cell at ({gx}, {gy})!")
                
            self.env.victims[(gx, gy)] = 0
            
        # 3. Synchronize drone spawn metadata
        for agent in self.agents:
            drone_id_str = str(agent.config.drone_id)
            spawn_meta = next((s for s in drone_spawns if drone_id_str in str(s.get("id", ""))), None)
            if spawn_meta:
                agent.config.world_spawn_x = spawn_meta["x"]
                agent.config.world_spawn_y = spawn_meta["y"]
                self.get_logger().info(f"Dynamically loaded spawn for Drone {drone_id_str}: ({spawn_meta['x']}, {spawn_meta['y']})")
            else:
                self.get_logger().warn(f"No drone_base spawn metadata found for Drone {drone_id_str}. Relying on launch config.")

    def tick(self):
        current_time = time.time()
        if self.mission_active and not self.land_commanded:
            if current_time - self.last_heartbeat_time >= 60.0:
                self.last_heartbeat_time = current_time
                elapsed = current_time - self.start_time
                valid_cells = (self.env.grid != -1).sum()
                explored = (self.env.grid == 1).sum()
                coverage = (explored / valid_cells) * 100 if valid_cells > 0 else 0
                rate = self.decisions_made / elapsed if elapsed > 0 else 0.0
                state_str = ", ".join([f"{a.config.drone_id}: {a.state.name}" for a in self.agents])
                self.get_logger().info(
                    f"\n==================================================\n"
                    f"Elapsed: {elapsed:.1f}s | Decisions: {self.decisions_made} / {self.max_decisions} ({rate:.2f} dec/s)\n"
                    f"Coverage: {coverage:.2f}% | Victims: {len(self.detected_victims)}/5\n"
                    f"Hover (Nat): {self.natural_hover_count} | Hover (Safe): {self.safety_forced_hover_count} | Overrides: {self.safety_overrides_count}\n"
                    f"States: {state_str}\n"
                    f"Last decision: {self.last_decision_time:.2f} | Time since prev: {current_time - self.last_decision_time:.2f}s\n"
                    f"=================================================="
                )
            
            if current_time - self.last_decision_time >= 120.0:
                self.get_logger().error("H6 WARNING: NO DECISION PROGRESS FOR 120 SECONDS")
                self.last_decision_time = current_time
                
        for agent in self.agents:
            agent.tick()
            
        self.telemetry_tick_counter += 1
        if self.telemetry_tick_counter % 2 == 0:
            self._publish_telemetry(current_time)
            
        if not self.mission_active:
            if all(a.state == FlightState.HOLD for a in self.agents):
                self.get_logger().info("All drones airborne. Starting QMIX inference.")
                self.mission_active = True
                self.global_step_state = "EXECUTING"
                self.execute_global_step()
            return

        if self.decisions_made >= self.max_decisions:
            if not self.land_commanded:
                self.get_logger().info("Max decisions reached. Commanding landing.")
                self.land_commanded = True
                self.global_step_state = "LANDING"
                for agent in self.agents:
                    agent.px4.publish_vehicle_command(21)
                    agent._log_transition(FlightState.LAND)
            return
            
        # Synchronization block
        all_ready = True
        for agent in self.agents:
            if agent.state == FlightState.WAYPOINT_NAVIGATION:
                if agent.state_timer > self.mission_config.waypoint_timeout:
                    self.get_logger().warn(f"[{agent.config.drone_id}] Action timed out! Forcing HOLD for resync.")
                    agent._log_transition(FlightState.HOLD)
            if agent.state != FlightState.HOLD:
                all_ready = False
                
        if all_ready:
            self.global_step_state = "READY_FOR_NEXT_STEP"
            self.execute_global_step()
            self.global_step_state = "EXECUTING"
        else:
            self.global_step_state = "WAITING_FOR_AGENTS"

    def _publish_telemetry(self, current_time):
        try:
            mission_status = "INITIALIZING"
            if self.mission_active:
                mission_status = "RUNNING"
            if self.land_commanded:
                mission_status = "LANDING"
            if all(a.state == FlightState.COMPLETE for a in self.agents):
                mission_status = "COMPLETE"
                
            elapsed = current_time - self.start_time
            valid_cells = (self.env.grid != -1).sum()
            explored = (self.env.grid == 1).sum()
            coverage = (explored / valid_cells) * 100 if valid_cells > 0 else 0
            
            drones_data = []
            for i, agent in enumerate(self.agents):
                lx, ly, lz = agent.px4.current_position
                wx = lx + agent.config.world_spawn_x
                wy = ly + agent.config.world_spawn_y
                wz = lz
                gx, gy = GridWorldTransform.world_to_grid(wx, wy)
                
                drone_id = agent.config.drone_id
                
                drones_data.append({
                    "id": str(drone_id),
                    "state": agent.state.name,
                    "x": round(float(wx), 2),
                    "y": round(float(wy), 2),
                    "z": round(-float(wz), 2),
                    "grid_x": int(gx),
                    "grid_y": int(gy),
                    "action": self.last_actions.get(drone_id, "None"),
                    "safety_override": self.last_safety_overrides.get(drone_id, False)
                })
                
            # Build explored cells list from authoritative SARGridEnv
            explored_cells = []
            for gx in range(self.env.x_size):
                for gy in range(self.env.y_size):
                    if self.env.grid[gx, gy] == 1:
                        explored_cells.append({"x": int(gx), "y": int(gy)})
            
            # Build victim state list
            victims_state = []
            for idx, ((vx, vy), status) in enumerate(self.env.victims.items()):
                victims_state.append({
                    "id": f"V{idx}",
                    "x": int(vx),
                    "y": int(vy),
                    "detected": bool(status == 1)
                })
            
            msg_dict = {
                "type": "telemetry",
                "timestamp": current_time,
                "mission": {
                    "status": mission_status,
                    "decision_count": self.decisions_made,
                    "max_decisions": self.max_decisions,
                    "global_step_state": getattr(self, 'global_step_state', "UNKNOWN"),
                    "coverage": round(float(coverage), 2),
                    "victims_detected": len(self.detected_victims),
                    "total_victims": len(self.env.victims),
                    "explored_count": int(explored),
                    "valid_count": int(valid_cells),
                    "safety_overrides": self.safety_overrides_count
                },
                "drones": drones_data,
                "explored_cells": explored_cells,
                "victims": victims_state
            }
            
            msg = String()
            msg.data = json.dumps(msg_dict)
            self.telemetry_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Telemetry error: {e}")

    def spawn_victim(self, gx, gy):
        wx, wy = GridWorldTransform.grid_to_world_center(gx, gy)
        self.get_logger().warn(f"DYNAMIC VICTIM MANAGER: Spawning victim at Grid ({gx}, {gy}) -> World ({wx:.1f}, {wy:.1f})")
        cmd = (
            f'gz service -s /world/realistic_sar/create '
            f'--reqtype gz.msgs.EntityFactory '
            f'--reptype gz.msgs.Boolean '
            f'--req "sdf_filename: \"/home/capstone/capstone_project_antigravity/assets_real/victims/victim_1/model.sdf\", '
            f'pose: {{position: {{x: {wx}, y: {wy}, z: 0.0}}}}"'
        )
        import subprocess
        subprocess.Popen(cmd, shell=True)

    def sync_env_state(self):
        """Synchronizes continuous telemetry with the discrete SARGridEnv internal state."""
        for i, agent in enumerate(self.agents):
            lx, ly, _ = agent.px4.current_position
            wx = lx + agent.config.world_spawn_x
            wy = ly + agent.config.world_spawn_y
            
            gx, gy = GridWorldTransform.world_to_grid(wx, wy)
            
            gx = max(0, min(gx, self.env.x_size - 1))
            gy = max(0, min(gy, self.env.y_size - 1))
            
            self.env.drone_positions[i] = [gx, gy]

    def execute_global_step(self):
        """Executes a fully synchronous RL step for all active agents simultaneously."""
        self.sync_env_state()
        new_cells, _ = self.env._update_fov_and_victims()
        
        # PHASE 1: BFS SEMANTIC FIX
        if sum(new_cells) > 0:
            self.env._update_global_bfs()
            
        # Dynamic Victim Manager
        valid_cells = (self.env.grid != -1).sum()
        explored = (self.env.grid == 1).sum()
        coverage = (explored / valid_cells) * 100 if valid_cells > 0 else 0
        if coverage > (5 - len(self.unspawned_victims)) * 20:
            if self.unspawned_victims:
                vgx, vgy = self.unspawned_victims.pop(0)
                self.spawn_victim(vgx, vgy)
                
        # 1. Collect all observations synchronously
        t_start = time.time()
        actions = []
        latencies = []
        for idx, agent in enumerate(self.agents):
            obs = self.env.get_agent_state(idx)
            action = self.qmix.select_action(idx, obs)
            actions.append(action)
            latencies.append((time.time() - t_start) * 1000.0)
            
        self.decisions_made += 1
        self.last_decision_time = time.time()
        
        action_names = {0: "+X", 1: "-X", 2: "+Y", 3: "-Y", 4: "Hover"}
        
        # 2. Dispatch all actions
        for idx, agent in enumerate(self.agents):
            action = actions[idx]
            curr_gx, curr_gy = self.env.drone_positions[idx]
            target_gx, target_gy = curr_gx, curr_gy
            
            if action == 0: target_gx += 1
            elif action == 1: target_gx -= 1
            elif action == 2: target_gy += 1
            elif action == 3: target_gy -= 1
            
            safety_override = False
            safe_gx, safe_gy, is_valid = GridWorldTransform.clamp_grid(target_gx, target_gy)
            if not is_valid:
                self.get_logger().warn(f"[{agent.config.drone_id}] Boundary violation prevented!")
                safety_override = True
                
            if self.env.grid[safe_gx, safe_gy] == -1:
                self.get_logger().warn(f"[{agent.config.drone_id}] Obstacle collision prevented! Falling back to hover.")
                safe_gx, safe_gy = curr_gx, curr_gy
                action = 4
                safety_override = True
                
            if safety_override:
                self.safety_overrides_count += 1
                if action == 4:
                    self.safety_forced_hover_count += 1
            else:
                if action == 4:
                    self.natural_hover_count += 1
                    
            target_wx, target_wy = GridWorldTransform.grid_to_world_center(safe_gx, safe_gy)
            lx = target_wx - agent.config.world_spawn_x
            ly = target_wy - agent.config.world_spawn_y
            lz = agent.initial_pos[2] - agent.mission.takeoff_altitude
            
            is_hover = (action == 4)
            agent.set_mission_goal_local(lx, ly, lz, require_min_dwell=is_hover)
            agent.state_timer = 0
            agent._log_transition(FlightState.WAYPOINT_NAVIGATION)
            
            self.last_actions[agent.config.drone_id] = action_names[action]
            self.last_safety_overrides[agent.config.drone_id] = safety_override
            
            self.log_decision(agent, curr_gx, curr_gy, action, action_names[action], target_gx, target_gy, target_wx, target_wy, latencies[idx], safety_override)
            self.get_logger().info(f"[{agent.config.drone_id}] Action: {action_names[action]} -> Grid ({safe_gx},{safe_gy}) -> World ({target_wx:.1f}, {target_wy:.1f})")

    def log_decision(self, agent, gx, gy, action, action_name, tgt_gx, tgt_gy, tgt_wx, tgt_wy, inf_time, safety):
        lx, ly, wz = agent.px4.current_position
        curr_wx = lx + agent.config.world_spawn_x
        curr_wy = ly + agent.config.world_spawn_y
        
        valid_cells = (self.env.grid != -1).sum()
        explored = (self.env.grid == 1).sum()
        coverage = explored / valid_cells if valid_cells > 0 else 0
        victims = sum(self.env.victims.values())
        
        dx = curr_wx - tgt_wx
        dy = curr_wy - tgt_wy
        dist = (dx**2 + dy**2)**0.5
        
        t = self.get_clock().now().nanoseconds / 1e9
        
        self.csv_writer.writerow([
            t, self.decisions_made, agent.config.drone_id,
            gx, gy, curr_wx, curr_wy, wz,
            action, action_name,
            tgt_gx, tgt_gy, tgt_wx, tgt_wy,
            dist, coverage, victims, inf_time, safety
        ])
        self.log_file.flush()

        # Log Trajectories
        drone_id_str = str(agent.config.drone_id)
        if "drone_0" in drone_id_str:
            self.traj0_writer.writerow([t, curr_wx, curr_wy, wz, gx, gy])
            self.traj0_file.flush()
        elif "drone_1" in drone_id_str:
            self.traj1_writer.writerow([t, curr_wx, curr_wy, wz, gx, gy])
            self.traj1_file.flush()
            
        # Check newly detected victims
        for v_pos, status in self.env.victims.items():
            if status == 1 and v_pos not in self.detected_victims:
                self.detected_victims.add(v_pos)
                self.victim_writer.writerow([t, self.decisions_made, v_pos[0], v_pos[1], drone_id_str])
                self.victim_file.flush()

    def destroy_node(self):
        self.log_file.close()
        self.traj0_file.close()
        self.traj1_file.close()
        self.victim_file.close()
        super().destroy_node()
