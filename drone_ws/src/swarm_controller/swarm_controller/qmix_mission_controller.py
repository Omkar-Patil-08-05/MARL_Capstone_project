import os
import csv
import time
import rclpy
from rclpy.node import Node
import sys

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
        
        # QMIX inference bridge
        self.qmix = QMIXAdapter(checkpoint_path, num_agents=len(self.agents))
        
        # Logging
        self.results_dir = os.path.join(os.getcwd(), "results", "phase5e8", "qmix")
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
        
        # State tracking
        self.decisions_made = 0
        self.mission_active = False
        self.land_commanded = False
        
        # Create control loop timer
        self.timer = self.create_timer(1.0 / self.mission_config.control_rate_hz, self.tick)
        self.get_logger().info("QMIX Mission Controller initialized.")

    def _initialize_grid_obstacles(self):
        """Populates the grid with static buildings from small_sar.sdf to match training obs."""
        buildings = [
            (10, 50, 60, 80),
            (60, 80, 10, 50),
            (62.5, 77.5, 62.5, 77.5),
            (20, 40, 12.5, 27.5)
        ]
        
        victims_world = [
            (30, 40), (80, 20), (20, 80), (60, 60), (85, 85)
        ]
        
        for gx in range(self.env.x_size):
            for gy in range(self.env.y_size):
                wx, wy = GridWorldTransform.grid_to_world_center(gx, gy)
                for (xmin, xmax, ymin, ymax) in buildings:
                    if xmin <= wx <= xmax and ymin <= wy <= ymax:
                        self.env.grid[gx, gy] = -1
                        break
        
        self.env.victims = {}
        for (vx, vy) in victims_world:
            gx, gy = GridWorldTransform.world_to_grid(vx, vy)
            self.env.victims[(gx, gy)] = 0

    def tick(self):
        for agent in self.agents:
            agent.tick()
            
        if not self.mission_active:
            if all(a.state == FlightState.HOLD for a in self.agents):
                self.get_logger().info("All drones airborne. Starting QMIX inference.")
                self.mission_active = True
                
                self.sync_env_state()
                self.env._update_fov_and_victims()
                
                for agent in self.agents:
                    self.plan_and_execute_action(agent)
            return

        if self.decisions_made >= self.max_decisions:
            if not self.land_commanded:
                self.get_logger().info("Max decisions reached. Commanding landing.")
                self.land_commanded = True
                for agent in self.agents:
                    agent.px4.publish_vehicle_command(21)
                    agent._log_transition(FlightState.LAND)
            return
            
        for agent in self.agents:
            if agent.state == FlightState.HOLD:
                self.plan_and_execute_action(agent)
            elif agent.state == FlightState.WAYPOINT_NAVIGATION:
                if agent.state_timer > self.mission_config.hold_duration:
                    self.get_logger().warn(f"[{agent.config.drone_id}] Action timed out! Replanning.")
                    self.plan_and_execute_action(agent)

    def sync_env_state(self):
        """Synchronizes continuous telemetry with the discrete SARGridEnv internal state."""
        for i, agent in enumerate(self.agents):
            lx, ly, _ = agent.px4.current_position
            # Fix: Add spawn offset to local odometry to get world coordinates
            wx = lx + agent.config.world_spawn_x
            wy = ly + agent.config.world_spawn_y
            
            gx, gy = GridWorldTransform.world_to_grid(wx, wy)
            
            gx = max(0, min(gx, self.env.x_size - 1))
            gy = max(0, min(gy, self.env.y_size - 1))
            
            self.env.drone_positions[i] = [gx, gy]

    def plan_and_execute_action(self, agent: DroneAgent):
        """Invokes the neural network, converts the action, and issues the waypoint."""
        self.sync_env_state()
        self.env._update_fov_and_victims()
        
        idx = self.agents.index(agent)
        
        t_start = time.time()
        obs = self.env.get_agent_state(idx)
        action = self.qmix.select_action(idx, obs)
        inference_latency_ms = (time.time() - t_start) * 1000.0
        
        self.decisions_made += 1
        
        action_names = {0: "+X", 1: "-X", 2: "+Y", 3: "-Y", 4: "Hover"}
        
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
            
        # Execute Command
        target_wx, target_wy = GridWorldTransform.grid_to_world_center(safe_gx, safe_gy)
        
        # Fix: DroneAgent local setpoints are relative to the physical spawn.
        lx = target_wx - agent.config.world_spawn_x
        ly = target_wy - agent.config.world_spawn_y
        lz = agent.initial_pos[2] - agent.mission.takeoff_altitude
        
        agent.set_mission_goal_local(lx, ly, lz)
        agent._log_transition(FlightState.WAYPOINT_NAVIGATION)
        
        self.log_decision(agent, curr_gx, curr_gy, action, action_names[action], target_gx, target_gy, target_wx, target_wy, inference_latency_ms, safety_override)
        self.get_logger().info(f"[{agent.config.drone_id}] Action: {action_names[action]} -> Grid ({safe_gx},{safe_gy}) -> World ({target_wx:.1f}, {target_wy:.1f})")

    def log_decision(self, agent, gx, gy, action, action_name, tgt_gx, tgt_gy, tgt_wx, tgt_wy, inf_time, safety):
        lx, ly, wz = agent.px4.current_position
        curr_wx = lx + agent.config.world_spawn_x
        curr_wy = ly + agent.config.world_spawn_y
        
        valid_cells = (self.env.grid != -1).sum()
        explored = (self.env.grid == 1).sum()
        coverage = explored / valid_cells
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

    def destroy_node(self):
        self.log_file.close()
        super().destroy_node()
