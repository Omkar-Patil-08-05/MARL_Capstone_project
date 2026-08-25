import math
import os
import csv
import json
from typing import List, Dict, Any
from rclpy.node import Node
from .drone_agent import DroneAgent, FlightState

def local_to_world(local_x, local_y, world_spawn_x, world_spawn_y, world_yaw):
    world_dx = local_x * math.cos(world_yaw) + local_y * math.sin(world_yaw)
    world_dy = local_x * math.sin(world_yaw) - local_y * math.cos(world_yaw)
    return world_spawn_x + world_dx, world_spawn_y + world_dy

def world_to_local(world_x, world_y, world_spawn_x, world_spawn_y, world_yaw):
    world_dx = world_x - world_spawn_x
    world_dy = world_y - world_spawn_y
    local_x = world_dx * math.cos(world_yaw) + world_dy * math.sin(world_yaw)
    local_y = world_dx * math.sin(world_yaw) - world_dy * math.cos(world_yaw)
    return local_x, local_y

class MissionController:
    def __init__(self, node: Node, agents: List[DroneAgent], control_rate_hz: int = 20):
        self.node = node
        self.agents = agents
        self.control_rate_hz = control_rate_hz
        
        # Test states
        self.goals_assigned = False
        self.mission_complete = False
        self.apf_activations = 0
        self.min_separation = float('inf')
        self.total_separation = 0.0
        self.separation_count = 0
        self.ticks_in_mission = 0
        self.apf_active_ticks = 0
        
        # Setup logging
        self.mode_name = "apf" if self.agents[0].mission.enable_apf else "baseline"
        self.results_dir = os.path.join(os.getcwd(), "results", "phase5d", self.mode_name)
        os.makedirs(self.results_dir, exist_ok=True)
        self.csv_files = {}
        self.csv_writers = {}
        for agent in self.agents:
            f = open(os.path.join(self.results_dir, f"{agent.config.drone_id}.csv"), 'w', newline='')
            writer = csv.writer(f)
            writer.writerow([
                "timestamp", "drone_id", "local_x", "local_y", "local_z",
                "world_x", "world_y", "world_z", 
                "goal_local_x", "goal_local_y", "goal_local_z",
                "goal_world_x", "goal_world_y", "goal_world_z",
                "distance_to_goal", "neighbor_distance", "apf_active", 
                "repulsion_world_x", "repulsion_world_y",
                "avoidance_local_x", "avoidance_local_y"
            ])
            self.csv_files[agent.config.drone_id] = f
            self.csv_writers[agent.config.drone_id] = writer
        
        self.timer = self.node.create_timer(1.0 / self.control_rate_hz, self.tick)
        self.node.get_logger().info(f"MissionController initialized with {len(self.agents)} agents. Mode: {self.mode_name}")

    def _assign_goals(self):
        # Assign goals: Swap positions
        # Drone 0 goes to Drone 1's spawn, and vice versa.
        # This implementation specifically assumes 2 agents for simplicity.
        if len(self.agents) == 2:
            agent0, agent1 = self.agents[0], self.agents[1]
            
            # Agent 0 goal is Agent 1's spawn
            g0_local_x, g0_local_y = world_to_local(
                agent1.config.world_spawn_x, agent1.config.world_spawn_y,
                agent0.config.world_spawn_x, agent0.config.world_spawn_y, agent0.config.world_yaw
            )
            agent0.set_mission_goal_local(g0_local_x, g0_local_y, agent0.initial_pos[2] - agent0.mission.takeoff_altitude)
            
            # Agent 1 goal is Agent 0's spawn
            g1_local_x, g1_local_y = world_to_local(
                agent0.config.world_spawn_x, agent0.config.world_spawn_y,
                agent1.config.world_spawn_x, agent1.config.world_spawn_y, agent1.config.world_yaw
            )
            agent1.set_mission_goal_local(g1_local_x, g1_local_y, agent1.initial_pos[2] - agent1.mission.takeoff_altitude)
            
            self.node.get_logger().info(f"Assigned swap goals.")
        self.goals_assigned = True

    def _write_summary(self):
        summary_path = os.path.join(self.results_dir, "summary.json")
        mean_sep = self.total_separation / self.separation_count if self.separation_count > 0 else 0
        
        final_errors = {}
        for agent in self.agents:
            lx, ly = agent.px4.current_position[0], agent.px4.current_position[1]
            gx, gy = agent.mission_goal_local[0], agent.mission_goal_local[1]
            err = math.sqrt((lx-gx)**2 + (ly-gy)**2)
            final_errors[agent.config.drone_id] = err
            
        data = {
            "mode": self.mode_name,
            "min_separation": self.min_separation,
            "mean_separation": mean_sep,
            "mission_duration_sec": self.ticks_in_mission / self.control_rate_hz,
            "apf_activation_duration_sec": self.apf_active_ticks / self.control_rate_hz,
            "final_waypoint_errors": final_errors,
            "emergency_triggered": self.min_separation <= self.agents[0].mission.emergency_distance
        }
        with open(summary_path, 'w') as f:
            json.dump(data, f, indent=4)
        self.node.get_logger().info(f"Wrote summary: {data}")

    def tick(self) -> None:
        # Check if we should assign goals
        if not self.goals_assigned:
            all_ready = all(a.initial_pos is not None for a in self.agents)
            if all_ready:
                self._assign_goals()
            
        if self.all_complete() and not self.mission_complete:
            self.mission_complete = True
            for f in self.csv_files.values():
                f.close()
            self._write_summary()

        navigating = all(a.get_state() == FlightState.WAYPOINT_NAVIGATION for a in self.agents)
        if navigating:
            self.ticks_in_mission += 1

        # APF and telemetry logging
        if navigating and len(self.agents) == 2:
            a0, a1 = self.agents[0], self.agents[1]
            mission = a0.mission
            
            w0_x, w0_y = local_to_world(a0.px4.current_position[0], a0.px4.current_position[1], a0.config.world_spawn_x, a0.config.world_spawn_y, a0.config.world_yaw)
            w1_x, w1_y = local_to_world(a1.px4.current_position[0], a1.px4.current_position[1], a1.config.world_spawn_x, a1.config.world_spawn_y, a1.config.world_yaw)
            
            dx = w0_x - w1_x
            dy = w0_y - w1_y
            dist = math.sqrt(dx*dx + dy*dy)
            
            self.min_separation = min(self.min_separation, dist)
            self.total_separation += dist
            self.separation_count += 1
            
            # Emergency guard
            if dist <= mission.emergency_distance:
                self.node.get_logger().warn(f"EMERGENCY GUARD TRIGGERED! dist={dist:.2f} <= {mission.emergency_distance}. Switching to HOLD.")
                a0._log_transition(FlightState.HOLD)
                a1._log_transition(FlightState.HOLD)
                
            apf_active = False
            r0_world_x, r0_world_y = 0.0, 0.0
            r1_world_x, r1_world_y = 0.0, 0.0
            
            if mission.enable_apf and dist > 1e-3 and dist < mission.safe_distance and dist > mission.emergency_distance:
                apf_active = True
                self.apf_active_ticks += 1
                
                # Magnitude
                M = mission.repulsive_gain * (1.0/dist - 1.0/mission.safe_distance)
                
                # Direction for a0 (away from a1)
                u0_x, u0_y = dx/dist, dy/dist
                r0_world_x = M * u0_x
                r0_world_y = M * u0_y
                
                # Clamp a0
                mag0 = math.sqrt(r0_world_x**2 + r0_world_y**2)
                if mag0 > mission.max_repulsion:
                    r0_world_x = r0_world_x / mag0 * mission.max_repulsion
                    r0_world_y = r0_world_y / mag0 * mission.max_repulsion
                    
                # Direction for a1 (away from a0)
                u1_x, u1_y = -dx/dist, -dy/dist
                r1_world_x = M * u1_x
                r1_world_y = M * u1_y
                
                # Clamp a1
                mag1 = math.sqrt(r1_world_x**2 + r1_world_y**2)
                if mag1 > mission.max_repulsion:
                    r1_world_x = r1_world_x / mag1 * mission.max_repulsion
                    r1_world_y = r1_world_y / mag1 * mission.max_repulsion
                    
            # Transform world repulsion to local avoidance offset
            # We treat the repulsion vector as a relative displacement. 
            # To transform a world displacement to local, we use world_to_local with spawn=(0,0).
            a0_avoid_x, a0_avoid_y = world_to_local(r0_world_x, r0_world_y, 0, 0, a0.config.world_yaw)
            a1_avoid_x, a1_avoid_y = world_to_local(r1_world_x, r1_world_y, 0, 0, a1.config.world_yaw)
            
            a0.set_avoidance_offset_local(a0_avoid_x, a0_avoid_y)
            a1.set_avoidance_offset_local(a1_avoid_x, a1_avoid_y)
            
            # Log telemetry
            t = self.node.get_clock().now().nanoseconds / 1e9
            for i, (a, wx, wy, rx, ry, ax, ay) in enumerate([(a0, w0_x, w0_y, r0_world_x, r0_world_y, a0_avoid_x, a0_avoid_y),
                                                             (a1, w1_x, w1_y, r1_world_x, r1_world_y, a1_avoid_x, a1_avoid_y)]):
                gx_world, gy_world = local_to_world(a.mission_goal_local[0], a.mission_goal_local[1], a.config.world_spawn_x, a.config.world_spawn_y, a.config.world_yaw)
                dx_g = wx - gx_world
                dy_g = wy - gy_world
                dist_g = math.sqrt(dx_g*dx_g + dy_g*dy_g)
                
                self.csv_writers[a.config.drone_id].writerow([
                    t, a.config.drone_id, a.px4.current_position[0], a.px4.current_position[1], a.px4.current_position[2],
                    wx, wy, a.px4.current_position[2], # using local z for world z
                    a.mission_goal_local[0], a.mission_goal_local[1], a.mission_goal_local[2],
                    gx_world, gy_world, a.mission_goal_local[2],
                    dist_g, dist, apf_active,
                    rx, ry, ax, ay
                ])

        for agent in self.agents:
            agent.tick()
            
    def all_complete(self) -> bool:
        return all(agent.get_state() == FlightState.COMPLETE for agent in self.agents)
        
    def any_failure(self) -> bool:
        return any(agent.get_state() == FlightState.FAILSAFE for agent in self.agents)
