#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import os
import csv
import json

from swarm_controller.drone_agent import DroneAgent, FlightState
from swarm_controller.configs import DroneConfig, MissionConfig

class TwoDroneDeterministicTest(Node):
    def __init__(self, mission_config, drone_configs):
        super().__init__('two_drone_deterministic_test')
        self.mission_config = mission_config
        
        self.agents = [DroneAgent(self, config, mission_config) for config in drone_configs]
        
        self.mission_active = False
        self.mission_completed = False
        self.phase = 0
        
        self.results_dir = os.path.join(os.getcwd(), "results", "phase5e8", "val")
        os.makedirs(self.results_dir, exist_ok=True)
        self.log_file = open(os.path.join(self.results_dir, "deterministic_val.csv"), "w", newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow([
            "timestamp", "drone_id", "phase", 
            "world_x", "world_y", "world_z", 
            "local_x", "local_y", "local_z",
            "state"
        ])
        
        self.timer = self.create_timer(1.0 / self.mission_config.control_rate_hz, self.tick)
        self.get_logger().info("Deterministic Validation Node initialized.")

    def tick(self):
        for agent in self.agents:
            agent.tick()
            
        if self.mission_completed:
            return
            
        if not self.mission_active:
            if all(a.state == FlightState.HOLD for a in self.agents):
                self.get_logger().info("Both drones airborne in HOLD. Starting deterministic Phase 1.")
                self.mission_active = True
                self.phase = 1
                
                # Drone 0: +8m X
                d0 = self.agents[0]
                d0.set_mission_goal_local(8.0, 0.0, d0.initial_pos[2] - self.mission_config.takeoff_altitude)
                d0._log_transition(FlightState.WAYPOINT_NAVIGATION)
                
                # Drone 1: +8m Y
                d1 = self.agents[1]
                d1.set_mission_goal_local(0.0, 8.0, d1.initial_pos[2] - self.mission_config.takeoff_altitude)
                d1._log_transition(FlightState.WAYPOINT_NAVIGATION)
            return

        # Phase 1: Wait for both to reach target and hold
        if self.phase == 1:
            if all(a.state == FlightState.HOLD for a in self.agents):
                self.get_logger().info("Both drones reached target. Returning home.")
                self.phase = 2
                for agent in self.agents:
                    agent.set_mission_goal_local(0.0, 0.0, agent.initial_pos[2] - self.mission_config.takeoff_altitude)
                    agent._log_transition(FlightState.WAYPOINT_NAVIGATION)

        # Phase 2: Wait for both to return and land
        elif self.phase == 2:
            if all(a.state == FlightState.COMPLETE for a in self.agents):
                self.get_logger().info("Mission Complete. Both drones landed and disarmed.")
                self.mission_completed = True
                self.log_file.close()

        # Logging
        t = self.get_clock().now().nanoseconds / 1e9
        for agent in self.agents:
            if agent.initial_pos:
                lx, ly, lz = agent.px4.current_position
                wx = lx + agent.config.world_spawn_x
                wy = ly + agent.config.world_spawn_y
                wz = lz # Z is usually absolute in Gazebo but PX4 odometry is down-positive relative. We just log raw.
                
                self.csv_writer.writerow([
                    t, agent.config.drone_id, self.phase,
                    wx, wy, wz,
                    lx, ly, lz,
                    agent.state.name
                ])

    def destroy_node(self):
        if not self.log_file.closed:
            self.log_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    mission_config = MissionConfig(
        enable_apf=False, 
        control_rate_hz=20, 
        hold_duration=40, # 2 seconds
        goal_tolerance=0.75
    )
    
    # Read spawn coordinates from metadata
    meta_path = os.path.expanduser("~/capstone_project_antigravity/worlds/generated_world_meta.json")
    with open(meta_path, "r") as f:
        meta = json.load(f)
    
    d0_spawn = meta["drone_base"]["spawns"][0]
    d1_spawn = meta["drone_base"]["spawns"][1]
    
    config0 = DroneConfig(
        drone_id="drone_0", namespace="drone_0", system_id=1,
        world_spawn_x=float(d0_spawn["x"]), world_spawn_y=float(d0_spawn["y"]), world_spawn_z=0.2, world_yaw=0.0
    )
    config1 = DroneConfig(
        drone_id="drone_1", namespace="drone_1", system_id=2,
        world_spawn_x=float(d1_spawn["x"]), world_spawn_y=float(d1_spawn["y"]), world_spawn_z=0.2, world_yaw=0.0
    )
    
    node = TwoDroneDeterministicTest(mission_config, [config0, config1])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
