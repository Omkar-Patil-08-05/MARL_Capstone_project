#!/usr/import/env python3
import rclpy
from rclpy.node import Node
import math
import os
import csv
from swarm_controller.configs import DroneConfig, MissionConfig
from swarm_controller.drone_agent import DroneAgent, FlightState

class SmokeTestNode(Node):
    def __init__(self):
        super().__init__('smoke_test_node')
        self.get_logger().info("Starting Smoke Test Node")
        
        mission_config = MissionConfig(enable_apf=False, control_rate_hz=20, hold_duration=400) # Longer hold so we can interrupt it
        
        c0 = DroneConfig(drone_id="drone_0", namespace="drone_0", system_id=1, world_spawn_x=12.0, world_spawn_y=12.0, world_spawn_z=0.5, world_yaw=0.0)
        c1 = DroneConfig(drone_id="drone_1", namespace="drone_1", system_id=2, world_spawn_x=12.0, world_spawn_y=20.0, world_spawn_z=0.5, world_yaw=0.0)
        
        self.a0 = DroneAgent(self, c0, mission_config)
        self.a1 = DroneAgent(self, c1, mission_config)
        
        self.agents = [self.a0, self.a1]
        
        self.timer = self.create_timer(1.0 / mission_config.control_rate_hz, self.tick)
        self.phase = 0
        self.phase_timer = 0
        
        os.makedirs("results", exist_ok=True)
        self.log_file = open("results/phase_5e_8c_smoke_test.csv", "w", newline="")
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow(["time", "drone_id", "state", "cmd_x", "cmd_y", "cmd_z", "meas_x", "meas_y", "meas_z"])
        
    def tick(self):
        for a in self.agents:
            a.tick()
            
            t = self.get_clock().now().nanoseconds / 1e9
            self.csv_writer.writerow([
                t, a.config.drone_id, a.get_state().name,
                a.mission_goal_local[0], a.mission_goal_local[1], a.mission_goal_local[2],
                a.px4.current_position[0], a.px4.current_position[1], a.px4.current_position[2]
            ])
            
        navigating = all(a.get_state() in [FlightState.WAYPOINT_NAVIGATION, FlightState.HOLD] for a in self.agents)
        
        if navigating:
            self.phase_timer += 1
            
            if self.phase == 0:
                # Assign initial waypoints (Local frame: X is forward, spawn is 0,0)
                # D0 world 20, 12 -> local +8, 0
                self.a0.set_mission_goal_local(8.0, 0.0, self.a0.initial_pos[2] - 5.0)
                # D1 world 20, 20 -> local +8, 0
                self.a1.set_mission_goal_local(8.0, 0.0, self.a1.initial_pos[2] - 5.0)
                
                # Make sure they are in WP state
                for a in self.agents:
                    a.state = FlightState.WAYPOINT_NAVIGATION
                    a.state_timer = 0
                self.phase = 1
                self.get_logger().info("PHASE 1: Flying to test waypoints")
                
            elif self.phase == 1 and self.phase_timer > 300: # 15 seconds
                # Assign return waypoints
                self.a0.set_mission_goal_local(0.0, 0.0, self.a0.initial_pos[2] - 5.0)
                self.a1.set_mission_goal_local(0.0, 0.0, self.a1.initial_pos[2] - 5.0)
                
                for a in self.agents:
                    a.state = FlightState.WAYPOINT_NAVIGATION
                    a.state_timer = 0
                    
                self.phase = 2
                self.get_logger().info("PHASE 2: Returning to spawn")
                
            elif self.phase == 2 and self.phase_timer > 600: # 30 seconds total
                self.get_logger().info("PHASE 3: Landing")
                for a in self.agents:
                    a.state = FlightState.LAND
                    a.state_timer = 0
                self.phase = 3
                
        if all(a.get_state() == FlightState.COMPLETE for a in self.agents):
            self.get_logger().info("Mission Complete!")
            self.log_file.close()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SmokeTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
