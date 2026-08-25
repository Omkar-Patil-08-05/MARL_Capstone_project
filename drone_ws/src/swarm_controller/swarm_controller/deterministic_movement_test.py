#!/usr/bin/env python3
import os
import csv
import math
import rclpy
from rclpy.node import Node
from swarm_controller.configs import DroneConfig, MissionConfig
from swarm_controller.drone_agent import DroneAgent, FlightState
from swarm_controller.grid_world_transform import GridWorldTransform

class DeterministicMovementTest(Node):
    def __init__(self):
        super().__init__('deterministic_movement_test_node')
        
        self.phase = 0
        self.phase_timer = 0
        
        # Configuration matches the approved requirements
        mission_config = MissionConfig(enable_apf=False, control_rate_hz=20, hold_duration=400, goal_tolerance=0.75)
        
        # drone_0: Start at cell (3,3) origin -> (12,12, Z)
        self.config0 = DroneConfig(drone_id="drone_0", namespace="drone_0", system_id=1, 
                                   world_spawn_x=12.0, world_spawn_y=12.0, world_spawn_z=0.2, world_yaw=0.0)
        # drone_1: Start at cell (3,5) origin -> (12,20, Z)
        self.config1 = DroneConfig(drone_id="drone_1", namespace="drone_1", system_id=2, 
                                   world_spawn_x=12.0, world_spawn_y=20.0, world_spawn_z=0.2, world_yaw=0.0)
                                   
        self.a0 = DroneAgent(self, self.config0, mission_config)
        self.a1 = DroneAgent(self, self.config1, mission_config)
        self.agents = [self.a0, self.a1]
        
        # CSV Logging Setup
        self.results_dir = os.path.join(os.getcwd(), "results", "deterministic_test")
        os.makedirs(self.results_dir, exist_ok=True)
        self.csv_files = {}
        self.csv_writers = {}
        self.max_displacements = {"drone_0": 0.0, "drone_1": 0.0}
        self.max_errors = {"drone_0": 0.0, "drone_1": 0.0}
        
        for agent in self.agents:
            f = open(os.path.join(self.results_dir, f"{agent.config.drone_id}.csv"), 'w', newline='')
            writer = csv.writer(f)
            writer.writerow([
                "timestamp", "state", "x", "y", "z",
                "target_x", "target_y", "target_z", "distance_to_target"
            ])
            self.csv_files[agent.config.drone_id] = f
            self.csv_writers[agent.config.drone_id] = writer

        self.timer = self.create_timer(1.0 / mission_config.control_rate_hz, self.tick)
        self.get_logger().info("Deterministic Movement Test Initialized.")

    def write_telemetry(self, agent: DroneAgent):
        t = self.get_clock().now().nanoseconds / 1e9
        
        lx, ly, lz = agent.px4.current_position
        gx, gy, gz = agent.mission_goal_local
        
        dx = lx - gx
        dy = ly - gy
        dist = math.sqrt(dx*dx + dy*dy)
        
        # Max error in HOLD state
        if agent.state == FlightState.HOLD:
            self.max_errors[agent.config.drone_id] = max(self.max_errors[agent.config.drone_id], dist)
            
        # Max displacement from spawn
        if agent.initial_pos is not None:
            disp = math.sqrt((lx - agent.initial_pos[0])**2 + (ly - agent.initial_pos[1])**2)
            self.max_displacements[agent.config.drone_id] = max(self.max_displacements[agent.config.drone_id], disp)
            
        self.csv_writers[agent.config.drone_id].writerow([
            t, agent.state.name, lx, ly, lz, gx, gy, gz, dist
        ])

    def tick(self):
        for a in self.agents:
            a.tick()
            self.write_telemetry(a)

        # Ensure estimator has initialized initial_pos before running state machine
        if any(a.initial_pos is None for a in self.agents):
            return

        all_nav_or_hold = all(a.get_state() in [FlightState.WAYPOINT_NAVIGATION, FlightState.HOLD] for a in self.agents)
        all_hold = all(a.get_state() == FlightState.HOLD for a in self.agents)
        all_complete = all(a.get_state() == FlightState.COMPLETE for a in self.agents)
        
        if self.phase == 0:
            if all_nav_or_hold:
                # Issue forward commands (+8m)
                # Note: DroneAgent uses Local frame which starts at 0,0 where the drone spawns.
                # +X for drone 0, +Y for drone 1
                self.a0.set_mission_goal_local(8.0, 0.0, self.a0.initial_pos[2] - self.a0.mission.takeoff_altitude)
                self.a1.set_mission_goal_local(0.0, 8.0, self.a1.initial_pos[2] - self.a1.mission.takeoff_altitude)
                self.a0._log_transition(FlightState.WAYPOINT_NAVIGATION)
                self.a1._log_transition(FlightState.WAYPOINT_NAVIGATION)
                self.phase = 1
                self.get_logger().info("Phase 1: Commanding forward waypoints (+8m)")
                
        elif self.phase == 1:
            if all_hold:
                self.phase = 2
                self.phase_timer = 0
                self.get_logger().info("Phase 2: Holding at forward waypoints")
                
        elif self.phase == 2:
            self.phase_timer += 1
            if self.phase_timer > 100:
                self.a0.set_mission_goal_local(0.0, 0.0, self.a0.initial_pos[2] - self.a0.mission.takeoff_altitude)
                self.a1.set_mission_goal_local(0.0, 0.0, self.a1.initial_pos[2] - self.a1.mission.takeoff_altitude)
                self.a0._log_transition(FlightState.WAYPOINT_NAVIGATION)
                self.a1._log_transition(FlightState.WAYPOINT_NAVIGATION)
                self.phase = 3
                self.get_logger().info("Phase 3: Commanding return waypoints (0m)")
                
        elif self.phase == 3:
            if all_hold:
                self.phase = 4
                self.phase_timer = 0
                self.get_logger().info("Phase 4: Holding at return waypoints")
                
        elif self.phase == 4:
            self.phase_timer += 1
            if self.phase_timer > 100:
                # Land is automatically triggered by DroneAgent HOLD state logic after hold_duration!
                self.phase = 5
                self.get_logger().info("Phase 5: DroneAgent autonomous LAND triggered")

        elif self.phase == 5:
            if all_complete:
                self.get_logger().info(f"Test Complete!")
                self.get_logger().info(f"Drone 0 max displacement: {self.max_displacements['drone_0']:.2f} m, max error: {self.max_errors['drone_0']:.2f} m")
                self.get_logger().info(f"Drone 1 max displacement: {self.max_displacements['drone_1']:.2f} m, max error: {self.max_errors['drone_1']:.2f} m")
                
                for f in self.csv_files.values():
                    f.close()
                self.destroy_node()
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DeterministicMovementTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Test exited: {e}")

if __name__ == '__main__':
    main()
