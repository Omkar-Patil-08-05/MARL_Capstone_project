#!/usr/bin/env python3
import argparse
import rclpy
from rclpy.node import Node

from swarm_controller.configs import DroneConfig, MissionConfig
from swarm_controller.drone_agent import DroneAgent
from swarm_controller.mission_controller import MissionController

class TwoDroneTestNode(Node):
    def __init__(self, active_drones, enable_apf=False):
        super().__init__('two_drone_test_node')
        self.get_logger().info(f"Starting Two Drone Test Node. Active drones: {active_drones}. APF mode: {enable_apf}")
        
        mission_config = MissionConfig(enable_apf=enable_apf)
        agents = []
        
        if "drone_0" in active_drones:
            config = DroneConfig(
                drone_id="drone_0", namespace="drone_0", system_id=1,
                world_spawn_x=24.0, world_spawn_y=120.0, world_spawn_z=0.5, world_yaw=0.0
            )
            agent = DroneAgent(self, config, mission_config)
            agents.append(agent)
            
        if "drone_1" in active_drones:
            config = DroneConfig(
                drone_id="drone_1", namespace="drone_1", system_id=2,
                world_spawn_x=72.0, world_spawn_y=120.0, world_spawn_z=0.5, world_yaw=0.0
            )
            agent = DroneAgent(self, config, mission_config)
            agents.append(agent)
            
        self.controller = MissionController(self, agents, mission_config.control_rate_hz)

def main(args=None):
    parser = argparse.ArgumentParser(description='Run two drone test.')
    parser.add_argument('--drones', nargs='+', default=['drone_0', 'drone_1'], help='List of drones to control, e.g., drone_0 drone_1')
    parser.add_argument('--apf', action='store_true', help='Enable APF collision avoidance.')
    
    # Parse known args so rclpy.init can take the rest
    parsed_args, ros_args = parser.parse_known_args()
    
    rclpy.init(args=ros_args)
    node = TwoDroneTestNode(parsed_args.drones, parsed_args.apf)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
