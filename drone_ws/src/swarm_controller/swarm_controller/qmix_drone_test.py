#!/usr/bin/env python3
import rclpy
import os
from swarm_controller.configs import DroneConfig, MissionConfig
from swarm_controller.qmix_mission_controller import QMIXMissionController

def main(args=None):
    rclpy.init(args=args)
    
    # 20 Hz control rate. 
    # For action timeout, we wait 4 seconds. 4 * 20 = 80 ticks.
    mission_config = MissionConfig(
        enable_apf=False, 
        control_rate_hz=20, 
        hold_duration=80, 
        goal_tolerance=0.75
    )
    
    config0 = DroneConfig(
        drone_id="drone_0", namespace="drone_0", system_id=1,
        world_spawn_x=12.0, world_spawn_y=12.0, world_spawn_z=0.2, world_yaw=0.0
    )
    config1 = DroneConfig(
        drone_id="drone_1", namespace="drone_1", system_id=2,
        world_spawn_x=12.0, world_spawn_y=20.0, world_spawn_z=0.2, world_yaw=0.0
    )
    
    checkpoint_path = os.path.expanduser("~/capstone_project_antigravity/marl_drone_project/models/qmix_sar_v3_best_best.pth")
    
    # Extended integration test (40 max decisions)
    node = QMIXMissionController(
        mission_config=mission_config,
        drone_configs=[config0, config1],
        checkpoint_path=checkpoint_path,
        max_decisions=40
    )
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Test exited: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
