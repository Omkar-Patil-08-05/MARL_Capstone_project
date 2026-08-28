#!/usr/bin/env python3
import rclpy
import os
import argparse
import json
import sys
from swarm_controller.configs import DroneConfig, MissionConfig
from swarm_controller.qmix_mission_controller import QMIXMissionController

def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', type=str, default='realistic_sar', help='Map ID to load')
    
    # Filter out ros args if present
    argv = sys.argv[1:]
    # Split ros args from script args if needed
    if '--ros-args' in argv:
        idx = argv.index('--ros-args')
        script_args = argv[:idx]
    else:
        script_args = argv
        
    parsed_args = parser.parse_args(script_args)
    map_id = parsed_args.map
    
    if map_id == "realistic_sar":
        meta_filename = "generated_world_meta.json"
    else:
        raise ValueError(f"Unsupported map ID for simulation: {map_id}")
        
    meta_path = os.path.expanduser(f"~/capstone_project_antigravity/worlds/{meta_filename}")
    with open(meta_path, 'r') as f:
        meta = json.load(f)
        
    spawns = meta.get("drone_base", {}).get("spawns", [])
    if len(spawns) < 2:
        raise ValueError("Map metadata does not contain 2 drone spawn points.")
        
    d0_x, d0_y = spawns[0]["x"], spawns[0]["y"]
    d1_x, d1_y = spawns[1]["x"], spawns[1]["y"]
    
    rclpy.init(args=args)
    
    # 20 Hz control rate. 
    # For action timeout, we wait 4 seconds. 4 * 20 = 80 ticks.
    mission_config = MissionConfig(
        enable_apf=False, 
        control_rate_hz=20, 
        hold_duration=3000, 
        waypoint_timeout=150,
        goal_tolerance=0.75
    )
    
    config0 = DroneConfig(
        drone_id="drone_0", namespace="drone_0", system_id=1,
        world_spawn_x=float(d0_x), world_spawn_y=float(d0_y), world_spawn_z=0.2, world_yaw=0.0
    )
    config1 = DroneConfig(
        drone_id="drone_1", namespace="drone_1", system_id=2,
        world_spawn_x=float(d1_x), world_spawn_y=float(d1_y), world_spawn_z=0.2, world_yaw=0.0
    )
    
    checkpoint_path = os.path.expanduser("~/capstone_project_antigravity/models/qmix_sar_v4_align_best.pth")
    
    # Extended integration test (300 max decisions for H4.2 long SAR physical test)
    node = QMIXMissionController(
        mission_config=mission_config,
        drone_configs=[config0, config1],
        checkpoint_path=checkpoint_path,
        max_decisions=300
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
