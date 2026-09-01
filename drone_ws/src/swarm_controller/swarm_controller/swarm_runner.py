#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import argparse
import os
import json
from std_msgs.msg import String
from swarm_controller.configs import DroneConfig, MissionConfig
from swarm_controller.drone_agent import DroneAgent, FlightState
from swarm_controller.qmix_mission_controller import QMIXMissionController

class SwarmOrchestratorNode(QMIXMissionController):
    """
    Subclasses QMIXMissionController to safely tick and publish telemetry for D2/D3
    without breaking the PyTorch dimension constraints for D0/D1.
    """
    def __init__(self, mission_config, qmix_configs, det_configs, checkpoint_path, max_decisions=300):
        super().__init__(mission_config, qmix_configs, checkpoint_path, max_decisions=max_decisions)
        self.get_logger().info(f"SwarmOrchestrator: {len(qmix_configs)} QMIX agents, {len(det_configs)} Deterministic agents.")

        self.det_agents = [DroneAgent(self, config, mission_config) for config in det_configs]

        # Instantiate the new Swarm Coordination Manager
        from swarm_controller.swarm_coordination_manager import SwarmCoordinationManager
        self.coordination_manager = SwarmCoordinationManager(self)

        # Explicitly subscribe to D2/D3 perception to feed the global VictimManager
        from std_msgs.msg import String
        for agent in self.det_agents:
            topic = f'/drone_{agent.config.drone_id}/camera/detection_data'
            sub = self.create_subscription(
                String,
                topic,
                lambda msg, d_id=agent.config.drone_id: self.detection_callback(msg, d_id),
                10
            )
            self.detection_subs.append(sub)

    def tick(self):
        super().tick()

        # Tick deterministic agents locally
        for agent in self.det_agents:
            agent.tick()

        # Delegate shared state updates and coordination logic
        self.coordination_manager.tick_coordination()

    def _publish_telemetry(self, current_time):
        # Temporarily append deterministic agents to self.agents so super() logs them cleanly
        original_agents = list(self.agents)
        self.agents.extend(self.det_agents)

        # Intercept publisher to inject coordination data safely
        original_publish = self.telemetry_pub.publish
        def intercept_publish(msg):
            data = json.loads(msg.data)
            data["coordination"] = {
                "active_frontiers": {str(k): list(v["target_cell"]) for k, v in self.coordination_manager.reserved_targets.items()},
                "safety_holds": {str(k): str(v) for k, v in self.coordination_manager.safety_holds.items()},
                "qmix_drones": len([a for a in original_agents]),
                "coord_drones": len([a for a in self.det_agents])
            }
            # Inject safety overrides count into mission
            data["mission"]["safety_overrides"] = len(self.coordination_manager.safety_holds)
            msg.data = json.dumps(data)
            original_publish(msg)

        self.telemetry_pub.publish = intercept_publish
        try:
            super()._publish_telemetry(current_time)
        finally:
            self.telemetry_pub.publish = original_publish
            self.agents = original_agents

class StandaloneDeterministicNode(Node):
    """
    Fallback for N=1 where QMIX cannot be instantiated at all due to tensor dims.
    """
    def __init__(self, mission_config, det_configs):
        super().__init__('standalone_det_node')
        from swarm_controller.grid_world_transform import GridWorldTransform
        from std_msgs.msg import String
        import json

        self.mission_config = mission_config
        self.agents = [DroneAgent(self, config, mission_config) for config in det_configs]
        self.telemetry_pub = self.create_publisher(String, '/swarm/telemetry', 10)
        self.timer = self.create_timer(1.0 / mission_config.control_rate_hz, self.tick)
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        self.det_phase = 0
        self.det_timer = 0

    def tick(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        for agent in self.agents:
            agent.tick()

        if any(a.initial_pos is None for a in self.agents):
            return

        all_ready = all(a.get_state() in [FlightState.WAYPOINT_NAVIGATION, FlightState.HOLD] for a in self.agents)
        all_hold = all(a.get_state() == FlightState.HOLD for a in self.agents)

        if all_ready:
            if self.det_phase == 0:
                for a in self.agents:
                    a.set_mission_goal_local(15.0, 0.0, a.initial_pos[2] - a.mission.takeoff_altitude)
                    if a.get_state() != FlightState.WAYPOINT_NAVIGATION:
                        a._log_transition(FlightState.WAYPOINT_NAVIGATION)
                self.det_phase = 1
                self.det_timer = 0

            elif self.det_phase == 1 and all_hold:
                self.det_timer += 1
                if self.det_timer > 40:
                    for a in self.agents:
                        a.set_mission_goal_local(15.0, 15.0, a.initial_pos[2] - a.mission.takeoff_altitude)
                        if a.get_state() != FlightState.WAYPOINT_NAVIGATION:
                            a._log_transition(FlightState.WAYPOINT_NAVIGATION)
                    self.det_phase = 2
                    self.det_timer = 0

            elif self.det_phase == 2 and all_hold:
                self.det_timer += 1
                if self.det_timer > 40:
                    for a in self.agents:
                        a.set_mission_goal_local(0.0, 15.0, a.initial_pos[2] - a.mission.takeoff_altitude)
                        if a.get_state() != FlightState.WAYPOINT_NAVIGATION:
                            a._log_transition(FlightState.WAYPOINT_NAVIGATION)
                    self.det_phase = 3
                    self.det_timer = 0

            elif self.det_phase == 3 and all_hold:
                self.det_timer += 1
                if self.det_timer > 40:
                    for a in self.agents:
                        a.set_mission_goal_local(0.0, 0.0, a.initial_pos[2] - a.mission.takeoff_altitude)
                        if a.get_state() != FlightState.WAYPOINT_NAVIGATION:
                            a._log_transition(FlightState.WAYPOINT_NAVIGATION)
                    self.det_phase = 0
                    self.det_timer = 0

        # Publish dummy telemetry
        from swarm_controller.grid_world_transform import GridWorldTransform
        import json

        drones_data = []
        for agent in self.agents:
            lx, ly, lz = agent.px4.current_position
            wx = lx + agent.config.world_spawn_x
            wy = ly + agent.config.world_spawn_y
            wz = lz
            gx, gy = GridWorldTransform.world_to_grid(wx, wy)

            drones_data.append({
                "id": str(agent.config.drone_id),
                "state": agent.state.name,
                "x": round(float(wx), 2),
                "y": round(float(wy), 2),
                "z": round(-float(wz), 2),
                "grid_x": int(gx),
                "grid_y": int(gy),
                "action": "DETERMINISTIC",
                "safety_override": False
            })

        msg_dict = {
            "type": "telemetry",
            "timestamp": current_time,
            "mission": {
                "status": "RUNNING",
                "decision_count": 0,
                "qmix_enabled": False,
                "apf_enabled": False
            },
            "drones": drones_data,
            "victims_state": [],
            "tracked_victims_state": [],
            "unique_victims_detected": 0,
            "coverage_percent": 0.0,
            "explored_cells": []
        }

        msg = String()
        msg.data = json.dumps(msg_dict)
        self.telemetry_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser()
    parser.add_argument('--map', type=str, default='realistic_sar')
    parser.add_argument('--drones', type=int, default=2)
    parsed, _ = parser.parse_known_args()

    # Load spawn coordinates
    workspace_dir = "/home/capstone/capstone_project_antigravity"
    if parsed.map == "realistic_sar":
        meta_file = os.path.join(workspace_dir, "worlds/generated_world_meta.json")
    else:
        meta_file = os.path.join(workspace_dir, "worlds/earthquake_world_meta.json")

    with open(meta_file, 'r') as f:
        meta = json.load(f)

    mission_config = MissionConfig(
        enable_apf=False,
        control_rate_hz=20,
        takeoff_altitude=15.0
    )

    all_configs = []
    for i in range(parsed.drones):
        s = meta['drone_base']['spawns'][i]
        c = DroneConfig(
            drone_id=f"drone_{i}",
            namespace=f"drone_{i}",
            system_id=i+1,
            world_spawn_x=float(s['x']),
            world_spawn_y=float(s['y']),
            world_spawn_z=0.2,
            world_yaw=0.0
        )
        all_configs.append(c)

    checkpoint_path = os.path.join(workspace_dir, "models/qmix_sar_v4_align_best.pth")

    if parsed.drones == 1:
        node = StandaloneDeterministicNode(mission_config, all_configs)
    elif parsed.drones == 2:
        # Strictly identical to qmix_drone_test.py for 2 drones baseline
        node = QMIXMissionController(mission_config, all_configs, checkpoint_path, max_decisions=300)
    else:
        # 3 or 4 drones -> orchestrator delegates 0,1 to QMIX and 2,3 to Deterministic
        qmix_configs = all_configs[0:2]
        det_configs = all_configs[2:]
        node = SwarmOrchestratorNode(mission_config, qmix_configs, det_configs, checkpoint_path, max_decisions=300)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
