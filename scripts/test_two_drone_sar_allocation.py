#!/usr/bin/env python3
"""
scripts/test_two_drone_sar_allocation.py

Verification test suite for Ground-Truth-Independent SAR Task Allocation and Mission Execution:
1. Two-Drone QMIX Task Allocation (Defect A fix verification).
2. Autonomous Discovery Ingestion and 1-to-1 Drone Assignment.
3. SAR Investigation Priority over QMIX Exploration (Defect B fix verification).
4. Physical Investigation Hover Dwell and Lifecycle Completion.
5. Post-Completion Return to QMIX Exploration.
6. Six-Drone Architecture Backwards Compatibility.
7. Strict Ground-Truth Isolation.
"""

import os
import sys
import json
import time
import unittest
from unittest.mock import MagicMock, patch

project_root = "/home/capstone/capstone_project_antigravity"
if project_root not in sys.path:
    sys.path.insert(0, project_root)

drone_ws_path = os.path.join(project_root, "drone_ws", "src", "swarm_controller")
if drone_ws_path not in sys.path:
    sys.path.insert(0, drone_ws_path)

import rclpy
from std_msgs.msg import String

from swarm_controller.configs import DroneConfig, MissionConfig
from swarm_controller.drone_agent import DroneAgent, FlightState
from swarm_controller.grid_world_transform import GridWorldTransform
from swarm_controller.sar_task_allocator import SARTaskAllocator, TaskState, SARTask
from swarm_controller.victim_manager import VictimManager, VictimState, TrackedVictim
from swarm_controller.qmix_mission_controller import QMIXMissionController
from swarm_controller.swarm_runner import SwarmOrchestratorNode


def mock_healthy_agent(agent, state=FlightState.WAYPOINT_NAVIGATION):
    agent.initial_pos = [0.0, 0.0, -15.0]
    agent.state = state
    agent.px4.connected = True
    agent.px4.arming_state = 2
    agent.px4.nav_state = 14  # NAVIGATION_STATE_OFFBOARD
    agent.px4.odometry_count = 10
    agent.px4.last_odometry_wall_time = time.time()


class TestTwoDroneSARAllocation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        GridWorldTransform.set_bounds(width=25, height=25, meters_per_cell=4.0, origin_x=0.0, origin_y=0.0)

        self.mission_config = MissionConfig(
            enable_apf=False,
            control_rate_hz=20,
            takeoff_altitude=15.0,
            goal_tolerance=2.0,
            min_waypoint_dwell_ticks=2,
            waypoint_wall_watchdog=90.0,
            stall_detection_sec=6.0
        )

        self.two_drone_configs = [
            DroneConfig(
                drone_id="drone_0",
                namespace="drone_0",
                system_id=1,
                world_spawn_x=21.0,
                world_spawn_y=25.0,
                world_spawn_z=0.2,
                world_yaw=0.0
            ),
            DroneConfig(
                drone_id="drone_1",
                namespace="drone_1",
                system_id=2,
                world_spawn_x=29.0,
                world_spawn_y=25.0,
                world_spawn_z=0.2,
                world_yaw=0.0
            )
        ]

        self.checkpoint_path = os.path.join(project_root, "models", "qmix_sar_v4_align_best.pth")
        self.meta_file = os.path.join(project_root, "worlds", "generated_world_meta.json")

    def test_01_two_drone_task_allocation_on_discovery(self):
        """Verify that in a 2-drone mission, an autonomous discovery triggers task allocation
        to the closest available QMIX drone (Defect A resolution)."""
        node = QMIXMissionController(
            mission_config=self.mission_config,
            drone_configs=self.two_drone_configs,
            checkpoint_path=self.checkpoint_path,
            max_decisions=50,
            meta_file=self.meta_file
        )
        try:
            # Set initial airborne states
            for a in node.agents:
                mock_healthy_agent(a, FlightState.WAYPOINT_NAVIGATION)

            # Drone 0 is at (20.0, 24.0); Drone 1 is at (30.0, 24.0)
            ln0, le0 = GridWorldTransform.world_enu_to_px4_ned(20.0, 24.0, 21.0, 25.0)
            node.agents[0].px4.current_position = [ln0, le0, -15.0]

            ln1, le1 = GridWorldTransform.world_enu_to_px4_ned(30.0, 24.0, 29.0, 25.0)
            node.agents[1].px4.current_position = [ln1, le1, -15.0]

            # Ingest autonomous discovery near Drone 0 at (16.0, 24.0)
            event_payload = {
                "event_type": "VICTIM_DISCOVERED",
                "timestamp": 10.0,
                "victim_track_id": 42,
                "confirming_drone_id": 0,
                "world_x": 16.0,
                "world_y": 24.0,
                "confidence": 0.55,
                "source": "YOLO_RGBD",
                "verification_mode": "RGBD_TEMPORAL",
                "evidence_state": "TEMPORALLY_CONFIRMED"
            }
            msg = String()
            msg.data = json.dumps(event_payload)
            node.discovery_callback(msg)

            # Assert task was created in DISCOVERED state
            self.assertEqual(len(node.task_allocator.tasks), 1)
            task_id = list(node.task_allocator.tasks.keys())[0]
            task = node.task_allocator.tasks[task_id]
            self.assertEqual(task.state, TaskState.DISCOVERED)

            # Trigger controller tick with active mission
            node.mission_active = True
            node.tick()

            # Task MUST be assigned to drone_0 (distance=4.0m vs drone_1 distance=14.0m)
            self.assertEqual(task.state, TaskState.ASSIGNED)
            self.assertEqual(task.assigned_drone_id, "drone_0")
            self.assertEqual(node.task_allocator.drone_assignments.get("drone_0"), task_id)
            self.assertNotIn("drone_1", node.task_allocator.drone_assignments)
        finally:
            node.destroy_node()

    def test_02_sar_investigation_priority_over_qmix_exploration(self):
        """Verify that when a QMIX drone has an assigned SAR task, its next waypoint steps
        toward the sensor-derived victim coordinates rather than dispatching exploratory QMIX actions
        (Defect B resolution)."""
        node = QMIXMissionController(
            mission_config=self.mission_config,
            drone_configs=self.two_drone_configs,
            checkpoint_path=self.checkpoint_path,
            max_decisions=50,
            meta_file=self.meta_file
        )
        try:
            for a in node.agents:
                mock_healthy_agent(a, FlightState.HOLD)

            # Drone 0 at (20.0, 24.0) -> grid (5, 6)
            # Drone 1 at (30.0, 24.0) -> grid (7, 6)
            ln0, le0 = GridWorldTransform.world_enu_to_px4_ned(20.0, 24.0, 21.0, 25.0)
            node.agents[0].px4.current_position = [ln0, le0, -15.0]

            ln1, le1 = GridWorldTransform.world_enu_to_px4_ned(30.0, 24.0, 29.0, 25.0)
            node.agents[1].px4.current_position = [ln1, le1, -15.0]

            # Ingest victim at (8.0, 24.0) -> grid (2, 6)
            event_payload = {
                "event_type": "VICTIM_DISCOVERED",
                "timestamp": 12.0,
                "victim_track_id": 99,
                "confirming_drone_id": 0,
                "world_x": 8.0,
                "world_y": 24.0,
                "confidence": 0.52,
                "source": "YOLO_RGBD",
                "verification_mode": "RGBD_TEMPORAL",
                "evidence_state": "TEMPORALLY_CONFIRMED"
            }
            msg = String()
            msg.data = json.dumps(event_payload)
            node.discovery_callback(msg)

            # Trigger tick to allocate task
            node.mission_active = True
            node.tick()

            # Verify assignment
            task_id = list(node.task_allocator.tasks.keys())[0]
            self.assertEqual(node.task_allocator.drone_assignments.get("drone_0"), task_id)

            # Execute global step
            node.execute_global_step()

            # Drone 0 current grid is (5, 6), victim grid is (2, 6).
            # Drone 0 MUST step -X (west) toward victim
            d0_action = node.last_actions.get("drone_0")
            self.assertTrue("SAR_" in d0_action, f"Expected SAR action for drone_0, got {d0_action}")

            # Drone 1 has NO assigned task, so it executes normal QMIX exploration
            d1_action = node.last_actions.get("drone_1")
            self.assertFalse("SAR_" in d1_action, f"Drone 1 should execute normal QMIX, got {d1_action}")
        finally:
            node.destroy_node()

    def test_03_investigation_dwell_and_task_completion(self):
        """Verify that when the assigned drone arrives at the victim location:
        1. Task transitions to INVESTIGATING.
        2. Dwell is accumulated.
        3. Task transitions to COMPLETED after duration.
        4. Drone assignment is released and returns to exploration."""
        node = QMIXMissionController(
            mission_config=self.mission_config,
            drone_configs=self.two_drone_configs,
            checkpoint_path=self.checkpoint_path,
            max_decisions=50,
            meta_file=self.meta_file
        )
        try:
            for a in node.agents:
                mock_healthy_agent(a, FlightState.WAYPOINT_NAVIGATION)

            # Place drone 0 directly at victim coordinates (8.0, 24.0)
            ln0, le0 = GridWorldTransform.world_enu_to_px4_ned(8.0, 24.0, 21.0, 25.0)
            node.agents[0].px4.current_position = [ln0, le0, -15.0]

            event_payload = {
                "event_type": "VICTIM_DISCOVERED",
                "timestamp": 20.0,
                "victim_track_id": 101,
                "confirming_drone_id": 0,
                "world_x": 8.0,
                "world_y": 24.0,
                "confidence": 0.60,
                "source": "YOLO_RGBD",
                "verification_mode": "RGBD_TEMPORAL",
                "evidence_state": "TEMPORALLY_CONFIRMED"
            }
            msg = String()
            msg.data = json.dumps(event_payload)
            node.discovery_callback(msg)

            task_id = list(node.task_allocator.tasks.keys())[0]
            task = node.task_allocator.tasks[task_id]

            node.mission_active = True
            # Sim time t = 20.0: Allocation
            node.agents[0].get_current_sim_time = lambda: 20.0
            node.tick()
            self.assertEqual(task.state, TaskState.ASSIGNED)

            # Sim time t = 20.1: Transition to APPROACHING -> INVESTIGATING (distance = 0 <= 3.0m)
            node.agents[0].get_current_sim_time = lambda: 20.1
            node.tick()
            self.assertEqual(task.state, TaskState.INVESTIGATING)

            # Sim time t = 23.0: Mid-investigation dwell (elapsed 2.9s < 5.0s)
            node.agents[0].get_current_sim_time = lambda: 23.0
            node.tick()
            self.assertEqual(task.state, TaskState.INVESTIGATING)
            self.assertIn("drone_0", node.task_allocator.drone_assignments)

            # Sim time t = 25.5: Dwell completed (elapsed 5.4s >= 5.0s)
            node.agents[0].get_current_sim_time = lambda: 25.5
            node.tick()
            self.assertEqual(task.state, TaskState.COMPLETED)
            self.assertNotIn("drone_0", node.task_allocator.drone_assignments)

            # On next step, drone 0 resumes normal exploration
            node.execute_global_step()
            d0_action = node.last_actions.get("drone_0")
            self.assertFalse("SAR_" in d0_action, f"Drone 0 should have returned to exploration, got {d0_action}")
        finally:
            node.destroy_node()

    def test_04_six_drone_backwards_compatibility(self):
        """Verify that six-drone orchestration still assigns tasks to deterministic agents (D2..D5)."""
        spawns = [
            {"x": 21.0, "y": 25.0},
            {"x": 29.0, "y": 25.0},
            {"x": 10.0, "y": 10.0},
            {"x": 30.0, "y": 30.0},
            {"x": -40.0, "y": 10.0},
            {"x": 50.0, "y": -10.0},
        ]
        all_configs = [
            DroneConfig(
                drone_id=f"drone_{i}",
                namespace=f"drone_{i}",
                system_id=i+1,
                world_spawn_x=spawns[i]["x"],
                world_spawn_y=spawns[i]["y"],
                world_spawn_z=0.2,
                world_yaw=0.0
            ) for i in range(6)
        ]
        qmix_configs = all_configs[0:2]
        det_configs = all_configs[2:6]

        node = SwarmOrchestratorNode(
            mission_config=self.mission_config,
            qmix_configs=qmix_configs,
            det_configs=det_configs,
            checkpoint_path=self.checkpoint_path,
            max_decisions=50,
            meta_file=self.meta_file
        )
        try:
            for a in node.agents + node.det_agents:
                mock_healthy_agent(a, FlightState.WAYPOINT_NAVIGATION)

            # Place D2 at (10.0, 10.0) -> close to (12.0, 12.0)
            ln2, le2 = GridWorldTransform.world_enu_to_px4_ned(10.0, 10.0, 10.0, 10.0)
            node.det_agents[0].px4.current_position = [ln2, le2, -15.0]

            event_payload = {
                "event_type": "VICTIM_DISCOVERED",
                "timestamp": 15.0,
                "victim_track_id": 200,
                "confirming_drone_id": 0,
                "world_x": 12.0,
                "world_y": 12.0,
                "confidence": 0.58,
                "source": "YOLO_RGBD",
                "verification_mode": "RGBD_TEMPORAL",
                "evidence_state": "TEMPORALLY_CONFIRMED"
            }
            msg = String()
            msg.data = json.dumps(event_payload)
            node.discovery_callback(msg)

            node.agents[0].get_current_sim_time = lambda: 15.0
            node.tick()

            task_id = list(node.task_allocator.tasks.keys())[0]
            task = node.task_allocator.tasks[task_id]

            # In 6-drone mode, deterministic agent (drone_2) MUST be assigned
            self.assertEqual(task.state, TaskState.ASSIGNED)
            self.assertEqual(task.assigned_drone_id, "drone_2")
            self.assertEqual(node.task_allocator.drone_assignments.get("drone_2"), task_id)
        finally:
            node.destroy_node()


if __name__ == "__main__":
    unittest.main()
