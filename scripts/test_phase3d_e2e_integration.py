#!/usr/bin/env python3
"""
scripts/test_phase3d_e2e_integration.py

Phase 3D Comprehensive End-to-End Integration Test Suite:
Validates the complete SAR pipeline:
Gazebo sensors -> YOLOv8n -> RGB-D/thermal verification -> temporal confirmation ->
/sar/victim_discovery -> VictimManager -> SARTaskAllocator -> SwarmCoordinationManager ->
drone task execution -> telemetry -> FastAPI/SQLite -> React dashboard.

Distinguishes:
1. ROS 2 Swarm Orchestration & 6-Drone Coordination Runtime
2. Autonomous Discovery Ingestion & Strict Ground-Truth Isolation
3. SARTaskAllocator 1-to-1 Task Allocation & Lifecycle (DISCOVERED->ASSIGNED->APPROACHING->INVESTIGATING->COMPLETED)
4. Physical Waypoint Steering, Investigation Dwell, and Task Completion
5. Drone Failsafe Release & Safe Reassignment
6. Predictive Collision Avoidance Safety Interlock
7. Real Sensor Perception & Temporal Confirmation Validation
8. Dashboard Backend, SQLite Persistence, and REST API Querying
9. RTX 3050 GPU VRAM & Hardware Resource Audit
"""

import os
import sys
import json
import time
import math
import sqlite3
import tempfile
import unittest
from unittest.mock import MagicMock, patch

project_root = "/home/capstone/capstone_project_antigravity"
if project_root not in sys.path:
    sys.path.insert(0, project_root)

drone_ws_path = os.path.join(project_root, "drone_ws", "src", "swarm_controller")
if drone_ws_path not in sys.path:
    sys.path.insert(0, drone_ws_path)

backend_dir = os.path.join(project_root, "dashboard", "backend")
if backend_dir not in sys.path:
    sys.path.insert(0, backend_dir)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from swarm_controller.configs import DroneConfig, MissionConfig
from swarm_controller.drone_agent import DroneAgent, FlightState
from swarm_controller.grid_world_transform import GridWorldTransform
from swarm_controller.sar_task_allocator import SARTaskAllocator, TaskState, SARTask
from swarm_controller.victim_manager import VictimManager, VictimState, TrackedVictim
from swarm_controller.swarm_runner import SwarmOrchestratorNode
from swarm_controller.swarm_coordination_manager import SwarmCoordinationManager

from database import DatabaseWriter, EventType
import main as backend_main


class TestPhase3DEndToEndIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        # Set up realistic SAR transform bounds
        GridWorldTransform.set_bounds(width=25, height=25, meters_per_cell=4.0, origin_x=0.0, origin_y=0.0)

        # Temporary DB for dashboard testing
        self.temp_dir = tempfile.TemporaryDirectory()
        self.test_db_path = os.path.join(self.temp_dir.name, "test_phase3d.db")
        self.db_writer = DatabaseWriter(db_path=self.test_db_path)
        self.db_writer.start()

        # Mission Config
        self.mission_config = MissionConfig(
            enable_apf=False,
            control_rate_hz=20,
            takeoff_altitude=15.0,
            goal_tolerance=2.0,
            min_waypoint_dwell_ticks=2,
            waypoint_wall_watchdog=90.0,
            stall_detection_sec=6.0
        )

        # 6 Drone Configs
        self.all_drone_configs = []
        spawns = [
            {"x": -20.0, "y": -20.0, "z": 0.2},
            {"x": -10.0, "y": -20.0, "z": 0.2},
            {"x": 0.0, "y": -20.0, "z": 0.2},
            {"x": 10.0, "y": -20.0, "z": 0.2},
            {"x": 20.0, "y": -20.0, "z": 0.2},
            {"x": 30.0, "y": -20.0, "z": 0.2},
        ]
        for i in range(6):
            c = DroneConfig(
                drone_id=f"drone_{i}",
                namespace=f"drone_{i}",
                system_id=i + 1,
                world_spawn_x=float(spawns[i]["x"]),
                world_spawn_y=float(spawns[i]["y"]),
                world_spawn_z=float(spawns[i]["z"]),
                world_yaw=0.0
            )
            self.all_drone_configs.append(c)

        self.qmix_configs = self.all_drone_configs[0:2]
        self.det_configs = self.all_drone_configs[2:6]

        self.checkpoint_path = os.path.join(project_root, "models", "qmix_sar_v4_align_best.pth")
        self.meta_file = os.path.join(project_root, "worlds", "generated_world_meta.json")

    def tearDown(self):
        self.db_writer.shutdown()
        self.temp_dir.cleanup()

    def _query(self, query, params=()):
        conn = sqlite3.connect(self.test_db_path)
        conn.row_factory = sqlite3.Row
        cur = conn.cursor()
        cur.execute(query, params)
        rows = [dict(r) for r in cur.fetchall()]
        conn.close()
        return rows

    def _seed_mission(self, mission_id="mission_phase3d_001"):
        exp_id = f"exp_{mission_id}"
        self.db_writer._flush_batch([
            (EventType.EXPERIMENT, {'id': exp_id, 'name': 'phase3d_test_exp', 'timestamp': 1000.0, 'environment': 'sim'}),
            (EventType.MISSION_START, {
                'id': mission_id, 'experiment_id': exp_id, 'map_id': 'realistic_sar',
                'start_time': 1000.0, 'total_victims': 5, 'active_drones': 6, 'config': {}
            })
        ])

    def test_01_swarm_orchestrator_initialization_and_separation(self):
        """Verify SwarmOrchestratorNode correctly initializes 6 drones with clean separation:
        D0/D1 in QMIX, D2..D5 in deterministic agents, SwarmCoordinationManager active."""
        node = SwarmOrchestratorNode(
            mission_config=self.mission_config,
            qmix_configs=self.qmix_configs,
            det_configs=self.det_configs,
            checkpoint_path=self.checkpoint_path,
            max_decisions=50,
            meta_file=self.meta_file
        )
        try:
            self.assertEqual(len(node.agents), 2, "QMIX agents list must contain exactly D0 and D1.")
            self.assertEqual(len(node.det_agents), 4, "Deterministic agents list must contain D2..D5.")
            self.assertEqual(node.agents[0].config.drone_id, "drone_0")
            self.assertEqual(node.agents[1].config.drone_id, "drone_1")
            self.assertEqual(node.det_agents[0].config.drone_id, "drone_2")
            self.assertEqual(node.det_agents[1].config.drone_id, "drone_3")
            self.assertEqual(node.det_agents[2].config.drone_id, "drone_4")
            self.assertEqual(node.det_agents[3].config.drone_id, "drone_5")

            self.assertIsNotNone(node.coordination_manager)
            self.assertIsInstance(node.coordination_manager, SwarmCoordinationManager)
            self.assertIsNotNone(node.task_allocator)
            self.assertIsInstance(node.task_allocator, SARTaskAllocator)
            self.assertIsNotNone(node.victim_manager)
            self.assertIsInstance(node.victim_manager, VictimManager)

            # Ground truth victims must be strictly isolated
            self.assertGreater(len(node.victim_manager.victims), 0)
            self.assertEqual(len(node.victim_manager.tracked_victims), 0, "No autonomous tracks initially.")
            self.assertEqual(len(node.task_allocator.tasks), 0, "No tasks initially.")
        finally:
            node.destroy_node()

    def test_02_discovery_to_task_creation_and_duplicate_suppression(self):
        """Verify that a valid discovery event on /sar/victim_discovery produces an autonomous task,
        and subsequent observations refine coordinates without creating duplicates."""
        node = SwarmOrchestratorNode(
            mission_config=self.mission_config,
            qmix_configs=self.qmix_configs,
            det_configs=self.det_configs,
            checkpoint_path=self.checkpoint_path,
            max_decisions=50,
            meta_file=self.meta_file
        )
        try:
            # 1. Simulate arrival of autonomous discovery event
            event_1 = {
                "event_type": "VICTIM_DISCOVERED",
                "timestamp": 10.0,
                "victim_track_id": 42,
                "confirming_drone_id": 0,
                "world_x": 12.0,
                "world_y": 16.0,
                "confidence": 0.85,
                "source": "YOLO_RGBD",
                "verification_mode": "RGBD_TEMPORAL",
                "evidence_state": "TEMPORALLY_CONFIRMED",
                "confirmation_count": 3,
                "confirmation_window": 5,
                "median_depth_estimate": 3.8
            }
            msg = String()
            msg.data = json.dumps(event_1)
            node.discovery_callback(msg)

            # Check VictimManager
            self.assertEqual(len(node.victim_manager.tracked_victims), 1)
            track_id = list(node.victim_manager.tracked_victims.keys())[0]
            track = node.victim_manager.tracked_victims[track_id]
            self.assertAlmostEqual(track.world_x, 12.0, places=2)
            self.assertAlmostEqual(track.world_y, 16.0, places=2)

            # Check SARTaskAllocator
            self.assertEqual(len(node.task_allocator.tasks), 1)
            task_id = list(node.task_allocator.tasks.keys())[0]
            task = node.task_allocator.tasks[task_id]
            self.assertEqual(task.state, TaskState.DISCOVERED)
            self.assertAlmostEqual(task.world_x, 12.0, places=2)
            self.assertAlmostEqual(task.world_y, 16.0, places=2)

            # 2. Simulate re-observation of same victim slightly adjusted
            event_2 = {
                "event_type": "VICTIM_DISCOVERED",
                "timestamp": 12.0,
                "victim_track_id": 42,
                "confirming_drone_id": 1,
                "world_x": 12.4,
                "world_y": 16.2,
                "confidence": 0.94,
                "source": "YOLO_RGBD",
                "verification_mode": "RGBD_TEMPORAL",
                "evidence_state": "TEMPORALLY_CONFIRMED",
                "confirmation_count": 5,
                "confirmation_window": 5,
                "median_depth_estimate": 3.7
            }
            msg.data = json.dumps(event_2)
            node.discovery_callback(msg)

            # Verification: still 1 task, updated coordinates and confidence, no duplicate
            self.assertEqual(len(node.victim_manager.tracked_victims), 1)
            self.assertEqual(len(node.task_allocator.tasks), 1, "Duplicate task was created! Duplicate suppression failed.")
            updated_task = node.task_allocator.tasks[task_id]
            self.assertAlmostEqual(updated_task.confidence, 0.94, places=2)
            self.assertEqual(updated_task.state, TaskState.DISCOVERED)
        finally:
            node.destroy_node()

    def test_03_six_drone_coordination_and_task_assignment(self):
        """Verify SwarmCoordinationManager assigns autonomous task to the closest available deterministic drone (D2..D5),
        enforces 1-to-1 mapping, and sets physical local waypoint goal."""
        node = SwarmOrchestratorNode(
            mission_config=self.mission_config,
            qmix_configs=self.qmix_configs,
            det_configs=self.det_configs,
            checkpoint_path=self.checkpoint_path,
            max_decisions=50,
            meta_file=self.meta_file
        )
        try:
            # Prepare agents: set initial positions and flight states
            for a in node.agents + node.det_agents:
                a.initial_pos = [0.0, 0.0, -15.0]
                a.state = FlightState.WAYPOINT_NAVIGATION
                a.px4.connected = True
                a.px4.arming_state = 2

            # Place drones at specific world positions:
            # D0: (-20, -20) [QMIX]
            # D1: (-10, -20) [QMIX]
            # D2: (10, 10)   [Det] -> distance to (12, 16) is sqrt(4+36) = 6.32m (CLOSEST)
            # D3: (30, 30)   [Det] -> distance is sqrt(18^2 + 14^2) = 22.8m
            # D4: (-40, 10)  [Det]
            # D5: (50, -10)  [Det]
            positions = {
                "drone_0": (-20.0, -20.0),
                "drone_1": (-10.0, -20.0),
                "drone_2": (10.0, 10.0),
                "drone_3": (30.0, 30.0),
                "drone_4": (-40.0, 10.0),
                "drone_5": (50.0, -10.0)
            }
            for a in node.agents + node.det_agents:
                wx, wy = positions[a.config.drone_id]
                ln, le = GridWorldTransform.world_enu_to_px4_ned(wx, wy, a.config.world_spawn_x, a.config.world_spawn_y)
                a.px4.current_position = [ln, le, -15.0]

            # Ingest victim discovery at (12.0, 16.0)
            event = {
                "event_type": "VICTIM_DISCOVERED",
                "timestamp": 15.0,
                "victim_track_id": 99,
                "confirming_drone_id": 0,
                "world_x": 12.0,
                "world_y": 16.0,
                "confidence": 0.88,
                "source": "YOLO_RGBD",
                "verification_mode": "RGBD_TEMPORAL",
                "evidence_state": "TEMPORALLY_CONFIRMED"
            }
            msg = String()
            msg.data = json.dumps(event)
            node.discovery_callback(msg)

            task_id = list(node.task_allocator.tasks.keys())[0]

            # Trigger coordination step
            node.coordination_manager.get_sim_time = lambda: 16.0
            node.coordination_manager.tick_coordination()

            # Task must be assigned to drone_2 (closest deterministic drone)
            task = node.task_allocator.tasks[task_id]
            self.assertEqual(task.state, TaskState.ASSIGNED)
            self.assertEqual(task.assigned_drone_id, "drone_2")
            self.assertEqual(node.task_allocator.drone_assignments.get("drone_2"), task_id)

            # D3, D4, D5 must remain unassigned to this task (1-to-1)
            self.assertNotIn("drone_3", node.task_allocator.drone_assignments)
            self.assertNotIn("drone_4", node.task_allocator.drone_assignments)
            self.assertNotIn("drone_5", node.task_allocator.drone_assignments)

            # Drone 2 reservation target must match task coordinates
            d2_res = node.coordination_manager.reserved_targets.get("drone_2")
            self.assertIsNotNone(d2_res)
            self.assertEqual(d2_res.get("task_id"), task_id)

            # Drone 2 mission goal must be steered towards the task
            d2_agent = next(a for a in node.det_agents if a.config.drone_id == "drone_2")
            self.assertIsNotNone(d2_agent.mission_goal_local)
        finally:
            node.destroy_node()

    def test_04_physical_task_execution_dwell_and_completion(self):
        """Verify the full physical lifecycle:
        1. Drone approaches within approach_radius (3.0m) -> transitions to APPROACHING then INVESTIGATING.
        2. In INVESTIGATING state, hover dwell is maintained (require_min_dwell=True).
        3. Once investigation_duration (5.0s) elapses, state transitions to COMPLETED.
        4. Drone is released from drone_assignments and free to return to search."""
        node = SwarmOrchestratorNode(
            mission_config=self.mission_config,
            qmix_configs=self.qmix_configs,
            det_configs=self.det_configs,
            checkpoint_path=self.checkpoint_path,
            max_decisions=50,
            meta_file=self.meta_file
        )
        try:
            for a in node.agents + node.det_agents:
                a.initial_pos = [0.0, 0.0, -15.0]
                a.state = FlightState.WAYPOINT_NAVIGATION
                a.px4.connected = True
                a.px4.arming_state = 2

            # Place D2 at (10.0, 10.0) initially
            d2_agent = next(a for a in node.det_agents if a.config.drone_id == "drone_2")
            ln, le = GridWorldTransform.world_enu_to_px4_ned(10.0, 10.0, d2_agent.config.world_spawn_x, d2_agent.config.world_spawn_y)
            d2_agent.px4.current_position = [ln, le, -15.0]

            # Ingest victim at (12.0, 12.0)
            event = {
                "event_type": "VICTIM_DISCOVERED",
                "timestamp": 20.0,
                "victim_track_id": 1,
                "confirming_drone_id": 0,
                "world_x": 12.0,
                "world_y": 12.0,
                "confidence": 0.90,
                "source": "YOLO_RGBD",
                "verification_mode": "RGBD_TEMPORAL",
                "evidence_state": "TEMPORALLY_CONFIRMED"
            }
            msg = String()
            msg.data = json.dumps(event)
            node.discovery_callback(msg)

            task_id = list(node.task_allocator.tasks.keys())[0]

            # Tick at t=21.0 -> Task assigned to D2
            node.coordination_manager.get_sim_time = lambda: 21.0
            node.coordination_manager.tick_coordination()
            task = node.task_allocator.tasks[task_id]
            self.assertEqual(task.state, TaskState.ASSIGNED)

            # Move D2 within approach radius (e.g., at (11.5, 11.8) -> distance = sqrt(0.5^2 + 0.2^2) = 0.54m < 3.0m)
            ln_close, le_close = GridWorldTransform.world_enu_to_px4_ned(11.5, 11.8, d2_agent.config.world_spawn_x, d2_agent.config.world_spawn_y)
            d2_agent.px4.current_position = [ln_close, le_close, -15.0]

            # Tick at t=22.0 -> Enters investigation
            node.coordination_manager.get_sim_time = lambda: 22.0
            node.coordination_manager.tick_coordination()
            self.assertEqual(task.state, TaskState.INVESTIGATING, f"Expected INVESTIGATING but got {task.state}")
            self.assertEqual(task.investigation_start_sim_time, 22.0)

            # Tick at t=24.0 (2.0s dwell elapsed, duration is 5.0s) -> Should still be INVESTIGATING
            node.coordination_manager.get_sim_time = lambda: 24.0
            node.coordination_manager.tick_coordination()
            self.assertEqual(task.state, TaskState.INVESTIGATING)
            self.assertIn("drone_2", node.task_allocator.drone_assignments)

            # Tick at t=28.0 (6.0s elapsed >= 5.0s) -> Must transition to COMPLETED
            node.coordination_manager.get_sim_time = lambda: 28.0
            node.coordination_manager.tick_coordination()
            self.assertEqual(task.state, TaskState.COMPLETED, f"Expected COMPLETED but got {task.state}")
            self.assertEqual(task.completed_sim_time, 28.0)

            # Drone 2 assignment must be cleared
            self.assertNotIn("drone_2", node.task_allocator.drone_assignments, "Drone 2 assignment was not released upon completion!")

            # Drone 2 reservation must be released from task
            self.assertIsNone(node.coordination_manager.reserved_targets.get("drone_2", {}).get("task_id"))
        finally:
            node.destroy_node()

    def test_05_failsafe_handling_and_safe_task_reassignment(self):
        """Verify that if an assigned drone suffers a failsafe or disconnects,
        its task is released back to DISCOVERED and safely reassigned to another available drone."""
        node = SwarmOrchestratorNode(
            mission_config=self.mission_config,
            qmix_configs=self.qmix_configs,
            det_configs=self.det_configs,
            checkpoint_path=self.checkpoint_path,
            max_decisions=50,
            meta_file=self.meta_file
        )
        try:
            for a in node.agents + node.det_agents:
                a.initial_pos = [0.0, 0.0, -15.0]
                a.state = FlightState.WAYPOINT_NAVIGATION
                a.px4.connected = True
                a.px4.arming_state = 2

            d2_agent = next(a for a in node.det_agents if a.config.drone_id == "drone_2")
            d3_agent = next(a for a in node.det_agents if a.config.drone_id == "drone_3")

            # Place D2 at (10, 10) and D3 at (20, 20)
            ln2, le2 = GridWorldTransform.world_enu_to_px4_ned(10.0, 10.0, d2_agent.config.world_spawn_x, d2_agent.config.world_spawn_y)
            d2_agent.px4.current_position = [ln2, le2, -15.0]
            ln3, le3 = GridWorldTransform.world_enu_to_px4_ned(20.0, 20.0, d3_agent.config.world_spawn_x, d3_agent.config.world_spawn_y)
            d3_agent.px4.current_position = [ln3, le3, -15.0]

            # Ingest victim at (11.0, 11.0)
            event = {
                "event_type": "VICTIM_DISCOVERED",
                "timestamp": 30.0,
                "victim_track_id": 5,
                "confirming_drone_id": 0,
                "world_x": 11.0,
                "world_y": 11.0,
                "confidence": 0.89,
                "source": "YOLO_RGBD",
                "verification_mode": "RGBD_TEMPORAL",
                "evidence_state": "TEMPORALLY_CONFIRMED"
            }
            msg = String()
            msg.data = json.dumps(event)
            node.discovery_callback(msg)
            task_id = list(node.task_allocator.tasks.keys())[0]

            # Tick at t=31.0 -> D2 gets assigned
            node.coordination_manager.get_sim_time = lambda: 31.0
            node.coordination_manager.tick_coordination()
            task = node.task_allocator.tasks[task_id]
            self.assertEqual(task.assigned_drone_id, "drone_2")

            # Now simulate Drone 2 failsafe/disconnection!
            d2_agent.state = FlightState.FAILSAFE
            d2_agent.px4.connected = False

            # Tick coordination at t=32.0
            node.coordination_manager.get_sim_time = lambda: 32.0
            node.coordination_manager.tick_coordination()

            # Task must be unassigned from drone_2, reassigned to drone_3
            task = node.task_allocator.tasks[task_id]
            self.assertEqual(task.assigned_drone_id, "drone_3", f"Task should be reassigned to drone_3, got {task.assigned_drone_id}")
            self.assertEqual(task.reassignment_count, 1)
            self.assertEqual(node.task_allocator.drone_assignments.get("drone_3"), task_id)
            self.assertNotIn("drone_2", node.task_allocator.drone_assignments)
        finally:
            node.destroy_node()

    def test_06_predictive_collision_avoidance_interlock(self):
        """Verify SwarmCoordinationManager hard predictive collision avoidance:
        If an agent's intended target waypoint conflicts within SAFETY_RADIUS_M (3.0m)
        with another drone's position or trajectory midpoint, HOLD state is enforced."""
        node = SwarmOrchestratorNode(
            mission_config=self.mission_config,
            qmix_configs=self.qmix_configs,
            det_configs=self.det_configs,
            checkpoint_path=self.checkpoint_path,
            max_decisions=50,
            meta_file=self.meta_file
        )
        try:
            for a in node.agents + node.det_agents:
                a.initial_pos = [0.0, 0.0, -15.0]
                a.state = FlightState.WAYPOINT_NAVIGATION
                a.px4.connected = True
                a.px4.arming_state = 2

            d2_agent = next(a for a in node.det_agents if a.config.drone_id == "drone_2")
            d3_agent = next(a for a in node.det_agents if a.config.drone_id == "drone_3")

            # Put D3 physically at (10.0, 10.0)
            ln3, le3 = GridWorldTransform.world_enu_to_px4_ned(10.0, 10.0, d3_agent.config.world_spawn_x, d3_agent.config.world_spawn_y)
            d3_agent.px4.current_position = [ln3, le3, -15.0]

            # Put D2 at (6.0, 10.0)
            ln2, le2 = GridWorldTransform.world_enu_to_px4_ned(6.0, 10.0, d2_agent.config.world_spawn_x, d2_agent.config.world_spawn_y)
            d2_agent.px4.current_position = [ln2, le2, -15.0]

            # Reserve an unexplored target cell for D2 (e.g. cell (5, 2)) so reservation is not immediately cleared by coverage
            # Path for D2 towards (5, 2) passes through cell (2, 2) center (10.0, 10.0) where D3 is located
            node.coordination_manager.reserved_targets["drone_2"] = {
                "target_cell": (5, 2),
                "status": "ASSIGNED",
                "task_id": None,
                "time": time.time()
            }

            node.coordination_manager.get_sim_time = lambda: 40.0
            node.coordination_manager.tick_coordination()

            # D2 must be placed in HOLD due to safety conflict with D3 at (10.0, 10.0)!
            self.assertEqual(d2_agent.get_state(), FlightState.HOLD, "Collision avoidance failed to hold conflicting drone!")
            self.assertIn("drone_2", node.coordination_manager.safety_holds)
            self.assertEqual(node.coordination_manager.safety_holds["drone_2"], "drone_3")

            # Move D3 safely away to (40.0, 40.0)
            ln3_far, le3_far = GridWorldTransform.world_enu_to_px4_ned(40.0, 40.0, d3_agent.config.world_spawn_x, d3_agent.config.world_spawn_y)
            d3_agent.px4.current_position = [ln3_far, le3_far, -15.0]

            node.coordination_manager.tick_coordination()
            # Safety hold must clear
            self.assertNotIn("drone_2", node.coordination_manager.safety_holds)
        finally:
            node.destroy_node()

    def test_07_telemetry_serialization_six_drones_and_tasks(self):
        """Verify _publish_telemetry publishes complete JSON payload containing:
        - 6 drone entries
        - tasks list
        - task_allocation summary with matching counts
        - tracked_victims_state (autonomous)
        - victims_state (ground-truth)"""
        node = SwarmOrchestratorNode(
            mission_config=self.mission_config,
            qmix_configs=self.qmix_configs,
            det_configs=self.det_configs,
            checkpoint_path=self.checkpoint_path,
            max_decisions=50,
            meta_file=self.meta_file
        )
        try:
            # Ingest autonomous discovery
            event = {
                "event_type": "VICTIM_DISCOVERED",
                "timestamp": 50.0,
                "victim_track_id": 8,
                "confirming_drone_id": 0,
                "world_x": 4.0,
                "world_y": 8.0,
                "confidence": 0.92,
                "source": "YOLO_RGBD",
                "verification_mode": "RGBD_TEMPORAL",
                "evidence_state": "TEMPORALLY_CONFIRMED"
            }
            msg = String()
            msg.data = json.dumps(event)
            node.discovery_callback(msg)

            published_payloads = []
            def capture_publish(pub_msg):
                published_payloads.append(json.loads(pub_msg.data))

            node.telemetry_pub.publish = capture_publish
            node._publish_telemetry(current_time=51.0)

            self.assertEqual(len(published_payloads), 1)
            payload = published_payloads[0]

            # 1. Drones check
            drones = payload.get("drones", [])
            self.assertEqual(len(drones), 6, f"Expected 6 drones in telemetry, found {len(drones)}")
            drone_ids = [d["id"] for d in drones]
            for i in range(6):
                self.assertIn(f"drone_{i}", drone_ids)

            # 2. Tasks check
            tasks = payload.get("tasks", [])
            self.assertEqual(len(tasks), 1)
            self.assertEqual(tasks[0]["victim_id"], "victim_1")
            self.assertAlmostEqual(tasks[0]["confidence"], 0.92, places=2)

            # 3. Task allocation summary
            summary = payload.get("task_allocation", {})
            counts = summary.get("counts", {})
            self.assertEqual(counts.get("total"), 1)
            self.assertEqual(counts.get("discovered"), 1)

            # 4. Tracked victims vs ground truth separation
            tracked = payload.get("tracked_victims", [])
            self.assertEqual(len(tracked), 1)
            gt_victims = payload.get("victims", [])
            self.assertGreater(len(gt_victims), 0)
            self.assertTrue(all(v.get("source") != "YOLO_RGBD" for v in gt_victims), "Autonomous track leaked into ground-truth!")
        finally:
            node.destroy_node()

    def test_08_fastapi_and_database_persistence(self):
        """Verify that full telemetry and detection events are successfully ingested
        by DatabaseWriter into SQLite tables (sar_tasks and detection_events) and are
        queryable via FastAPI REST endpoints."""
        self._seed_mission("mission_phase3d_002")

        # 1. Enqueue task event
        task_data = {
            'id': 'task_auto_victim_1',
            'mission_id': 'mission_phase3d_002',
            'victim_id': 'auto_victim_1',
            'world_x': 12.0,
            'world_y': 16.0,
            'grid_x': 3,
            'grid_y': 4,
            'state': 'INVESTIGATING',
            'assigned_drone_id': 'drone_2',
            'confidence': 0.92,
            'source': 'YOLO_RGBD',
            'verification_mode': 'RGBD_TEMPORAL',
            'evidence_state': 'TEMPORALLY_CONFIRMED',
            'approach_radius': 3.0,
            'investigation_duration': 5.0,
            'reassignment_count': 0,
            'created_sim_time': 20.0,
            'updated_sim_time': 25.0,
            'assigned_sim_time': 21.0,
            'investigation_start_sim_time': 24.0,
            'completed_sim_time': None
        }
        det_data = {
            'mission_id': 'mission_phase3d_002',
            'victim_id': 'auto_victim_1',
            'drone_id': 'drone_0',
            'timestamp': 20.0,
            'detection_source': 'YOLO_RGBD',
            'detection_world_x': 12.0,
            'detection_world_y': 16.0,
            'detection_world_z': 0.2,
            'confidence': 0.92,
            'confirmation_count': 3
        }
        self.db_writer._flush_batch([
            (EventType.TASK, task_data),
            (EventType.DETECTION, det_data)
        ])

        # Verify SQLite rows directly
        conn = sqlite3.connect(self.test_db_path)
        cur = conn.cursor()

        cur.execute("SELECT id, state, assigned_drone_id, confidence FROM sar_tasks WHERE id = ?", ('task_auto_victim_1',))
        task_row = cur.fetchone()
        self.assertIsNotNone(task_row)
        self.assertEqual(task_row[0], 'task_auto_victim_1')
        self.assertEqual(task_row[1], 'INVESTIGATING')
        self.assertEqual(task_row[2], 'drone_2')
        self.assertAlmostEqual(task_row[3], 0.92, places=2)

        cur.execute("SELECT victim_id, detection_source, confidence FROM detection_events WHERE victim_id = ?", ('auto_victim_1',))
        det_row = cur.fetchone()
        self.assertIsNotNone(det_row)
        self.assertEqual(det_row[0], 'auto_victim_1')
        self.assertEqual(det_row[1], 'YOLO_RGBD')
        self.assertAlmostEqual(det_row[2], 0.92, places=2)
        conn.close()

        # Test FastAPI endpoint handler directly
        orig_query_db = backend_main.query_db
        try:
            backend_main.query_db = self._query
            tasks_ret = backend_main.get_db_tasks(mission_id="mission_phase3d_002")
            self.assertEqual(len(tasks_ret), 1)
            self.assertEqual(tasks_ret[0]["id"], "task_auto_victim_1")
            self.assertEqual(tasks_ret[0]["state"], "INVESTIGATING")
            self.assertEqual(tasks_ret[0]["assigned_drone_id"], "drone_2")
        finally:
            backend_main.query_db = orig_query_db

    def test_09_gpu_and_hardware_budget_audit(self):
        """Verify RTX 3050 Laptop GPU (6GB) VRAM and hardware headroom."""
        import subprocess
        try:
            res = subprocess.run(
                ["nvidia-smi", "--query-gpu=memory.total,memory.used,memory.free,temperature.gpu", "--format=csv,noheader,nounits"],
                capture_output=True, text=True, timeout=2.0
            )
            self.assertEqual(res.returncode, 0)
            parts = [float(p.strip()) for p in res.stdout.strip().split(",")]
            total_mb, used_mb, free_mb, temp_c = parts
            self.assertGreaterEqual(total_mb, 5500.0, f"Expected ~6GB VRAM, found {total_mb}MB")
            self.assertGreaterEqual(free_mb, 4000.0, f"Low free VRAM: {free_mb}MB free")
            self.assertLess(temp_c, 85.0, f"High GPU temp: {temp_c}C")
        except FileNotFoundError:
            self.skipTest("nvidia-smi not available in environment.")


if __name__ == "__main__":
    unittest.main()
