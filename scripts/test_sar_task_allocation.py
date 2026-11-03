#!/usr/bin/env python3
"""
Test Suite for SAR Task Allocation & Mission Coordination (Phase 3B)
=====================================================================
Validates all Phase 3B requirements:
1. Discovery-to-task creation
2. Duplicate victim suppression
3. One-task / one-drone assignment enforcement
4. Multi-drone assignment across 6 drones
5. Unavailable drone handling (failsafe, disconnect)
6. Safe task reassignment
7. Task state transitions (DISCOVERED -> ASSIGNED -> APPROACHING -> INVESTIGATING -> COMPLETED, and FAILED)
8. Ground-truth and autonomous victim isolation
9. Telemetry serialization and JSON schema compliance
10. Simulation-time consistency
11. QMIXMissionController telemetry integration
"""

import os
import sys
import math
import json
import unittest

# Ensure workspace source has precedence over any stale system paths
ws_src = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'drone_ws', 'src', 'swarm_controller'))
if ws_src not in sys.path:
    sys.path.insert(0, ws_src)

project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

# Ensure ROS 2 and px4_msgs can be imported when running standalone
ros2_paths = [
    '/opt/ros/jazzy/lib/python3.12/site-packages',
    os.path.abspath(os.path.join(os.path.dirname(__file__), '../drone_ws/install/px4_msgs/lib/python3.12/site-packages')),
]
for p in ros2_paths:
    if os.path.exists(p) and p not in sys.path:
        sys.path.append(p)

from swarm_controller.sar_task_allocator import SARTaskAllocator, SARTask, TaskState
from swarm_controller.victim_manager import VictimManager, TrackedVictim
from swarm_controller.grid_world_transform import GridWorldTransform
from swarm_controller.configs import DroneConfig, MissionConfig
from marl_drone_project.env.sar_env import SARGridEnv


class MockPX4:
    def __init__(self, pos=(0.0, 0.0, -15.0), vel=(0.0, 0.0, 0.0)):
        self.current_position = list(pos)
        self.current_velocity = list(vel)

    def is_connected(self):
        return True

    def is_offboard(self):
        return True

    def is_estimator_ready(self):
        return True

    def publish_vehicle_command(self, cmd):
        pass


class MockDroneAgent:
    def __init__(self, drone_id: str, spawn_x: float = 0.0, spawn_y: float = 0.0):
        self.config = DroneConfig(
            drone_id=drone_id,
            namespace=drone_id,
            system_id=1,
            world_spawn_x=spawn_x,
            world_spawn_y=spawn_y,
            world_spawn_z=0.2,
            world_yaw=0.0
        )
        self.mission = MissionConfig()
        self.px4 = MockPX4()
        from swarm_controller.drone_agent import FlightState
        self.state = FlightState.HOLD
        self.initial_pos = [0.0, 0.0, -15.0]
        self.mission_goal_local = [0.0, 0.0, -15.0]

    def get_state(self):
        return self.state

    def _log_transition(self, new_state):
        self.state = new_state

    def get_current_sim_time(self):
        return 50.0

    def set_mission_goal_local(self, n, e, d, require_min_dwell=False):
        self.mission_goal_local = [n, e, d]


class MockNode:
    def __init__(self):
        self._logs = []

    def get_logger(self):
        return self

    def info(self, msg):
        self._logs.append(("INFO", msg))

    def warn(self, msg):
        self._logs.append(("WARN", msg))

    def error(self, msg):
        self._logs.append(("ERROR", msg))


class TestSARTaskAllocation(unittest.TestCase):

    def setUp(self):
        GridWorldTransform.set_bounds(width=25, height=25, meters_per_cell=4.0, origin_x=0.0, origin_y=0.0)
        self.mock_node = MockNode()
        self.allocator = SARTaskAllocator(
            node=self.mock_node,
            approach_radius=3.0,
            investigation_duration=5.0,
            max_reassignments=3
        )

    def test_01_discovery_to_task_creation(self):
        """Test that autonomous TrackedVictims are correctly converted into SARTasks."""
        tracked = {
            "track_0": TrackedVictim(
                t_id="track_0",
                world_x=20.0,
                world_y=40.0,
                source="multispectral_verifier",
                confidence=0.92,
                drone_id=2,
                sim_time=10.5,
                verification_mode="MULTISPECTRAL_RGB_THERMAL",
                evidence_state="TEMPORALLY_CONFIRMED"
            )
        }

        new_ids = self.allocator.sync_from_tracked_victims(tracked, sim_time=10.5)
        self.assertEqual(len(new_ids), 1)
        self.assertEqual(new_ids[0], "task_track_0")

        self.assertIn("task_track_0", self.allocator.tasks)
        task = self.allocator.tasks["task_track_0"]
        self.assertEqual(task.victim_id, "track_0")
        self.assertAlmostEqual(task.world_x, 20.0, places=2)
        self.assertAlmostEqual(task.world_y, 40.0, places=2)
        self.assertEqual(task.state, TaskState.DISCOVERED)
        self.assertIsNone(task.assigned_drone_id)
        self.assertAlmostEqual(task.confidence, 0.92, places=2)
        self.assertEqual(task.verification_mode, "MULTISPECTRAL_RGB_THERMAL")
        self.assertEqual(task.evidence_state, "TEMPORALLY_CONFIRMED")
        self.assertAlmostEqual(task.created_sim_time, 10.5, places=2)

    def test_02_duplicate_victim_suppression(self):
        """Test that re-observations of the same victim update existing task without creating duplicate tasks."""
        track = TrackedVictim(
            t_id="track_0",
            world_x=20.0,
            world_y=40.0,
            confidence=0.85,
            drone_id=2,
            sim_time=10.0
        )
        tracked = {"track_0": track}
        new_ids1 = self.allocator.sync_from_tracked_victims(tracked, sim_time=10.0)
        self.assertEqual(len(new_ids1), 1)
        self.assertEqual(len(self.allocator.tasks), 1)

        # Re-observation from another drone with smoothed position
        track.update(
            world_x=21.0,
            world_y=41.0,
            confidence=0.95,
            drone_id=3,
            sim_time=12.0
        )
        new_ids2 = self.allocator.sync_from_tracked_victims(tracked, sim_time=12.0)
        
        # No new tasks should be created
        self.assertEqual(len(new_ids2), 0)
        self.assertEqual(len(self.allocator.tasks), 1)
        
        # Existing task should have updated coordinates and confidence
        task = self.allocator.tasks["task_track_0"]
        self.assertAlmostEqual(task.world_x, track.world_x, places=2)
        self.assertAlmostEqual(task.world_y, track.world_y, places=2)
        self.assertAlmostEqual(task.confidence, 0.95, places=2)
        self.assertAlmostEqual(task.updated_sim_time, 12.0, places=2)

    def test_03_one_task_one_drone_enforcement(self):
        """Test that a drone can only be assigned to one task, and a task can only have one drone."""
        tracked = {
            "track_0": TrackedVictim("track_0", 10.0, 10.0, sim_time=5.0),
            "track_1": TrackedVictim("track_1", 80.0, 80.0, sim_time=5.0)
        }
        self.allocator.sync_from_tracked_victims(tracked, sim_time=5.0)
        self.assertEqual(len(self.allocator.tasks), 2)

        # Only 1 drone available
        available_drones = {"drone_2": (12.0, 12.0)}
        assigned = self.allocator.allocate_tasks(available_drones, sim_time=6.0)

        self.assertEqual(len(assigned), 1)
        self.assertIn("drone_2", assigned)
        assigned_task_id = assigned["drone_2"]
        self.assertEqual(assigned_task_id, "task_track_0")

        # Verify task states
        self.assertEqual(self.allocator.tasks["task_track_0"].state, TaskState.ASSIGNED)
        self.assertEqual(self.allocator.tasks["task_track_0"].assigned_drone_id, "drone_2")
        self.assertEqual(self.allocator.tasks["task_track_1"].state, TaskState.DISCOVERED)
        self.assertIsNone(self.allocator.tasks["task_track_1"].assigned_drone_id)

        # Attempt to allocate again with drone_2: should NOT assign second task
        assigned2 = self.allocator.allocate_tasks(available_drones, sim_time=7.0)
        self.assertEqual(len(assigned2), 0)
        self.assertEqual(self.allocator.tasks["task_track_1"].state, TaskState.DISCOVERED)

    def test_04_multi_drone_assignment_six_drone_swarm(self):
        """Test distance-based task allocation across a 6-drone swarm (D2..D5 available)."""
        tracked = {
            "track_0": TrackedVictim("track_0", 10.0, 10.0, sim_time=1.0),
            "track_1": TrackedVictim("track_1", 50.0, 50.0, sim_time=1.0),
            "track_2": TrackedVictim("track_2", 80.0, 20.0, sim_time=1.0)
        }
        self.allocator.sync_from_tracked_victims(tracked, sim_time=1.0)

        # Available coordination drones in a 6-drone swarm
        available_drones = {
            "drone_2": (8.0, 8.0),    # Closest to track_0
            "drone_3": (48.0, 48.0),  # Closest to track_1
            "drone_4": (82.0, 22.0),  # Closest to track_2
            "drone_5": (0.0, 80.0)    # Idle / unassigned
        }

        assignments = self.allocator.allocate_tasks(available_drones, sim_time=2.0)
        self.assertEqual(len(assignments), 3)

        self.assertEqual(assignments["drone_2"], "task_track_0")
        self.assertEqual(assignments["drone_3"], "task_track_1")
        self.assertEqual(assignments["drone_4"], "task_track_2")
        self.assertNotIn("drone_5", assignments)

        # All 3 tasks must be ASSIGNED to distinct drones
        assigned_drones = [self.allocator.tasks[f"task_track_{i}"].assigned_drone_id for i in range(3)]
        self.assertEqual(len(set(assigned_drones)), 3)

    def test_05_complete_lifecycle_transitions(self):
        """Test full state progression: DISCOVERED -> ASSIGNED -> APPROACHING -> INVESTIGATING -> COMPLETED."""
        tracked = {"track_0": TrackedVictim("track_0", 30.0, 30.0, sim_time=10.0)}
        self.allocator.sync_from_tracked_victims(tracked, sim_time=10.0)
        task = self.allocator.tasks["task_track_0"]
        self.assertEqual(task.state, TaskState.DISCOVERED)

        # 1. Allocate -> ASSIGNED
        self.allocator.allocate_tasks({"drone_2": (10.0, 10.0)}, sim_time=11.0)
        self.assertEqual(task.state, TaskState.ASSIGNED)
        self.assertEqual(task.assigned_sim_time, 11.0)

        # 2. Drone moves -> APPROACHING
        self.allocator.update_task_lifecycle(
            drone_positions={"drone_2": (15.0, 15.0)},
            drone_states={"drone_2": "WAYPOINT_NAVIGATION"},
            sim_time=12.0
        )
        self.assertEqual(task.state, TaskState.APPROACHING)

        # 3. Drone arrives within approach radius (3.0m) -> INVESTIGATING
        self.allocator.update_task_lifecycle(
            drone_positions={"drone_2": (29.0, 29.5)},  # dist = sqrt(1^2 + 0.5^2) = 1.11m <= 3.0m
            drone_states={"drone_2": "WAYPOINT_NAVIGATION"},
            sim_time=15.0
        )
        self.assertEqual(task.state, TaskState.INVESTIGATING)
        self.assertEqual(task.investigation_start_sim_time, 15.0)

        # 4. Drone dwells for 2.0s (< 5.0s duration) -> still INVESTIGATING
        self.allocator.update_task_lifecycle(
            drone_positions={"drone_2": (29.0, 29.5)},
            drone_states={"drone_2": "HOLD"},
            sim_time=17.0
        )
        self.assertEqual(task.state, TaskState.INVESTIGATING)
        self.assertAlmostEqual(task.investigation_progress_sec, 2.0, places=2)

        # 5. Dwell completes (5.5s >= 5.0s duration) -> COMPLETED
        self.allocator.update_task_lifecycle(
            drone_positions={"drone_2": (29.0, 29.5)},
            drone_states={"drone_2": "HOLD"},
            sim_time=20.5
        )
        self.assertEqual(task.state, TaskState.COMPLETED)
        self.assertEqual(task.completed_sim_time, 20.5)

        # Assigned drone should be released
        self.assertNotIn("drone_2", self.allocator.drone_assignments)

    def test_06_unavailable_drone_handling_and_safe_reassignment(self):
        """Test that drone failsafe/disconnect safely resets task to DISCOVERED and allows reassignment."""
        tracked = {"track_0": TrackedVictim("track_0", 40.0, 40.0, sim_time=10.0)}
        self.allocator.sync_from_tracked_victims(tracked, sim_time=10.0)
        self.allocator.allocate_tasks({"drone_2": (30.0, 30.0)}, sim_time=11.0)
        task = self.allocator.tasks["task_track_0"]
        self.assertEqual(task.state, TaskState.ASSIGNED)
        self.assertEqual(task.assigned_drone_id, "drone_2")

        # Drone 2 enters FAILSAFE
        self.allocator.update_task_lifecycle(
            drone_positions={"drone_2": (30.0, 30.0)},
            drone_states={"drone_2": "FAILSAFE"},
            sim_time=12.0
        )

        # Task should be safely reset to DISCOVERED with reassignment count = 1
        self.assertEqual(task.state, TaskState.DISCOVERED)
        self.assertIsNone(task.assigned_drone_id)
        self.assertEqual(task.reassignment_count, 1)
        self.assertNotIn("drone_2", self.allocator.drone_assignments)

        # Another healthy drone (drone_3) is available
        new_assignments = self.allocator.allocate_tasks({"drone_3": (35.0, 35.0)}, sim_time=13.0)
        self.assertEqual(len(new_assignments), 1)
        self.assertEqual(new_assignments["drone_3"], "task_track_0")
        self.assertEqual(task.state, TaskState.ASSIGNED)
        self.assertEqual(task.assigned_drone_id, "drone_3")

    def test_07_exceeded_max_reassignments_transitions_to_failed(self):
        """Test that task transitions to FAILED when exceeding max reassignments."""
        tracked = {"track_0": TrackedVictim("track_0", 40.0, 40.0, sim_time=10.0)}
        self.allocator.sync_from_tracked_victims(tracked, sim_time=10.0)
        task = self.allocator.tasks["task_track_0"]

        # Fail drone 3 times (max_reassignments = 3)
        for i in range(1, 4):
            drone_id = f"drone_{i}"
            self.allocator.allocate_tasks({drone_id: (0.0, 0.0)}, sim_time=10.0 + i)
            self.assertEqual(task.assigned_drone_id, drone_id)
            self.allocator.update_task_lifecycle(
                drone_positions={drone_id: (0.0, 0.0)},
                drone_states={drone_id: "FAILSAFE"},
                sim_time=10.0 + i + 0.5
            )
            self.assertEqual(task.reassignment_count, i)

        # 4th failure must exceed max_reassignments -> FAILED
        drone_id = "drone_4"
        self.allocator.allocate_tasks({drone_id: (0.0, 0.0)}, sim_time=15.0)
        self.allocator.update_task_lifecycle(
            drone_positions={drone_id: (0.0, 0.0)},
            drone_states={drone_id: "FAILSAFE"},
            sim_time=16.0
        )
        self.assertEqual(task.state, TaskState.FAILED)
        self.assertEqual(task.failure_reason, "EXCEEDED_MAX_REASSIGNMENTS")

    def test_08_ground_truth_and_autonomous_victim_isolation(self):
        """Verify strict isolation: ground-truth victims are never ingested into SARTaskAllocator."""
        env = SARGridEnv(num_drones=2, max_steps=100)
        meta = {
            "victims": [
                {"id": "gt_victim_0", "grid": {"x": 2, "y": 2}, "world": {"x": 10.0, "y": 10.0, "z": 0.1}},
                {"id": "gt_victim_1", "grid": {"x": 5, "y": 5}, "world": {"x": 20.0, "y": 20.0, "z": 0.1}},
                {"id": "gt_victim_2", "grid": {"x": 7, "y": 7}, "world": {"x": 30.0, "y": 30.0, "z": 0.1}}
            ],
            "obstacles": []
        }
        vm = VictimManager(self.mock_node, meta, "realistic_sar", env, seed=42)

        # 3 ground truth victims exist
        self.assertEqual(len(vm.victims), 3)
        self.assertEqual(len(vm.tracked_victims), 0)

        # Sync task allocator from tracked_victims
        self.allocator.sync_from_tracked_victims(vm.tracked_victims, sim_time=10.0)

        # MUST BE 0 tasks created! Ground-truth victims are never ingested!
        self.assertEqual(len(self.allocator.tasks), 0)

        # Now simulate real perception discovery
        vm.process_discovery_event(
            world_x=15.0,
            world_y=25.0,
            source="multispectral_verifier",
            confidence=0.98,
            drone_id=2,
            sim_time=15.0,
            source_track_id=1,
            verification_mode="MULTISPECTRAL_RGB_THERMAL"
        )
        self.assertEqual(len(vm.tracked_victims), 1)

        # Sync again
        self.allocator.sync_from_tracked_victims(vm.tracked_victims, sim_time=15.0)

        # Exactly 1 autonomous task created
        self.assertEqual(len(self.allocator.tasks), 1)
        self.assertIn("task_victim_1", self.allocator.tasks)

    def test_09_telemetry_serialization_and_json_compliance(self):
        """Test telemetry formatting, JSON serialization, and counts structure."""
        tracked = {
            "track_0": TrackedVictim("track_0", 10.0, 10.0, sim_time=1.0),
            "track_1": TrackedVictim("track_1", 20.0, 20.0, sim_time=2.0),
            "track_2": TrackedVictim("track_2", 30.0, 30.0, sim_time=3.0)
        }
        self.allocator.sync_from_tracked_victims(tracked, sim_time=3.0)

        # Allocate track_0
        self.allocator.allocate_tasks({"drone_2": (10.0, 10.0)}, sim_time=4.0)

        # Complete track_0
        self.allocator.tasks["task_track_0"].transition_to(TaskState.APPROACHING, 4.5)
        self.allocator.tasks["task_track_0"].transition_to(TaskState.INVESTIGATING, 5.0)
        self.allocator.tasks["task_track_0"].transition_to(TaskState.COMPLETED, 10.5)

        # Allocate track_1
        self.allocator.allocate_tasks({"drone_3": (20.0, 20.0)}, sim_time=11.0)

        telem = self.allocator.get_telemetry()

        # Must be valid JSON
        json_str = json.dumps(telem)
        parsed = json.loads(json_str)

        self.assertIn("tasks", parsed)
        self.assertIn("assignments", parsed)
        self.assertIn("counts", parsed)

        self.assertEqual(parsed["counts"]["total"], 3)
        self.assertEqual(parsed["counts"]["completed"], 1)
        self.assertEqual(parsed["counts"]["assigned"], 1)
        self.assertEqual(parsed["counts"]["discovered"], 1)

    def test_10_simulation_time_consistency(self):
        """Test that task progression respects simulation time intervals."""
        tracked = {"track_0": TrackedVictim("track_0", 10.0, 10.0, sim_time=100.0)}
        self.allocator.sync_from_tracked_victims(tracked, sim_time=100.0)
        self.allocator.allocate_tasks({"drone_2": (10.0, 10.0)}, sim_time=102.0)
        
        # Start investigating at sim_time = 110.0
        self.allocator.tasks["task_track_0"].transition_to(TaskState.APPROACHING, 105.0)
        self.allocator.tasks["task_track_0"].transition_to(TaskState.INVESTIGATING, 110.0)
        task = self.allocator.tasks["task_track_0"]

        # 4 seconds later (sim_time = 114.0): not completed yet (duration = 5.0)
        self.allocator.update_task_lifecycle(
            drone_positions={"drone_2": (10.0, 10.0)},
            drone_states={"drone_2": "HOLD"},
            sim_time=114.0
        )
        self.assertEqual(task.state, TaskState.INVESTIGATING)

        # 5.1 seconds later (sim_time = 115.1): completed
        self.allocator.update_task_lifecycle(
            drone_positions={"drone_2": (10.0, 10.0)},
            drone_states={"drone_2": "HOLD"},
            sim_time=115.1
        )
        self.assertEqual(task.state, TaskState.COMPLETED)
        self.assertEqual(task.completed_sim_time, 115.1)

    def test_11_qmix_mission_controller_telemetry_integration(self):
        """Test that QMIXMissionController correctly integrates task allocation in telemetry."""
        from swarm_controller.qmix_mission_controller import QMIXMissionController
        from swarm_controller.drone_agent import FlightState
        from std_msgs.msg import String

        class MockPub:
            def __init__(self):
                self.published_msgs = []
            def publish(self, msg):
                self.published_msgs.append(msg)

        class MockAgent:
            def __init__(self, drone_id):
                self.config = DroneConfig(
                    drone_id=drone_id,
                    namespace=drone_id,
                    system_id=1,
                    world_spawn_x=10.0,
                    world_spawn_y=20.0,
                    world_spawn_z=0.2,
                    world_yaw=0.0
                )
                self.state = FlightState.HOLD
                self.px4 = MockPX4(pos=[1.0, 2.0, -15.0], vel=[0.1, 0.2, 0.0])

            def get_current_sim_time(self):
                return 26.0

        class MockController:
            def __init__(self):
                self.victim_manager = VictimManager(
                    node=MockNode(),
                    metadata={"victims": []},
                    world_name="test_world",
                    env=SARGridEnv(num_drones=1, max_steps=100),
                    seed=42
                )
                self.task_allocator = SARTaskAllocator(node=MockNode())
                self._logger = MockNode()
                self.mission_active = True
                self.land_commanded = False
                self.decisions_made = 2
                self.max_decisions = 30
                self.safety_overrides_count = 0
                self.agents = [MockAgent("drone_0")]
                self.env = self.victim_manager.env
                self.last_actions = {"drone_0": "+X"}
                self.last_safety_overrides = {"drone_0": False}
                self.telemetry_pub = MockPub()

            def get_logger(self):
                return self._logger

        controller = MockController()

        # 1. Test discovery_callback ingests discovery and updates task_allocator
        event_payload = json.dumps({
            "event_type": "VICTIM_DISCOVERED",
            "timestamp": 25.0,
            "victim_track_id": 1,
            "confirming_drone_id": 2,
            "world_x": 18.5,
            "world_y": 28.5,
            "confidence": 0.94,
            "verification_mode": "MULTISPECTRAL_RGB_THERMAL",
            "evidence_state": "TEMPORALLY_CONFIRMED"
        })
        msg = String()
        msg.data = event_payload

        QMIXMissionController.discovery_callback(controller, msg)

        # Verify task was created in controller.task_allocator
        self.assertEqual(len(controller.task_allocator.tasks), 1)
        self.assertIn("task_victim_1", controller.task_allocator.tasks)

        # 2. Test _publish_telemetry formats and publishes tasks & task_allocation
        QMIXMissionController._publish_telemetry(controller, current_time=26.0)

        self.assertEqual(len(controller.telemetry_pub.published_msgs), 1)
        telem = json.loads(controller.telemetry_pub.published_msgs[0].data)

        # Assert telemetry contracts
        self.assertIn("task_allocation", telem)
        self.assertIn("tasks", telem)
        self.assertIn("coordination", telem)
        self.assertIn("task_assignments", telem["coordination"])

        self.assertEqual(telem["task_allocation"]["counts"]["total"], 1)
        self.assertEqual(len(telem["tasks"]), 1)
        self.assertEqual(telem["tasks"][0]["task_id"], "task_victim_1")
        self.assertAlmostEqual(telem["tasks"][0]["world_x"], 18.5, places=2)

    def test_12_swarm_coordination_manager_task_execution(self):
        """Test that SwarmCoordinationManager steers deterministic drones toward autonomous tasks."""
        from swarm_controller.swarm_coordination_manager import SwarmCoordinationManager
        from swarm_controller.drone_agent import FlightState

        class MockCoordOrchestrator:
            def __init__(self):
                self.env = SARGridEnv(num_drones=3, max_steps=100)
                self.env.reset()
                self.agents = [
                    MockDroneAgent("drone_0", 0.0, 0.0),
                    MockDroneAgent("drone_1", 4.0, 4.0)
                ]
                self.det_agents = [
                    MockDroneAgent("drone_2", 8.0, 8.0)
                ]
                for a in self.agents + self.det_agents:
                    a.state = FlightState.HOLD
                    a.initial_pos = [0.0, 0.0, -15.0]

                self.victim_manager = VictimManager(
                    node=MockNode(),
                    metadata={"victims": []},
                    world_name="test_world",
                    env=self.env,
                    seed=42
                )
                self.task_allocator = SARTaskAllocator(node=MockNode(), approach_radius=3.0, investigation_duration=2.0)
                self._logger = MockNode()
                self._sim_time = 100.0

            def get_logger(self):
                return self._logger

        orchestrator = MockCoordOrchestrator()
        coord_mgr = SwarmCoordinationManager(orchestrator)

        # 1. Simulate autonomous perception discovery of victim at (20.0, 20.0) -> grid (5, 5)
        orchestrator.victim_manager.process_discovery_event(
            world_x=20.0,
            world_y=20.0,
            source="multispectral_verifier",
            confidence=0.96,
            drone_id=2,
            sim_time=100.0,
            source_track_id=1,
            verification_mode="MULTISPECTRAL_RGB_THERMAL"
        )

        # 2. Run coordination tick
        coord_mgr.tick_coordination()

        # Verify task is created and assigned to drone_2
        self.assertEqual(len(orchestrator.task_allocator.tasks), 1)
        task = orchestrator.task_allocator.tasks["task_victim_1"]
        self.assertIn(task.state, (TaskState.ASSIGNED, TaskState.APPROACHING))
        self.assertEqual(task.assigned_drone_id, "drone_2")
        self.assertIn("drone_2", coord_mgr.reserved_targets)
        self.assertEqual(coord_mgr.reserved_targets["drone_2"]["task_id"], "task_victim_1")

        # 3. Simulate drone arrival at target -> tick again
        d2 = orchestrator.det_agents[0]
        # Set drone physical position near victim (dist < 3.0m)
        d2.px4.current_position = [20.0 - 8.0, 20.0 - 8.0, -15.0]  # ENU -> NED conversion matches world (20.0, 20.0)
        orchestrator.task_allocator.update_task_lifecycle(
            drone_positions={"drone_2": (20.0, 20.0)},
            drone_states={"drone_2": "HOLD"},
            sim_time=105.0
        )
        self.assertEqual(task.state, TaskState.INVESTIGATING)

        # 4. Advance time past investigation duration -> completes task
        orchestrator.task_allocator.update_task_lifecycle(
            drone_positions={"drone_2": (20.0, 20.0)},
            drone_states={"drone_2": "HOLD"},
            sim_time=108.0
        )
        self.assertEqual(task.state, TaskState.COMPLETED)
        self.assertNotIn("drone_2", orchestrator.task_allocator.drone_assignments)

        # 5. Run coordination tick after completion -> drone resumes frontier exploration without holding completed task
        coord_mgr.tick_coordination()
        if "drone_2" in coord_mgr.reserved_targets:
            self.assertNotEqual(coord_mgr.reserved_targets["drone_2"].get("task_id"), "task_victim_1")


def main():
    suite = unittest.TestLoader().loadTestsFromTestCase(TestSARTaskAllocation)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)


if __name__ == '__main__':
    main()
