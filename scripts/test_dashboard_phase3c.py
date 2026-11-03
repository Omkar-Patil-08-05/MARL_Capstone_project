#!/usr/bin/env python3
"""
Test Suite for Dashboard Integration with Autonomous SAR Tasks (Phase 3C)
========================================================================
Validates Phase 3C deliverables:
1. SQLite schema support for `sar_tasks` table and indices.
2. Idempotent upsert logic for SAR tasks in `DatabaseWriter`.
3. Idempotent persistence of autonomous multispectral detections in `detection_events`.
4. Strict separation between ground-truth proximity and autonomous detections.
5. Ingestion of `tracked_victims` and `tasks` from telemetry into the database queue.
6. REST API endpoint `/api/db/tasks` querying tasks by mission_id.
7. Live mission snapshot propagation with `tracked_victims`, `tasks`, and `task_allocation`.
"""

import os
import sys
import json
import time
import sqlite3
import tempfile
import unittest

project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

backend_dir = os.path.abspath(os.path.join(project_root, 'dashboard', 'backend'))
if backend_dir not in sys.path:
    sys.path.insert(0, backend_dir)

from database import DatabaseWriter, EventType


class TestDashboardPhase3C(unittest.TestCase):
    def setUp(self):
        # Create a fresh temporary database for isolated testing
        self.temp_dir = tempfile.TemporaryDirectory()
        self.test_db_path = os.path.join(self.temp_dir.name, "test_antigravity.db")
        self.db_writer = DatabaseWriter(db_path=self.test_db_path)
        self.db_writer.start()

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

    def _seed_mission(self, mission_id="test_mission_001"):
        """Seed experiment and mission records to satisfy foreign key constraints."""
        exp_id = f"exp_{mission_id}"
        self.db_writer._flush_batch([
            (EventType.EXPERIMENT, {'id': exp_id, 'name': 'test_exp', 'timestamp': 1000.0, 'environment': 'sim'}),
            (EventType.MISSION_START, {
                'id': mission_id,
                'experiment_id': exp_id,
                'map_id': 'dense_forest',
                'drone_count': 3,
                'victim_count': 5,
                'start_time': 1000.0,
                'status': 'RUNNING'
            })
        ])

    def test_01_sqlite_sar_tasks_table_and_columns(self):
        """Verify sar_tasks table schema and required columns exist."""
        columns = self._query("PRAGMA table_info(sar_tasks)")
        col_names = {c['name'] for c in columns}
        required_cols = {
            'id', 'mission_id', 'victim_id', 'world_x', 'world_y', 'grid_x', 'grid_y',
            'state', 'assigned_drone_id', 'confidence', 'verification_mode',
            'evidence_state', 'created_sim_time', 'updated_sim_time',
            'assigned_sim_time', 'investigation_start_sim_time', 'completed_sim_time',
            'failed_sim_time', 'failure_reason', 'reassignment_count'
        }
        missing = required_cols - col_names
        self.assertEqual(len(missing), 0, f"Missing columns in sar_tasks table: {missing}")

        # Check index existence
        indices = self._query("PRAGMA index_list(sar_tasks)")
        index_names = {idx['name'] for idx in indices}
        self.assertIn("idx_sar_tasks_mission", index_names)

    def test_02_sar_tasks_idempotent_upsert(self):
        """Verify idempotent task upsert without duplicate key crash."""
        mission_id = "test_mission_001"
        self._seed_mission(mission_id)
        task_id = "task_victim_0"

        # 1. Enqueue task creation (DISCOVERED -> ASSIGNED)
        task_data_1 = {
            'id': task_id,
            'mission_id': mission_id,
            'victim_id': 'victim_0',
            'world_x': 24.5,
            'world_y': -12.3,
            'grid_x': 12,
            'grid_y': 8,
            'state': 'ASSIGNED',
            'assigned_drone_id': 'drone_1',
            'confidence': 0.88,
            'verification_mode': 'RGBD_THERMAL',
            'evidence_state': 'TEMPORALLY_CONFIRMED',
            'created_sim_time': 5.0,
            'updated_sim_time': 5.2,
            'assigned_sim_time': 5.2,
            'investigation_start_sim_time': None,
            'completed_sim_time': None,
            'failed_sim_time': None,
            'failure_reason': None,
            'reassignment_count': 0
        }
        self.db_writer._flush_batch([(EventType.TASK, task_data_1)])

        rows = self._query("SELECT * FROM sar_tasks WHERE id = ?", (task_id,))
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]['state'], 'ASSIGNED')
        self.assertEqual(rows[0]['assigned_drone_id'], 'drone_1')
        self.assertEqual(rows[0]['confidence'], 0.88)

        # 2. Update task state to INVESTIGATING with updated sim time
        task_data_2 = {
            'id': task_id,
            'mission_id': mission_id,
            'victim_id': 'victim_0',
            'world_x': 24.5,
            'world_y': -12.3,
            'grid_x': 12,
            'grid_y': 8,
            'state': 'INVESTIGATING',
            'assigned_drone_id': 'drone_1',
            'confidence': 0.94,
            'verification_mode': 'RGBD_THERMAL',
            'evidence_state': 'VERIFIED',
            'created_sim_time': 5.0,
            'updated_sim_time': 8.5,
            'assigned_sim_time': 5.2,
            'investigation_start_sim_time': 8.5,
            'completed_sim_time': None,
            'failed_sim_time': None,
            'failure_reason': None,
            'reassignment_count': 0
        }
        self.db_writer._flush_batch([(EventType.TASK, task_data_2)])

        # Check that it updated in-place without duplicate row creation
        rows_after = self._query("SELECT * FROM sar_tasks WHERE id = ?", (task_id,))
        self.assertEqual(len(rows_after), 1)
        self.assertEqual(rows_after[0]['state'], 'INVESTIGATING')
        self.assertEqual(rows_after[0]['investigation_start_sim_time'], 8.5)
        self.assertEqual(rows_after[0]['confidence'], 0.94)

        # 3. Transition to COMPLETED
        task_data_3 = {
            'id': task_id,
            'mission_id': mission_id,
            'victim_id': 'victim_0',
            'world_x': 24.5,
            'world_y': -12.3,
            'grid_x': 12,
            'grid_y': 8,
            'state': 'COMPLETED',
            'assigned_drone_id': 'drone_1',
            'confidence': 0.96,
            'verification_mode': 'RGBD_THERMAL',
            'evidence_state': 'VERIFIED',
            'created_sim_time': 5.0,
            'updated_sim_time': 11.5,
            'assigned_sim_time': 5.2,
            'investigation_start_sim_time': 8.5,
            'completed_sim_time': 11.5,
            'failed_sim_time': None,
            'failure_reason': None,
            'reassignment_count': 0
        }
        self.db_writer._flush_batch([(EventType.TASK, task_data_3)])

        rows_final = self._query("SELECT * FROM sar_tasks WHERE id = ?", (task_id,))
        self.assertEqual(len(rows_final), 1)
        self.assertEqual(rows_final[0]['state'], 'COMPLETED')
        self.assertEqual(rows_final[0]['completed_sim_time'], 11.5)

    def test_03_autonomous_detection_persistence_and_isolation(self):
        """Verify autonomous detections persist in detection_events with proper source and isolation."""
        mission_id = "test_mission_isolation"
        self._seed_mission(mission_id)

        # Record GT Proximity event
        gt_det = {
            'mission_id': mission_id,
            'victim_id': 'victim_1',
            'drone_id': 'drone_0',
            'timestamp': 10.0,
            'detection_source': 'GT_PROXIMITY',
            'detection_world_x': 10.0,
            'detection_world_y': 15.0,
            'euclidean_distance': 1.5,
            'confidence': 1.0
        }

        # Record Autonomous Perception event for same victim
        auto_det = {
            'mission_id': mission_id,
            'victim_id': 'victim_1',
            'drone_id': 'drone_2',
            'timestamp': 10.5,
            'detection_source': 'multispectral_verifier',
            'detection_world_x': 10.2,
            'detection_world_y': 14.8,
            'detection_world_z': 0.1,
            'euclidean_distance': 0.28,
            'confidence': 0.91,
            'confirmation_count': 3
        }

        self.db_writer._flush_batch([
            (EventType.DETECTION, gt_det),
            (EventType.DETECTION, auto_det)
        ])

        detections = self._query("SELECT * FROM detection_events WHERE mission_id = ?", (mission_id,))
        self.assertEqual(len(detections), 2, "Both GT and autonomous detection events should persist separately")

        sources = {d['detection_source'] for d in detections}
        self.assertIn('GT_PROXIMITY', sources)
        self.assertIn('multispectral_verifier', sources)

        auto_rows = [d for d in detections if d['detection_source'] == 'multispectral_verifier']
        self.assertEqual(len(auto_rows), 1)
        self.assertAlmostEqual(auto_rows[0]['confidence'], 0.91)
        self.assertEqual(auto_rows[0]['drone_id'], 'drone_2')

    def test_04_backend_telemetry_listener_queueing(self):
        """Verify listener_callback extracts tracked_victims and tasks and enqueues them."""
        import main as backend_main

        # Point backend db_writer to test db
        orig_writer = backend_main.db_writer
        backend_main.db_writer = self.db_writer
        backend_main.mission_manager.active_mission_id = "mission_stream_test"
        self._seed_mission("mission_stream_test")

        try:
            telemetry_data = {
                "type": "telemetry",
                "timestamp": 123456789.0,
                "mission": {
                    "coverage": 45.0,
                    "victims_detected": 1,
                    "total_victims": 5,
                    "decision_count": 50,
                    "max_decisions": 300,
                    "safety_overrides": 0
                },
                "drones": [
                    {"id": "drone_0", "state": "SEARCHING", "action": "MOVE_N", "x": 10.0, "y": 20.0, "z": -15.0, "safety_override": False}
                ],
                "victims": [
                    {"id": "victim_0", "state": "DETECTED", "detected": True, "world_x": 10.0, "world_y": 20.0, "detected_by": "drone_0", "detection_time": 12.0}
                ],
                "tracked_victims": [
                    {
                        "id": "victim_0",
                        "world_x": 10.1,
                        "world_y": 20.2,
                        "confidence": 0.93,
                        "detected_by": "drone_0",
                        "source": "multispectral_verifier",
                        "confirmation_count": 4,
                        "sim_time": 12.2
                    }
                ],
                "tasks": [
                    {
                        "task_id": "task_victim_0",
                        "victim_id": "victim_0",
                        "world_x": 10.1,
                        "world_y": 20.2,
                        "grid_x": 5,
                        "grid_y": 10,
                        "state": "ASSIGNED",
                        "assigned_drone_id": "drone_0",
                        "confidence": 0.93,
                        "verification_mode": "RGBD_THERMAL",
                        "created_sim_time": 12.2,
                        "updated_sim_time": 12.2,
                        "assigned_sim_time": 12.2
                    }
                ]
            }

            # Create mock ROS String message
            msg = type('MockRosString', (), {'data': json.dumps(telemetry_data)})()
            # Instantiate TelemetrySubscriber skeleton or call listener_callback
            sub = backend_main.TelemetrySubscriber.__new__(backend_main.TelemetrySubscriber)
            sub.loop = None
            backend_main._LAST_TIMESTAMP = None  # Reset monotonic timestamp check

            import unittest.mock
            with unittest.mock.patch('asyncio.run_coroutine_threadsafe'):
                sub.listener_callback(msg)

            # Allow background worker to process queue and commit batch (>1.0s flush interval)
            time.sleep(1.5)

            # Verify sar_tasks received the task
            task_rows = self._query("SELECT * FROM sar_tasks WHERE mission_id = ?", ("mission_stream_test",))
            self.assertEqual(len(task_rows), 1)
            self.assertEqual(task_rows[0]['id'], "task_victim_0")
            self.assertEqual(task_rows[0]['state'], "ASSIGNED")

            # Verify detection_events received autonomous detection
            det_rows = self._query("SELECT * FROM detection_events WHERE mission_id = ?", ("mission_stream_test",))
            sources = [d['detection_source'] for d in det_rows]
            self.assertIn("multispectral_verifier", sources)
            self.assertIn("GT_PROXIMITY", sources)
        finally:
            backend_main.db_writer = orig_writer
            backend_main.mission_manager.active_mission_id = None

    def test_05_api_db_tasks_query(self):
        """Verify get_db_tasks returns sar_tasks ordered by created_sim_time."""
        import main as backend_main

        self._seed_mission("mission_alpha")
        self._seed_mission("mission_beta")

        # Temporarily mock query_db in main to use our test DB
        def mock_query_db(query, args=(), one=False):
            rows = self._query(query, args)
            return (rows[0] if rows else None) if one else rows

        orig_query_db = backend_main.query_db
        backend_main.query_db = mock_query_db

        try:
            # Seed 2 tasks for mission_alpha, 1 task for mission_beta
            t_alpha_2 = {
                'id': 'task_alpha_2',
                'mission_id': 'mission_alpha',
                'victim_id': 'victim_2',
                'world_x': 50.0,
                'world_y': 50.0,
                'grid_x': 25,
                'grid_y': 25,
                'state': 'DISCOVERED',
                'assigned_drone_id': None,
                'confidence': 0.82,
                'verification_mode': 'RGBD_THERMAL',
                'evidence_state': 'TEMPORALLY_CONFIRMED',
                'created_sim_time': 15.0,
                'updated_sim_time': 15.0,
                'assigned_sim_time': None,
                'investigation_start_sim_time': None,
                'completed_sim_time': None,
                'failed_sim_time': None,
                'failure_reason': None,
                'reassignment_count': 0
            }
            t_alpha_1 = {
                'id': 'task_alpha_1',
                'mission_id': 'mission_alpha',
                'victim_id': 'victim_1',
                'world_x': 10.0,
                'world_y': 10.0,
                'grid_x': 5,
                'grid_y': 5,
                'state': 'COMPLETED',
                'assigned_drone_id': 'drone_0',
                'confidence': 0.95,
                'verification_mode': 'RGBD_THERMAL',
                'evidence_state': 'VERIFIED',
                'created_sim_time': 5.0,
                'updated_sim_time': 9.0,
                'assigned_sim_time': 5.1,
                'investigation_start_sim_time': 6.0,
                'completed_sim_time': 9.0,
                'failed_sim_time': None,
                'failure_reason': None,
                'reassignment_count': 0
            }
            t_beta_1 = {
                'id': 'task_beta_1',
                'mission_id': 'mission_beta',
                'victim_id': 'victim_beta',
                'world_x': 0.0,
                'world_y': 0.0,
                'grid_x': 0,
                'grid_y': 0,
                'state': 'ASSIGNED',
                'assigned_drone_id': 'drone_1',
                'confidence': 0.9,
                'verification_mode': 'RGBD_THERMAL',
                'evidence_state': 'TEMPORALLY_CONFIRMED',
                'created_sim_time': 1.0,
                'updated_sim_time': 1.0,
                'assigned_sim_time': 1.0,
                'investigation_start_sim_time': None,
                'completed_sim_time': None,
                'failed_sim_time': None,
                'failure_reason': None,
                'reassignment_count': 0
            }
            self.db_writer._flush_batch([
                (EventType.TASK, t_alpha_2),
                (EventType.TASK, t_alpha_1),
                (EventType.TASK, t_beta_1)
            ])

            # Call get_db_tasks endpoint handler
            res_alpha = backend_main.get_db_tasks(mission_id="mission_alpha")
            self.assertEqual(len(res_alpha), 2)
            # Verify ordering by created_sim_time ASC (task_alpha_1 created at 5.0 before task_alpha_2 at 15.0)
            self.assertEqual(res_alpha[0]['id'], 'task_alpha_1')
            self.assertEqual(res_alpha[1]['id'], 'task_alpha_2')

            res_beta = backend_main.get_db_tasks(mission_id="mission_beta")
            self.assertEqual(len(res_beta), 1)
            self.assertEqual(res_beta[0]['id'], 'task_beta_1')
        finally:
            backend_main.query_db = orig_query_db

    def test_06_live_mission_snapshot_integration(self):
        """Verify get_live_mission_snapshot unpacks tasks, tracked_victims, and task_allocation."""
        import main as backend_main

        backend_main.mission_manager.state = "RUNNING"
        backend_main.mission_manager.active_mission_id = "live_snap_test"
        backend_main.mission_manager.active_map_id = "dense_forest"
        backend_main.mission_manager.drone_count = 3
        backend_main.mission_manager.mission_start_time = 1000.0

        sample_telem = {
            "type": "telemetry",
            "timestamp": 1050.0,
            "mission": {
                "coverage": 62.5,
                "victims_detected": 2,
                "total_victims": 5,
                "decision_count": 80,
                "max_decisions": 300,
                "safety_overrides": 1,
                "rtf": 1.0
            },
            "drones": [
                {"id": "drone_0", "state": "SEARCHING", "action": "MOVE_E", "x": 15.0, "y": 25.0, "z": -15.0, "safety_override": False}
            ],
            "victims": [
                {"id": "victim_0", "state": "DETECTED", "world_x": 15.0, "world_y": 25.0, "detected_by": "drone_0"}
            ],
            "tracked_victims": [
                {
                    "id": "victim_0",
                    "world_x": 15.1,
                    "world_y": 25.1,
                    "confidence": 0.95,
                    "detected_by": "drone_0",
                    "source": "multispectral_verifier",
                    "verification_mode": "RGBD_THERMAL",
                    "confirmation_count": 3
                }
            ],
            "tasks": [
                {
                    "task_id": "task_victim_0",
                    "victim_id": "victim_0",
                    "world_x": 15.1,
                    "world_y": 25.1,
                    "state": "INVESTIGATING",
                    "assigned_drone_id": "drone_0",
                    "confidence": 0.95,
                    "investigation_progress_sec": 1.5,
                    "investigation_duration": 3.0
                }
            ],
            "task_allocation": {
                "total_tasks": 1,
                "active_tasks": 1,
                "completed_tasks": 0,
                "failed_tasks": 0,
                "state_counts": {"INVESTIGATING": 1},
                "active_assignments": {"drone_0": "task_victim_0"}
            },
            "coordination": {
                "active_frontiers": {"drone_1": [20, 30]},
                "safety_holds": {},
                "qmix_drones": 2,
                "coord_drones": 1,
                "task_assignments": {"drone_0": "task_victim_0"}
            }
        }

        orig_telem = backend_main._LATEST_TELEMETRY
        backend_main._LATEST_TELEMETRY = json.dumps(sample_telem)

        try:
            snapshot = backend_main.get_live_mission_snapshot()
            self.assertTrue(snapshot["active"])
            self.assertEqual(snapshot["mission_id"], "live_snap_test")
            self.assertIn("tracked_victims", snapshot)
            self.assertEqual(len(snapshot["tracked_victims"]), 1)
            self.assertEqual(snapshot["tracked_victims"][0]["id"], "victim_0")
            self.assertEqual(snapshot["tracked_victims"][0]["verification_mode"], "RGBD_THERMAL")

            self.assertIn("tasks", snapshot)
            self.assertEqual(len(snapshot["tasks"]), 1)
            self.assertEqual(snapshot["tasks"][0]["task_id"], "task_victim_0")
            self.assertEqual(snapshot["tasks"][0]["state"], "INVESTIGATING")

            self.assertIn("task_allocation", snapshot)
            self.assertEqual(snapshot["task_allocation"]["active_tasks"], 1)

            self.assertIn("coordination", snapshot)
            self.assertEqual(snapshot["coordination"]["task_assignments"]["drone_0"], "task_victim_0")
        finally:
            backend_main._LATEST_TELEMETRY = orig_telem
            backend_main.mission_manager.state = "IDLE"
            backend_main.mission_manager.active_mission_id = None


if __name__ == '__main__':
    unittest.main()
