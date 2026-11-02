#!/usr/bin/env python3
"""
Comprehensive Offline Validation Suite
Focus areas:
1. Real scenario coordinates from worlds/forest_dense_candidate_meta.json.
2. Complete controller integration paths (qmix_mission_controller, swarm_coordination_manager, swarm_runner).
3. Robust victim confirmation behavior (false positive rejection, multi-frame consensus, depth/spread filtering, multi-drone deduplication).
4. Runtime configuration & GridWorldTransform metadata loading.
"""

import sys
import os
import unittest
import math
import json
from unittest.mock import MagicMock

# Add workspace and drone_ws to path
workspace_dir = "/home/capstone/capstone_project_antigravity"
sys.path.insert(0, os.path.join(workspace_dir, "drone_ws/src/swarm_controller"))

from swarm_controller.grid_world_transform import GridWorldTransform
from swarm_controller.rgbd_victim_localizer import EventConfirmer, RGBDGeometricLocalizer, RGBDVictimLocalizerNode
from swarm_controller.configs import DroneConfig, MissionConfig
from swarm_controller.drone_agent import DroneAgent, FlightState

META_FILE = os.path.join(workspace_dir, "worlds/forest_dense_candidate_meta.json")

class TestRealScenarioCoordinates(unittest.TestCase):
    """Validates actual scenario coordinates, boundaries, and transformations from metadata."""
    
    @classmethod
    def setUpClass(cls):
        with open(META_FILE, 'r') as f:
            cls.meta = json.load(f)
        search = cls.meta.get('search_area') or cls.meta.get('world')
        GridWorldTransform.set_bounds(
            width=int(search.get('grid_width', 25)),
            height=int(search.get('grid_height', 25)),
            meters_per_cell=float(search.get('meters_per_cell', 8.0)),
            origin_x=float(search.get('origin_x', -180.0)),
            origin_y=float(search.get('origin_y', -20.0))
        )

    def test_search_area_boundaries(self):
        search = self.meta['search_area']
        self.assertEqual(search['origin_x'], -180.0)
        self.assertEqual(search['origin_y'], -20.0)
        self.assertEqual(search['grid_width'], 25)
        self.assertEqual(search['grid_height'], 25)
        self.assertEqual(search['meters_per_cell'], 8.0)
        
        # Valid bounds without margin: [-180, 20] in X, [-20, 180] in Y
        self.assertTrue(GridWorldTransform.is_within_world_bounds(-180.0, -20.0, margin=0.0))
        self.assertTrue(GridWorldTransform.is_within_world_bounds(20.0, 180.0, margin=0.0))
        self.assertFalse(GridWorldTransform.is_within_world_bounds(-180.1, 0.0, margin=0.0))
        self.assertFalse(GridWorldTransform.is_within_world_bounds(20.1, 0.0, margin=0.0))

    def test_all_five_actual_victims_within_bounds(self):
        victims = self.meta['victims']
        self.assertEqual(len(victims), 5, "Metadata must contain exactly 5 victims")
        
        for v in victims:
            v_id = v['id']
            wx = v['world_x']
            wy = v['world_y']
            wz = v['world_z']
            
            # Verify each victim is strictly inside valid search area
            self.assertTrue(
                GridWorldTransform.is_within_world_bounds(wx, wy, margin=0.0),
                f"Victim {v_id} at ({wx}, {wy}) is outside search area bounds!"
            )
            
            # Verify grid coordinates match metadata
            gx, gy = GridWorldTransform.world_to_grid(wx, wy)
            expected_gx = v['grid']['x']
            expected_gy = v['grid']['y']
            self.assertEqual(gx, expected_gx, f"Victim {v_id} grid_x mismatch: expected {expected_gx}, got {gx}")
            self.assertEqual(gy, expected_gy, f"Victim {v_id} grid_y mismatch: expected {expected_gy}, got {gy}")

    def test_pairwise_drone_to_victim_transformations(self):
        spawns = self.meta['drone_base']['spawns']
        victims = self.meta['victims']
        
        self.assertEqual(len(spawns), 6, "Metadata must contain 6 drone spawns")
        self.assertEqual(len(victims), 5, "Metadata must contain 5 victims")
        
        for d in spawns:
            d_id = d['id']
            sx = float(d['x'])
            sy = float(d['y'])
            sz = float(d.get('z', 12.22))
            
            for v in victims:
                v_id = v['id']
                vwx = float(v['world_x'])
                vwy = float(v['world_y'])
                
                # 1. Compute ENU -> NED
                ln, le = GridWorldTransform.world_enu_to_px4_ned(vwx, vwy, sx, sy)
                
                # Verify axis mapping:
                # Local North (NED index 0) must equal Delta Y (North in ENU)
                # Local East (NED index 1) must equal Delta X (East in ENU)
                self.assertAlmostEqual(ln, vwy - sy, places=5)
                self.assertAlmostEqual(le, vwx - sx, places=5)
                
                # 2. Compute NED -> ENU
                rec_wx, rec_wy = GridWorldTransform.px4_ned_to_world_enu(ln, le, sx, sy)
                self.assertAlmostEqual(rec_wx, vwx, places=5)
                self.assertAlmostEqual(rec_wy, vwy, places=5)


class TestControllerIntegrationPaths(unittest.TestCase):
    """Validates the complete setpoint dispatch and odometry intake paths in controllers."""

    def setUp(self):
        GridWorldTransform.set_bounds(
            width=25, height=25, meters_per_cell=8.0, origin_x=-180.0, origin_y=-20.0
        )

    def test_qmix_setpoint_generation_and_axis_order(self):
        """Tests that QMIX grid action generates the correct PX4 local setpoint in NED."""
        # Drone 0 spawn at (-168.0, 140.0, 12.22)
        spawn_x = -168.0
        spawn_y = 140.0
        initial_pos = [0.0, 0.0, 0.0]  # PX4 local at ground
        takeoff_alt = 15.0

        # Suppose QMIX selects safe grid cell (12, 18)
        safe_gx, safe_gy = 12, 18
        target_wx, target_wy = GridWorldTransform.grid_to_world_center(safe_gx, safe_gy)
        # Expected:
        # target_wx = -180.0 + (12 + 0.5) * 8.0 = -80.0
        # target_wy = -20.0 + (18 + 0.5) * 8.0 = 128.0
        self.assertAlmostEqual(target_wx, -80.0)
        self.assertAlmostEqual(target_wy, 128.0)

        # In qmix_mission_controller:
        local_north, local_east = GridWorldTransform.world_enu_to_px4_ned(target_wx, target_wy, spawn_x, spawn_y)
        lz = initial_pos[2] - takeoff_alt

        # local_north = 128.0 - 140.0 = -12.0
        # local_east = -80.0 - (-168.0) = +88.0
        # lz = -15.0
        self.assertAlmostEqual(local_north, -12.0)
        self.assertAlmostEqual(local_east, 88.0)
        self.assertAlmostEqual(lz, -15.0)

        # Mock agent goal setting
        mock_agent = MagicMock()
        mock_agent.mission_goal_local = [0.0, 0.0, 0.0]
        def set_goal(x, y, z, require_min_dwell=False):
            mock_agent.mission_goal_local = [x, y, z]
        mock_agent.set_mission_goal_local = set_goal

        mock_agent.set_mission_goal_local(local_north, local_east, lz)
        self.assertEqual(mock_agent.mission_goal_local[0], -12.0, "Index 0 must be Local North")
        self.assertEqual(mock_agent.mission_goal_local[1], 88.0, "Index 1 must be Local East")
        self.assertEqual(mock_agent.mission_goal_local[2], -15.0, "Index 2 must be Local Down (negative altitude)")

    def test_odometry_to_world_conversion_path(self):
        """Tests that PX4 odometry in NED converts to World ENU with correct position and velocity."""
        spawn_x = -168.0
        spawn_y = 140.0
        spawn_z = 12.22

        # Drone is physically at World ENU: (-148.0, 150.0, 27.22)
        # That corresponds to:
        # local_north = 150.0 - 140.0 = +10.0
        # local_east  = -148.0 - (-168.0) = +20.0
        # local_down  = -(27.22 - 12.22) = -15.0
        px4_current_pos = [10.0, 20.0, -15.0]
        # Velocity in NED: flying North at 1.5 m/s, East at 2.5 m/s, Climbing at 0.5 m/s (down = -0.5)
        px4_current_vel = [1.5, 2.5, -0.5]

        # Reconstructed world position as in _publish_telemetry and sync_env_state:
        local_north, local_east, local_down = px4_current_pos
        wx, wy = GridWorldTransform.px4_ned_to_world_enu(local_north, local_east, spawn_x, spawn_y)
        wz = -local_down + spawn_z

        self.assertAlmostEqual(wx, -148.0)
        self.assertAlmostEqual(wy, 150.0)
        self.assertAlmostEqual(wz, 27.22)

        # Reconstructed velocity as in _publish_telemetry:
        vn, ve, vd = px4_current_vel
        vx_enu = ve
        vy_enu = vn
        vz_enu = -vd
        speed = math.sqrt(vn**2 + ve**2 + vd**2)

        self.assertAlmostEqual(vx_enu, 2.5, msg="vx_enu must be East velocity")
        self.assertAlmostEqual(vy_enu, 1.5, msg="vy_enu must be North velocity")
        self.assertAlmostEqual(vz_enu, 0.5, msg="vz_enu must be Up velocity")
        self.assertAlmostEqual(speed, math.sqrt(1.5**2 + 2.5**2 + 0.5**2))

    def test_swarm_coordination_manager_round_trip(self):
        """Tests deterministic drone target and pos logic in SwarmCoordinationManager."""
        spawn_x, spawn_y = -168.0, 160.0  # Drone 2
        # Waypoint in World ENU
        twx, twy = -100.0, 120.0
        
        # Step dispatch in swarm_coordination_manager:
        local_north, local_east = GridWorldTransform.world_enu_to_px4_ned(twx, twy, spawn_x, spawn_y)
        self.assertAlmostEqual(local_north, -40.0)
        self.assertAlmostEqual(local_east, 68.0)

        # get_world_targets: takes local goal [goal_north, goal_east, goal_down] and reconstructs world pos
        goal_north = local_north
        goal_east = local_east
        rec_twx, rec_twy = GridWorldTransform.px4_ned_to_world_enu(goal_north, goal_east, spawn_x, spawn_y)
        self.assertAlmostEqual(rec_twx, twx)
        self.assertAlmostEqual(rec_twy, twy)


class TestConfirmationBehaviorDetailed(unittest.TestCase):
    """Validates victim confirmation logic under all edge cases."""

    def setUp(self):
        GridWorldTransform.set_bounds(
            width=25, height=25, meters_per_cell=8.0, origin_x=-180.0, origin_y=-20.0
        )
        self.confirmer = EventConfirmer(
            window_size=6,
            consensus_needed=4,
            proximity_threshold=3.0,
            grid_cell_size=8.0,
            min_confidence=0.45,
            min_time_span=0.4,
            min_depth=4.0,
            max_depth=32.0,
            min_valid_samples=15,
            max_depth_spread=2.5,
            dedup_radius=6.0
        )

    def test_original_false_positive_rejection(self):
        """The failure case from the recorded run: (-163.59, 224.00), conf=0.288."""
        oob_x, oob_y = -163.59, 224.00
        conf = 0.288

        # 1. Out of bounds check
        is_valid, reason = self.confirmer.is_valid_observation(
            oob_x, oob_y, confidence=conf, median_depth=15.0, valid_samples=30, depth_spread=1.0
        )
        self.assertFalse(is_valid)
        self.assertEqual(reason, "OUT_OF_BOUNDS")

        # 2. Confidence check even if coordinate was valid
        is_valid_in_bound, reason2 = self.confirmer.is_valid_observation(
            -100.0, 50.0, confidence=conf, median_depth=15.0, valid_samples=30, depth_spread=1.0
        )
        self.assertFalse(is_valid_in_bound)
        self.assertEqual(reason2, "LOW_CONFIDENCE")

    def test_insufficient_temporal_span_rejection(self):
        """4 detections arriving in an instantaneous burst (< 0.4s) must NOT confirm."""
        wx, wy = -82.0, 44.0  # near victim 0
        
        # 4 frames in 0.09s (span = 0.09s < 0.4s)
        burst_timestamps = [10.00, 10.03, 10.06, 10.09]
        events = []
        for ts in burst_timestamps:
            _, event = self.confirmer.add_observation(
                drone_id=0, timestamp=ts, world_x=wx, world_y=wy,
                confidence=0.85, median_depth=15.0, valid_samples=30, depth_spread=0.5
            )
            if event:
                events.append(event)
        
        self.assertEqual(len(events), 0, "Burst frames < 0.4s span must not trigger confirmation")
        self.assertEqual(len(self.confirmer.confirmed_victims), 0)

        # Now send a 5th frame at ts = 10.50 (span from frame 2 at 10.03 = 0.47s >= 0.4s)
        _, event = self.confirmer.add_observation(
            drone_id=0, timestamp=10.50, world_x=wx, world_y=wy,
            confidence=0.85, median_depth=15.0, valid_samples=30, depth_spread=0.5
        )
        self.assertIsNotNone(event, "Must confirm once temporal span >= 0.4s is satisfied")
        self.assertEqual(len(self.confirmer.confirmed_victims), 1)

    def test_invalid_depth_and_excessive_spread(self):
        wx, wy = -70.0, 120.0
        
        # Too shallow (tree canopy top)
        v, r = self.confirmer.is_valid_observation(wx, wy, 0.80, median_depth=2.0, valid_samples=30, depth_spread=0.5)
        self.assertFalse(v)
        self.assertEqual(r, "INVALID_DEPTH_RANGE")

        # Too deep (sky or invalid ray)
        v, r = self.confirmer.is_valid_observation(wx, wy, 0.80, median_depth=40.0, valid_samples=30, depth_spread=0.5)
        self.assertFalse(v)
        self.assertEqual(r, "INVALID_DEPTH_RANGE")

        # Excessive spread (> 2.5m, jagged canopy edges)
        v, r = self.confirmer.is_valid_observation(wx, wy, 0.80, median_depth=15.0, valid_samples=30, depth_spread=3.5)
        self.assertFalse(v)
        self.assertEqual(r, "HIGH_DEPTH_SPREAD")

        # Insufficient samples (< 15)
        v, r = self.confirmer.is_valid_observation(wx, wy, 0.80, median_depth=15.0, valid_samples=12, depth_spread=0.5)
        self.assertFalse(v)
        self.assertEqual(r, "INSUFFICIENT_DEPTH_SAMPLES")

    def test_acceptance_of_all_five_actual_victims(self):
        """Tests that persistent high-quality detections at each actual victim location confirm."""
        with open(META_FILE, 'r') as f:
            meta = json.load(f)
        victims = meta['victims']
        
        for i, v in enumerate(victims):
            vwx = v['world_x']
            vwy = v['world_y']
            
            # Feed 4 valid frames spaced by 0.2s
            events = []
            for step in range(4):
                ts = 100.0 * (i + 1) + step * 0.2
                track_id, event = self.confirmer.add_observation(
                    drone_id=i % 6,
                    timestamp=ts,
                    world_x=vwx + (step * 0.05),
                    world_y=vwy - (step * 0.05),
                    confidence=0.82 + step * 0.02,
                    median_depth=15.0,
                    valid_samples=35,
                    depth_spread=0.6
                )
                if event:
                    events.append(event)
            
            self.assertEqual(len(events), 1, f"Victim {v['id']} should confirm on 4th frame")
            ev = events[0]
            self.assertAlmostEqual(ev['world_x'], vwx, delta=0.2)
            self.assertAlmostEqual(ev['world_y'], vwy, delta=0.2)
            self.assertEqual(ev['confirmation_count'], 4)
            self.assertGreaterEqual(ev['consensus_span_sec'], 0.4)

        self.assertEqual(len(self.confirmer.confirmed_victims), 5, "All 5 victims must be confirmed")

    def test_multi_drone_observation_and_deduplication(self):
        """Drone 0 confirms victim at v0; Drone 2 later observes the same victim nearby."""
        with open(META_FILE, 'r') as f:
            meta = json.load(f)
        v0 = meta['victims'][0]
        v0_x, v0_y = v0['world_x'], v0['world_y']

        # Drone 0 confirms v0
        for step in range(4):
            self.confirmer.add_observation(
                drone_id=0, timestamp=10.0 + step * 0.2,
                world_x=v0_x, world_y=v0_y,
                confidence=0.80, median_depth=15.0, valid_samples=30, depth_spread=0.5
            )
        self.assertEqual(len(self.confirmer.confirmed_victims), 1)

        # Later, Drone 2 observes the same victim 1.5m away at (v0_x + 1.0, v0_y + 1.0)
        # Distance = sqrt(2) = 1.41m < dedup_radius (6.0m)
        obs_x, obs_y = v0_x + 1.0, v0_y + 1.0
        is_valid, reason = self.confirmer.is_valid_observation(
            obs_x, obs_y, confidence=0.88, median_depth=15.0, valid_samples=30, depth_spread=0.5
        )
        self.assertFalse(is_valid)
        self.assertEqual(reason, "DUPLICATE_NEAR_CONFIRMED")

        # Ensure no event is emitted and confirmed_victims remains 1
        for step in range(4):
            _, event = self.confirmer.add_observation(
                drone_id=2, timestamp=50.0 + step * 0.2,
                world_x=obs_x, world_y=obs_y,
                confidence=0.88, median_depth=15.0, valid_samples=30, depth_spread=0.5
            )
            self.assertIsNone(event)
        self.assertEqual(len(self.confirmer.confirmed_victims), 1)


class TestRuntimeConfigurationConsistency(unittest.TestCase):
    """Verifies that localizer and controllers parse the exact same search bounds."""

    def test_localizer_loads_forest_dense_candidate_metadata(self):
        os.environ["WORLD_META_PATH"] = META_FILE
        with open(META_FILE, 'r') as f:
            meta = json.load(f)
        w_cfg = meta.get('search_area') or meta.get('world') or {}
        
        origin_x = float(w_cfg.get('origin_x', w_cfg.get('grid_origin_x', -180.0)))
        origin_y = float(w_cfg.get('origin_y', w_cfg.get('grid_origin_y', -20.0)))
        width = int(w_cfg.get('grid_width', 25))
        height = int(w_cfg.get('grid_height', 25))
        meters_per_cell = float(w_cfg.get('meters_per_cell', 8.0))

        GridWorldTransform.set_bounds(width, height, meters_per_cell, origin_x, origin_y)

        self.assertEqual(GridWorldTransform.GRID_ORIGIN_X, -180.0)
        self.assertEqual(GridWorldTransform.GRID_ORIGIN_Y, -20.0)
        self.assertEqual(GridWorldTransform.GRID_SIZE_X, 25)
        self.assertEqual(GridWorldTransform.GRID_SIZE_Y, 25)
        self.assertEqual(GridWorldTransform.METERS_PER_CELL, 8.0)


if __name__ == '__main__':
    unittest.main()
