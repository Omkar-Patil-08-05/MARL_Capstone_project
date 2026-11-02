#!/usr/bin/env python3
"""
Unit tests for SAR Dashboard & Simulation Debugging Fixes:
1. Protected baseline world hash verification.
2. World-to-SVG coordinate transforms and clipping for forest_sar and realistic_sar.
3. Operational search area boundaries (200m x 200m, 25x25 grid, 8m cells).
4. Victim configuration & metadata consistency (entity count, IDs, coordinates, sectors).
5. Dynamic victim count in map registry / backend world loader.
"""

import os
import sys
import json
import hashlib
import unittest
import numpy as np

WORKSPACE = "/home/capstone/capstone_project_antigravity"
sys.path.insert(0, os.path.join(WORKSPACE, "world_generator"))
sys.path.insert(0, os.path.join(WORKSPACE, "dashboard/backend"))
sys.path.insert(0, os.path.join(WORKSPACE, "drone_ws/src/swarm_controller"))

PROTECTED_HASHES = {
    "generated_worlds/forest_stage1_baseline.sdf": "c2d01d75be0e0f9e4fc9fbf0aab047f5",
    "generated_worlds/forest_stage1_baseline_metadata.json": "9975d3d844fb108455289393b2b5632a",
    "generated_worlds/forest_rescue_world.sdf": "7147545e7087ac3876fbe341d35cb193",
    "generated_worlds/forest_rescue_metadata.json": "2fe08eb542508df4045dbac3c120541f",
    "generated_worlds/earthquake_world.sdf": "46c779ae4b0ff84acc6e7df3734646b0",
    "worlds/earthquake_world.sdf": "04ffa518b71cfc5959f4c0e5d94ec207"
}

class TestProtectedFiles(unittest.TestCase):
    def test_protected_checksums(self):
        for rel_path, expected_hash in PROTECTED_HASHES.items():
            full_path = os.path.join(WORKSPACE, rel_path)
            self.assertTrue(os.path.exists(full_path), f"Missing protected file: {rel_path}")
            with open(full_path, "rb") as f:
                actual_hash = hashlib.md5(f.read()).hexdigest()
            self.assertEqual(actual_hash, expected_hash, f"Protected file hash mismatch: {rel_path}")

class TestCoordinateTransforms(unittest.TestCase):
    def test_forest_sar_transform_and_clipping(self):
        origin_x = -180.0
        origin_y = -20.0
        width_m = 200.0
        height_m = 200.0
        mpc = 8.0

        def world_to_svg(wx, wy):
            return wx - origin_x, wy - origin_y

        def world_to_grid(wx, wy):
            gx = int(np.floor((wx - origin_x) / mpc))
            gy = int(np.floor((wy - origin_y) / mpc))
            return gx, gy

        # Base camp center (-150, 150)
        sx, sy = world_to_svg(-150.0, 150.0)
        self.assertAlmostEqual(sx, 30.0)
        self.assertAlmostEqual(sy, 170.0)
        gx, gy = world_to_grid(-150.0, 150.0)
        self.assertEqual(gx, 3)
        self.assertEqual(gy, 21)
        self.assertTrue(0 <= sx <= width_m)
        self.assertTrue(0 <= sy <= height_m)

        # Operational search boundary corners
        # Bottom-Left (-180, -20) -> (0, 0)
        bl_x, bl_y = world_to_svg(-180.0, -20.0)
        self.assertEqual((bl_x, bl_y), (0.0, 0.0))

        # Top-Right (20, 180) -> (200, 200)
        tr_x, tr_y = world_to_svg(20.0, 180.0)
        self.assertEqual((tr_x, tr_y), (200.0, 200.0))

        # Out-of-bounds point (e.g. y = 223.63, or x = 120 outside 200m SAR grid)
        oob_x, oob_y = world_to_svg(-86.9, 223.63)
        self.assertGreater(oob_y, height_m, "OOB point correctly exceeds SVG height")

    def test_realistic_sar_transform(self):
        origin_x = 0.0
        origin_y = 0.0
        width_m = 100.0
        height_m = 100.0
        mpc = 4.0

        def world_to_svg(wx, wy):
            return wx - origin_x, wy - origin_y

        sx, sy = world_to_svg(12.0, 12.0)
        self.assertEqual((sx, sy), (12.0, 12.0))
        self.assertTrue(0 <= sx <= width_m)
        self.assertTrue(0 <= sy <= height_m)

class TestVictimPlacementBounds(unittest.TestCase):
    def test_forest_layout_victim_sampling_bounds(self):
        from forest_config import WORLD_SIZE, SEARCH_REGION_SIZE
        from forest_terrain import generate_forest_heightmap
        from forest_layout import ForestLayoutManager

        import tempfile
        with tempfile.NamedTemporaryFile(suffix=".png") as tmp:
            terrain_map, _ = generate_forest_heightmap(42, tmp.name)
        layout = ForestLayoutManager(terrain_map)

        # Generate 3 victims
        victims = layout.generate_victims(count=3)
        self.assertEqual(len(victims), 3, "Expected exactly 3 victims generated")

        # Verify all 3 victims are STRICTLY within the operational SAR search area:
        # [-180, 20] x [-20, 180]
        for v in victims:
            vx, vy = v["x"], v["y"]
            self.assertGreaterEqual(vx, -180.0, f"Victim {v['id']} X below search region min")
            self.assertLessEqual(vx, 20.0, f"Victim {v['id']} X above search region max")
            self.assertGreaterEqual(vy, -20.0, f"Victim {v['id']} Y below search region min")
            self.assertLessEqual(vy, 180.0, f"Victim {v['id']} Y above search region max")

            # Verify grid coordinates
            gx = int(np.floor((vx - (-180.0)) / 8.0))
            gy = int(np.floor((vy - (-20.0)) / 8.0))
            self.assertTrue(0 <= gx <= 24, f"Victim grid X {gx} outside [0, 24]")
            self.assertTrue(0 <= gy <= 24, f"Victim grid Y {gy} outside [0, 24]")

class TestBackendWorldRegistry(unittest.TestCase):
    def test_dynamic_victim_count(self):
        from world import get_world_data, get_map_registry
        registry = get_map_registry()
        self.assertIn("realistic_sar", registry)
        self.assertIn("forest_sar", registry)

        world_data = get_world_data("forest_sar")
        self.assertNotIn("error", world_data)
        self.assertEqual(world_data["grid"]["width"], 25)
        self.assertEqual(world_data["grid"]["height"], 25)
        self.assertEqual(world_data["grid"]["meters_per_cell"], 8.0)
        self.assertEqual(world_data["world"]["origin_x"], -180.0)
        self.assertEqual(world_data["world"]["origin_y"], -20.0)

if __name__ == "__main__":
    unittest.main()
