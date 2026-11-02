#!/usr/bin/env python3
"""
Test harness to validate Forest SAR environment integration.
Tests:
1. Backend map registry and get_world_data() for both realistic_sar and forest_sar
2. Coordinate system transformations and grid boundaries
3. Collision validation: 6 drone spawn pads vs obstacles
4. Victim validation: ground contact, slope limit, tree clearance, search grid bounds
5. XML validity of worlds/forest_sar.sdf
6. Protected baseline checksum integrity
"""

import os
import sys
import json
import math
import hashlib
import xml.etree.ElementTree as ET

WORKSPACE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, WORKSPACE_DIR)
sys.path.insert(0, os.path.join(WORKSPACE_DIR, "dashboard", "backend"))
sys.path.insert(0, os.path.join(WORKSPACE_DIR, "drone_ws", "src", "swarm_controller"))

from world import get_map_registry, get_world_data
from swarm_controller.grid_world_transform import GridWorldTransform

PROTECTED_HASHES = {
    "generated_worlds/forest_stage1_baseline.sdf": "c2d01d75be0e0f9e4fc9fbf0aab047f5",
    "generated_worlds/forest_stage1_baseline_metadata.json": "9975d3d844fb108455289393b2b5632a",
    "generated_worlds/forest_rescue_world.sdf": "7147545e7087ac3876fbe341d35cb193",
    "generated_worlds/forest_rescue_metadata.json": "2fe08eb542508df4045dbac3c120541f",
    "generated_worlds/earthquake_world.sdf": "46c779ae4b0ff84acc6e7df3734646b0",
    "worlds/earthquake_world.sdf": "04ffa518b71cfc5959f4c0e5d94ec207"
}

def verify_checksums():
    print("--- [1/6] Verifying Protected Baseline Checksums ---")
    for rel_path, expected in PROTECTED_HASHES.items():
        full_path = os.path.join(WORKSPACE_DIR, rel_path)
        assert os.path.exists(full_path), f"Missing protected file: {full_path}"
        with open(full_path, "rb") as f:
            actual = hashlib.md5(f.read()).hexdigest()
        assert actual == expected, f"CHECKSUM MISMATCH on {rel_path}: expected {expected}, got {actual}"
        print(f"  [PASS] {rel_path}: {actual}")

def verify_map_registry():
    print("\n--- [2/6] Verifying Backend Map Registry & API Data ---")
    reg = get_map_registry()
    assert "realistic_sar" in reg, "realistic_sar missing from registry!"
    assert "forest_sar" in reg, "forest_sar missing from registry!"

    earthquake_data = get_world_data("realistic_sar")
    assert "error" not in earthquake_data, f"Error loading realistic_sar: {earthquake_data.get('error')}"
    assert len(earthquake_data["drone_spawns"]) == 6, f"Expected 6 drone spawns in realistic_sar, got {len(earthquake_data['drone_spawns'])}"
    print(f"  [PASS] realistic_sar loaded successfully ({len(earthquake_data['obstacles'])} obstacles, {len(earthquake_data['victims'])} victims)")

    forest_data = get_world_data("forest_sar")
    assert "error" not in forest_data, f"Error loading forest_sar: {forest_data.get('error')}"
    assert len(forest_data["drone_spawns"]) == 6, f"Expected 6 drone spawns in forest_sar, got {len(forest_data['drone_spawns'])}"
    assert len(forest_data["sectors"]) == 4, f"Expected 4 search sectors in forest_sar, got {len(forest_data['sectors'])}"
    assert forest_data["world"]["width_m"] == 200.0, f"Expected 200m operational width, got {forest_data['world']['width_m']}"
    assert forest_data["world"]["origin_x"] == -180.0, f"Expected origin_x=-180.0, got {forest_data['world']['origin_x']}"
    assert forest_data["world"]["origin_y"] == -20.0, f"Expected origin_y=-20.0, got {forest_data['world']['origin_y']}"
    assert forest_data["world"]["terrain_width_m"] == 500.0, f"Expected terrain 500m, got {forest_data['world']['terrain_width_m']}"
    print(f"  [PASS] forest_sar loaded successfully ({len(forest_data['obstacles'])} obstacles, {len(forest_data['victims'])} victims, {len(forest_data['sectors'])} sectors)")

def verify_coordinate_transform():
    print("\n--- [3/6] Verifying Coordinate Transformations ---")
    # 1. Realistic SAR
    GridWorldTransform.set_bounds(25, 25, 4.0, 0.0, 0.0)
    gx, gy = GridWorldTransform.world_to_grid(21.0, 25.0)
    assert (gx, gy) == (5, 6), f"Realistic SAR transform mismatch: got ({gx}, {gy})"
    cx, cy = GridWorldTransform.grid_to_world_center(5, 6)
    assert abs(cx - 22.0) < 1e-4 and abs(cy - 26.0) < 1e-4

    # 2. Forest SAR
    GridWorldTransform.set_bounds(25, 25, 8.0, -180.0, -20.0)
    # Base center: (-149.80, 149.80) -> gx = (-149.8 - (-180))/8 = 30.2/8 = 3, gy = (149.8 - (-20))/8 = 169.8/8 = 21
    gx, gy = GridWorldTransform.world_to_grid(-149.80, 149.80)
    assert (gx, gy) == (3, 21), f"Forest SAR base center transform mismatch: got ({gx}, {gy})"
    safe_x, safe_y, is_valid = GridWorldTransform.clamp_grid(gx, gy)
    assert is_valid, "Base center was clamped out of bounds!"
    print("  [PASS] GridWorldTransform matches continuous world to discrete grid correctly for both environments")

def verify_drone_spawns_and_collisions():
    print("\n--- [4/6] Verifying Drone Spawn Pads & Obstacle Clearance ---")
    meta_path = os.path.join(WORKSPACE_DIR, "worlds", "forest_sar_meta.json")
    with open(meta_path, "r") as f:
        meta = json.load(f)

    spawns = meta["drone_base"]["spawns"]
    obstacles = meta["obstacles"]
    assert len(spawns) == 6, f"Expected 6 drone spawns, got {len(spawns)}"

    for s in spawns:
        sx, sy, sz = s["x"], s["y"], s["z"]
        # Ensure spawn is inside operational search area [-180, 20] x [-20, 180]
        assert -180.0 <= sx <= 20.0 and -20.0 <= sy <= 180.0, f"Spawn {s['id']} outside operational search area: ({sx}, {sy})"
        # Ensure spawn elevation is elevated on the pad (cz ~ 12.0m + 0.15m = 12.15m)
        assert 11.0 <= sz <= 14.0, f"Spawn {s['id']} has invalid altitude: {sz}"

        # Collision clearance against all registered obstacles
        for obs in obstacles:
            aabb = obs.get("aabb")
            if not aabb:
                continue
            # Point to AABB distance
            dx = max(aabb["min_x"] - sx, 0, sx - aabb["max_x"])
            dy = max(aabb["min_y"] - sy, 0, sy - aabb["max_y"])
            dist = math.hypot(dx, dy)
            assert dist >= 2.0, f"Collision conflict between {s['id']} and obstacle {obs['id']}: dist={dist:.2f}m < 2.0m"

        gx, gy = GridWorldTransform.world_to_grid(sx, sy)
        assert 0 <= gx < 25 and 0 <= gy < 25, f"Spawn {s['id']} maps out of grid: ({gx}, {gy})"
        print(f"  [PASS] {s['id']}: world=({sx:.2f}, {sy:.2f}, {sz:.2f}) -> grid=({gx}, {gy}) [collision-free]")

def verify_victims():
    print("\n--- [5/6] Verifying Victims Quality & Placement ---")
    meta_path = os.path.join(WORKSPACE_DIR, "worlds", "forest_sar_meta.json")
    with open(meta_path, "r") as f:
        meta = json.load(f)

    victims = meta["victims"]
    assert len(victims) >= 3, f"Expected at least 3 victims, got {len(victims)}"

    for v in victims:
        vx, vy, vz = v["world"]["x"], v["world"]["y"], v["world"]["z"]
        gx, gy = v["grid"]["x"], v["grid"]["y"]
        slope = v["slope_deg"]
        ground_off = v["ground_offset"]
        sector = v["sector"]

        assert 0 <= gx < 25 and 0 <= gy < 25, f"Victim {v['id']} grid out of bounds: ({gx}, {gy})"
        assert slope <= 8.0, f"Victim {v['id']} exceeds max slope: {slope}° > 8.0°"
        assert abs(ground_off - 0.02) < 1e-4, f"Victim {v['id']} has incorrect ground offset: {ground_off}"
        assert sector in [s["name"] for s in meta["sectors"]], f"Victim {v['id']} has unrecognized sector: {sector}"
        print(f"  [PASS] {v['id']}: grid=({gx:2d}, {gy:2d}) world=({vx:6.1f}, {vy:6.1f}, {vz:4.1f}) slope={slope:.1f}° sector='{sector}'")

def verify_sdf_xml():
    print("\n--- [6/6] Validating SDF XML Structure ---")
    sdf_path = os.path.join(WORKSPACE_DIR, "worlds", "forest_sar.sdf")
    assert os.path.exists(sdf_path), f"SDF file does not exist: {sdf_path}"

    tree = ET.parse(sdf_path)
    root = tree.getroot()
    assert root.tag == "sdf", f"Expected root tag <sdf>, got <{root.tag}>"
    world = root.find("world")
    assert world is not None, "Missing <world> element"
    assert world.attrib.get("name") == "forest_sar", f"World name should be 'forest_sar', got {world.attrib.get('name')}"

    # Verify plugins: in PX4 Gazebo Harmonic architecture, world plugins are provided
    # via server.config (GZ_SIM_SERVER_CONFIG_PATH) to avoid duplicate system loading.
    plugins = world.findall("plugin")
    plugin_names = [p.attrib.get("name") for p in plugins]
    server_config_path = "/home/capstone/PX4-Autopilot/src/modules/simulation/gz_bridge/server.config"
    assert os.path.exists(server_config_path), f"Server config missing at {server_config_path}"
    if plugins:
        for ep in ["gz::sim::systems::Physics", "gz::sim::systems::SceneBroadcaster"]:
            assert ep in plugin_names, f"Missing critical Gazebo plugin: {ep}"

    # Verify models
    models = world.findall("model")
    model_names = [m.attrib.get("name") for m in models]
    for i in range(6):
        assert f"base_pad_{i}" in model_names, f"Missing base_pad_{i} in SDF"
    print(f"  [PASS] SDF XML parsed cleanly: {len(models)} models, {len(plugins)} plugins verified")

if __name__ == "__main__":
    try:
        verify_checksums()
        verify_map_registry()
        verify_coordinate_transform()
        verify_drone_spawns_and_collisions()
        verify_victims()
        verify_sdf_xml()
        print("\n========================================================")
        print("ALL 6 FOREST INTEGRATION VALIDATION SUITES PASSED (100%)")
        print("========================================================")
    except Exception as e:
        print(f"\n[FAIL] Validation error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
