#!/usr/bin/env python3
"""
scripts/test_detection_semantics.py

Deterministic evaluation of SAR victim detection semantics across models:
1. SARGridEnv discrete 3x3 FOV (Chebyshev grid window)
2. Live SITL Proximity Detection (Euclidean r <= 3.5m)
3. Proposed Inscribed Euclidean Detection (r = 1.5 * L_cell: 6.0m for realistic, 12.0m for forest)
4. Downward RGB-D Camera Footprint at 15m cruise altitude (FOV 60° x 45° -> 17.32m x 12.43m ground box)

Evaluates realistic_sar (4m cells) and forest_sar (8m cells) at representative offsets:
- 0m, 3.5m, 4m, 5.5m, 6m, 7m (axial)
- Corner and boundary offsets
- Drone off-center offsets
"""

import math
import numpy as np
import sys
import os

WORKSPACE = "/home/capstone/capstone_project_antigravity"
sys.path.insert(0, os.path.join(WORKSPACE, "marl_drone_project/env"))
sys.path.insert(0, os.path.join(WORKSPACE, "drone_ws/src/swarm_controller"))

from sar_env import SARGridEnv
from swarm_controller.grid_world_transform import GridWorldTransform

def evaluate_models(map_name, cell_size, drone_wx, drone_wy, victim_wx, victim_wy):
    # Set GridWorldTransform parameters
    GridWorldTransform.set_bounds(25, 25, meters_per_cell=cell_size, origin_x=0.0, origin_y=0.0)
    
    # 1. Coordinate calculation
    drone_gx, drone_gy = GridWorldTransform.world_to_grid(drone_wx, drone_wy)
    victim_gx, victim_gy = GridWorldTransform.world_to_grid(victim_wx, victim_wy)
    
    euclidean_dist = math.hypot(victim_wx - drone_wx, victim_wy - drone_wy)
    dx_m = abs(victim_wx - drone_wx)
    dy_m = abs(victim_wy - drone_wy)
    
    # 2. Model 1: SARGridEnv (offline 3x3 discrete grid Chebyshev FOV)
    # Drone at (drone_gx, drone_gy), victim at (victim_gx, victim_gy)
    grid_dx = abs(victim_gx - drone_gx)
    grid_dy = abs(victim_gy - drone_gy)
    sar_grid_detected = (grid_dx <= 1 and grid_dy <= 1)
    
    # Also verify with actual SARGridEnv instance
    env = SARGridEnv(x_size=25, y_size=25, num_drones=1, fov_size=3, max_steps=10, num_victims=1)
    env.grid.fill(0)
    env.drone_positions = [[drone_gx, drone_gy]]
    env.victims = {(victim_gx, victim_gy): 0}
    env.metrics['victims_detected'] = 0
    _, new_v = env._update_fov_and_victims()
    actual_sar_detected = (new_v[0] > 0)
    
    # 3. Model 2: Live physical proximity gate (r <= 3.5m)
    live_3_5m_detected = (euclidean_dist <= 3.5)
    
    # 4. Model 3: Physical proximity gate with r = 1.5 * cell_size (6m for 4m cells, 12m for 8m cells)
    radius_1_5_cell = 1.5 * cell_size
    live_1_5_cell_detected = (euclidean_dist <= radius_1_5_cell)
    
    # 5. Model 4: Camera Optical Footprint at 15m altitude
    # Downward camera: pitch = -90 deg. Horizontal FOV = 60°, Vertical FOV = 45°
    # Footprint half-width X = 15 * tan(30°) = 8.660 m
    # Footprint half-width Y = 15 * tan(22.5°) = 6.213 m
    # Assuming camera aligned with world axes (yaw = 0):
    cam_half_x = 15.0 * math.tan(math.radians(30.0)) # 8.660 m
    cam_half_y = 15.0 * math.tan(math.radians(22.5)) # 6.213 m
    camera_detected = (dx_m <= cam_half_x and dy_m <= cam_half_y)
    
    return {
        "drone_world": (round(drone_wx, 2), round(drone_wy, 2)),
        "drone_grid": (drone_gx, drone_gy),
        "victim_world": (round(victim_wx, 2), round(victim_wy, 2)),
        "victim_grid": (victim_gx, victim_gy),
        "euclidean_dist": round(euclidean_dist, 3),
        "offset_m": (round(dx_m, 2), round(dy_m, 2)),
        "sar_grid_detected": sar_grid_detected,
        "actual_sar_detected": actual_sar_detected,
        "live_3_5m_detected": live_3_5m_detected,
        "live_1_5_cell_detected": live_1_5_cell_detected,
        "camera_detected": camera_detected
    }

def run_suite():
    print("=" * 110)
    print("DETERMINISTIC SAR VICTIM DETECTION SEMANTICS COMPARISON SUITE")
    print("=" * 110)
    
    # ==========================================
    # TEST SUITE 1: realistic_sar (4m Grid Cells)
    # ==========================================
    cell_size = 4.0
    drone_center_wx, drone_center_wy = 50.0, 50.0 # Grid (12, 12)
    
    print("\n--- TEST SET 1: realistic_sar (4.0m cells) | Drone at Cell (12,12) Center (50.0m, 50.0m) ---")
    
    offsets_realistic = [
        ("Axial 0.0m (Co-located)", 0.0, 0.0),
        ("Axial 3.5m (At 3.5m boundary)", 3.5, 0.0),
        ("Axial 4.0m (1-cell neighbor center)", 4.0, 0.0),
        ("Axial 5.5m (Inside adjacent cell)", 5.5, 0.0),
        ("Axial 6.0m (Cell boundary edge)", 6.0, 0.0),
        ("Axial 7.0m (Outside 3x3 footprint)", 7.0, 0.0),
        ("Corner 3.5m x 3.5m (d = 4.95m)", 3.5, 3.5),
        ("Corner 4.0m x 4.0m (d = 5.66m, cell 13,13 center)", 4.0, 4.0),
        ("Corner 5.5m x 5.5m (d = 7.78m, inside corner cell)", 5.5, 5.5),
        ("Corner 5.9m x 5.9m (d = 8.34m, near outer corner)", 5.9, 5.9),
        ("Boundary Edge Inside (5.99m, 0.0m)", 5.99, 0.0),
        ("Boundary Edge Outside (6.01m, 0.0m)", 6.01, 0.0),
        ("Corner Outside (6.01m, 6.01m, d = 8.50m)", 6.01, 6.01),
        ("Camera Rect Edge (8.0m X, 0.0m Y)", 8.0, 0.0),
        ("Camera Rect Out (0.0m X, 7.0m Y)", 0.0, 7.0),
    ]
    
    header = f"{'Case Description':<38} | {'Dist':<6} | {'Grid Offset':<11} | {'SAR Grid':<8} | {'Live 3.5m':<9} | {'Radius 6.0m':<11} | {'Camera FOV':<10}"
    print(header)
    print("-" * len(header))
    
    results_realistic = []
    for desc, ox, oy in offsets_realistic:
        res = evaluate_models("realistic_sar", 4.0, drone_center_wx, drone_center_wy, drone_center_wx + ox, drone_center_wy + oy)
        results_realistic.append((desc, res))
        grid_off_str = f"({res['victim_grid'][0]-res['drone_grid'][0]},{res['victim_grid'][1]-res['drone_grid'][1]})"
        print(f"{desc:<38} | {res['euclidean_dist']:<6.2f} | {grid_off_str:<11} | {str(res['sar_grid_detected']):<8} | {str(res['live_3_5m_detected']):<9} | {str(res['live_1_5_cell_detected']):<11} | {str(res['camera_detected']):<10}")
        
    # Test off-center drone position within cell 12
    print("\n--- TEST SET 1B: realistic_sar Drone Off-Center (51.5m, 50.0m, still in cell (12,12)) ---")
    drone_off_wx, drone_off_wy = 51.5, 50.0
    offsets_offcenter = [
        ("Victim at 57.0m (dist=5.5m, cell 14)", 57.0, 50.0),
        ("Victim at 45.0m (dist=6.5m, cell 11)", 45.0, 50.0),
    ]
    for desc, vx, vy in offsets_offcenter:
        res = evaluate_models("realistic_sar", 4.0, drone_off_wx, drone_off_wy, vx, vy)
        grid_off_str = f"({res['victim_grid'][0]-res['drone_grid'][0]},{res['victim_grid'][1]-res['drone_grid'][1]})"
        print(f"{desc:<38} | {res['euclidean_dist']:<6.2f} | {grid_off_str:<11} | {str(res['sar_grid_detected']):<8} | {str(res['live_3_5m_detected']):<9} | {str(res['live_1_5_cell_detected']):<11} | {str(res['camera_detected']):<10}")

    # ==========================================
    # TEST SUITE 2: forest_sar (8m Grid Cells)
    # ==========================================
    print("\n--- TEST SET 2: forest_sar (8.0m cells) | Drone at Cell (12,12) Center (100.0m, 100.0m) ---")
    drone_forest_wx, drone_forest_wy = 100.0, 100.0
    
    offsets_forest = [
        ("Axial 0.0m (Co-located)", 0.0, 0.0),
        ("Axial 3.5m (Live default threshold)", 3.5, 0.0),
        ("Axial 4.0m (Inside same 8m cell)", 4.0, 0.0),
        ("Axial 6.0m (Camera Y-limit)", 6.0, 0.0),
        ("Axial 8.0m (1-cell neighbor center)", 8.0, 0.0),
        ("Axial 11.5m (Inside adjacent cell)", 11.5, 0.0),
        ("Axial 12.0m (Cell boundary edge)", 12.0, 0.0),
        ("Axial 13.0m (Outside 3x3 footprint)", 13.0, 0.0),
        ("Corner 7.0m x 7.0m (d = 9.90m, cell 13,13)", 7.0, 7.0),
        ("Corner 8.0m x 8.0m (d = 11.31m, cell 13,13 center)", 8.0, 8.0),
        ("Corner 11.5m x 11.5m (d = 16.26m, inside corner)", 11.5, 11.5),
        ("Corner 11.9m x 11.9m (d = 16.83m, near outer corner)", 11.9, 11.9),
        ("Boundary Edge Inside (11.99m, 0.0m)", 11.99, 0.0),
        ("Boundary Edge Outside (12.01m, 0.0m)", 12.01, 0.0),
    ]
    
    header_forest = f"{'Case Description':<38} | {'Dist':<6} | {'Grid Offset':<11} | {'SAR Grid':<8} | {'Live 3.5m':<9} | {'Radius 12.0m':<12} | {'Camera FOV':<10}"
    print(header_forest)
    print("-" * len(header_forest))
    
    results_forest = []
    for desc, ox, oy in offsets_forest:
        res = evaluate_models("forest_sar", 8.0, drone_forest_wx, drone_forest_wy, drone_forest_wx + ox, drone_forest_wy + oy)
        results_forest.append((desc, res))
        grid_off_str = f"({res['victim_grid'][0]-res['drone_grid'][0]},{res['victim_grid'][1]-res['drone_grid'][1]})"
        print(f"{desc:<38} | {res['euclidean_dist']:<6.2f} | {grid_off_str:<11} | {str(res['sar_grid_detected']):<8} | {str(res['live_3_5m_detected']):<9} | {str(res['live_1_5_cell_detected']):<12} | {str(res['camera_detected']):<10}")
        
    print("\n" + "=" * 110)
    print("DETECTION SEMANTICS SUMMARY MATRIX")
    print("=" * 110)
    print("1. In SARGridEnv (Training/Eval): Detection is strictly discrete Chebyshev grid-cell containment (dx <= 1 and dy <= 1).")
    print("   - For realistic_sar (4m): Footprint is a 12m x 12m square (max Euclidean distance to corner = 8.49m).")
    print("   - For forest_sar (8m): Footprint is a 24m x 24m square (max Euclidean distance to corner = 16.97m).")
    print("2. In Live SITL Proximity Detection: Detection is an isotropic Euclidean circle of radius r <= 3.5m.")
    print("   - For realistic_sar (4m): Rejects 73.3% of the training FOV area! Any victim beyond 3.5m (up to 8.49m) is MISSED.")
    print("   - For forest_sar (8m): Rejects 93.3% of the training FOV area! Any victim beyond 3.5m (up to 16.97m) is MISSED.")
    print("3. In Physical Camera FOV (15m cruise, 60°x45°): Ground footprint is a 17.32m x 12.43m rectangle.")
    print("   - Inscribes an axial radius of 6.21m across Y, 8.66m across X.")
    print("4. Radius 1.5 * L_cell (6.0m for realistic, 12.0m for forest):")
    print("   - Exactly matches the axial cell boundary of the 3x3 grid footprint.")
    print("   - Does NOT cover the 4 outer corners of the discrete 3x3 square (corner distance 8.49m > 6.0m).")

if __name__ == "__main__":
    run_suite()
