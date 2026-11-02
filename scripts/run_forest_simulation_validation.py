#!/usr/bin/env python3
"""
Forest SAR Simulation Validation Benchmark.
Validates the complete SAR search pipeline on forest_sar geometry:
- Origin: (-180.0, -20.0), Cell Size: 8.0m, Grid: 25x25 (422 valid cells, 203 obstacles)
- Synchronized simulation-time dynamic victims
- Appropriate detection radius: r = 1.5 * L_cell = 12.0m
- QMIX and Frontier-Search policy execution
- Telemetry, detection events, and coverage metrics
"""

import os
import sys
import json
import time
import math
import random
import numpy as np
import torch
from typing import Dict, List, Any, Tuple

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(REPO_ROOT)
sys.path.append(os.path.join(REPO_ROOT, 'marl_drone_project'))
sys.path.append(os.path.join(REPO_ROOT, 'drone_ws', 'src', 'swarm_controller'))

from env.sar_env import SARGridEnv
from train.networks.qmix import RNNAgent
from swarm_controller.grid_world_transform import GridWorldTransform

def load_forest_grid():
    meta_path = os.path.join(REPO_ROOT, 'worlds', 'forest_sar_meta.json')
    with open(meta_path) as f:
        meta = json.load(f)

    obstacles = meta.get('obstacles', [])
    grid = np.zeros((25, 25), dtype=np.int8)

    METERS_PER_CELL = 8.0
    ORIGIN_X = -180.0
    ORIGIN_Y = -20.0

    for gx in range(25):
        for gy in range(25):
            cell_min_x = ORIGIN_X + gx * METERS_PER_CELL
            cell_max_x = ORIGIN_X + (gx + 1) * METERS_PER_CELL
            cell_min_y = ORIGIN_Y + gy * METERS_PER_CELL
            cell_max_y = ORIGIN_Y + (gy + 1) * METERS_PER_CELL

            for obs in obstacles:
                aabb = obs.get('aabb')
                if not aabb: continue
                if (aabb['max_x'] > cell_min_x and aabb['min_x'] < cell_max_x and
                    aabb['max_y'] > cell_min_y and aabb['min_y'] < cell_max_y):
                    grid[gx, gy] = -1
                    break

    return grid, meta

def manhattan_distance(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

def get_nearest_unexplored_forest(grid, start_pos, target_mask=None):
    unexplored = []
    for i in range(25):
        for j in range(25):
            if grid[i, j] == 0:
                if target_mask is None or (i, j) not in target_mask:
                    unexplored.append((i, j))
    if not unexplored:
        return None
    unexplored.sort(key=lambda p: manhattan_distance(start_pos, p))
    return unexplored[0]

def get_action_towards_forest(start_pos, target_pos, grid):
    x, y = start_pos
    moves = [(0, (x + 1, y)), (1, (x - 1, y)), (2, (x, y + 1)), (3, (x, y - 1))]
    valid_moves = []
    for a, (nx, ny) in moves:
        if 0 <= nx < 25 and 0 <= ny < 25 and grid[nx, ny] != -1:
            valid_moves.append((a, (nx, ny)))
    if not valid_moves:
        return 4
    valid_moves.sort(key=lambda m: manhattan_distance(m[1], target_pos))
    return valid_moves[0][0]

def run_forest_episode(policy: str, seed: int, max_steps: int = 150, agent_net=None, device=None) -> Dict[str, Any]:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)

    grid_template, meta = load_forest_grid()
    n_valid = int(np.sum(grid_template != -1))
    assert n_valid == 422, f"Expected 422 valid cells for forest_sar, got {n_valid}"

    GridWorldTransform.set_bounds(25, 25, 8.0, -180.0, -20.0)

    # Initialize SARGridEnv and override with forest grid
    env = SARGridEnv(num_drones=2, num_victims=3, max_steps=max_steps, seed=seed)
    env.reset()
    env.grid = grid_template.copy()

    # Set spawn pads in Sector Alpha: drone_0=(1, 20), drone_1=(6, 20)
    env.drone_positions = [[1, 20], [6, 20]]

    # Place 3 victims: 1 static, 2 dynamic
    valid_cells = [(x, y) for x in range(25) for y in range(25) if env.grid[x, y] != -1 and (x, y) not in [(1, 20), (6, 20)]]
    chosen = random.sample(valid_cells, 3)
    env.victims = {(c[0], c[1]): 0 for c in chosen}

    victim_state = {
        f"victim_{i}": {
            'grid': list(chosen[i]),
            'world': list(GridWorldTransform.grid_to_world_center(chosen[i][0], chosen[i][1])),
            'is_dynamic': (i > 0),
            'detected': False,
            'detected_step': None,
            'detected_by': None,
            'detection_dist': None
        }
        for i in range(3)
    }

    env._update_fov_and_victims()
    env._update_global_bfs()

    hidden_state = None
    if policy == 'qmix':
        hidden_state = agent_net.init_hidden().expand(2, -1).to(device)

    step = 0
    trajectories = [[] for _ in range(2)]
    for i, p in enumerate(env.drone_positions):
        trajectories[i].append(list(p))

    detection_events = []
    safety_overrides = 0
    inter_drone_dists = []
    detection_radius = 12.0  # 1.5 * 8.0m cell resolution

    while step < max_steps:
        # 1. Move dynamic victims
        for v_id, v_info in victim_state.items():
            if v_info['is_dynamic']:
                vgx, vgy = v_info['grid']
                cand = []
                for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1), (0, 0)]:
                    nvx, nvy = vgx + dx, vgy + dy
                    if 0 <= nvx < 25 and 0 <= nvy < 25 and env.grid[nvx, nvy] != -1:
                        cand.append((nvx, nvy))
                if cand and random.random() < 0.3:
                    new_gx, new_gy = random.choice(cand)
                    v_info['grid'] = [new_gx, new_gy]
                    v_info['world'] = list(GridWorldTransform.grid_to_world_center(new_gx, new_gy))

        # Synchronize dynamic victims to env
        env.victims = {(v['grid'][0], v['grid'][1]): (1 if v['detected'] else 0) for v in victim_state.values()}

        # 2. Select actions
        if policy == 'qmix':
            local_obs = [env.get_agent_state(i) for i in range(2)]
            obs_tensor = torch.FloatTensor(np.array(local_obs)).to(device)
            with torch.no_grad():
                q_vals, hidden_state = agent_net(obs_tensor, hidden_state)
            raw_actions = [q_vals[i].argmax().item() for i in range(2)]

            # Safety shield
            actions = []
            for i in range(2):
                a = raw_actions[i]
                cx, cy = env.drone_positions[i]
                nx, ny = cx, cy
                if a == 0: nx += 1
                elif a == 1: nx -= 1
                elif a == 2: ny += 1
                elif a == 3: ny -= 1

                if nx < 0 or nx >= 25 or ny < 0 or ny >= 25 or env.grid[nx, ny] == -1:
                    actions.append(4)
                    safety_overrides += 1
                else:
                    actions.append(a)

        elif policy == 'frontier':
            actions = []
            target_mask = set()
            for d in range(2):
                pos = env.drone_positions[d]
                target = get_nearest_unexplored_forest(env.grid, pos, target_mask)
                if target:
                    for i in range(-1, 2):
                        for j in range(-1, 2):
                            target_mask.add((target[0] + i, target[1] + j))
                    a = get_action_towards_forest(pos, target, env.grid)
                    actions.append(a)
                else:
                    actions.append(4)

        # 3. Environment step
        _, _, rewards, done, info = env.step(actions)
        step += 1

        for d in range(2):
            trajectories[d].append(list(env.drone_positions[d]))

        # 4. Proximity detection check with 12.0m radius
        for d in range(2):
            dwx, dwy = GridWorldTransform.grid_to_world_center(env.drone_positions[d][0], env.drone_positions[d][1])
            for v_id, v_info in victim_state.items():
                if not v_info['detected']:
                    vwx, vwy = v_info['world']
                    dist = math.hypot(dwx - vwx, dwy - vwy)
                    if dist <= detection_radius:
                        v_info['detected'] = True
                        v_info['detected_step'] = step
                        v_info['detected_by'] = f"drone_{d}"
                        v_info['detection_dist'] = round(dist, 2)
                        detection_events.append({
                            'victim_id': v_id,
                            'detected_by': f"drone_{d}",
                            'step': step,
                            'distance_m': round(dist, 2),
                            'world_x': vwx,
                            'world_y': vwy
                        })

        # Inter-drone distance
        d0_w = GridWorldTransform.grid_to_world_center(env.drone_positions[0][0], env.drone_positions[0][1])
        d1_w = GridWorldTransform.grid_to_world_center(env.drone_positions[1][0], env.drone_positions[1][1])
        inter_drone_dists.append(math.hypot(d0_w[0] - d1_w[0], d0_w[1] - d1_w[1]))

        # Early stop if >= 95% coverage and all victims detected
        n_explored = int(np.sum(env.grid == 1))
        if (n_explored / n_valid) >= 0.95 and all(v['detected'] for v in victim_state.values()):
            break

    n_explored = int(np.sum(env.grid == 1))
    cov_pct = (n_explored / n_valid) * 100.0
    victims_detected = sum(1 for v in victim_state.values() if v['detected'])

    return {
        'policy': policy,
        'seed': seed,
        'steps': step,
        'valid_cells': n_valid,
        'obstacle_cells': int(np.sum(env.grid == -1)),
        'explored_cells': n_explored,
        'coverage_pct': round(cov_pct, 2),
        'victims_detected': victims_detected,
        'total_victims': 3,
        'detection_events': detection_events,
        'safety_overrides': safety_overrides,
        'min_inter_drone_dist_m': round(min(inter_drone_dists), 2) if inter_drone_dists else 0.0,
        'mean_inter_drone_dist_m': round(float(np.mean(inter_drone_dists)), 2) if inter_drone_dists else 0.0
    }

def main():
    output_dir = os.path.join(REPO_ROOT, 'results', 'forest_sar_validation')
    os.makedirs(output_dir, exist_ok=True)

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model_path = os.path.join(REPO_ROOT, 'models', 'qmix_sar_v4_align_best.pth')
    ckpt = torch.load(model_path, map_location=device)

    # Initialize QMIX network with exact input_shape=29
    agent_net = RNNAgent(input_shape=29, rnn_hidden_dim=64, n_actions=5).to(device)
    agent_net.load_state_dict(ckpt['agent_net'])
    agent_net.eval()
    print("[FOREST EVAL] Loaded QMIX checkpoint successfully (input_shape=29).")

    test_seeds = [201, 202, 203, 204, 205]
    forest_results = {'qmix': [], 'frontier': []}

    print("=" * 80)
    print("RUNNING FOREST SAR VALIDATION EXPERIMENT (5 REPRODUCIBLE SEEDS)")
    print("Grid: 25x25 (200m x 200m) | L_cell = 8.0m | r_det = 12.0m | N_valid = 422")
    print("=" * 80)

    for pol in ['qmix', 'frontier']:
        print(f"\n>>> Running Forest Evaluation: {pol.upper()}")
        for s in test_seeds:
            res = run_forest_episode(pol, s, max_steps=150, agent_net=agent_net, device=device)
            forest_results[pol].append(res)
            print(f"  Seed {s}: Cov={res['coverage_pct']:5.2f}% ({res['explored_cells']}/{res['valid_cells']}) | Vict={res['victims_detected']}/{res['total_victims']} | Steps={res['steps']:3d} | MinDist={res['min_inter_drone_dist_m']:5.1f}m | Overrides={res['safety_overrides']}")

    # Save results
    forest_file = os.path.join(output_dir, 'forest_sar_validation_results.json')
    with open(forest_file, 'w') as f:
        json.dump(forest_results, f, indent=4)
    print(f"\n[FOREST EVAL] Detailed results saved to: {forest_file}")

    # Summary table
    print("\n" + "=" * 80)
    print("FOREST SAR SUMMARY COMPARISON")
    print("=" * 80)
    for pol in ['qmix', 'frontier']:
        covs = [r['coverage_pct'] for r in forest_results[pol]]
        victims = [r['victims_detected'] for r in forest_results[pol]]
        overrides = [r['safety_overrides'] for r in forest_results[pol]]
        print(f"[{pol.upper()}] Mean Coverage: {np.mean(covs):.2f}% ± {np.std(covs):.2f}% | Victims Detected: {np.mean(victims):.2f}/3 ({sum(victims)}/15, {sum(victims)/15*100:.1f}%) | Overrides: {sum(overrides)}")

if __name__ == '__main__':
    main()
