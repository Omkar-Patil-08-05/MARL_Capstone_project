#!/usr/bin/env python3
"""
Multi-Seed Evaluation and Fair Baseline Comparison Benchmark.
Evaluates QMIX (checkpoint: models/qmix_sar_v4_align_best.pth) against
Frontier-Search (Heuristic) and Random Walk baselines across 10 deterministic seeds
on the exact realistic_sar obstacle map (377 valid cells, 248 obstacles).
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

# Add repository root and marl_drone_project to path
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(REPO_ROOT)
sys.path.append(os.path.join(REPO_ROOT, 'marl_drone_project'))

from env.sar_env import SARGridEnv
from train.networks.qmix import RNNAgent

def manhattan_distance(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

def euclidean_distance(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def get_nearest_unexplored(env: SARGridEnv, start_pos: Tuple[int, int], target_mask=None):
    unexplored = []
    for i in range(env.x_size):
        for j in range(env.y_size):
            if env.grid[i, j] == 0:  # 0 is unexplored
                if target_mask is None or (i, j) not in target_mask:
                    unexplored.append((i, j))
    if not unexplored:
        return None
    unexplored.sort(key=lambda p: manhattan_distance(start_pos, p))
    return unexplored[0]

def get_action_towards(start_pos: Tuple[int, int], target_pos: Tuple[int, int], env: SARGridEnv) -> int:
    x, y = start_pos
    moves = [
        (0, (x + 1, y)),
        (1, (x - 1, y)),
        (2, (x, y + 1)),
        (3, (x, y - 1))
    ]
    valid_moves = []
    for a, (nx, ny) in moves:
        if 0 <= nx < env.x_size and 0 <= ny < env.y_size:
            if env.grid[nx, ny] != -1:
                valid_moves.append((a, (nx, ny)))
    if not valid_moves:
        return 4  # Hover
    valid_moves.sort(key=lambda m: manhattan_distance(m[1], target_pos))
    return valid_moves[0][0]

def run_single_episode(policy_type: str, seed: int, max_steps: int = 150, agent_net=None, device=None) -> Dict[str, Any]:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)

    env = SARGridEnv(num_drones=2, num_victims=3, max_steps=max_steps, seed=seed)
    env.curriculum_stage = 3  # Loads realistic_sar metadata obstacles
    env.reset()

    # Verify obstacle count matches realistic_sar
    n_obstacles = int(np.sum(env.grid == -1))
    n_valid = int(np.sum(env.grid != -1))
    assert n_valid == 377, f"Expected 377 valid cells, got {n_valid}"

    hidden_state = None
    if policy_type == 'qmix':
        hidden_state = agent_net.init_hidden().expand(env.num_drones, -1).to(device)

    trajectories = [[] for _ in range(env.num_drones)]
    for i, p in enumerate(env.drone_positions):
        trajectories[i].append((p[0], p[1]))

    step_latencies = []
    first_victim_step = None
    all_victims_step = None
    safety_overrides = 0
    inter_drone_dists = []
    cell_collisions = 0

    done = False
    step = 0
    total_victims = len(env.victims)

    while not done and step < max_steps:
        t0 = time.perf_counter()
        actions = []

        if policy_type == 'qmix':
            local_obs = [env.get_agent_state(i) for i in range(env.num_drones)]
            obs_tensor = torch.FloatTensor(np.array(local_obs)).to(device)
            with torch.no_grad():
                q_vals, hidden_state = agent_net(obs_tensor, hidden_state)
            raw_actions = [q_vals[i].argmax().item() for i in range(env.num_drones)]
            lat_ms = (time.perf_counter() - t0) * 1000.0
            step_latencies.append(lat_ms)

            # Apply deterministic multi-agent safety shield (same as qmix_mission_controller)
            proposed = []
            for i in range(env.num_drones):
                a = raw_actions[i]
                gx, gy = env.drone_positions[i]
                tgx, tgy = gx, gy
                if a == 0: tgx += 1
                elif a == 1: tgx -= 1
                elif a == 2: tgy += 1
                elif a == 3: tgy -= 1

                # Bounds check
                if tgx < 0 or tgx >= env.x_size or tgy < 0 or tgy >= env.y_size or env.grid[tgx, tgy] == -1:
                    proposed.append((gx, gy, 4, True))
                else:
                    proposed.append((tgx, tgy, a, False))

            # Swap conflict check
            final_actions = []
            for i in range(env.num_drones):
                other = 1 - i
                tgx_i, tgy_i, act_i, over_i = proposed[i]
                tgx_o, tgy_o, act_o, over_o = proposed[other]
                curr_i = env.drone_positions[i]
                curr_o = env.drone_positions[other]

                if (tgx_i, tgy_i) == (curr_o[0], curr_o[1]) and (tgx_o, tgy_o) == (curr_i[0], curr_i[1]):
                    final_actions.append(4)  # Hover
                    safety_overrides += 1
                elif tgx_i == tgx_o and tgy_i == tgy_o and i > other:
                    final_actions.append(4)  # Priority to drone 0
                    safety_overrides += 1
                else:
                    if over_i:
                        safety_overrides += 1
                    final_actions.append(act_i)
            actions = final_actions

        elif policy_type == 'frontier':
            target_mask = set()
            for d in range(env.num_drones):
                pos = env.drone_positions[d]
                target = get_nearest_unexplored(env, pos, target_mask)
                if target:
                    for i in range(-env.fov_radius, env.fov_radius + 1):
                        for j in range(-env.fov_radius, env.fov_radius + 1):
                            target_mask.add((target[0] + i, target[1] + j))
                    a = get_action_towards(pos, target, env)
                    actions.append(a)
                else:
                    actions.append(4)
            lat_ms = (time.perf_counter() - t0) * 1000.0
            step_latencies.append(lat_ms)

        elif policy_type == 'random':
            for d in range(env.num_drones):
                # Sample valid random move
                pos = env.drone_positions[d]
                valid_a = []
                for a_cand, (nx, ny) in [(0, (pos[0]+1, pos[1])), (1, (pos[0]-1, pos[1])), (2, (pos[0], pos[1]+1)), (3, (pos[0], pos[1]-1)), (4, pos)]:
                    if 0 <= nx < env.x_size and 0 <= ny < env.y_size and env.grid[nx, ny] != -1:
                        valid_a.append(a_cand)
                actions.append(random.choice(valid_a) if valid_a else 4)
            lat_ms = (time.perf_counter() - t0) * 1000.0
            step_latencies.append(lat_ms)

        # Step environment
        prev_victims = env.metrics['victims_detected']
        _, _, rewards, done, info = env.step(actions)
        step += 1

        curr_victims = env.metrics['victims_detected']
        if curr_victims > 0 and first_victim_step is None:
            first_victim_step = step
        if curr_victims == total_victims and all_victims_step is None:
            all_victims_step = step

        # Record trajectory and inter-drone distance
        p0, p1 = env.drone_positions[0], env.drone_positions[1]
        trajectories[0].append((p0[0], p0[1]))
        trajectories[1].append((p1[0], p1[1]))

        dist_cells = euclidean_distance(p0, p1)
        inter_drone_dists.append(dist_cells * 4.0)  # in meters
        if p0 == p1:
            cell_collisions += 1

    # Path lengths (in meters)
    path_lengths = []
    for d_traj in trajectories:
        plen = 0.0
        for k in range(1, len(d_traj)):
            plen += euclidean_distance(d_traj[k-1], d_traj[k]) * 4.0
        path_lengths.append(plen)

    explored_cells = int(np.sum(env.grid == 1))
    coverage_pct = (explored_cells / n_valid) * 100.0
    victims_found = env.metrics['victims_detected']

    return {
        'policy': policy_type,
        'seed': seed,
        'steps_executed': step,
        'explored_cells': explored_cells,
        'valid_cells': n_valid,
        'coverage_pct': round(coverage_pct, 2),
        'victims_found': victims_found,
        'total_victims': total_victims,
        'first_victim_step': first_victim_step,
        'all_victims_step': all_victims_step,
        'mission_complete': (victims_found == total_victims and coverage_pct >= 85.0),
        'path_length_drone_0_m': round(path_lengths[0], 2),
        'path_length_drone_1_m': round(path_lengths[1], 2),
        'total_path_length_m': round(sum(path_lengths), 2),
        'min_inter_drone_dist_m': round(min(inter_drone_dists), 2) if inter_drone_dists else 0.0,
        'mean_inter_drone_dist_m': round(float(np.mean(inter_drone_dists)), 2) if inter_drone_dists else 0.0,
        'collisions': cell_collisions,
        'safety_overrides': safety_overrides,
        'mean_latency_ms': round(float(np.mean(step_latencies)), 2) if step_latencies else 0.0,
        'max_latency_ms': round(float(np.max(step_latencies)), 2) if step_latencies else 0.0
    }

def main():
    output_dir = os.path.join(REPO_ROOT, 'results', 'multi_seed_evaluation')
    os.makedirs(output_dir, exist_ok=True)

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"[EVAL] Target execution device: {device}")

    # Load trained QMIX agent
    model_path = os.path.join(REPO_ROOT, 'models', 'qmix_sar_v4_align_best.pth')
    assert os.path.exists(model_path), f"Checkpoint not found at {model_path}"
    ckpt = torch.load(model_path, map_location=device)

    # Temporary env to extract observation dimension
    temp_env = SARGridEnv(num_drones=2, num_victims=3, max_steps=150)
    temp_env.curriculum_stage = 3
    temp_env.reset()
    obs_dim = len(temp_env.get_agent_state(0))
    n_actions = temp_env.action_space

    agent_net = RNNAgent(input_shape=obs_dim, rnn_hidden_dim=64, n_actions=n_actions).to(device)
    agent_net.load_state_dict(ckpt['agent_net'])
    agent_net.eval()
    print(f"[EVAL] Successfully loaded QMIX checkpoint: {model_path}")

    # Evaluation seeds: 10 independent reproducible seeds
    evaluation_seeds = [101, 102, 103, 104, 105, 106, 107, 108, 109, 110]
    policies = ['qmix', 'frontier', 'random']

    all_results = {p: [] for p in policies}

    print(f"\n{'='*90}")
    print(f"BENCHMARKING 3 POLICIES ACROSS 10 IDENTICAL SEEDS ON realistic_sar (N_valid = 377)")
    print(f"{'='*90}")

    for policy in policies:
        print(f"\n>>> Running evaluation for: {policy.upper()} POLICY")
        for seed in evaluation_seeds:
            res = run_single_episode(policy, seed, max_steps=150, agent_net=agent_net, device=device)
            all_results[policy].append(res)
            print(f"  Seed {seed:3d}: Cov={res['coverage_pct']:5.2f}% | Vict={res['victims_found']}/{res['total_victims']} | Steps={res['steps_executed']:3d} | 1st_V={str(res['first_victim_step']):>4s} | All_V={str(res['all_victims_step']):>4s} | MinDist={res['min_inter_drone_dist_m']:5.2f}m | Lat={res['mean_latency_ms']:4.2f}ms")

    # Save detailed episode-level JSON
    raw_results_file = os.path.join(output_dir, 'multi_seed_raw_episodes.json')
    with open(raw_results_file, 'w') as f:
        json.dump(all_results, f, indent=4)
    print(f"\n[EVAL] Raw episode-level results saved to: {raw_results_file}")

    # Compute aggregate metrics
    summary = {}
    for policy, ep_list in all_results.items():
        covs = [e['coverage_pct'] for e in ep_list]
        victims = [e['victims_found'] for e in ep_list]
        total_v = ep_list[0]['total_victims']
        first_steps = [e['first_victim_step'] for e in ep_list if e['first_victim_step'] is not None]
        all_steps = [e['all_victims_step'] for e in ep_list if e['all_victims_step'] is not None]
        path_lens = [e['total_path_length_m'] for e in ep_list]
        min_dists = [e['min_inter_drone_dist_m'] for e in ep_list]
        overrides = [e['safety_overrides'] for e in ep_list]
        latencies = [e['mean_latency_ms'] for e in ep_list]
        completed = [e['mission_complete'] for e in ep_list]

        summary[policy] = {
            'episodes_count': len(ep_list),
            'coverage_mean': round(float(np.mean(covs)), 2),
            'coverage_std': round(float(np.std(covs)), 2),
            'coverage_min': round(float(np.min(covs)), 2),
            'coverage_max': round(float(np.max(covs)), 2),
            'victims_found_mean': round(float(np.mean(victims)), 2),
            'victim_detection_rate_pct': round((sum(victims) / (len(ep_list) * total_v)) * 100.0, 2),
            'all_victims_found_rate_pct': round((sum(1 for v in victims if v == total_v) / len(ep_list)) * 100.0, 2),
            'mean_steps_to_first_victim': round(float(np.mean(first_steps)), 2) if first_steps else None,
            'mean_steps_to_all_victims': round(float(np.mean(all_steps)), 2) if all_steps else None,
            'mission_completion_rate_pct': round((sum(completed) / len(ep_list)) * 100.0, 2),
            'mean_total_path_length_m': round(float(np.mean(path_lens)), 2),
            'mean_min_inter_drone_dist_m': round(float(np.mean(min_dists)), 2),
            'total_safety_overrides': sum(overrides),
            'mean_safety_overrides_per_ep': round(float(np.mean(overrides)), 2),
            'mean_inference_latency_ms': round(float(np.mean(latencies)), 2)
        }

    summary_file = os.path.join(output_dir, 'multi_seed_summary.json')
    with open(summary_file, 'w') as f:
        json.dump(summary, f, indent=4)
    print(f"[EVAL] Summary metrics saved to: {summary_file}")

    # Print markdown table
    print(f"\n{'='*90}")
    print("AGGREGATE BENCHMARK RESULTS (10 EPISODES PER CONDITION, 150 STEPS BUDGET)")
    print(f"{'='*90}")
    print(f"{'Metric':<32} | {'QMIX (Trained)':<16} | {'Frontier Search':<16} | {'Random Walk':<16}")
    print("-" * 90)
    print(f"{'Mean Coverage (%)':<32} | {summary['qmix']['coverage_mean']:>6.2f} ± {summary['qmix']['coverage_std']:<6.2f} | {summary['frontier']['coverage_mean']:>6.2f} ± {summary['frontier']['coverage_std']:<6.2f} | {summary['random']['coverage_mean']:>6.2f} ± {summary['random']['coverage_std']:<6.2f}")
    print(f"{'Coverage Range [Min, Max] (%)':<32} | [{summary['qmix']['coverage_min']}, {summary['qmix']['coverage_max']}]% | [{summary['frontier']['coverage_min']}, {summary['frontier']['coverage_max']}]% | [{summary['random']['coverage_min']}, {summary['random']['coverage_max']}]%")
    print(f"{'Mean Victims Detected (/ 3)':<32} | {summary['qmix']['victims_found_mean']:>14.2f} | {summary['frontier']['victims_found_mean']:>14.2f} | {summary['random']['victims_found_mean']:>14.2f}")
    print(f"{'Victim Detection Rate (%)':<32} | {summary['qmix']['victim_detection_rate_pct']:>13.2f}% | {summary['frontier']['victim_detection_rate_pct']:>13.2f}% | {summary['random']['victim_detection_rate_pct']:>13.2f}%")
    print(f"{'All-Victims Intercepted (%)':<32} | {summary['qmix']['all_victims_found_rate_pct']:>13.2f}% | {summary['frontier']['all_victims_found_rate_pct']:>13.2f}% | {summary['random']['all_victims_found_rate_pct']:>13.2f}%")
    print(f"{'Steps to 1st Victim (mean)':<32} | {str(summary['qmix']['mean_steps_to_first_victim']):>14s} | {str(summary['frontier']['mean_steps_to_first_victim']):>14s} | {str(summary['random']['mean_steps_to_first_victim']):>14s}")
    print(f"{'Steps to All Victims (mean)':<32} | {str(summary['qmix']['mean_steps_to_all_victims']):>14s} | {str(summary['frontier']['mean_steps_to_all_victims']):>14s} | {str(summary['random']['mean_steps_to_all_victims']):>14s}")
    print(f"{'Mission Completion Rate (%)':<32} | {summary['qmix']['mission_completion_rate_pct']:>13.2f}% | {summary['frontier']['mission_completion_rate_pct']:>13.2f}% | {summary['random']['mission_completion_rate_pct']:>13.2f}%")
    print(f"{'Mean Total Path Length (m)':<32} | {summary['qmix']['mean_total_path_length_m']:>14.1f} | {summary['frontier']['mean_total_path_length_m']:>14.1f} | {summary['random']['mean_total_path_length_m']:>14.1f}")
    print(f"{'Mean Min Inter-Drone Dist (m)':<32} | {summary['qmix']['mean_min_inter_drone_dist_m']:>14.2f} | {summary['frontier']['mean_min_inter_drone_dist_m']:>14.2f} | {summary['random']['mean_min_inter_drone_dist_m']:>14.2f}")
    print(f"{'Shield Overrides (Total / Ep)':<32} | {summary['qmix']['total_safety_overrides']:>6d} ({summary['qmix']['mean_safety_overrides_per_ep']}/ep) | {summary['frontier']['total_safety_overrides']:>6d} (0/ep) | {summary['random']['total_safety_overrides']:>6d} (0/ep)")
    print(f"{'Decision Latency (ms)':<32} | {summary['qmix']['mean_inference_latency_ms']:>14.2f} | {summary['frontier']['mean_inference_latency_ms']:>14.2f} | {summary['random']['mean_inference_latency_ms']:>14.2f}")
    print(f"{'='*90}")

if __name__ == '__main__':
    main()
