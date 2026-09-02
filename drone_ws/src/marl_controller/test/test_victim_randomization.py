import sys
import os
import json
import random

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from marl_controller.state_builder import StateBuilder
from marl_controller.action_mapper import ActionMapper
from marl_controller.evaluator_node import EvaluatorNode

def get_victims_for_episode(base_seed, episode_id):
    # Dummy setup just to run the generation logic
    meta_path = "/home/capstone/capstone_project_antigravity/worlds/generated_world_meta.json"
    with open(meta_path, 'r') as f:
        meta = json.load(f)

    obs_set = set()
    for obs in meta.get("obstacles", []):
        aabb = obs["aabb"]
        for gx in range(25):
            for gy in range(25):
                if (aabb["max_x"] > gx * 4.0) and (aabb["min_x"] < (gx + 1) * 4.0):
                    if (aabb["max_y"] > gy * 4.0) and (aabb["min_y"] < (gy + 1) * 4.0):
                        obs_set.add((gx, gy))
    obstacle_cells = list(obs_set)

    canonical_start = [(2, 2), (12, 2), (22, 2), (2, 12), (12, 15), (22, 12)]

    episode_seed = base_seed + episode_id
    rng = random.Random(episode_seed)

    valid_cells = []
    for gx in range(25):
        for gy in range(25):
            if (gx, gy) not in obstacle_cells and (gx, gy) not in canonical_start:
                valid_cells.append((gx, gy))

    sampled_victims = rng.sample(valid_cells, 5)
    return sampled_victims, len(valid_cells), obstacle_cells, canonical_start

def main():
    base_seed = 100
    print(f"Testing reproducibility with base_seed = {base_seed}")

    # Run 1
    ep1_r1, num_valid, obs, starts = get_victims_for_episode(base_seed, 1)
    ep2_r1, _, _, _ = get_victims_for_episode(base_seed, 2)
    ep3_r1, _, _, _ = get_victims_for_episode(base_seed, 3)

    # Run 2
    ep1_r2, _, _, _ = get_victims_for_episode(base_seed, 1)
    ep2_r2, _, _, _ = get_victims_for_episode(base_seed, 2)
    ep3_r2, _, _, _ = get_victims_for_episode(base_seed, 3)

    print(f"Number of valid candidate cells: {num_valid}")
    print(f"Episode 1 Run 1 victims: {ep1_r1}")
    print(f"Episode 2 Run 1 victims: {ep2_r1}")
    print(f"Episode 3 Run 1 victims: {ep3_r1}")

    # Assertions
    assert ep1_r1 == ep1_r2, "Episode 1 not reproducible!"
    assert ep2_r1 == ep2_r2, "Episode 2 not reproducible!"
    assert ep3_r1 == ep3_r2, "Episode 3 not reproducible!"
    print("PASS: Reproducibility verified.")

    assert ep1_r1 != ep2_r1, "Episode 1 and 2 victims collided!"
    assert ep2_r1 != ep3_r1, "Episode 2 and 3 victims collided!"
    print("PASS: Cross-episode uniqueness verified.")

    # Verification
    for ep, vlist in [(1, ep1_r1), (2, ep2_r1), (3, ep3_r1)]:
        assert len(set(vlist)) == 5, "Not exactly 5 unique victims"
        for v in vlist:
            assert v not in obs, f"Victim {v} in obstacle!"
            assert v not in starts, f"Victim {v} in start cell!"
    print("PASS: All victim rules (uniqueness, not-obstacle, not-start) verified.")

if __name__ == '__main__':
    main()
