import sys
import os
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from env.sar_env import SARGridEnv

def manhattan_distance(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

def get_nearest_unexplored(env, start_pos, target_mask=None):
    # Find all unexplored cells
    unexplored = []
    for i in range(env.x_size):
        for j in range(env.y_size):
            if env.grid[i, j] == 0:  # 0 is unexplored
                if target_mask is None or (i, j) not in target_mask:
                    unexplored.append((i, j))
                    
    if not unexplored:
        return None
        
    # Sort by distance
    unexplored.sort(key=lambda p: manhattan_distance(start_pos, p))
    return unexplored[0]

def get_action_towards(start_pos, target_pos, env):
    """Greedy action towards target, avoiding buildings if possible"""
    x, y = start_pos
    tx, ty = target_pos
    
    # Possible moves: 0:+X, 1:-X, 2:+Y, 3:-Y, 4:Hover
    moves = [
        (0, (x+1, y)),
        (1, (x-1, y)),
        (2, (x, y+1)),
        (3, (x, y-1))
    ]
    
    # Filter valid moves (inside bounds, not building)
    valid_moves = []
    for a, (nx, ny) in moves:
        if 0 <= nx < env.x_size and 0 <= ny < env.y_size:
            if env.grid[nx, ny] != -1:
                valid_moves.append((a, (nx, ny)))
                
    if not valid_moves:
        return 4 # Hover if completely stuck (shouldn't happen unless boxed in)
        
    # Sort valid moves by distance to target
    valid_moves.sort(key=lambda m: manhattan_distance(m[1], target_pos))
    return valid_moves[0][0]

def run_feasibility_test():
    print("--- DETERMINISTIC COVERAGE FEASIBILITY TEST ---")
    env = SARGridEnv(num_drones=2, max_steps=300, seed=42)
    env.reset()
    
    total_searchable = np.sum(env.grid != -1)
    print(f"Total Searchable Cells: {total_searchable}")
    
    done = False
    step = 0
    
    while not done and step < env.max_steps:
        actions = []
        target_mask = set()
        
        for d in range(env.num_drones):
            pos = env.drone_positions[d]
            target = get_nearest_unexplored(env, pos, target_mask)
            
            if target:
                # Add target and surrounding FOV to mask so other drone doesn't go to same spot
                for i in range(-env.fov_radius, env.fov_radius + 1):
                    for j in range(-env.fov_radius, env.fov_radius + 1):
                        target_mask.add((target[0]+i, target[1]+j))
                        
                a = get_action_towards(pos, target, env)
                actions.append(a)
            else:
                actions.append(4) # Hover if nowhere to go
                
        _, _, _, done, info = env.step(actions)
        step += 1
        
        # Stop early if 100% is reached
        if info['coverage'] >= 0.99:
            break
            
    # Final Report
    metrics = info['metrics']
    explored = np.sum(env.grid == 1)
    
    print("\n--- RESULTS ---")
    print(f"Steps taken: {step} / {env.max_steps}")
    print(f"Explored Cells: {explored} / {total_searchable}")
    print(f"Final Coverage: {info['coverage']*100:.2f}%")
    print(f"Victims Detected: {metrics['victims_detected']}")
    print(f"Building Collisions: {metrics.get('collisions', 0)}") # Approx since drone-drone adds to this too
    print(f"Boundary Collisions: {metrics.get('boundary_collisions', 0)}")
    
    if info['coverage'] >= 0.99:
        print("\nCONCLUSION: SUCCESS. 100% coverage IS physically reachable within 300 steps.")
    else:
        print("\nCONCLUSION: FAILURE. The environment might be too complex or the heuristic got stuck.")

if __name__ == '__main__':
    run_feasibility_test()
