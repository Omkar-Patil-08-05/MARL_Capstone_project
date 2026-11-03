import numpy as np
import json
import os
from collections import deque

class SARGridEnv:
    """
    Lightweight MARL environment for Search and Rescue (SAR).
    - Configurable grid (default 25x25).
    - 2D movement (fixed cruise altitude).
    - Structured building generation.
    - Static victims with downward FOV detection.
    - Compact global state for QMIX hypernetworks.
    """
    def __init__(self, x_size=25, y_size=25, num_drones=2, fov_size=3, max_steps=300, num_victims=5, seed=None):
        self.x_size = x_size
        self.y_size = y_size
        self.num_drones = num_drones
        self.fov_radius = fov_size // 2
        self.max_steps = max_steps
        self.num_victims = num_victims
        self.curriculum_stage = 1
        
        if seed is not None:
            np.random.seed(seed)
            
        self.grid = np.zeros((self.x_size, self.y_size), dtype=np.int8)           # 0: unexplored, 1: explored, -1: obstacle
        self.victims = {}        # dict: (x, y) -> status (0: hidden, 1: detected)
        self.drone_positions = []
        self.drone_coverage = [set() for _ in range(self.num_drones)]
        self.steps_since_progress = [0] * self.num_drones
        self.last_frontier_dist = [0.0] * self.num_drones
        self.bfs_dist_map = np.full((self.x_size, self.y_size), np.inf)
        self.bfs_next_step = np.zeros((self.x_size, self.y_size, 2))
        
        # 5 actions: +X, -X, +Y, -Y, Hover
        self.action_space = 5 
        self.current_step = 0
        self.metrics = {}
        self.all_victims_bonus_granted = False
        
    def reset(self, episode_num=None):
        if episode_num is not None:
            if episode_num < 500:
                self.curriculum_stage = 1
            elif episode_num < 1500:
                self.curriculum_stage = 2
            else:
                self.curriculum_stage = 3
                
        self.current_step = 0
        self.grid = np.zeros((self.x_size, self.y_size), dtype=np.int8)
        self.all_victims_bonus_granted = False
        
        self.metrics = {
            'collisions': 0,
            'boundary_collisions': 0,
            'near_collisions': 0,
            'victims_detected': 0,
            'redundant_steps': 0,
            'mission_time': 0,
            'hover_count': 0,
            'bfs_progress': 0.0
        }
        
        self.drone_coverage = [set() for _ in range(self.num_drones)]
        self.steps_since_progress = [0] * self.num_drones
        self.last_frontier_dist = [0.0] * self.num_drones

        
        self._generate_buildings()
        self._place_victims()
        self._spawn_drones()
        
        self._update_fov_and_victims()
        self._update_global_bfs()
        self.last_frontier_dist = [self._get_bfs_frontier_target(pos)[0] for pos in self.drone_positions]
        
        return self.get_global_state()
        
    def _update_global_bfs(self):
        """
        Runs a multi-source BFS from all unexplored cells to compute the shortest path
        and next-step directions for the entire grid.
        """
        self.bfs_dist_map = np.full((self.x_size, self.y_size), np.inf)
        self.bfs_next_step = np.zeros((self.x_size, self.y_size, 2))
        
        unexplored = np.argwhere(self.grid == 0)
        if len(unexplored) == 0:
            return
            
        queue = deque([tuple(pos) for pos in unexplored])
        
        for (x, y) in queue:
            self.bfs_dist_map[x, y] = 0.0
            
        directions = [(1,0), (-1,0), (0,1), (0,-1)]
        
        while queue:
            cx, cy = queue.popleft()
            current_dist = self.bfs_dist_map[cx, cy]
            
            for dx, dy in directions:
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < self.x_size and 0 <= ny < self.y_size:
                    if self.grid[nx, ny] != -1:
                        if current_dist + 1 < self.bfs_dist_map[nx, ny]:
                            self.bfs_dist_map[nx, ny] = current_dist + 1
                            # The next step from (nx, ny) to reach the target is (cx, cy)
                            # So the vector is (cx - nx, cy - ny)
                            self.bfs_next_step[nx, ny, 0] = float(cx - nx)
                            self.bfs_next_step[nx, ny, 1] = float(cy - ny)
                            queue.append((nx, ny))

    def _get_bfs_frontier_target(self, start_pos):
        """
        Returns (bfs_distance, dx, dy) to the nearest reachable unexplored cell
        using the cached global BFS maps.
        """
        sx, sy = start_pos
        dist = self.bfs_dist_map[sx, sy]
        if np.isinf(dist) or dist == 0:
            return 0.0, 0.0, 0.0
            
        dx = self.bfs_next_step[sx, sy, 0]
        dy = self.bfs_next_step[sx, sy, 1]
        return float(dist), float(dx), float(dy)
        
    def _generate_buildings(self):
        if self.curriculum_stage == 3:
            # Realistic map
            meta_path = os.path.join(os.getcwd(), 'worlds', 'generated_world_meta.json')
            if os.path.exists(meta_path):
                with open(meta_path, 'r') as f:
                    meta = json.load(f)
                for obs in meta.get("obstacles", []):
                    # AABB intersection
                    aabb = obs.get("aabb", {})
                    min_wx, max_wx = aabb.get("min_x", 0), aabb.get("max_x", 0)
                    min_wy, max_wy = aabb.get("min_y", 0), aabb.get("max_y", 0)
                    
                    for gx in range(self.x_size):
                        for gy in range(self.y_size):
                            cell_min_x, cell_max_x = gx * 4.0, (gx + 1) * 4.0
                            cell_min_y, cell_max_y = gy * 4.0, (gy + 1) * 4.0
                            
                            # AABB overlap check
                            overlap_x = max_wx > cell_min_x and min_wx < cell_max_x
                            overlap_y = max_wy > cell_min_y and min_wy < cell_max_y
                            if overlap_x and overlap_y:
                                self.grid[gx, gy] = -1
                return
            else:
                print("WARNING: Meta file not found, falling back to Stage 2.")
                
        # Stage 1 and Stage 2
        num_blocks = max(1, (self.x_size * self.y_size) // 150) if self.curriculum_stage == 1 else (self.x_size * self.y_size) // 40
        for _ in range(num_blocks):
            bx = np.random.randint(2, self.x_size - 4)
            by = np.random.randint(2, self.y_size - 4)
            bw = np.random.randint(2, 5)
            bh = np.random.randint(2, 5)
            
            for i in range(bx, bx + bw):
                for j in range(by, by + bh):
                    if i < self.x_size and j < self.y_size:
                        self.grid[i, j] = -1
                        
    def _place_victims(self):
        self.victims = {}
        placed = 0
        while placed < self.num_victims:
            vx = np.random.randint(0, self.x_size)
            vy = np.random.randint(0, self.y_size)
            if self.grid[vx, vy] != -1 and (vx, vy) not in self.victims:
                self.victims[(vx, vy)] = 0
                placed += 1
                
    def _spawn_drones(self):
        self.drone_positions = []
        for _ in range(self.num_drones):
            while True:
                x = np.random.randint(0, self.x_size)
                y = np.random.randint(0, self.y_size)
                if self.grid[x, y] != -1 and [x, y] not in self.drone_positions:
                    self.drone_positions.append([x, y])
                    break
                    
    def _update_fov_and_victims(self):
        """Updates the grid based on the drones' FOV. Returns (new_cells_per_drone, new_victims_per_drone)."""
        new_cells = [0] * self.num_drones
        new_victims = [0] * self.num_drones
        
        for d in range(self.num_drones):
            dx, dy = self.drone_positions[d]
            for i in range(-self.fov_radius, self.fov_radius + 1):
                for j in range(-self.fov_radius, self.fov_radius + 1):
                    cx, cy = dx + i, dy + j
                    if 0 <= cx < self.x_size and 0 <= cy < self.y_size:
                        if self.grid[cx, cy] != -1:  # don't explore obstacles
                            if self.grid[cx, cy] == 0:
                                self.grid[cx, cy] = 1
                                self.drone_coverage[d].add((cx, cy))
                                new_cells[d] += 1
                                
                            # Check for victim detection
                            if (cx, cy) in self.victims and self.victims[(cx, cy)] == 0:
                                self.victims[(cx, cy)] = 1
                                new_victims[d] += 1
                                self.metrics['victims_detected'] += 1
                                
        return new_cells, new_victims

    def step(self, actions):
        """
        Takes discrete actions for each drone. 
        Returns global_state, local_states, rewards, done, info.
        """
        rewards = [0.0] * self.num_drones
        new_positions = []
        
        for i in range(self.num_drones):
            x, y = self.drone_positions[i]
            nx, ny = x, y
            
            a = actions[i]
            hit_boundary = False
            
            if a == 0:
                if x+1 < self.x_size:
                    nx = x+1
                else:
                    hit_boundary = True
            elif a == 1:
                if x-1 >= 0:
                    nx = x-1
                else:
                    hit_boundary = True
            elif a == 2:
                if y+1 < self.y_size:
                    ny = y+1
                else:
                    hit_boundary = True
            elif a == 3:
                if y-1 >= 0:
                    ny = y-1
                else:
                    hit_boundary = True
            elif a == 4:
                self.metrics['hover_count'] += 1
            
            if hit_boundary:
                rewards[i] -= 10.0
                self.metrics['boundary_collisions'] += 1
            
            # Obstacle collision check
            if not hit_boundary and self.grid[nx, ny] == -1:
                nx, ny = x, y
                rewards[i] -= 10.0
                self.metrics['collisions'] += 1
                
            new_positions.append([nx, ny])
            
        # Drone-Drone Collision & Near-Collision Check
        for i in range(self.num_drones):
            for j in range(i+1, self.num_drones):
                dist = abs(new_positions[i][0] - new_positions[j][0]) + abs(new_positions[i][1] - new_positions[j][1])
                if dist == 0:
                    # Drone-Drone Collision
                    new_positions[i] = self.drone_positions[i]
                    new_positions[j] = self.drone_positions[j]
                    rewards[i] -= 20.0
                    rewards[j] -= 20.0
                    self.metrics['collisions'] += 2
                elif dist == 1 or dist == 2:
                    # Near Collision penalty
                    rewards[i] -= 2.0
                    rewards[j] -= 2.0
                    self.metrics['near_collisions'] += 2
                    
        # Update positions physically
        self.drone_positions = new_positions
        
        # Apply FOV
        new_cells, new_victims = self._update_fov_and_victims()
        
        if sum(new_cells) > 0:
            self._update_global_bfs()
        
        # Evaluate Step Reward
        for i in range(self.num_drones):
            rewards[i] -= 0.1  # Mission time penalty
            
            # Track BFS frontier dist for shaping
            current_dist, _, _ = self._get_bfs_frontier_target(self.drone_positions[i])
            
            if new_cells[i] > 0:
                rewards[i] += 1.0 * new_cells[i] # Coverage reward
                self.steps_since_progress[i] = 0
            else:
                # Potential-based shaping towards nearest BFS frontier
                if current_dist > 0 and self.last_frontier_dist[i] > 0:
                    delta = self.last_frontier_dist[i] - current_dist
                    rewards[i] += 0.2 * delta  # Reward for moving closer to frontier along BFS path
                    if delta > 0:
                        self.metrics['bfs_progress'] += delta
                
                # Stagnation penalty
                self.steps_since_progress[i] += 1
                if self.steps_since_progress[i] > 15:
                    rewards[i] -= 0.5
                
                self.metrics['redundant_steps'] += 1
                
            self.last_frontier_dist[i] = current_dist
            
            # Cooperative Partitioning Incentive
            if self.num_drones > 1:
                # Find the minimum Manhattan distance to any teammate
                min_team_dist = float('inf')
                for j in range(self.num_drones):
                    if i != j:
                        dist = abs(self.drone_positions[i][0] - self.drone_positions[j][0]) + abs(self.drone_positions[i][1] - self.drone_positions[j][1])
                        if dist < min_team_dist:
                            min_team_dist = dist
                
                if min_team_dist >= 8:
                    rewards[i] += 0.05
                
            if new_victims[i] > 0:
                rewards[i] += 50.0 * new_victims[i] # Victim discovery reward
                
        self.current_step += 1
        self.metrics['mission_time'] = self.current_step
        
        # Check Episode Termination
        valid_cells = np.sum(self.grid != -1)
        explored_cells = np.sum(self.grid == 1)
        coverage = explored_cells / valid_cells if valid_cells > 0 else 0
        
        # All victims one-time bonus
        if self.metrics['victims_detected'] == self.num_victims and not self.all_victims_bonus_granted:
            self.all_victims_bonus_granted = True
            for i in range(self.num_drones):
                rewards[i] += 50.0
        
        done = False
        if coverage >= 0.99:
            done = True
            for i in range(self.num_drones):
                rewards[i] += 200.0 
        elif self.current_step >= self.max_steps:
            done = True
            
        global_state = self.get_global_state()
        local_states = [self.get_agent_state(i) for i in range(self.num_drones)]
        
        info = {
            'coverage': coverage,
            'metrics': self.metrics.copy()
        }
            
        return global_state, local_states, rewards, done, info
        
    def get_agent_state(self, agent_id):
        """
        Creates a decentralized local observation for the specified agent.
        Shape scales linearly with number of drones and fixed FOV size.
        """
        x, y = self.drone_positions[agent_id]
        
        # 1. Own normalized position (2)
        own_pos = [x / self.x_size, y / self.y_size]
        
        # 2. Agent ID (One-Hot) (num_drones)
        agent_onehot = [0.0] * self.num_drones
        agent_onehot[agent_id] = 1.0
        
        # 3. Other agents' normalized positions ( (num_drones-1) * 2 )
        other_pos = []
        for i, p in enumerate(self.drone_positions):
            if i != agent_id:
                other_pos.extend([p[0] / self.x_size, p[1] / self.y_size])
                
        # NEW: Teammate Relative Vector ( (num_drones-1) * 2 )
        team_vec = []
        for i, p in enumerate(self.drone_positions):
            if i != agent_id:
                dx, dy = p[0] - x, p[1] - y
                norm = max(abs(dx) + abs(dy), 1.0)
                team_vec.extend([dx / norm, dy / norm])
                
        # NEW: Frontier Direction (2) - updated to BFS
        _, fx, fy = self._get_bfs_frontier_target((x, y))
        frontier_vec = [fx, fy]
        
        # NEW: Local Frontier Density (1)
        r = 5
        min_x, max_x = max(0, x-r), min(self.x_size, x+r+1)
        min_y, max_y = max(0, y-r), min(self.y_size, y+r+1)
        window = self.grid[min_x:max_x, min_y:max_y]
        density = [float(np.sum(window == 0) / max(1, window.size))]
                
        # 4. Local FOV representation (obstacles and explored status)
        local_obs = []
        local_exp = []
        
        for i in range(-self.fov_radius, self.fov_radius + 1):
            for j in range(-self.fov_radius, self.fov_radius + 1):
                cx, cy = x + i, y + j
                if 0 <= cx < self.x_size and 0 <= cy < self.y_size:
                    val = self.grid[cx, cy]
                    if val == -1:
                        local_obs.append(1.0)
                        local_exp.append(0.0)
                    else:
                        local_obs.append(0.0)
                        local_exp.append(float(val))
                else:
                    # Treat grid boundary as an obstacle in FOV
                    local_obs.append(1.0)
                    local_exp.append(0.0)
                    
        state = np.array(own_pos + agent_onehot + other_pos + team_vec + frontier_vec + density + local_obs + local_exp, dtype=np.float32)
        return state
        
    def get_global_state(self):
        """
        Creates a compact global state strictly for the QMIX mixer hypernetworks.
        Instead of a massive full-grid flatten, it extracts the exact locations.
        """
        drone_pos = []
        for x, y in self.drone_positions:
            drone_pos.extend([x / self.x_size, y / self.y_size])
            
        victim_pos = []
        for (vx, vy), status in self.victims.items():
            victim_pos.extend([vx / self.x_size, vy / self.y_size, float(status)])
            
        valid_cells = np.sum(self.grid != -1)
        explored_cells = np.sum(self.grid == 1)
        coverage = explored_cells / valid_cells if valid_cells > 0 else 0.0
        
        global_state = np.concatenate([
            np.array(drone_pos, dtype=np.float32),
            np.array(victim_pos, dtype=np.float32),
            np.array([coverage], dtype=np.float32)
        ])
        
        return global_state
