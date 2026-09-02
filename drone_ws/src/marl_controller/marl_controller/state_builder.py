"""
ROS 2 observation builder that produces the exact 49D vector expected
by the trained N=6 QMIX policy.

Observation layout (source: SARGridEnv.get_agent_state):
  [0:2]   own_pos        — normalised (x/25, y/25)
  [2:8]   agent_onehot   — 6-element one-hot
  [8:18]  other_pos      — 5 teammates × 2 normalised coords
  [18:28] team_vec       — 5 teammates × 2 direction-normalised deltas
  [28:30] frontier_vec   — BFS next-step direction to nearest unexplored
  [30:31] density        — fraction of unexplored cells in r=5 window
  [31:40] local_obs      — 3×3 obstacle map (row-major, fov_radius=1)
  [40:49] local_exp      — 3×3 explored map (row-major, fov_radius=1)

Grid conventions:
  - 25×25 discrete grid, integer coords 0-24.
  - Cell values: 0 = unexplored, 1 = explored, -1 = obstacle.
  - Gazebo world ↔ grid:  grid_x = world_x + 12,  grid_y = world_y + 12
    (cell_size = 1.0, offset = 12, matching spawn_drones.sh OFFSET=-12).
"""

import numpy as np
from collections import deque


class StateBuilder:
    """
    Builds 49D observations for six drones from their grid positions
    and a shared exploration/obstacle grid.

    This class is intentionally ROS-agnostic: it operates purely on
    grid-level data so it can be unit-tested without a running ROS graph.
    The controller node is responsible for converting Gazebo poses to
    grid coordinates before calling this builder.
    """

    def __init__(
        self,
        num_drones: int = 6,
        x_size: int = 25,
        y_size: int = 25,
        fov_radius: int = 1,
        cell_size: float = 1.0,
        grid_offset: float = 12.0,
    ):
        self.num_drones = num_drones
        self.x_size = x_size
        self.y_size = y_size
        self.fov_radius = fov_radius
        self.cell_size = cell_size
        self.grid_offset = grid_offset

        # Shared grid:  0 = unexplored, 1 = explored, -1 = obstacle
        self.grid: np.ndarray = np.zeros((x_size, y_size), dtype=np.int8)

        # Discrete grid positions for each drone — (x, y) integers
        self.drone_positions: list[tuple[int, int]] = [(0, 0)] * num_drones

        # BFS caches (recomputed via update_bfs)
        self.bfs_dist_map: np.ndarray = np.full((x_size, y_size), np.inf)
        self.bfs_next_step: np.ndarray = np.zeros((x_size, y_size, 2))

    # ── Coordinate conversion ────────────────────────────────────

    def world_to_grid(self, wx: float, wy: float) -> tuple[int, int]:
        """Convert continuous Gazebo world coords to discrete grid cell."""
        gx = int(round(wx / self.cell_size + self.grid_offset))
        gy = int(round(wy / self.cell_size + self.grid_offset))
        gx = max(0, min(self.x_size - 1, gx))
        gy = max(0, min(self.y_size - 1, gy))
        return (gx, gy)

    def grid_to_world(self, gx: int, gy: int) -> tuple[float, float]:
        """Convert discrete grid cell to Gazebo world coords."""
        wx = (gx - self.grid_offset) * self.cell_size
        wy = (gy - self.grid_offset) * self.cell_size
        return (wx, wy)

    # ── State updates ────────────────────────────────────────────

    def reset_episode(self, obstacle_cells: list[tuple[int, int]] | None = None):
        """
        Reset exploration grid and BFS for a new episode.

        Parameters
        ----------
        obstacle_cells : optional list of (gx, gy) tuples
            Static obstacle locations to mark as -1 in the grid.
            If None, the grid starts fully open (no obstacles).
        """
        self.grid = np.zeros((self.x_size, self.y_size), dtype=np.int8)
        if obstacle_cells:
            for gx, gy in obstacle_cells:
                if 0 <= gx < self.x_size and 0 <= gy < self.y_size:
                    self.grid[gx, gy] = -1
        self.drone_positions = [(0, 0)] * self.num_drones
        self.update_bfs()

    def update_drone_positions(
        self,
        positions: list[tuple[int, int]],
    ):
        """
        Update grid positions for all drones and mark cells as explored.

        Parameters
        ----------
        positions : list of (gx, gy) integer tuples, length num_drones
        """
        assert len(positions) == self.num_drones
        self.drone_positions = list(positions)
        for gx, gy in positions:
            if 0 <= gx < self.x_size and 0 <= gy < self.y_size:
                if self.grid[gx, gy] != -1:
                    self.grid[gx, gy] = 1
                # Also mark FOV cells as explored (matching SARGridEnv._update_fov_and_victims)
                for di in range(-self.fov_radius, self.fov_radius + 1):
                    for dj in range(-self.fov_radius, self.fov_radius + 1):
                        cx, cy = gx + di, gy + dj
                        if 0 <= cx < self.x_size and 0 <= cy < self.y_size:
                            if self.grid[cx, cy] != -1:
                                self.grid[cx, cy] = 1

    def update_bfs(self):
        """
        Multi-source BFS from all unexplored cells.
        Exact replica of SARGridEnv._update_global_bfs().
        """
        self.bfs_dist_map = np.full((self.x_size, self.y_size), np.inf)
        self.bfs_next_step = np.zeros((self.x_size, self.y_size, 2))

        unexplored = np.argwhere(self.grid == 0)
        if len(unexplored) == 0:
            return

        queue = deque([tuple(pos) for pos in unexplored])
        for x, y in queue:
            self.bfs_dist_map[x, y] = 0.0

        directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]

        while queue:
            cx, cy = queue.popleft()
            current_dist = self.bfs_dist_map[cx, cy]
            for dx, dy in directions:
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < self.x_size and 0 <= ny < self.y_size:
                    if self.grid[nx, ny] != -1:
                        if current_dist + 1 < self.bfs_dist_map[nx, ny]:
                            self.bfs_dist_map[nx, ny] = current_dist + 1
                            self.bfs_next_step[nx, ny, 0] = float(cx - nx)
                            self.bfs_next_step[nx, ny, 1] = float(cy - ny)
                            queue.append((nx, ny))

    # ── Observation construction ─────────────────────────────────

    def get_agent_state(self, agent_id: int) -> np.ndarray:
        """
        Build the exact 49D observation for one agent.
        Semantically identical to SARGridEnv.get_agent_state().

        Returns
        -------
        np.ndarray of shape (49,), dtype float32
        """
        x, y = self.drone_positions[agent_id]

        # [0:2] Own normalised position
        own_pos = [x / self.x_size, y / self.y_size]

        # [2:8] Agent one-hot
        agent_onehot = [0.0] * self.num_drones
        agent_onehot[agent_id] = 1.0

        # [8:18] Other agents' normalised positions
        other_pos = []
        for i, p in enumerate(self.drone_positions):
            if i != agent_id:
                other_pos.extend([p[0] / self.x_size, p[1] / self.y_size])

        # [18:28] Teammate relative vectors
        team_vec = []
        for i, p in enumerate(self.drone_positions):
            if i != agent_id:
                dx, dy = p[0] - x, p[1] - y
                norm = max(abs(dx) + abs(dy), 1.0)
                team_vec.extend([dx / norm, dy / norm])

        # [28:30] Frontier direction (BFS next-step)
        dist = self.bfs_dist_map[x, y]
        if np.isinf(dist) or dist == 0:
            frontier_vec = [0.0, 0.0]
        else:
            frontier_vec = [
                float(self.bfs_next_step[x, y, 0]),
                float(self.bfs_next_step[x, y, 1]),
            ]

        # [30:31] Local frontier density (r=5, 11×11 window)
        r = 5
        min_x = max(0, x - r)
        max_x = min(self.x_size, x + r + 1)
        min_y = max(0, y - r)
        max_y = min(self.y_size, y + r + 1)
        window = self.grid[min_x:max_x, min_y:max_y]
        density = [float(np.sum(window == 0) / max(1, window.size))]

        # [31:40] Local obstacle map (3×3, fov_radius=1)
        local_obs = []
        # [40:49] Local explored map (3×3, fov_radius=1)
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
                    # Boundary → treated as obstacle
                    local_obs.append(1.0)
                    local_exp.append(0.0)

        state = np.array(
            own_pos + agent_onehot + other_pos + team_vec
            + frontier_vec + density + local_obs + local_exp,
            dtype=np.float32,
        )
        return state

    def get_all_states(self) -> list[np.ndarray]:
        """Build 49D observations for all drones. Returns list of length num_drones."""
        return [self.get_agent_state(i) for i in range(self.num_drones)]