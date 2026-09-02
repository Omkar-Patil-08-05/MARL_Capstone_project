"""
Action mapper for translating discrete QMIX actions to Gazebo velocity commands.

Provides a clean abstraction layer between the policy's discrete action space
and the physical Gazebo VelocityControl plugin.  The mapper converts each
action index into a planar Twist command suitable for publishing on the
per-drone `/model/<name>/cmd_vel` Gazebo topic via the ros_gz_bridge.

Phase 1 verified interface:
  - Plugin: gz::sim::systems::VelocityControl
  - Topic:  /model/<drone_name>/cmd_vel
  - Msg:    gz.msgs.Twist  (bridged as geometry_msgs/msg/Twist)

The mapper does NOT implement a full position controller yet.  It provides
the velocity vector for each action and exposes a `get_target_grid_cell()`
helper so that the controller node (Phase 5) can later implement the
cell-to-cell motion loop.
"""

from geometry_msgs.msg import Twist


class ActionMapper:
    """
    Translates discrete QMIX actions into bounded planar Twist commands.

    Action semantics (matching SARGridEnv):
        0 → +X
        1 → −X
        2 → +Y
        3 → −Y
        4 → Hover (zero velocity)
    """

    ACTION_PLUS_X  = 0
    ACTION_MINUS_X = 1
    ACTION_PLUS_Y  = 2
    ACTION_MINUS_Y = 3
    ACTION_HOVER   = 4

    # Grid-cell deltas indexed by action
    _DX = {0: +1, 1: -1, 2:  0, 3:  0, 4: 0}
    _DY = {0:  0, 1:  0, 2: +1, 3: -1, 4: 0}

    def __init__(
        self,
        max_speed: float = 2.0,
        grid_size: int = 25,
        cell_size: float = 1.0,
        grid_offset: float = 12.0,
        command_timeout: float = 1.0,
    ):
        """
        Parameters
        ----------
        max_speed : float
            Maximum horizontal velocity (m/s) commanded per axis.
        grid_size : int
            Number of cells per axis in the discrete grid (0 .. grid_size-1).
        cell_size : float
            Physical size of one grid cell in metres.
        command_timeout : float
            Maximum seconds a single action command should be held before
            the controller forces a stop (safety fallback).
        """
        self.max_speed = max_speed
        self.grid_size = grid_size
        self.cell_size = cell_size
        self.grid_offset = grid_offset
        self.command_timeout = command_timeout

    # ── Discrete → Velocity ──────────────────────────────────────

    def action_to_twist(self, action: int) -> Twist:
        """
        Convert a single discrete action index to a geometry_msgs/Twist.

        The Twist contains bounded linear.x / linear.y velocity.
        All other fields (linear.z, angular.*) are zero.
        """
        twist = Twist()

        if action == self.ACTION_PLUS_X:
            twist.linear.x = self.max_speed
        elif action == self.ACTION_MINUS_X:
            twist.linear.x = -self.max_speed
        elif action == self.ACTION_PLUS_Y:
            twist.linear.y = self.max_speed
        elif action == self.ACTION_MINUS_Y:
            twist.linear.y = -self.max_speed
        # ACTION_HOVER and any invalid action → zero twist (safe default)

        return twist

    def stop_twist(self) -> Twist:
        """Return a zero-velocity Twist (hover / emergency stop)."""
        return Twist()

    # ── Grid helpers (for Phase 5 position controller) ───────────

    def get_target_grid_cell(
        self,
        current_x: int,
        current_y: int,
        action: int,
    ) -> tuple[int, int]:
        """
        Compute the target grid cell for a given action, clamped to [0, grid_size-1].

        Parameters
        ----------
        current_x, current_y : int
            Current discrete grid coordinates.
        action : int
            Discrete action index.

        Returns
        -------
        (target_x, target_y) : tuple[int, int]
        """
        dx = self._DX.get(action, 0)
        dy = self._DY.get(action, 0)
        tx = max(0, min(self.grid_size - 1, current_x + dx))
        ty = max(0, min(self.grid_size - 1, current_y + dy))
        return (tx, ty)

    def grid_to_world(self, gx: int, gy: int, altitude: float = 2.0) -> tuple[float, float, float]:
        """
        Convert discrete grid coordinates to Gazebo world coordinates.

        The training grid is 25×25 (indices 0-24).
        Gazebo world origin is at the centre, so the mapping is:
            world_x = (gx - 12) * cell_size
            world_y = (gy - 12) * cell_size
        This matches the existing spawn_drones.sh OFFSET=-12 convention.
        """
        wx = (gx - self.grid_offset) * self.cell_size
        wy = (gy - self.grid_offset) * self.cell_size
        return (wx, wy, altitude)

    def world_to_grid(self, wx: float, wy: float) -> tuple[int, int]:
        """
        Convert Gazebo world coordinates to the nearest discrete grid cell.
        """
        gx = int(round(wx / self.cell_size + self.grid_offset))
        gy = int(round(wy / self.cell_size + self.grid_offset))
        gx = max(0, min(self.grid_size - 1, gx))
        gy = max(0, min(self.grid_size - 1, gy))
        return (gx, gy)