"""
Motion Controller for translating grid-cell actions to physical Gazebo trajectories.
Handles independent motion states for 6 drones, ensuring cell-to-cell discrete movement.
"""

import time
import math
from geometry_msgs.msg import Twist
from marl_controller.action_mapper import ActionMapper

# Motion States
STATE_READY = "READY"
STATE_MOVING = "MOVING"
STATE_ARRIVED = "ARRIVED"
STATE_INVALID_TARGET = "INVALID_TARGET"
STATE_COMPLETE = "COMPLETE_FOR_STEP"

class MultiAgentMotionController:
    def __init__(
        self,
        num_drones: int = 6,
        action_mapper: ActionMapper = None,
        max_speed: float = 2.0,
        arrival_tolerance: float = 0.10,
        movement_timeout: float = 1.5,
    ):
        self.num_drones = num_drones
        self.action_mapper = action_mapper if action_mapper else ActionMapper(max_speed=max_speed)

        self.max_speed = max_speed
        self.arrival_tolerance = arrival_tolerance
        self.movement_timeout = movement_timeout

        # State tracking per agent
        self.states = [STATE_READY] * self.num_drones
        self.start_times = [0.0] * self.num_drones

        self.active_actions = [ActionMapper.ACTION_HOVER] * self.num_drones
        self.target_grid_cells = [(0, 0)] * self.num_drones
        self.target_world_positions = [(0.0, 0.0)] * self.num_drones

        # For proportional control
        self.p_gain = 2.0

    def dispatch_actions(self, actions: list[int], current_grid_positions: list[tuple[int, int]], grid_array):
        """
        Receive joint actions from policy, compute targets, and transition drones to MOVING or INVALID_TARGET.
        grid_array is the 25x25 grid where -1 represents obstacles.
        """
        for i in range(self.num_drones):
            action = actions[i]
            self.active_actions[i] = action
            cx, cy = current_grid_positions[i]

            if action == self.action_mapper.ACTION_HOVER:
                self.states[i] = STATE_COMPLETE
                self.target_grid_cells[i] = (cx, cy)
                tx, ty, _ = self.action_mapper.grid_to_world(cx, cy)
                self.target_world_positions[i] = (tx, ty)
                continue

            tx, ty = self.action_mapper.get_target_grid_cell(cx, cy, action)

            # Check validity
            if tx == cx and ty == cy:
                # Boundary collision (get_target_grid_cell clamps)
                self.states[i] = STATE_INVALID_TARGET
                self.target_grid_cells[i] = (cx, cy)
                wx, wy, _ = self.action_mapper.grid_to_world(cx, cy)
                self.target_world_positions[i] = (wx, wy)
                continue

            if grid_array[tx, ty] == -1:
                # Obstacle collision
                self.states[i] = STATE_INVALID_TARGET
                self.target_grid_cells[i] = (cx, cy)
                wx, wy, _ = self.action_mapper.grid_to_world(cx, cy)
                self.target_world_positions[i] = (wx, wy)
                continue

            # Valid movement
            self.states[i] = STATE_MOVING
            self.start_times[i] = time.time()
            self.target_grid_cells[i] = (tx, ty)
            wx, wy, _ = self.action_mapper.grid_to_world(tx, ty)
            self.target_world_positions[i] = (wx, wy)

    def step(self, current_world_positions: list[tuple[float, float]]) -> list[Twist]:
        """
        Step the control loop. Computes Twists for all drones and updates their arrival states.
        Returns the list of Twists to publish.
        """
        twists = []
        now = time.time()

        for i in range(self.num_drones):
            state = self.states[i]
            twist = Twist()

            if state in [STATE_READY, STATE_COMPLETE, STATE_INVALID_TARGET, STATE_ARRIVED]:
                # In these states, hover
                twists.append(twist)

                # Transition ARRIVED/INVALID_TARGET to COMPLETE if not already
                if state in [STATE_ARRIVED, STATE_INVALID_TARGET]:
                    self.states[i] = STATE_COMPLETE
                continue

            if state == STATE_MOVING:
                wx, wy = current_world_positions[i]
                tx, ty = self.target_world_positions[i]

                dx = tx - wx
                dy = ty - wy
                dist = math.hypot(dx, dy)

                if dist < self.arrival_tolerance:
                    self.states[i] = STATE_ARRIVED
                    twists.append(twist) # zero velocity
                    continue

                if now - self.start_times[i] > self.movement_timeout:
                    self.states[i] = STATE_ARRIVED # Treat timeout as arrived/stopped
                    twists.append(twist)
                    continue

                # Proportional velocity
                vx = dx * self.p_gain
                vy = dy * self.p_gain

                # Clamp magnitude
                v_mag = math.hypot(vx, vy)
                if v_mag > self.max_speed:
                    vx = (vx / v_mag) * self.max_speed
                    vy = (vy / v_mag) * self.max_speed

                twist.linear.x = float(vx)
                twist.linear.y = float(vy)
                twists.append(twist)

        return twists

    def all_complete(self) -> bool:
        """Returns True if all drones have finished their movement for this step."""
        for state in self.states:
            if state not in [STATE_COMPLETE, STATE_READY]:
                return False
        return True

    def prepare_next_step(self):
        """Reset all drones to READY state."""
        for i in range(self.num_drones):
            self.states[i] = STATE_READY
