import os
import time
import math
import json
import numpy as np

import rclpy
from geometry_msgs.msg import Twist, PoseStamped

from marl_controller.controller_node import ControllerNode, ACTION_NAMES
from marl_controller.motion_controller import STATE_INVALID_TARGET, STATE_ARRIVED, STATE_COMPLETE

class SmokeTestNode(ControllerNode):
    def __init__(self):
        super().__init__()
        self.max_steps = 20
        self.history = []
        self.step_start_poses = {} # to track where they started
        self.step_start_times = {}

    def _control_callback(self):
        if not self._episode_active:
            return

        now = time.time()
        all_valid = True
        current_world_positions = []

        for idx in range(self.num_drones):
            if idx not in self._world_poses:
                all_valid = False
            elif now - self._pose_timestamps[idx] > self.stale_timeout:
                all_valid = False
            else:
                current_world_positions.append(self._world_poses[idx])

        if not all_valid:
            self._hover_all()
            return

        if self.motion_controller.all_complete():
            grid_positions = []
            for idx in range(self.num_drones):
                wx, wy = current_world_positions[idx]
                gx, gy = self.state_builder.world_to_grid(wx, wy)
                grid_positions.append((gx, gy))

            # Record outcome of previous step
            if self._step_count > 0:
                for idx in range(self.num_drones):
                    mc = self.motion_controller
                    action = mc.active_actions[idx]

                    tx, ty = mc.target_grid_cells[idx]
                    fx, fy = grid_positions[idx]

                    wx, wy = current_world_positions[idx]
                    twx, twy = mc.target_world_positions[idx]

                    error = math.hypot(wx - twx, wy - twy)
                    duration = now - self.step_start_times.get(idx, now)

                    # For hover or invalid target, distance error to target is not meaningful in the same way, but target == current

                    self.history.append({
                        "step": self._step_count,
                        "drone_id": idx,
                        "current_grid_x": self.step_start_grid[idx][0],
                        "current_grid_y": self.step_start_grid[idx][1],
                        "action": int(action),
                        "action_name": ACTION_NAMES[action],
                        "target_grid_x": int(tx),
                        "target_grid_y": int(ty),
                        "final_grid_x": int(fx),
                        "final_grid_y": int(fy),
                        "motion_result": "VALID" if (fx==tx and fy==ty and action!=4) else ("HOVER" if action==4 else "INVALID_TARGET"),
                        "movement_duration": float(duration),
                        "arrival_error": float(error)
                    })

            if self._step_count >= self.max_steps:
                self.get_logger().info("SMOKE TEST COMPLETE.")
                # Dump history
                with open("/tmp/smoke_test_history.json", "w") as f:
                    json.dump(self.history, f, indent=2)
                self._episode_active = False
                self._hover_all()
                import sys
                sys.exit(0)

            # Update state builder
            self.state_builder.update_drone_positions(grid_positions)
            self.state_builder.update_bfs()

            # QMIX
            obs = self.state_builder.get_all_states()
            obs_array = np.array(obs, dtype=np.float32)
            actions = self.qmix.get_actions(obs_array)

            # Save step start info
            self.step_start_grid = grid_positions.copy()
            for idx in range(self.num_drones):
                self.step_start_times[idx] = now

            self.motion_controller.prepare_next_step()
            self.motion_controller.dispatch_actions(actions, grid_positions, self.state_builder.grid)

            # Note if dispatch says invalid target, correct the motion_result logic later via states, but since we know it now:
            # Actually, the result is better pulled from mc.states before prepare_next_step?
            # Wait, mc.states is ALL COMPLETE. So we lost the INVALID_TARGET state?
            # Yes, because the fast loop overwrote INVALID_TARGET -> COMPLETE.
            # I will fix that in post-processing: if action != 4 and fx == start_x and fy == start_y, it was INVALID_TARGET.

            self._step_count += 1
            explored = np.sum(self.state_builder.grid == 1)
            valid = np.sum(self.state_builder.grid != -1)
            self.get_logger().info(f"Step {self._step_count} coverage={explored/max(1,valid):.2f}")

        twists = self.motion_controller.step(current_world_positions)
        for idx in range(self.num_drones):
            self._cmd_pubs[idx].publish(twists[idx])


def main(args=None):
    rclpy.init(args=args)
    node = SmokeTestNode()
    node.start_episode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_episode()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
