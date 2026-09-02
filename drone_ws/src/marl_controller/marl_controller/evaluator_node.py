import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import time
import json
import math
import subprocess
import numpy as np
import torch

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped

from controller_node import ControllerNode, ACTION_NAMES
from motion_controller import STATE_INVALID_TARGET, STATE_ARRIVED, STATE_COMPLETE

class EvaluatorNode(ControllerNode):
    def __init__(self, num_episodes=10, output_dir="/home/capstone/.gemini/antigravity-ide/brain/000ffb3a-3f6b-440e-96fb-2706b50dc733/gazebo_evaluation/resume_test", cell_size=1.0, grid_offset=12.0, base_seed=100):
        super().__init__()

        self.cell_size = cell_size
        self.grid_offset = grid_offset
        self.base_seed = base_seed

        # Override ControllerNode's default objects with scaled ones
        from marl_controller.state_builder import StateBuilder
        from marl_controller.action_mapper import ActionMapper
        self.state_builder = StateBuilder(num_drones=self.num_drones, cell_size=self.cell_size, grid_offset=self.grid_offset)
        max_speed = self.get_parameter("max_speed").value
        self.action_mapper = ActionMapper(max_speed=max_speed, cell_size=self.cell_size, grid_offset=self.grid_offset)
        self.motion_controller.action_mapper = self.action_mapper

        if self.cell_size == 4.0:
            self.motion_controller.movement_timeout = 5.0 # Give it 5 seconds to travel 4m at 2m/s

        self.eval_dir = output_dir
        val_dir = os.path.join(self.eval_dir, "validation")
        os.makedirs(val_dir, exist_ok=True)

        # Cleanup any orphaned .tmp files
        import glob
        tmp_files = glob.glob(os.path.join(val_dir, "episode_*.json.tmp"))
        for tmp_f in tmp_files:
            try:
                os.remove(tmp_f)
            except OSError:
                pass

        # Find highest episode JSON
        json_files = glob.glob(os.path.join(val_dir, "episode_*.json"))
        highest = 0
        for f in json_files:
            basename = os.path.basename(f)
            try:
                ep_num = int(basename.replace("episode_", "").replace(".json", ""))
                highest = max(highest, ep_num)
            except ValueError:
                pass

        self.current_episode = highest + 1
        self.target_episodes = highest + num_episodes

        self.eval_state = "RESETTING" # RESETTING, VALIDATING, RUNNING, DONE

        self.metrics = {}
        self.history = []

        # Load victims
        meta_path = "/home/capstone/capstone_project_antigravity/worlds/generated_world_meta.json"
        with open(meta_path, 'r') as f:
            meta = json.load(f)

        self.ground_truth_victims = {}
        for v in meta.get("victims", []):
            gx, gy = v["grid"]["x"], v["grid"]["y"]
            self.ground_truth_victims[(gx, gy)] = False # Found status

        self.total_victims = len(self.ground_truth_victims)

        # Parse obstacles from metadata
        self.obstacle_cells = []
        for obs in meta.get("obstacles", []):
            aabb = obs["aabb"]
            for gx in range(self.state_builder.x_size):
                for gy in range(self.state_builder.y_size):
                    wx, wy = self.state_builder.grid_to_world(gx, gy)
                    cell_min_x = wx - (self.cell_size / 2.0) if self.cell_size == 4.0 else wx
                    cell_min_y = wy - (self.cell_size / 2.0) if self.cell_size == 4.0 else wy

                    # city.py generated AABBs where grid boundaries are exactly gx * 4.0.
                    # Our cell_size=4.0 and grid_offset=0 means grid_to_world gives us the exact min coordinate!
                    # Wait, no. ActionMapper.grid_to_world returns the corner or the center?
                    # Let's just use the exact city.py logic for 4m, and empty for 1m to preserve baseline.
                    pass

        if self.cell_size == 4.0:
            obs_set = set()
            for obs in meta.get("obstacles", []):
                aabb = obs["aabb"]
                for gx in range(25):
                    for gy in range(25):
                        if (aabb["max_x"] > gx * 4.0) and (aabb["min_x"] < (gx + 1) * 4.0):
                            if (aabb["max_y"] > gy * 4.0) and (aabb["min_y"] < (gy + 1) * 4.0):
                                obs_set.add((gx, gy))
            self.obstacle_cells = list(obs_set)

        if self.cell_size == 4.0:
            self.canonical_start = [
                (2, 2),
                (12, 2),
                (22, 2),
                (2, 12),
                (12, 15), # Shifted Drone 5 to avoid maj_bldg_9
                (22, 12)
            ]
        else:
            self.canonical_start = [
                (2, 2),
                (12, 2),
                (22, 2),
                (2, 12),
                (12, 12),
                (22, 12)
            ]

        self.summary_rows = []

        # Start the first reset will be called externally
        self.get_logger().info(f"EVALUATOR INITIALIZED. Beginning Episode {self.current_episode}.")

    def trigger_reset(self, skip_gz_reset=False):
        self.eval_state = "RESETTING"
        self._hover_all()

        if not skip_gz_reset:
            for idx in range(self.num_drones):
                cx, cy = self.canonical_start[idx]
                wx, wy = self.state_builder.grid_to_world(cx, cy)
                req = f'name: "drone{idx+1}", position: {{x: {wx}, y: {wy}, z: 0.2}}'
                res = subprocess.run(["gz", "service", "-s", "/world/realistic_sar/set_pose", "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean", "--timeout", "5000", "--req", req], capture_output=True, text=True)
                if res.returncode != 0:
                    self.get_logger().error(f"Gazebo set_pose failed for drone {idx+1}: {res.stderr}")
                    sys.exit(1)
        self.state_builder.reset_episode(obstacle_cells=self.obstacle_cells)

        self.qmix.reset_episode()
        self.motion_controller.prepare_next_step()

        if self.cell_size == 4.0:
            import random
            episode_seed = self.base_seed + self.current_episode
            rng = random.Random(episode_seed)

            valid_cells = []
            for gx in range(self.state_builder.x_size):
                for gy in range(self.state_builder.y_size):
                    if (gx, gy) not in self.obstacle_cells and (gx, gy) not in self.canonical_start:
                        valid_cells.append((gx, gy))

            if len(valid_cells) < 5:
                self.get_logger().error(f"Failed to find 5 valid victim cells! Only {len(valid_cells)} available.")
                sys.exit(1)

            sampled_victims = rng.sample(valid_cells, 5)
            self.ground_truth_victims = {cell: False for cell in sampled_victims}
            self.total_victims = 5
        else:
            for k in self.ground_truth_victims:
                self.ground_truth_victims[k] = False

        episode_seed = self.base_seed + self.current_episode if self.cell_size == 4.0 else self.current_episode

        self.metrics = {
            "episode_id": self.current_episode,
            "seed": episode_seed,
            "episode_seed": episode_seed,
            "checkpoint": "qmix_sar_v4_align_best.pth",
            "metadata": {
                "arrival_tolerance": self.motion_controller.arrival_tolerance,
                "max_speed": self.motion_controller.max_speed,
                "movement_timeout": self.motion_controller.movement_timeout,
                "control_frequency": 20.0
            },
            "starting_grid_positions": self.canonical_start,
            "starting_world_positions": [self.state_builder.grid_to_world(cx, cy) for cx, cy in self.canonical_start],
            "duration": 0.0,
            "policy_steps": 0,
            "coverage": 0.0,
            "victims_found": 0,
            "total_victims": self.total_victims,
            "victim_locations": [[vx, vy] for vx, vy in self.ground_truth_victims.keys()],
            "victim_discoveries": {},
            "success_5_5": False,
            "invalid_actions": 0,
            "hover_actions": 0,
            "movement_time": 0.0,
            "movement_time_per_drone": [0.0]*self.num_drones,
            "timeouts": 0,
            "collisions": "unavailable for this prototype",
            "pose_freshness_failures": 0,
            "qmix_inference_failures": 0,
            "travelled_distance": [0.0]*self.num_drones,
            "history": []
        }

        self._step_count = 0
        self._episode_active = True
        self.episode_start_time = time.time()
        self.eval_state = "VALIDATING"
        self.validation_start = time.time()

    def _control_callback(self):
        if self.eval_state == "DONE":
            return

        now = time.time()

        if self.eval_state == "VALIDATING":
            if now - self.validation_start > 30.0:
                self.get_logger().error(f"Validation timeout! Drones didn't reset in time. Lenses: {len(self._world_poses)}")
                for idx in range(self.num_drones):
                    if idx in self._world_poses:
                        wx, wy = self._world_poses[idx]
                        gx, gy = self.state_builder.world_to_grid(wx, wy)
                        self.get_logger().error(f"Drone {idx}: World({wx:.2f}, {wy:.2f}) -> Grid({gx}, {gy}) != Canonical{self.canonical_start[idx]}")
                sys.exit(1)

            # Check drones exist and pose freshness
            if len(self._world_poses) != self.num_drones:
                return # wait

            stale = False
            for idx in range(self.num_drones):
                if now - self._pose_timestamps.get(idx, 0) > self.stale_timeout:
                    stale = True
                    break

            if stale:
                return

            # Verify all drones are at their canonical starting positions
            valid = True
            for idx in range(self.num_drones):
                wx, wy = self._world_poses[idx]
                gx, gy = self.state_builder.world_to_grid(wx, wy)
                cx, cy = self.canonical_start[idx]
                if gx != cx or gy != cy:
                    valid = False
                    break
            if not valid:
                return # Wait for drones to settle

            # Validate QMIX hidden state
            if not torch.all(self.qmix.hidden_state == 0):
                self.get_logger().error("QMIX hidden state failed to reset!")
                sys.exit(1)

            # Validate Victim state
            if any(self.ground_truth_victims.values()):
                self.get_logger().error("Victims failed to reset!")
                sys.exit(1)

            # Validate Grid state
            if np.any(self.state_builder.grid == 1):
                self.get_logger().error("StateBuilder exploration grid failed to reset!")
                sys.exit(1)

            self.get_logger().info(f"Episode {self.current_episode} RESET VALIDATED. Running.")
            self.episode_start_time = time.time()
            self.eval_state = "RUNNING"
            # Fall through to run immediately

        if self.eval_state == "RUNNING":
            all_valid = True
            current_world_positions = []

            for idx in range(self.num_drones):
                if idx not in self._world_poses:
                    all_valid = False
                    self.metrics["pose_freshness_failures"] += 1
                elif now - self._pose_timestamps[idx] > self.stale_timeout:
                    all_valid = False
                    self.metrics["pose_freshness_failures"] += 1
                else:
                    current_world_positions.append(self._world_poses[idx])

            if not all_valid:
                # Log only once every second to avoid flooding
                if now - getattr(self, "last_stale_log", 0) > 1.0:
                    self.get_logger().error(f"Poses stale! Lenses: {len(self._world_poses)}")
                    self.last_stale_log = now
                self._hover_all()
                return

            # Track individual drone completion times for duration logging
            for idx in range(self.num_drones):
                if self.motion_controller.states[idx] == "COMPLETE_FOR_STEP" and getattr(self, f"step_end_time_{idx}", None) is None:
                    setattr(self, f"step_end_time_{idx}", now)

            if self.motion_controller.all_complete():
                grid_positions = []
                for idx in range(self.num_drones):
                    wx, wy = current_world_positions[idx]
                    gx, gy = self.state_builder.world_to_grid(wx, wy)
                    grid_positions.append((gx, gy))

                # Track metrics for the PREVIOUS step (if any)
                if self._step_count > 0:
                    self.metrics["policy_steps"] += 1

                    step_record = {
                        "step_index": self._step_count - 1,
                        "actions": [int(self.motion_controller.active_actions[i]) for i in range(self.num_drones)],
                        "start_world": [[float(getattr(self, f"step_start_world_{i}")[0]), float(getattr(self, f"step_start_world_{i}")[1])] for i in range(self.num_drones)],
                        "start_grid": [[int(getattr(self, f"step_start_grid_{i}")[0]), int(getattr(self, f"step_start_grid_{i}")[1])] for i in range(self.num_drones)],
                        "target_grid": [[int(self.motion_controller.target_grid_cells[i][0]), int(self.motion_controller.target_grid_cells[i][1])] for i in range(self.num_drones)],
                        "end_world": [[float(current_world_positions[i][0]), float(current_world_positions[i][1])] for i in range(self.num_drones)],
                        "end_grid": [[int(grid_positions[i][0]), int(grid_positions[i][1])] for i in range(self.num_drones)],
                        "per_drone_duration": [float(getattr(self, f"step_end_time_{i}", now) - getattr(self, f"step_start_time_{i}", now)) for i in range(self.num_drones)],
                        "invalid_actions": [bool(getattr(self, f"step_invalid_{i}", False)) for i in range(self.num_drones)],
                        "hover_actions": [bool(self.motion_controller.active_actions[i] == 4) for i in range(self.num_drones)],
                        "motion_status": [str(getattr(self, f"step_status_{i}", "UNKNOWN")) for i in range(self.num_drones)]
                    }
                    step_record["step_duration"] = max(step_record["per_drone_duration"])
                    self.metrics["history"].append(step_record)

                    for idx in range(self.num_drones):
                        action = self.motion_controller.active_actions[idx]
                        if action == 4:
                            self.metrics["hover_actions"] += 1

                        if getattr(self, f"step_invalid_{idx}", False):
                            self.metrics["invalid_actions"] += 1

                        dur = step_record["per_drone_duration"][idx]
                        self.metrics["movement_time"] += dur
                        self.metrics["movement_time_per_drone"][idx] += dur

                        # Timestamps are now tracked independently, we don't guess timeout heuristically here for invalid_actions
                        # But we still increment aggregate timeout metric for legacy reporting if duration > timeout
                        if dur > self.motion_controller.movement_timeout * 0.95 and action != 4 and not step_record["invalid_actions"][idx]:
                            self.metrics["timeouts"] += 1

                        # Travelled distance
                        start_wx, start_wy = getattr(self, f"step_start_world_{idx}", current_world_positions[idx])
                        wx, wy = current_world_positions[idx]
                        self.metrics["travelled_distance"][idx] += math.hypot(wx - start_wx, wy - start_wy)

                # Check episode completion condition (max 30 steps or 99% coverage)
                explored = np.sum(self.state_builder.grid == 1)
                valid_cells = np.sum(self.state_builder.grid != -1)
                coverage = explored / max(1, valid_cells)
                self.metrics["coverage"] = float(coverage)

                # Victim Ground Truth Check
                for (vx, vy), found in self.ground_truth_victims.items():
                    if not found and self.state_builder.grid[vx, vy] == 1:
                        self.ground_truth_victims[(vx, vy)] = True
                        self.metrics["victims_found"] += 1

                        # Find which drone discovered it (first drone whose FOV covers it)
                        discovering_drone = -1
                        for idx, (gx, gy) in enumerate(grid_positions):
                            if max(abs(gx - vx), abs(gy - vy)) <= self.state_builder.fov_radius:
                                discovering_drone = idx
                                break

                        vid = str(list(self.ground_truth_victims.keys()).index((vx, vy)))
                        self.metrics["victim_discoveries"][vid] = {
                            "step": self._step_count,
                            "drone": discovering_drone,
                            "world": self.state_builder.grid_to_world(vx, vy),
                            "grid": [vx, vy]
                        }

                if coverage >= 0.99 or self._step_count >= 300:
                    self.end_episode()
                    return

                # Store start values for next step tracking
                for idx in range(self.num_drones):
                    setattr(self, f"step_start_grid_{idx}", grid_positions[idx])
                    setattr(self, f"step_start_world_{idx}", current_world_positions[idx])
                    setattr(self, f"step_start_time_{idx}", now)
                    setattr(self, f"step_end_time_{idx}", None)

                # Update StateBuilder
                self.state_builder.update_drone_positions(grid_positions)
                self.state_builder.update_bfs()

                try:
                    obs = self.state_builder.get_all_states()
                    obs_array = np.array(obs, dtype=np.float32)
                    actions = self.qmix.get_actions(obs_array)
                    self.get_logger().info(f"Step {self._step_count} actions: {actions}")
                except Exception as e:
                    self.metrics["qmix_inference_failures"] += 1
                    self.get_logger().error(f"Inference failed: {e}")
                    self._hover_all()
                    return

                self.motion_controller.prepare_next_step()
                self.motion_controller.dispatch_actions(actions, grid_positions, self.state_builder.grid)
                self._step_count += 1

                # Capture motion states immediately after dispatch to reliably track INVALID_TARGET
                for idx in range(self.num_drones):
                    state = self.motion_controller.states[idx]
                    setattr(self, f"step_status_{idx}", state)
                    setattr(self, f"step_invalid_{idx}", (state == "INVALID_TARGET"))

            twists = self.motion_controller.step(current_world_positions)
            for idx in range(self.num_drones):
                self._cmd_pubs[idx].publish(twists[idx])

    def end_episode(self):
        self.metrics["duration"] = time.time() - self.episode_start_time

        # Final world positions
        final_world = []
        final_grid = []
        for idx in range(self.num_drones):
            wx, wy = self._world_poses.get(idx, (0.0, 0.0))
            gx, gy = self.state_builder.world_to_grid(wx, wy)
            final_world.append([float(wx), float(wy)])
            final_grid.append([int(gx), int(gy)])

        self.metrics["final_world_positions"] = final_world
        self.metrics["final_grid_positions"] = final_grid
        self.metrics["success_5_5"] = (self.metrics["victims_found"] == self.total_victims)
        self.metrics["final_grid_state"] = self.state_builder.grid.tolist()

        # Save JSON
        val_dir = os.path.join(self.eval_dir, "validation")
        os.makedirs(val_dir, exist_ok=True)
        filename = f"episode_{self.current_episode:03d}.json"
        final_path = os.path.join(val_dir, filename)
        tmp_path = final_path + ".tmp"

        if os.path.exists(final_path):
            raise FileExistsError(f"CRITICAL: {final_path} already exists. Refusing to overwrite!")

        with open(tmp_path, "w") as f:
            json.dump(self.metrics, f, indent=2)

        os.replace(tmp_path, final_path)

        self.summary_rows.append(self.metrics)
        self.get_logger().info(f"Episode {self.current_episode} complete. Coverage: {self.metrics['coverage']:.2f}")

        if self.current_episode >= self.target_episodes:
            self.eval_state = "DONE"
            self.write_summary()
            self.get_logger().info("EVALUATION HARNESS COMPLETE.")
            sys.exit(0)
        else:
            self.current_episode += 1
            self.trigger_reset()

    def write_summary(self):
        import csv
        csv_path = os.path.join(self.eval_dir, "summary.csv")
        keys = ["episode_id", "seed", "duration", "policy_steps", "coverage", "victims_found", "total_victims", "invalid_actions", "hover_actions", "timeouts", "collisions"]
        with open(csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=keys, extrasaction='ignore')
            writer.writeheader()
            for row in self.summary_rows:
                writer.writerow(row)

        meta_path = os.path.join(self.eval_dir, "metadata.json")
        with open(meta_path, "w") as f:
            json.dump({
                "checkpoint": "qmix_sar_v4_align_best.pth",
                "arrival_tolerance": self.motion_controller.arrival_tolerance,
                "max_speed": self.motion_controller.max_speed,
                "movement_timeout": self.motion_controller.movement_timeout,
                "control_frequency": self.get_parameter("control_frequency").value,
                "episodes": self.target_episodes
            }, f, indent=2)


def main(args=None):
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--num-episodes", type=int, default=10)
    parser.add_argument("--output-dir", type=str, default="/home/capstone/.gemini/antigravity-ide/brain/000ffb3a-3f6b-440e-96fb-2706b50dc733/gazebo_evaluation/resume_test")
    parser.add_argument("--cell-size", type=float, default=1.0)
    parser.add_argument("--grid-offset", type=float, default=12.0)
    parser.add_argument("--seed", type=int, default=100)
    parsed_args, ros_args = parser.parse_known_args(sys.argv[1:])

    if args is not None:
        ros_args = args

    rclpy.init(args=ros_args)
    node = EvaluatorNode(
        num_episodes=parsed_args.num_episodes,
        output_dir=parsed_args.output_dir,
        cell_size=parsed_args.cell_size,
        grid_offset=parsed_args.grid_offset,
        base_seed=parsed_args.seed
    )
    node.trigger_reset(skip_gz_reset=(parsed_args.cell_size == 1.0))
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
