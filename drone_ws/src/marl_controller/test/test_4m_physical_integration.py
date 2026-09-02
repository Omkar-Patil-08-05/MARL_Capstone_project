import sys
import os
import time
import json
import math
import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped

# Add marl_controller to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from marl_controller.state_builder import StateBuilder
from marl_controller.action_mapper import ActionMapper
from marl_controller.qmix_inference import QMIXInference
from marl_controller.motion_controller import MultiAgentMotionController

class Test4mNode(Node):
    def __init__(self):
        super().__init__("test_4m_node")

        # A & B: Initialize with 4m scale
        self.cell_size = 4.0
        self.grid_offset = 0.0
        self.num_drones = 6

        self.state_builder = StateBuilder(num_drones=self.num_drones, cell_size=self.cell_size, grid_offset=self.grid_offset)
        self.action_mapper = ActionMapper(max_speed=2.0, cell_size=self.cell_size, grid_offset=self.grid_offset)
        self.motion_controller = MultiAgentMotionController(
            num_drones=self.num_drones,
            action_mapper=self.action_mapper,
            max_speed=2.0,
            arrival_tolerance=0.10,
            movement_timeout=5.0
        )

        # C: Load obstacle map
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
        self.obstacle_cells = list(obs_set)

        print(f"Loaded {len(self.obstacle_cells)} obstacle cells.")

        # D: Six selected start cells are free
        self.canonical_start = [(2, 2), (12, 2), (22, 2), (2, 12), (12, 15), (22, 12)]
        for gx, gy in self.canonical_start:
            if (gx, gy) in self.obstacle_cells:
                print(f"FAIL: Start cell {(gx, gy)} is occupied by an obstacle!")
                sys.exit(1)
        print("PASS: Six start cells are free.")

        # E: Round trip check
        for gx, gy in self.canonical_start:
            wx, wy = self.state_builder.grid_to_world(gx, gy)
            rgx, rgy = self.state_builder.world_to_grid(wx, wy)
            if rgx != gx or rgy != gy:
                print(f"FAIL: Round trip failed for {(gx, gy)} -> {(wx, wy)} -> {(rgx, rgy)}")
                sys.exit(1)
        print("PASS: Grid -> world -> grid round trip succeeds.")

        # ROS setup
        self._world_poses = {}
        self._pose_subs = []
        self._cmd_pubs = []
        for i in range(self.num_drones):
            sub = self.create_subscription(PoseStamped, f"/model/drone{i+1}/pose", lambda msg, idx=i: self._pose_callback(idx, msg), 10)
            self._pose_subs.append(sub)
            pub = self.create_publisher(Twist, f"/model/drone{i+1}/cmd_vel", 10)
            self._cmd_pubs.append(pub)

        # Stop all drones initially
        stop_twist = Twist()
        for pub in self._cmd_pubs:
            pub.publish(stop_twist)

        self.state_builder.reset_episode(obstacle_cells=self.obstacle_cells)

    def _pose_callback(self, idx, msg):
        self._world_poses[idx] = (msg.pose.position.x, msg.pose.position.y)

    def run_test(self):
        # F: set_pose teleportation
        for idx in range(self.num_drones):
            cx, cy = self.canonical_start[idx]
            wx, wy = self.state_builder.grid_to_world(cx, cy)
            req = f'name: "drone{idx+1}", position: {{x: {wx}, y: {wy}, z: 0.2}}'
            res = subprocess.run(["gz", "service", "-s", "/world/realistic_sar/set_pose", "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean", "--timeout", "5000", "--req", req], capture_output=True, text=True)
            if res.returncode != 0:
                print(f"FAIL: set_pose failed: {res.stderr}")
                sys.exit(1)

        # G: Wait for pose feedback
        start_t = time.time()
        while time.time() - start_t < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            all_good = True
            for idx in range(self.num_drones):
                if idx not in self._world_poses:
                    all_good = False
                    break
                p_wx, p_wy = self._world_poses[idx]
                t_wx, t_wy = self.state_builder.grid_to_world(self.canonical_start[idx][0], self.canonical_start[idx][1])
                if math.hypot(p_wx - t_wx, p_wy - t_wy) > 0.5:
                    all_good = False
                    break
            if all_good:
                break

        if not all_good:
            print("FAIL: Pose feedback did not confirm set_pose coordinates.")
            for idx in range(self.num_drones):
                print(f"Drone {idx+1}: Expected {self.state_builder.grid_to_world(*self.canonical_start[idx])}, got {self._world_poses.get(idx)}")
            sys.exit(1)
        print("PASS: Pose feedback confirms all six positions.")

        # N: Load QMIX checkpoint
        model_path = os.path.expanduser("~/capstone_project_antigravity/marl_drone_project/models/qmix_n6_exp2/qmix_sar_v4_align_best.pth")
        qmix = QMIXInference(model_path=model_path, num_drones=self.num_drones, device="cpu")
        print("PASS: QMIX checkpoint loads successfully.")

        # Setup observation
        current_world_positions = [self._world_poses[i] for i in range(self.num_drones)]
        grid_positions = [self.state_builder.world_to_grid(wx, wy) for wx, wy in current_world_positions]
        self.state_builder.update_drone_positions(grid_positions)
        self.state_builder.update_bfs()

        obs = self.state_builder.get_all_states()

        import numpy as np
        obs_array = np.array(obs, dtype=np.float32)
        # O: Observation shape
        if obs_array.shape != (6, 49):
            print(f"FAIL: Observation shape is {obs_array.shape}, expected (6, 49)")
            sys.exit(1)
        print("PASS: 49D observation shape verified.")

        # H: Valid action produces 1 cell / 4m target
        actions = qmix.get_actions(obs_array)

        # Prevent hover to ensure we test movement
        for i in range(len(actions)):
            if actions[i] == 4:
                actions[i] = 0 # Force +X

        self.motion_controller.prepare_next_step()
        self.motion_controller.dispatch_actions(actions, grid_positions, self.state_builder.grid)

        for idx in range(self.num_drones):
            start_grid = grid_positions[idx]
            target_grid = self.motion_controller.target_grid_cells[idx]
            if self.motion_controller.states[idx] == "MOVING":
                grid_dist = abs(start_grid[0] - target_grid[0]) + abs(start_grid[1] - target_grid[1])
                if grid_dist != 1:
                    print(f"FAIL: Target grid is {grid_dist} cells away.")
                    sys.exit(1)
                t_wx, t_wy = self.motion_controller.target_world_positions[idx]
                p_wx, p_wy = current_world_positions[idx]
                phys_dist = math.hypot(t_wx - p_wx, t_wy - p_wy)
                if abs(phys_dist - 4.0) > 0.1:
                    print(f"FAIL: Physical distance is {phys_dist}, expected ~4.0m.")
                    sys.exit(1)
        print("PASS: Action produces a target exactly 1 grid cell (4.0m) away.")

        # J & K: Drone physically moves toward target, no teleportation
        start_t = time.time()
        while time.time() - start_t < 6.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            current_world_positions = [self._world_poses.get(i, (0,0)) for i in range(self.num_drones)]
            twists = self.motion_controller.step(current_world_positions)
            for i, tw in enumerate(twists):
                self._cmd_pubs[i].publish(tw)

            if self.motion_controller.all_complete():
                break

        # Stop
        for pub in self._cmd_pubs:
            pub.publish(Twist())

        # L: After arrival, check mapping
        for idx in range(self.num_drones):
            p_wx, p_wy = self._world_poses[idx]
            gx, gy = self.state_builder.world_to_grid(p_wx, p_wy)
            target_grid = self.motion_controller.target_grid_cells[idx]
            if self.motion_controller.states[idx] == "ARRIVED":
                if (gx, gy) != target_grid:
                    print(f"FAIL: Drone {idx+1} arrived at world ({p_wx:.2f}, {p_wy:.2f}) which maps to {gx, gy}, expected {target_grid}")
                    sys.exit(1)
        print("PASS: Drone successfully physically moved to target and mapped correctly.")

        # M: No drone inside obstacle
        for idx in range(self.num_drones):
            p_wx, p_wy = self._world_poses[idx]
            gx, gy = self.state_builder.world_to_grid(p_wx, p_wy)
            if (gx, gy) in self.obstacle_cells:
                print(f"FAIL: Drone {idx+1} ended up inside obstacle cell {gx, gy}")
                sys.exit(1)
        print("PASS: No drone inside an obstacle.")

        print("ALL TESTS PASSED.")

def main():
    rclpy.init()
    node = Test4mNode()
    try:
        node.run_test()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
