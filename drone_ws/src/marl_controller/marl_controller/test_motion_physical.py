import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from marl_controller.motion_controller import MultiAgentMotionController
from marl_controller.action_mapper import ActionMapper
import time
import math
import sys
import numpy as np

class PhysicalTestNode(Node):
    def __init__(self):
        super().__init__('physical_test_node')
        self.num_drones = 6
        self.action_mapper = ActionMapper(max_speed=2.0)
        self.mc = MultiAgentMotionController(num_drones=6, action_mapper=self.action_mapper)

        self.poses = {}
        self.cmd_pubs = []
        self.subs = []

        for i in range(6):
            name = f"drone{i+1}"
            self.cmd_pubs.append(self.create_publisher(Twist, f'/model/{name}/cmd_vel', 10))
            self.subs.append(self.create_subscription(
                PoseStamped, f'/model/{name}/pose',
                lambda msg, idx=i: self.pose_cb(idx, msg), 10
            ))

        self.timer = self.create_timer(0.05, self.control_loop)

        self.test_phase = 0
        self.phase_start_time = 0
        self.start_poses = {}

    def pose_cb(self, idx, msg):
        self.poses[idx] = (msg.pose.position.x, msg.pose.position.y)

    def control_loop(self):
        if len(self.poses) < 6:
            return

        current_world_positions = [self.poses[i] for i in range(6)]

        if self.mc.all_complete():
            if self.test_phase == 0:
                self.get_logger().info("=== PHASE 0: HOVER VALIDATION ===")
                self.start_poses = dict(self.poses)
                self.phase_start_time = time.time()
                self.mc.prepare_next_step()
                # Dummy grid pos for drone 0: it is at (-10, -10), grid is (2, 2)
                grid_pos = [self.action_mapper.world_to_grid(*self.poses[i]) for i in range(6)]
                # Mock grid open everywhere

                grid = np.zeros((25, 25), dtype=np.int8)
                self.mc.dispatch_actions([4]*6, grid_pos, grid)
                self.test_phase = 1

            elif self.test_phase == 1:
                dur = time.time() - self.phase_start_time
                self.get_logger().info(f"Hover complete. Duration: {dur:.2f}s")
                self.get_logger().info("=== PHASE 1: +X MOVEMENT DRONE 0 ===")
                self.start_poses = dict(self.poses)
                self.phase_start_time = time.time()
                self.mc.prepare_next_step()
                grid_pos = [self.action_mapper.world_to_grid(*self.poses[i]) for i in range(6)]
                grid = np.zeros((25, 25), dtype=np.int8)
                actions = [4]*6
                actions[0] = 0 # +X
                self.mc.dispatch_actions(actions, grid_pos, grid)
                self.test_phase = 2

            elif self.test_phase == 2:
                dur = time.time() - self.phase_start_time
                dx = self.poses[0][0] - self.start_poses[0][0]
                self.get_logger().info(f"+X Movement complete. Duration: {dur:.2f}s, dx: {dx:.3f}m")
                self.get_logger().info("=== PHASE 2: -X, +Y, -Y MOVEMENT DRONE 0 ===")
                self.start_poses = dict(self.poses)
                self.phase_start_time = time.time()
                self.mc.prepare_next_step()
                grid_pos = [self.action_mapper.world_to_grid(*self.poses[i]) for i in range(6)]
                grid = np.zeros((25, 25), dtype=np.int8)
                actions = [4]*6
                actions[0] = 1 # -X
                self.mc.dispatch_actions(actions, grid_pos, grid)
                self.test_phase = 3

            elif self.test_phase == 3:
                dur = time.time() - self.phase_start_time
                dx = self.poses[0][0] - self.start_poses[0][0]
                self.get_logger().info(f"-X Movement complete. Duration: {dur:.2f}s, dx: {dx:.3f}m")
                self.get_logger().info("=== PHASE 3: ALL DRONES +Y MOVEMENT ===")
                self.start_poses = dict(self.poses)
                self.phase_start_time = time.time()
                self.mc.prepare_next_step()
                grid_pos = [self.action_mapper.world_to_grid(*self.poses[i]) for i in range(6)]
                grid = np.zeros((25, 25), dtype=np.int8)
                actions = [2]*6 # +Y
                self.mc.dispatch_actions(actions, grid_pos, grid)
                self.test_phase = 4

            elif self.test_phase == 4:
                dur = time.time() - self.phase_start_time
                dy = self.poses[0][1] - self.start_poses[0][1]
                self.get_logger().info(f"All +Y Movement complete. Duration: {dur:.2f}s, dy drone0: {dy:.3f}m")
                self.get_logger().info("=== PHASE 4: INVALID TARGET DRONE 0 ===")
                self.start_poses = dict(self.poses)
                self.phase_start_time = time.time()
                self.mc.prepare_next_step()
                grid_pos = [self.action_mapper.world_to_grid(*self.poses[i]) for i in range(6)]
                grid = np.zeros((25, 25), dtype=np.int8)
                grid[grid_pos[0][0]+1, grid_pos[0][1]] = -1 # Obstacle blocking +X
                actions = [4]*6
                actions[0] = 0 # +X into obstacle
                self.mc.dispatch_actions(actions, grid_pos, grid)
                self.test_phase = 5

            elif self.test_phase == 5:
                dur = time.time() - self.phase_start_time
                dx = self.poses[0][0] - self.start_poses[0][0]
                self.get_logger().info(f"Invalid Target complete. Duration: {dur:.2f}s, dx (should be ~0): {dx:.3f}m")
                self.get_logger().info("ALL TESTS DONE.")
                sys.exit(0)

        twists = self.mc.step(current_world_positions)
        for i in range(6):
            self.cmd_pubs[i].publish(twists[i])

def main():
    rclpy.init()
    node = PhysicalTestNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
