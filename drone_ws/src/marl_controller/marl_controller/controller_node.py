import sys
import os

sys.path.append(os.path.expanduser("~/capstone_project"))
import rclpy
from rclpy.node import Node
from collections import deque
import random
from results.logger import ResultLogger
from marl_controller.state_builder import StateBuilder
from marl_controller.qmix_inference import QMIXInference
from marl_controller.action_mapper import ActionMapper


class ControllerNode(Node):

    def __init__(self):
        super().__init__('marl_controller')

        self.num_drones = 6

        self.drone_names = [
            "drone1", "drone2", "drone3",
            "drone4", "drone5", "drone6"
        ]

        self.state_builder = StateBuilder(num_drones=self.num_drones)
        self.action_mapper = ActionMapper()
        self.logger = ResultLogger()

        self.current_positions = {
            "drone1": [3, 3, 2.0],
            "drone2": [12, 3, 2.0],
            "drone3": [21, 3, 2.0],
            "drone4": [3, 12, 2.0],
            "drone5": [12, 12, 2.0],
            "drone6": [21, 12, 2.0],
        }

        self.visited = set()

        self.history = {
            name: deque(maxlen=5)
            for name in self.drone_names
        }

        # 🌲 TREE OBSTACLES (GRID COORDS)
        self.obstacles = {
            (4, 4),
            (8, 6),
            (12, 10),
            (16, 14),
            (20, 18),
            (6, 18),
            (18, 6),
            (10, 20),
            (14, 8)
        }

        self.qmix = None

        self.timer = self.create_timer(0.9, self.control_loop) 

        self.get_logger().info("MARL Controller Started")

    def get_unvisited_targets(self):
        targets = []
        for x in range(25):
            for y in range(25):
                if (x, y) not in self.visited and (x, y) not in self.obstacles:
                    targets.append((x, y))
        return targets

    def control_loop(self):

        ordered_positions = [
            self.current_positions[name]
            for name in self.drone_names
        ]

        self.state_builder.update_positions(ordered_positions)
        states = self.state_builder.get_all_states()

        if self.qmix is None:
            print("Loading QMIX...")
            self.qmix = QMIXInference(
                model_path='/home/capstone/capstone_project/drone_ws/src/marl_controller/models/qmix_phase1.pth',
                num_drones=self.num_drones,
                state_size=len(states[0]),
                action_size=6
            )
            print("QMIX Loaded")

        q_actions = self.qmix.get_actions(states)

        occupied = set()
        new_positions = {}

        coverage = len(self.visited) / (25 * 25)

        cleanup_mode = coverage > 0.6
        final_mode = coverage > 0.9

        targets = self.get_unvisited_targets()

        for i, drone_name in enumerate(self.drone_names):

            pos = self.current_positions[drone_name]

            best_score = -1e9
            best_pos = pos

            for a in range(4):

                test_pos = self.action_mapper.get_next_pos(pos, a)
                x, y = test_pos[0], test_pos[1]

                # HARD OBSTACLE BLOCK
                if (x, y) in self.obstacles:
                    continue 

                score = 0

                if (x, y) in occupied:
                    score -= 300

                if (x, y) not in self.visited:
                    score += 30
                else:
                    score -= 5

                if targets:
                    min_dist = min(abs(x - tx) + abs(y - ty) for tx, ty in targets)

                    score += 25 / (min_dist + 1)

                    if cleanup_mode:
                        score += 40 / (min_dist + 1)

                    if final_mode:
                        score += 100 / (min_dist + 1)

                if (x, y) in self.history[drone_name]:
                    score -= 10

                if a == (q_actions[i] % 4):
                    score += 2

                if score > best_score:
                    best_score = score
                    best_pos = test_pos

            new_positions[drone_name] = best_pos
            occupied.add((best_pos[0], best_pos[1]))

        for drone_name, new_pos in new_positions.items():

            old_pos = self.current_positions[drone_name]

            self.current_positions[drone_name] = new_pos
            self.visited.add((new_pos[0], new_pos[1]))
            self.history[drone_name].append((new_pos[0], new_pos[1]))

            if new_pos != old_pos:
                self.action_mapper.move_drone(drone_name, new_pos)

        coverage = len(self.visited) / (25 * 25)
        print(f"Coverage: {coverage:.2f}")
        self.logger.log(coverage, ordered_positions)

        if coverage >= 0.98:
            print("✅ Coverage complete. Stopping...")

            self.logger.close()

            import os
            os._exit(0)   # FORCE EXIT (important for ROS)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()