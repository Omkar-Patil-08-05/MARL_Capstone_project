"""
MARL Controller Node — N=6 QMIX Policy Executor.

Coordinates the full inference pipeline:

  Gazebo drone poses
       ↓
  StateBuilder  (world→grid, 49D observations)
       ↓
  QMIXInference (batched RNNAgent, persistent hidden state)
       ↓
  ActionMapper  (discrete action → bounded Twist)
       ↓
  Gazebo VelocityControl

Architecture constraints:
  - Six drones with deterministic IDs 0-5.
  - Hidden state persists across timer callbacks; reset only on episode reset.
  - No epsilon / random exploration / heuristic overrides.
  - No teleportation (no gz service set_pose).
  - Safety: stale/missing/NaN state → hover.
"""

import os
import time
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped

from marl_controller.state_builder import StateBuilder
from marl_controller.qmix_inference import QMIXInference
from marl_controller.action_mapper import ActionMapper
from marl_controller.motion_controller import MultiAgentMotionController


# ── Default configuration ────────────────────────────────────────

DEFAULT_MODEL_PATH = os.path.expanduser(
    "~/capstone_project_antigravity/marl_drone_project/models/"
    "qmix_n6_exp2/qmix_sar_v4_align_best.pth"
)
NUM_DRONES = 6
CONTROL_FREQUENCY = 20.0    # Hz for physical control loop
STALE_TIMEOUT = 3.0         # seconds before a drone's pose is considered stale
DRONE_ALTITUDE = 2.0        # fixed Z for the planar model
ACTION_NAMES = ["+X", "-X", "+Y", "-Y", "Hover"]


class ControllerNode(Node):
    """ROS 2 node that runs the trained N=6 QMIX policy in Gazebo."""

    def __init__(self):
        super().__init__("marl_controller")

        # ── Parameters (overridable via ROS 2 params) ────────────
        self.declare_parameter("num_drones", NUM_DRONES)
        self.declare_parameter("model_path", DEFAULT_MODEL_PATH)
        self.declare_parameter("control_frequency", CONTROL_FREQUENCY)
        self.declare_parameter("stale_timeout", STALE_TIMEOUT)
        self.declare_parameter("max_speed", 2.0)
        self.declare_parameter("arrival_tolerance", 0.10)
        self.declare_parameter("movement_timeout", 2.0)

        self.num_drones = self.get_parameter("num_drones").value
        model_path = self.get_parameter("model_path").value
        control_freq = self.get_parameter("control_frequency").value
        self.stale_timeout = self.get_parameter("stale_timeout").value
        max_speed = self.get_parameter("max_speed").value
        arrival_tolerance = self.get_parameter("arrival_tolerance").value
        movement_timeout = self.get_parameter("movement_timeout").value

        # ── Drone naming convention ──────────────────────────────
        # drone1 .. drone6  →  agent IDs 0..5
        self.drone_names = [f"drone{i+1}" for i in range(self.num_drones)]

        # ── Sub-components ───────────────────────────────────────
        self.state_builder = StateBuilder(num_drones=self.num_drones)
        self.action_mapper = ActionMapper(max_speed=max_speed)
        self.qmix = QMIXInference(
            model_path=model_path,
            num_drones=self.num_drones,
            device="cpu",
        )
        self.motion_controller = MultiAgentMotionController(
            num_drones=self.num_drones,
            action_mapper=self.action_mapper,
            max_speed=max_speed,
            arrival_tolerance=arrival_tolerance,
            movement_timeout=movement_timeout,
        )

        # ── Pose state tracking ──────────────────────────────────
        # World-frame (continuous) poses from Gazebo, keyed by agent index.
        self._world_poses: dict[int, tuple[float, float]] = {}
        self._pose_timestamps: dict[int, float] = {}

        # ── ROS 2 interfaces ────────────────────────────────────
        self._cmd_pubs: list = []
        self._pose_subs: list = []

        for idx in range(self.num_drones):
            name = self.drone_names[idx]

            # Publisher: velocity commands via ros_gz_bridge
            pub = self.create_publisher(
                Twist,
                f"/model/{name}/cmd_vel",
                10,
            )
            self._cmd_pubs.append(pub)

            # Subscriber: pose feedback from Gazebo via bridge
            sub = self.create_subscription(
                PoseStamped,
                f"/model/{name}/pose",
                lambda msg, i=idx: self._pose_callback(i, msg),
                10,
            )
            self._pose_subs.append(sub)

        # ── Episode state ────────────────────────────────────────
        self._step_count = 0
        self._episode_active = False

        # ── Control timer ────────────────────────────────────────
        self._timer = self.create_timer(1.0 / control_freq, self._control_callback)

        self.get_logger().info(
            f"✅ MARL Controller initialised  |  "
            f"drones={self.num_drones}  freq={control_freq}Hz  "
            f"model={os.path.basename(model_path)}"
        )

    # ── Episode lifecycle ────────────────────────────────────────

    def start_episode(self, obstacle_cells=None):
        """
        Begin a new evaluation episode.

        Resets hidden state, exploration grid, and step counter.
        Called explicitly (e.g. by a service or from main).
        """
        self.qmix.reset_episode()
        self.state_builder.reset_episode(obstacle_cells=obstacle_cells)
        self.motion_controller.prepare_next_step()
        self._step_count = 0
        self._episode_active = True
        self.get_logger().info("🔄 Episode started — hidden state & grid reset.")

    def stop_episode(self):
        """End the current episode. Hover all drones."""
        self._episode_active = False
        self._hover_all()
        self.get_logger().info("⏹  Episode stopped.")

    # ── Pose ingestion ───────────────────────────────────────────

    def update_pose(self, agent_idx: int, world_x: float, world_y: float):
        """
        Feed a drone's current Gazebo world-frame position.

        This is the primary data-ingestion entry point.  In the first
        integration the controller node calls this from ground-truth
        Gazebo pose data (e.g. /world/default/pose/info via bridge).
        """
        self._world_poses[agent_idx] = (world_x, world_y)
        self._pose_timestamps[agent_idx] = time.time()

    def _pose_callback(self, agent_idx: int, msg: PoseStamped):
        """Handle incoming PoseStamped for a specific drone."""
        world_x = msg.pose.position.x
        world_y = msg.pose.position.y
        self.update_pose(agent_idx, world_x, world_y)

    # ── Main control loop ────────────────────────────────────────

    def _control_callback(self):
        """Timer callback: fast physical control loop."""
        if not self._episode_active:
            return

        # 1. Check pose freshness for all drones
        now = time.time()
        all_valid = True
        current_world_positions = []

        for idx in range(self.num_drones):
            if idx not in self._world_poses:
                all_valid = False
            elif now - self._pose_timestamps[idx] > self.stale_timeout:
                self.get_logger().warn(f"Drone {idx} ({self.drone_names[idx]}): pose stale — hovering.")
                all_valid = False
            else:
                current_world_positions.append(self._world_poses[idx])

        if not all_valid:
            self._hover_all()
            return

        # 2. Check if a new QMIX policy decision is needed
        if self.motion_controller.all_complete():
            grid_positions = []
            for idx in range(self.num_drones):
                wx, wy = current_world_positions[idx]
                gx, gy = self.state_builder.world_to_grid(wx, wy)
                grid_positions.append((gx, gy))

            # Update state builder (marks cells explored, updates positions)
            self.state_builder.update_drone_positions(grid_positions)
            self.state_builder.update_bfs()

            # Build 49D observations for all drones
            observations = self.state_builder.get_all_states()  # list of 6 × (49,)

            # Validate — reject NaN / Inf
            obs_array = np.array(observations, dtype=np.float32)  # (6, 49)
            if np.any(np.isnan(obs_array)) or np.any(np.isinf(obs_array)):
                self.get_logger().error("NaN/Inf in observations — hovering all drones.")
                self._hover_all()
                return

            # QMIX inference
            actions = self.qmix.get_actions(obs_array)  # list of 6 ints

            # Dispatch actions to motion controller
            self.motion_controller.prepare_next_step()
            self.motion_controller.dispatch_actions(actions, grid_positions, self.state_builder.grid)

            self._step_count += 1

            # Coverage metric
            explored = np.sum(self.state_builder.grid == 1)
            valid = np.sum(self.state_builder.grid != -1)
            coverage = explored / max(1, valid)

            if self._step_count % 10 == 0:
                self.get_logger().info(
                    f"Step {self._step_count}  |  "
                    f"coverage={coverage:.2f}  |  "
                    f"actions={[ACTION_NAMES[a] for a in actions]}"
                )

        # 3. Physical control step
        twists = self.motion_controller.step(current_world_positions)

        for idx in range(self.num_drones):
            self._cmd_pubs[idx].publish(twists[idx])

    # ── Safety helpers ───────────────────────────────────────────

    def _hover_all(self):
        """Publish zero velocity to every drone."""
        stop = self.action_mapper.stop_twist()
        for pub in self._cmd_pubs:
            pub.publish(stop)

    def _publish_action(self, agent_idx: int, action: int):
        """Publish a single action to one drone."""
        twist = self.action_mapper.action_to_twist(action)
        self._cmd_pubs[agent_idx].publish(twist)


# ── Entry point ──────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    node.start_episode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_episode()
        node.destroy_node()
        rclpy.shutdown()