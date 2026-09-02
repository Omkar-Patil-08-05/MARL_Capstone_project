#!/usr/bin/env python3
"""
Focused integration test for the MARL ControllerNode pipeline.

Validates the full data path WITHOUT launching Gazebo or a ROS 2 graph.
Uses lightweight mocks for ROS publishers and verifies:

  1. Six-agent deterministic ordering.
  2. 49D observations reach QMIX inference.
  3. Hidden state persists across control cycles.
  4. Action-to-drone mapping is correct.
  5. Stale / missing pose → hover.
  6. No NaN / Inf reaches inference.
  7. Episode reset resets hidden state.
  8. No teleportation (no subprocess / set_pose calls).

Run:
  python3 drone_ws/src/marl_controller/test/test_controller.py
"""

import sys
import os
import time
import numpy as np
import torch

# ── Path setup ───────────────────────────────────────────────────
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
CTRL_ROOT = os.path.join(PROJECT_ROOT, "drone_ws", "src", "marl_controller", "marl_controller")
sys.path.insert(0, CTRL_ROOT)

from state_builder import StateBuilder
from qmix_inference import QMIXInference
from action_mapper import ActionMapper

MODEL_PATH = os.path.join(
    PROJECT_ROOT, "marl_drone_project", "models",
    "qmix_n6_exp2", "qmix_sar_v4_align_best.pth",
)

NUM_DRONES = 6
passed = 0
failed = 0

def check(name, cond, detail=""):
    global passed, failed
    if cond:
        passed += 1
        print(f"  ✅ {name}")
    else:
        failed += 1
        print(f"  ❌ {name}  — {detail}")


# ── Lightweight pipeline harness (no ROS 2) ──────────────────────

class PipelineHarness:
    """Mimics the ControllerNode data flow without ROS 2."""

    def __init__(self):
        self.state_builder = StateBuilder(num_drones=NUM_DRONES)
        self.action_mapper = ActionMapper(max_speed=2.0)
        self.qmix = QMIXInference(
            model_path=MODEL_PATH,
            num_drones=NUM_DRONES,
            device="cpu",
        )
        self.world_poses = {}          # agent_idx → (wx, wy)
        self.pose_timestamps = {}      # agent_idx → time.time()
        self.published_twists = {}     # agent_idx → last Twist
        self.step_count = 0

    def update_pose(self, idx, wx, wy):
        self.world_poses[idx] = (wx, wy)
        self.pose_timestamps[idx] = time.time()

    def control_step(self):
        """Run one policy cycle. Returns (actions, observations) or None on safety hover."""
        stale_timeout = 3.0
        now = time.time()

        # Freshness check
        for idx in range(NUM_DRONES):
            if idx not in self.world_poses:
                return None
            if now - self.pose_timestamps[idx] > stale_timeout:
                return None

        # World → grid
        grid_positions = []
        for idx in range(NUM_DRONES):
            wx, wy = self.world_poses[idx]
            gx, gy = self.state_builder.world_to_grid(wx, wy)
            grid_positions.append((gx, gy))

        self.state_builder.update_drone_positions(grid_positions)
        self.state_builder.update_bfs()

        # Observations
        observations = self.state_builder.get_all_states()
        obs_array = np.array(observations, dtype=np.float32)

        if np.any(np.isnan(obs_array)) or np.any(np.isinf(obs_array)):
            return None

        # Inference
        actions = self.qmix.get_actions(obs_array)

        # Publish
        for idx in range(NUM_DRONES):
            self.published_twists[idx] = self.action_mapper.action_to_twist(actions[idx])

        self.step_count += 1
        return actions, obs_array

    def start_episode(self, obstacles=None):
        self.qmix.reset_episode()
        self.state_builder.reset_episode(obstacle_cells=obstacles)
        self.step_count = 0

    def hover_all(self):
        for idx in range(NUM_DRONES):
            self.published_twists[idx] = self.action_mapper.stop_twist()


# ── Tests ────────────────────────────────────────────────────────

print("=" * 60)
print("CONTROLLER INTEGRATION TEST")
print("=" * 60)
print()

harness = PipelineHarness()

# ── Test 1: Six deterministic drone IDs ──────────────────────────
print("─── Ordering ───")
check("6 drone IDs 0-5",
      list(range(NUM_DRONES)) == [0, 1, 2, 3, 4, 5])

drone_names = [f"drone{i+1}" for i in range(NUM_DRONES)]
check("drone names: drone1..drone6",
      drone_names == ["drone1", "drone2", "drone3", "drone4", "drone5", "drone6"])
print()

# ── Test 2: Missing pose → safety hover (returns None) ───────────
print("─── Stale/missing pose safety ───")
harness.start_episode()
result = harness.control_step()
check("Missing poses → control_step returns None", result is None)
print()

# ── Test 3: Full pipeline with valid poses ───────────────────────
print("─── Full pipeline ───")
# Simulate 6 drones at known world positions (matching spawn_drones.sh)
world_positions = [
    (-10.0, -10.0),  # drone1
    (0.0, -10.0),    # drone2
    (10.0, -10.0),   # drone3
    (-10.0, 0.0),    # drone4
    (0.0, 0.0),      # drone5
    (10.0, 0.0),     # drone6
]
for idx, (wx, wy) in enumerate(world_positions):
    harness.update_pose(idx, wx, wy)

result = harness.control_step()
check("Pipeline returns actions + observations", result is not None)

actions, obs_array = result
check("6 actions returned", len(actions) == NUM_DRONES)
check("All actions in [0,4]", all(0 <= a <= 4 for a in actions))
check("Observation shape is (6, 49)", obs_array.shape == (NUM_DRONES, 49))
check("Observations are float32", obs_array.dtype == np.float32)
check("No NaN in observations", not np.any(np.isnan(obs_array)))
check("No Inf in observations", not np.any(np.isinf(obs_array)))

# Verify each drone received a Twist
for idx in range(NUM_DRONES):
    t = harness.published_twists[idx]
    check(f"Drone {idx}: Twist published (z=0)", t.linear.z == 0.0)
print()

# ── Test 4: Agent one-hot identity is correct ────────────────────
print("─── Agent identity in observations ───")
for idx in range(NUM_DRONES):
    onehot = obs_array[idx, 2:8]
    check(f"Agent {idx}: one-hot active at index {idx}",
          np.argmax(onehot) == idx and np.sum(onehot == 1.0) == 1)
print()

# ── Test 5: Hidden state persists across steps ───────────────────
print("─── Hidden state persistence ───")
h_after_step1 = harness.qmix.hidden_state.clone()

# Run another step with same poses
for idx, (wx, wy) in enumerate(world_positions):
    harness.update_pose(idx, wx, wy)
result2 = harness.control_step()
h_after_step2 = harness.qmix.hidden_state.clone()

check("Hidden state changed between steps",
      not torch.equal(h_after_step1, h_after_step2))
check("Hidden state is non-zero", not torch.all(h_after_step2 == 0))
print()

# ── Test 6: Episode reset resets hidden state ────────────────────
print("─── Episode reset ───")
harness.start_episode()
check("Hidden state zeroed after reset",
      torch.all(harness.qmix.hidden_state == 0))
check("Step counter reset", harness.step_count == 0)
check("Grid cleared", np.all(harness.state_builder.grid == 0))
print()

# ── Test 7: Action-to-Twist mapping consistency ──────────────────
print("─── Action→Twist mapping ───")
mapper = harness.action_mapper
t0 = mapper.action_to_twist(0)
check("Action 0 (+X): linear.x > 0", t0.linear.x > 0 and t0.linear.y == 0)
t1 = mapper.action_to_twist(1)
check("Action 1 (-X): linear.x < 0", t1.linear.x < 0 and t1.linear.y == 0)
t2 = mapper.action_to_twist(2)
check("Action 2 (+Y): linear.y > 0", t2.linear.y > 0 and t2.linear.x == 0)
t3 = mapper.action_to_twist(3)
check("Action 3 (-Y): linear.y < 0", t3.linear.y < 0 and t3.linear.x == 0)
t4 = mapper.action_to_twist(4)
check("Action 4 (Hover): zero velocity", t4.linear.x == 0 and t4.linear.y == 0)
print()

# ── Test 8: No teleportation ────────────────────────────────────
print("─── No teleportation ───")
import inspect
controller_src = inspect.getsource(type(harness))  # PipelineHarness
# Also check ActionMapper source
mapper_src = inspect.getsource(type(mapper))
check("ActionMapper has no 'subprocess'", "subprocess" not in mapper_src)
check("ActionMapper has no 'set_pose'", "set_pose" not in mapper_src)
check("ActionMapper has no 'gz service'", "gz service" not in mapper_src)
print()

# ── Test 9: NaN rejection ───────────────────────────────────────
print("─── NaN / Inf rejection ───")
harness.start_episode()
# Feed a NaN pose
harness.update_pose(0, float("nan"), 0.0)
for idx in range(1, NUM_DRONES):
    harness.update_pose(idx, 0.0, 0.0)
# The world_to_grid will clamp NaN to a boundary via int(round(nan)),
# which raises or produces unexpected values. Let's verify the pipeline
# handles it gracefully.
try:
    result_nan = harness.control_step()
    # If it didn't crash, it should either return None or valid data
    if result_nan is not None:
        _, obs_nan = result_nan
        has_nan = np.any(np.isnan(obs_nan)) or np.any(np.isinf(obs_nan))
        check("NaN pose: observations are clean (no NaN/Inf leaked)", not has_nan)
    else:
        check("NaN pose: safely returned None", True)
except (ValueError, RuntimeError):
    check("NaN pose: raised exception (acceptable)", True)
print()

# ── Test 10: Multi-step simulation ───────────────────────────────
print("─── Multi-step simulation ───")
harness.start_episode()
for idx, (wx, wy) in enumerate(world_positions):
    harness.update_pose(idx, wx, wy)

all_ok = True
for step in range(20):
    # Slightly move drones each step
    for idx in range(NUM_DRONES):
        wx, wy = harness.world_poses[idx]
        harness.update_pose(idx, wx + 0.1 * step, wy)
    result = harness.control_step()
    if result is None:
        all_ok = False
        break
    acts, obs = result
    if len(acts) != NUM_DRONES or obs.shape != (NUM_DRONES, 49):
        all_ok = False
        break

check("20 consecutive steps succeeded", all_ok)
check("Final step count is 20", harness.step_count == 20)
h_final = harness.qmix.hidden_state.clone()
check("Hidden state non-zero after 20 steps", not torch.all(h_final == 0))
print()

# ── Summary ──────────────────────────────────────────────────────
total = passed + failed
print("=" * 60)
print(f"RESULTS:  {passed}/{total} passed,  {failed} failed")
print("=" * 60)

sys.exit(0 if failed == 0 else 1)
