#!/usr/bin/env python3
"""
Standalone validation test for the QMIX ROS 2 bridge components.

Tests:
  Part A – QMIXInference: checkpoint loading, batched inference,
           hidden-state persistence, episode reset, greedy argmax.
  Part B – ActionMapper: discrete→Twist mapping, grid↔world helpers.

Run from the project root:
  python3 drone_ws/src/marl_controller/marl_controller/test_bridge.py
"""

import sys
import os
import numpy as np

# ── Path setup so we can import without colcon build ─────────────
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, THIS_DIR)

# ── PART A: QMIX Inference ──────────────────────────────────────
print("=" * 60)
print("PART A — QMIXInference Validation")
print("=" * 60)

from qmix_inference import QMIXInference
import torch

MODEL_PATH = os.path.join(
    os.path.dirname(__file__),
    "..", "..", "..", "..",        # drone_ws/src/marl_controller/marl_controller → project root
    "marl_drone_project", "models", "qmix_n6_exp2", "qmix_sar_v4_align_best.pth",
)
MODEL_PATH = os.path.abspath(MODEL_PATH)
assert os.path.isfile(MODEL_PATH), f"Checkpoint not found: {MODEL_PATH}"

NUM_DRONES = 6
OBS_DIM = 49
HIDDEN_DIM = 64

qmix = QMIXInference(
    model_path=MODEL_PATH,
    num_drones=NUM_DRONES,
    obs_dim=OBS_DIM,
    rnn_hidden_dim=HIDDEN_DIM,
    device="cpu",
)

# Test 1 — Initial hidden state shape
assert qmix.hidden_state.shape == (NUM_DRONES, HIDDEN_DIM), \
    f"Hidden state shape mismatch: {qmix.hidden_state.shape}"
print("✅ Test 1: Hidden state shape is (6, 64)")

# Test 2 — Hidden state starts at zero
assert torch.all(qmix.hidden_state == 0), "Hidden state not zeroed after init"
print("✅ Test 2: Hidden state initialised to zeros")

# Test 3 — Single inference call
obs = np.random.randn(NUM_DRONES, OBS_DIM).astype(np.float32)
actions = qmix.get_actions(obs)
assert len(actions) == NUM_DRONES, f"Expected {NUM_DRONES} actions, got {len(actions)}"
assert all(0 <= a <= 4 for a in actions), f"Invalid action values: {actions}"
print(f"✅ Test 3: Got {NUM_DRONES} valid actions: {actions}")

# Test 4 — Hidden state is now non-zero (persists)
assert not torch.all(qmix.hidden_state == 0), "Hidden state should be non-zero after inference"
h_after_step1 = qmix.hidden_state.clone()
print("✅ Test 4: Hidden state persists (non-zero after inference)")

# Test 5 — Second inference updates hidden state further
obs2 = np.random.randn(NUM_DRONES, OBS_DIM).astype(np.float32)
actions2 = qmix.get_actions(obs2)
assert len(actions2) == NUM_DRONES
h_after_step2 = qmix.hidden_state.clone()
assert not torch.equal(h_after_step1, h_after_step2), \
    "Hidden state should change between inference steps"
print(f"✅ Test 5: Hidden state updated on second call: {actions2}")

# Test 6 — reset_episode() zeros hidden state
qmix.reset_episode()
assert torch.all(qmix.hidden_state == 0), "Hidden state not zeroed after reset"
print("✅ Test 6: reset_episode() correctly zeros hidden state")

# Test 7 — No gradients produced
for p in qmix.agent_net.parameters():
    assert p.grad is None, "Gradients should not exist in eval mode"
print("✅ Test 7: No gradients created during inference")

# Test 8 — Model is in eval mode
assert not qmix.agent_net.training, "Model should be in eval mode"
print("✅ Test 8: Model is in eval mode")

# Test 9 — Multiple sequential steps (simulating an episode)
qmix.reset_episode()
for step in range(10):
    obs_step = np.random.randn(NUM_DRONES, OBS_DIM).astype(np.float32)
    acts = qmix.get_actions(obs_step)
    assert len(acts) == NUM_DRONES
    assert all(0 <= a <= 4 for a in acts)
print("✅ Test 9: 10 sequential inference steps succeeded")

print()

# ── PART B: ActionMapper ────────────────────────────────────────
print("=" * 60)
print("PART B — ActionMapper Validation")
print("=" * 60)

# Minimal mock of geometry_msgs so we can test without ROS 2 installed
# If ROS 2 is available, the real import in action_mapper.py will work.
# For standalone testing we provide a shim.
try:
    from action_mapper import ActionMapper
    print("  (imported ActionMapper with real ROS 2 geometry_msgs)")
except ImportError:
    # Create a lightweight Twist stand-in
    class _Vec3:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    class _Twist:
        def __init__(self):
            self.linear = _Vec3()
            self.angular = _Vec3()

    # Patch the import
    import types
    fake_msg = types.ModuleType("geometry_msgs.msg")
    fake_msg.Twist = _Twist
    sys.modules["geometry_msgs"] = types.ModuleType("geometry_msgs")
    sys.modules["geometry_msgs.msg"] = fake_msg
    from action_mapper import ActionMapper
    print("  (imported ActionMapper with mock Twist)")

mapper = ActionMapper(max_speed=2.0, grid_size=25, cell_size=1.0)

# Test B1 — Action 0 → +X
t = mapper.action_to_twist(0)
assert t.linear.x == 2.0 and t.linear.y == 0.0
print("✅ Test B1: Action 0 → +X (2.0, 0.0)")

# Test B2 — Action 1 → −X
t = mapper.action_to_twist(1)
assert t.linear.x == -2.0 and t.linear.y == 0.0
print("✅ Test B2: Action 1 → −X (-2.0, 0.0)")

# Test B3 — Action 2 → +Y
t = mapper.action_to_twist(2)
assert t.linear.x == 0.0 and t.linear.y == 2.0
print("✅ Test B3: Action 2 → +Y (0.0, 2.0)")

# Test B4 — Action 3 → −Y
t = mapper.action_to_twist(3)
assert t.linear.x == 0.0 and t.linear.y == -2.0
print("✅ Test B4: Action 3 → −Y (0.0, -2.0)")

# Test B5 — Action 4 → Hover
t = mapper.action_to_twist(4)
assert t.linear.x == 0.0 and t.linear.y == 0.0
print("✅ Test B5: Action 4 → Hover (0.0, 0.0)")

# Test B6 — stop_twist
t = mapper.stop_twist()
assert t.linear.x == 0.0 and t.linear.y == 0.0 and t.linear.z == 0.0
print("✅ Test B6: stop_twist() → zero velocity")

# Test B7 — Grid target with boundary clamping
assert mapper.get_target_grid_cell(0, 0, 1) == (0, 0), "Should clamp at boundary"
assert mapper.get_target_grid_cell(24, 24, 0) == (24, 24), "Should clamp at boundary"
assert mapper.get_target_grid_cell(12, 12, 0) == (13, 12)
assert mapper.get_target_grid_cell(12, 12, 3) == (12, 11)
assert mapper.get_target_grid_cell(12, 12, 4) == (12, 12), "Hover → same cell"
print("✅ Test B7: Grid target computation with clamping")

# Test B8 — grid↔world round-trip
for gx in range(25):
    for gy in range(25):
        wx, wy, _ = mapper.grid_to_world(gx, gy)
        rgx, rgy = mapper.world_to_grid(wx, wy)
        assert (rgx, rgy) == (gx, gy), f"Round-trip failed: ({gx},{gy}) → ({wx},{wy}) → ({rgx},{rgy})"
print("✅ Test B8: grid↔world round-trip for all 625 cells")

# Test B9 — World origin matches spawn convention
wx, wy, _ = mapper.grid_to_world(0, 0)
assert wx == -12.0 and wy == -12.0, f"Grid (0,0) should map to world (-12,-12), got ({wx},{wy})"
wx, wy, _ = mapper.grid_to_world(12, 12)
assert wx == 0.0 and wy == 0.0, f"Grid (12,12) should map to world (0,0), got ({wx},{wy})"
print("✅ Test B9: Coordinate convention matches spawn_drones.sh OFFSET=-12")

print()
print("=" * 60)
print("ALL TESTS PASSED")
print("=" * 60)
