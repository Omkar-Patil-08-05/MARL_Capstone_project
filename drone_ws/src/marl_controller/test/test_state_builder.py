#!/usr/bin/env python3
"""
Cross-validation test: StateBuilder vs SARGridEnv.

Seeds a SARGridEnv, copies its exact grid and drone positions into a
StateBuilder, then asserts that get_agent_state() produces bit-identical
49D observations for all 6 agents across multiple timesteps.

Run:
  python3 drone_ws/src/marl_controller/test/test_state_builder.py
"""

import sys
import os
import numpy as np

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
MARL_ROOT = os.path.join(PROJECT_ROOT, "marl_drone_project")
CTRL_ROOT = os.path.join(PROJECT_ROOT, "drone_ws", "src", "marl_controller", "marl_controller")
sys.path.insert(0, MARL_ROOT)
sys.path.insert(0, CTRL_ROOT)

from env.sar_env import SARGridEnv
from state_builder import StateBuilder

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

# ── Setup ────────────────────────────────────────────────────────
env = SARGridEnv(num_drones=NUM_DRONES, max_steps=300, seed=42)
env.reset(episode_num=2000)

sb = StateBuilder(num_drones=NUM_DRONES)

# Copy grid and obstacle layout from env
sb.grid = env.grid.copy()
sb.drone_positions = [tuple(p) for p in env.drone_positions]
sb.update_bfs()

print("=" * 60)
print("STATE BUILDER CROSS-VALIDATION TEST")
print("=" * 60)
print()

# ── Test 1: Initial observations match ───────────────────────────
print("─── Initial state (after reset) ───")
for agent_id in range(NUM_DRONES):
    env_obs = env.get_agent_state(agent_id)
    sb_obs = sb.get_agent_state(agent_id)
    match = np.allclose(env_obs, sb_obs, atol=1e-6)
    check(f"Agent {agent_id}: observations match",
          match,
          f"max_diff={np.max(np.abs(env_obs - sb_obs)):.8f}" if not match else "")

print()

# ── Test 2: Dimension and dtype ──────────────────────────────────
print("─── Dimension & dtype ───")
for agent_id in range(NUM_DRONES):
    obs = sb.get_agent_state(agent_id)
    check(f"Agent {agent_id}: shape == (49,)", obs.shape == (49,), f"got {obs.shape}")
    check(f"Agent {agent_id}: dtype == float32", obs.dtype == np.float32, f"got {obs.dtype}")
    check(f"Agent {agent_id}: no NaN", not np.any(np.isnan(obs)))
    check(f"Agent {agent_id}: no Inf", not np.any(np.isinf(obs)))

print()

# ── Test 3: Simulate several steps and compare ──────────────────
print("─── Multi-step simulation (5 steps) ───")
np.random.seed(123)
for step_i in range(5):
    # Random actions for all drones
    actions = [np.random.randint(0, 5) for _ in range(NUM_DRONES)]
    env.step(actions)

    # Sync state builder to env
    sb.grid = env.grid.copy()
    sb.drone_positions = [tuple(p) for p in env.drone_positions]
    sb.update_bfs()

    all_match = True
    max_diff = 0.0
    for agent_id in range(NUM_DRONES):
        env_obs = env.get_agent_state(agent_id)
        sb_obs = sb.get_agent_state(agent_id)
        diff = np.max(np.abs(env_obs - sb_obs))
        max_diff = max(max_diff, diff)
        if not np.allclose(env_obs, sb_obs, atol=1e-6):
            all_match = False

    check(f"Step {step_i+1}: all 6 agents match  (max_diff={max_diff:.8f})",
          all_match)

print()

# ── Test 4: BFS frontier consistency ─────────────────────────────
print("─── BFS frontier ───")
check("bfs_dist_map shape matches",
      sb.bfs_dist_map.shape == env.bfs_dist_map.shape)
check("bfs_dist_map values match",
      np.allclose(sb.bfs_dist_map, env.bfs_dist_map, equal_nan=True))
check("bfs_next_step shape matches",
      sb.bfs_next_step.shape == env.bfs_next_step.shape)
check("bfs_next_step values match",
      np.allclose(sb.bfs_next_step, env.bfs_next_step))

print()

# ── Test 5: Coordinate conversion round-trip ─────────────────────
print("─── Coordinate conversion ───")
for gx in range(25):
    for gy in range(25):
        wx, wy = sb.grid_to_world(gx, gy)
        rgx, rgy = sb.world_to_grid(wx, wy)
        if (rgx, rgy) != (gx, gy):
            check(f"Round-trip ({gx},{gy})", False, f"got ({rgx},{rgy})")
            break
    else:
        continue
    break
else:
    check("grid↔world round-trip for all 625 cells", True)

# Verify origin convention
wx0, wy0 = sb.grid_to_world(0, 0)
check("Grid (0,0) → world (-12, -12)", wx0 == -12.0 and wy0 == -12.0,
      f"got ({wx0}, {wy0})")

print()

# ── Test 6: reset_episode clears state ───────────────────────────
print("─── Episode reset ───")
sb.reset_episode()
check("Grid is all zeros after reset",
      np.all(sb.grid == 0))
check("Drone positions reset to (0,0)",
      all(p == (0, 0) for p in sb.drone_positions))

print()

# ── Summary ──────────────────────────────────────────────────────
total = passed + failed
print("=" * 60)
print(f"RESULTS:  {passed}/{total} passed,  {failed} failed")
print("=" * 60)

sys.exit(0 if failed == 0 else 1)
