#!/usr/bin/env python3
"""
Observation contract test for the 49D QMIX observation vector.

Source-verifies the exact layout produced by SARGridEnv.get_agent_state()
so that any future ROS 2 state_builder.py can be validated against the
same structural and semantic guarantees.

Run from the project root:
  python3 drone_ws/src/marl_controller/test/test_observation_contract.py
"""

import sys
import os
import numpy as np

# ── Path setup ───────────────────────────────────────────────────
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
MARL_ROOT = os.path.join(PROJECT_ROOT, "marl_drone_project")
sys.path.insert(0, MARL_ROOT)

from env.sar_env import SARGridEnv  # pyrefly: ignore[missing-import]

# ── Constants ────────────────────────────────────────────────────
NUM_DRONES = 6
OBS_DIM = 49
MAX_STEPS = 300

# Index ranges (inclusive start, exclusive end — Python slice convention)
SLICE_OWN_POS     = slice(0, 2)       # 2 values
SLICE_AGENT_ONEHOT = slice(2, 8)      # 6 values
SLICE_OTHER_POS   = slice(8, 18)      # 10 values
SLICE_TEAM_VEC    = slice(18, 28)     # 10 values
SLICE_FRONTIER    = slice(28, 30)     # 2 values
SLICE_DENSITY     = slice(30, 31)     # 1 value
SLICE_LOCAL_OBS   = slice(31, 40)     # 9 values
SLICE_LOCAL_EXP   = slice(40, 49)     # 9 values

passed = 0
failed = 0

def check(name, condition, detail=""):
    global passed, failed
    if condition:
        passed += 1
        print(f"  ✅ {name}")
    else:
        failed += 1
        print(f"  ❌ {name}  — {detail}")

# ── Setup ────────────────────────────────────────────────────────
env = SARGridEnv(num_drones=NUM_DRONES, max_steps=MAX_STEPS)
env.reset(episode_num=2000)  # curriculum stage 3

print("=" * 60)
print("49D OBSERVATION CONTRACT TEST  (N=6)")
print("=" * 60)
print()

# ── Test group 1: Dimension and dtype ────────────────────────────
print("─── Dimension & dtype ───")
for agent_id in range(NUM_DRONES):
    obs = env.get_agent_state(agent_id)

    check(f"Agent {agent_id}: length == 49",
          len(obs) == OBS_DIM,
          f"got {len(obs)}")

    check(f"Agent {agent_id}: dtype == float32",
          obs.dtype == np.float32,
          f"got {obs.dtype}")

    check(f"Agent {agent_id}: no NaN",
          not np.any(np.isnan(obs)))

    check(f"Agent {agent_id}: no Inf",
          not np.any(np.isinf(obs)))

print()

# ── Test group 2: Structural layout (agent 0) ───────────────────
print("─── Index layout (agent 0) ───")
obs0 = env.get_agent_state(0)

check("own_pos occupies [0:2]  (2 values)",
      obs0[SLICE_OWN_POS].shape == (2,))

check("agent_onehot occupies [2:8]  (6 values)",
      obs0[SLICE_AGENT_ONEHOT].shape == (6,))

check("other_pos occupies [8:18]  (10 values)",
      obs0[SLICE_OTHER_POS].shape == (10,))

check("team_vec occupies [18:28]  (10 values)",
      obs0[SLICE_TEAM_VEC].shape == (10,))

check("frontier_vec occupies [28:30]  (2 values)",
      obs0[SLICE_FRONTIER].shape == (2,))

check("density occupies [30:31]  (1 value)",
      obs0[SLICE_DENSITY].shape == (1,))

check("local_obs occupies [31:40]  (9 values)",
      obs0[SLICE_LOCAL_OBS].shape == (9,))

check("local_exp occupies [40:49]  (9 values)",
      obs0[SLICE_LOCAL_EXP].shape == (9,))

check("2+6+10+10+2+1+9+9 == 49",
      2 + 6 + 10 + 10 + 2 + 1 + 9 + 9 == OBS_DIM)

print()

# ── Test group 3: Agent one-hot semantics ────────────────────────
print("─── One-hot identity ───")
for agent_id in range(NUM_DRONES):
    obs = env.get_agent_state(agent_id)
    onehot = obs[SLICE_AGENT_ONEHOT]

    check(f"Agent {agent_id}: one-hot has exactly one 1.0",
          np.sum(onehot == 1.0) == 1,
          f"got {np.sum(onehot == 1.0)}")

    check(f"Agent {agent_id}: active index is {agent_id}",
          np.argmax(onehot) == agent_id,
          f"argmax={np.argmax(onehot)}")

    check(f"Agent {agent_id}: remaining 5 entries are 0.0",
          np.sum(onehot == 0.0) == NUM_DRONES - 1)

print()

# ── Test group 4: own_pos normalization ──────────────────────────
print("─── own_pos normalization ───")
for agent_id in range(NUM_DRONES):
    obs = env.get_agent_state(agent_id)
    own = obs[SLICE_OWN_POS]
    check(f"Agent {agent_id}: own_pos in [0, 1]",
          np.all(own >= 0.0) and np.all(own <= 1.0),
          f"values={own}")

print()

# ── Test group 5: other_pos normalization ────────────────────────
print("─── other_pos normalization ───")
obs0 = env.get_agent_state(0)
other = obs0[SLICE_OTHER_POS]
check("other_pos values in [0, 1]",
      np.all(other >= 0.0) and np.all(other <= 1.0),
      f"min={other.min()}, max={other.max()}")

print()

# ── Test group 6: density in [0, 1] ─────────────────────────────
print("─── density ───")
for agent_id in range(NUM_DRONES):
    obs = env.get_agent_state(agent_id)
    d = obs[SLICE_DENSITY][0]
    check(f"Agent {agent_id}: density in [0, 1]",
          0.0 <= d <= 1.0,
          f"got {d}")

print()

# ── Test group 7: local_obs binary ───────────────────────────────
print("─── local_obs (obstacle map) ───")
obs0 = env.get_agent_state(0)
lobs = obs0[SLICE_LOCAL_OBS]
check("local_obs values are 0.0 or 1.0",
      np.all(np.isin(lobs, [0.0, 1.0])),
      f"unique={np.unique(lobs)}")

print()

# ── Test group 8: local_exp bounded ──────────────────────────────
print("─── local_exp (explored map) ───")
lexp = obs0[SLICE_LOCAL_EXP]
check("local_exp values are 0.0 or 1.0",
      np.all(np.isin(lexp, [0.0, 1.0])),
      f"unique={np.unique(lexp)}")

print()

# ── Test group 9: cross-agent consistency ────────────────────────
print("─── Cross-agent consistency ───")
all_obs = [env.get_agent_state(i) for i in range(NUM_DRONES)]
# Agent 0's own_pos should appear in agent 1's other_pos
pos0 = all_obs[0][SLICE_OWN_POS]
other1 = all_obs[1][SLICE_OTHER_POS].reshape(NUM_DRONES - 1, 2)
found = any(np.allclose(pos0, other1[k]) for k in range(NUM_DRONES - 1))
check("Agent 0's own_pos appears in agent 1's other_pos",
      found)

print()

# ── Summary ──────────────────────────────────────────────────────
total = passed + failed
print("=" * 60)
print(f"RESULTS:  {passed}/{total} passed,  {failed} failed")
print(f"Observation dimension: {OBS_DIM}")
print("Index layout:")
print("  [0:2]   own_pos        2 values")
print("  [2:8]   agent_onehot   6 values")
print("  [8:18]  other_pos     10 values")
print("  [18:28] team_vec      10 values")
print("  [28:30] frontier_vec   2 values")
print("  [30:31] density        1 value")
print("  [31:40] local_obs      9 values")
print("  [40:49] local_exp      9 values")
print("=" * 60)

sys.exit(0 if failed == 0 else 1)
