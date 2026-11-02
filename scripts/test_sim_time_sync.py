#!/usr/bin/env python3
"""
scripts/test_sim_time_sync.py

Unit tests for simulation-time synchronization in VictimManager:
1. Normal simulation time progression advances moving victims.
2. Paused simulation (dt = 0) does NOT advance victims or timers.
3. Discontinuous backward jump (clock reset / rewind) does NOT crash or move backward.
4. Large jump (dt > 2.0s) resyncs without teleportation.
5. First update initializes cleanly without delta step.
"""

import unittest
import math
import sys
import os

WORKSPACE = "/home/capstone/capstone_project_antigravity"
sys.path.insert(0, os.path.join(WORKSPACE, "drone_ws/src/swarm_controller"))
sys.path.insert(0, os.path.join(WORKSPACE, "marl_drone_project/env"))

from swarm_controller.victim_manager import VictimManager, VictimState, MovementState
from sar_env import SARGridEnv

class MockClock:
    def __init__(self, initial_sim_time_sec=0.0):
        self.sim_time_sec = initial_sim_time_sec

    def now(self):
        class TimeObj:
            def __init__(self, sec):
                self.nanoseconds = int(sec * 1e9)
        return TimeObj(self.sim_time_sec)

class MockNode:
    def __init__(self, initial_time=100.0):
        self.clock = MockClock(initial_time)

    def get_clock(self):
        return self.clock

    def get_logger(self):
        class Logger:
            def info(self, msg): pass
            def warn(self, msg): pass
            def error(self, msg): pass
        return Logger()

class TestVictimManagerSimTimeSync(unittest.TestCase):
    def setUp(self):
        self.mock_node = MockNode(initial_time=100.0)
        self.env = SARGridEnv(x_size=25, y_size=25, num_drones=2)
        self.env.reset()
        
        self.metadata = {
            "victims": [
                {"id": "v_stat", "grid": {"x": 5, "y": 5}, "world": {"x": 22.0, "y": 22.0, "z": 0.2}},
                {"id": "v_slow", "grid": {"x": 10, "y": 10}, "world": {"x": 42.0, "y": 42.0, "z": 0.2}},
            ]
        }
        self.vm = VictimManager(self.mock_node, self.metadata, "test_world", self.env, seed=42)

    def test_first_update_initializes_cleanly(self):
        # Initial state
        v_slow = self.vm.victims["v_slow"]
        initial_x = v_slow.world_x
        initial_y = v_slow.world_y

        # Call update at initial sim time
        self.vm.update()
        
        # Positions must be unchanged on the very first update
        self.assertEqual(v_slow.world_x, initial_x)
        self.assertEqual(v_slow.world_y, initial_y)
        self.assertEqual(self.vm.last_update_time, 100.0)

    def test_paused_simulation_does_not_advance_victims(self):
        self.vm.update() # First call initializes
        v_slow = self.vm.victims["v_slow"]
        
        # Force victim into MOVING state towards (50.0, 50.0)
        v_slow.movement_state = MovementState.MOVING
        v_slow.dest_world_x = 50.0
        v_slow.dest_world_y = 50.0
        v_slow.speed = 1.0 # 1.0 m/s
        
        pos_before = (v_slow.world_x, v_slow.world_y)

        # Simulation is PAUSED (clock remains 100.0)
        for _ in range(10):
            self.vm.update()
            
        self.assertEqual(v_slow.world_x, pos_before[0])
        self.assertEqual(v_slow.world_y, pos_before[1])

    def test_normal_simulation_advances_victims(self):
        self.vm.update() # First call initializes at t=100.0
        v_slow = self.vm.victims["v_slow"]
        
        v_slow.movement_state = MovementState.MOVING
        v_slow.dest_world_x = 52.0
        v_slow.dest_world_y = 42.0 # Move purely +X
        v_slow.speed = 1.0 # 1 m/s
        
        x_before = v_slow.world_x

        # Advance sim time by 0.5s
        self.mock_node.clock.sim_time_sec = 100.5
        self.vm.update()

        # Should have moved exactly speed * dt = 1.0 * 0.5 = 0.5m
        self.assertAlmostEqual(v_slow.world_x, x_before + 0.5, places=2)

    def test_clock_reset_rewind_does_not_move_backward(self):
        self.vm.update() # Initialized at 100.0
        v_slow = self.vm.victims["v_slow"]
        v_slow.movement_state = MovementState.MOVING
        v_slow.dest_world_x = 52.0
        v_slow.dest_world_y = 42.0
        v_slow.speed = 1.0

        pos_before = (v_slow.world_x, v_slow.world_y)

        # Simulation rewinds / resets to t=50.0
        self.mock_node.clock.sim_time_sec = 50.0
        self.vm.update()

        # Position should NOT jump backward or blow up
        self.assertEqual(v_slow.world_x, pos_before[0])
        self.assertEqual(self.vm.last_update_time, 50.0)

    def test_large_time_jump_is_clamped_safely(self):
        self.vm.update() # Initialized at 100.0
        v_slow = self.vm.victims["v_slow"]
        v_slow.movement_state = MovementState.MOVING
        v_slow.dest_world_x = 52.0
        v_slow.dest_world_y = 42.0
        v_slow.speed = 1.0

        pos_before = (v_slow.world_x, v_slow.world_y)

        # Huge jump of 50 seconds (e.g. paused for a long time)
        self.mock_node.clock.sim_time_sec = 150.0
        self.vm.update()

        # Should resync without teleporting across the world
        self.assertEqual(v_slow.world_x, pos_before[0])
        self.assertEqual(self.vm.last_update_time, 150.0)

if __name__ == "__main__":
    unittest.main()
