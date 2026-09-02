import time
import pytest
import numpy as np
from marl_controller.motion_controller import (
    MultiAgentMotionController,
    STATE_READY, STATE_MOVING, STATE_ARRIVED, STATE_INVALID_TARGET, STATE_COMPLETE
)
from marl_controller.action_mapper import ActionMapper

def test_motion_controller_hover():
    mc = MultiAgentMotionController(num_drones=1)
    grid = np.zeros((25, 25), dtype=np.int8)

    mc.dispatch_actions([ActionMapper.ACTION_HOVER], [(12, 12)], grid)
    assert mc.states[0] == STATE_COMPLETE

    twists = mc.step([(0.0, 0.0)])
    assert twists[0].linear.x == 0.0
    assert twists[0].linear.y == 0.0
    assert mc.all_complete()

def test_motion_controller_valid_move():
    mc = MultiAgentMotionController(num_drones=1, max_speed=2.0)
    grid = np.zeros((25, 25), dtype=np.int8)

    # +X from (12, 12) -> (13, 12)
    mc.dispatch_actions([ActionMapper.ACTION_PLUS_X], [(12, 12)], grid)
    assert mc.states[0] == STATE_MOVING

    # Current pos at (0, 0)
    twists = mc.step([(0.0, 0.0)])
    assert twists[0].linear.x > 0.0
    assert twists[0].linear.y == 0.0

    # Fake arrival
    twists = mc.step([(0.95, 0.0)])
    assert mc.states[0] == STATE_ARRIVED
    assert twists[0].linear.x == 0.0

    # Step again to complete
    twists = mc.step([(0.95, 0.0)])
    assert mc.states[0] == STATE_COMPLETE
    assert mc.all_complete()

def test_motion_controller_invalid_move():
    mc = MultiAgentMotionController(num_drones=1)
    grid = np.zeros((25, 25), dtype=np.int8)
    grid[13, 12] = -1 # Obstacle

    # +X into obstacle
    mc.dispatch_actions([ActionMapper.ACTION_PLUS_X], [(12, 12)], grid)
    assert mc.states[0] == STATE_INVALID_TARGET

    twists = mc.step([(0.0, 0.0)])
    assert twists[0].linear.x == 0.0
    assert mc.states[0] == STATE_COMPLETE

def test_motion_controller_timeout():
    mc = MultiAgentMotionController(num_drones=1, movement_timeout=0.1)
    grid = np.zeros((25, 25), dtype=np.int8)

    mc.dispatch_actions([ActionMapper.ACTION_MINUS_X], [(12, 12)], grid)
    assert mc.states[0] == STATE_MOVING

    time.sleep(0.15)
    twists = mc.step([(0.0, 0.0)]) # Have not moved, but timeout exceeded
    assert mc.states[0] == STATE_ARRIVED

    twists = mc.step([(0.0, 0.0)])
    assert mc.states[0] == STATE_COMPLETE

def test_motion_controller_independent_drones():
    mc = MultiAgentMotionController(num_drones=2)
    grid = np.zeros((25, 25), dtype=np.int8)

    # Drone 0 +X, Drone 1 Hover
    mc.dispatch_actions([ActionMapper.ACTION_PLUS_X, ActionMapper.ACTION_HOVER], [(12, 12), (12, 14)], grid)

    assert mc.states[0] == STATE_MOVING
    assert mc.states[1] == STATE_COMPLETE
    assert not mc.all_complete()

    # Drone 0 arrives
    twists = mc.step([(0.95, 0.0), (0.0, 2.0)])
    assert mc.states[0] == STATE_ARRIVED
    assert mc.states[1] == STATE_COMPLETE

    mc.step([(0.95, 0.0), (0.0, 2.0)])
    assert mc.all_complete()
