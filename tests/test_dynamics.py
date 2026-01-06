"""
TESTS FOR DRONE DYNAMICS - Automated Checks That Movement Works Correctly

WHAT THIS FILE DOES:
This file contains automated tests that verify the drone dynamics (movement physics)
work correctly. These tests run automatically to make sure nothing is broken.

WHAT GETS TESTED:
- Forward motion: Does the drone move forward correctly?
- Turning: Does the drone turn correctly?
- Angle wrapping: Do angles stay in the correct range?
- Combined motion: Does turning while moving work?

FOR BEGINNERS:
- Tests = automated checks that code works correctly
- Run with: pytest tests/test_dynamics.py
- If tests pass = everything works as expected
- If tests fail = something is broken and needs fixing
- These are like quality control checks
"""

import pytest
import numpy as np
from fluxspace_sim.drone.state import DroneState
from fluxspace_sim.drone.dynamics import DroneDynamics


def test_dynamics_forward_motion():
    """Test that drone moves forward when yaw=0 and v_cmd=1."""
    dynamics = DroneDynamics(dt=1.0)
    state = DroneState(x=0.0, y=0.0, yaw=0.0, v=0.0)
    
    # Command forward velocity
    new_state = dynamics.step(state, v_cmd=1.0, yaw_rate_cmd=0.0)
    
    # After 1 second at 1 m/s forward (yaw=0), x should increase by ~1, y unchanged
    assert abs(new_state.x - 1.0) < 1e-6
    assert abs(new_state.y - 0.0) < 1e-6
    assert new_state.yaw == 0.0
    assert new_state.v == 1.0


def test_dynamics_yaw_rate():
    """Test that yaw changes with yaw_rate_cmd."""
    dynamics = DroneDynamics(dt=1.0)
    state = DroneState(x=0.0, y=0.0, yaw=0.0, v=1.0)
    
    # Command yaw rate
    new_state = dynamics.step(state, v_cmd=1.0, yaw_rate_cmd=0.5)
    
    # After 1 second at 0.5 rad/s, yaw should increase by 0.5
    assert abs(new_state.yaw - 0.5) < 1e-6
    assert new_state.v == 1.0


def test_dynamics_yaw_wrapping():
    """Test that yaw wraps to [-pi, pi]."""
    dynamics = DroneDynamics(dt=1.0)
    state = DroneState(x=0.0, y=0.0, yaw=np.pi - 0.1, v=1.0)
    
    # Add enough yaw rate to cross pi boundary
    new_state = dynamics.step(state, v_cmd=1.0, yaw_rate_cmd=0.3)
    
    # Yaw should be wrapped
    assert -np.pi <= new_state.yaw <= np.pi


def test_dynamics_turning_motion():
    """Test motion when turning."""
    dynamics = DroneDynamics(dt=0.1)
    state = DroneState(x=0.0, y=0.0, yaw=0.0, v=1.0)
    
    # Turn 90 degrees while moving
    for _ in range(10):
        state = dynamics.step(state, v_cmd=1.0, yaw_rate_cmd=np.pi / 2)  # 90 deg/s
    
    # After 1 second, should have turned 90 degrees
    assert abs(state.yaw - np.pi / 2) < 0.1
    # Position should have changed (moving in a curve)
    assert abs(state.x) > 0.1 or abs(state.y) > 0.1

