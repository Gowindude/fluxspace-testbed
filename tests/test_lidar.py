"""Tests for lidar sensor."""

import pytest
import numpy as np
from fluxspace_sim.world import World2D, CircleObstacle
from fluxspace_sim.drone.state import DroneState
from fluxspace_sim.sensors.lidar2d import Lidar2D


def test_lidar_bounds_only():
    """Test lidar scan with only world bounds, no obstacles."""
    world = World2D(xmin=0.0, ymin=0.0, xmax=10.0, ymax=10.0, obstacles=[])
    sensor = Lidar2D(n_rays=60, fov_deg=270, max_range=10.0)
    state = DroneState(x=5.0, y=5.0, yaw=0.0, v=0.0)
    
    scan = sensor.scan(state, world)
    
    assert len(scan) == 60
    assert all(0 <= d <= 10.0 for d in scan)
    
    # Forward ray (yaw=0) should hit right boundary at x=10
    # Ray angle 0 is at index 30 (middle of 60 rays, but FOV is 270 deg)
    # Actually, with 270 deg FOV and 60 rays, angles go from -135 to +135 deg
    # Forward (0 deg) should be at index 30 (middle)
    forward_idx = 30  # Approximate - exact depends on implementation
    forward_dist = scan[forward_idx]
    
    # Distance to right boundary (x=10) from x=5, y=5, heading 0 deg
    expected_dist = 5.0
    assert abs(forward_dist - expected_dist) < 0.5  # Allow some tolerance


def test_lidar_with_obstacle():
    """Test lidar scan with obstacle in front."""
    world = World2D(
        xmin=0.0,
        ymin=0.0,
        xmax=10.0,
        ymax=10.0,
        obstacles=[CircleObstacle(cx=7.0, cy=5.0, radius=1.0)]
    )
    sensor = Lidar2D(n_rays=60, fov_deg=270, max_range=10.0)
    state = DroneState(x=5.0, y=5.0, yaw=0.0, v=0.0)
    
    scan = sensor.scan(state, world)
    
    assert len(scan) == 60
    assert all(0 <= d <= 10.0 for d in scan)
    
    # Forward ray should hit obstacle before boundary
    # Distance from (5,5) to circle at (7,5) with radius 1 is 2 - 1 = 1
    forward_idx = 30
    forward_dist = scan[forward_idx]
    
    # Should be less than distance to boundary (5.0)
    assert forward_dist < 5.0
    # Should be approximately 1.0 (distance to obstacle edge)
    assert abs(forward_dist - 1.0) < 0.3


def test_lidar_ray_angles():
    """Test that ray angles are computed correctly."""
    sensor = Lidar2D(n_rays=60, fov_deg=270, max_range=10.0)
    state = DroneState(x=0.0, y=0.0, yaw=0.0, v=0.0)
    
    angles = sensor.get_ray_angles(state)
    
    assert len(angles) == 60
    # With yaw=0, angles should span [-fov/2, fov/2] = [-135 deg, +135 deg]
    assert angles[0] < 0  # First ray should be negative
    assert angles[-1] > 0  # Last ray should be positive
    # Middle ray should be approximately 0
    assert abs(angles[30]) < 0.1


def test_lidar_max_range():
    """Test that lidar returns max_range when no obstacle detected."""
    world = World2D(xmin=0.0, ymin=0.0, xmax=100.0, ymax=100.0, obstacles=[])
    sensor = Lidar2D(n_rays=10, fov_deg=270, max_range=10.0)
    state = DroneState(x=50.0, y=50.0, yaw=0.0, v=0.0)
    
    scan = sensor.scan(state, world)
    
    # All rays should be max_range since boundaries are far away
    assert all(d == 10.0 for d in scan)

