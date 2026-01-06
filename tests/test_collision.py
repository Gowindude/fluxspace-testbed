"""
TESTS FOR COLLISION DETECTION - Automated Checks That Collisions Work Correctly

WHAT THIS FILE DOES:
This file contains automated tests that verify collision detection works correctly.
It checks that the simulator correctly detects when the drone hits obstacles or boundaries.

WHAT GETS TESTED:
- Circle obstacles: Does the drone detect collisions with round obstacles?
- Rectangle obstacles: Does the drone detect collisions with rectangular obstacles?
- Boundaries: Does the drone detect when it goes outside the world boundaries?

FOR BEGINNERS:
- Tests verify that collision detection (crash detection) works correctly
- Run with: pytest tests/test_collision.py
- Important for safety - ensures the drone knows when it crashes!
"""

import pytest
from fluxspace_sim.world import World2D, CircleObstacle, RectObstacle


def test_collision_circle_obstacle():
    """Test collision detection with circle obstacle."""
    world = World2D(
        xmin=0.0,
        ymin=0.0,
        xmax=10.0,
        ymax=10.0,
        obstacles=[CircleObstacle(cx=5.0, cy=5.0, radius=1.0)]
    )
    
    drone_radius = 0.25
    
    # At center of obstacle - should collide
    assert world.collision(5.0, 5.0, drone_radius) == True
    
    # At edge of obstacle (radius + drone_radius = 1.25 from center) - should collide
    assert world.collision(5.0 + 1.25, 5.0, drone_radius) == True
    
    # Just outside collision zone - should not collide
    assert world.collision(5.0 + 1.26, 5.0, drone_radius) == False
    
    # Far from obstacle - should not collide
    assert world.collision(7.0, 5.0, drone_radius) == False


def test_collision_rect_obstacle():
    """Test collision detection with rectangular obstacle."""
    world = World2D(
        xmin=0.0,
        ymin=0.0,
        xmax=10.0,
        ymax=10.0,
        obstacles=[RectObstacle(xmin=3.0, ymin=3.0, xmax=5.0, ymax=5.0)]
    )
    
    drone_radius = 0.25
    
    # Inside obstacle - should collide
    assert world.collision(4.0, 4.0, drone_radius) == True
    
    # At edge (accounting for drone radius) - should collide
    assert world.collision(3.0 - 0.24, 4.0, drone_radius) == True
    
    # Just outside - should not collide
    assert world.collision(3.0 - 0.26, 4.0, drone_radius) == False


def test_collision_boundaries():
    """Test collision detection with world boundaries."""
    world = World2D(
        xmin=0.0,
        ymin=0.0,
        xmax=10.0,
        ymax=10.0,
        obstacles=[]
    )
    
    drone_radius = 0.25
    
    # Inside bounds - should not collide
    assert world.collision(5.0, 5.0, drone_radius) == False
    
    # Outside bounds - should collide
    assert world.collision(-0.1, 5.0, drone_radius) == True
    assert world.collision(10.1, 5.0, drone_radius) == True
    assert world.collision(5.0, -0.1, drone_radius) == True
    assert world.collision(5.0, 10.1, drone_radius) == True
    
    # At boundary (accounting for radius) - should collide
    assert world.collision(0.24, 5.0, drone_radius) == True
    assert world.collision(9.76, 5.0, drone_radius) == True

