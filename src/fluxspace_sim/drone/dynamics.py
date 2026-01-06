"""
DRONE DYNAMICS - How the Drone Moves (Physics/Movement Rules)

WHAT THIS FILE DOES:
This file contains the physics model that determines how the drone moves. Given a command
(like "go forward at 1 m/s" and "turn at 0.5 rad/s"), it calculates where the drone will be
after a small amount of time passes.

THE MOVEMENT MODEL (Simple Unicycle):
- If facing direction θ and moving at speed v, position changes by:
  - x changes by: v × cos(θ) × time_step
  - y changes by: v × sin(θ) × time_step
- Heading (yaw) changes by: yaw_rate × time_step
- This is like a car that can turn and move forward (can't move sideways)

FOR BEGINNERS:
- Dynamics = the rules of how something moves
- Takes commands (speed, turn rate) and calculates new position
- Runs every simulation step (like every frame in a video game)
- Simple model = no complex physics like air resistance or momentum
- Think of it like moving a character in a top-down game: position + direction + speed = new position
"""

import numpy as np
from .state import DroneState


class DroneDynamics:
    """Simple unicycle dynamics model for the drone."""
    
    def __init__(self, dt: float = 0.05):
        """
        Args:
            dt: Time step for integration
        """
        self.dt = dt
    
    def step(
        self,
        state: DroneState,
        v_cmd: float,
        yaw_rate_cmd: float,
    ) -> DroneState:
        """
        Propagate state forward one time step.
        
        Uses simple kinematic model:
            x += v * cos(yaw) * dt
            y += v * sin(yaw) * dt
            yaw += yaw_rate * dt
            v = v_cmd (instantaneous, or could use first-order lag)
        
        Args:
            state: Current state
            v_cmd: Commanded forward velocity (m/s)
            yaw_rate_cmd: Commanded yaw rate (rad/s)
            
        Returns:
            New state after one time step
        """
        # Update velocity (instantaneous for simplicity)
        v = v_cmd
        
        # Update heading
        yaw = state.yaw + yaw_rate_cmd * self.dt
        
        # Normalize yaw to [-pi, pi]
        yaw = ((yaw + np.pi) % (2 * np.pi)) - np.pi
        
        # Update position
        x = state.x + v * np.cos(state.yaw) * self.dt
        y = state.y + v * np.sin(state.yaw) * self.dt
        
        return DroneState(x=x, y=y, yaw=yaw, v=v)

