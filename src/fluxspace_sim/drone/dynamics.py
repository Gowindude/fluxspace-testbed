"""Drone dynamics model (unicycle/kinematic model)."""

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

