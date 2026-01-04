"""Drone state representation."""

from dataclasses import dataclass


@dataclass
class DroneState:
    """State of the drone in 2D."""
    
    x: float  # x position (m)
    y: float  # y position (m)
    yaw: float  # heading angle (rad)
    v: float  # forward velocity (m/s)
    
    def copy(self) -> "DroneState":
        """Return a copy of this state."""
        return DroneState(x=self.x, y=self.y, yaw=self.yaw, v=self.v)
    
    def __repr__(self):
        return f"DroneState(x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f}, v={self.v:.2f})"

