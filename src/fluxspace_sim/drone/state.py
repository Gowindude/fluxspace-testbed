"""
DRONE STATE - What the Drone Knows About Itself

WHAT THIS FILE DOES:
This file defines a simple data structure that holds all the information about the drone's current
situation. Think of it like a "status card" that follows the drone around.

THE DRONE STATE CONTAINS:
- x, y: Position in 2D space (where the drone is, in meters)
- yaw: Which direction the drone is facing (angle in radians, like a compass)
- v: Velocity/speed (how fast the drone is moving forward, in meters per second)

FOR BEGINNERS:
- State = all the information about where the drone is and how it's moving
- x, y are like coordinates on a map
- yaw is like which way you're facing (north, east, south, west, or any angle in between)
- v is how fast you're going (like speedometer in a car)
- Every step of the simulation, the state gets updated
"""

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

