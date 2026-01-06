"""
DRONE CONTROLLER - The "Brain" That Decides What the Drone Should Do

WHAT THIS FILE DOES:
This file contains the controller classes that decide what commands to give to the drone.
The controller is like the "brain" - it looks at where the drone is, where it wants to go,
and figures out what actions to take.

CONTROLLER TYPES:
1. Controller (base class) - Defines the interface that all controllers must follow
2. WaypointController - Follows a list of waypoints (destination points)

HOW WAYPOINT CONTROLLER WORKS:
- Given a list of points to visit: [(x1, y1), (x2, y2), ...]
- Calculates direction to current target waypoint
- Figures out how much to turn to face that direction
- Commands the drone: "Go this speed, turn this much"
- When close enough to a waypoint, moves to next one

FOR BEGINNERS:
- Controller = the logic that controls the drone (like autopilot)
- Waypoint = a destination point (like a pin on a map)
- The controller calculates: "Which way should I turn? How fast should I go?"
- Returns commands: (velocity, yaw_rate) which tell the drone what to do
- Like a GPS that not only shows the route but also tells you to turn left/right
"""

from abc import ABC, abstractmethod
import numpy as np
from .state import DroneState
from ..config import (
    DEFAULT_CONTROLLER_K_YAW,
    DEFAULT_CONTROLLER_V_CMD,
    DEFAULT_CONTROLLER_WAYPOINT_TOLERANCE,
)


class Controller(ABC):
    """Base class for drone controllers."""
    
    @abstractmethod
    def compute_control(self, state: DroneState, sensor_data: dict | None = None) -> tuple[float, float]:
        """
        Compute control commands.
        
        Args:
            state: Current drone state
            sensor_data: Optional sensor data (e.g., lidar scan)
            
        Returns:
            (v_cmd, yaw_rate_cmd) tuple
        """
        pass


class WaypointController(Controller):
    """Simple waypoint-following controller."""
    
    def __init__(
        self,
        waypoints: list[tuple[float, float]],
        k_yaw: float = DEFAULT_CONTROLLER_K_YAW,
        v_cmd: float = DEFAULT_CONTROLLER_V_CMD,
        waypoint_tolerance: float = DEFAULT_CONTROLLER_WAYPOINT_TOLERANCE,
    ):
        """
        Args:
            waypoints: List of (x, y) waypoints to follow
            k_yaw: Proportional gain for yaw rate control
            v_cmd: Commanded forward velocity (m/s)
            waypoint_tolerance: Distance threshold to consider waypoint reached (m)
        """
        self.waypoints = waypoints
        self.current_waypoint_idx = 0
        self.k_yaw = k_yaw
        self.v_cmd = v_cmd
        self.waypoint_tolerance = waypoint_tolerance
    
    def compute_control(self, state: DroneState, sensor_data: dict | None = None) -> tuple[float, float]:
        """
        Compute control to follow waypoints.
        
        Returns:
            (v_cmd, yaw_rate_cmd)
        """
        if self.current_waypoint_idx >= len(self.waypoints):
            # All waypoints reached, stop
            return 0.0, 0.0
        
        target_x, target_y = self.waypoints[self.current_waypoint_idx]
        
        # Compute distance to current waypoint
        dx = target_x - state.x
        dy = target_y - state.y
        dist = np.sqrt(dx * dx + dy * dy)
        
        # Check if waypoint reached
        if dist < self.waypoint_tolerance:
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.waypoints):
                return 0.0, 0.0
            target_x, target_y = self.waypoints[self.current_waypoint_idx]
            dx = target_x - state.x
            dy = target_y - state.y
        
        # Compute desired heading
        desired_yaw = np.arctan2(dy, dx)
        
        # Compute heading error (wrap to [-pi, pi])
        yaw_error = desired_yaw - state.yaw
        yaw_error = ((yaw_error + np.pi) % (2 * np.pi)) - np.pi
        
        # Compute yaw rate command (proportional control with saturation)
        yaw_rate_cmd = self.k_yaw * yaw_error
        max_yaw_rate = 2.0  # rad/s
        yaw_rate_cmd = np.clip(yaw_rate_cmd, -max_yaw_rate, max_yaw_rate)
        
        # Reduce velocity when close to waypoint or when turning sharply
        v = self.v_cmd
        if dist < 2.0:
            v = self.v_cmd * (dist / 2.0)  # Slow down when approaching
        if abs(yaw_error) > np.pi / 4:
            v = self.v_cmd * 0.5  # Slow down when turning sharply
        
        return v, yaw_rate_cmd
    
    def reset(self):
        """Reset controller to first waypoint."""
        self.current_waypoint_idx = 0

