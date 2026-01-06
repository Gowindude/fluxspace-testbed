"""
LIDAR 2D SENSOR - Simulates a Laser Range Finder That Detects Obstacles

WHAT THIS FILE DOES:
This file simulates a lidar (Light Detection and Ranging) sensor - like the sensors on self-driving
cars. The sensor shoots out invisible "rays" (like laser beams) in different directions and measures
how far they travel before hitting something.

HOW IT WORKS:
1. Shoots out multiple rays in different directions (like a fan of laser beams)
2. Each ray travels until it hits an obstacle or boundary
3. Measures the distance each ray traveled
4. Returns an array of distances: [dist1, dist2, dist3, ...]
   - Each distance tells you how far the nearest obstacle is in that direction
   - If no obstacle, returns max_range (sensor's maximum detection distance)

SENSOR PARAMETERS:
- n_rays: How many rays to shoot (more = more detailed but slower)
- fov_deg: Field of view (how wide the sensor sees, like 270° = most of a circle)
- max_range: Maximum distance it can detect (like how far you can see)

FOR BEGINNERS:
- Lidar = a sensor that measures distances to objects
- Like a radar but using light/lasers instead of radio waves
- Returns distances in each direction (like a radar screen showing objects)
- Used by controllers to avoid obstacles
- The "scan" method is called every simulation step to update what the drone "sees"
"""

import numpy as np
from ..world import World2D
from ..drone.state import DroneState
from ..config import DEFAULT_LIDAR_N_RAYS, DEFAULT_LIDAR_FOV_DEG, DEFAULT_LIDAR_MAX_RANGE


class Lidar2D:
    """2D Lidar sensor that casts rays and returns distances."""
    
    def __init__(
        self,
        n_rays: int = DEFAULT_LIDAR_N_RAYS,
        fov_deg: float = DEFAULT_LIDAR_FOV_DEG,
        max_range: float = DEFAULT_LIDAR_MAX_RANGE,
    ):
        """
        Args:
            n_rays: Number of rays to cast
            fov_deg: Field of view in degrees (typically 270 or 360)
            max_range: Maximum range of the sensor (m)
        """
        self.n_rays = n_rays
        self.fov_deg = fov_deg
        self.fov_rad = np.deg2rad(fov_deg)
        self.max_range = max_range
        
        # Compute ray angles relative to forward direction
        # Angles are centered around 0 (forward), spanning [-fov/2, fov/2]
        if n_rays == 1:
            self.ray_angles = np.array([0.0])
        else:
            self.ray_angles = np.linspace(-self.fov_rad / 2, self.fov_rad / 2, n_rays)
    
    def scan(self, state: DroneState, world: World2D) -> np.ndarray:
        """
        Perform a lidar scan from the current drone state.
        
        Args:
            state: Current drone state
            world: World to scan
            
        Returns:
            Array of distances (length n_rays). Values are in [0, max_range].
            max_range indicates no obstacle detected within range.
        """
        distances = np.full(self.n_rays, self.max_range)
        
        for i, ray_angle_rel in enumerate(self.ray_angles):
            # Compute absolute ray angle (relative to world frame)
            ray_angle = state.yaw + ray_angle_rel
            
            # Ray direction vector (normalized)
            ray_dir = (np.cos(ray_angle), np.sin(ray_angle))
            ray_start = (state.x, state.y)
            
            # Find intersection with world bounds
            dist_bounds = world.ray_bounds_intersection(ray_start, ray_dir)
            
            min_dist = self.max_range
            if dist_bounds is not None and dist_bounds < min_dist:
                min_dist = dist_bounds
            
            # Find intersections with obstacles
            for obs in world.obstacles:
                dist_obs = obs.ray_intersection(ray_start, ray_dir)
                if dist_obs is not None and dist_obs < min_dist:
                    min_dist = dist_obs
            
            distances[i] = min_dist
        
        return distances
    
    def get_ray_angles(self, state: DroneState) -> np.ndarray:
        """
        Get the absolute angles of all rays in the world frame.
        
        Args:
            state: Current drone state
            
        Returns:
            Array of ray angles (rad)
        """
        return state.yaw + self.ray_angles

