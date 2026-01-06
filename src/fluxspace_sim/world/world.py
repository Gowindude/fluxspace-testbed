"""
WORLD MODEL - The Complete 2D Environment Where the Drone Operates

WHAT THIS FILE DOES:
This file creates the complete 2D world that contains:
- Boundaries: The edges of the world (like walls that define the playing field)
- Obstacles: Objects in the world that the drone should avoid
- Collision detection: Checks if the drone hits anything

MAIN FEATURES:
- point_in_obstacle(): Checks if a point is inside any obstacle
- point_in_bounds(): Checks if a point is within the world boundaries
- collision(): Checks if the drone (modeled as a circle) collides with obstacles or boundaries
- ray_bounds_intersection(): Used by lidar to detect world boundaries

FOR BEGINNERS:
- World = the entire 2D space where simulation happens (like a map)
- Boundaries = the edges (left, right, top, bottom) - if drone goes past these, it crashes
- Obstacles = objects to avoid (walls, pillars, etc.)
- Collision detection = checking if drone hit something (like collision in a video game)
"""

from typing import List
from .obstacles import Obstacle
from ..config import DEFAULT_DRONE_RADIUS


class World2D:
    """2D world with boundaries and obstacles."""
    
    def __init__(
        self,
        xmin: float,
        ymin: float,
        xmax: float,
        ymax: float,
        obstacles: List[Obstacle] | None = None,
    ):
        """
        Args:
            xmin, ymin, xmax, ymax: World boundaries
            obstacles: List of obstacles in the world
        """
        if xmin >= xmax or ymin >= ymax:
            raise ValueError("Invalid world bounds")
        self.xmin = xmin
        self.ymin = ymin
        self.xmax = xmax
        self.ymax = ymax
        self.obstacles = obstacles if obstacles is not None else []
    
    def point_in_obstacle(self, x: float, y: float) -> bool:
        """Check if a point is inside any obstacle."""
        return any(obs.point_inside(x, y) for obs in self.obstacles)
    
    def point_in_bounds(self, x: float, y: float) -> bool:
        """Check if a point is within world boundaries."""
        return self.xmin <= x <= self.xmax and self.ymin <= y <= self.ymax
    
    def ray_bounds_intersection(
        self, ray_start: tuple[float, float], ray_dir: tuple[float, float]
    ) -> float | None:
        """
        Compute intersection of a ray with world boundaries.
        
        Args:
            ray_start: (x, y) start point of ray
            ray_dir: (dx, dy) direction vector (normalized)
            
        Returns:
            Distance along ray to intersection, or None if no intersection.
        """
        from .geometry import ray_bounds_intersection as ray_bounds
        return ray_bounds(ray_start, ray_dir, self.xmin, self.ymin, self.xmax, self.ymax)
    
    def collision(self, x: float, y: float, drone_radius: float = DEFAULT_DRONE_RADIUS) -> bool:
        """
        Check if drone at position (x, y) collides with obstacles or boundaries.
        
        Args:
            x, y: Drone position
            drone_radius: Radius of the drone (modeled as circle)
            
        Returns:
            True if collision detected
        """
        # Check if center is outside bounds (accounting for drone radius)
        if (x - drone_radius < self.xmin or x + drone_radius > self.xmax or
            y - drone_radius < self.ymin or y + drone_radius > self.ymax):
            return True
        
        # Check collision with obstacles (expand obstacle by drone_radius)
        from .obstacles import CircleObstacle, RectObstacle
        for obs in self.obstacles:
            if isinstance(obs, CircleObstacle):
                # Circle-circle collision: distance < sum of radii
                dx = x - obs.cx
                dy = y - obs.cy
                dist_sq = dx * dx + dy * dy
                if dist_sq <= (obs.radius + drone_radius) ** 2:
                    return True
            elif isinstance(obs, RectObstacle):
                # Rectangle collision: expand rectangle by drone_radius
                # Check if point is within expanded rectangle
                expanded_xmin = obs.xmin - drone_radius
                expanded_xmax = obs.xmax + drone_radius
                expanded_ymin = obs.ymin - drone_radius
                expanded_ymax = obs.ymax + drone_radius
                if (expanded_xmin <= x <= expanded_xmax and
                    expanded_ymin <= y <= expanded_ymax):
                    return True
        
        return False
    
    def __repr__(self):
        return f"World2D(bounds=({self.xmin}, {self.ymin}) to ({self.xmax}, {self.ymax}), {len(self.obstacles)} obstacles)"

