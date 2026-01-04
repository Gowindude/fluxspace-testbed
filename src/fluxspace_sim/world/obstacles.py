"""Obstacle definitions for the 2D world."""

from abc import ABC, abstractmethod


class Obstacle(ABC):
    """Base class for obstacles in the 2D world."""
    
    @abstractmethod
    def point_inside(self, x: float, y: float) -> bool:
        """Check if a point is inside the obstacle."""
        pass
    
    @abstractmethod
    def ray_intersection(
        self, ray_start: tuple[float, float], ray_dir: tuple[float, float]
    ) -> float | None:
        """
        Compute distance along ray to intersection with obstacle.
        
        Returns:
            Distance to intersection, or None if no intersection.
        """
        pass


class CircleObstacle(Obstacle):
    """Circular obstacle."""
    
    def __init__(self, cx: float, cy: float, radius: float):
        """
        Args:
            cx, cy: Center coordinates
            radius: Radius of the circle
        """
        self.cx = cx
        self.cy = cy
        self.radius = radius
    
    def point_inside(self, x: float, y: float) -> bool:
        """Check if point is inside the circle."""
        dx = x - self.cx
        dy = y - self.cy
        return dx * dx + dy * dy <= self.radius * self.radius
    
    def ray_intersection(
        self, ray_start: tuple[float, float], ray_dir: tuple[float, float]
    ) -> float | None:
        """Compute ray-circle intersection."""
        from .geometry import ray_circle_intersection
        return ray_circle_intersection(
            ray_start, ray_dir, (self.cx, self.cy), self.radius
        )
    
    def __repr__(self):
        return f"CircleObstacle(cx={self.cx}, cy={self.cy}, r={self.radius})"


class RectObstacle(Obstacle):
    """Axis-aligned rectangular obstacle."""
    
    def __init__(self, xmin: float, ymin: float, xmax: float, ymax: float):
        """
        Args:
            xmin, ymin: Lower-left corner
            xmax, ymax: Upper-right corner
        """
        if xmin >= xmax or ymin >= ymax:
            raise ValueError("Invalid rectangle bounds")
        self.xmin = xmin
        self.ymin = ymin
        self.xmax = xmax
        self.ymax = ymax
    
    def point_inside(self, x: float, y: float) -> bool:
        """Check if point is inside the rectangle."""
        return self.xmin <= x <= self.xmax and self.ymin <= y <= self.ymax
    
    def ray_intersection(
        self, ray_start: tuple[float, float], ray_dir: tuple[float, float]
    ) -> float | None:
        """Compute ray-rectangle intersection."""
        from .geometry import ray_rect_intersection
        return ray_rect_intersection(
            ray_start, ray_dir, self.xmin, self.ymin, self.xmax, self.ymax
        )
    
    def __repr__(self):
        return f"RectObstacle(xmin={self.xmin}, ymin={self.ymin}, xmax={self.xmax}, ymax={self.ymax})"

