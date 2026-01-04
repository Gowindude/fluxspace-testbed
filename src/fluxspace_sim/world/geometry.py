"""Geometry utility functions for ray casting and angle operations."""

import numpy as np


def wrap_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle


def ray_circle_intersection(
    ray_start: tuple[float, float],
    ray_dir: tuple[float, float],
    circle_center: tuple[float, float],
    circle_radius: float,
) -> float | None:
    """
    Compute intersection of a ray with a circle.
    
    Args:
        ray_start: (x, y) start point of ray
        ray_dir: (dx, dy) direction vector (normalized)
        circle_center: (cx, cy) center of circle
        circle_radius: radius of circle
        
    Returns:
        Distance along ray to intersection, or None if no intersection.
        Returns the closest positive intersection.
    """
    sx, sy = ray_start
    dx, dy = ray_dir
    cx, cy = circle_center
    r = circle_radius
    
    # Vector from circle center to ray start
    ocx = sx - cx
    ocy = sy - cy
    
    # Quadratic: a*t^2 + b*t + c = 0
    a = dx * dx + dy * dy
    b = 2 * (ocx * dx + ocy * dy)
    c = ocx * ocx + ocy * ocy - r * r
    
    discriminant = b * b - 4 * a * c
    
    if discriminant < 0:
        return None
    
    sqrt_disc = np.sqrt(discriminant)
    t1 = (-b - sqrt_disc) / (2 * a)
    t2 = (-b + sqrt_disc) / (2 * a)
    
    # Return smallest positive t
    valid_ts = [t for t in [t1, t2] if t > 1e-6]
    if not valid_ts:
        return None
    return min(valid_ts)


def ray_rect_intersection(
    ray_start: tuple[float, float],
    ray_dir: tuple[float, float],
    xmin: float,
    ymin: float,
    xmax: float,
    ymax: float,
) -> float | None:
    """
    Compute intersection of a ray with an axis-aligned rectangle.
    
    Args:
        ray_start: (x, y) start point of ray
        ray_dir: (dx, dy) direction vector (normalized)
        xmin, ymin, xmax, ymax: rectangle bounds
        
    Returns:
        Distance along ray to intersection, or None if no intersection.
    """
    sx, sy = ray_start
    dx, dy = ray_dir
    
    if abs(dx) < 1e-9:
        dx = 1e-9 if dx >= 0 else -1e-9
    if abs(dy) < 1e-9:
        dy = 1e-9 if dy >= 0 else -1e-9
    
    # Intersection with vertical lines (x = xmin, x = xmax)
    t_xmin = (xmin - sx) / dx if dx != 0 else float('inf')
    t_xmax = (xmax - sx) / dx if dx != 0 else float('inf')
    
    # Intersection with horizontal lines (y = ymin, y = ymax)
    t_ymin = (ymin - sy) / dy if dy != 0 else float('inf')
    t_ymax = (ymax - sy) / dy if dy != 0 else float('inf')
    
    # Check which intersections are valid (within rectangle bounds)
    valid_ts = []
    
    for t in [t_xmin, t_xmax]:
        if t > 1e-6:
            y_intersect = sy + t * dy
            if ymin <= y_intersect <= ymax:
                valid_ts.append(t)
    
    for t in [t_ymin, t_ymax]:
        if t > 1e-6:
            x_intersect = sx + t * dx
            if xmin <= x_intersect <= xmax:
                valid_ts.append(t)
    
    if not valid_ts:
        return None
    return min(valid_ts)


def ray_bounds_intersection(
    ray_start: tuple[float, float],
    ray_dir: tuple[float, float],
    xmin: float,
    ymin: float,
    xmax: float,
    ymax: float,
) -> float | None:
    """
    Compute intersection of a ray with world boundaries.
    Same as ray_rect_intersection but named for clarity.
    """
    return ray_rect_intersection(ray_start, ray_dir, xmin, ymin, xmax, ymax)

