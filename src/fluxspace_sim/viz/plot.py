"""Static plotting functions."""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
from ..world import World2D
from ..sim.logger import SimLogger


def plot_trajectory(
    world: World2D,
    logger: SimLogger,
    n_lidar_rays: int = 5,
    figsize: tuple[float, float] = (10, 8),
    save_path: str | None = None,
):
    """
    Plot world, obstacles, trajectory, and sample lidar rays.
    
    Args:
        world: World model
        logger: Simulation logger
        n_lidar_rays: Number of lidar rays to plot (evenly spaced in time)
        figsize: Figure size
        save_path: Optional path to save figure
    """
    fig, ax = plt.subplots(figsize=figsize)
    
    # Plot world boundaries
    ax.plot([world.xmin, world.xmax, world.xmax, world.xmin, world.xmin],
            [world.ymin, world.ymin, world.ymax, world.ymax, world.ymin],
            'k-', linewidth=2, label='Boundaries')
    
    # Plot obstacles
    from ..world.obstacles import CircleObstacle, RectObstacle
    for obs in world.obstacles:
        if isinstance(obs, CircleObstacle):
            circle = Circle((obs.cx, obs.cy), obs.radius, color='red', alpha=0.5, label='Obstacle' if obs == world.obstacles[0] else '')
            ax.add_patch(circle)
        elif isinstance(obs, RectObstacle):
            width = obs.xmax - obs.xmin
            height = obs.ymax - obs.ymin
            rect = Rectangle((obs.xmin, obs.ymin), width, height, color='red', alpha=0.5, label='Obstacle' if obs == world.obstacles[0] else '')
            ax.add_patch(rect)
    
    # Plot trajectory
    x_arr = np.array(logger.x)
    y_arr = np.array(logger.y)
    ax.plot(x_arr, y_arr, 'b-', linewidth=1.5, alpha=0.7, label='Trajectory')
    ax.plot(x_arr[0], y_arr[0], 'go', markersize=10, label='Start')
    ax.plot(x_arr[-1], y_arr[-1], 'ro', markersize=10, label='End')
    
    # Plot sample lidar rays
    if logger.lidar_scans and any(scan is not None for scan in logger.lidar_scans):
        n_steps = len(logger.lidar_scans)
        indices = np.linspace(0, n_steps - 1, n_lidar_rays, dtype=int)
        
        for idx in indices:
            if logger.lidar_scans[idx] is not None:
                scan = logger.lidar_scans[idx]
                x_pos = logger.x[idx]
                y_pos = logger.y[idx]
                yaw = logger.yaw[idx]
                
                # Get ray angles (approximate - we need to know sensor parameters)
                # For simplicity, assume 270 deg FOV, 60 rays
                n_rays = len(scan)
                fov_rad = np.deg2rad(270)
                ray_angles_rel = np.linspace(-fov_rad / 2, fov_rad / 2, n_rays)
                
                # Plot a subset of rays
                ray_indices = np.linspace(0, n_rays - 1, 20, dtype=int)
                for ray_idx in ray_indices:
                    ray_angle = yaw + ray_angles_rel[ray_idx]
                    dist = scan[ray_idx]
                    if dist < 10.0:  # Only plot if within max range
                        end_x = x_pos + dist * np.cos(ray_angle)
                        end_y = y_pos + dist * np.sin(ray_angle)
                        ax.plot([x_pos, end_x], [y_pos, end_y], 'c-', alpha=0.3, linewidth=0.5)
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Trajectory Plot')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    
    return fig, ax

