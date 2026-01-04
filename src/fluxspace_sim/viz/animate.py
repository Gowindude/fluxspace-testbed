"""Animation functions."""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle, Rectangle
from ..world import World2D
from ..sim.logger import SimLogger
from ..config import DEFAULT_DRONE_RADIUS


def animate_simulation(
    world: World2D,
    logger: SimLogger,
    show_lidar: bool = True,
    figsize: tuple[float, float] = (10, 8),
    save_path: str | None = None,
    interval: int = 50,
):
    """
    Animate the simulation.
    
    Args:
        world: World model
        logger: Simulation logger
        show_lidar: Whether to show lidar rays
        figsize: Figure size
        save_path: Optional path to save animation (.gif)
        interval: Animation interval in ms
        
    Returns:
        Animation object
    """
    fig, ax = plt.subplots(figsize=figsize)
    
    # Set up axes
    ax.set_xlim(world.xmin - 1, world.xmax + 1)
    ax.set_ylim(world.ymin - 1, world.ymax + 1)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Simulation Animation')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    
    # Plot world boundaries
    ax.plot([world.xmin, world.xmax, world.xmax, world.xmin, world.xmin],
            [world.ymin, world.ymin, world.ymax, world.ymax, world.ymin],
            'k-', linewidth=2)
    
    # Plot obstacles (static)
    from ..world.obstacles import CircleObstacle, RectObstacle
    for obs in world.obstacles:
        if isinstance(obs, CircleObstacle):
            circle = Circle((obs.cx, obs.cy), obs.radius, color='red', alpha=0.5)
            ax.add_patch(circle)
        elif isinstance(obs, RectObstacle):
            width = obs.xmax - obs.xmin
            height = obs.ymax - obs.ymin
            rect = Rectangle((obs.xmin, obs.ymin), width, height, color='red', alpha=0.5)
            ax.add_patch(rect)
    
    # Plot full trajectory (faded)
    x_arr = np.array(logger.x)
    y_arr = np.array(logger.y)
    ax.plot(x_arr, y_arr, 'b-', linewidth=1, alpha=0.3)
    
    # Initialize animated elements
    drone_circle = Circle((0, 0), DEFAULT_DRONE_RADIUS, color='blue', alpha=0.7)
    ax.add_patch(drone_circle)
    
    # Drone direction indicator
    direction_line, = ax.plot([], [], 'b-', linewidth=2)
    
    # Lidar rays
    lidar_lines = []
    if show_lidar and logger.lidar_scans and len(logger.lidar_scans) > 0 and logger.lidar_scans[0] is not None:
        n_rays = len(logger.lidar_scans[0])
        # Show subset of rays for performance
        n_show = min(20, n_rays)
        ray_indices = np.linspace(0, n_rays - 1, n_show, dtype=int)
        for _ in ray_indices:
            line, = ax.plot([], [], 'c-', alpha=0.4, linewidth=0.5)
            lidar_lines.append(line)
    
    time_text = ax.text(0.02, 0.98, '', transform=ax.transAxes, verticalalignment='top',
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    def animate(frame):
        if frame >= len(logger.x):
            return drone_circle, direction_line, time_text, *lidar_lines
        
        x = logger.x[frame]
        y = logger.y[frame]
        yaw = logger.yaw[frame]
        
        # Update drone position
        drone_circle.center = (x, y)
        
        # Update direction indicator
        dir_len = DEFAULT_DRONE_RADIUS * 2
        dir_x = x + dir_len * np.cos(yaw)
        dir_y = y + dir_len * np.sin(yaw)
        direction_line.set_data([x, dir_x], [y, dir_y])
        
        # Update lidar rays
        if show_lidar and frame < len(logger.lidar_scans) and logger.lidar_scans[frame] is not None:
            scan = logger.lidar_scans[frame]
            n_rays = len(scan)
            fov_rad = np.deg2rad(270)  # Default FOV
            ray_angles_rel = np.linspace(-fov_rad / 2, fov_rad / 2, n_rays)
            
            n_show = len(lidar_lines)
            ray_indices = np.linspace(0, n_rays - 1, n_show, dtype=int)
            
            for i, ray_idx in enumerate(ray_indices):
                if i < len(lidar_lines):
                    ray_angle = yaw + ray_angles_rel[ray_idx]
                    dist = scan[ray_idx]
                    if dist < 10.0:
                        end_x = x + dist * np.cos(ray_angle)
                        end_y = y + dist * np.sin(ray_angle)
                        lidar_lines[i].set_data([x, end_x], [y, end_y])
                    else:
                        lidar_lines[i].set_data([], [])
        
        # Update time text
        time_text.set_text(f'Time: {logger.time[frame]:.2f} s')
        
        return drone_circle, direction_line, time_text, *lidar_lines
    
    anim = animation.FuncAnimation(
        fig, animate, frames=len(logger.x), interval=interval, blit=True, repeat=True
    )
    
    if save_path:
        anim.save(save_path, writer='pillow', fps=20)
    
    return anim

