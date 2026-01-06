"""
WAYPOINT-FOLLOWING DEMO - Tests the Controller by Following a Path

WHAT THIS SCRIPT DOES:
This script demonstrates the waypoint-following controller. It gives the drone a list of
destination points and lets the controller figure out how to navigate to each one. The
controller automatically calculates turns and speed adjustments.

THE DEMO:
1. Creates a world with boundaries and obstacles
2. Places a drone at starting position (1, 1)
3. Defines waypoints to visit: [(8,1), (8,5), (2,5), (2,1)] - makes a rectangle path
4. Uses WaypointController to automatically navigate
5. Controller calculates: direction to waypoint, how much to turn, how fast to go
6. Generates outputs: plot, animation, and data log

FOR BEGINNERS:
- This shows how controllers work (the "brain" of the drone)
- Waypoint = a destination point (like a pin on a map)
- The controller automatically figures out how to reach each waypoint
- More advanced than open-loop - responds to where the drone is
- Run this to see autonomous navigation in action!

TO RUN:
  python scripts/run_demo_waypoints.py
"""

import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from fluxspace_sim.world import World2D, CircleObstacle, RectObstacle
from fluxspace_sim.drone.state import DroneState
from fluxspace_sim.drone.controller import WaypointController
from fluxspace_sim.sensors.lidar2d import Lidar2D
from fluxspace_sim.sim.simulator import Simulator
from fluxspace_sim.viz.plot import plot_trajectory
from fluxspace_sim.viz.animate import animate_simulation


def main():
    """Run waypoint-following demo."""
    print("Setting up world...")
    
    # Create world: bounds (0..10, 0..6)
    world = World2D(
        xmin=0.0,
        ymin=0.0,
        xmax=10.0,
        ymax=6.0,
        obstacles=[
            CircleObstacle(cx=5.0, cy=3.0, radius=1.0),
            RectObstacle(xmin=7.0, ymin=1.0, xmax=8.5, ymax=2.5),
            CircleObstacle(cx=2.0, cy=4.5, radius=0.8),
        ]
    )
    
    # Initialize drone at (1, 1, yaw=0)
    initial_state = DroneState(x=1.0, y=1.0, yaw=0.0, v=0.0)
    
    # Create sensor
    sensor = Lidar2D(n_rays=60, fov_deg=270, max_range=10.0)
    
    # Define waypoints
    waypoints = [(8.0, 1.0), (8.0, 5.0), (2.0, 5.0), (2.0, 1.0)]
    
    # Create waypoint controller
    controller = WaypointController(
        waypoints=waypoints,
        k_yaw=2.0,
        v_cmd=1.0,
        waypoint_tolerance=0.3,
    )
    
    # Create simulator
    sim = Simulator(
        world=world,
        initial_state=initial_state,
        sensor=sensor,
        controller=controller,
        dt=0.05,
    )
    
    print("Running simulation with waypoint controller...")
    print(f"Waypoints: {waypoints}")
    
    # Run simulation for sufficient duration
    duration = 30.0  # Should be enough to reach all waypoints
    logger = sim.run(duration=duration)
    
    print(f"Simulation complete. Logged {len(logger.time)} steps.")
    if logger.collision.any() if hasattr(logger.collision, 'any') else any(logger.collision):
        print("Warning: Collision detected!")
    
    # Create outputs directory
    outputs_dir = Path(__file__).parent.parent / "outputs"
    outputs_dir.mkdir(exist_ok=True)
    
    # Save plot
    print("Generating trajectory plot...")
    plot_trajectory(world, logger, save_path=str(outputs_dir / "waypoints_traj.png"))
    print(f"Saved: {outputs_dir / 'waypoints_traj.png'}")
    
    # Save animation
    print("Generating animation...")
    animate_simulation(world, logger, save_path=str(outputs_dir / "waypoints.gif"))
    print(f"Saved: {outputs_dir / 'waypoints.gif'}")
    
    # Save log
    print("Saving log data...")
    logger.save_npz(str(outputs_dir / "waypoints_log.npz"))
    print(f"Saved: {outputs_dir / 'waypoints_log.npz'}")
    
    print("Demo complete!")


if __name__ == "__main__":
    main()

