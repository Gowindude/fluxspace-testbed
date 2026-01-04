"""Open-loop control demo script."""

import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from fluxspace_sim.world import World2D, CircleObstacle, RectObstacle
from fluxspace_sim.drone.state import DroneState
from fluxspace_sim.sensors.lidar2d import Lidar2D
from fluxspace_sim.sim.simulator import Simulator
from fluxspace_sim.viz.plot import plot_trajectory
from fluxspace_sim.viz.animate import animate_simulation


def main():
    """Run open-loop control demo."""
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
    
    # Create simulator (no controller - we'll use open-loop controls)
    sim = Simulator(
        world=world,
        initial_state=initial_state,
        sensor=sensor,
        controller=None,
        dt=0.05,
    )
    
    print("Running simulation with open-loop controls...")
    
    # Create control sequence for ~20 seconds
    # Start moving forward, then turn, then move forward again
    duration = 20.0
    n_steps = int(duration / sim.dt)
    
    control_sequence = []
    for i in range(n_steps):
        t = i * sim.dt
        if t < 5.0:
            # Move forward
            v_cmd = 1.0
            yaw_rate_cmd = 0.0
        elif t < 8.0:
            # Turn left
            v_cmd = 0.8
            yaw_rate_cmd = 0.5
        elif t < 13.0:
            # Move forward
            v_cmd = 1.0
            yaw_rate_cmd = 0.0
        elif t < 16.0:
            # Turn right
            v_cmd = 0.8
            yaw_rate_cmd = -0.5
        else:
            # Move forward
            v_cmd = 1.0
            yaw_rate_cmd = 0.0
        
        control_sequence.append((v_cmd, yaw_rate_cmd))
    
    # Run simulation
    logger = sim.run(duration=duration, control_sequence=control_sequence)
    
    print(f"Simulation complete. Logged {len(logger.time)} steps.")
    if logger.collision.any() if hasattr(logger.collision, 'any') else any(logger.collision):
        print("Warning: Collision detected!")
    
    # Create outputs directory
    outputs_dir = Path(__file__).parent.parent / "outputs"
    outputs_dir.mkdir(exist_ok=True)
    
    # Save plot
    print("Generating trajectory plot...")
    plot_trajectory(world, logger, save_path=str(outputs_dir / "open_loop_traj.png"))
    print(f"Saved: {outputs_dir / 'open_loop_traj.png'}")
    
    # Save animation
    print("Generating animation...")
    animate_simulation(world, logger, save_path=str(outputs_dir / "open_loop.gif"))
    print(f"Saved: {outputs_dir / 'open_loop.gif'}")
    
    # Save log
    print("Saving log data...")
    logger.save_npz(str(outputs_dir / "open_loop_log.npz"))
    print(f"Saved: {outputs_dir / 'open_loop_log.npz'}")
    
    print("Demo complete!")


if __name__ == "__main__":
    main()

