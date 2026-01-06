"""
SIMULATOR - The Main Engine That Runs Everything

WHAT THIS FILE DOES:
This is the core engine that coordinates all the pieces and runs the simulation step by step.
It's like the "game engine" that makes everything work together.

SIMULATION LOOP (what happens each step):
1. Get control commands (from controller or manual input)
2. Perform lidar scan (what does the drone "see"?)
3. Log current state (record everything)
4. Update drone position (move the drone based on commands)
5. Check for collisions (did the drone crash?)
6. If collision, stop simulation

THE SIMULATOR CONNECTS:
- World (the environment)
- Drone State (where the drone is)
- Dynamics (how the drone moves)
- Sensor (what the drone sees)
- Controller (what the drone should do)
- Logger (recording everything)

FOR BEGINNERS:
- Simulator = the main program that runs the simulation
- Like a video game engine that updates everything 20 times per second
- Each "step" is like one frame in a movie
- "run()" method runs the whole simulation automatically
- "step()" method runs just one step (useful for debugging)
- Returns a logger with all the recorded data
"""

from ..world import World2D
from ..drone.state import DroneState
from ..drone.dynamics import DroneDynamics
from ..drone.controller import Controller
from ..sensors.lidar2d import Lidar2D
from .logger import SimLogger
from ..config import DEFAULT_DT, DEFAULT_DRONE_RADIUS


class Simulator:
    """Main simulator that steps through time."""
    
    def __init__(
        self,
        world: World2D,
        initial_state: DroneState,
        sensor: Lidar2D,
        controller: Controller | None = None,
        dt: float = DEFAULT_DT,
        drone_radius: float = DEFAULT_DRONE_RADIUS,
    ):
        """
        Args:
            world: 2D world model
            initial_state: Initial drone state
            sensor: Lidar sensor
            controller: Optional controller (if None, must provide control_override in step)
            dt: Time step (seconds)
            drone_radius: Radius of drone for collision detection
        """
        self.world = world
        self.state = initial_state
        self.dynamics = DroneDynamics(dt=dt)
        self.sensor = sensor
        self.controller = controller
        self.dt = dt
        self.drone_radius = drone_radius
        self.logger = SimLogger()
        self.collided = False
    
    def step(self, control_override: tuple[float, float] | None = None) -> bool:
        """
        Step the simulation forward one time step.
        
        Args:
            control_override: Optional (v_cmd, yaw_rate_cmd) tuple to override controller
            
        Returns:
            False if collision occurred (simulation should stop), True otherwise
        """
        if self.collided:
            return False
        
        # Compute control
        if control_override is not None:
            v_cmd, yaw_rate_cmd = control_override
            # Perform lidar scan (for logging only)
            lidar_scan = self.sensor.scan(self.state, self.world)
        elif self.controller is not None:
            # Get lidar scan for controller and logging
            lidar_scan = self.sensor.scan(self.state, self.world)
            v_cmd, yaw_rate_cmd = self.controller.compute_control(self.state, {"lidar": lidar_scan})
        else:
            raise ValueError("No controller and no control_override provided")
        
        # Log state before update
        t = len(self.logger.time) * self.dt
        self.logger.log(
            t=t,
            x=self.state.x,
            y=self.state.y,
            yaw=self.state.yaw,
            v=self.state.v,
            v_cmd=v_cmd,
            yaw_rate_cmd=yaw_rate_cmd,
            lidar_scan=lidar_scan,
            collision=False,
        )
        
        # Propagate dynamics
        self.state = self.dynamics.step(self.state, v_cmd, yaw_rate_cmd)
        
        # Check collision
        if self.world.collision(self.state.x, self.state.y, self.drone_radius):
            self.collided = True
            self.logger.collision[-1] = True
            return False
        
        return True
    
    def run(self, duration: float, control_sequence: list[tuple[float, float]] | None = None) -> SimLogger:
        """
        Run simulation for a specified duration.
        
        Args:
            duration: Simulation duration in seconds
            control_sequence: Optional list of (v_cmd, yaw_rate_cmd) tuples.
                            If provided and controller is None, use these controls.
                            If shorter than needed, repeat last value or stop.
                            
        Returns:
            SimLogger object with logged data
        """
        n_steps = int(duration / self.dt)
        control_idx = 0
        
        for _ in range(n_steps):
            if control_sequence is not None:
                if control_idx < len(control_sequence):
                    control = control_sequence[control_idx]
                    control_idx += 1
                else:
                    # Use last control or stop
                    control = control_sequence[-1] if control_sequence else (0.0, 0.0)
                if not self.step(control_override=control):
                    break
            else:
                if not self.step():
                    break
        
        return self.logger
    
    def reset(self, initial_state: DroneState | None = None):
        """Reset simulator to initial state."""
        if initial_state is not None:
            self.state = initial_state
        self.logger = SimLogger()
        self.collided = False
        if self.controller is not None and hasattr(self.controller, "reset"):
            self.controller.reset()

