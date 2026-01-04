# FluxSpace 2D Drone Testbed - Usage Guide

This guide explains how to use the drone testbed simulator to create and run simulations.

## Table of Contents
1. [Quick Start](#quick-start)
2. [Core Concepts](#core-concepts)
3. [Running Existing Demos](#running-existing-demos)
4. [Creating Custom Simulations](#creating-custom-simulations)
5. [Extending the Testbed](#extending-the-testbed)
6. [Common Use Cases](#common-use-cases)

---

## Quick Start

### Installation
```bash
# Install dependencies
pip install -r requirements.txt

# Install the package in editable mode
pip install -e .
```

### Run a Demo
```bash
# Open-loop control demo
python scripts/run_demo_open_loop.py

# Waypoint-following demo
python scripts/run_demo_waypoints.py
```

Outputs (plots, animations, logs) will be saved to the `outputs/` directory.

---

## Core Concepts

### 1. **World (World2D)**
The 2D environment contains:
- **Boundaries**: Rectangular world limits (xmin, xmax, ymin, ymax)
- **Obstacles**: Circles or rectangles that the drone should avoid
- **Collision Detection**: Checks if drone collides with obstacles or boundaries

### 2. **Drone State**
The drone's state includes:
- `x, y`: Position (meters)
- `yaw`: Heading angle (radians, -π to π)
- `v`: Forward velocity (m/s)

### 3. **Dynamics**
Simple unicycle model:
- Position updates based on velocity and heading
- Heading changes based on yaw rate command
- No complex physics (kinematic model only)

### 4. **Sensors**
- **Lidar2D**: Simulated 2D lidar/rangefinder
  - Returns distances to obstacles/boundaries
  - Configurable FOV (field of view), number of rays, max range
  - Used for obstacle detection and path planning

### 5. **Controllers**
- **WaypointController**: Follows a list of waypoints
- **Custom Controllers**: Implement the `Controller` interface

### 6. **Simulator**
- Steps through time with fixed timestep (dt)
- Logs state, control commands, and sensor data
- Detects collisions and stops simulation if needed

---

## Running Existing Demos

### Demo 1: Open-Loop Control
```bash
python scripts/run_demo_open_loop.py
```

**What it does:**
- Creates a world with boundaries (0-10m x 0-6m) and obstacles
- Places drone at (1, 1)
- Applies a fixed sequence of control commands:
  - Move forward for 5 seconds
  - Turn left for 3 seconds
  - Move forward for 5 seconds
  - Turn right for 3 seconds
  - Move forward for remaining time

**Outputs:**
- `outputs/open_loop_traj.png`: Static plot showing trajectory
- `outputs/open_loop.gif`: Animated visualization
- `outputs/open_loop_log.npz`: Simulation data (load with numpy)

### Demo 2: Waypoint Following
```bash
python scripts/run_demo_waypoints.py
```

**What it does:**
- Same world as demo 1
- Uses WaypointController to follow waypoints: [(8,1), (8,5), (2,5), (2,1)]
- Controller automatically computes heading and velocity to reach each waypoint

**Outputs:**
- `outputs/waypoints_traj.png`: Static plot showing trajectory
- `outputs/waypoints.gif`: Animated visualization
- `outputs/waypoints_log.npz`: Simulation data

---

## Creating Custom Simulations

Here's a step-by-step guide to create your own simulation:

### Step 1: Import Required Modules
```python
from fluxspace_sim.world import World2D, CircleObstacle, RectObstacle
from fluxspace_sim.drone.state import DroneState
from fluxspace_sim.drone.controller import WaypointController
from fluxspace_sim.sensors.lidar2d import Lidar2D
from fluxspace_sim.sim.simulator import Simulator
from fluxspace_sim.viz.plot import plot_trajectory
from fluxspace_sim.viz.animate import animate_simulation
```

### Step 2: Create a World
```python
# Define world boundaries
world = World2D(
    xmin=0.0,
    ymin=0.0,
    xmax=10.0,
    ymax=10.0,
    obstacles=[
        CircleObstacle(cx=5.0, cy=5.0, radius=1.5),
        RectObstacle(xmin=2.0, ymin=2.0, xmax=4.0, ymax=4.0),
        # Add more obstacles as needed
    ]
)
```

### Step 3: Set Initial Drone State
```python
initial_state = DroneState(
    x=1.0,      # Starting x position (meters)
    y=1.0,      # Starting y position (meters)
    yaw=0.0,    # Starting heading (radians, 0 = facing +x)
    v=0.0       # Initial velocity (m/s)
)
```

### Step 4: Create Sensor
```python
sensor = Lidar2D(
    n_rays=60,        # Number of lidar rays
    fov_deg=270,      # Field of view (degrees)
    max_range=10.0    # Maximum detection range (meters)
)
```

### Step 5: Create Controller (Optional)
```python
# Option A: Waypoint controller
waypoints = [(3.0, 3.0), (7.0, 3.0), (7.0, 7.0), (3.0, 7.0)]
controller = WaypointController(
    waypoints=waypoints,
    k_yaw=2.0,              # Yaw rate proportional gain
    v_cmd=1.0,              # Commanded velocity (m/s)
    waypoint_tolerance=0.3  # Distance to consider waypoint reached (m)
)

# Option B: No controller (use open-loop controls)
controller = None
```

### Step 6: Create and Run Simulator
```python
# Create simulator
sim = Simulator(
    world=world,
    initial_state=initial_state,
    sensor=sensor,
    controller=controller,
    dt=0.05,  # Time step (seconds)
)

# Option A: Run with controller
logger = sim.run(duration=30.0)  # Run for 30 seconds

# Option B: Run with open-loop control sequence
control_sequence = [
    (1.0, 0.0),   # (v_cmd, yaw_rate_cmd) for step 1
    (1.0, 0.0),   # Step 2
    (1.0, 0.5),   # Step 3: turn left
    # ... more controls
]
logger = sim.run(duration=10.0, control_sequence=control_sequence)
```

### Step 7: Visualize Results
```python
# Static plot
plot_trajectory(world, logger, save_path="my_trajectory.png")

# Animation
animate_simulation(world, logger, save_path="my_animation.gif")

# Save data
logger.save_npz("my_simulation_log.npz")
```

### Complete Example
```python
from fluxspace_sim.world import World2D, CircleObstacle
from fluxspace_sim.drone.state import DroneState
from fluxspace_sim.drone.controller import WaypointController
from fluxspace_sim.sensors.lidar2d import Lidar2D
from fluxspace_sim.sim.simulator import Simulator
from fluxspace_sim.viz.plot import plot_trajectory

# Create world
world = World2D(0, 0, 10, 10, obstacles=[CircleObstacle(5, 5, 1.0)])

# Initialize drone
initial_state = DroneState(x=1.0, y=1.0, yaw=0.0, v=0.0)

# Create sensor and controller
sensor = Lidar2D()
waypoints = [(8, 1), (8, 8), (2, 8), (2, 1)]
controller = WaypointController(waypoints=waypoints)

# Run simulation
sim = Simulator(world, initial_state, sensor, controller)
logger = sim.run(duration=30.0)

# Visualize
plot_trajectory(world, logger, save_path="outputs/my_sim.png")
print(f"Simulation complete: {len(logger.time)} steps")
```

---

## Extending the Testbed

### Creating a Custom Controller

To create a custom controller, implement the `Controller` interface:

```python
from fluxspace_sim.drone.controller import Controller
from fluxspace_sim.drone.state import DroneState
import numpy as np

class MyCustomController(Controller):
    def __init__(self, target_x, target_y):
        self.target_x = target_x
        self.target_y = target_y
    
    def compute_control(self, state: DroneState, sensor_data: dict | None = None):
        """
        Compute control commands.
        
        Args:
            state: Current drone state
            sensor_data: Optional dict with sensor data (e.g., {'lidar': np.array})
        
        Returns:
            (v_cmd, yaw_rate_cmd): Tuple of commanded velocity and yaw rate
        """
        # Compute desired heading to target
        dx = self.target_x - state.x
        dy = self.target_y - state.y
        desired_yaw = np.arctan2(dy, dx)
        
        # Compute yaw error
        yaw_error = desired_yaw - state.yaw
        yaw_error = ((yaw_error + np.pi) % (2 * np.pi)) - np.pi
        
        # Simple proportional control
        v_cmd = 1.0
        yaw_rate_cmd = 2.0 * yaw_error
        
        return v_cmd, yaw_rate_cmd
```

Use your custom controller:
```python
controller = MyCustomController(target_x=8.0, target_y=5.0)
sim = Simulator(world, initial_state, sensor, controller)
logger = sim.run(duration=20.0)
```

### Using Lidar Data in Controller

```python
class ObstacleAvoidanceController(Controller):
    def compute_control(self, state: DroneState, sensor_data: dict | None = None):
        if sensor_data and 'lidar' in sensor_data:
            lidar_scan = sensor_data['lidar']
            
            # Find minimum distance (closest obstacle)
            min_dist = np.min(lidar_scan)
            
            # If too close, turn away
            if min_dist < 2.0:
                v_cmd = 0.5  # Slow down
                yaw_rate_cmd = 1.0  # Turn right
            else:
                v_cmd = 1.0
                yaw_rate_cmd = 0.0
        
        return v_cmd, yaw_rate_cmd
```

### Step-by-Step Simulation

For more control, use the `step()` method:

```python
sim = Simulator(world, initial_state, sensor, controller)

for i in range(100):
    # Option 1: Use controller
    success = sim.step()
    
    # Option 2: Override controller
    success = sim.step(control_override=(1.0, 0.5))
    
    if not success:  # Collision detected
        print("Collision!")
        break
    
    # Access current state
    print(f"Position: ({sim.state.x:.2f}, {sim.state.y:.2f})")
    
    # Get lidar scan manually
    lidar_scan = sensor.scan(sim.state, world)
    print(f"Min distance: {np.min(lidar_scan):.2f}")
```

---

## Common Use Cases

### 1. Test Path Planning Algorithm
```python
# Generate path with your planner
path = my_path_planner.plan(start=(1,1), goal=(9,9), world=world)

# Convert to waypoints
waypoints = [(p.x, p.y) for p in path]

# Run simulation
controller = WaypointController(waypoints=waypoints)
sim = Simulator(world, initial_state, sensor, controller)
logger = sim.run(duration=60.0)

# Analyze results
if logger.collision.any():
    print("Path caused collision!")
else:
    print("Path successful!")
```

### 2. Test Obstacle Avoidance
```python
# Create controller that uses lidar
controller = ObstacleAvoidanceController()

# Run with obstacles
sim = Simulator(world, initial_state, sensor, controller)
logger = sim.run(duration=30.0)

# Check if avoided obstacles
plot_trajectory(world, logger)
```

### 3. Analyze Sensor Data
```python
import numpy as np

# Load logged data
data = np.load("outputs/waypoints_log.npz")

# Access arrays
times = data['time']
x_positions = data['x']
y_positions = data['y']
lidar_scans = data['lidar_scans']  # Array of scans

# Analyze
for i, scan in enumerate(lidar_scans):
    if scan is not None:
        min_dist = np.nanmin(scan)
        print(f"Step {i}: Min obstacle distance = {min_dist:.2f}m")
```

### 4. Compare Different Controllers
```python
controllers = [
    WaypointController(waypoints=[(8,1), (8,5)]),
    MyCustomController(target_x=8, target_y=5),
]

for i, controller in enumerate(controllers):
    sim = Simulator(world, initial_state, sensor, controller)
    logger = sim.run(duration=20.0)
    plot_trajectory(world, logger, save_path=f"outputs/controller_{i}.png")
```

---

## Tips and Best Practices

1. **Time Step**: Use `dt=0.05` (50ms) for smooth simulations. Smaller values are more accurate but slower.

2. **Sensor Parameters**:
   - `n_rays=60`: Good balance of resolution and performance
   - `fov_deg=270`: Common for front-facing lidar (avoids rear)
   - `max_range=10.0`: Adjust based on world size

3. **Collision Detection**: The drone is modeled as a circle with radius 0.25m. Obstacles are expanded by this radius for collision checking.

4. **Waypoint Tolerance**: Use 0.2-0.5m tolerance for waypoint following. Too small and the drone may oscillate.

5. **Visualization**: 
   - Static plots are faster for analysis
   - Animations are great for presentations
   - Log files (.npz) preserve all data for later analysis

6. **Performance**: For long simulations, consider logging lidar scans only every N steps to reduce memory usage.

---

## Next Steps

- Experiment with different world configurations
- Implement your own path planning algorithm
- Create controllers that use sensor data for obstacle avoidance
- Add noise to sensors for more realistic simulations
- Integrate with your autonomy stack

For questions or issues, check the code comments or modify the demo scripts as examples!

