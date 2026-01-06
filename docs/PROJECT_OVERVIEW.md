# Project Overview - How Each File Works

This document explains how each file in the FluxSpace 2D Drone Testbed works, organized by module.

## 📁 Project Structure Overview

```
fluxspace-testbed/
├── src/fluxspace_sim/          # Main simulation package
│   ├── config.py               # Default settings/constants
│   ├── world/                  # World environment
│   │   ├── geometry.py         # Math helper functions
│   │   ├── obstacles.py        # Obstacle definitions
│   │   └── world.py            # Complete world with boundaries
│   ├── drone/                  # Drone components
│   │   ├── state.py            # What the drone "knows" about itself
│   │   ├── dynamics.py         # How the drone moves
│   │   └── controller.py       # How the drone decides what to do
│   ├── sensors/                # Sensors
│   │   └── lidar2d.py          # Simulated lidar sensor
│   ├── sim/                    # Simulation engine
│   │   ├── simulator.py        # Main simulation loop
│   │   └── logger.py           # Records everything that happens
│   └── viz/                    # Visualization
│       ├── plot.py             # Static images
│       └── animate.py          # Moving animations
├── scripts/                    # Demo programs
│   ├── run_demo_open_loop.py  # Test with manual controls
│   └── run_demo_waypoints.py  # Test with waypoint following
└── tests/                      # Automated tests
```

---

## 🎯 Core Files Explained

### Configuration (`config.py`)
**Purpose**: Stores default values used throughout the simulation.
- Like a settings file - all the "default" numbers
- Examples: How fast the drone moves by default, how many lidar rays, etc.
- Makes it easy to change settings in one place

### World Module (`world/`)

#### `geometry.py`
**Purpose**: Math helper functions for calculating intersections.
- Contains complex math calculations
- Checks if a "ray" (like a laser beam) hits a circle or rectangle
- Used by the lidar sensor to detect obstacles
- Think of it as the "calculator" for geometric problems

#### `obstacles.py`
**Purpose**: Defines what obstacles look like.
- `CircleObstacle`: Round obstacles (like pillars)
- `RectObstacle`: Rectangular obstacles (like walls)
- Each obstacle knows if a point is inside it
- Each obstacle can tell lidar where it intersects

#### `world.py`
**Purpose**: The complete 2D environment.
- Combines boundaries (the edges of the world) and obstacles
- Checks if the drone collides with anything
- The "playing field" where the drone operates

### Drone Module (`drone/`)

#### `state.py`
**Purpose**: Stores what the drone knows about itself.
- The drone's position (x, y)
- Which direction it's facing (yaw angle)
- How fast it's moving (velocity)
- Like a "status card" the drone carries around

#### `dynamics.py`
**Purpose**: The physics of how the drone moves.
- Takes commands (go forward, turn left) and calculates new position
- Uses simple math: if moving forward at speed V, position changes by V × time
- Updates everything based on time passing (like a clock ticking)
- No complex physics - just basic movement math

#### `controller.py`
**Purpose**: The "brain" that decides what the drone should do.
- `WaypointController`: Looks at waypoints and figures out how to reach them
- Calculates: "Which direction do I need to turn? How fast should I go?"
- Like a GPS that tells you where to go and how to get there

### Sensors Module (`sensors/`)

#### `lidar2d.py`
**Purpose**: Simulated lidar (light detection and ranging) sensor.
- Simulates a sensor that shoots out rays (like laser beams) in all directions
- Measures distance to obstacles by seeing where rays hit
- Returns an array of distances: [distance1, distance2, distance3, ...]
- Like a radar that shows you how far obstacles are in each direction

### Simulation Module (`sim/`)

#### `simulator.py`
**Purpose**: The main engine that runs everything.
- Coordinates all the pieces: world, drone, sensors, controller
- Runs the simulation step-by-step (like frames in a movie)
- Each step: get commands → move drone → check sensors → check collisions
- Stops if the drone crashes

#### `logger.py`
**Purpose**: Records everything that happens.
- Like a "black box" recorder
- Saves: position every step, sensor readings, control commands
- Can save to file for later analysis
- Lets you replay or analyze what happened

### Visualization Module (`viz/`)

#### `plot.py`
**Purpose**: Creates static images of the simulation.
- Draws the world, obstacles, and drone's path
- Shows where the drone went (trajectory)
- Can show lidar rays as lines
- Like taking a photo of the simulation

#### `animate.py`
**Purpose**: Creates moving animations.
- Shows the drone moving through the world over time
- Like a video of the simulation
- Can show lidar rays moving as the drone moves
- Makes it easy to see what happened visually

### Demo Scripts (`scripts/`)

#### `run_demo_open_loop.py`
**Purpose**: Tests the simulator with manual control commands.
- Pre-programmed sequence: "Go forward 5 seconds, turn left 3 seconds..."
- Tests if the basic simulation works
- Good for debugging

#### `run_demo_waypoints.py`
**Purpose**: Tests waypoint-following controller.
- Gives the drone a list of points to visit
- Lets the controller figure out how to get there
- Tests if the controller works correctly

### Tests (`tests/`)

#### Purpose: Automated checks that everything works correctly.
- `test_dynamics.py`: Verifies the drone moves correctly
- `test_collision.py`: Verifies collision detection works
- `test_lidar.py`: Verifies the lidar sensor works
- Like quality control - ensures nothing is broken

---

## 🔄 How It All Works Together

1. **Setup Phase**:
   - Create a world with boundaries and obstacles
   - Create a drone with starting position
   - Create a sensor (lidar)
   - Create a controller (or use manual controls)

2. **Simulation Loop** (repeats many times per second):
   - Controller decides: "What should I do?" → gives commands
   - Dynamics updates: "How do I move?" → changes position
   - Sensor scans: "What do I see?" → measures distances
   - World checks: "Did I crash?" → collision detection
   - Logger records: "What happened?" → saves data

3. **Visualization Phase**:
   - Take all the logged data
   - Draw it as an image or animation
   - Save to file for viewing

---

## 🎓 Key Concepts for Beginners

- **State**: What the drone knows (position, direction, speed)
- **Dynamics**: The rules of how the drone moves
- **Controller**: The logic that decides what to do
- **Sensor**: What the drone can "see" (lidar)
- **World**: The environment with obstacles
- **Simulator**: The engine that runs everything
- **Logger**: Records everything that happens

Think of it like a video game:
- The **world** is the level
- The **drone** is the character
- The **controller** is the AI that controls the character
- The **sensor** is what the character can see
- The **simulator** is the game engine
- The **logger** is the replay system

