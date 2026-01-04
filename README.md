# FluxSpace 2D Drone Testbed

A minimal 2D simulation environment for testing drone path planning and autonomy algorithms.

## Features

- Simple 2D world with boundaries and obstacles (circles, rectangles)
- Unicycle-style drone dynamics model
- Simulated 2D lidar/rangefinder sensor
- Waypoint-following controller
- Visualization and animation tools
- Modular design for easy extension

## Installation

```bash
pip install -e .
```

Or install dependencies directly:
```bash
pip install numpy matplotlib pytest
```

## Usage

Run the demo scripts:

```bash
# Open-loop control demo
python scripts/run_demo_open_loop.py

# Waypoint-following demo
python scripts/run_demo_waypoints.py
```

Outputs (plots, animations, logs) will be saved to `outputs/`.

## Running Tests

```bash
pytest
```

## Project Structure

- `src/fluxspace_sim/` - Main simulation package
  - `world/` - World model, obstacles, geometry
  - `drone/` - Drone state, dynamics, controllers
  - `sensors/` - Sensor models (lidar)
  - `sim/` - Simulator and logging
  - `viz/` - Visualization tools
- `scripts/` - Demo scripts
- `tests/` - Unit tests
- `outputs/` - Generated outputs (gitignored)

## Documentation

See [docs/USAGE_GUIDE.md](docs/USAGE_GUIDE.md) for a comprehensive usage guide with examples.

## Extending

To add a custom controller, implement the controller interface and pass it to the simulator. To add path planning, integrate with the controller and sensor interfaces.

