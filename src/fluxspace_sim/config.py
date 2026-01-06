"""
CONFIGURATION FILE - Default Settings for the Simulation

WHAT THIS FILE DOES:
This file stores all the default settings used throughout the simulation. Think of it as a
settings menu where we define default values that can be used by other parts of the code.

WHY IT EXISTS:
Instead of hardcoding numbers everywhere in the code, we put default values here. This makes
it easy to change settings in one place and have them apply everywhere.

FOR BEGINNERS:
- These are just default numbers (like speed limits or sensor settings)
- You can change these values to tweak how the simulation behaves
- Other files import these values and use them
"""

# Default simulation parameters
DEFAULT_DT = 0.05  # seconds
DEFAULT_DRONE_RADIUS = 0.25  # meters

# Default lidar parameters
DEFAULT_LIDAR_N_RAYS = 60
DEFAULT_LIDAR_FOV_DEG = 270
DEFAULT_LIDAR_MAX_RANGE = 10.0  # meters

# Default controller parameters
DEFAULT_CONTROLLER_K_YAW = 2.0
DEFAULT_CONTROLLER_V_CMD = 1.0  # m/s
DEFAULT_CONTROLLER_WAYPOINT_TOLERANCE = 0.3  # meters

