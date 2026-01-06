"""
SIMULATION LOGGER - Records Everything That Happens During Simulation

WHAT THIS FILE DOES:
This file creates a "black box recorder" that saves all the important information from the simulation.
Every step, it records where the drone was, what it was doing, what it saw, etc.

WHAT GETS LOGGED:
- Time: When each event happened
- Position: x, y coordinates every step
- Heading: yaw angle (which way drone was facing)
- Velocity: How fast the drone was moving
- Commands: What control commands were given
- Lidar scans: What the sensor detected (optional, can be memory-intensive)
- Collisions: Whether the drone crashed

HOW TO USE:
- Logger automatically records data as simulation runs
- After simulation, call save_npz() to save all data to a file
- Load the file later to analyze what happened

FOR BEGINNERS:
- Logger = a recorder that saves everything (like flight recorder on an airplane)
- Stores data in lists (one entry per simulation step)
- Can save to .npz file for later analysis
- Used by visualization tools to create plots and animations
"""

import numpy as np
from typing import List


class SimLogger:
    """Logger for simulation data."""
    
    def __init__(self):
        """Initialize empty logger."""
        self.time: List[float] = []
        self.x: List[float] = []
        self.y: List[float] = []
        self.yaw: List[float] = []
        self.v: List[float] = []
        self.v_cmd: List[float] = []
        self.yaw_rate_cmd: List[float] = []
        self.lidar_scans: List[np.ndarray] = []
        self.collision: List[bool] = []
    
    def log(
        self,
        t: float,
        x: float,
        y: float,
        yaw: float,
        v: float,
        v_cmd: float,
        yaw_rate_cmd: float,
        lidar_scan: np.ndarray | None = None,
        collision: bool = False,
    ):
        """
        Log a simulation step.
        
        Args:
            t: Time
            x, y, yaw, v: State
            v_cmd, yaw_rate_cmd: Control commands
            lidar_scan: Optional lidar scan (only logged every few steps to save memory)
            collision: Whether collision occurred
        """
        self.time.append(t)
        self.x.append(x)
        self.y.append(y)
        self.yaw.append(yaw)
        self.v.append(v)
        self.v_cmd.append(v_cmd)
        self.yaw_rate_cmd.append(yaw_rate_cmd)
        if lidar_scan is not None:
            self.lidar_scans.append(lidar_scan.copy())
        else:
            self.lidar_scans.append(None)
        self.collision.append(collision)
    
    def get_arrays(self) -> dict:
        """Get all logged data as numpy arrays."""
        return {
            "time": np.array(self.time),
            "x": np.array(self.x),
            "y": np.array(self.y),
            "yaw": np.array(self.yaw),
            "v": np.array(self.v),
            "v_cmd": np.array(self.v_cmd),
            "yaw_rate_cmd": np.array(self.yaw_rate_cmd),
            "lidar_scans": self.lidar_scans,
            "collision": np.array(self.collision),
        }
    
    def save_npz(self, filename: str):
        """Save logged data to .npz file."""
        data = self.get_arrays()
        # Convert lidar_scans list to array if possible
        lidar_list = data.pop("lidar_scans")
        if lidar_list and any(scan is not None for scan in lidar_list):
            # Pad to same length
            max_len = max(len(scan) for scan in lidar_list if scan is not None)
            lidar_array = np.full((len(lidar_list), max_len), np.nan)
            for i, scan in enumerate(lidar_list):
                if scan is not None:
                    lidar_array[i, :len(scan)] = scan
            data["lidar_scans"] = lidar_array
        np.savez(filename, **data)

