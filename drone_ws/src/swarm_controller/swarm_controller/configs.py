from dataclasses import dataclass
from typing import Optional

@dataclass
class DroneConfig:
    drone_id: str
    namespace: str
    system_id: int
    world_spawn_x: float
    world_spawn_y: float
    world_spawn_z: float
    world_yaw: float

@dataclass
class MissionConfig:
    control_rate_hz: int = 20
    takeoff_altitude: float = 15.0
    hold_duration: int = 20  # ticks at 20Hz -> 1s
    waypoint_timeout: int = 300 # ticks at 20Hz -> 15s (legacy tick counter)
    landing_timeout: int = 400
    estimator_timeout: int = 900 # ticks at 20Hz -> 45s to accommodate multi-instance EKF2 lockstep convergence
    min_waypoint_dwell_ticks: int = 40

    # Simulation-time and watchdog parameters
    waypoint_sim_timeout: float = 20.0       # Max simulation seconds for waypoint arrival before genuine timeout
    waypoint_wall_watchdog: float = 90.0     # Outer safety wall-clock watchdog (sec) preventing deadlock
    min_waypoint_dwell_sec: float = 1.0      # Minimum dwell time at goal in simulation seconds
    stall_detection_sec: float = 6.0         # Simulation seconds of zero distance progress before stall resolution

    
    # APF & Waypoint parameters
    enable_apf: bool = False
    safe_distance: float = 12.0
    emergency_distance: float = 5.0
    repulsive_gain: float = 30.0  # M = repulsive_gain * (1/d - 1/safe)
    max_repulsion: float = 15.0
    goal_tolerance: float = 1.9
    max_control_offset: float = 15.0
