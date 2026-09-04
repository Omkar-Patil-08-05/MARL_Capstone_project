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
    waypoint_timeout: int = 300 # ticks at 20Hz -> 15s
    landing_timeout: int = 400
    estimator_timeout: int = 200
    min_waypoint_dwell_ticks: int = 40

    
    # APF & Waypoint parameters
    enable_apf: bool = False
    safe_distance: float = 12.0
    emergency_distance: float = 5.0
    repulsive_gain: float = 30.0  # M = repulsive_gain * (1/d - 1/safe)
    max_repulsion: float = 15.0
    goal_tolerance: float = 1.9
    max_control_offset: float = 15.0
