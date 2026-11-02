import math
import time as _pytime
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry, VehicleStatus
from typing import Optional, List
from .configs import DroneConfig

class PX4Interface:
    def __init__(self, node: Node, config: DroneConfig):
        self.node = node
        self.config = config
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        qos_profile_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Build topics dynamically based on namespace
        ns = f"/{self.config.namespace}" if self.config.namespace else ""
        
        self.offboard_control_mode_publisher = self.node.create_publisher(
            OffboardControlMode, f'{ns}/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.node.create_publisher(
            TrajectorySetpoint, f'{ns}/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.node.create_publisher(
            VehicleCommand, f'{ns}/fmu/in/vehicle_command', qos_profile_reliable)
            
        self.vehicle_odometry_subscriber = self.node.create_subscription(
            VehicleOdometry, f'{ns}/fmu/out/vehicle_odometry', self._vehicle_odometry_callback, qos_profile)
        self.vehicle_status_subscriber = self.node.create_subscription(
            VehicleStatus, f'{ns}/fmu/out/vehicle_status_v1', self._vehicle_status_callback, qos_profile)
            
        # Telemetry State (authoritative from vehicle_status)
        self.nav_state: int = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state: int = VehicleStatus.ARMING_STATE_DISARMED
        self.pre_flight_checks_pass: bool = False
        self.current_position: List[float] = [0.0, 0.0, 0.0]
        self.current_velocity: List[float] = [0.0, 0.0, 0.0]
        self.connected: bool = False
        self.odometry_count: int = 0
        self.status_count: int = 0
        self.last_odometry_time_ns: int = 0
        self.last_odometry_wall_time: float = 0.0
        self.last_status_wall_time: float = 0.0
        self.current_sim_time: float = 0.0
        
    def _vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        self.pre_flight_checks_pass = msg.pre_flight_checks_pass
        self.status_count += 1
        self.last_status_wall_time = _pytime.time()
        self.connected = True
        
    def _vehicle_odometry_callback(self, msg: VehicleOdometry) -> None:
        self.current_position = [float(msg.position[0]), float(msg.position[1]), float(msg.position[2])]
        self.current_velocity = [float(msg.velocity[0]), float(msg.velocity[1]), float(msg.velocity[2])]
        self.odometry_count += 1
        self.last_odometry_time_ns = self.node.get_clock().now().nanoseconds
        self.last_odometry_wall_time = _pytime.time()
        # msg.timestamp is in microseconds from simulation clock
        if msg.timestamp > 0:
            self.current_sim_time = float(msg.timestamp) * 1e-6
        self.connected = True
        
    def is_connected(self) -> bool:
        if self.odometry_count == 0:
            return False
        now = _pytime.time()
        return (now - self.last_odometry_wall_time) < 4.0
        
    def is_estimator_ready(self) -> bool:
        """
        Estimator readiness requires verified reception of multiple odometry frames,
        finite 3D coordinates, and timestamp freshness within 2.0 seconds.
        Does not depend on non-zero altitude, correctly supporting ground-level takeoff.
        """
        if self.odometry_count < 5:
            return False
        if not all(math.isfinite(c) for c in self.current_position):
            return False
        now = _pytime.time()
        if (now - self.last_odometry_wall_time) > 2.5:
            return False
        return True
        
    def is_armed(self) -> bool:
        return self.arming_state == VehicleStatus.ARMING_STATE_ARMED
        
    def is_offboard(self) -> bool:
        return self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    def publish_offboard_control_mode(self) -> None:
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self, pos: List[float], yaw: float = 0.0) -> None:
        msg = TrajectorySetpoint()
        msg.position = pos
        msg.yaw = yaw
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.jerk = [float('nan'), float('nan'), float('nan')]
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command: int, **kwargs: float) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = kwargs.get("param1", 0.0)
        msg.param2 = kwargs.get("param2", 0.0)
        msg.param3 = kwargs.get("param3", 0.0)
        msg.param4 = kwargs.get("param4", 0.0)
        msg.param5 = kwargs.get("param5", 0.0)
        msg.param6 = kwargs.get("param6", 0.0)
        msg.param7 = kwargs.get("param7", 0.0)
        
        # Use the explicit system ID for this drone
        msg.target_system = self.config.system_id
        msg.target_component = 1
        msg.source_system = 255 # Standard GCS source system
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)
