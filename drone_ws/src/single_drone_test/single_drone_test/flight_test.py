#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from enum import Enum, auto
from typing import Optional, List

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry, VehicleStatus

class FlightState(Enum):
    WAIT_FOR_CONNECTION = auto()
    WAIT_FOR_ESTIMATOR = auto()
    STREAM_SETPOINTS = auto()
    REQUEST_OFFBOARD = auto()
    REQUEST_ARM = auto()
    TAKEOFF = auto()
    HOLD = auto()
    TRANSLATE = auto()
    RETURN = auto()
    LAND = auto()
    DISARM = auto()
    COMPLETE = auto()
    FAILSAFE = auto()

class SingleDroneTestNode(Node):
    def __init__(self) -> None:
        super().__init__('single_drone_test_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos_profile)

        # Internal state
        self.nav_state: int = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state: int = VehicleStatus.ARMING_STATE_DISARMED
        self.vehicle_pos: List[float] = [0.0, 0.0, 0.0]
        self.vehicle_connected: bool = False
        
        self.state: FlightState = FlightState.WAIT_FOR_CONNECTION
        self.state_timer: int = 0
        self.initial_pos: Optional[List[float]] = None
        self.target_pos: List[float] = [0.0, 0.0, 0.0]

        self.control_timer = self.create_timer(0.05, self.timer_callback) # 20 Hz
        
        self.get_logger().info("Single Drone Test Node initialized.")

    def log_transition(self, new_state: FlightState) -> None:
        if self.state != new_state:
            self.get_logger().info(f"[{new_state.name}]")
            self.state = new_state
            self.state_timer = 0

    def vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        self.vehicle_connected = True

    def vehicle_odometry_callback(self, msg: VehicleOdometry) -> None:
        self.vehicle_pos = [msg.position[0], msg.position[1], msg.position[2]]

    def publish_offboard_control_mode(self) -> None:
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self, x: float, y: float, z: float) -> None:
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = 0.0  # Pointing North
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
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
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        # Failsafe checks
        if self.state not in [FlightState.WAIT_FOR_CONNECTION, FlightState.WAIT_FOR_ESTIMATOR, FlightState.FAILSAFE, FlightState.COMPLETE, FlightState.DISARM]:
            if not self.vehicle_connected:
                self.get_logger().error("Connection lost!")
                self.log_transition(FlightState.FAILSAFE)
                
        if self.state in [FlightState.TAKEOFF, FlightState.HOLD, FlightState.TRANSLATE, FlightState.RETURN]:
            if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.get_logger().error("Offboard mode lost unexpectedly!")
                self.log_transition(FlightState.FAILSAFE)
        
        # State machine
        if self.state == FlightState.WAIT_FOR_CONNECTION:
            if self.vehicle_connected:
                self.log_transition(FlightState.WAIT_FOR_ESTIMATOR)

        elif self.state == FlightState.WAIT_FOR_ESTIMATOR:
            if self.vehicle_pos != [0.0, 0.0, 0.0] and self.vehicle_pos[2] != 0.0:
                self.initial_pos = list(self.vehicle_pos)
                self.target_pos = list(self.initial_pos)
                self.get_logger().info(f"Initial position established: {self.initial_pos}")
                self.log_transition(FlightState.STREAM_SETPOINTS)

        elif self.state == FlightState.STREAM_SETPOINTS:
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(*self.target_pos)
            self.state_timer += 1
            if self.state_timer >= 40: # Stream for 2 seconds (40 * 0.05)
                self.log_transition(FlightState.REQUEST_OFFBOARD)

        elif self.state == FlightState.REQUEST_OFFBOARD:
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(*self.target_pos)
            
            if self.state_timer % 20 == 0:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                
            if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.log_transition(FlightState.REQUEST_ARM)
            self.state_timer += 1

        elif self.state == FlightState.REQUEST_ARM:
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(*self.target_pos)
            
            if self.state_timer % 20 == 0:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
                
            if self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.log_transition(FlightState.TAKEOFF)
            self.state_timer += 1

        elif self.state == FlightState.TAKEOFF:
            self.target_pos = [self.initial_pos[0], self.initial_pos[1], self.initial_pos[2] - 5.0] # NED: Z down
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(*self.target_pos)
            
            z_error = abs(self.vehicle_pos[2] - self.target_pos[2])
            if z_error < 0.5:
                self.log_transition(FlightState.HOLD)

        elif self.state == FlightState.HOLD:
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(*self.target_pos)
            
            self.state_timer += 1
            if self.state_timer >= 200: # 10 seconds at 20 Hz
                self.log_transition(FlightState.TRANSLATE)

        elif self.state == FlightState.TRANSLATE:
            self.target_pos = [self.initial_pos[0] + 5.0, self.initial_pos[1], self.initial_pos[2] - 5.0]
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(*self.target_pos)
            
            x_error = abs(self.vehicle_pos[0] - self.target_pos[0])
            if x_error < 0.5:
                self.state_timer += 1
                if self.state_timer >= 100: # Wait 5 seconds after reaching translation
                    self.log_transition(FlightState.RETURN)
            else:
                self.state_timer = 0

        elif self.state == FlightState.RETURN:
            self.target_pos = [self.initial_pos[0], self.initial_pos[1], self.initial_pos[2] - 5.0]
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(*self.target_pos)
            
            x_error = abs(self.vehicle_pos[0] - self.target_pos[0])
            if x_error < 0.5:
                self.state_timer += 1
                if self.state_timer >= 100:
                    self.log_transition(FlightState.LAND)
            else:
                self.state_timer = 0

        elif self.state == FlightState.LAND:
            # Send land command continuously
            if self.state_timer % 20 == 0:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                
            if self.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.log_transition(FlightState.DISARM)
            self.state_timer += 1

        elif self.state == FlightState.DISARM:
            # Drone is disarmed
            self.log_transition(FlightState.COMPLETE)

        elif self.state == FlightState.COMPLETE:
            pass
            
        elif self.state == FlightState.FAILSAFE:
            self.get_logger().info("In Failsafe state. Allowing PX4 to handle.", throttle_duration_sec=2.0)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = SingleDroneTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
