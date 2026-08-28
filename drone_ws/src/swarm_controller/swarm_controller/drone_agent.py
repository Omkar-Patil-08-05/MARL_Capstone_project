from enum import Enum, auto
from typing import Optional, List
import math
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand
from .configs import DroneConfig, MissionConfig
from .px4_interface import PX4Interface

class FlightState(Enum):
    WAIT_FOR_CONNECTION = auto()
    WAIT_FOR_ESTIMATOR = auto()
    STREAM_SETPOINTS = auto()
    REQUEST_OFFBOARD = auto()
    REQUEST_ARM = auto()
    TAKEOFF = auto()
    WAYPOINT_NAVIGATION = auto()
    HOLD = auto()
    LAND = auto()
    DISARM = auto()
    COMPLETE = auto()
    FAILSAFE = auto()

class DroneAgent:
    def __init__(self, node: Node, config: DroneConfig, mission: MissionConfig):
        self.node = node
        self.config = config
        self.mission = mission
        self.px4 = PX4Interface(node, config)
        
        self.state: FlightState = FlightState.WAIT_FOR_CONNECTION
        self.state_timer: int = 0
        self.initial_pos: Optional[List[float]] = None
        
        # Immutable goal for the current state (local frame)
        self.mission_goal_local: List[float] = [0.0, 0.0, 0.0]
        # Temporary offset computed by APF (local frame)
        self.avoidance_offset_local: List[float] = [0.0, 0.0, 0.0]
        
        # H8 minimum dwell flag for safety-forced hover
        self.require_min_dwell: bool = False
        
    def _log_transition(self, new_state: FlightState) -> None:
        if self.state != new_state:
            self.node.get_logger().info(f"[{self.config.drone_id}] Transition: {self.state.name} -> {new_state.name}")
            self.state = new_state
            self.state_timer = 0
            
    def get_state(self) -> FlightState:
        return self.state

    def set_mission_goal_local(self, x: float, y: float, z: float, require_min_dwell: bool = False) -> None:
        self.mission_goal_local = [x, y, z]
        self.require_min_dwell = require_min_dwell

    def set_avoidance_offset_local(self, offset_x: float, offset_y: float) -> None:
        self.avoidance_offset_local = [offset_x, offset_y, 0.0]

    def tick(self) -> None:
        # Failsafe checks
        if self.state not in [FlightState.WAIT_FOR_CONNECTION, FlightState.WAIT_FOR_ESTIMATOR, FlightState.FAILSAFE, FlightState.COMPLETE, FlightState.DISARM]:
            if not self.px4.is_connected():
                self.node.get_logger().error(f"[{self.config.drone_id}] Connection lost!")
                self._log_transition(FlightState.FAILSAFE)
                
        if self.state in [FlightState.TAKEOFF, FlightState.WAYPOINT_NAVIGATION, FlightState.HOLD]:
            if not self.px4.is_offboard():
                self.node.get_logger().error(f"[{self.config.drone_id}] Offboard mode lost unexpectedly!")
                self._log_transition(FlightState.FAILSAFE)

        # State machine
        if self.state == FlightState.WAIT_FOR_CONNECTION:
            if self.px4.is_connected():
                self._log_transition(FlightState.WAIT_FOR_ESTIMATOR)

        elif self.state == FlightState.WAIT_FOR_ESTIMATOR:
            if self.px4.is_estimator_ready():
                self.initial_pos = list(self.px4.current_position)
                self.mission_goal_local = list(self.initial_pos)
                self.node.get_logger().info(f"[{self.config.drone_id}] Initial position established: {self.initial_pos}")
                self._log_transition(FlightState.STREAM_SETPOINTS)
            else:
                self.state_timer += 1
                if self.state_timer >= self.mission.estimator_timeout:
                    self.node.get_logger().error(f"[{self.config.drone_id}] Estimator timeout.")
                    self._log_transition(FlightState.FAILSAFE)

        elif self.state == FlightState.STREAM_SETPOINTS:
            self.px4.publish_offboard_control_mode()
            self.px4.publish_trajectory_setpoint(self.mission_goal_local)
            self.state_timer += 1
            if self.state_timer >= self.mission.control_rate_hz * 2: # Stream for 2 seconds
                self._log_transition(FlightState.REQUEST_OFFBOARD)

        elif self.state == FlightState.REQUEST_OFFBOARD:
            self.px4.publish_offboard_control_mode()
            self.px4.publish_trajectory_setpoint(self.mission_goal_local)
            
            if self.state_timer % self.mission.control_rate_hz == 0:
                self.px4.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                
            if self.px4.is_offboard():
                self._log_transition(FlightState.REQUEST_ARM)
            self.state_timer += 1

        elif self.state == FlightState.REQUEST_ARM:
            self.px4.publish_offboard_control_mode()
            self.px4.publish_trajectory_setpoint(self.mission_goal_local)
            
            if self.state_timer % self.mission.control_rate_hz == 0:
                self.px4.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
                
            if self.px4.is_armed():
                self._log_transition(FlightState.TAKEOFF)
            self.state_timer += 1

        elif self.state == FlightState.TAKEOFF:
            takeoff_target = [self.initial_pos[0], self.initial_pos[1], self.initial_pos[2] - self.mission.takeoff_altitude] # NED: Z down
            self.px4.publish_offboard_control_mode()
            self.px4.publish_trajectory_setpoint(takeoff_target)
            
            z_error = abs(self.px4.current_position[2] - takeoff_target[2])
            if z_error < 0.5:
                self._log_transition(FlightState.WAYPOINT_NAVIGATION)

        elif self.state == FlightState.WAYPOINT_NAVIGATION:
            self.state_timer += 1
            # compute temporary target by adding APF offset to mission goal
            control_target_local = [
                self.mission_goal_local[0] + self.avoidance_offset_local[0],
                self.mission_goal_local[1] + self.avoidance_offset_local[1],
                self.mission_goal_local[2] # Z stays fixed at mission_goal
            ]
            
            self.px4.publish_offboard_control_mode()
            self.px4.publish_trajectory_setpoint(control_target_local)
            
            # evaluate distance to IMMUTABLE mission goal
            dx = self.px4.current_position[0] - self.mission_goal_local[0]
            dy = self.px4.current_position[1] - self.mission_goal_local[1]
            dist_to_goal = math.sqrt(dx*dx + dy*dy)
            
            if dist_to_goal < self.mission.goal_tolerance:
                if not self.require_min_dwell or self.state_timer >= self.mission.min_waypoint_dwell_ticks:
                    # Goal reached
                    self.avoidance_offset_local = [0.0, 0.0, 0.0]
                    self._log_transition(FlightState.HOLD)

        elif self.state == FlightState.HOLD:
            self.px4.publish_offboard_control_mode()
            self.px4.publish_trajectory_setpoint(self.mission_goal_local)
            
            self.state_timer += 1
            if self.state_timer >= self.mission.hold_duration:
                self._log_transition(FlightState.LAND)

        elif self.state == FlightState.LAND:
            if self.state_timer % self.mission.control_rate_hz == 0:
                self.px4.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                
            if not self.px4.is_armed():
                self._log_transition(FlightState.DISARM)
            self.state_timer += 1

        elif self.state == FlightState.DISARM:
            self._log_transition(FlightState.COMPLETE)

        elif self.state == FlightState.COMPLETE:
            pass
            
        elif self.state == FlightState.FAILSAFE:
            if self.state_timer % (self.mission.control_rate_hz * 2) == 0:
                self.node.get_logger().info(f"[{self.config.drone_id}] In Failsafe state. Allowing PX4 to handle.")
            self.state_timer += 1
