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
        
        # Consistent simulation & wall-clock timing tracking
        import time as _pytime
        self.state_start_sim_time: float = 0.0
        self.state_start_wall_time: float = _pytime.time()
        self.last_goal_dist: float = 999.0
        self.last_progress_sim_time: float = 0.0
        
        # Immutable goal for the current state (local frame)
        self.mission_goal_local: List[float] = [0.0, 0.0, 0.0]
        # Temporary offset computed by APF (local frame)
        self.avoidance_offset_local: List[float] = [0.0, 0.0, 0.0]
        
        # Minimum dwell flag for hover actions
        self.require_min_dwell: bool = False

    def get_current_sim_time(self) -> float:
        """
        Returns the authoritative simulation clock time in seconds from ROS 2 (/clock).
        If the clock has not started yet or is unavailable, returns 0.0 without
        mixing wall-clock or foreign monotonic timestamps.
        Wall-clock timeouts and pauses are handled exclusively by elapsed_wall_time().
        """
        try:
            now_ns = self.node.get_clock().now().nanoseconds
            if now_ns > 0:
                return now_ns * 1e-9
        except Exception:
            pass
        return 0.0

    def elapsed_sim_time(self) -> float:
        """Returns simulation seconds elapsed since entering the current state."""
        curr = self.get_current_sim_time()
        if curr <= 0.0 or self.state_start_sim_time <= 0.0:
            if curr > 0.0 and self.state_start_sim_time <= 0.0:
                self.state_start_sim_time = curr
                self.last_progress_sim_time = curr
            return 0.0
        return max(0.0, curr - self.state_start_sim_time)

    def elapsed_wall_time(self) -> float:
        """Returns wall-clock seconds elapsed since entering the current state."""
        import time as _pytime
        return max(0.0, _pytime.time() - self.state_start_wall_time)

    def is_stalled(self) -> bool:
        """Detects whether drone is stuck without forward progress toward goal."""
        if self.state != FlightState.WAYPOINT_NAVIGATION:
            return False
        # If the drone has already reached within goal tolerance, it is not stalled
        dx = self.px4.current_position[0] - self.mission_goal_local[0]
        dy = self.px4.current_position[1] - self.mission_goal_local[1]
        dist_to_goal = math.sqrt(dx*dx + dy*dy)
        if dist_to_goal < self.mission.goal_tolerance:
            return False

        stall_sec = getattr(self.mission, 'stall_detection_sec', 6.0)
        curr_sim = self.get_current_sim_time()
        if curr_sim <= 0.0 or self.last_progress_sim_time <= 0.0:
            return False
        time_without_progress = curr_sim - self.last_progress_sim_time
        # Only declare stalled after at least 8.0s sim in navigation
        return (self.elapsed_sim_time() > 8.0) and (time_without_progress > stall_sec)
        
    def _log_transition(self, new_state: FlightState) -> None:
        if self.state != new_state:
            self.node.get_logger().info(f"[{self.config.drone_id}] Transition: {self.state.name} -> {new_state.name}")
            self.state = new_state
            self.state_timer = 0
            import time as _pytime
            self.state_start_wall_time = _pytime.time()
            self.state_start_sim_time = self.get_current_sim_time()
            self.last_progress_sim_time = self.state_start_sim_time
            self.last_goal_dist = 999.0
            
    def get_state(self) -> FlightState:
        return self.state

    def set_mission_goal_local(self, x: float, y: float, z: float, require_min_dwell: bool = False) -> None:
        self.mission_goal_local = [x, y, z]
        self.require_min_dwell = require_min_dwell
        self.state_start_sim_time = self.get_current_sim_time()
        import time as _pytime
        self.state_start_wall_time = _pytime.time()
        self.last_goal_dist = 999.0
        self.last_progress_sim_time = self.state_start_sim_time

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
                self.node.get_logger().error(f"[{self.config.drone_id}] Offboard mode lost unexpectedly! (nav_state={self.px4.nav_state})")
                self._log_transition(FlightState.FAILSAFE)

        # State machine
        if self.state == FlightState.WAIT_FOR_CONNECTION:
            if self.px4.is_connected():
                self._log_transition(FlightState.WAIT_FOR_ESTIMATOR)
            else:
                self.state_timer += 1
                if self.state_timer >= self.mission.control_rate_hz * 30: # 30s timeout
                    self.node.get_logger().error(
                        f"[{self.config.drone_id}] Timeout waiting for initial PX4 connection after 30s!"
                    )
                    self._log_transition(FlightState.FAILSAFE)

        elif self.state == FlightState.WAIT_FOR_ESTIMATOR:
            if self.px4.is_estimator_ready():
                self.initial_pos = list(self.px4.current_position)
                self.mission_goal_local = list(self.initial_pos)
                self.node.get_logger().info(f"[{self.config.drone_id}] Initial position established: {self.initial_pos}")
                self._log_transition(FlightState.STREAM_SETPOINTS)
            else:
                self.state_timer += 1
                if self.state_timer >= self.mission.estimator_timeout:
                    self.node.get_logger().error(
                        f"[{self.config.drone_id}] Estimator timeout after {self.state_timer} ticks ({self.state_timer/self.mission.control_rate_hz:.1f}s): "
                        f"received {self.px4.odometry_count} odometry frames, pos={self.px4.current_position}"
                    )
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
            
            # Send set mode command every 0.5s
            if self.state_timer % (self.mission.control_rate_hz // 2) == 0:
                self.px4.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                
            # Verify actual vehicle state transition
            if self.px4.is_offboard():
                self.node.get_logger().info(f"[{self.config.drone_id}] Confirmed OFFBOARD mode active.")
                self._log_transition(FlightState.REQUEST_ARM)
            elif self.state_timer >= self.mission.control_rate_hz * 10: # 10s timeout
                self.node.get_logger().error(
                    f"[{self.config.drone_id}] OFFBOARD mode rejected after 10s! Current nav_state={self.px4.nav_state}"
                )
                self._log_transition(FlightState.FAILSAFE)
            self.state_timer += 1

        elif self.state == FlightState.REQUEST_ARM:
            self.px4.publish_offboard_control_mode()
            self.px4.publish_trajectory_setpoint(self.mission_goal_local)
            
            # Send arm command every 0.5s
            if self.state_timer % (self.mission.control_rate_hz // 2) == 0:
                self.px4.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
                
            # Verify actual vehicle arming state
            if self.px4.is_armed():
                self.node.get_logger().info(f"[{self.config.drone_id}] Confirmed vehicle ARMED.")
                self._log_transition(FlightState.TAKEOFF)
            elif self.state_timer >= self.mission.control_rate_hz * 10: # 10s timeout
                self.node.get_logger().error(
                    f"[{self.config.drone_id}] ARMING rejected after 10s! arming_state={self.px4.arming_state}, pre_flight_checks_pass={self.px4.pre_flight_checks_pass}"
                )
                self._log_transition(FlightState.FAILSAFE)
            self.state_timer += 1

        elif self.state == FlightState.TAKEOFF:
            takeoff_target = [self.initial_pos[0], self.initial_pos[1], self.initial_pos[2] - self.mission.takeoff_altitude] # NED: Z down
            self.px4.publish_offboard_control_mode()
            self.px4.publish_trajectory_setpoint(takeoff_target)
            
            z_error = abs(self.px4.current_position[2] - takeoff_target[2])
            if z_error < 0.5:
                self.mission_goal_local = takeoff_target
                self._log_transition(FlightState.HOLD)
            elif self.state_timer >= self.mission.control_rate_hz * 35: # 35s takeoff timeout
                self.node.get_logger().error(
                    f"[{self.config.drone_id}] Takeoff timeout after 35s! Current Z={self.px4.current_position[2]:.2f}, target Z={takeoff_target[2]:.2f}, z_error={z_error:.2f}m"
                )
                self._log_transition(FlightState.FAILSAFE)
            self.state_timer += 1

        elif self.state == FlightState.WAYPOINT_NAVIGATION:
            self.state_timer += 1
            # compute temporary target by adding APF offset to mission goal
            control_target_local = [
                self.mission_goal_local[0] + self.avoidance_offset_local[0],
                self.mission_goal_local[1] + self.avoidance_offset_local[1],
                self.mission_goal_local[2] # Z stays fixed at mission_goal
            ]
            
            # evaluate distance to IMMUTABLE mission goal
            dx = self.px4.current_position[0] - self.mission_goal_local[0]
            dy = self.px4.current_position[1] - self.mission_goal_local[1]
            dist_to_goal = math.sqrt(dx*dx + dy*dy)
            
            # Progress tracking for stall detection
            if dist_to_goal < self.last_goal_dist - 0.25:
                self.last_goal_dist = dist_to_goal
                self.last_progress_sim_time = self.get_current_sim_time()

            # Calculate yaw to face the target (atan2(y_diff, x_diff))
            # Note: dx, dy above are (current - goal). We want (goal - current) -> (-dx, -dy)
            target_yaw = math.atan2(-dy, -dx)
            
            self.px4.publish_offboard_control_mode()
            self.px4.publish_trajectory_setpoint(control_target_local, yaw=target_yaw)
            
            if dist_to_goal < self.mission.goal_tolerance:
                # Goal reached within tolerance. Check dwell requirements in simulation time:
                min_dwell_sec = getattr(self.mission, 'min_waypoint_dwell_sec', 1.0)
                dwell_satisfied = (not self.require_min_dwell) or (self.elapsed_sim_time() >= min_dwell_sec) or (self.state_timer >= self.mission.min_waypoint_dwell_ticks)
                if dwell_satisfied:
                    # Goal successfully reached
                    self.avoidance_offset_local = [0.0, 0.0, 0.0]
                    self._log_transition(FlightState.HOLD)

        elif self.state == FlightState.HOLD:
            self.px4.publish_offboard_control_mode()
            self.px4.publish_trajectory_setpoint(self.mission_goal_local)
            self.state_timer += 1

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
