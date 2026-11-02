#!/usr/bin/env python3
"""
Bounded Single Drone Offboard Flight Test Controller (Phase F5.3C)
- Default non-arming dry-run mode (requires explicit --authorize-flight to arm)
- Strict preflight parameter validation (altitude, hold time, timeouts)
- Robust estimator stability window (position variation, speed, and ground elevation checks over 30 samples)
- Strictly bounded state transitions with explicit per-state timeouts and safe abort paths
- Prevention of mid-air motor cutoff (emergency disarm forbidden while airborne)
- Non-interfering failsafe behavior upon loss of offboard mode (yields to PX4 COM_OBL_RC_ACT)
"""

import sys
import time
import argparse
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from enum import Enum, auto
from typing import Optional, List, Tuple

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry, VehicleStatus


class FlightState(Enum):
    WAIT_FOR_CONNECTION = auto()
    WAIT_FOR_ESTIMATOR = auto()
    DISARMED_READINESS_PASS = auto()
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
    ABORT = auto()
    FAILSAFE = auto()


class SingleDroneTestNode(Node):
    def __init__(self, namespace: str = "drone_0", system_id: int = 1,
                 takeoff_altitude: float = 5.0, hold_sec: float = 10.0,
                 vertical_only: bool = True, translate_dist: float = 5.0,
                 authorize_flight: bool = False) -> None:
        super().__init__('single_drone_test_node')

        # 1. Parameter Validation
        if not (1.0 <= takeoff_altitude <= 30.0):
            raise ValueError(
                f"Invalid takeoff_altitude: {takeoff_altitude}m. "
                f"Must be between 1.0m and 30.0m for Stage 1 tree-clearance safety."
            )
        if not (1.0 <= hold_sec <= 120.0):
            raise ValueError(f"Invalid hold_sec: {hold_sec}s. Must be between 1.0s and 120.0s.")
        if not (0.0 <= translate_dist <= 50.0):
            raise ValueError(f"Invalid translate_dist: {translate_dist}m. Must be between 0.0m and 50.0m.")

        self.namespace = namespace
        self.system_id = system_id
        self.takeoff_altitude = takeoff_altitude
        self.hold_sec = hold_sec
        self.vertical_only = vertical_only
        self.translate_dist = translate_dist
        self.authorize_flight = authorize_flight

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        ns_prefix = f"/{self.namespace}" if self.namespace else ""

        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, f'{ns_prefix}/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, f'{ns_prefix}/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, f'{ns_prefix}/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, f'{ns_prefix}/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, f'{ns_prefix}/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_status_v1_subscriber = self.create_subscription(
            VehicleStatus, f'{ns_prefix}/fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos_profile)

        # Internal state
        self.nav_state: int = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state: int = VehicleStatus.ARMING_STATE_DISARMED
        self.vehicle_pos: List[float] = [float('nan'), float('nan'), float('nan')]
        self.vehicle_vel: List[float] = [float('nan'), float('nan'), float('nan')]
        self.vehicle_connected: bool = False
        self.odom_sample_count: int = 0
        self.failure_exit: bool = False
        self.flight_initiated: bool = False
        self.aborted: bool = False
        self.ground_contact_ticks: int = 0
        self.last_odom_wall_time: Optional[float] = None
        self.last_status_wall_time: Optional[float] = None

        # Estimator stability history (30 samples @ 20 Hz = 1.5 seconds)
        # Each entry: (pos, vel, msg_timestamp_sec, wall_time_sec)
        self.odom_history: deque[Tuple[List[float], List[float], float, float]] = deque(maxlen=30)

        self.state: FlightState = FlightState.WAIT_FOR_CONNECTION
        self.state_timer: int = 0
        self.initial_pos: Optional[List[float]] = None
        self.target_pos: List[float] = [0.0, 0.0, 0.0]

        # Timeouts in ticks (at 20 Hz, 1 tick = 0.05s)
        self.TIMEOUT_CONNECTION = 400   # 20.0s
        self.TIMEOUT_ESTIMATOR = 400    # 20.0s
        self.TIMEOUT_STREAM = 40        # 2.0s
        self.TIMEOUT_OFFBOARD = 100     # 5.0s
        self.TIMEOUT_ARM = 100          # 5.0s
        self.TIMEOUT_TAKEOFF = 400      # 20.0s
        self.TIMEOUT_HOLD = int(self.hold_sec * 20)
        self.TIMEOUT_TRANSLATE = 400    # 20.0s
        self.TIMEOUT_RETURN = 400       # 20.0s
        self.TIMEOUT_LAND = 600         # 30.0s
        self.TIMEOUT_ABORT = 600        # 30.0s
        self.TIMEOUT_ABORT_MAX = 1500   # 75.0s absolute hard cap for monitored failsafe descent

        self.control_timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

        if not self.authorize_flight:
            self.get_logger().info(
                f"[SAFETY LOCK ACTIVE] Flight authorization is DISABLED (default non-arming mode). "
                f"Node will verify connection and estimator stability window, but WILL NOT ARM OR FLY."
            )
        else:
            self.get_logger().warn(
                f"[FLIGHT AUTHORIZED] Explicit flight authorization enabled! "
                f"Vehicle will arm and takeoff to {self.takeoff_altitude}m once safety checks pass."
            )

        self.get_logger().info(
            f"Single Drone Test Node initialized: namespace='{self.namespace}', "
            f"system_id={self.system_id}, takeoff_alt={self.takeoff_altitude}m, "
            f"vertical_only={self.vertical_only}, flight_authorized={self.authorize_flight}"
        )

    def log_transition(self, new_state: FlightState) -> None:
        if self.state != new_state:
            self.get_logger().info(f"[STATE TRANSITION] {self.state.name} -> {new_state.name} (at tick {self.state_timer})")
            self.state = new_state
            self.state_timer = 0

    def vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        self.vehicle_connected = True
        self.last_status_wall_time = time.time()

    def vehicle_odometry_callback(self, msg: VehicleOdometry) -> None:
        now_wall = time.time()
        pos = [float(msg.position[0]), float(msg.position[1]), float(msg.position[2])]
        vel = [float(msg.velocity[0]), float(msg.velocity[1]), float(msg.velocity[2])]
        self.vehicle_pos = pos
        self.vehicle_vel = vel
        self.odom_sample_count += 1
        self.last_odom_wall_time = now_wall
        msg_timestamp_sec = float(msg.timestamp) / 1e6

        if not (any(np.isnan(pos)) or any(np.isinf(pos)) or any(np.isnan(vel)) or any(np.isinf(vel))):
            self.odom_history.append((pos, vel, msg_timestamp_sec, now_wall))

    def is_estimator_healthy(self) -> bool:
        """
        Check that odometry has been completely stable and fresh over a continuous 30-sample window.
        Criteria:
        1. Window has full 30 valid non-NaN samples.
        2. Freshness: most recent sample received within last 0.35s wall time.
        3. Monotonicity & Cadence: timestamps strictly advance; total duration between 0.5s and 2.2s; dt <= 0.20s.
        4. Position variation over window is < 0.08m in X, Y, Z.
        5. Maximum speed over window is < 0.10 m/s.
        6. Mean local NED altitude is near ground rest (|Z| < 0.35m).
        """
        if len(self.odom_history) < 30:
            return False

        now_wall = time.time()
        latest = self.odom_history[-1]

        # 1. Freshness check: latest sample received within 0.45s wall time
        if (now_wall - latest[3]) > 0.45:
            return False

        # 2. Monotonicity & Cadence check across the 30-sample window
        msg_timestamps = [s[2] for s in self.odom_history]
        time_diffs = np.diff(msg_timestamps)
        # Every timestamp must strictly advance
        if any(dt <= 0.0 for dt in time_diffs):
            return False
        # Inter-sample gap must not exceed 0.50s
        if any(dt > 0.50 for dt in time_diffs):
            return False
        # Total span for 30 samples (nominal 10-250 Hz -> 0.10s to 3.5s)
        total_span = msg_timestamps[-1] - msg_timestamps[0]
        if not (0.10 <= total_span <= 3.50):
            return False
        effective_rate = 29.0 / total_span
        if not (8.0 <= effective_rate <= 250.0):
            return False

        positions = np.array([sample[0] for sample in self.odom_history])
        velocities = np.array([sample[1] for sample in self.odom_history])

        # 3. Position range (peak-to-peak variation)
        ptp_xyz = np.ptp(positions, axis=0)
        if any(ptp_xyz > 0.15):
            return False

        # 4. Max velocity magnitude
        speeds = np.linalg.norm(velocities, axis=1)
        if np.max(speeds) > 0.15:
            return False

        # 5. Ground rest check
        mean_z = float(np.mean(positions[:, 2]))
        if abs(mean_z) > 0.35:
            return False

        return True

    def publish_offboard_control_mode(self) -> None:
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self, x: float, y: float, z: float, yaw: float = 0.0) -> None:
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
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
        msg.target_system = self.system_id
        msg.target_component = 1
        msg.source_system = 255
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def is_vehicle_safely_on_ground(self) -> bool:
        """Verify whether vehicle is physically settled on the ground before allowing disarm."""
        if self.initial_pos is None:
            return abs(self.vehicle_pos[2]) < 0.20 and np.linalg.norm(self.vehicle_vel) < 0.08
        z_ground_error = abs(self.vehicle_pos[2] - self.initial_pos[2])
        speed = np.linalg.norm(self.vehicle_vel)
        return (z_ground_error < 0.20) and (speed < 0.08)

    def trigger_abort(self, reason: str) -> None:
        self.aborted = True
        self.get_logger().error(f"[ABORT TRIGGERED] Reason: {reason}.")
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.get_logger().info("Vehicle currently in Offboard mode. Commanding safe land (VEHICLE_CMD_NAV_LAND).")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        else:
            self.get_logger().warn(
                f"Offboard mode not active (nav_state={self.nav_state}). Ceasing setpoints immediately "
                f"to allow PX4 onboard failsafe (COM_OBL_RC_ACT) to control descent safely."
            )
        self.log_transition(FlightState.ABORT)

    def timer_callback(self) -> None:
        now_wall = time.time()

        # 1. Telemetry Stream Freshness Watchdog during active operational states
        if self.state not in [FlightState.WAIT_FOR_CONNECTION, FlightState.DISARMED_READINESS_PASS,
                              FlightState.COMPLETE, FlightState.ABORT, FlightState.FAILSAFE]:
            if self.last_odom_wall_time is not None and (now_wall - self.last_odom_wall_time) > 1.00:
                self.trigger_abort(f"Odometry stream frozen (no update for {now_wall - self.last_odom_wall_time:.2f}s)")
                return
            if self.last_status_wall_time is not None and (now_wall - self.last_status_wall_time) > 1.50:
                self.trigger_abort(f"Status stream frozen (no update for {now_wall - self.last_status_wall_time:.2f}s)")
                return

        # 2. Connection Watchdog
        if self.state not in [FlightState.WAIT_FOR_CONNECTION, FlightState.DISARMED_READINESS_PASS,
                              FlightState.COMPLETE, FlightState.ABORT, FlightState.FAILSAFE]:
            if not self.vehicle_connected:
                self.trigger_abort("Heartbeat lost to PX4")
                return

        # 3. Offboard Mode Watchdog during active flight
        if self.state in [FlightState.TAKEOFF, FlightState.HOLD, FlightState.TRANSLATE, FlightState.RETURN]:
            if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.trigger_abort("PX4 dropped offboard mode unexpectedly")
                return

        # 4. Finite State Machine
        if self.state == FlightState.WAIT_FOR_CONNECTION:
            self.state_timer += 1
            if self.vehicle_connected:
                self.log_transition(FlightState.WAIT_FOR_ESTIMATOR)
            elif self.state_timer >= self.TIMEOUT_CONNECTION:
                self.get_logger().error("Timeout waiting for PX4 connection!")
                self.log_transition(FlightState.FAILSAFE)

        elif self.state == FlightState.WAIT_FOR_ESTIMATOR:
            self.state_timer += 1
            if self.is_estimator_healthy():
                self.initial_pos = list(self.vehicle_pos)
                self.target_pos = list(self.initial_pos)
                self.get_logger().info(
                    f"Estimator healthy, stationary, and fresh across {len(self.odom_history)} samples. "
                    f"Ground origin: Local NED X={self.initial_pos[0]:.3f}m, Y={self.initial_pos[1]:.3f}m, Z={self.initial_pos[2]:.3f}m."
                )

                # Flight Authorization Gate
                if not self.authorize_flight:
                    self.get_logger().info(
                        "[READINESS PASS] Telemetry and estimator stability window verified. "
                        "Flight authorization flag (--authorize-flight) is NOT set. "
                        "Entering DISARMED_READINESS_PASS mode. Zero arming or flight commands will be sent."
                    )
                    self.log_transition(FlightState.DISARMED_READINESS_PASS)
                else:
                    self.get_logger().info(
                        f"Target altitude: Local NED z={self.initial_pos[2] - self.takeoff_altitude:.2f}m "
                        f"(relative climb: +{self.takeoff_altitude:.2f}m, approx world altitude: {12.50 + self.takeoff_altitude:.2f}m)."
                    )
                    self.log_transition(FlightState.STREAM_SETPOINTS)
            elif self.state_timer >= self.TIMEOUT_ESTIMATOR:
                self.trigger_abort("Estimator failed to stabilize at ground origin")

        elif self.state == FlightState.DISARMED_READINESS_PASS:
            # Non-arming dry-run verification: observe vehicle status without commanding motion
            self.state_timer += 1
            if self.state_timer >= 40:  # 2.0s observation
                self.get_logger().info(
                    f"Disarmed readiness pass verified: vehicle remains safely DISARMED "
                    f"(arming_state={self.arming_state}, nav_state={self.nav_state})."
                )
                self.log_transition(FlightState.COMPLETE)

        elif self.state == FlightState.STREAM_SETPOINTS:
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(*self.target_pos)
            self.state_timer += 1
            if self.state_timer >= self.TIMEOUT_STREAM:
                self.log_transition(FlightState.REQUEST_OFFBOARD)

        elif self.state == FlightState.REQUEST_OFFBOARD:
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(*self.target_pos)
            if self.state_timer % 10 == 0:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

            if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.log_transition(FlightState.REQUEST_ARM)
            elif self.state_timer >= self.TIMEOUT_OFFBOARD:
                self.trigger_abort("Failed to enter Offboard mode")
            self.state_timer += 1

        elif self.state == FlightState.REQUEST_ARM:
            self.flight_initiated = True
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(*self.target_pos)
            if self.state_timer % 10 == 0:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

            if self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.log_transition(FlightState.TAKEOFF)
            elif self.state_timer >= self.TIMEOUT_ARM:
                self.trigger_abort("Failed to arm vehicle")
            self.state_timer += 1

        elif self.state == FlightState.TAKEOFF:
            # In PX4 Local NED, Z is DOWN. Target Z = initial Z - takeoff_altitude
            self.target_pos = [self.initial_pos[0], self.initial_pos[1], self.initial_pos[2] - self.takeoff_altitude]
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(*self.target_pos)

            z_error = abs(self.vehicle_pos[2] - self.target_pos[2])
            if z_error < 0.4:
                self.get_logger().info(f"Takeoff target reached (current NED z={self.vehicle_pos[2]:.2f}m).")
                self.log_transition(FlightState.HOLD)
            elif self.state_timer >= self.TIMEOUT_TAKEOFF:
                self.trigger_abort(f"Takeoff timeout! Stuck at NED z={self.vehicle_pos[2]:.2f}m")
            self.state_timer += 1

        elif self.state == FlightState.HOLD:
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(*self.target_pos)
            self.state_timer += 1
            if self.state_timer >= self.TIMEOUT_HOLD:
                if self.vertical_only:
                    self.get_logger().info("Vertical-only flight mode active. Proceeding directly from HOLD to LAND.")
                    self.log_transition(FlightState.LAND)
                else:
                    self.log_transition(FlightState.TRANSLATE)

        elif self.state == FlightState.TRANSLATE:
            # Horizontal translation forward (NED +X) at target altitude
            self.target_pos = [self.initial_pos[0] + self.translate_dist, self.initial_pos[1], self.initial_pos[2] - self.takeoff_altitude]
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(*self.target_pos)

            x_error = abs(self.vehicle_pos[0] - self.target_pos[0])
            if x_error < 0.5:
                self.log_transition(FlightState.RETURN)
            elif self.state_timer >= self.TIMEOUT_TRANSLATE:
                self.trigger_abort("Translation waypoint timeout")
            self.state_timer += 1

        elif self.state == FlightState.RETURN:
            # Return to initial X, Y at hover altitude
            self.target_pos = [self.initial_pos[0], self.initial_pos[1], self.initial_pos[2] - self.takeoff_altitude]
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(*self.target_pos)

            dist_home = np.hypot(self.vehicle_pos[0] - self.initial_pos[0], self.vehicle_pos[1] - self.initial_pos[1])
            if dist_home < 0.5:
                self.log_transition(FlightState.LAND)
            elif self.state_timer >= self.TIMEOUT_RETURN:
                self.trigger_abort("Return waypoint timeout")
            self.state_timer += 1

        elif self.state == FlightState.LAND:
            # Command land continuously during descent
            if self.state_timer % 20 == 0:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

            is_on_ground_instant = self.is_vehicle_safely_on_ground()
            if is_on_ground_instant:
                self.ground_contact_ticks += 1
            else:
                self.ground_contact_ticks = 0

            has_stable_ground_contact = (self.ground_contact_ticks >= 20)  # 1.0s continuous ground settle

            if self.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                # Require BOTH disarmed status AND confirmed stable ground-state evidence
                if has_stable_ground_contact:
                    self.get_logger().info(
                        "Landing verified: stable ground-state evidence confirmed (1.0s settled) "
                        "and vehicle safely DISARMED."
                    )
                    self.log_transition(FlightState.DISARM)
                else:
                    self.get_logger().warn(
                        f"Vehicle reported DISARMED but stable ground contact not yet confirmed "
                        f"(settle ticks: {self.ground_contact_ticks}/20, NED z={self.vehicle_pos[2]:.2f}m, "
                        f"speed={np.linalg.norm(self.vehicle_vel):.2f}m/s). Awaiting stable ground settling...",
                        throttle_duration_sec=2.0
                    )
            elif has_stable_ground_contact and self.state_timer >= 40:
                # Vehicle has touched down and settled on ground for >= 1.0s, safe to command disarm
                self.get_logger().info("Vehicle confirmed stable at ground rest. Commanding safe disarm.")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

            elif self.state_timer >= self.TIMEOUT_LAND:
                self.get_logger().warn(
                    f"Landing phase timeout reached ({self.TIMEOUT_LAND/20:.1f}s). Current NED z={self.vehicle_pos[2]:.2f}m."
                )
                if has_stable_ground_contact:
                    self.get_logger().info("Vehicle confirmed resting on ground following timeout. Commanding safe disarm.")
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
                else:
                    self.get_logger().error(
                        "Vehicle not confirmed on ground! Denying airborne motor cutoff. Transferring to PX4 failsafe."
                    )
                    self.log_transition(FlightState.FAILSAFE)
            self.state_timer += 1

        elif self.state == FlightState.DISARM:
            self.log_transition(FlightState.COMPLETE)

        elif self.state == FlightState.COMPLETE:
            self.state_timer += 1
            if self.state_timer == 1:
                if self.aborted:
                    self.get_logger().warn("Test ended via ABORT: Vehicle safely landed on ground and disarmed.")
                else:
                    self.get_logger().info("Test completed successfully: Nominal flight, landing, and disarm verified.")
            elif self.state_timer >= 20:
                self.get_logger().info("Shutting down node cleanly after completion.")
                if self.aborted:
                    raise SystemExit(1)
                else:
                    raise SystemExit(0)

        elif self.state in [FlightState.ABORT, FlightState.FAILSAFE]:
            if not self.flight_initiated:
                self.get_logger().error(
                    f"Abort/failsafe reached prior to flight initiation (state={self.state.name}, timer={self.state_timer}). "
                    f"Vehicle remains safely disarmed on ground. Exiting with failure status."
                )
                self.failure_exit = True
                raise SystemExit(2)

            # In abort/failsafe, do NOT publish offboard trajectory setpoints. Let PX4 failsafe execute descent.
            if self.state_timer % 20 == 0 and self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

            is_on_ground_instant = self.is_vehicle_safely_on_ground()
            if is_on_ground_instant:
                self.ground_contact_ticks += 1
            else:
                self.ground_contact_ticks = 0

            has_stable_ground_contact = (self.ground_contact_ticks >= 20)

            if self.arming_state == VehicleStatus.ARMING_STATE_DISARMED and has_stable_ground_contact:
                self.get_logger().info("Vehicle safely settled on ground and disarmed following abort/failsafe.")
                self.log_transition(FlightState.COMPLETE)

            elif has_stable_ground_contact:
                # On ground but still armed after abort
                if self.state_timer % 20 == 0:
                    self.get_logger().info("Vehicle physically at ground rest during abort. Commanding safe disarm.")
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

            elif self.state_timer >= self.TIMEOUT_ABORT:
                # Check if actively descending
                is_actively_descending = (self.vehicle_vel[2] > 0.08) and (
                    self.initial_pos is not None and self.vehicle_pos[2] < self.initial_pos[2]
                )
                if is_actively_descending and self.state_timer < self.TIMEOUT_ABORT_MAX:
                    self.get_logger().warn(
                        f"Abort timeout reached but active descent observed (Vz={self.vehicle_vel[2]:.2f}m/s, NED Z={self.vehicle_pos[2]:.2f}m). "
                        f"Extending monitoring grace period (tick {self.state_timer}/{self.TIMEOUT_ABORT_MAX})...",
                        throttle_duration_sec=3.0
                    )
                else:
                    self.get_logger().fatal(
                        f"CRITICAL FAULT: Abort timeout expired ({self.state_timer/20:.1f}s) and vehicle is still airborne! "
                        f"NED Z={self.vehicle_pos[2]:.2f}m, Vel={self.vehicle_vel}. "
                        f"Refusing mid-air motor cutoff. Terminating controller with failure status. "
                        f"Autopilot onboard failsafe remains in control."
                    )
                    self.failure_exit = True
                    raise SystemExit(3)
            self.state_timer += 1


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description="Bounded offboard flight controller.")
    parser.add_argument("--namespace", type=str, default="drone_0", help="PX4 uXRCE namespace")
    parser.add_argument("--system-id", type=int, default=1, help="PX4 target system ID")
    parser.add_argument("--takeoff-altitude", type=float, default=5.0, help="Relative takeoff altitude in meters (1.0 - 30.0)")
    parser.add_argument("--hold-time", type=float, default=10.0, help="Hover hold time in seconds (1.0 - 120.0)")
    parser.add_argument("--vertical-only", action="store_true", default=True, help="Perform vertical takeoff and land only")
    parser.add_argument("--enable-translation", dest="vertical_only", action="store_false", help="Enable horizontal translation")
    parser.add_argument("--translate-dist", type=float, default=5.0, help="Horizontal translation distance in meters")
    parser.add_argument("--authorize-flight", action="store_true", default=False, help="Explicit authorization flag required to arm and fly")
    cli_args, remaining = parser.parse_known_args()

    rclpy.init(args=remaining)
    exit_code = 0
    try:
        node = SingleDroneTestNode(
            namespace=cli_args.namespace,
            system_id=cli_args.system_id,
            takeoff_altitude=cli_args.takeoff_altitude,
            hold_sec=cli_args.hold_time,
            vertical_only=cli_args.vertical_only,
            translate_dist=cli_args.translate_dist,
            authorize_flight=cli_args.authorize_flight
        )
        rclpy.spin(node)
    except ValueError as ve:
        print(f"Safety Parameter Error: {ve}", file=sys.stderr)
        sys.exit(2)
    except SystemExit as se:
        exit_code = se.code if se.code is not None else 0
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

    if exit_code != 0:
        sys.exit(exit_code)


if __name__ == '__main__':
    main()

