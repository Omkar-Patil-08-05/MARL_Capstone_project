#!/usr/bin/env python3
from __future__ import annotations
"""
Non-Flight PX4 Readiness & Connectivity Verification Node (Phase F5.3B/D)
Validates that:
1. PX4 instance 0 establishes connection via MicroXRCEAgent.
2. VehicleStatus confirms the vehicle is strictly DISARMED (zero flight/arming commands issued).
3. VehicleOdometry streams valid, non-NaN estimates within ground-rest physical tolerances:
   - Position: no NaN/Inf, |Z| <= 0.35m
   - Velocity: no NaN/Inf, resting speed <= 0.15m/s
   - Freshness: monotonic timestamp advancement, stream stall detection <= 1.5s
4. Thermal camera topic continues streaming concurrently.
5. Immediate termination on any fatal validation error.
Note: Autonomous offboard-loss failsafe parameter compliance (COM_OBL_RC_ACT=4, COM_OF_LOSS_T=1.0000)
is verified and enforced directly at the launcher level via px4-param.
"""

import sys
import time
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image

try:
    from px4_msgs.msg import VehicleStatus, VehicleOdometry
    PX4_MSGS_AVAILABLE = True
except ImportError:
    PX4_MSGS_AVAILABLE = False


class PX4ReadinessVerifier(Node):
    def __init__(self, namespace: str = "drone_0", required_samples: int = 15, timeout_sec: float = 25.0):
        super().__init__('px4_readiness_verifier')
        self.namespace = namespace
        self.required_samples = required_samples
        self.timeout_sec = timeout_sec

        self.status_count = 0
        self.odometry_count = 0
        self.thermal_count = 0

        self.last_arming_state = None
        self.last_nav_state = None
        self.last_pos = None
        self.last_vel = None
        self.last_odom_timestamp = None
        self.last_odom_wall_time = None
        self.last_status_wall_time = None

        self.fatal_error = False
        self.validation_errors = []

        qos_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        ns_prefix = f"/{self.namespace}" if self.namespace else ""

        self.status_sub = self.create_subscription(
            VehicleStatus,
            f"{ns_prefix}/fmu/out/vehicle_status",
            self.status_callback,
            qos_px4
        )
        self.status_v1_sub = self.create_subscription(
            VehicleStatus,
            f"{ns_prefix}/fmu/out/vehicle_status_v1",
            self.status_callback,
            qos_px4
        )

        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            f"{ns_prefix}/fmu/out/vehicle_odometry",
            self.odom_callback,
            qos_px4
        )

        self.thermal_sub = self.create_subscription(
            Image,
            f"{ns_prefix}/thermal/image_raw",
            self.thermal_callback,
            qos_sensor
        )

        self.get_logger().info(f"PX4 Readiness Verifier initialized for namespace: '{self.namespace}'")
        self.get_logger().info(f"Target: {self.required_samples} samples per topic within {self.timeout_sec}s")

    def status_callback(self, msg: VehicleStatus):
        now = time.time()
        self.status_count += 1
        self.last_arming_state = msg.arming_state
        self.last_nav_state = msg.nav_state
        self.last_status_wall_time = now

        # Strict safety check: vehicle MUST remain disarmed
        if msg.arming_state != VehicleStatus.ARMING_STATE_DISARMED:
            err = f"CRITICAL SAFETY VIOLATION: Vehicle is NOT DISARMED! arming_state={msg.arming_state}"
            self.get_logger().error(err)
            self.validation_errors.append(err)
            self.fatal_error = True

    def odom_callback(self, msg: VehicleOdometry):
        now = time.time()
        self.odometry_count += 1
        pos = [float(msg.position[0]), float(msg.position[1]), float(msg.position[2])]
        vel = [float(msg.velocity[0]), float(msg.velocity[1]), float(msg.velocity[2])]

        # Freshness: message timestamp must strictly advance
        msg_ts = int(msg.timestamp)
        if self.last_odom_timestamp is not None:
            if msg_ts <= self.last_odom_timestamp:
                err = f"Telemetry Stagnation: Odometry timestamp did not advance: {msg_ts} <= {self.last_odom_timestamp}"
                self.get_logger().error(err)
                self.validation_errors.append(err)
                self.fatal_error = True
        self.last_odom_timestamp = msg_ts
        self.last_odom_wall_time = now

        # Validate Position
        if any(np.isnan(pos)) or any(np.isinf(pos)):
            err = f"FATAL: Invalid NaN/Inf in odometry position: {pos}"
            self.get_logger().error(err)
            self.validation_errors.append(err)
            self.fatal_error = True
            return

        # Validate Velocity
        if any(np.isnan(vel)) or any(np.isinf(vel)):
            err = f"FATAL: Invalid NaN/Inf in odometry velocity: {vel}"
            self.get_logger().error(err)
            self.validation_errors.append(err)
            self.fatal_error = True
            return

        # Ground Rest Physical Sanity Checks
        speed = float(np.linalg.norm(vel))
        if speed > 0.15:
            err = f"FATAL: Excessive resting velocity: speed={speed:.3f}m/s > 0.15m/s"
            self.get_logger().error(err)
            self.validation_errors.append(err)
            self.fatal_error = True
            return

        if abs(pos[2]) > 0.35:
            err = f"FATAL: Resting altitude out of bounds: Z={pos[2]:.3f}m (|Z| > 0.35m)"
            self.get_logger().error(err)
            self.validation_errors.append(err)
            self.fatal_error = True
            return

        self.last_pos = pos
        self.last_vel = vel

    def thermal_callback(self, msg: Image):
        self.thermal_count += 1


def main():
    if not PX4_MSGS_AVAILABLE:
        print("[ERROR] px4_msgs ROS 2 package is not available in environment!", file=sys.stderr)
        sys.exit(1)

    parser = argparse.ArgumentParser(description="Verify PX4 SITL connectivity, odometry, and disarmed status.")
    parser.add_argument("--namespace", type=str, default="drone_0", help="PX4 uXRCE namespace")
    parser.add_argument("--samples", type=int, default=15, help="Number of telemetry samples required")
    parser.add_argument("--timeout", type=float, default=25.0, help="Readiness timeout in seconds")
    args = parser.parse_args()

    rclpy.init()
    node = PX4ReadinessVerifier(
        namespace=args.namespace,
        required_samples=args.samples,
        timeout_sec=args.timeout
    )

    start_time = time.time()
    try:
        while rclpy.ok():
            if node.fatal_error:
                node.get_logger().error("Aborting verification immediately due to fatal telemetry error!")
                break

            # Freshness / stream stall detection after initial samples arrive
            now = time.time()
            if node.odometry_count > 0 and node.last_odom_wall_time is not None:
                if (now - node.last_odom_wall_time) > 1.5:
                    err = f"FATAL: Odometry stream stalled for {now - node.last_odom_wall_time:.2f}s > 1.5s!"
                    node.get_logger().error(err)
                    node.validation_errors.append(err)
                    node.fatal_error = True
                    break

            elapsed = now - start_time
            if elapsed > node.timeout_sec:
                node.get_logger().warn(f"Readiness check timed out after {node.timeout_sec}s.")
                break

            # Check completion criteria: sufficient samples across all channels
            if (node.status_count >= node.required_samples and
                node.odometry_count >= node.required_samples and
                node.thermal_count >= 5):
                break

            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    # Safety and Acceptance Evaluation
    failure_reasons = []

    if node.status_count < node.required_samples:
        failure_reasons.append(
            f"Insufficient VehicleStatus messages: received {node.status_count}/{node.required_samples}"
        )

    if node.odometry_count < node.required_samples:
        failure_reasons.append(
            f"Insufficient VehicleOdometry messages: received {node.odometry_count}/{node.required_samples}"
        )

    if node.thermal_count < 5:
        failure_reasons.append(
            f"Insufficient Thermal Camera frames: received {node.thermal_count}/5 during PX4 startup"
        )

    if node.last_arming_state != VehicleStatus.ARMING_STATE_DISARMED:
        failure_reasons.append(
            f"Vehicle state is NOT DISARMED: arming_state={node.last_arming_state}"
        )

    if node.validation_errors:
        failure_reasons.extend(node.validation_errors)

    passed = (len(failure_reasons) == 0)

    print("\n" + "=" * 70)
    print("PX4 NON-FLIGHT READINESS VERIFICATION REPORT (PHASE F5.3B)")
    print("=" * 70)
    print(f"Namespace:             /{args.namespace}")
    print(f"VehicleStatus Count:   {node.status_count} (Arming State: DISARMED confirmed: {node.last_arming_state == VehicleStatus.ARMING_STATE_DISARMED})")
    print(f"Odometry Sample Count: {node.odometry_count}")
    if node.last_pos is not None:
        print(f"Last Position (NED):   X={node.last_pos[0]:.3f}m, Y={node.last_pos[1]:.3f}m, Z={node.last_pos[2]:.3f}m")
    if node.last_vel is not None:
        print(f"Last Velocity (NED):   Vx={node.last_vel[0]:.3f}m/s, Vy={node.last_vel[1]:.3f}m/s, Vz={node.last_vel[2]:.3f}m/s")
    print(f"Thermal Frames Count:  {node.thermal_count} (Stream active during PX4 initialization)")
    print("=" * 70)

    if passed:
        print("[ACCEPTANCE RESULT] PASS — PX4 startup verified. Zero flight commands issued. Vehicle is safely DISARMED.")
        print("=" * 70)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    else:
        print("[ACCEPTANCE RESULT] FAIL — Readiness criteria not met:")
        for r in failure_reasons:
            print(f"  * {r}")
        print("=" * 70)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)


if __name__ == '__main__':
    main()
