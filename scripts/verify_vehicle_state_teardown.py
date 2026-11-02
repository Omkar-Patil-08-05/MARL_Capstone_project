#!/usr/bin/env python3
"""
Authoritative Telemetry Teardown Safety Verifier (Phase F5.3D)
Queries live PX4 ROS 2 DDS telemetry to independently verify that:
1. Vehicle is authoritatively DISARMED (VehicleStatus.arming_state == 1).
2. Vehicle is authoritatively LANDED and at ground contact (VehicleLandDetected.landed == True and ground_contact == True).
3. Physical resting state is confirmed: speed <= 0.20 m/s and altitude near ground level.

Exit Codes:
  0: SAFE_FOR_TEARDOWN (authoritatively disarmed and landed at ground rest)
  2: VEHICLE_ARMED (critical safety fault: vehicle is still armed)
  3: VEHICLE_AIRBORNE (critical safety fault: vehicle is still airborne or moving)
  4: STATE_UNRESOLVED_OR_TIMEOUT (telemetry missing, stale, or incomplete)
"""

import sys
import time
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

try:
    from px4_msgs.msg import VehicleStatus, VehicleLandDetected, VehicleOdometry
    PX4_MSGS_AVAILABLE = True
except ImportError:
    PX4_MSGS_AVAILABLE = False


class TeardownSafetyVerifier(Node):
    def __init__(self, namespace: str = "drone_0", timeout_sec: float = 4.0):
        super().__init__('teardown_safety_verifier')
        self.namespace = namespace
        self.timeout_sec = timeout_sec

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        ns_prefix = f"/{self.namespace}" if self.namespace else ""

        self.status_sub = self.create_subscription(
            VehicleStatus, f'{ns_prefix}/fmu/out/vehicle_status', self.status_callback, qos_profile)
        self.status_v1_sub = self.create_subscription(
            VehicleStatus, f'{ns_prefix}/fmu/out/vehicle_status_v1', self.status_callback, qos_profile)
        self.land_detected_sub = self.create_subscription(
            VehicleLandDetected, f'{ns_prefix}/fmu/out/vehicle_land_detected', self.land_callback, qos_profile)
        self.odom_sub = self.create_subscription(
            VehicleOdometry, f'{ns_prefix}/fmu/out/vehicle_odometry', self.odom_callback, qos_profile)

        self.arming_state = None
        self.nav_state = None
        self.landed = None
        self.ground_contact = None
        self.pos = None
        self.vel = None

        self.last_status_time = None
        self.last_land_time = None
        self.last_odom_time = None

    def status_callback(self, msg: VehicleStatus):
        self.arming_state = int(msg.arming_state)
        self.nav_state = int(msg.nav_state)
        self.last_status_time = time.monotonic()

    def land_callback(self, msg: VehicleLandDetected):
        self.landed = bool(msg.landed)
        self.ground_contact = bool(msg.ground_contact)
        self.last_land_time = time.monotonic()

    def odom_callback(self, msg: VehicleOdometry):
        self.pos = [float(msg.position[0]), float(msg.position[1]), float(msg.position[2])]
        self.vel = [float(msg.velocity[0]), float(msg.velocity[1]), float(msg.velocity[2])]
        self.last_odom_time = time.monotonic()


def main():
    if not PX4_MSGS_AVAILABLE:
        print("[ERROR] px4_msgs is not available in environment!", file=sys.stderr)
        sys.exit(4)

    parser = argparse.ArgumentParser(description="Authoritative Telemetry Teardown Safety Verifier")
    parser.add_argument("--namespace", type=str, default="drone_0", help="PX4 namespace")
    parser.add_argument("--timeout", type=float, default=4.0, help="Query timeout in seconds")
    parser.add_argument("--max-age", type=float, default=1.5, help="Maximum allowable telemetry age in seconds")
    parser.add_argument("--max-speed", type=float, default=0.20, help="Maximum allowable resting speed in m/s")
    parser.add_argument("--max-altitude", type=float, default=0.35, help="Maximum allowable altitude deviation from ground in m")
    args = parser.parse_args()

    rclpy.init()
    node = TeardownSafetyVerifier(namespace=args.namespace, timeout_sec=args.timeout)

    start_mono = time.monotonic()
    confirmed_safe = False
    exit_code = 4

    try:
        while time.monotonic() - start_mono < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            now_mono = time.monotonic()

            # Enforce Telemetry Freshness & Coherecy:
            # 1. All 3 streams must have been received strictly AFTER this verification attempt started.
            # 2. All 3 streams must have arrived within the freshness window (now - timestamp <= max_age).
            all_received_in_session = (
                node.last_status_time is not None and node.last_status_time >= start_mono and
                node.last_land_time is not None and node.last_land_time >= start_mono and
                node.last_odom_time is not None and node.last_odom_time >= start_mono
            )

            if not all_received_in_session:
                continue

            status_age = now_mono - node.last_status_time
            land_age = now_mono - node.last_land_time
            odom_age = now_mono - node.last_odom_time

            if status_age > args.max_age or land_age > args.max_age or odom_age > args.max_age:
                # Stale telemetry detected during check; continue spinning to await fresh messages
                continue

            # 3. Multi-stream temporal coherence: The skew between streams must not exceed max_age
            timestamp_skew = (
                max(node.last_status_time, node.last_land_time, node.last_odom_time) -
                min(node.last_status_time, node.last_land_time, node.last_odom_time)
            )
            if timestamp_skew > args.max_age:
                # Streams are skewed across time; continue spinning for a coherent snapshot
                continue

            # 1. Arming state evaluation: strictly require ARMING_STATE_DISARMED (1)
            if node.arming_state != VehicleStatus.ARMING_STATE_DISARMED:
                if node.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                    print(
                        f"[CRITICAL SAFETY FAULT] Authoritative telemetry confirms vehicle is still ARMED! "
                        f"(arming_state={node.arming_state} [ARMED], age={status_age:.3f}s)",
                        file=sys.stderr
                    )
                else:
                    print(
                        f"[CRITICAL SAFETY FAULT] Authoritative telemetry reports non-disarmed arming state! "
                        f"(arming_state={node.arming_state}, expected ARMING_STATE_DISARMED=1, age={status_age:.3f}s)",
                        file=sys.stderr
                    )
                exit_code = 2
                break

            # 2. Landed & ground contact evaluation (fail-closed if airborne)
            if not node.landed or not node.ground_contact:
                print(
                    f"[CRITICAL SAFETY FAULT] Authoritative telemetry confirms vehicle is NOT safely landed! "
                    f"(landed={node.landed}, ground_contact={node.ground_contact}, age={land_age:.3f}s)",
                    file=sys.stderr
                )
                exit_code = 3
                break

            # 3. Physical resting speed evaluation (fail-closed if kinetic motion detected)
            speed = float(np.linalg.norm(node.vel))
            if speed > args.max_speed:
                print(
                    f"[CRITICAL SAFETY FAULT] Vehicle motion detected during teardown check! "
                    f"(speed={speed:.3f}m/s > {args.max_speed:.3f}m/s, age={odom_age:.3f}s)",
                    file=sys.stderr
                )
                exit_code = 3
                break

            # 4. Explicit near-ground position evaluation (fail-closed if altitude exceeds ground threshold)
            altitude_drift = abs(node.pos[2])
            if altitude_drift > args.max_altitude:
                print(
                    f"[CRITICAL SAFETY FAULT] Vehicle position violates ground-level resting constraint! "
                    f"(Local NED Z={node.pos[2]:.3f}m > threshold {args.max_altitude:.3f}m, age={odom_age:.3f}s)",
                    file=sys.stderr
                )
                exit_code = 3
                break

            # Vehicle is authoritatively verified FRESH, DISARMED, LANDED, AT REST, and AT GROUND LEVEL
            print("=" * 70)
            print("[AUTHORITATIVE TEARDOWN CHECK] PASS: SAFE FOR TEARDOWN")
            print("=" * 70)
            print(f"  Arming State:      {node.arming_state} (ARMING_STATE_DISARMED) [Fresh: {status_age:.3f}s]")
            print(f"  Landed Flag:       {node.landed} (True) [Fresh: {land_age:.3f}s]")
            print(f"  Ground Contact:    {node.ground_contact} (True) [Fresh: {land_age:.3f}s]")
            print(f"  Resting Speed:     {speed:.3f} m/s (<= {args.max_speed:.3f} m/s) [Fresh: {odom_age:.3f}s]")
            print(f"  Position Z:        {node.pos[2]:.3f} m (<= {args.max_altitude:.3f} m ground tolerance)")
            print(f"  Session Freshness: All 3 streams received within last {max(status_age, land_age, odom_age):.3f}s (max_age={args.max_age:.1f}s)")
            print(f"  Temporal Skew:     {timestamp_skew:.3f}s (<= {args.max_age:.1f}s multi-stream coherence)")
            print("=" * 70)
            exit_code = 0
            confirmed_safe = True
            break

    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not confirmed_safe and exit_code == 4:
        now_mono = time.monotonic()
        status_age_str = f"{now_mono - node.last_status_time:.3f}s" if node.last_status_time is not None else "NEVER_RECEIVED"
        land_age_str = f"{now_mono - node.last_land_time:.3f}s" if node.last_land_time is not None else "NEVER_RECEIVED"
        odom_age_str = f"{now_mono - node.last_odom_time:.3f}s" if node.last_odom_time is not None else "NEVER_RECEIVED"
        print(
            f"[ERROR] Authoritative state query timed out after {args.timeout:.1f}s without fresh, coherent telemetry!\n"
            f"  Stream Freshness Status:\n"
            f"    vehicle_status:        age={status_age_str} (max_allowed={args.max_age:.1f}s)\n"
            f"    vehicle_land_detected: age={land_age_str} (max_allowed={args.max_age:.1f}s)\n"
            f"    vehicle_odometry:      age={odom_age_str} (max_allowed={args.max_age:.1f}s)\n"
            f"  Fail-closed requirement: All 3 streams must arrive within session and have age <= {args.max_age:.1f}s.",
            file=sys.stderr
        )

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
