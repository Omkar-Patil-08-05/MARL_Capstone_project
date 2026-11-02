#!/usr/bin/env python3
"""
Simulated Telemetry Test Suite for Landing Completion & Disarm Logic (Phase F5.3D Gate 3)
Verifies:
1. Nominal Touchdown & Disarm: Landing completion declared only after sustained stable ground settle (>=20 ticks) AND disarmed status.
2. Bouncing: Touchdown interrupted by bounce resets ground-contact timer and prevents premature landing completion.
3. Stale Odometry: Stalled odometry during landing triggers telemetry freshness watchdog and aborts.
4. Airborne Disarm: Disarmed status reported while airborne (Z = -5.0m) strictly prevents landing completion.
"""

import sys
import time
import numpy as np

import rclpy
from rclpy.node import Node
from single_drone_test.flight_test import SingleDroneTestNode, FlightState
from px4_msgs.msg import VehicleStatus, VehicleOdometry


def run_tests():
    rclpy.init()
    passed_cases = []
    failed_cases = []

    print("=" * 70)
    print("LANDING COMPLETION & DISARM LOGIC TELEMETRY VERIFICATION (GATE 3)")
    print("=" * 70)

    # -------------------------------------------------------------------------
    # Case 1: Nominal Touchdown and Disarm
    # -------------------------------------------------------------------------
    print("\n[Case 1/4] Testing Nominal Touchdown and Disarm Verification...")
    try:
        node1 = SingleDroneTestNode(namespace="drone_0", authorize_flight=False)
        node1.initial_pos = [0.0, 0.0, 0.0]
        node1.vehicle_pos = [0.0, 0.0, 0.0]
        node1.vehicle_vel = [0.01, 0.01, 0.00]
        node1.vehicle_connected = True
        node1.flight_initiated = True
        node1.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        node1.last_odom_wall_time = time.time()
        node1.last_status_wall_time = time.time()
        node1.state = FlightState.LAND

        # Run 10 ticks: vehicle is on ground, but ticks < 20
        for _ in range(10):
            node1.last_odom_wall_time = time.time()
            node1.last_status_wall_time = time.time()
            node1.timer_callback()

        assert node1.state == FlightState.LAND, f"Premature transition at tick 10! State: {node1.state}"
        assert node1.ground_contact_ticks == 10, f"Expected 10 ticks, got {node1.ground_contact_ticks}"

        # Run remaining 10 ticks to reach 20 ticks (1.0s settled)
        for _ in range(10):
            node1.last_odom_wall_time = time.time()
            node1.last_status_wall_time = time.time()
            node1.timer_callback()

        # At tick 20, with ARMING_STATE_DISARMED and stable ground contact, should transition to DISARM -> COMPLETE
        assert node1.state in [FlightState.DISARM, FlightState.COMPLETE], f"Failed to complete landing! State: {node1.state}"
        print("  -> PASSED: Sustained 20-tick ground settle required before declaring landing complete.")
        passed_cases.append("Case 1: Nominal Touchdown and Disarm")
        node1.destroy_node()
    except Exception as e:
        print(f"  -> FAILED Case 1: {e}")
        failed_cases.append(("Case 1", str(e)))

    # -------------------------------------------------------------------------
    # Case 2: Bouncing on Touchdown
    # -------------------------------------------------------------------------
    print("\n[Case 2/4] Testing Bouncing on Touchdown (Timer Reset Verification)...")
    try:
        node2 = SingleDroneTestNode(namespace="drone_0", authorize_flight=False)
        node2.initial_pos = [0.0, 0.0, 0.0]
        node2.vehicle_pos = [0.0, 0.0, 0.0]
        node2.vehicle_vel = [0.01, 0.01, 0.00]
        node2.vehicle_connected = True
        node2.flight_initiated = True
        node2.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        node2.last_odom_wall_time = time.time()
        node2.last_status_wall_time = time.time()
        node2.state = FlightState.LAND

        # Initial touchdown for 6 ticks
        for _ in range(6):
            node2.last_odom_wall_time = time.time()
            node2.last_status_wall_time = time.time()
            node2.timer_callback()
        assert node2.ground_contact_ticks == 6, f"Expected 6 ticks, got {node2.ground_contact_ticks}"

        # Simulate bounce: altitude rises to -0.40m (40cm above ground) and speed = 0.30m/s
        node2.vehicle_pos = [0.0, 0.0, -0.40]
        node2.vehicle_vel = [0.10, 0.10, -0.25]
        node2.last_odom_wall_time = time.time()
        node2.last_status_wall_time = time.time()
        node2.timer_callback()

        # Counter must be reset to 0!
        assert node2.ground_contact_ticks == 0, f"Bounce failed to reset counter! Counter: {node2.ground_contact_ticks}"
        assert node2.state == FlightState.LAND, f"State transitioned unexpectedly after bounce! State: {node2.state}"

        # Re-contact ground for 8 ticks (total 8 < 20)
        node2.vehicle_pos = [0.0, 0.0, 0.02]
        node2.vehicle_vel = [0.01, 0.01, 0.00]
        for _ in range(8):
            node2.last_odom_wall_time = time.time()
            node2.last_status_wall_time = time.time()
            node2.timer_callback()

        assert node2.ground_contact_ticks == 8
        assert node2.state == FlightState.LAND, f"Prematurely declared complete after bounce! State: {node2.state}"
        print("  -> PASSED: Bounce immediately resets ground-contact timer; landing blocked.")
        passed_cases.append("Case 2: Bouncing on Touchdown")
        node2.destroy_node()
    except Exception as e:
        print(f"  -> FAILED Case 2: {e}")
        failed_cases.append(("Case 2", str(e)))

    # -------------------------------------------------------------------------
    # Case 3: Stale Odometry during Landing
    # -------------------------------------------------------------------------
    print("\n[Case 3/4] Testing Stale Odometry Stream Rejection during Landing...")
    try:
        node3 = SingleDroneTestNode(namespace="drone_0", authorize_flight=False)
        node3.initial_pos = [0.0, 0.0, 0.0]
        node3.vehicle_pos = [0.0, 0.0, -1.0]  # Descending at 1m altitude
        node3.vehicle_vel = [0.0, 0.0, 0.40]  # Downward velocity
        node3.vehicle_connected = True
        node3.flight_initiated = True
        node3.state = FlightState.LAND

        # Stale odometry: last odom update was 1.2s ago (> 1.0s threshold)
        node3.last_odom_wall_time = time.time() - 1.20
        node3.last_status_wall_time = time.time()

        # timer_callback must detect frozen odometry and trigger abort
        node3.timer_callback()

        assert node3.state == FlightState.ABORT, f"Failed to abort on stale odometry! State: {node3.state}"
        print("  -> PASSED: Stale odometry (>1.0s) immediately detected by watchdog; abort triggered.")
        passed_cases.append("Case 3: Stale Odometry during Landing")
        node3.destroy_node()
    except Exception as e:
        print(f"  -> FAILED Case 3: {e}")
        failed_cases.append(("Case 3", str(e)))

    # -------------------------------------------------------------------------
    # Case 4: Disarmed Status while Airborne
    # -------------------------------------------------------------------------
    print("\n[Case 4/4] Testing Premature Disarmed Status while Airborne...")
    try:
        node4 = SingleDroneTestNode(namespace="drone_0", authorize_flight=False)
        node4.initial_pos = [0.0, 0.0, 0.0]
        node4.vehicle_pos = [0.0, 0.0, -5.0]  # 5.0m airborne in local NED
        node4.vehicle_vel = [0.0, 0.0, 0.0]
        node4.vehicle_connected = True
        node4.flight_initiated = True
        node4.arming_state = VehicleStatus.ARMING_STATE_DISARMED  # Glitched/premature disarm!
        node4.state = FlightState.LAND

        # Run for 25 ticks
        for _ in range(25):
            node4.last_odom_wall_time = time.time()
            node4.last_status_wall_time = time.time()
            node4.timer_callback()

        # Because vehicle is at -5.0m (airborne), ground contact ticks must be 0 and state must NOT transition!
        assert node4.ground_contact_ticks == 0, f"Ground contact ticks should be 0, got {node4.ground_contact_ticks}"
        assert node4.state == FlightState.LAND, f"CRITICAL HAZARD: Declared complete while airborne! State: {node4.state}"
        print("  -> PASSED: Disarm status while airborne strictly rejected; controller held in LAND.")
        passed_cases.append("Case 4: Disarmed Status while Airborne")
        node4.destroy_node()
    except Exception as e:
        print(f"  -> FAILED Case 4: {e}")
        failed_cases.append(("Case 4", str(e)))

    rclpy.shutdown()

    # -------------------------------------------------------------------------
    # Final Report
    # -------------------------------------------------------------------------
    print("\n" + "=" * 70)
    print("GATE 3 VERIFICATION SUMMARY")
    print("=" * 70)
    print(f"Total Cases:  4")
    print(f"Passed:       {len(passed_cases)}")
    print(f"Failed:       {len(failed_cases)}")
    print("=" * 70)

    if len(failed_cases) == 0:
        print("[ACCEPTANCE RESULT] PASS — All 4 landing & disarm telemetry cases verified successfully.")
        print("=" * 70)
        sys.exit(0)
    else:
        print("[ACCEPTANCE RESULT] FAIL — Some test cases failed:")
        for name, err in failed_cases:
            print(f"  * {name}: {err}")
        print("=" * 70)
        sys.exit(1)


if __name__ == '__main__':
    run_tests()
