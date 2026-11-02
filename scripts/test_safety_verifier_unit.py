#!/usr/bin/env python3
"""
Unit and Integration Test Suite for Authoritative Teardown Verifier (Phase F5.3D)
Tests verify_vehicle_state_teardown.py against all safety conditions and failure modes:
1. Fresh, disarmed, landed, at rest, on ground -> Expects RC 0 (SAFE)
2. Armed vehicle (arming_state == 2) -> Expects RC 2 (VEHICLE_ARMED)
3. Non-disarmed / Unexpected arming state (arming_state == 3 e.g. ARMING) -> Expects RC 2 (FAIL-CLOSED)
4. Unrecognized arming state (arming_state == 99) -> Expects RC 2 (FAIL-CLOSED)
5. Airborne vehicle (landed == False) -> Expects RC 3 (VEHICLE_AIRBORNE)
6. Kinetic motion (speed > max_speed) -> Expects RC 3 (VEHICLE_AIRBORNE)
7. Position above ground (Z > max_altitude) -> Expects RC 3 (VEHICLE_AIRBORNE)
8. In-session stale telemetry (stream received in-session, then ceases beyond max_age) -> Expects RC 4 (TIMEOUT/STALE)
9. Incomplete telemetry (stream never published in-session) -> Expects RC 4 (TIMEOUT/INCOMPLETE)
"""

import sys
import time
import subprocess
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleStatus, VehicleLandDetected, VehicleOdometry


class MockPX4TelemetryPublisher(Node):
    def __init__(self, namespace="drone_0"):
        super().__init__('mock_px4_telemetry_pub')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        ns_prefix = f"/{namespace}" if namespace else ""
        self.status_pub = self.create_publisher(VehicleStatus, f'{ns_prefix}/fmu/out/vehicle_status', qos)
        self.land_pub = self.create_publisher(VehicleLandDetected, f'{ns_prefix}/fmu/out/vehicle_land_detected', qos)
        self.odom_pub = self.create_publisher(VehicleOdometry, f'{ns_prefix}/fmu/out/vehicle_odometry', qos)

        # State configurations
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.nav_state = 18  # AUTO_LAND
        self.landed = True
        self.ground_contact = True
        self.position = [0.0, 0.0, 0.02]
        self.velocity = [0.01, 0.01, 0.01]

        # Enable flags per stream
        self.pub_status = True
        self.pub_land = True
        self.pub_odom = True

        self.timer = self.create_timer(0.05, self.publish_telemetry)  # 20 Hz

    def publish_telemetry(self):
        t_us = int(self.get_clock().now().nanoseconds / 1000)

        if self.pub_status:
            s_msg = VehicleStatus()
            s_msg.timestamp = t_us
            s_msg.arming_state = self.arming_state
            s_msg.nav_state = self.nav_state
            self.status_pub.publish(s_msg)

        if self.pub_land:
            l_msg = VehicleLandDetected()
            l_msg.timestamp = t_us
            l_msg.landed = self.landed
            l_msg.ground_contact = self.ground_contact
            self.land_pub.publish(l_msg)

        if self.pub_odom:
            o_msg = VehicleOdometry()
            o_msg.timestamp = t_us
            o_msg.position = [float(p) for p in self.position]
            o_msg.velocity = [float(v) for v in self.velocity]
            self.odom_pub.publish(o_msg)


def run_verifier(timeout=2.0, max_age=1.0, max_speed=0.20, max_altitude=0.35, tracker=None):
    cmd = [
        sys.executable,
        "/home/capstone/capstone_project_antigravity/scripts/verify_vehicle_state_teardown.py",
        "--namespace", "drone_0",
        "--timeout", str(timeout),
        "--max-age", str(max_age),
        "--max-speed", str(max_speed),
        "--max-altitude", str(max_altitude),
    ]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if tracker is not None:
        tracker.append(proc)
    out, err = proc.communicate()
    return proc.returncode, out, err


def main():
    rclpy.init()
    mock_node = MockPX4TelemetryPublisher(namespace="drone_0")
    spin_thread = threading.Thread(target=rclpy.spin, args=(mock_node,), daemon=True)
    spin_thread.start()

    print("=" * 70)
    print("RUNNING UNIT TESTS FOR verify_vehicle_state_teardown.py")
    print("=" * 70)

    results = []
    spawned_procs = []

    try:
        # Test 1: Baseline Clean Pass (Disarmed, Landed, Speed 0.017 m/s, Z = 0.02 m)
        mock_node.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        mock_node.landed = True
        mock_node.ground_contact = True
        mock_node.position = [0.0, 0.0, 0.02]
        mock_node.velocity = [0.01, 0.01, 0.01]
        mock_node.pub_status = True
        mock_node.pub_land = True
        mock_node.pub_odom = True
        time.sleep(0.3)
        rc, out, err = run_verifier(tracker=spawned_procs)
        assert rc == 0, f"Expected 0, got {rc}. Stderr: {err}"
        print("[TEST 1/9 PASS] Clean teardown state -> Exit 0 (SAFE_FOR_TEARDOWN)")
        results.append(("Clean teardown state", 0, rc))

        # Test 2: Armed Vehicle (arming_state == 2) -> Must fail closed with Exit 2
        mock_node.arming_state = VehicleStatus.ARMING_STATE_ARMED
        time.sleep(0.3)
        rc, out, err = run_verifier(tracker=spawned_procs)
        assert rc == 2, f"Expected 2, got {rc}. Stderr: {err}"
        print("[TEST 2/9 PASS] Armed vehicle (arming_state=2) -> Exit 2 (VEHICLE_ARMED)")
        results.append(("Armed vehicle (state=2)", 2, rc))

        # Test 3: Unexpected Arming State (arming_state == 3 e.g. ARMING) -> Must fail closed with Exit 2 (P0 Resolution)
        mock_node.arming_state = 3
        time.sleep(0.3)
        rc, out, err = run_verifier(tracker=spawned_procs)
        assert rc == 2, f"Expected 2, got {rc}. Stderr: {err}"
        print("[TEST 3/9 PASS] Unexpected arming state (arming_state=3) -> Exit 2 (FAIL-CLOSED)")
        results.append(("Unexpected arming state (3)", 2, rc))

        # Test 4: Unrecognized Arming State (arming_state == 99) -> Must fail closed with Exit 2 (P0 Resolution)
        mock_node.arming_state = 99
        time.sleep(0.3)
        rc, out, err = run_verifier(tracker=spawned_procs)
        assert rc == 2, f"Expected 2, got {rc}. Stderr: {err}"
        print("[TEST 4/9 PASS] Unrecognized arming state (arming_state=99) -> Exit 2 (FAIL-CLOSED)")
        results.append(("Unrecognized arming state (99)", 2, rc))
        mock_node.arming_state = VehicleStatus.ARMING_STATE_DISARMED

        # Test 5: Airborne Vehicle (landed == False) -> Must fail closed with Exit 3
        mock_node.landed = False
        mock_node.ground_contact = False
        time.sleep(0.3)
        rc, out, err = run_verifier(tracker=spawned_procs)
        assert rc == 3, f"Expected 3, got {rc}. Stderr: {err}"
        print("[TEST 5/9 PASS] Airborne vehicle (landed=False) -> Exit 3 (VEHICLE_AIRBORNE)")
        results.append(("Airborne vehicle", 3, rc))
        mock_node.landed = True
        mock_node.ground_contact = True

        # Test 6: Excessive Speed (speed = 0.50 m/s > 0.20 m/s) -> Must fail closed with Exit 3
        mock_node.velocity = [0.40, 0.30, 0.0]  # norm = 0.50 m/s
        time.sleep(0.3)
        rc, out, err = run_verifier(max_speed=0.20, tracker=spawned_procs)
        assert rc == 3, f"Expected 3, got {rc}. Stderr: {err}"
        print("[TEST 6/9 PASS] High velocity vehicle (0.50 m/s > 0.20 m/s) -> Exit 3 (VEHICLE_AIRBORNE)")
        results.append(("High velocity vehicle", 3, rc))
        mock_node.velocity = [0.01, 0.01, 0.01]

        # Test 7: Excessive Altitude (|Z| = 1.20 m > 0.35 m) -> Must fail closed with Exit 3
        mock_node.position = [0.0, 0.0, -1.20]  # 1.2m above ground in NED
        time.sleep(0.3)
        rc, out, err = run_verifier(max_altitude=0.35, tracker=spawned_procs)
        assert rc == 3, f"Expected 3, got {rc}. Stderr: {err}"
        print("[TEST 7/9 PASS] High altitude vehicle (|Z|=1.2m > 0.35m) -> Exit 3 (VEHICLE_AIRBORNE)")
        results.append(("High altitude vehicle", 3, rc))
        mock_node.position = [0.0, 0.0, 0.02]

        # Test 8: In-Session Stale Telemetry (P1 Resolution: stream received in-session, then ceases and expires)
        mock_node.pub_status = False
        mock_node.pub_land = True
        mock_node.pub_odom = True

        cmd_stale = [
            sys.executable,
            "/home/capstone/capstone_project_antigravity/scripts/verify_vehicle_state_teardown.py",
            "--namespace", "drone_0",
            "--timeout", "1.4",
            "--max-age", "0.4",
            "--max-speed", "0.20",
            "--max-altitude", "0.35",
        ]
        proc_stale = subprocess.Popen(cmd_stale, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        spawned_procs.append(proc_stale)
        time.sleep(0.08)  # Allow in-session receipt of odom and land
        mock_node.pub_odom = False  # Cease odom stream (now captured in-session, but going stale)
        time.sleep(0.45)  # Wait until in-session odom age exceeds max_age=0.4s
        mock_node.pub_status = True  # Enable status stream, completing in-session 3-stream reception

        out_stale, err_stale = proc_stale.communicate()
        rc_stale = proc_stale.returncode
        assert rc_stale == 4, f"Expected 4 for in-session stale telemetry, got {rc_stale}. Stderr: {err_stale}"
        assert "vehicle_odometry" in err_stale, f"Expected odom freshness failure in stderr: {err_stale}"
        print("[TEST 8/9 PASS] In-session stale telemetry (stream captured then expired) -> Exit 4 (TIMEOUT/STALE)")
        results.append(("In-session stale stream", 4, rc_stale))
        mock_node.pub_odom = True

        # Test 9: Incomplete Telemetry (Status never published in-session) -> Exit 4
        mock_node.pub_status = False
        time.sleep(0.4)
        rc, out, err = run_verifier(timeout=1.0, max_age=0.5, tracker=spawned_procs)
        assert rc == 4, f"Expected 4, got {rc}. Stderr: {err}"
        assert "NEVER_RECEIVED" in err, f"Expected NEVER_RECEIVED for status in stderr: {err}"
        print("[TEST 9/9 PASS] Incomplete telemetry stream (status never published) -> Exit 4 (TIMEOUT/INCOMPLETE)")
        results.append(("Incomplete telemetry stream", 4, rc))
        mock_node.pub_status = True

    finally:
        # Guarantee child process and mock node cleanup even if an assertion fails
        for proc in spawned_procs:
            if proc.poll() is None:
                try:
                    proc.kill()
                    proc.wait(timeout=1.0)
                except Exception:
                    pass
        if mock_node is not None:
            mock_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print("=" * 70)
    print("ALL 9 VERIFIER UNIT TESTS PASSED SUCCESSFULLY!")
    print("=" * 70)
    for name, expected, actual in results:
        print(f"  {name:30}: Expected RC={expected}, Actual RC={actual} [MATCH]")
    print("=" * 70)


if __name__ == '__main__':
    main()
