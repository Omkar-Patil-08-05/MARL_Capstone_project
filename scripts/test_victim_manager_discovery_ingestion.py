#!/usr/bin/env python3
"""
Dedicated Test Suite for VictimManager Autonomous Discovery Ingestion (Phase 3A)

Verifies:
  1. Valid autonomous discovery ingestion and full metadata persistence.
  2. Decoupling of source-local track IDs from global internal victim IDs (collision test).
  3. Multi-drone same-victim spatial association, observation count, and EMA smoothing.
  4. Autonomous association radius boundary behavior (within 6.0m vs outside 6.0m).
  5. Ambiguous contention safeguard (equidistant / close candidates -> conservative new track).
  6. Simulation timestamp zero (0.0) legitimately accepted.
  7. Negative, NaN, Inf simulation timestamps, coordinates, and confidences safely rejected.
  8. Null thermal metadata preservation without overwriting on subsequent detections.
  9. Preservation of source, source_track_id, and verification_mode as separate metadata fields.
  10. Strict ground-truth isolation (vm.victims remains untouched by autonomous ingestion).
  11. Backward compatibility for TrackedVictim.id, get_dict(), and dashboard telemetry.
  12. Configuration validation for AUTONOMOUS_ASSOCIATION_RADIUS with explicit fallback.
  13. QMIXMissionController discovery_callback ROS message ingestion and exception safety.
  14. Legacy positional signature backward compatibility.
"""

import os
import sys
import math
import time
import json
from typing import Dict, Any

# Ensure workspace packages can be imported
ws_src = os.path.abspath(os.path.join(os.path.dirname(__file__), '../drone_ws/src/swarm_controller'))
if ws_src not in sys.path:
    sys.path.insert(0, ws_src)

repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if repo_root not in sys.path:
    sys.path.insert(0, repo_root)

# Ensure ROS 2 and px4_msgs can be imported when running standalone
ros2_paths = [
    '/opt/ros/jazzy/lib/python3.12/site-packages',
    os.path.abspath(os.path.join(os.path.dirname(__file__), '../drone_ws/install/px4_msgs/lib/python3.12/site-packages')),
]
for p in ros2_paths:
    if os.path.exists(p) and p not in sys.path:
        sys.path.append(p)


from swarm_controller.victim_manager import VictimManager, TrackedVictim, VictimState
from swarm_controller.grid_world_transform import GridWorldTransform
from std_msgs.msg import String


class MockEnv:
    def __init__(self, x_size=100, y_size=100):
        self.x_size = x_size
        self.y_size = y_size
        import numpy as np
        self.grid = np.zeros((x_size, y_size), dtype=np.int32)
        self.victims = {}


class MockLogger:
    def __init__(self):
        self.infos = []
        self.warns = []
        self.errors = []

    def info(self, msg):
        self.infos.append(msg)

    def warn(self, msg):
        self.warns.append(msg)

    def error(self, msg):
        self.errors.append(msg)


class MockNode:
    def __init__(self):
        self._logger = MockLogger()

    def get_logger(self):
        return self._logger


def create_test_victim_manager(auto_radius=None, seed=42):
    meta = {
        "victims": [
            {
                "id": "V0",
                "grid": {"x": 10, "y": 10},
                "world": {"x": 20.0, "y": 20.0, "z": 0.1}
            },
            {
                "id": "V1",
                "grid": {"x": 40, "y": 40},
                "world": {"x": 80.0, "y": 80.0, "z": 0.1}
            }
        ]
    }
    node = MockNode()
    env = MockEnv()
    vm = VictimManager(
        node=node,
        metadata=meta,
        world_name="test_sar_world",
        env=env,
        seed=seed,
        autonomous_association_radius=auto_radius
    )
    return vm, node, env


def run_test(name: str, fn):
    try:
        fn()
        print(f"[PASS] {name}")
        return True
    except AssertionError as e:
        print(f"[FAIL] {name}: AssertionError: {e}")
        return False
    except Exception as e:
        print(f"[FAIL] {name}: Exception: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_valid_autonomous_discovery_ingestion():
    vm, _, _ = create_test_victim_manager()
    tid, is_new = vm.process_discovery_event(
        world_x=25.0,
        world_y=35.0,
        source="multispectral_verifier",
        confidence=0.95,
        drone_id=1,
        sim_time=14.5,
        source_track_id=7,
        verification_mode="MULTISPECTRAL_FUSION",
        evidence_state="TEMPORALLY_CONFIRMED",
        thermal_peak_k=308.2,
        thermal_delta_k=11.4,
        confirmation_count=4,
        confirmation_window=10,
        consensus_span_sec=1.25,
        median_depth_estimate=8.5,
        depth_valid_samples=420,
        depth_spread=0.25
    )
    assert is_new is True, "First detection must be marked is_new=True"
    assert tid == "victim_1", f"Expected 'victim_1', got '{tid}'"
    assert "victim_1" in vm.tracked_victims

    track = vm.tracked_victims["victim_1"]
    assert track.id == "victim_1"
    assert track.world_x == 25.0
    assert track.world_y == 35.0
    assert track.source == "multispectral_verifier"
    assert track.source_track_id == 7
    assert track.verification_mode == "MULTISPECTRAL_FUSION"
    assert track.confidence == 0.95
    assert track.detected_by == 1
    assert track.sim_time == 14.5
    assert track.first_seen_sim == 14.5
    assert track.last_seen_sim == 14.5
    assert track.thermal_peak_k == 308.2
    assert track.thermal_delta_k == 11.4
    assert track.confirmation_count == 4
    assert track.observations == 1


def test_source_local_track_id_collision_decoupling():
    vm, _, _ = create_test_victim_manager()
    # Drone 1 reports detection with source_track_id=1 at (15.0, 15.0)
    tid1, is_new1 = vm.process_discovery_event(
        world_x=15.0,
        world_y=15.0,
        source="rgb_thermal_fusion_d1",
        confidence=0.88,
        drone_id=1,
        sim_time=5.0,
        source_track_id=1,
        verification_mode="MULTISPECTRAL_FUSION"
    )
    # Drone 2 reports detection with source_track_id=1 at (70.0, 70.0) (different victim far away)
    tid2, is_new2 = vm.process_discovery_event(
        world_x=70.0,
        world_y=70.0,
        source="rgb_thermal_fusion_d2",
        confidence=0.91,
        drone_id=2,
        sim_time=6.0,
        source_track_id=1,
        verification_mode="MULTISPECTRAL_FUSION"
    )
    assert is_new1 is True and is_new2 is True, "Both distant detections must create new tracks"
    assert tid1 != tid2, f"Track IDs must be distinct despite identical source_track_id: {tid1} vs {tid2}"
    assert len(vm.tracked_victims) == 2, "Expected exactly 2 tracked victims"

    t1 = vm.tracked_victims[tid1]
    t2 = vm.tracked_victims[tid2]
    assert t1.source_track_id == 1 and t2.source_track_id == 1, "Source local IDs must both be preserved"
    assert t1.world_x == 15.0 and t2.world_x == 70.0


def test_multidrone_same_victim_association_and_smoothing():
    vm, _, _ = create_test_victim_manager(auto_radius=6.0)
    # First detection from Drone 1
    tid1, is_new1 = vm.process_discovery_event(
        world_x=30.0,
        world_y=30.0,
        source="multispectral_verifier",
        confidence=0.80,
        drone_id=1,
        sim_time=10.0
    )
    # Second detection from Drone 2 nearby (dist = sqrt(1.0^2 + 0.5^2) = 1.12m <= 6.0m)
    tid2, is_new2 = vm.process_discovery_event(
        world_x=31.0,
        world_y=30.5,
        source="multispectral_verifier",
        confidence=0.94,
        drone_id=2,
        sim_time=11.2
    )
    assert is_new1 is True, "First detection must be new"
    assert is_new2 is False, "Second detection must merge with existing track"
    assert tid1 == tid2 == "victim_1", "Both detections must map to the same track"
    assert len(vm.tracked_victims) == 1

    t = vm.tracked_victims["victim_1"]
    assert t.observations == 2, f"Expected 2 observations, got {t.observations}"
    assert t.confidence == 0.94, "Confidence should take maximum observed value"
    assert t.detected_by == 2, "Latest reporting drone should be recorded"
    assert t.last_seen_sim == 11.2
    assert t.first_seen_sim == 10.0

    # EMA check: alpha = 0.7
    # x = 30.0 * 0.3 + 31.0 * 0.7 = 9.0 + 21.7 = 30.7
    # y = 30.0 * 0.3 + 30.5 * 0.7 = 9.0 + 21.35 = 30.35
    assert abs(t.world_x - 30.7) < 1e-3, f"Expected world_x ~ 30.7, got {t.world_x}"
    assert abs(t.world_y - 30.35) < 1e-3, f"Expected world_y ~ 30.35, got {t.world_y}"


def test_autonomous_association_radius_boundary():
    vm, _, _ = create_test_victim_manager(auto_radius=6.0)
    # Base track at (40.0, 40.0)
    vm.process_discovery_event(world_x=40.0, world_y=40.0, confidence=0.9, drone_id=1, sim_time=1.0)

    # Ingestion at distance 5.0m <= 6.0m (within radius) -> should merge
    tid_merge, is_new_merge = vm.process_discovery_event(world_x=45.0, world_y=40.0, confidence=0.9, drone_id=1, sim_time=2.0)
    assert is_new_merge is False
    assert tid_merge == "victim_1"

    # Current smoothed x is 40.0*0.3 + 45.0*0.7 = 43.5.
    # Ingestion at (55.0, 40.0) -> distance is 55.0 - 43.5 = 11.5m > 6.0m -> should create new track
    tid_new, is_new = vm.process_discovery_event(world_x=55.0, world_y=40.0, confidence=0.9, drone_id=1, sim_time=3.0)
    assert is_new is True
    assert tid_new == "victim_2"
    assert len(vm.tracked_victims) == 2


def test_ambiguity_contention_conservative_handling():
    vm, _, _ = create_test_victim_manager(auto_radius=6.0)
    # Track 1 at (50.0, 50.0)
    tid1, is_new1 = vm.process_discovery_event(world_x=50.0, world_y=50.0, confidence=0.9, drone_id=1, sim_time=1.0)
    assert is_new1 is True and tid1 == "victim_1"

    # Track 2 at (60.0, 50.0) (separated by 10.0m > 6.0m -> distinct track)
    tid2, is_new2 = vm.process_discovery_event(world_x=60.0, world_y=50.0, confidence=0.9, drone_id=2, sim_time=2.0)
    assert is_new2 is True and tid2 == "victim_2"
    assert len(vm.tracked_victims) == 2

    # A new detection arrives at (55.0, 50.0) (exactly midway: d1 = 5.0m, d2 = 5.0m, delta = 0.0m < 1.0m)
    # Both tracks are within 6.0m and difference between best and second-best is < ambiguity_margin (1.0m)
    tid_ambig, is_new_ambig = vm.process_discovery_event(world_x=55.0, world_y=50.0, confidence=0.9, drone_id=1, sim_time=3.0)
    assert is_new_ambig is True, "Ambiguous detection must trigger conservative handling and create a new track"
    assert tid_ambig == "victim_3"
    assert len(vm.tracked_victims) == 3

    # Now a detection arrives clearly close to Track 1 at (49.5, 50.0)
    # d1 = 0.5m, next closest (victim_3 at 55.0) d3 = 5.5m. delta = 5.0m >= 1.0m
    tid_clear, is_new_clear = vm.process_discovery_event(world_x=49.5, world_y=50.0, confidence=0.9, drone_id=1, sim_time=4.0)
    assert is_new_clear is False, "Unambiguous detection must merge with nearest track"
    assert tid_clear == "victim_1"
    assert vm.tracked_victims["victim_1"].observations == 2


def test_ineligible_candidate_does_not_cause_ambiguity_contention():
    """Regression test:
    Nearest candidate is within 6.0m (5.5m) and second candidate is outside 6.0m (6.2m)
    but within 1.0m of nearest candidate's distance (|6.2 - 5.5| = 0.7m < 1.0m).
    This must NOT be classified as contention with an ineligible candidate;
    it must merge with the eligible nearest candidate.
    """
    vm, _, _ = create_test_victim_manager(auto_radius=6.0)

    # Track 1 at (5.5, 0.0) -> distance from origin (0, 0) is 5.5m (within 6.0m: ELIGIBLE)
    tid1, is_new1 = vm.process_discovery_event(
        world_x=5.5, world_y=0.0, confidence=0.9, drone_id=1, sim_time=1.0, source_track_id=1
    )
    assert is_new1 is True and tid1 == "victim_1"

    # Track 2 at (0.0, 6.2) -> distance from origin (0, 0) is 6.2m (outside 6.0m: INELIGIBLE)
    # Distance between Track 1 and Track 2 is sqrt(5.5^2 + 6.2^2) = 8.29m > 6.0m -> distinct track
    tid2, is_new2 = vm.process_discovery_event(
        world_x=0.0, world_y=6.2, confidence=0.9, drone_id=2, sim_time=2.0, source_track_id=2
    )
    assert is_new2 is True and tid2 == "victim_2"
    assert len(vm.tracked_victims) == 2

    # Detection at (0.0, 0.0):
    # d(Track 1) = 5.5m <= 6.0m (eligible)
    # d(Track 2) = 6.2m > 6.0m (ineligible)
    # delta = 6.2 - 5.5 = 0.7m < ambiguity_margin (1.0m)
    # Ineligible Track 2 MUST NOT trigger ambiguity contention!
    # Detection must merge directly with eligible Track 1.
    tid_ingest, is_new_ingest = vm.process_discovery_event(
        world_x=0.0, world_y=0.0, confidence=0.95, drone_id=1, sim_time=3.0, source_track_id=1
    )
    assert is_new_ingest is False, "Must not be classified as contention with ineligible candidate"
    assert tid_ingest == "victim_1", f"Expected merge with victim_1, got {tid_ingest}"
    assert len(vm.tracked_victims) == 2, "Must remain 2 tracks, no new track created"
    assert vm.tracked_victims["victim_1"].observations == 2

    # Now verify that if second candidate IS eligible (within 6.0m), contention DOES trigger:
    vm2, _, _ = create_test_victim_manager(auto_radius=6.0)
    vm2.process_discovery_event(world_x=5.2, world_y=0.0, confidence=0.9, drone_id=1, sim_time=1.0)
    vm2.process_discovery_event(world_x=0.0, world_y=5.6, confidence=0.9, drone_id=2, sim_time=2.0)
    assert len(vm2.tracked_victims) == 2
    # Detection at (0.0, 0.0): d1 = 5.2m, d2 = 5.6m. Both <= 6.0m! Delta = 0.4m < 1.0m.
    # Genuine ambiguity between eligible candidates -> creates new track
    tid_contend, is_new_contend = vm2.process_discovery_event(
        world_x=0.0, world_y=0.0, confidence=0.9, drone_id=1, sim_time=3.0
    )
    assert is_new_contend is True, "Genuine contention between eligible candidates must create new track"
    assert tid_contend == "victim_3"
    assert len(vm2.tracked_victims) == 3


def test_sim_timestamp_zero_accepted():
    vm, _, _ = create_test_victim_manager()
    tid, is_new = vm.process_discovery_event(world_x=12.0, world_y=14.0, confidence=0.9, drone_id=1, sim_time=0.0)
    assert is_new is True
    assert tid == "victim_1"
    track = vm.tracked_victims["victim_1"]
    assert track.sim_time == 0.0
    assert track.first_seen_sim == 0.0
    assert track.last_seen_sim == 0.0


def test_negative_and_nonfinite_inputs_rejected():
    vm, _, _ = create_test_victim_manager()
    # Negative sim_time rejected
    tid, is_new = vm.process_discovery_event(world_x=10.0, world_y=10.0, confidence=0.9, drone_id=1, sim_time=-1.0)
    assert tid is None and is_new is False

    # NaN sim_time rejected
    tid, is_new = vm.process_discovery_event(world_x=10.0, world_y=10.0, confidence=0.9, drone_id=1, sim_time=float('nan'))
    assert tid is None and is_new is False

    # Inf coordinates rejected
    tid, is_new = vm.process_discovery_event(world_x=float('inf'), world_y=10.0, confidence=0.9, drone_id=1, sim_time=1.0)
    assert tid is None and is_new is False

    tid, is_new = vm.process_discovery_event(world_x=10.0, world_y=float('nan'), confidence=0.9, drone_id=1, sim_time=1.0)
    assert tid is None and is_new is False

    # Out-of-range confidence rejected
    tid, is_new = vm.process_discovery_event(world_x=10.0, world_y=10.0, confidence=1.5, drone_id=1, sim_time=1.0)
    assert tid is None and is_new is False

    tid, is_new = vm.process_discovery_event(world_x=10.0, world_y=10.0, confidence=-0.1, drone_id=1, sim_time=1.0)
    assert tid is None and is_new is False

    # Negative drone_id rejected
    tid, is_new = vm.process_discovery_event(world_x=10.0, world_y=10.0, confidence=0.9, drone_id=-1, sim_time=1.0)
    assert tid is None and is_new is False

    assert len(vm.tracked_victims) == 0, "No invalid detections should be saved"


def test_missing_mandatory_fields_rejected():
    """Verifies that missing or invalid mandatory publisher fields are rejected with diagnostics."""
    vm, _, _ = create_test_victim_manager()

    base_payload = {
        "event_type": "VICTIM_DISCOVERED",
        "timestamp": 10.0,
        "victim_track_id": 1,
        "confirming_drone_id": 1,
        "world_x": 20.0,
        "world_y": 20.0,
        "confidence": 0.9
    }

    # 1. Missing event_type
    p = dict(base_payload)
    del p["event_type"]
    ok, _, reason = VictimManager.validate_event_payload(p)
    assert ok is False and "event_type" in reason

    # 2. Wrong event_type
    p = dict(base_payload, event_type="WRONG_EVENT")
    ok, _, reason = VictimManager.validate_event_payload(p)
    assert ok is False and "event_type" in reason

    # 3. Missing timestamp
    p = dict(base_payload)
    del p["timestamp"]
    ok, _, reason = VictimManager.validate_event_payload(p)
    assert ok is False and "timestamp" in reason

    # 4. Missing victim_track_id
    p = dict(base_payload)
    del p["victim_track_id"]
    ok, _, reason = VictimManager.validate_event_payload(p)
    assert ok is False and "victim_track_id" in reason

    # 5. Missing confirming_drone_id
    p = dict(base_payload)
    del p["confirming_drone_id"]
    ok, _, reason = VictimManager.validate_event_payload(p)
    assert ok is False and "confirming_drone_id" in reason

    # 6. Missing world_x
    p = dict(base_payload)
    del p["world_x"]
    ok, _, reason = VictimManager.validate_event_payload(p)
    assert ok is False and "world_x" in reason

    # 7. Missing world_y
    p = dict(base_payload)
    del p["world_y"]
    ok, _, reason = VictimManager.validate_event_payload(p)
    assert ok is False and "world_y" in reason

    # 8. Missing confidence
    p = dict(base_payload)
    del p["confidence"]
    ok, _, reason = VictimManager.validate_event_payload(p)
    assert ok is False and "confidence" in reason

    # 9. Direct process_discovery_event calls with missing mandatory parameters
    assert vm.process_discovery_event(world_x=None, world_y=20.0, confidence=0.9, drone_id=1, sim_time=1.0) == (None, False)
    assert vm.process_discovery_event(world_x=20.0, world_y=None, confidence=0.9, drone_id=1, sim_time=1.0) == (None, False)
    assert vm.process_discovery_event(world_x=20.0, world_y=20.0, confidence=None, drone_id=1, sim_time=1.0) == (None, False)
    assert vm.process_discovery_event(world_x=20.0, world_y=20.0, confidence=0.9, drone_id=None, sim_time=1.0) == (None, False)
    assert vm.process_discovery_event(world_x=20.0, world_y=20.0, confidence=0.9, drone_id=1, sim_time=None) == (None, False)

    assert len(vm.tracked_victims) == 0, "No invalid events should create tracks"


def test_absent_source_metadata_handled_without_fabrication():
    """Verifies that an absent source identifier yields UNKNOWN_AUTONOMOUS_SOURCE and never defaults to multispectral_verifier."""
    vm, _, _ = create_test_victim_manager()

    payload = {
        "event_type": "VICTIM_DISCOVERED",
        "timestamp": 8.5,
        "victim_track_id": 5,
        "confirming_drone_id": 2,
        "world_x": 35.0,
        "world_y": 45.0,
        "confidence": 0.88,
        "verification_mode": "MULTISPECTRAL_RGB_THERMAL"
        # NOTE: No "source" key in payload!
    }

    ok, parsed, reason = VictimManager.validate_event_payload(payload)
    assert ok is True, f"Validation failed: {reason}"
    assert parsed["source"] == "UNKNOWN_AUTONOMOUS_SOURCE"
    assert parsed["source"] != "multispectral_verifier", "Must NEVER default missing source to 'multispectral_verifier'"
    assert parsed["verification_mode"] == "MULTISPECTRAL_RGB_THERMAL"
    assert parsed["source_track_id"] == 5

    tid, is_new = vm.process_discovery_event(**parsed)
    assert is_new is True
    track = vm.tracked_victims[tid]
    assert track.source == "UNKNOWN_AUTONOMOUS_SOURCE"
    assert track.source != "multispectral_verifier"
    assert track.verification_mode == "MULTISPECTRAL_RGB_THERMAL"
    assert track.source_track_id == 5

    d = track.get_dict()
    assert d["source"] == "UNKNOWN_AUTONOMOUS_SOURCE"
    assert d["source"] != "multispectral_verifier"
    assert d["verification_mode"] == "MULTISPECTRAL_RGB_THERMAL"
    assert d["source_track_id"] == 5


def test_invalid_optional_numeric_fields_rejected():
    """Verifies that non-finite or negative values for optional numeric fields are rejected."""
    vm, _, _ = create_test_victim_manager()

    base = {
        "event_type": "VICTIM_DISCOVERED",
        "timestamp": 1.0,
        "victim_track_id": 1,
        "confirming_drone_id": 0,
        "world_x": 10.0,
        "world_y": 10.0,
        "confidence": 0.9
    }

    # Negative median_depth_estimate
    ok, _, _ = VictimManager.validate_event_payload(dict(base, median_depth_estimate=-1.0))
    assert ok is False

    # NaN median_depth_estimate
    ok, _, _ = VictimManager.validate_event_payload(dict(base, median_depth_estimate=float('nan')))
    assert ok is False

    # Inf median_depth_estimate
    ok, _, _ = VictimManager.validate_event_payload(dict(base, median_depth_estimate=float('inf')))
    assert ok is False

    # Negative consensus_span_sec
    ok, _, _ = VictimManager.validate_event_payload(dict(base, consensus_span_sec=-0.5))
    assert ok is False

    # Negative depth_spread
    ok, _, _ = VictimManager.validate_event_payload(dict(base, depth_spread=-0.1))
    assert ok is False

    # Negative thermal_peak_k
    ok, _, _ = VictimManager.validate_event_payload(dict(base, thermal_peak_k=-20.0))
    assert ok is False

    # Non-positive grid_cell_size_m
    ok, _, _ = VictimManager.validate_event_payload(dict(base, grid_cell_size_m=0.0))
    assert ok is False

    ok, _, _ = VictimManager.validate_event_payload(dict(base, grid_cell_size_m=-1.0))
    assert ok is False

    # Direct call rejection in process_discovery_event
    res = vm.process_discovery_event(world_x=10.0, world_y=10.0, confidence=0.9, drone_id=0, sim_time=1.0, median_depth_estimate=-2.5)
    assert res == (None, False)

    res = vm.process_discovery_event(world_x=10.0, world_y=10.0, confidence=0.9, drone_id=0, sim_time=1.0, thermal_peak_k=-5.0)
    assert res == (None, False)

    # Direct construction of TrackedVictim with invalid optional numeric raises ValueError
    try:
        TrackedVictim("t1", 10.0, 10.0, confidence=0.9, drone_id=0, sim_time=1.0, median_depth_estimate=-1.0)
        assert False, "TrackedVictim must reject negative median_depth_estimate"
    except ValueError:
        pass


def test_fractional_optional_integer_fields_rejected():
    """Verifies that fractional floats and negative values for optional integer fields are rejected."""
    vm, _, _ = create_test_victim_manager()

    base = {
        "event_type": "VICTIM_DISCOVERED",
        "timestamp": 1.0,
        "victim_track_id": 1,
        "confirming_drone_id": 0,
        "world_x": 10.0,
        "world_y": 10.0,
        "confidence": 0.9
    }

    # Fractional confirmation_count
    ok, _, _ = VictimManager.validate_event_payload(dict(base, confirmation_count=4.5))
    assert ok is False

    # Fractional confirmation_window
    ok, _, _ = VictimManager.validate_event_payload(dict(base, confirmation_window=10.2))
    assert ok is False

    # Fractional depth_valid_samples
    ok, _, _ = VictimManager.validate_event_payload(dict(base, depth_valid_samples=15.8))
    assert ok is False

    # Negative integer confirmation_count
    ok, _, _ = VictimManager.validate_event_payload(dict(base, confirmation_count=-1))
    assert ok is False

    # Negative integer depth_valid_samples
    ok, _, _ = VictimManager.validate_event_payload(dict(base, depth_valid_samples=-5))
    assert ok is False

    # Boolean confirmation_count (isinstance(True, int) is True, must be rejected)
    ok, _, _ = VictimManager.validate_event_payload(dict(base, confirmation_count=True))
    assert ok is False

    # Direct call rejection in process_discovery_event
    res = vm.process_discovery_event(world_x=10.0, world_y=10.0, confidence=0.9, drone_id=0, sim_time=1.0, confirmation_count=4.5)
    assert res == (None, False)

    # TrackedVictim direct construction rejection
    try:
        TrackedVictim("t1", 10.0, 10.0, confidence=0.9, drone_id=0, sim_time=1.0, confirmation_count=4.5)
        assert False, "TrackedVictim must reject fractional confirmation_count"
    except ValueError:
        pass


def test_null_thermal_metadata_preservation():
    vm, _, _ = create_test_victim_manager()
    # Initial detection from RGB-D with None thermal stats
    tid, _ = vm.process_discovery_event(
        world_x=22.0,
        world_y=24.0,
        source="rgbd_localizer",
        confidence=0.9,
        drone_id=1,
        sim_time=1.0,
        verification_mode="RGBD_GEOMETRIC",
        thermal_peak_k=None,
        thermal_delta_k=None
    )
    t = vm.tracked_victims[tid]
    assert t.thermal_peak_k is None
    assert t.thermal_delta_k is None
    d = t.get_dict()
    assert d["thermal_peak_k"] is None
    assert d["thermal_delta_k"] is None

    # Update with multispectral thermal data
    vm.process_discovery_event(
        world_x=22.2,
        world_y=24.1,
        source="multispectral_verifier",
        confidence=0.9,
        drone_id=1,
        sim_time=2.0,
        verification_mode="MULTISPECTRAL_FUSION",
        thermal_peak_k=307.5,
        thermal_delta_k=10.2
    )
    assert t.thermal_peak_k == 307.5
    assert t.thermal_delta_k == 10.2

    # Subsequent RGB-D detection with None thermal data must NOT overwrite existing thermal values
    vm.process_discovery_event(
        world_x=22.1,
        world_y=24.0,
        source="rgbd_localizer",
        confidence=0.9,
        drone_id=1,
        sim_time=3.0,
        verification_mode="RGBD_GEOMETRIC",
        thermal_peak_k=None,
        thermal_delta_k=None
    )
    assert t.thermal_peak_k == 307.5, "Valid thermal peak must be preserved when later detection lacks thermal"
    assert t.thermal_delta_k == 10.2, "Valid thermal delta must be preserved when later detection lacks thermal"


def test_source_and_verification_mode_distinct_metadata():
    vm, _, _ = create_test_victim_manager()
    tid, _ = vm.process_discovery_event(
        world_x=18.0,
        world_y=18.0,
        source="rgbd_auxiliary_localizer",
        confidence=0.9,
        drone_id=1,
        sim_time=1.0,
        verification_mode="RGBD_GEOMETRIC",
        source_track_id=101
    )
    t = vm.tracked_victims[tid]
    assert t.source == "rgbd_auxiliary_localizer"
    assert t.verification_mode == "RGBD_GEOMETRIC"
    assert t.source_track_id == 101
    assert t.source != t.verification_mode, "Source and verification_mode must not be conflated"

    d = t.get_dict()
    assert d["source"] == "rgbd_auxiliary_localizer"
    assert d["verification_mode"] == "RGBD_GEOMETRIC"
    assert d["source_track_id"] == 101


def test_ground_truth_victims_strict_isolation():
    vm, _, _ = create_test_victim_manager()
    gt_v0 = vm.victims["V0"]
    gt_v1 = vm.victims["V1"]
    assert gt_v0.state == VictimState.UNDETECTED
    assert gt_v1.state == VictimState.UNDETECTED

    # Ingest autonomous discoveries near V0
    for i in range(5):
        vm.process_discovery_event(world_x=20.0 + i * 0.1, world_y=20.0 + i * 0.1, confidence=0.9, drone_id=1, sim_time=float(i))

    assert gt_v0.state == VictimState.UNDETECTED, "Ground truth state must remain UNDETECTED"
    assert gt_v0.detected_by is None
    assert gt_v0.detection_time is None
    assert len(vm.victims) == 2, "Ground truth dictionary must remain strictly isolated"
    assert len(vm.tracked_victims) == 1, "Autonomous discoveries stored separately in tracked_victims"


def test_backward_compatibility_get_dict_and_dashboard_telemetry():
    vm, _, _ = create_test_victim_manager()
    vm.process_discovery_event(
        world_x=33.33,
        world_y=44.44,
        source="multispectral_verifier",
        confidence=0.912,
        drone_id=2,
        sim_time=25.5,
        source_track_id=5,
        verification_mode="MULTISPECTRAL_FUSION",
        thermal_peak_k=308.5,
        thermal_delta_k=12.1
    )
    t = vm.tracked_victims["victim_1"]
    d = t.get_dict()

    # Verify all 13 legacy fields
    required_legacy_keys = [
        "id", "x", "y", "world_x", "world_y", "detected", "state",
        "source", "confidence", "detected_by", "last_seen_sec_ago",
        "gt_error", "observations"
    ]
    for k in required_legacy_keys:
        assert k in d, f"Missing required legacy key '{k}'"

    assert isinstance(d["id"], str)
    assert isinstance(d["x"], int)
    assert isinstance(d["y"], int)
    assert isinstance(d["world_x"], float)
    assert isinstance(d["world_y"], float)
    assert d["detected"] is True
    assert d["state"] == "DETECTED"
    assert d["confidence"] == 0.912
    assert d["detected_by"] == 2
    assert isinstance(d["last_seen_sec_ago"], float)
    assert d["observations"] == 1

    # Verify new metadata keys
    assert d["sim_time"] == 25.5
    assert d["first_seen_sim"] == 25.5
    assert d["last_seen_sim"] == 25.5
    assert d["source_track_id"] == 5
    assert d["verification_mode"] == "MULTISPECTRAL_FUSION"
    assert d["thermal_peak_k"] == 308.5
    assert d["thermal_delta_k"] == 12.1

    # Test JSON serialization
    serialized = json.dumps(d)
    assert len(serialized) > 0
    deserialized = json.loads(serialized)
    assert deserialized["id"] == "victim_1"


def test_association_radius_configuration_and_fallback():
    # 1. Valid custom constructor parameter
    vm1, _, _ = create_test_victim_manager(auto_radius=8.5)
    assert vm1.autonomous_association_radius == 8.5

    # 2. Invalid non-numeric env var fallback to 6.0
    os.environ["AUTONOMOUS_ASSOCIATION_RADIUS"] = "invalid_number"
    vm2, _, _ = create_test_victim_manager()
    assert vm2.autonomous_association_radius == 6.0

    # 3. Invalid negative env var fallback to 6.0
    os.environ["AUTONOMOUS_ASSOCIATION_RADIUS"] = "-4.0"
    vm3, _, _ = create_test_victim_manager()
    assert vm3.autonomous_association_radius == 6.0

    # 4. Valid env var
    os.environ["AUTONOMOUS_ASSOCIATION_RADIUS"] = "7.2"
    vm4, _, _ = create_test_victim_manager()
    assert vm4.autonomous_association_radius == 7.2

    # Reset env var
    os.environ.pop("AUTONOMOUS_ASSOCIATION_RADIUS", None)


def test_qmix_discovery_callback_end_to_end():
    from swarm_controller.qmix_mission_controller import QMIXMissionController
    
    # Minimal mock mission controller setup
    class DummyController:
        def __init__(self):
            meta = {"victims": []}
            self.victim_manager = VictimManager(
                node=MockNode(),
                metadata=meta,
                world_name="test_world",
                env=MockEnv(),
                seed=42
            )
            self._logger = MockLogger()

        def get_logger(self):
            return self._logger

    dc = DummyController()
    
    # Test valid String message via QMIXMissionController.discovery_callback
    valid_payload = {
        "event_type": "VICTIM_DISCOVERED",
        "timestamp": 42.0,
        "victim_track_id": 3,
        "confirming_drone_id": 1,
        "world_x": 48.0,
        "world_y": 52.0,
        "confidence": 0.93,
        "verification_mode": "MULTISPECTRAL_FUSION",
        "thermal_peak_k": 309.1,
        "thermal_delta_k": 12.5,
        "confirmation_count": 4,
        "confirmation_window": 10,
        "consensus_span_sec": 1.4,
        "median_depth_estimate": 7.2,
        "depth_valid_samples": 300,
        "depth_spread": 0.18
    }
    msg = String()
    msg.data = json.dumps(valid_payload)
    
    QMIXMissionController.discovery_callback(dc, msg)
    assert len(dc.victim_manager.tracked_victims) == 1
    t = dc.victim_manager.tracked_victims["victim_1"]
    assert t.world_x == 48.0
    assert t.world_y == 52.0
    assert t.thermal_peak_k == 309.1
    assert t.source_track_id == 3
    assert t.source == "UNKNOWN_AUTONOMOUS_SOURCE", "Publisher payload without source must yield UNKNOWN_AUTONOMOUS_SOURCE"

    # Test malformed / invalid messages (must not raise exception)
    malformed_msg = String()
    malformed_msg.data = "NOT_A_JSON"
    QMIXMissionController.discovery_callback(dc, malformed_msg)

    wrong_event_msg = String()
    wrong_event_msg.data = json.dumps({"event_type": "OTHER_EVENT", "world_x": 10, "world_y": 10})
    QMIXMissionController.discovery_callback(dc, wrong_event_msg)

    neg_ts_msg = String()
    neg_ts_msg.data = json.dumps({"event_type": "VICTIM_DISCOVERED", "timestamp": -2.0, "world_x": 10, "world_y": 10})
    QMIXMissionController.discovery_callback(dc, neg_ts_msg)

    assert len(dc.victim_manager.tracked_victims) == 1, "Malformed messages must be safely dropped"


def test_legacy_positional_signature_compatibility():
    vm, _, _ = create_test_victim_manager()
    # Call legacy positional: (track_id, world_x, world_y, source, confidence, drone_id)
    tid, is_new = vm.process_discovery_event(99, 16.5, 26.5, "rgbd_legacy", 0.85, 2)
    assert is_new is True
    assert tid == "victim_1"
    t = vm.tracked_victims["victim_1"]
    assert t.world_x == 16.5
    assert t.world_y == 26.5
    assert t.source == "rgbd_legacy"
    assert t.confidence == 0.85
    assert t.detected_by == 2
    assert t.source_track_id == 99


def test_atomic_tracked_victim_updates():
    """
    Issue 1: Verifies atomic TrackedVictim updates.
    If any incoming mandatory or optional field fails validation,
    the update is rejected and EVERY existing attribute remains completely unchanged.
    """
    track = TrackedVictim(
        t_id="victim_atomic_test",
        world_x=20.0,
        world_y=25.0,
        source="rgb_thermal_initial",
        confidence=0.85,
        drone_id=1,
        sim_time=5.0,
        source_track_id=10,
        verification_mode="MULTISPECTRAL_FUSION",
        evidence_state="TEMPORALLY_CONFIRMED",
        thermal_peak_k=308.5,
        thermal_delta_k=11.2,
        confirmation_count=4,
        confirmation_window=10,
        consensus_span_sec=1.2,
        median_depth_estimate=6.5,
        depth_valid_samples=250,
        depth_spread=0.15
    )

    def snapshot(t):
        return {
            "id": t.id,
            "world_x": t.world_x,
            "world_y": t.world_y,
            "grid_x": t.grid_x,
            "grid_y": t.grid_y,
            "source": t.source,
            "confidence": t.confidence,
            "detected_by": t.detected_by,
            "sim_time": t.sim_time,
            "first_seen_sim": t.first_seen_sim,
            "last_seen_sim": t.last_seen_sim,
            "status": t.status,
            "observations": t.observations,
            "source_track_id": t.source_track_id,
            "verification_mode": t.verification_mode,
            "evidence_state": t.evidence_state,
            "thermal_peak_k": t.thermal_peak_k,
            "thermal_delta_k": t.thermal_delta_k,
            "confirmation_count": t.confirmation_count,
            "confirmation_window": t.confirmation_window,
            "consensus_span_sec": t.consensus_span_sec,
            "median_depth_estimate": t.median_depth_estimate,
            "depth_valid_samples": t.depth_valid_samples,
            "depth_spread": t.depth_spread
        }

    baseline = snapshot(track)

    # 1. Attempt update with invalid confidence (> 1.0, negative, NaN, Inf, bool)
    for bad_conf in [1.5, -0.2, float('nan'), float('inf'), True]:
        try:
            track.update(world_x=30.0, world_y=35.0, confidence=bad_conf, drone_id=2, sim_time=6.0)
            assert False, f"Expected ValueError for bad confidence {bad_conf}"
        except (ValueError, TypeError):
            pass
        assert snapshot(track) == baseline, f"Track state modified after bad confidence {bad_conf}!"

    # 2. Attempt update with non-finite or negative simulation timestamp
    for bad_st in [float('nan'), float('inf'), -1.0, bool]:
        try:
            track.update(world_x=30.0, world_y=35.0, confidence=0.9, drone_id=2, sim_time=bad_st)
            assert False, f"Expected ValueError for bad sim_time {bad_st}"
        except (ValueError, TypeError):
            pass
        assert snapshot(track) == baseline, f"Track state modified after bad sim_time {bad_st}!"

    # 3. Attempt update with invalid coordinates (NaN, Inf)
    for bad_coord in [float('nan'), float('inf')]:
        try:
            track.update(world_x=bad_coord, world_y=35.0, confidence=0.9, drone_id=2, sim_time=6.0)
            assert False, f"Expected ValueError for bad world_x {bad_coord}"
        except (ValueError, TypeError):
            pass
        assert snapshot(track) == baseline, f"Track state modified after bad world_x {bad_coord}!"

        try:
            track.update(world_x=30.0, world_y=bad_coord, confidence=0.9, drone_id=2, sim_time=6.0)
            assert False, f"Expected ValueError for bad world_y {bad_coord}"
        except (ValueError, TypeError):
            pass
        assert snapshot(track) == baseline, f"Track state modified after bad world_y {bad_coord}!"

    # 4. Attempt update with invalid optional thermal metadata (negative thermal peak, NaN, Inf)
    for bad_th in [-5.0, float('nan'), float('inf')]:
        try:
            track.update(world_x=30.0, world_y=35.0, confidence=0.9, drone_id=2, sim_time=6.0, thermal_peak_k=bad_th)
            assert False, f"Expected ValueError for bad thermal_peak_k {bad_th}"
        except (ValueError, TypeError):
            pass
        assert snapshot(track) == baseline, f"Track state modified after bad thermal_peak_k {bad_th}!"

    # 5. Attempt update with invalid optional depth metadata (negative depth, NaN, Inf)
    for bad_depth in [-2.0, float('nan'), float('inf')]:
        try:
            track.update(world_x=30.0, world_y=35.0, confidence=0.9, drone_id=2, sim_time=6.0, median_depth_estimate=bad_depth)
            assert False, f"Expected ValueError for bad median_depth_estimate {bad_depth}"
        except (ValueError, TypeError):
            pass
        assert snapshot(track) == baseline, f"Track state modified after bad median_depth_estimate {bad_depth}!"

    # 6. Attempt update with fractional optional integer (e.g. 4.5, negative, bool)
    for bad_int in [4.5, -3, True]:
        try:
            track.update(world_x=30.0, world_y=35.0, confidence=0.9, drone_id=2, sim_time=6.0, confirmation_count=bad_int)
            assert False, f"Expected ValueError for bad confirmation_count {bad_int}"
        except (ValueError, TypeError):
            pass
        assert snapshot(track) == baseline, f"Track state modified after bad confirmation_count {bad_int}!"

    # 7. Valid update succeeds cleanly
    track.update(world_x=22.0, world_y=27.0, confidence=0.92, drone_id=2, sim_time=7.0)
    assert track.observations == 2
    assert track.confidence == 0.92
    assert track.detected_by == 2
    assert track.sim_time == 7.0
    # EMA check: 20.0 * 0.3 + 22.0 * 0.7 = 21.4
    assert abs(track.world_x - 21.4) < 1e-3
    assert abs(track.world_y - 26.4) < 1e-3


def test_out_of_order_simulation_timestamps():
    """
    Issue 2: Verifies handling of out-of-order and equal simulation timestamps.
    - An older event (sim_time < last_seen_sim) does NOT regress last_seen_sim or sim_time.
    - Earliest simulation timestamp (first_seen_sim) updates to min(first_seen_sim, sim_time).
    - Position smoothing (EMA alpha=0.7) and monotonic confidence max are applied.
    - Observation count increments for every sighting.
    - Metadata from newer event is retained (not overwritten by older event).
    - Equal timestamps are handled deterministically.
    """
    vm, _, _ = create_test_victim_manager(auto_radius=6.0)

    # 1. First event at sim_time = 10.0
    tid1, is_new1 = vm.process_discovery_event(
        world_x=20.0,
        world_y=20.0,
        source="drone_1_sensor",
        confidence=0.80,
        drone_id=1,
        sim_time=10.0,
        source_track_id=101,
        verification_mode="RGBD"
    )
    assert is_new1 is True
    track = vm.tracked_victims[tid1]
    assert track.first_seen_sim == 10.0
    assert track.last_seen_sim == 10.0
    assert track.sim_time == 10.0
    assert track.observations == 1
    assert track.confidence == 0.80
    assert track.detected_by == 1
    assert track.source == "drone_1_sensor"
    assert track.verification_mode == "RGBD"
    assert track.source_track_id == 101

    # 2. Delayed / out-of-order event at sim_time = 8.0 arrives (e.g. from Drone 2)
    # Distance: (21.0 - 20.0)^2 + (21.0 - 20.0)^2 = 2.0 <= 36.0 (dist ~ 1.41m <= 6.0m)
    tid2, is_new2 = vm.process_discovery_event(
        world_x=21.0,
        world_y=21.0,
        source="drone_2_sensor",
        confidence=0.92,
        drone_id=2,
        sim_time=8.0,
        source_track_id=202,
        verification_mode="MULTISPECTRAL_FUSION",
        thermal_peak_k=307.5
    )
    assert is_new2 is False, "Must associate with existing track victim_1"
    assert tid2 == tid1

    # Timestamp regression check: last_seen_sim and sim_time MUST NOT regress backwards
    assert track.last_seen_sim == 10.0, f"last_seen_sim regressed! Expected 10.0, got {track.last_seen_sim}"
    assert track.sim_time == 10.0, f"sim_time regressed! Expected 10.0, got {track.sim_time}"

    # Earliest simulation timestamp updates to 8.0
    assert track.first_seen_sim == 8.0, f"first_seen_sim expected 8.0, got {track.first_seen_sim}"

    # Observation count increments
    assert track.observations == 2, f"observations expected 2, got {track.observations}"

    # Confidence takes maximum
    assert track.confidence == 0.92, f"confidence expected 0.92, got {track.confidence}"

    # Position smoothing applies: 20.0 * 0.3 + 21.0 * 0.7 = 20.7
    assert abs(track.world_x - 20.7) < 1e-3
    assert abs(track.world_y - 20.7) < 1e-3

    # Metadata policy: active metadata anchored to the newer event (drone 1 at 10.0s),
    # while previously unpopulated fields (thermal_peak_k) are backfilled
    assert track.detected_by == 1, "Latest reporting drone in simulation time (Drone 1 at t=10.0) should be retained"
    assert track.source == "drone_1_sensor"
    assert track.thermal_peak_k == 307.5, "Previously null thermal metadata should be backfilled from older event"

    # 3. Equal timestamp event at sim_time = 10.0 arrives (from Drone 3)
    tid3, is_new3 = vm.process_discovery_event(
        world_x=20.7,
        world_y=20.7,
        source="drone_3_sensor",
        confidence=0.88,
        drone_id=3,
        sim_time=10.0,
        source_track_id=303,
        verification_mode="MULTISPECTRAL_FUSION"
    )
    assert is_new3 is False
    assert tid3 == tid1
    assert track.last_seen_sim == 10.0
    assert track.sim_time == 10.0
    assert track.first_seen_sim == 8.0
    assert track.observations == 3
    assert track.confidence == 0.92  # max(0.92, 0.88)
    assert track.detected_by == 3   # equal timestamp: updated to latest reporting drone
    assert track.source == "drone_3_sensor"


def test_qmix_mission_controller_runtime_import_and_entrypoint():
    """
    Issue 3: Verifies that qmix_mission_controller properly imports rclpy,
    provides an executable main() entry point, and resolves all names cleanly at runtime.
    """
    import swarm_controller.qmix_mission_controller as qmc

    # 1. Verify import rclpy is present at runtime
    assert hasattr(qmc, 'rclpy'), "qmix_mission_controller must import rclpy"
    assert qmc.rclpy is not None

    # 2. Verify main() entry point is defined and callable
    assert hasattr(qmc, 'main'), "qmix_mission_controller must define a main() entry point"
    assert callable(qmc.main), "main must be callable"

    # 3. Verify rclpy name resolution and lifecycle execution
    # Initialize and shutdown rclpy headlessly to verify no unresolved symbols or NameErrors
    was_ok = qmc.rclpy.ok()
    if not was_ok:
        qmc.rclpy.init()
    assert qmc.rclpy.ok() is True

    # Check that QMIXMissionController class is accessible and subclass of Node
    from rclpy.node import Node
    assert issubclass(qmc.QMIXMissionController, Node)

    if not was_ok:
        qmc.rclpy.shutdown()
        assert qmc.rclpy.ok() is False


def main():
    print("=====================================================================")
    print("Running VictimManager Autonomous Discovery Ingestion Test Suite (Phase 3A)")
    print("=====================================================================")
    tests = [
        ("Valid Autonomous Discovery Ingestion", test_valid_autonomous_discovery_ingestion),
        ("Source-Local Track ID Collision Decoupling", test_source_local_track_id_collision_decoupling),
        ("Multi-Drone Same-Victim Association and Smoothing", test_multidrone_same_victim_association_and_smoothing),
        ("Autonomous Association Radius Boundary", test_autonomous_association_radius_boundary),
        ("Ambiguity Contention Conservative Handling", test_ambiguity_contention_conservative_handling),
        ("Ineligible Candidate Does Not Cause Ambiguity Contention", test_ineligible_candidate_does_not_cause_ambiguity_contention),
        ("Simulation Timestamp Zero Accepted", test_sim_timestamp_zero_accepted),
        ("Negative and Non-Finite Inputs Rejected", test_negative_and_nonfinite_inputs_rejected),
        ("Missing Mandatory Fields Rejected", test_missing_mandatory_fields_rejected),
        ("Absent Source Metadata Handled Without Fabrication", test_absent_source_metadata_handled_without_fabrication),
        ("Invalid Optional Numeric Fields Rejected", test_invalid_optional_numeric_fields_rejected),
        ("Fractional Optional Integer Fields Rejected", test_fractional_optional_integer_fields_rejected),
        ("Null Thermal Metadata Preservation", test_null_thermal_metadata_preservation),
        ("Source and Verification Mode Distinct Metadata", test_source_and_verification_mode_distinct_metadata),
        ("Ground Truth Victims Strict Isolation", test_ground_truth_victims_strict_isolation),
        ("Backward Compatibility for get_dict and Telemetry", test_backward_compatibility_get_dict_and_dashboard_telemetry),
        ("Association Radius Configuration and Fallback", test_association_radius_configuration_and_fallback),
        ("QMIX Controller Discovery Callback End-to-End", test_qmix_discovery_callback_end_to_end),
        ("Legacy Positional Signature Compatibility", test_legacy_positional_signature_compatibility),
        ("Atomic TrackedVictim Updates (Issue 1)", test_atomic_tracked_victim_updates),
        ("Out-of-Order Simulation Timestamps (Issue 2)", test_out_of_order_simulation_timestamps),
        ("QMIX Controller Runtime Import and Entrypoint (Issue 3)", test_qmix_mission_controller_runtime_import_and_entrypoint),
    ]

    passed = 0
    failed = 0
    for name, fn in tests:
        if run_test(name, fn):
            passed += 1
        else:
            failed += 1

    print("=====================================================================")
    print(f"Test Summary: {passed}/{len(tests)} passed, {failed} failed.")
    print("=====================================================================")

    if failed > 0:
        sys.exit(1)
    else:
        sys.exit(0)


if __name__ == "__main__":
    main()

