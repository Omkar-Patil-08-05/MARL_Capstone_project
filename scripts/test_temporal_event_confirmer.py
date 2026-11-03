#!/usr/bin/env python3
"""
Unit Test Suite for Shared TemporalEventConfirmer Module (Phase 2 Remediation - Stage 1)

Verifies:
  1. Multiple detections from a single frame cannot satisfy temporal consensus
  2. Four distinct camera frames spanning >= min_time_span confirm a victim
  3. Two drones cross-associating observations into a unified consensus track
  4. Moving victim continuous trajectory tracking without spawning duplicate track IDs
  5. Spatial deduplication preventing duplicate discoveries near confirmed victims
  6. Temporary loss of visibility (occlusion coasting) and recovery
  7. Repeated observations after confirmation do not fire duplicate discovery events
"""

import os
import sys
import math
import numpy as np

# Ensure workspace packages can be imported
ws_src = os.path.abspath(os.path.join(os.path.dirname(__file__), '../drone_ws/src/swarm_controller'))
if ws_src not in sys.path:
    sys.path.insert(0, ws_src)

from swarm_controller.temporal_event_confirmer import TemporalEventConfirmer
from swarm_controller.grid_world_transform import GridWorldTransform


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
        return False


def test_multiple_detections_from_single_frame():
    """
    CRITICAL AUDIT FIX:
    Multiple YOLO detections in the exact same frame (e.g. 4 detections at t=1.0s)
    must NOT satisfy consensus. Only 1 observation per camera frame is credited.
    """
    confirmer = TemporalEventConfirmer(
        window_size=6,
        consensus_needed=4,
        min_time_span=0.4,
        proximity_threshold=3.0
    )

    t_frame = 1.000
    drone_id = 0

    # 4 detections in the exact same camera frame (e.g., body parts or candidate bboxes)
    # in the previous buggy code, this would immediately confirm a victim!
    tid1, evt1 = confirmer.add_observation(drone_id, t_frame, 10.0, 10.0, 0.70, 8.0)
    tid2, evt2 = confirmer.add_observation(drone_id, t_frame, 10.2, 10.1, 0.75, 8.1)
    tid3, evt3 = confirmer.add_observation(drone_id, t_frame,  9.9, 10.0, 0.65, 8.0)
    tid4, evt4 = confirmer.add_observation(drone_id, t_frame, 10.1,  9.9, 0.82, 8.0)

    assert tid1 == tid2 == tid3 == tid4, f"All detections should associate to track {tid1}"
    assert evt1 is None, "Frame 1 should not trigger discovery"
    assert evt2 is None, "Duplicate frame 1 detection 2 must not trigger discovery"
    assert evt3 is None, "Duplicate frame 1 detection 3 must not trigger discovery"
    assert evt4 is None, "Duplicate frame 1 detection 4 must not trigger discovery"

    track = confirmer.tracks[tid1]
    # Check that only 1 unique frame is credited in seen_frames
    assert len(track['seen_frames']) == 1, f"Expected 1 unique frame, got {len(track['seen_frames'])}"
    # History contains 1 observation record (the highest confidence 0.82)
    assert len(track['history']) == 1, f"Expected 1 observation record, got {len(track['history'])}"
    assert np.isclose(track['history'][0]['conf'], 0.82), "Expected updated highest confidence 0.82"
    assert track['discovered'] is False, "Track must not be discovered from a single frame"


def test_four_distinct_frames_consensus():
    """
    Four distinct frames over >= 0.4s must achieve consensus and emit VICTIM_DISCOVERED.
    """
    confirmer = TemporalEventConfirmer(
        window_size=6,
        consensus_needed=4,
        min_time_span=0.4,
        proximity_threshold=3.0
    )

    drone_id = 0
    t_stamps = [1.0, 1.25, 1.50, 1.75] # Span = 0.75s >= 0.4s

    events = []
    tids = []
    for t in t_stamps:
        tid, evt = confirmer.add_observation(drone_id, t, 15.0, 25.0, 0.80, 10.0)
        tids.append(tid)
        events.append(evt)

    # First 3 frames should not trigger event
    assert events[0] is None
    assert events[1] is None
    assert events[2] is None

    # 4th frame should trigger VICTIM_DISCOVERED
    assert events[3] is not None, "4th unique frame must trigger VICTIM_DISCOVERED"
    evt = events[3]
    assert evt["event_type"] == "VICTIM_DISCOVERED"
    assert evt["victim_track_id"] == tids[0]
    assert evt["confirming_drone_id"] == drone_id
    assert abs(evt["world_x"] - 15.0) < 0.1
    assert abs(evt["world_y"] - 25.0) < 0.1
    assert evt["confirmation_count"] == 4
    assert evt["consensus_span_sec"] >= 0.4
    assert len(confirmer.confirmed_victims) == 1


def test_two_drones_cross_drone_consensus():
    """
    Two drones observing the same victim from different perspectives fuse observations.
    Drone 0 observes at t=1.0, 1.4s.
    Drone 1 observes at t=1.2, 1.6s.
    Combined: 4 unique frames spanning 0.6s -> Confirmed!
    """
    confirmer = TemporalEventConfirmer(
        window_size=6,
        consensus_needed=4,
        min_time_span=0.4,
        proximity_threshold=3.0
    )

    # Frame 1: Drone 0
    tid1, evt1 = confirmer.add_observation(0, 1.0, 30.0, 40.0, 0.78, 9.0)
    # Frame 2: Drone 1 (slightly noisy coordinate, e.g. 0.4m difference)
    tid2, evt2 = confirmer.add_observation(1, 1.2, 30.3, 39.8, 0.82, 9.5)
    # Frame 3: Drone 0
    tid3, evt3 = confirmer.add_observation(0, 1.4, 30.1, 40.1, 0.76, 9.0)
    # Frame 4: Drone 1
    tid4, evt4 = confirmer.add_observation(1, 1.6, 29.9, 40.0, 0.85, 9.5)

    assert tid1 == tid2 == tid3 == tid4, f"Observations from both drones must fuse into same track: {tid1} vs {tid2}"
    assert evt1 is None and evt2 is None and evt3 is None
    assert evt4 is not None, "Cross-drone 4th observation must confirm victim"
    assert evt4["confirming_drone_id"] == 1
    assert evt4["confirmation_count"] == 4


def test_moving_victim_tracking_without_duplicate_id():
    """
    CRITICAL AUDIT FIX:
    A mobile victim moves at 1.0 m/s over 11 seconds (total displacement = 11m).
    In the old code, when displacement exceeded dedup_radius (6m), a second track ID was created.
    With TemporalEventConfirmer, the moving centroid updates dynamically, maintaining track ID.
    """
    confirmer = TemporalEventConfirmer(
        window_size=6,
        consensus_needed=4,
        min_time_span=0.4,
        proximity_threshold=3.0,
        dedup_radius=6.0,
        max_victim_speed_m_s=1.5
    )

    # Steps: victim moves along +X at ~1.0 m/s
    path = [
        (1.0, 50.0, 50.0),
        (2.0, 51.0, 50.0),
        (3.0, 52.0, 50.0),
        (4.0, 53.0, 50.0), # Frame 4: Confirmed near (51.5, 50.0)
        (7.0, 56.0, 50.0), # Keeps moving
        (10.0, 59.0, 50.0), # Total displacement from start = 9m (> 6m dedup radius!)
        (12.0, 61.0, 50.0)
    ]

    tids = []
    events = []
    for t, x, y in path:
        tid, evt = confirmer.add_observation(0, t, x, y, 0.80, 8.0)
        tids.append(tid)
        events.append(evt)

    # All observations must belong to the exact same track ID 1
    assert all(tid == tids[0] for tid in tids), f"Track ID drifted! Got {set(tids)}"
    assert events[3] is not None, "Should confirm on 4th observation"

    # Subsequent observations must NOT emit duplicate discovery events
    assert events[4] is None
    assert events[5] is None
    assert events[6] is None

    # Exactly 1 confirmed victim in registry, and its location updated to the moving position
    assert len(confirmer.confirmed_victims) == 1
    conf_v = confirmer.confirmed_victims[0]
    assert conf_v['track_id'] == tids[0]
    # The confirmed coordinate followed the victim past the 6m dedup radius (moved from 50m to >56m)
    assert conf_v['x'] > 56.0, f"Confirmed position did not follow moving victim: x={conf_v['x']}"


def test_duplicate_detection_rejection():
    """
    Observations near an already confirmed victim must be suppressed.
    """
    confirmer = TemporalEventConfirmer(consensus_needed=4, dedup_radius=6.0)

    # Confirm victim 1 at (10.0, 10.0)
    for t in [1.0, 1.3, 1.6, 1.9]:
        confirmer.add_observation(0, t, 10.0, 10.0, 0.85, 10.0)
    assert len(confirmer.confirmed_victims) == 1

    # Candidate 2: Drone 2 sees candidate 2.5m away at t=3.0s (within 6m dedup radius)
    tid_near, evt_near = confirmer.add_observation(2, 3.0, 11.5, 11.5, 0.80, 10.0)
    # Candidate should associate with victim 1 or be suppressed without duplicate event
    assert evt_near is None
    assert len(confirmer.confirmed_victims) == 1


def test_temporary_loss_of_visibility_and_recovery():
    """
    Victim is seen at t=1.0, 1.2s, then occluded for 3.0s (t=4.2s), then seen at t=4.4, 4.6s.
    Coasting tracker retains candidate track and achieves consensus upon reappearance.
    """
    confirmer = TemporalEventConfirmer(
        window_size=6,
        consensus_needed=4,
        min_time_span=0.4,
        max_track_coasting_sec=15.0
    )

    tid1, _ = confirmer.add_observation(0, 1.0, 20.0, 20.0, 0.75, 12.0)
    tid2, _ = confirmer.add_observation(0, 1.2, 20.1, 19.9, 0.78, 12.0)

    # 3.0 second occlusion gap (e.g. tree canopy)
    tid3, _ = confirmer.add_observation(0, 4.2, 20.0, 20.1, 0.80, 12.0)
    tid4, evt4 = confirmer.add_observation(0, 4.4, 19.9, 20.0, 0.82, 12.0)

    assert tid1 == tid2 == tid3 == tid4, "Track must survive occlusion coasting"
    assert evt4 is not None, "Consensus must be achieved upon acquiring 4th frame"
    assert evt4["confirmation_count"] == 4


def test_repeated_observations_after_confirmation():
    """
    Hovering drone sending continuous observations after confirmation.
    Zero duplicate events must ever be fired.
    """
    confirmer = TemporalEventConfirmer(consensus_needed=4)

    # Confirm victim at (70.0, 70.0)
    for t in [1.0, 1.2, 1.4, 1.6]:
        confirmer.add_observation(0, t, 70.0, 70.0, 0.85, 10.0)

    assert len(confirmer.confirmed_victims) == 1

    # Drone continues hovering over victim for 10 frames
    duplicate_events = []
    for t in np.linspace(1.8, 4.0, 10):
        _, evt = confirmer.add_observation(0, float(t), 70.0 + np.random.uniform(-0.2, 0.2), 70.0, 0.85, 10.0)
        if evt is not None:
            duplicate_events.append(evt)

    assert len(duplicate_events) == 0, f"Expected 0 duplicate events, got {len(duplicate_events)}"
    assert len(confirmer.confirmed_victims) == 1


def main():
    print("=" * 75)
    print("PHASE 2 REMEDIATION: STAGE 1 TEMPORAL EVENT CONFIRMER TESTS")
    print("=" * 75)

    tests = [
        ("Single-Frame Multiple Detections Rejection", test_multiple_detections_from_single_frame),
        ("Four Distinct Frames Temporal Consensus", test_four_distinct_frames_consensus),
        ("Two Drones Cross-Drone Track Association", test_two_drones_cross_drone_consensus),
        ("Mobile Victim Trajectory Tracking (No Duplicate IDs)", test_moving_victim_tracking_without_duplicate_id),
        ("Spatial Deduplication Suppression Near Confirmed Victims", test_duplicate_detection_rejection),
        ("Temporary Occlusion Coasting & Re-acquisition", test_temporary_loss_of_visibility_and_recovery),
        ("Post-Confirmation Repeated Observation Suppression", test_repeated_observations_after_confirmation),
    ]

    passed = 0
    for name, fn in tests:
        if run_test(name, fn):
            passed += 1

    print("=" * 75)
    print(f"RESULTS: {passed}/{len(tests)} TESTS PASSED")
    print("=" * 75)
    return 0 if passed == len(tests) else 1


if __name__ == '__main__':
    sys.exit(main())
