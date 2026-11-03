#!/usr/bin/env python3
"""
Multi-Drone Evaluation & Temporal Consensus Test Suite (Phase 2 Remediation - Stage 5)

Evaluates the multi-drone temporal tracking subsystem across:
  1. Single-drone repeated observations (frame uniqueness rejection)
  2. Multi-drone collaborative consensus (Drone 0 & Drone 1 reach consensus)
  3. Duplicate discovery suppression (no re-discovery within dedup radius)
  4. Spatially distinct nearby victims (distinct IDs for victims separated > dedup_radius)
  5. Out-of-order timestamp handling and multi-drone arrival jitter
  6. Mobile victim kinematic trajectory tracking without track fragmentation
  7. Asynchronous observation rates across swarm (5 Hz vs 2 Hz)
"""

import math
from typing import Dict, Any, List

from swarm_controller.temporal_event_confirmer import TemporalEventConfirmer
from swarm_controller.grid_world_transform import GridWorldTransform


class MultiDroneEvaluator:
    """Rigorous evaluation of multi-drone spatio-temporal consensus."""

    @staticmethod
    def run_all_tests() -> Dict[str, Any]:
        tests = [
            ("Single Drone Repeated Frame Uniqueness", MultiDroneEvaluator.test_single_drone_repeated_frames),
            ("Cross-Drone Collaborative Consensus", MultiDroneEvaluator.test_cross_drone_consensus),
            ("Post-Confirmation Duplicate Discovery Suppression", MultiDroneEvaluator.test_duplicate_discovery_suppression),
            ("Distinct Nearby Victims (>6m Separation)", MultiDroneEvaluator.test_distinct_nearby_victims),
            ("Out-of-Order Multi-Drone Arrival Jitter", MultiDroneEvaluator.test_out_of_order_timestamps),
            ("Mobile Victim Dynamic Kinematic Tracking", MultiDroneEvaluator.test_mobile_victim_tracking),
            ("Asynchronous Observation Rates Across Swarm", MultiDroneEvaluator.test_asynchronous_observation_rates)
        ]

        results = {}
        all_passed = True
        for name, fn in tests:
            try:
                res = fn()
                results[name] = {"passed": True, "details": res}
            except AssertionError as e:
                results[name] = {"passed": False, "error": f"AssertionError: {e}"}
                all_passed = False
            except Exception as e:
                results[name] = {"passed": False, "error": f"Exception: {e}"}
                all_passed = False

        return {
            "all_passed": all_passed,
            "total_tests": len(tests),
            "passed_tests": sum(1 for r in results.values() if r["passed"]),
            "test_results": results
        }

    @staticmethod
    def test_single_drone_repeated_frames() -> str:
        """Frame uniqueness guarantees 10 detections from the exact same timestamp cannot satisfy consensus."""
        confirmer = TemporalEventConfirmer(window_size=6, consensus_needed=4, min_time_span=0.4)
        ts = 10.0
        events = []
        for i in range(10):
            tid, evt = confirmer.add_observation(
                drone_id=0, timestamp=ts, world_x=20.0, world_y=50.0,
                confidence=0.85, median_depth=10.0
            )
            if evt:
                events.append(evt)

        assert len(events) == 0, f"Expected 0 events from identical timestamp, got {len(events)}"
        assert len(confirmer.tracks[1]['history']) == 1, "Track history must only retain 1 observation for frame"
        return "10 detections from identical frame strictly collapsed to 1 observation; 0 spurious consensus events"

    @staticmethod
    def test_cross_drone_consensus() -> str:
        """Drone 0 and Drone 1 alternating observations seamlessly fuse into 1 victim track and reach consensus."""
        confirmer = TemporalEventConfirmer(window_size=6, consensus_needed=4, min_time_span=0.4)
        observations = [
            (0, 1.0, 20.0, 50.0),
            (1, 1.15, 20.2, 50.1),
            (0, 1.30, 20.1, 49.9),
            (1, 1.45, 19.9, 50.2),
        ]
        events = []
        for d_id, ts, x, y in observations:
            tid, evt = confirmer.add_observation(
                drone_id=d_id, timestamp=ts, world_x=x, world_y=y,
                confidence=0.88, median_depth=10.0
            )
            if evt:
                events.append(evt)

        assert len(events) == 1, f"Expected exactly 1 discovery event, got {len(events)}"
        assert events[0]["victim_track_id"] == 1
        assert events[0]["confirmation_count"] == 4
        assert events[0]["consensus_span_sec"] >= 0.4
        return "Drones 0 and 1 successfully reached cross-drone consensus at t=1.45s (span=0.45s)"

    @staticmethod
    def test_duplicate_discovery_suppression() -> str:
        """After confirmation, subsequent observations from any drone do NOT emit duplicate discovery events."""
        confirmer = TemporalEventConfirmer(window_size=6, consensus_needed=4, min_time_span=0.4, dedup_radius=6.0)
        # Establish confirmation
        for i in range(4):
            confirmer.add_observation(0, 1.0 + i * 0.15, 20.0, 50.0, 0.85, 10.0)

        assert len(confirmer.confirmed_victims) == 1

        # 10 subsequent observations from Drone 2, 3, 4 near the confirmed location
        suppressed_events = []
        for i in range(10):
            d_id = (i % 4) + 1
            tid, evt = confirmer.add_observation(
                drone_id=d_id, timestamp=2.0 + i * 0.2, world_x=20.5, world_y=50.3,
                confidence=0.90, median_depth=10.0
            )
            if evt:
                suppressed_events.append(evt)

        assert len(suppressed_events) == 0, f"Expected 0 duplicate events, got {len(suppressed_events)}"
        return "10 subsequent observations across 4 drones cleanly suppressed; 0 duplicate discovery events"

    @staticmethod
    def test_distinct_nearby_victims() -> str:
        """Victims separated by > dedup_radius (8m > 6m) correctly receive distinct track IDs and discovery events."""
        confirmer = TemporalEventConfirmer(window_size=6, consensus_needed=4, min_time_span=0.4, dedup_radius=6.0)
        # Victim A at (20.0, 50.0)
        events_a = []
        for i in range(4):
            tid_a, evt = confirmer.add_observation(0, 1.0 + i * 0.15, 20.0, 50.0, 0.85, 10.0)
            if evt:
                events_a.append(evt)

        # Victim B at (28.0, 50.0) - separated by 8.0m (> 6.0m)
        events_b = []
        for i in range(4):
            tid_b, evt = confirmer.add_observation(1, 1.1 + i * 0.15, 28.0, 50.0, 0.85, 10.0)
            if evt:
                events_b.append(evt)

        assert len(events_a) == 1, "Victim A must emit discovery"
        assert len(events_b) == 1, "Victim B must emit discovery"
        assert events_a[0]["victim_track_id"] != events_b[0]["victim_track_id"], "Must have distinct track IDs"
        assert len(confirmer.confirmed_victims) == 2, "Must record 2 confirmed victims"
        return "Victim A (track 1) and Victim B (track 2) separated by 8m correctly confirmed as distinct targets"

    @staticmethod
    def test_out_of_order_timestamps() -> str:
        """Arrival jitter with out-of-order timestamps is sorted properly and satisfies consensus span."""
        confirmer = TemporalEventConfirmer(window_size=6, consensus_needed=4, min_time_span=0.4)
        # Observations arrive out-of-order due to network latency: 1.3s, 1.0s, 1.45s, 1.15s
        jitters = [
            (0, 1.30, 20.1, 50.0),
            (1, 1.00, 20.0, 50.1),
            (0, 1.45, 20.2, 49.9),
            (1, 1.15, 20.0, 50.0)
        ]
        events = []
        for d, ts, x, y in jitters:
            tid, evt = confirmer.add_observation(d, ts, x, y, 0.85, 10.0)
            if evt:
                events.append(evt)

        assert len(events) == 1, f"Expected 1 discovery event from jittered frames, got {len(events)}"
        assert events[0]["consensus_span_sec"] == 0.45, f"Expected span 0.45s, got {events[0]['consensus_span_sec']}"
        return "Jittered frames [1.30, 1.00, 1.45, 1.15] correctly sorted to compute true consensus span of 0.45s"

    @staticmethod
    def test_mobile_victim_tracking() -> str:
        """A walking victim (1.0 m/s) dynamically updates track centroid without spawning duplicate IDs."""
        confirmer = TemporalEventConfirmer(window_size=6, consensus_needed=4, min_time_span=0.4, max_victim_speed_m_s=1.5)
        # Confirm victim starting at (20.0, 50.0)
        for i in range(4):
            confirmer.add_observation(0, 1.0 + i * 0.15, 20.0, 50.0, 0.85, 10.0)

        # Victim walks 8 meters eastward over 8 seconds: (20.0 -> 28.0)
        new_track_events = []
        for step in range(8):
            cur_x = 20.0 + (step + 1) * 1.0
            ts = 2.0 + step * 1.0
            tid, evt = confirmer.add_observation(1, ts, cur_x, 50.0, 0.85, 10.0)
            if evt:
                new_track_events.append(evt)
            assert tid == 1, f"Mobile observation must associate with track 1, got {tid}"

        assert len(new_track_events) == 0, "No duplicate events should be spawned while tracking mobile victim"
        assert confirmer.confirmed_victims[0]['x'] > 25.0, "Confirmed position estimate must track moving victim"
        return "Mobile victim moved 8m across 8s; tracked continuously under track 1 with 0 spurious track IDs"

    @staticmethod
    def test_asynchronous_observation_rates() -> str:
        """Drone 0 at 5 Hz (0.2s) and Drone 1 at 2 Hz (0.5s) correctly integrate."""
        confirmer = TemporalEventConfirmer(window_size=6, consensus_needed=4, min_time_span=0.4)
        obs = [
            (0, 1.0, 20.0, 50.0),
            (0, 1.2, 20.1, 50.0),
            (1, 1.5, 20.0, 50.1),  # 2 Hz drone
            (0, 1.6, 20.1, 49.9)
        ]
        events = []
        for d, ts, x, y in obs:
            tid, evt = confirmer.add_observation(d, ts, x, y, 0.85, 10.0)
            if evt:
                events.append(evt)

        assert len(events) == 1, "Asynchronous rates must satisfy consensus"
        assert events[0]["consensus_span_sec"] == 0.60
        return "Asynchronous streams (5 Hz + 2 Hz) integrated seamlessly over 0.60s span"
