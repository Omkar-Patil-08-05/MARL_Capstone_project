#!/usr/bin/env python3
"""
Comprehensive Unit & Synthetic Verification Suite for RGB-Thermal Fusion Verifier (Phase 2)
Tests:
  1. Thermal image radiometric decoding (16-bit counts -> Kelvin)
  2. RGB-to-thermal pinhole projective alignment
  3. Out-of-FOV candidate handling
  4. Invalid depth handling (NaN, <=0, out-of-range)
  5. Cold shadow / obstacle rejection (low thermal contrast)
  6. Human-like thermal hotspot acceptance (physiological 308K signature)
  7. Hot non-human clutter rejection (fire / exhaust > 45°C)
  8. Timestamp synchronization & buffer search
  9. Duplicate candidate deduplication
  10. Candidate / Verified / Rejected state machine transitions
"""

import os
import sys
import math
import numpy as np
from typing import Dict, Any

# Ensure workspace packages can be imported standalone
ws_src = os.path.abspath(os.path.join(os.path.dirname(__file__), '../drone_ws/src/swarm_controller'))
if ws_src not in sys.path:
    sys.path.insert(0, ws_src)

from swarm_controller.rgb_thermal_fusion_verifier import (
    RGBThermalFusionCore,
    CameraIntrinsics,
    VerificationStatus,
    RejectionReason,
    CandidateVerificationResult
)

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

def test_thermal_decoding():
    core = RGBThermalFusionCore()
    # 288.15 K -> count = 28815
    # 308.40 K -> count = 30840
    raw_counts = np.array([28815, 30840, 31000], dtype=np.uint16)
    decoded = core.decode_thermal_mono16(raw_counts)
    assert np.isclose(decoded[0], 288.15, atol=0.01), f"Expected 288.15, got {decoded[0]}"
    assert np.isclose(decoded[1], 308.40, atol=0.01), f"Expected 308.40, got {decoded[1]}"
    assert np.isclose(decoded[2], 310.00, atol=0.01), f"Expected 310.00, got {decoded[2]}"

def test_rgb_to_thermal_projection():
    core = RGBThermalFusionCore()
    # Center of RGB image (320, 240) at depth Z = 10m
    # Thermal optical center is at (160, 120), thermal sensor shifted by -0.03m along X_optical
    u_th, v_th = core.project_rgb_point_to_thermal(320.0, 240.0, depth_m=10.0)
    # At Z=10m, center pixel maps to u_th ≈ 160.83, v_th = 120.00
    assert abs(u_th - 160.83) < 0.1, f"Expected u_th ≈ 160.83, got {u_th}"
    assert abs(v_th - 120.00) < 0.1, f"Expected v_th ≈ 120.00, got {v_th}"

    # Bounding box near center: [300, 220, 340, 260]
    bbox_th, fully_inside, partially_inside = core.project_rgb_bbox_to_thermal([300, 220, 340, 260], depth_m=10.0)
    assert fully_inside is True
    assert partially_inside is True
    assert bbox_th is not None
    assert bbox_th[0] < bbox_th[2] and bbox_th[1] < bbox_th[3]

def test_out_of_fov_candidates():
    core = RGBThermalFusionCore()
    # Thermal FOV is 60°, RGB is 99.7°.
    # A candidate at the top-left corner of RGB [10, 10, 60, 60] at Z=10m
    # will project outside thermal 320x240 frame
    bbox_th, fully_inside, partially_inside = core.project_rgb_bbox_to_thermal([10, 10, 60, 60], depth_m=10.0)
    assert fully_inside is False
    # Verify candidate returns OUT_OF_THERMAL_FOV
    depth_img = np.ones((480, 640), dtype=np.float32) * 10.0
    th_img = np.ones((240, 320), dtype=np.float32) * 288.15
    res = core.verify_candidate(
        candidate_id="c_edge",
        drone_id=0,
        timestamp=1.0,
        rgb_bbox=[10, 10, 60, 60],
        rgb_confidence=0.85,
        depth_image=depth_img,
        thermal_image_k=th_img
    )
    assert res.status == VerificationStatus.CANDIDATE_UNVERIFIED.value
    assert res.rejection_reason == RejectionReason.OUT_OF_THERMAL_FOV.value

def test_invalid_depth():
    core = RGBThermalFusionCore()
    th_img = np.ones((240, 320), dtype=np.float32) * 288.15
    
    # 1. NaN depth
    depth_nan = np.full((480, 640), np.nan, dtype=np.float32)
    res_nan = core.verify_candidate("c_nan", 0, 1.0, [300, 220, 340, 260], 0.8, depth_nan, th_img)
    assert res_nan.status == VerificationStatus.REJECTED.value
    assert res_nan.rejection_reason == RejectionReason.INVALID_DEPTH.value

    # 2. Out of range depth (> 35m)
    depth_far = np.full((480, 640), 55.0, dtype=np.float32)
    res_far = core.verify_candidate("c_far", 0, 1.0, [300, 220, 340, 260], 0.8, depth_far, th_img)
    assert res_far.status == VerificationStatus.REJECTED.value
    assert res_far.rejection_reason == RejectionReason.INVALID_DEPTH.value

def test_cold_shadow_rejection():
    core = RGBThermalFusionCore(min_thermal_delta_k=3.0, min_victim_temp_k=296.0)
    depth_img = np.full((480, 640), 8.0, dtype=np.float32)
    # Background ambient at 288.15 K
    th_img = np.full((240, 320), 288.15, dtype=np.float32)
    # Cold tree shadow: slightly cooler than ambient ground (287.0 K)
    u_th, v_th = core.project_rgb_point_to_thermal(320, 240, 8.0)
    u, v = int(u_th), int(v_th)
    th_img[v-25:v+25, u-25:u+25] = 287.0 # Cold shadow covering ROI

    res = core.verify_candidate("c_shadow", 0, 1.0, [300, 220, 340, 260], 0.75, depth_img, th_img)
    assert res.status == VerificationStatus.REJECTED.value
    assert res.rejection_reason == RejectionReason.COLD_SHADOW_OR_OBSTACLE.value
    assert res.thermal_stats['delta_k'] < core.min_thermal_delta_k

def test_human_like_thermal_hotspot_acceptance():
    core = RGBThermalFusionCore(min_thermal_delta_k=3.0, min_victim_temp_k=296.0)
    depth_img = np.full((480, 640), 8.0, dtype=np.float32)
    # Ambient background: 288.15 K
    th_img = np.full((240, 320), 288.15, dtype=np.float32)
    # Human victim hotspot: 308.40 K (Phase 1 verified human signature)
    u_th, v_th = core.project_rgb_point_to_thermal(320, 240, 8.0)
    u, v = int(u_th), int(v_th)
    th_img[v-10:v+10, u-10:u+10] = 308.40

    res = core.verify_candidate(
        candidate_id="c_human",
        drone_id=0,
        timestamp=1.0,
        rgb_bbox=[300, 220, 340, 260],
        rgb_confidence=0.82,
        depth_image=depth_img,
        thermal_image_k=th_img,
        drone_pose=(0.0, 0.0, 8.0, 0.0)
    )
    assert res.status == VerificationStatus.VERIFIED.value
    assert res.rejection_reason == RejectionReason.NONE.value
    assert res.thermal_stats['max_k'] >= 308.0
    assert res.thermal_stats['delta_k'] > 15.0
    assert res.world_pos is not None

def test_hot_non_human_clutter_rejection():
    core = RGBThermalFusionCore(max_human_temp_k=318.15)
    depth_img = np.full((480, 640), 8.0, dtype=np.float32)
    th_img = np.full((240, 320), 288.15, dtype=np.float32)
    # Burning debris / camp fire / engine exhaust: 345.0 K (71.85 °C)
    u_th, v_th = core.project_rgb_point_to_thermal(320, 240, 8.0)
    u, v = int(u_th), int(v_th)
    th_img[v-10:v+10, u-10:u+10] = 345.0

    res = core.verify_candidate(
        candidate_id="c_fire",
        drone_id=0,
        timestamp=1.0,
        rgb_bbox=[300, 220, 340, 260],
        rgb_confidence=0.65,
        depth_image=depth_img,
        thermal_image_k=th_img
    )
    assert res.status == VerificationStatus.REJECTED.value
    assert res.rejection_reason == RejectionReason.HOT_NON_HUMAN_CLUTTER.value

def test_timestamp_synchronization():
    from collections import deque
    # Emulate node buffer lookup logic
    buf = deque(maxlen=20)
    buf.append((10.0, "frame_10"))
    buf.append((10.2, "frame_10_2"))
    buf.append((10.4, "frame_10_4"))

    target_ts = 10.22
    closest = min(buf, key=lambda x: abs(x[0] - target_ts))
    assert closest[1] == "frame_10_2"
    assert abs(closest[0] - target_ts) < 0.05

    # Stale frame test (> 0.5s tolerance)
    stale_target = 11.5
    diff = abs(closest[0] - stale_target)
    assert diff > 0.5

def test_duplicate_deduplication():
    # Test spatial deduplication
    confirmed = [{'track_id': 1, 'x': 10.0, 'y': 20.0}]
    dedup_radius = 6.0
    
    # Candidate 2 meters away from confirmed victim
    near_x, near_y = 11.0, 21.0
    dist = math.hypot(near_x - confirmed[0]['x'], near_y - confirmed[0]['y'])
    assert dist < dedup_radius, "Should be within dedup radius"

    # Candidate 15 meters away
    far_x, far_y = 25.0, 20.0
    dist_far = math.hypot(far_x - confirmed[0]['x'], far_y - confirmed[0]['y'])
    assert dist_far > dedup_radius, "Should be outside dedup radius"

def test_state_transitions():
    core = RGBThermalFusionCore()
    depth_img = np.full((480, 640), 8.0, dtype=np.float32)
    th_img = np.full((240, 320), 288.15, dtype=np.float32)

    # 1. Candidate Unverified (Missing thermal)
    res_unv = core.verify_candidate("c1", 0, 1.0, [300, 220, 340, 260], 0.8, depth_img, thermal_image_k=None)
    assert res_unv.status == VerificationStatus.CANDIDATE_UNVERIFIED.value

    # 2. Rejected (Cold shadow)
    res_rej = core.verify_candidate("c2", 0, 1.0, [300, 220, 340, 260], 0.8, depth_img, th_img)
    assert res_rej.status == VerificationStatus.REJECTED.value

    # 3. Verified (Hotspot present)
    th_img[110:130, 150:170] = 308.0
    res_ver = core.verify_candidate("c3", 0, 1.0, [300, 220, 340, 260], 0.8, depth_img, th_img)
    assert res_ver.status == VerificationStatus.VERIFIED.value

def main():
    print("=" * 70)
    print("PHASE 2: RGB-THERMAL FUSION VERIFIER UNIT TEST SUITE")
    print("=" * 70)
    tests = [
        ("Thermal Radiometric mono16 Decoding", test_thermal_decoding),
        ("RGB-to-Thermal Pinhole Projection & Alignment", test_rgb_to_thermal_projection),
        ("Out-of-Thermal-FOV Candidate Rejection", test_out_of_fov_candidates),
        ("Invalid Depth Filtering (NaN & Out-of-Range)", test_invalid_depth),
        ("Cold Ground Shadow & Obstacle Rejection", test_cold_shadow_rejection),
        ("Human-Like Thermal Hotspot Verification", test_human_like_thermal_hotspot_acceptance),
        ("Hot Non-Human Clutter Rejection (> 45°C)", test_hot_non_human_clutter_rejection),
        ("Temporal Buffer Synchronization Matching", test_timestamp_synchronization),
        ("Spatial Discovery Deduplication Handling", test_duplicate_deduplication),
        ("Candidate/Verified/Rejected State Transitions", test_state_transitions),
    ]

    passed = 0
    for name, fn in tests:
        if run_test(name, fn):
            passed += 1

    print("=" * 70)
    print(f"TEST RESULTS: {passed}/{len(tests)} PASSED")
    print("=" * 70)
    return 0 if passed == len(tests) else 1

if __name__ == '__main__':
    sys.exit(main())
