#!/usr/bin/env python3
"""
Rigorous Multi-Feature Thermal Verification & Classification Test Suite (Phase 2 Remediation - Stage 4)

Validates:
  1. Standing human victim posture verification
  2. Sitting human victim posture verification
  3. Lying / prone human victim posture verification
  4. Cold ambient ground rejection
  5. Cold shadow / tree obstacle rejection
  6. Sun-heated concrete slab rejection (uniform diffuse heating)
  7. Sun-heated rock rejection (isothermal non-human morphology)
  8. Warm vehicle panel rejection (geometric size / diffuse heat)
  9. Hot exhaust / campfire clutter rejection (> 45°C)
  10. Human-sized warm non-human object rejection (flat thermal profile)
  11. Invalid / corrupt thermal frame handling
  12. Saturated thermal sensor image rejection
  13. Empty ROI handling
  14. Partially occluded victim handling with explicit uncertainty
  15. Multiple nearby thermal regions segmentation
  16. Misaligned RGB-thermal projection rejection
  17. Feature extraction and verification processing latency benchmark
"""

import os
import sys
import math
import time
import numpy as np

# Ensure workspace packages can be imported standalone
ws_src = os.path.abspath(os.path.join(os.path.dirname(__file__), '../drone_ws/src/swarm_controller'))
if ws_src not in sys.path:
    sys.path.insert(0, ws_src)

from swarm_controller.rgb_thermal_fusion_verifier import (
    RGBThermalFusionCore,
    CameraIntrinsics,
    ThermalFeatureConfig,
    ThermalStats,
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
        import traceback
        print(f"[FAIL] {name}: Exception: {e}")
        traceback.print_exc()
        return False


def test_standing_victim():
    """
    Standing adult human: height ~1.75m, width ~0.55m at Z=8.0m.
    In RGB: w_px ≈ 19, h_px ≈ 59.
    Thermal signature: 308.4K torso/head core, 288.15K ambient background.
    """
    core = RGBThermalFusionCore()
    depth_img = np.full((480, 640), 8.0, dtype=np.float32)
    th_img = np.full((240, 320), 288.15, dtype=np.float32)

    # Place victim at center of image
    # RGB center (320, 240) -> thermal center (161, 120)
    u_th, v_th = core.project_rgb_point_to_thermal(320, 240, 8.0)
    u, v = int(round(u_th)), int(round(v_th))
    # Human hotspot in thermal (height 30px, width 10px)
    th_img[v-15:v+15, u-5:u+5] = 304.0  # Outer body/limbs
    th_img[v-8:v+8, u-3:u+3] = 308.4    # Warm thoracic core

    res = core.verify_candidate(
        candidate_id="standing_human",
        drone_id=0,
        timestamp=1.0,
        rgb_bbox=[310, 210, 330, 270],  # ~20x60 px
        rgb_confidence=0.88,
        depth_image=depth_img,
        thermal_image_k=th_img,
        drone_pose=(0.0, 0.0, 8.0, 0.0)
    )

    assert res.status in [VerificationStatus.THERMAL_SUPPORTED.value, "VERIFIED"], f"Expected THERMAL_SUPPORTED, got {res.status}"
    assert res.rejection_reason == RejectionReason.NONE.value
    assert res.thermal_stats['max_k'] >= 308.0
    assert res.thermal_stats['delta_k'] > 15.0
    assert res.thermal_stats['core_periphery_delta_k'] > 1.0
    assert res.verification_score >= 0.70
    assert res.uncertainty_score < 0.40


def test_sitting_victim():
    """
    Sitting human: height ~1.1m, width ~0.7m at Z=7.0m.
    Thermal signature: 308.0K core, 288.15K ambient.
    """
    core = RGBThermalFusionCore()
    depth_img = np.full((480, 640), 7.0, dtype=np.float32)
    th_img = np.full((240, 320), 288.15, dtype=np.float32)

    u_th, v_th = core.project_rgb_point_to_thermal(320, 240, 7.0)
    u, v = int(round(u_th)), int(round(v_th))
    th_img[v-12:v+12, u-8:u+8] = 303.5
    th_img[v-6:v+6, u-4:u+4] = 308.0

    res = core.verify_candidate(
        candidate_id="sitting_human",
        drone_id=1,
        timestamp=1.0,
        rgb_bbox=[306, 219, 334, 261],  # 28x42 px
        rgb_confidence=0.84,
        depth_image=depth_img,
        thermal_image_k=th_img,
        drone_pose=(0.0, 0.0, 7.0, 0.0)
    )

    assert res.status in [VerificationStatus.THERMAL_SUPPORTED.value, "VERIFIED"]
    assert res.thermal_stats['core_periphery_delta_k'] > 1.0
    assert res.verification_score >= 0.70


def test_lying_victim():
    """
    Lying/prone human: height ~0.55m, width ~1.8m at Z=8.0m.
    Thermal signature: 308.1K, 288.15K ambient.
    """
    core = RGBThermalFusionCore()
    depth_img = np.full((480, 640), 8.0, dtype=np.float32)
    th_img = np.full((240, 320), 288.15, dtype=np.float32)

    u_th, v_th = core.project_rgb_point_to_thermal(320, 240, 8.0)
    u, v = int(round(u_th)), int(round(v_th))
    th_img[v-6:v+6, u-16:u+16] = 303.0
    th_img[v-3:v+3, u-7:u+7] = 308.1

    res = core.verify_candidate(
        candidate_id="lying_human",
        drone_id=2,
        timestamp=1.0,
        rgb_bbox=[290, 230, 350, 250],  # 60x20 px
        rgb_confidence=0.82,
        depth_image=depth_img,
        thermal_image_k=th_img,
        drone_pose=(0.0, 0.0, 8.0, 0.0)
    )

    assert res.status in [VerificationStatus.THERMAL_SUPPORTED.value, "VERIFIED"]
    assert res.thermal_stats['aspect_ratio'] > 2.0
    assert res.verification_score >= 0.70


def test_cold_ground():
    """Uniform ambient ground (288.15K) with no thermal anomaly."""
    core = RGBThermalFusionCore()
    depth_img = np.full((480, 640), 8.0, dtype=np.float32)
    th_img = np.full((240, 320), 288.15, dtype=np.float32)

    res = core.verify_candidate(
        candidate_id="cold_ground",
        drone_id=0,
        timestamp=1.0,
        rgb_bbox=[310, 210, 330, 270],
        rgb_confidence=0.75,
        depth_image=depth_img,
        thermal_image_k=th_img
    )

    assert res.status == VerificationStatus.REJECTED.value
    assert res.rejection_reason == RejectionReason.COLD_SHADOW_OR_OBSTACLE.value


def test_cold_shadow():
    """Tree shadow cooler than ground (286.0K vs 288.15K)."""
    core = RGBThermalFusionCore()
    depth_img = np.full((480, 640), 8.0, dtype=np.float32)
    th_img = np.full((240, 320), 288.15, dtype=np.float32)
    th_img[100:150, 140:180] = 286.0

    res = core.verify_candidate(
        candidate_id="cold_shadow",
        drone_id=0,
        timestamp=1.0,
        rgb_bbox=[300, 220, 340, 260],
        rgb_confidence=0.70,
        depth_image=depth_img,
        thermal_image_k=th_img
    )

    assert res.status == VerificationStatus.REJECTED.value
    assert res.rejection_reason == RejectionReason.COLD_SHADOW_OR_OBSTACLE.value


def test_sun_heated_concrete():
    """
    Sun-heated concrete slab: T = 304.5K (inside 296K-318K range).
    However, the entire rectangular patch is uniformly hot with zero core-periphery delta
    and flat spatial gradient. Must be rejected as UNIFORM_DIFFUSE_HEAT.
    """
    core = RGBThermalFusionCore()
    depth_img = np.full((480, 640), 8.0, dtype=np.float32)
    th_img = np.full((240, 320), 288.15, dtype=np.float32)

    u_th, v_th = core.project_rgb_point_to_thermal(320, 240, 8.0)
    u, v = int(round(u_th)), int(round(v_th))
    # Entire concrete slab extends across candidate region
    th_img[v-40:v+40, u-20:u+20] = 304.5

    res = core.verify_candidate(
        candidate_id="heated_concrete",
        drone_id=0,
        timestamp=1.0,
        rgb_bbox=[310, 210, 330, 270],
        rgb_confidence=0.75,
        depth_image=depth_img,
        thermal_image_k=th_img
    )

    assert res.status == VerificationStatus.REJECTED.value
    assert res.rejection_reason == RejectionReason.UNIFORM_DIFFUSE_HEAT.value, f"Expected UNIFORM_DIFFUSE_HEAT, got {res.rejection_reason}"


def test_sun_heated_rock():
    """
    Sun-heated rock: T = 301.5K.
    Isothermal surface without biological core-to-periphery concentration (core_delta <= 0).
    YOLO confidence is marginal (0.48). Must be rejected as NON_HUMAN_THERMAL_MORPHOLOGY.
    """
    core = RGBThermalFusionCore()
    depth_img = np.full((480, 640), 8.0, dtype=np.float32)
    th_img = np.full((240, 320), 288.15, dtype=np.float32)

    u_th, v_th = core.project_rgb_point_to_thermal(320, 240, 8.0)
    u, v = int(round(u_th)), int(round(v_th))
    # Rock filling the candidate region; core is slightly cooler than sun-exposed edges
    th_img[v-25:v+25, u-12:u+12] = 301.5
    th_img[v-10:v+10, u-5:u+5] = 300.8

    res = core.verify_candidate(
        candidate_id="heated_rock",
        drone_id=0,
        timestamp=1.0,
        rgb_bbox=[310, 220, 330, 260],
        rgb_confidence=0.48,
        depth_image=depth_img,
        thermal_image_k=th_img
    )

    assert res.status == VerificationStatus.REJECTED.value
    assert res.rejection_reason in [
        RejectionReason.NON_HUMAN_THERMAL_MORPHOLOGY.value,
        RejectionReason.LOW_MULTISPECTRAL_SCORE.value
    ]


def test_warm_vehicle_panel():
    """
    Warm vehicle engine hood: T = 312.0K.
    Dimensions 3.0m x 1.8m (exceeds max human physical size 2.4m).
    Must be rejected as GEOMETRIC_IMPLAUSIBLE.
    """
    core = RGBThermalFusionCore()
    depth_img = np.full((480, 640), 8.0, dtype=np.float32)
    th_img = np.full((240, 320), 288.15, dtype=np.float32)

    # 100 x 60 px in RGB at Z=8m -> 2.96m x 1.77m (max dim = 2.96m > 2.40m)
    res = core.verify_candidate(
        candidate_id="vehicle_panel",
        drone_id=0,
        timestamp=1.0,
        rgb_bbox=[270, 210, 370, 270],  # 100px wide
        rgb_confidence=0.72,
        depth_image=depth_img,
        thermal_image_k=th_img
    )

    assert res.status == VerificationStatus.REJECTED.value
    assert res.rejection_reason == RejectionReason.GEOMETRIC_IMPLAUSIBLE.value


def test_hot_exhaust_or_fire():
    """Burning debris / engine exhaust: T = 345.0K (> 45°C)."""
    core = RGBThermalFusionCore()
    depth_img = np.full((480, 640), 8.0, dtype=np.float32)
    th_img = np.full((240, 320), 288.15, dtype=np.float32)
    th_img[110:130, 150:170] = 345.0

    res = core.verify_candidate(
        candidate_id="fire_exhaust",
        drone_id=0,
        timestamp=1.0,
        rgb_bbox=[310, 220, 330, 260],
        rgb_confidence=0.70,
        depth_image=depth_img,
        thermal_image_k=th_img
    )

    assert res.status == VerificationStatus.REJECTED.value
    assert res.rejection_reason == RejectionReason.HOT_NON_HUMAN_CLUTTER.value


def test_human_sized_warm_non_human_object():
    """
    A non-human object with human dimensions (1.6m x 0.6m) and temperature in physiological
    range (302.5K), but completely isothermal (zero core concentration and low RGB confidence).
    Must NOT be verified as a human.
    """
    core = RGBThermalFusionCore()
    depth_img = np.full((480, 640), 8.0, dtype=np.float32)
    th_img = np.full((240, 320), 288.15, dtype=np.float32)

    u_th, v_th = core.project_rgb_point_to_thermal(320, 240, 8.0)
    u, v = int(round(u_th)), int(round(v_th))
    # Uniform warm object filling the candidate bbox
    th_box = th_img.copy()
    th_box[v-28:v+28, u-11:u+11] = 302.5

    res = core.verify_candidate(
        candidate_id="warm_cardboard_box",
        drone_id=0,
        timestamp=1.0,
        rgb_bbox=[310, 215, 330, 265],  # Human-sized 20x50 px
        rgb_confidence=0.45,             # Marginal false positive RGB detection
        depth_image=depth_img,
        thermal_image_k=th_box
    )

    assert res.status == VerificationStatus.REJECTED.value
    assert res.rejection_reason in [
        RejectionReason.UNIFORM_DIFFUSE_HEAT.value,
        RejectionReason.NON_HUMAN_THERMAL_MORPHOLOGY.value,
        RejectionReason.LOW_MULTISPECTRAL_SCORE.value
    ]


def test_invalid_thermal_frame():
    """Corrupted / NaN thermal frame handling."""
    core = RGBThermalFusionCore()
    depth_img = np.full((480, 640), 8.0, dtype=np.float32)
    th_nan = np.full((240, 320), np.nan, dtype=np.float32)

    res = core.verify_candidate(
        candidate_id="corrupt_frame",
        drone_id=0,
        timestamp=1.0,
        rgb_bbox=[310, 220, 330, 260],
        rgb_confidence=0.85,
        depth_image=depth_img,
        thermal_image_k=th_nan
    )

    assert res.status == VerificationStatus.REJECTED.value


def test_saturated_thermal_image():
    """Thermal image pegged at saturation limit (323.15K) with zero variance."""
    core = RGBThermalFusionCore()
    depth_img = np.full((480, 640), 8.0, dtype=np.float32)
    th_sat = np.full((240, 320), 323.15, dtype=np.float32)

    res = core.verify_candidate(
        candidate_id="saturated_sensor",
        drone_id=0,
        timestamp=1.0,
        rgb_bbox=[310, 220, 330, 260],
        rgb_confidence=0.80,
        depth_image=depth_img,
        thermal_image_k=th_sat
    )

    assert res.status == VerificationStatus.REJECTED.value
    assert res.rejection_reason in [
        RejectionReason.SATURATED_OR_CORRUPT_FRAME.value,
        RejectionReason.HOT_NON_HUMAN_CLUTTER.value
    ]


def test_empty_roi():
    """Empty or inverted bounding box."""
    core = RGBThermalFusionCore()
    th_img = np.full((240, 320), 288.15, dtype=np.float32)
    stats = core.analyze_thermal_roi(th_img, [150, 100, 150, 100])  # 0x0
    assert stats is None


def test_partially_occluded_victim():
    """Partially occluded victim: hotspot visible in partial ROI with flagged uncertainty."""
    core = RGBThermalFusionCore()
    depth_img = np.full((480, 640), 8.0, dtype=np.float32)
    th_img = np.full((240, 320), 288.15, dtype=np.float32)

    u_th, v_th = core.project_rgb_point_to_thermal(320, 240, 8.0)
    u, v = int(round(u_th)), int(round(v_th))
    # Hotspot partially visible (e.g., upper body visible, lower occluded)
    th_img[v-12:v, u-4:u+4] = 308.0

    res = core.verify_candidate(
        candidate_id="occluded_victim",
        drone_id=0,
        timestamp=1.0,
        rgb_bbox=[310, 210, 330, 270],
        rgb_confidence=0.75,
        depth_image=depth_img,
        thermal_image_k=th_img,
        drone_pose=(0.0, 0.0, 8.0, 0.0)
    )

    # Handled safely: either verified with uncertainty or rejected gracefully
    assert res.status in [VerificationStatus.THERMAL_SUPPORTED.value, VerificationStatus.REJECTED.value]
    if res.status == VerificationStatus.THERMAL_SUPPORTED.value:
        assert res.uncertainty_score > 0.0, "Should have non-zero uncertainty for partial occlusion"


def test_multiple_nearby_thermal_regions():
    """Candidate next to a secondary warm rock; connected components correctly isolates candidate."""
    core = RGBThermalFusionCore()
    depth_img = np.full((480, 640), 8.0, dtype=np.float32)
    th_img = np.full((240, 320), 288.15, dtype=np.float32)

    u_th, v_th = core.project_rgb_point_to_thermal(320, 240, 8.0)
    u, v = int(round(u_th)), int(round(v_th))

    # Human victim at candidate center
    th_img[v-15:v+15, u-5:u+5] = 304.0
    th_img[v-8:v+8, u-3:u+3] = 308.4
    # Nearby secondary hotspot (e.g. heated rock 30 pixels away)
    th_img[v-10:v+10, u+25:u+35] = 301.0

    res = core.verify_candidate(
        candidate_id="cluttered_scene",
        drone_id=0,
        timestamp=1.0,
        rgb_bbox=[310, 210, 330, 270],
        rgb_confidence=0.85,
        depth_image=depth_img,
        thermal_image_k=th_img,
        drone_pose=(0.0, 0.0, 8.0, 0.0)
    )

    assert res.status in [VerificationStatus.THERMAL_SUPPORTED.value, "VERIFIED"]
    assert res.thermal_stats['primary_component_fraction'] > 0.10


def test_misaligned_rgb_thermal_projection():
    """Misaligned RGB detection falls on cold ground next to the victim."""
    core = RGBThermalFusionCore()
    depth_img = np.full((480, 640), 8.0, dtype=np.float32)
    th_img = np.full((240, 320), 288.15, dtype=np.float32)

    # Victim is at (160, 120), but RGB bbox detects at (200, 240), projecting to (100, 120)
    th_img[110:130, 155:165] = 308.4  # Victim hotspot at 160

    res = core.verify_candidate(
        candidate_id="misaligned_det",
        drone_id=0,
        timestamp=1.0,
        rgb_bbox=[200, 220, 240, 260],   # Off-target bbox
        rgb_confidence=0.70,
        depth_image=depth_img,
        thermal_image_k=th_img
    )

    assert res.status == VerificationStatus.REJECTED.value
    assert res.rejection_reason == RejectionReason.COLD_SHADOW_OR_OBSTACLE.value


def test_latency_benchmark():
    """Benchmarks feature extraction latency over 100 iterations. Must be < 1.0ms per ROI."""
    core = RGBThermalFusionCore()
    th_img = np.full((240, 320), 288.15, dtype=np.float32)
    th_img[110:130, 155:165] = 308.4

    t0 = time.perf_counter()
    N = 100
    for _ in range(N):
        core.analyze_thermal_roi(th_img, [150, 105, 170, 135], phys_dims_m=(0.6, 1.7), rgb_confidence=0.85)
    t1 = time.perf_counter()

    mean_ms = ((t1 - t0) / N) * 1000.0
    print(f"      [BENCHMARK] Multi-Feature Thermal Analysis: {mean_ms:.3f} ms / ROI ({1000.0/mean_ms:.0f} ROIs/sec)")
    assert mean_ms < 1.0, f"Mean latency {mean_ms:.3f}ms exceeds 1.0ms limit!"


def main():
    print("=" * 75)
    print("PHASE 2 REMEDIATION: STAGE 4 MULTI-FEATURE THERMAL TEST SUITE")
    print("=" * 75)

    tests = [
        ("Standing Human Victim Posture Verification", test_standing_victim),
        ("Sitting Human Victim Posture Verification", test_sitting_victim),
        ("Lying / Prone Human Victim Posture Verification", test_lying_victim),
        ("Cold Ground Rejection", test_cold_ground),
        ("Cold Shadow & Obstacle Rejection", test_cold_shadow),
        ("Sun-Heated Concrete Slab Rejection (Uniform Diffuse Heat)", test_sun_heated_concrete),
        ("Sun-Heated Rock Rejection (Isothermal Morphology)", test_sun_heated_rock),
        ("Warm Vehicle Panel Rejection (Geometric Size / Diffuse)", test_warm_vehicle_panel),
        ("Hot Clutter Rejection (> 45°C Fire / Exhaust)", test_hot_exhaust_or_fire),
        ("Human-Sized Warm Non-Human Object Rejection", test_human_sized_warm_non_human_object),
        ("Corrupt / NaN Thermal Frame Handling", test_invalid_thermal_frame),
        ("Saturated Thermal Sensor Image Rejection", test_saturated_thermal_image),
        ("Empty ROI Handling", test_empty_roi),
        ("Partially Occluded Victim Handling", test_partially_occluded_victim),
        ("Multiple Nearby Thermal Regions Segmentation", test_multiple_nearby_thermal_regions),
        ("Misaligned RGB-Thermal Projection Rejection", test_misaligned_rgb_thermal_projection),
        ("Multi-Feature Extraction Latency Benchmark", test_latency_benchmark),
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
