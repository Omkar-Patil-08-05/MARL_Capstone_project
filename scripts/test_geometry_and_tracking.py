#!/usr/bin/env python3
"""
Rigorous RGB-to-Thermal Geometric Transformation & Tracking Verification (Phase 2 Remediation - Stage 3)

Validates:
  1. Zero-baseline projection (ideal coaxial sensor alignment)
  2. Positive horizontal baseline disparity
  3. Negative horizontal baseline disparity (actual 30mm physical configuration)
  4. Vertical baseline displacement disparity
  5. Non-zero relative rotation (rigid-body SE(3) rotation matrix)
  6. Near-range projection disparity (Z = 2.0m)
  7. Far-range projection disparity (Z = 35.0m)
  8. Thermal image boundary clamping and coordinate integrity
  9. Invalid depth handling (NaN, negative, zero, infinity)
  10. Out-of-thermal-FOV candidate rejection
  11. Principal point alignment and analytical disparity vs depth verification
"""

import os
import sys
import math
import numpy as np

# Ensure workspace packages can be imported
ws_src = os.path.abspath(os.path.join(os.path.dirname(__file__), '../drone_ws/src/swarm_controller'))
if ws_src not in sys.path:
    sys.path.insert(0, ws_src)

from swarm_controller.rgb_thermal_fusion_verifier import (
    RGBThermalFusionCore,
    CameraIntrinsics,
    VerificationStatus,
    RejectionReason
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


def test_zero_baseline_projection():
    """
    With zero baseline t = (0, 0, 0) and R = I, optical center of RGB (320, 240)
    must project identically to the optical center of Thermal (160, 120).
    """
    core = RGBThermalFusionCore(translation_rgb_to_th=(0.0, 0.0, 0.0))
    u_th, v_th = core.project_rgb_point_to_thermal(320.0, 240.0, depth_m=10.0)
    assert np.isclose(u_th, 160.0, atol=1e-3), f"Expected 160.0, got {u_th}"
    assert np.isclose(v_th, 120.0, atol=1e-3), f"Expected 120.0, got {v_th}"


def test_positive_horizontal_baseline():
    """
    With translation dx = +0.030m along optical X, target should shift to left in thermal:
    u_th = 160.0 - (fx_th * 0.030 / Z)
    At Z = 10m: u_th = 160.0 - 0.8315 = 159.1685 px
    """
    core = RGBThermalFusionCore(translation_rgb_to_th=(0.030, 0.0, 0.0))
    u_th, v_th = core.project_rgb_point_to_thermal(320.0, 240.0, depth_m=10.0)
    expected_u = 160.0 - (277.17 * 0.030 / 10.0)
    assert np.isclose(u_th, expected_u, atol=0.05), f"Expected {expected_u:.3f}, got {u_th:.3f}"
    assert np.isclose(v_th, 120.0, atol=1e-3)


def test_negative_horizontal_baseline_physical():
    """
    Physical configuration: Thermal is at Y_link = +0.015m (Left of drone).
    In downward optical frame, X_optical = -Y_link -> dx = -0.030m.
    At Z = 10m, center pixel must shift RIGHT:
    u_th = 160.0 + (277.17 * 0.030 / 10.0) = 160.8315 px
    v_th = 120.0 px (identically zero vertical disparity)
    """
    core = RGBThermalFusionCore(translation_rgb_to_th=(-0.030, 0.0, 0.0))
    u_th, v_th = core.project_rgb_point_to_thermal(320.0, 240.0, depth_m=10.0)
    expected_u = 160.0 + (277.17 * 0.030 / 10.0)
    assert np.isclose(u_th, expected_u, atol=0.05), f"Expected {expected_u:.3f}, got {u_th:.3f}"
    assert np.isclose(v_th, 120.0, atol=1e-3), f"Expected 120.0, got {v_th:.3f}"


def test_vertical_baseline_displacement():
    """
    Test vertical translation dy = +0.020m along optical Y:
    v_th = 120.0 - (fy_th * 0.020 / Z)
    At Z = 10m: v_th = 120.0 - (277.17 * 0.020 / 10.0) = 119.446 px
    """
    core = RGBThermalFusionCore(translation_rgb_to_th=(0.0, 0.020, 0.0))
    u_th, v_th = core.project_rgb_point_to_thermal(320.0, 240.0, depth_m=10.0)
    expected_v = 120.0 - (277.17 * 0.020 / 10.0)
    assert np.isclose(u_th, 160.0, atol=1e-3)
    assert np.isclose(v_th, expected_v, atol=0.05), f"Expected {expected_v:.3f}, got {v_th:.3f}"


def test_non_zero_relative_rotation():
    """
    Test SE(3) rigid-body transformation with non-zero relative rotation.
    Rotate by +2 degrees (0.0349 rad) yaw around optical axis Z:
    R = [[cos, -sin, 0], [sin, cos, 0], [0, 0, 1]]
    """
    theta = math.radians(2.0)
    R = np.array([
        [math.cos(theta), -math.sin(theta), 0.0],
        [math.sin(theta),  math.cos(theta), 0.0],
        [0.0,              0.0,             1.0]
    ])
    core = RGBThermalFusionCore(
        translation_rgb_to_th=(0.0, 0.0, 0.0),
        rotation_rgb_to_th=R
    )
    # Target at RGB (400, 240) -> X_c = (400-320)*10 / 270 = 2.963m, Y_c = 0
    u_th, v_th = core.project_rgb_point_to_thermal(400.0, 240.0, depth_m=10.0)
    # Rotated point: X_th = X_c * cos(2°), Y_th = X_c * sin(2°)
    # u_th = fx * X_th / 10 + 160, v_th = fy * Y_th / 10 + 120
    x_c = (400.0 - 320.0) * 10.0 / core.rgb_cam.fx
    x_th = x_c * math.cos(theta)
    y_th = x_c * math.sin(theta)
    exp_u = core.th_cam.fx * x_th / 10.0 + 160.0
    exp_v = core.th_cam.fy * y_th / 10.0 + 120.0

    assert np.isclose(u_th, exp_u, atol=0.05), f"Expected {exp_u:.3f}, got {u_th:.3f}"
    assert np.isclose(v_th, exp_v, atol=0.05), f"Expected {exp_v:.3f}, got {v_th:.3f}"


def test_near_and_far_range_projections():
    """
    Tests analytical disparity scaling inversely with depth Z:
    Delta_u(Z) = fx_th * baseline / Z
    At Z = 2.0m:  Delta_u = 277.17 * 0.030 / 2.0  = +4.157 px
    At Z = 35.0m: Delta_u = 277.17 * 0.030 / 35.0 = +0.238 px
    """
    core = RGBThermalFusionCore() # Uses default (-0.030, 0, 0)
    # Near range Z = 2.0m
    u_near, v_near = core.project_rgb_point_to_thermal(320.0, 240.0, depth_m=2.0)
    exp_near_u = 160.0 + (277.17 * 0.030 / 2.0)
    assert np.isclose(u_near, exp_near_u, atol=0.05), f"Near Z=2m: expected {exp_near_u:.3f}, got {u_near:.3f}"
    assert np.isclose(v_near, 120.0, atol=1e-3)

    # Far range Z = 35.0m
    u_far, v_far = core.project_rgb_point_to_thermal(320.0, 240.0, depth_m=35.0)
    exp_far_u = 160.0 + (277.17 * 0.030 / 35.0)
    assert np.isclose(u_far, exp_far_u, atol=0.05), f"Far Z=35m: expected {exp_far_u:.3f}, got {u_far:.3f}"
    assert np.isclose(v_far, 120.0, atol=1e-3)


def test_image_boundary_handling():
    """
    Tests bounding-box clamping at the exact boundaries of the 320x240 thermal sensor.
    """
    core = RGBThermalFusionCore()
    # A candidate spanning the border
    bbox_rgb = [200, 150, 440, 330]
    bbox_th, fully_inside, partially_inside = core.project_rgb_bbox_to_thermal(bbox_rgb, depth_m=10.0)

    assert partially_inside is True
    assert bbox_th is not None
    assert 0 <= bbox_th[0] < bbox_th[2] <= 320
    assert 0 <= bbox_th[1] < bbox_th[3] <= 240


def test_invalid_depth_rejections():
    """
    Tests that invalid depth inputs (<=0, NaN, Inf) are safely caught and rejected.
    """
    core = RGBThermalFusionCore()
    invalid_depths = [-5.0, 0.0, float('nan'), float('inf')]

    for d in invalid_depths:
        try:
            core.project_rgb_point_to_thermal(320.0, 240.0, depth_m=d)
            assert False, f"Expected ValueError for depth={d}"
        except ValueError:
            pass # Correctly caught

    # Bbox projection returns None for invalid depth
    bbox_th, fully, part = core.project_rgb_bbox_to_thermal([300, 220, 340, 260], depth_m=-2.0)
    assert bbox_th is None
    assert fully is False and part is False


def test_out_of_thermal_fov_rejection():
    """
    Thermal FOV is 60°, RGB FOV is 99.7°.
    An object at RGB pixel (20, 20) with Z=10m must project outside thermal 320x240 frame.
    """
    core = RGBThermalFusionCore()
    bbox_th, fully_inside, partially_inside = core.project_rgb_bbox_to_thermal([10, 10, 50, 50], depth_m=10.0)
    assert fully_inside is False


def test_camera_intrinsics_consistency():
    """
    Verifies that CameraIntrinsics.from_fov matches the SDF camera specifications.
    RGB: 640x480, HFOV=1.74 rad -> fx=fy=269.97 px, cx=320, cy=240
    Thermal: 320x240, HFOV=1.047 rad -> fx=fy=277.17 px, cx=160, cy=120
    """
    rgb_cam = CameraIntrinsics.from_fov(640, 480, 1.74)
    assert np.isclose(rgb_cam.fx, 269.97, atol=0.1)
    assert np.isclose(rgb_cam.fy, 269.97, atol=0.1)
    assert rgb_cam.cx == 320.0
    assert rgb_cam.cy == 240.0

    th_cam = CameraIntrinsics.from_fov(320, 240, 1.047)
    assert np.isclose(th_cam.fx, 277.17, atol=0.1)
    assert np.isclose(th_cam.fy, 277.17, atol=0.1)
    assert th_cam.cx == 160.0
    assert th_cam.cy == 120.0


def main():
    print("=" * 75)
    print("PHASE 2 REMEDIATION: STAGE 3 GEOMETRY & TRACKING TEST SUITE")
    print("=" * 75)

    tests = [
        ("Zero-Baseline Projection Alignment", test_zero_baseline_projection),
        ("Positive Horizontal Baseline Disparity", test_positive_horizontal_baseline),
        ("Physical Negative Horizontal Baseline Disparity (-30mm)", test_negative_horizontal_baseline_physical),
        ("Vertical Baseline Displacement Disparity", test_vertical_baseline_displacement),
        ("Non-Zero SE(3) Rigid-Body Rotation Transformation", test_non_zero_relative_rotation),
        ("Near-Range and Far-Range Analytical Disparity Scaling", test_near_and_far_range_projections),
        ("Image Boundary Clamping and FOV Integrity", test_image_boundary_handling),
        ("Invalid Depth Rejections (NaN, <=0, Inf)", test_invalid_depth_rejections),
        ("Out-of-Thermal-FOV Candidate Rejection", test_out_of_thermal_fov_rejection),
        ("Camera Intrinsics Derivation Consistency with SDF", test_camera_intrinsics_consistency),
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
