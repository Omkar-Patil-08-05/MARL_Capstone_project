"""
Unit and Regression Test Suite for Research-Grade Evaluation Module
Verifies:
1. Canonical checkpoint verification (cryptographic audit)
2. Evaluation seeds specification and integrity
3. Sortie execution, observation alignment, and output schema
4. Safety envelope bypass in ablation mode
5. Silent mode transport isolation
"""

import os
import sys
import pytest
import rclpy

_repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)

_drone_ws_src = os.path.join(_repo_root, "drone_ws", "src", "swarm_controller")
if _drone_ws_src not in sys.path:
    sys.path.insert(0, _drone_ws_src)

from scripts.evaluation.run_qmix_research_evaluation import (
    verify_checkpoints,
    run_single_sortie,
    EVALUATION_SEEDS,
    CANONICAL_QMIX_PATH,
    CANONICAL_QMIX_SHA256,
    CANONICAL_YOLO_PATH,
    CANONICAL_YOLO_SHA256
)


@pytest.fixture(scope="module", autouse=True)
def init_ros():
    if not rclpy.ok():
        rclpy.init()
    yield
    # Keep context intact for subsequent tests in session


def test_verify_canonical_checkpoints():
    """Ensure both QMIX and YOLO models exist and match invariant SHA-256 digests."""
    is_valid, manifest = verify_checkpoints()
    assert is_valid is True, f"Checkpoint verification failed: {manifest}"
    assert manifest["qmix"]["sha256"] == CANONICAL_QMIX_SHA256
    assert manifest["yolo"]["sha256"] == CANONICAL_YOLO_SHA256
    assert manifest["qmix"]["status"] == "VALID"
    assert manifest["yolo"]["status"] == "VALID"


def test_evaluation_seeds_specification():
    """Verify exact 10 evaluation seeds specified for Phase 8 research validation."""
    expected_seeds = [11, 23, 37, 41, 53, 67, 79, 83, 97, 101]
    assert len(EVALUATION_SEEDS) == 10
    assert EVALUATION_SEEDS == expected_seeds


def test_single_sortie_execution_and_schema():
    """Verify execution of a single sortie and completeness of output metrics."""
    result = run_single_sortie(
        condition_name="test_qmix_p2p",
        seed=11,
        num_drones=2,
        policy_type="qmix",
        enable_safety_envelope=True,
        enable_task_claims=True,
        silent_mode=False,
        max_decisions=3
    )

    required_keys = [
        "condition", "seed", "num_drones", "policy_type", "enable_safety_envelope",
        "enable_task_claims", "silent_mode", "packet_loss_rate", "latency_ms",
        "success", "detection_rate", "victims_found", "total_victims",
        "localization_mae_m", "localization_rmse_m", "time_to_first_det_s",
        "time_to_full_det_s", "area_coverage_pct", "search_throughput_m2_s",
        "total_distance_m", "energy_consumption_wh", "min_separation_m",
        "collision_violations", "safety_envelope_overrides", "task_claims_count",
        "redundant_inspections", "messages_sent_total", "messages_recv_total",
        "bytes_sent_total", "packet_delivery_ratio", "mean_decision_latency_ms",
        "max_decision_latency_ms"
    ]
    for key in required_keys:
        assert key in result, f"Missing key '{key}' in sortie result schema"

    assert result["num_drones"] == 2
    assert result["seed"] == 11
    assert result["min_separation_m"] >= 1.40


def test_ablation_no_safety_envelope_execution():
    """Verify that disabling the safety envelope executes without unhandled local exceptions."""
    result = run_single_sortie(
        condition_name="test_ablation_no_safety",
        seed=23,
        num_drones=2,
        policy_type="qmix",
        enable_safety_envelope=False,
        enable_task_claims=True,
        silent_mode=False,
        max_decisions=3
    )
    assert result["condition"] == "test_ablation_no_safety"
    assert result["enable_safety_envelope"] is False
    assert "collision_violations" in result


def test_silent_mode_isolation():
    """Verify that silent mode does not deliver external peer messages."""
    result = run_single_sortie(
        condition_name="test_silent_mode",
        seed=37,
        num_drones=2,
        policy_type="qmix",
        enable_safety_envelope=True,
        enable_task_claims=True,
        silent_mode=True,
        max_decisions=3
    )
    assert result["silent_mode"] is True
    # With 100% loss/silent mode, received peer messages should be 0
    assert result["messages_recv_total"] == 0
