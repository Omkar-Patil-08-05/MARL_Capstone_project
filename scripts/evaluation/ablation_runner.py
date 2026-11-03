#!/usr/bin/env python3
"""
Ablation Study Runner (Phase 2 Remediation - Stage 5)

Evaluates 6 sequential perception modes on the exact same dataset of 150 scenarios:
  1. RGB_ONLY: Accepts candidates based purely on RGB detector confidence and depth.
  2. BASIC_TEMP: Adds scalar physiological temperature gating (296K <= T_max <= 318.15K).
  3. TEMP_AND_CONTRAST: Adds local background annular contrast (Delta_T >= 3.0K).
  4. TEMP_AND_MORPHOLOGY: Adds spatial gradient, core-periphery delta, and occupancy limits.
  5. FULL_STAGE4_MVS: Full Stage 4 pipeline (MVS score >= 0.45, all 12 rejection gates, uncertainty).
  6. FULL_PLUS_TEMPORAL: Full Stage 4 pipeline + multi-frame spatio-temporal consensus.
"""

from typing import List, Dict, Any
import numpy as np

from swarm_controller.rgb_thermal_fusion_verifier import (
    RGBThermalFusionCore,
    VerificationStatus,
    RejectionReason
)
from swarm_controller.temporal_event_confirmer import TemporalEventConfirmer
from .scenario_generator import EvaluationScenario
from .metrics_calculator import MetricsCalculator, MetricSummary


class AblationRunner:
    """Executes systematic ablation study across perception pipeline configurations."""

    def __init__(self):
        self.core = RGBThermalFusionCore()

    def run_all_ablations(self, scenarios: List[EvaluationScenario]) -> Dict[str, Dict[str, Any]]:
        """
        Runs all 6 ablation modes on the provided scenarios and returns summary metrics.
        """
        modes = [
            "1_RGB_ONLY",
            "2_BASIC_TEMP",
            "3_TEMP_AND_CONTRAST",
            "4_TEMP_AND_MORPHOLOGY",
            "5_FULL_STAGE4_MVS",
            "6_FULL_PLUS_TEMPORAL"
        ]

        results = {}
        for mode in modes:
            records = self.evaluate_mode(mode, scenarios)
            summary = MetricsCalculator.compute_summary(records)
            results[mode] = {
                "summary": summary.to_dict(),
                "records": records
            }

        return results

    def evaluate_mode(self, mode: str, scenarios: List[EvaluationScenario]) -> List[Dict[str, Any]]:
        records = []
        confirmer = TemporalEventConfirmer(window_size=6, consensus_needed=4, min_time_span=0.4)

        for sc in scenarios:
            pred_verified = False
            status = "REJECTED"
            reason = "UNKNOWN"
            mvs_score = 0.0

            # -------------------------------------------------------------
            # Mode 1: RGB Confidence Only
            # -------------------------------------------------------------
            if mode == "1_RGB_ONLY":
                if sc.rgb_confidence >= 0.35 and sc.depth_image is not None and not np.all(np.isnan(sc.depth_image)):
                    if 3.0 <= sc.depth_m <= 35.0:
                        pred_verified = True
                        status = "VERIFIED"
                        reason = "NONE"
                    else:
                        status = "REJECTED"
                        reason = "INVALID_DEPTH"
                else:
                    status = "REJECTED"
                    reason = "LOW_CONF_OR_NO_DEPTH"

            # -------------------------------------------------------------
            # Mode 2: Basic Temperature Window
            # -------------------------------------------------------------
            elif mode == "2_BASIC_TEMP":
                if sc.rgb_confidence >= 0.35 and sc.depth_image is not None and (3.0 <= sc.depth_m <= 35.0):
                    if sc.thermal_image_k is not None and not np.all(np.isnan(sc.thermal_image_k)):
                        t_max = float(np.nanmax(sc.thermal_image_k))
                        if 296.0 <= t_max <= 318.15:
                            pred_verified = True
                            status = "VERIFIED"
                            reason = "NONE"
                        elif t_max > 318.15:
                            reason = "HOT_NON_HUMAN_CLUTTER"
                        else:
                            reason = "COLD_SHADOW_OR_OBSTACLE"
                    else:
                        reason = "MISSING_SENSOR_DATA"
                else:
                    reason = "INVALID_DEPTH_OR_CONF"

            # -------------------------------------------------------------
            # Mode 3: Temperature + Local Contrast
            # -------------------------------------------------------------
            elif mode == "3_TEMP_AND_CONTRAST":
                if sc.rgb_confidence >= 0.35 and sc.depth_image is not None and (3.0 <= sc.depth_m <= 35.0):
                    if sc.thermal_image_k is not None and not np.all(np.isnan(sc.thermal_image_k)):
                        bbox_th, fully, part = self.core.project_rgb_bbox_to_thermal(sc.rgb_bbox, sc.depth_m)
                        if part and bbox_th is not None:
                            stats = self.core.analyze_thermal_roi(sc.thermal_image_k, bbox_th)
                            if stats is not None:
                                if stats.max_k > 318.15:
                                    reason = "HOT_NON_HUMAN_CLUTTER"
                                elif stats.max_k < 296.0 or stats.delta_k < 3.0:
                                    reason = "COLD_SHADOW_OR_OBSTACLE"
                                else:
                                    pred_verified = True
                                    status = "VERIFIED"
                                    reason = "NONE"
                            else:
                                reason = "OUT_OF_THERMAL_FOV"
                        else:
                            reason = "OUT_OF_THERMAL_FOV"
                    else:
                        reason = "MISSING_SENSOR_DATA"
                else:
                    reason = "INVALID_DEPTH_OR_CONF"

            # -------------------------------------------------------------
            # Mode 4: Temperature + Contrast + Spatial Morphology
            # -------------------------------------------------------------
            elif mode == "4_TEMP_AND_MORPHOLOGY":
                if sc.rgb_confidence >= 0.35 and sc.depth_image is not None and (3.0 <= sc.depth_m <= 35.0):
                    if sc.thermal_image_k is not None and not np.all(np.isnan(sc.thermal_image_k)):
                        bbox_th, fully, part = self.core.project_rgb_bbox_to_thermal(sc.rgb_bbox, sc.depth_m)
                        if part and bbox_th is not None:
                            stats = self.core.analyze_thermal_roi(sc.thermal_image_k, bbox_th)
                            if stats is not None:
                                if stats.max_k > 318.15:
                                    reason = "HOT_NON_HUMAN_CLUTTER"
                                elif stats.max_k < 296.0 or stats.delta_k < 3.0:
                                    reason = "COLD_SHADOW_OR_OBSTACLE"
                                elif stats.hot_pixel_fraction > 0.90 and stats.core_periphery_delta_k < 0.30:
                                    reason = "UNIFORM_DIFFUSE_HEAT"
                                elif stats.core_periphery_delta_k <= 0.0:
                                    reason = "NON_HUMAN_THERMAL_MORPHOLOGY"
                                elif stats.grad_mean < 0.20:
                                    reason = "LOW_THERMAL_GRADIENT"
                                else:
                                    pred_verified = True
                                    status = "VERIFIED"
                                    reason = "NONE"
                            else:
                                reason = "OUT_OF_THERMAL_FOV"
                        else:
                            reason = "OUT_OF_THERMAL_FOV"
                    else:
                        reason = "MISSING_SENSOR_DATA"
                else:
                    reason = "INVALID_DEPTH_OR_CONF"

            # -------------------------------------------------------------
            # Mode 5: Full Stage 4 Multi-Feature Verification
            # -------------------------------------------------------------
            elif mode == "5_FULL_STAGE4_MVS":
                res = self.core.verify_candidate(
                    candidate_id=f"ablation_{sc.scenario_id}",
                    drone_id=sc.drone_id,
                    timestamp=1.0,
                    rgb_bbox=sc.rgb_bbox,
                    rgb_confidence=sc.rgb_confidence,
                    depth_image=sc.depth_image,
                    thermal_image_k=sc.thermal_image_k,
                    drone_pose=sc.drone_pose
                )
                pred_verified = (res.status in [VerificationStatus.THERMAL_SUPPORTED.value, "VERIFIED"])
                status = res.status
                reason = res.rejection_reason
                mvs_score = res.verification_score

            # -------------------------------------------------------------
            # Mode 6: Full Stage 4 + Temporal Confirmation
            # -------------------------------------------------------------
            elif mode == "6_FULL_PLUS_TEMPORAL":
                res = self.core.verify_candidate(
                    candidate_id=f"ablation_{sc.scenario_id}",
                    drone_id=sc.drone_id,
                    timestamp=1.0,
                    rgb_bbox=sc.rgb_bbox,
                    rgb_confidence=sc.rgb_confidence,
                    depth_image=sc.depth_image,
                    thermal_image_k=sc.thermal_image_k,
                    drone_pose=sc.drone_pose
                )
                if res.status in [VerificationStatus.THERMAL_SUPPORTED.value, "VERIFIED"]:
                    # Feed 4 distinct sequential frames separated by 0.15s to simulate multi-frame arrival
                    scenario_confirmer = TemporalEventConfirmer(window_size=6, consensus_needed=4, min_time_span=0.4)
                    disc_event = None
                    for step_i in range(4):
                        ts = 1.0 + step_i * 0.15
                        res_copy = type(res)(
                            candidate_id=f"ablation_{sc.scenario_id}_{step_i}",
                            drone_id=sc.drone_id,
                            timestamp=ts,
                            rgb_confidence=res.rgb_confidence,
                            rgb_bbox=res.rgb_bbox,
                            projected_thermal_bbox=res.projected_thermal_bbox,
                            depth_m=res.depth_m,
                            physical_dims_m=res.physical_dims_m,
                            thermal_stats=res.thermal_stats,
                            status=res.status,
                            rejection_reason=res.rejection_reason,
                            world_pos=res.world_pos,
                            grid_pos=res.grid_pos,
                            evidence_state=res.evidence_state,
                            verification_score=res.verification_score,
                            uncertainty_score=res.uncertainty_score
                        )
                        tid, disc_event = scenario_confirmer.add_multispectral_observation(res_copy)

                    if disc_event is not None:
                        pred_verified = True
                        status = VerificationStatus.TEMPORALLY_CONFIRMED.value
                        reason = "NONE"
                    else:
                        pred_verified = False
                        status = VerificationStatus.RGB_CANDIDATE.value
                        reason = "INSUFFICIENT_TEMPORAL_CONSENSUS"
                else:
                    pred_verified = False
                    status = res.status
                    reason = res.rejection_reason
                mvs_score = res.verification_score

            records.append({
                "scenario_id": sc.scenario_id,
                "name": sc.name,
                "category": sc.category,
                "subcategory": sc.subcategory,
                "gt_is_victim": sc.gt_is_victim,
                "pred_verified": pred_verified,
                "status": status,
                "rejection_reason": reason,
                "mvs_score": mvs_score
            })

        return records
