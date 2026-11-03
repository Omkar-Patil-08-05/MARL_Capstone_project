#!/usr/bin/env python3
"""
Ground-Truth Spatial Matching & Performance Evaluation Module (Phase 4 / Evaluation)

Features:
  1. Strict One-to-One Spatial Matching:
     Uses the Hungarian algorithm (linear sum assignment) on horizontal 2D Euclidean
     distance matrices between autonomous victim tracks and ground-truth coordinates.
  2. Configurable Spatial Matching Gate:
     Configurable matching threshold (default 3.0m). Matches beyond this distance are
     classified as False Positives, and the corresponding GT as False Negatives.
  3. Track-Level Identity & Deduplication:
     Evaluates against unique autonomous tracks (or deduplicated spatial clusters) to
     strictly prevent repeated frames of the same victim from inflating True Positives.
  4. Complete Metric Suite:
     Computes TP, FP, FN, Precision, Recall, F1-Score, Mean Error, Median Error,
     Max Error, RMSE, and per-drone detection distribution.
  5. Export Capabilities:
     Exports full audit details to JSON and CSV formats for formal reporting.
"""

import math
import json
import csv
from typing import List, Dict, Optional, Tuple, Any
from dataclasses import dataclass, asdict, field

try:
    import numpy as np
    from scipy.optimize import linear_sum_assignment
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False


@dataclass
class MatchPair:
    gt_id: str
    track_id: str
    gt_x: float
    gt_y: float
    est_x: float
    est_y: float
    distance_error_m: float
    confidence: float
    drone_id: Optional[int] = None
    verification_mode: Optional[str] = None
    sim_time: Optional[float] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "gt_id": self.gt_id,
            "track_id": self.track_id,
            "gt_pos": [round(self.gt_x, 2), round(self.gt_y, 2)],
            "est_pos": [round(self.est_x, 2), round(self.est_y, 2)],
            "error_m": round(self.distance_error_m, 3),
            "confidence": round(self.confidence, 3),
            "drone_id": self.drone_id,
            "verification_mode": self.verification_mode,
            "sim_time": round(self.sim_time, 3) if self.sim_time is not None else None
        }


@dataclass
class GroundTruthMatchResult:
    total_gt: int
    total_tracks: int
    tp: int
    fp: int
    fn: int
    precision: float
    recall: float
    f1_score: float
    matching_threshold_m: float
    mean_error_m: float
    median_error_m: float
    max_error_m: float
    rmse_m: float
    per_drone_detections: Dict[int, int]
    matched_pairs: List[MatchPair] = field(default_factory=list)
    unmatched_gt: List[Dict[str, Any]] = field(default_factory=list)
    unmatched_tracks: List[Dict[str, Any]] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "total_ground_truth_victims": self.total_gt,
            "total_autonomous_tracks": self.total_tracks,
            "true_positives": self.tp,
            "false_positives": self.fp,
            "false_negatives": self.fn,
            "precision": round(self.precision, 4),
            "recall": round(self.recall, 4),
            "f1_score": round(self.f1_score, 4),
            "matching_threshold_m": self.matching_threshold_m,
            "mean_localization_error_m": round(self.mean_error_m, 3),
            "median_localization_error_m": round(self.median_error_m, 3),
            "max_localization_error_m": round(self.max_error_m, 3),
            "rmse_localization_error_m": round(self.rmse_m, 3),
            "per_drone_detections": self.per_drone_detections,
            "matched_pairs": [p.to_dict() for p in self.matched_pairs],
            "unmatched_ground_truth": self.unmatched_gt,
            "unmatched_autonomous_tracks": self.unmatched_tracks
        }


class GroundTruthMatcher:
    """
    Rigorously matches autonomous victim detections/tracks against ground-truth coordinates.
    """

    def __init__(self, default_threshold_m: float = 3.0):
        self.default_threshold_m = float(default_threshold_m)

    @staticmethod
    def _euclidean_dist_2d(x1: float, y1: float, x2: float, y2: float) -> float:
        return math.hypot(x1 - x2, y1 - y2)

    def match(
        self,
        ground_truth: List[Dict[str, Any]],
        autonomous_tracks: List[Dict[str, Any]],
        threshold_m: Optional[float] = None
    ) -> GroundTruthMatchResult:
        """
        Executes globally optimal one-to-one spatial matching between autonomous tracks
        and ground-truth victims.

        Args:
            ground_truth: List of dicts with keys ['id', 'world_x', 'world_y'] (and optional 'world_z')
            autonomous_tracks: List of dicts with keys ['id', 'world_x', 'world_y', 'confidence']
            threshold_m: Maximum horizontal Euclidean distance in meters to accept a true positive match.
        """
        gate = float(threshold_m if threshold_m is not None else self.default_threshold_m)
        num_gt = len(ground_truth)
        num_det = len(autonomous_tracks)

        # Per-drone counter
        per_drone: Dict[int, int] = {}
        for d in autonomous_tracks:
            d_id = d.get('drone_id', d.get('detected_by'))
            if d_id is not None:
                try:
                    d_int = int(d_id)
                    per_drone[d_int] = per_drone.get(d_int, 0) + 1
                except (ValueError, TypeError):
                    pass

        # Handle trivial edge cases
        if num_gt == 0 and num_det == 0:
            return GroundTruthMatchResult(
                total_gt=0, total_tracks=0, tp=0, fp=0, fn=0,
                precision=1.0, recall=1.0, f1_score=1.0,
                matching_threshold_m=gate,
                mean_error_m=0.0, median_error_m=0.0, max_error_m=0.0, rmse_m=0.0,
                per_drone_detections=per_drone
            )

        if num_gt == 0:
            return GroundTruthMatchResult(
                total_gt=0, total_tracks=num_det, tp=0, fp=num_det, fn=0,
                precision=0.0, recall=0.0, f1_score=0.0,
                matching_threshold_m=gate,
                mean_error_m=0.0, median_error_m=0.0, max_error_m=0.0, rmse_m=0.0,
                per_drone_detections=per_drone,
                unmatched_tracks=[dict(d) for d in autonomous_tracks]
            )

        if num_det == 0:
            return GroundTruthMatchResult(
                total_gt=num_gt, total_tracks=0, tp=0, fp=0, fn=num_gt,
                precision=0.0, recall=0.0, f1_score=0.0,
                matching_threshold_m=gate,
                mean_error_m=0.0, median_error_m=0.0, max_error_m=0.0, rmse_m=0.0,
                per_drone_detections=per_drone,
                unmatched_gt=[dict(g) for g in ground_truth]
            )

        # Build distance matrix (rows: detections, cols: ground truth)
        cost_matrix = []
        for det in autonomous_tracks:
            row = []
            dx = float(det['world_x'])
            dy = float(det['world_y'])
            for gt in ground_truth:
                gx = float(gt['world_x'])
                gy = float(gt['world_y'])
                row.append(self._euclidean_dist_2d(dx, dy, gx, gy))
            cost_matrix.append(row)

        matched_pairs: List[MatchPair] = []
        matched_det_indices = set()
        matched_gt_indices = set()

        if SCIPY_AVAILABLE:
            cost_arr = np.array(cost_matrix)
            row_ind, col_ind = linear_sum_assignment(cost_arr)
            for r, c in zip(row_ind, col_ind):
                dist = cost_arr[r, c]
                if dist <= gate:
                    det = autonomous_tracks[r]
                    gt = ground_truth[c]
                    matched_det_indices.add(r)
                    matched_gt_indices.add(c)
                    matched_pairs.append(MatchPair(
                        gt_id=str(gt.get('id', f'gt_{c}')),
                        track_id=str(det.get('id', f'track_{r}')),
                        gt_x=float(gt['world_x']),
                        gt_y=float(gt['world_y']),
                        est_x=float(det['world_x']),
                        est_y=float(det['world_y']),
                        distance_error_m=float(dist),
                        confidence=float(det.get('confidence', 1.0)),
                        drone_id=det.get('drone_id', det.get('detected_by')),
                        verification_mode=det.get('verification_mode'),
                        sim_time=det.get('sim_time', det.get('timestamp'))
                    ))
        else:
            # Optimal greedy bipartite matching fallback
            candidate_pairs = []
            for r in range(num_det):
                for c in range(num_gt):
                    candidate_pairs.append((cost_matrix[r][c], r, c))
            candidate_pairs.sort(key=lambda x: x[0])

            for dist, r, c in candidate_pairs:
                if dist > gate:
                    break
                if r not in matched_det_indices and c not in matched_gt_indices:
                    matched_det_indices.add(r)
                    matched_gt_indices.add(c)
                    det = autonomous_tracks[r]
                    gt = ground_truth[c]
                    matched_pairs.append(MatchPair(
                        gt_id=str(gt.get('id', f'gt_{c}')),
                        track_id=str(det.get('id', f'track_{r}')),
                        gt_x=float(gt['world_x']),
                        gt_y=float(gt['world_y']),
                        est_x=float(det['world_x']),
                        est_y=float(det['world_y']),
                        distance_error_m=float(dist),
                        confidence=float(det.get('confidence', 1.0)),
                        drone_id=det.get('drone_id', det.get('detected_by')),
                        verification_mode=det.get('verification_mode'),
                        sim_time=det.get('sim_time', det.get('timestamp'))
                    ))

        # Classify outcomes
        tp = len(matched_pairs)
        fp = num_det - tp
        fn = num_gt - tp

        precision = (tp / (tp + fp)) if (tp + fp) > 0 else 0.0
        recall = (tp / (tp + fn)) if (tp + fn) > 0 else 0.0
        f1 = (2.0 * precision * recall / (precision + recall)) if (precision + recall) > 0 else 0.0

        # Localization error statistics
        errors = [p.distance_error_m for p in matched_pairs]
        if errors:
            mean_err = float(sum(errors) / len(errors))
            sorted_err = sorted(errors)
            n_err = len(sorted_err)
            if n_err % 2 == 1:
                median_err = float(sorted_err[n_err // 2])
            else:
                median_err = float((sorted_err[n_err // 2 - 1] + sorted_err[n_err // 2]) / 2.0)
            max_err = float(max(errors))
            rmse_err = float(math.sqrt(sum(e**2 for e in errors) / len(errors)))
        else:
            mean_err = 0.0
            median_err = 0.0
            max_err = 0.0
            rmse_err = 0.0

        unmatched_gt = [ground_truth[c] for c in range(num_gt) if c not in matched_gt_indices]
        unmatched_tracks = [autonomous_tracks[r] for r in range(num_det) if r not in matched_det_indices]

        return GroundTruthMatchResult(
            total_gt=num_gt,
            total_tracks=num_det,
            tp=tp,
            fp=fp,
            fn=fn,
            precision=precision,
            recall=recall,
            f1_score=f1,
            matching_threshold_m=gate,
            mean_error_m=mean_err,
            median_error_m=median_err,
            max_error_m=max_err,
            rmse_m=rmse_err,
            per_drone_detections=per_drone,
            matched_pairs=matched_pairs,
            unmatched_gt=unmatched_gt,
            unmatched_tracks=unmatched_tracks
        )

    @staticmethod
    def export_json(result: GroundTruthMatchResult, output_path: str):
        with open(output_path, 'w') as f:
            json.dump(result.to_dict(), f, indent=2)

    @staticmethod
    def export_csv(result: GroundTruthMatchResult, output_path: str):
        with open(output_path, 'w', newline='') as f:
            writer = csv.writer(f)
            # Section 1: Summary KPIs
            writer.writerow(["Metric", "Value"])
            writer.writerow(["Total Ground Truth", result.total_gt])
            writer.writerow(["Total Autonomous Tracks", result.total_tracks])
            writer.writerow(["True Positives (TP)", result.tp])
            writer.writerow(["False Positives (FP)", result.fp])
            writer.writerow(["False Negatives (FN)", result.fn])
            writer.writerow(["Precision", f"{result.precision:.4f}"])
            writer.writerow(["Recall", f"{result.recall:.4f}"])
            writer.writerow(["F1-Score", f"{result.f1_score:.4f}"])
            writer.writerow(["Matching Threshold (m)", result.matching_threshold_m])
            writer.writerow(["Mean Error (m)", f"{result.mean_error_m:.3f}"])
            writer.writerow(["Median Error (m)", f"{result.median_error_m:.3f}"])
            writer.writerow(["Max Error (m)", f"{result.max_error_m:.3f}"])
            writer.writerow(["RMSE (m)", f"{result.rmse_m:.3f}"])
            writer.writerow([])

            # Section 2: Matched Pairs
            writer.writerow(["Matched GT ID", "Track ID", "GT X", "GT Y", "Est X", "Est Y", "Error (m)", "Confidence", "Drone ID", "Mode", "Sim Time"])
            for p in result.matched_pairs:
                writer.writerow([
                    p.gt_id, p.track_id, p.gt_x, p.gt_y, p.est_x, p.est_y,
                    f"{p.distance_error_m:.3f}", f"{p.confidence:.3f}",
                    p.drone_id, p.verification_mode, p.sim_time
                ])
            writer.writerow([])

            # Section 3: Unmatched GT
            writer.writerow(["Unmatched GT ID", "GT X", "GT Y"])
            for g in result.unmatched_gt:
                writer.writerow([g.get('id'), g.get('world_x'), g.get('world_y')])
            writer.writerow([])

            # Section 4: Unmatched Tracks
            writer.writerow(["Unmatched Track ID", "Est X", "Est Y", "Confidence", "Drone ID"])
            for t in result.unmatched_tracks:
                writer.writerow([t.get('id'), t.get('world_x'), t.get('world_y'), t.get('confidence'), t.get('drone_id')])
