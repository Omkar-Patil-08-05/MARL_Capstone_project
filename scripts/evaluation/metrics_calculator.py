#!/usr/bin/env python3
"""
Evaluation Metrics Calculator (Phase 2 Remediation - Stage 5)

Computes:
  - Confusion Matrix (TP, FP, TN, FN)
  - Precision, Recall, Specificity, Accuracy, F1-Score, False Positive Rate (FPR)
  - 95% Wilson Score Confidence Intervals
  - Per-Category and Per-Subcategory Statistical Breakdowns
  - Failure Case Analysis
"""

import math
from typing import List, Dict, Any, Tuple
from dataclasses import dataclass, asdict


@dataclass
class MetricSummary:
    tp: int
    fp: int
    tn: int
    fn: int
    total: int
    precision: float
    precision_ci95: Tuple[float, float]
    recall: float
    recall_ci95: Tuple[float, float]
    specificity: float
    specificity_ci95: Tuple[float, float]
    accuracy: float
    accuracy_ci95: Tuple[float, float]
    f1_score: float
    fpr: float

    def to_dict(self) -> Dict[str, Any]:
        return {
            "tp": self.tp,
            "fp": self.fp,
            "tn": self.tn,
            "fn": self.fn,
            "total": self.total,
            "precision": round(self.precision, 4),
            "precision_ci95": [round(x, 4) for x in self.precision_ci95],
            "recall": round(self.recall, 4),
            "recall_ci95": [round(x, 4) for x in self.recall_ci95],
            "specificity": round(self.specificity, 4),
            "specificity_ci95": [round(x, 4) for x in self.specificity_ci95],
            "accuracy": round(self.accuracy, 4),
            "accuracy_ci95": [round(x, 4) for x in self.accuracy_ci95],
            "f1_score": round(self.f1_score, 4),
            "fpr": round(self.fpr, 4)
        }


def wilson_score_ci(k: int, n: int, confidence: float = 0.95) -> Tuple[float, float]:
    """Computes Wilson score interval for proportion k/n."""
    if n <= 0:
        return (0.0, 0.0)
    z = 1.95996  # 95% two-sided normal quantile
    p = k / n
    denom = 1.0 + (z**2) / n
    center = (p + (z**2) / (2 * n)) / denom
    margin = (z * math.sqrt((p * (1.0 - p) + (z**2) / (4 * n)) / n)) / denom
    return (max(0.0, center - margin), min(1.0, center + margin))


class MetricsCalculator:
    """Computes rigorous statistical evaluation metrics and breakdowns."""

    @staticmethod
    def compute_summary(records: List[Dict[str, Any]]) -> MetricSummary:
        """
        records: list of dicts with keys:
          'gt_is_victim': bool
          'pred_verified': bool
        """
        tp = sum(1 for r in records if r['gt_is_victim'] and r['pred_verified'])
        fp = sum(1 for r in records if not r['gt_is_victim'] and r['pred_verified'])
        tn = sum(1 for r in records if not r['gt_is_victim'] and not r['pred_verified'])
        fn = sum(1 for r in records if r['gt_is_victim'] and not r['pred_verified'])
        total = len(records)

        precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
        recall = tp / (tp + fn) if (tp + fn) > 0 else 0.0
        specificity = tn / (tn + fp) if (tn + fp) > 0 else 0.0
        accuracy = (tp + tn) / total if total > 0 else 0.0
        f1 = (2.0 * precision * recall) / (precision + recall) if (precision + recall) > 0 else 0.0
        fpr = fp / (fp + tn) if (fp + tn) > 0 else 0.0

        p_ci = wilson_score_ci(tp, tp + fp)
        r_ci = wilson_score_ci(tp, tp + fn)
        s_ci = wilson_score_ci(tn, tn + fp)
        a_ci = wilson_score_ci(tp + tn, total)

        return MetricSummary(
            tp=tp, fp=fp, tn=tn, fn=fn, total=total,
            precision=precision, precision_ci95=p_ci,
            recall=recall, recall_ci95=r_ci,
            specificity=specificity, specificity_ci95=s_ci,
            accuracy=accuracy, accuracy_ci95=a_ci,
            f1_score=f1, fpr=fpr
        )

    @staticmethod
    def breakdown_by_category(records: List[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
        cats = sorted(list({r['category'] for r in records}))
        breakdowns = {}
        for c in cats:
            sub = [r for r in records if r['category'] == c]
            m = MetricsCalculator.compute_summary(sub)
            breakdowns[c] = m.to_dict()
        return breakdowns

    @staticmethod
    def breakdown_by_subcategory(records: List[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
        subcats = sorted(list({r['subcategory'] for r in records}))
        breakdowns = {}
        for sc in subcats:
            sub = [r for r in records if r['subcategory'] == sc]
            m = MetricsCalculator.compute_summary(sub)
            breakdowns[sc] = m.to_dict()
        return breakdowns

    @staticmethod
    def get_failure_cases(records: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """Returns all False Positives (FP) and False Negatives (FN)."""
        failures = []
        for r in records:
            if r['gt_is_victim'] and not r['pred_verified']:
                failures.append({
                    "scenario_id": r["scenario_id"],
                    "name": r["name"],
                    "type": "FALSE_NEGATIVE",
                    "category": r["category"],
                    "subcategory": r["subcategory"],
                    "status": r.get("status"),
                    "rejection_reason": r.get("rejection_reason"),
                    "mvs_score": r.get("mvs_score"),
                    "notes": r.get("notes", "")
                })
            elif not r['gt_is_victim'] and r['pred_verified']:
                failures.append({
                    "scenario_id": r["scenario_id"],
                    "name": r["name"],
                    "type": "FALSE_POSITIVE",
                    "category": r["category"],
                    "subcategory": r["subcategory"],
                    "status": r.get("status"),
                    "rejection_reason": r.get("rejection_reason"),
                    "mvs_score": r.get("mvs_score"),
                    "notes": r.get("notes", "")
                })
        return failures
