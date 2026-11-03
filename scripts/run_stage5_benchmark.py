#!/usr/bin/env python3
"""
Master Execution Script for Stage 5 Benchmark (Phase 2 Remediation)
Statistical Evaluation, Robustness Benchmark & Ablation Suite

Usage:
  python3 scripts/run_stage5_benchmark.py
"""

import os
import sys
import json
import csv
import time
from typing import Dict, Any, List

# Ensure workspace packages can be imported standalone
ws_src = os.path.abspath(os.path.join(os.path.dirname(__file__), '../drone_ws/src/swarm_controller'))
if ws_src not in sys.path:
    sys.path.insert(0, ws_src)

scripts_dir = os.path.abspath(os.path.dirname(__file__))
if scripts_dir not in sys.path:
    sys.path.insert(0, scripts_dir)

from evaluation.scenario_generator import ScenarioGenerator, EvaluationScenario
from evaluation.metrics_calculator import MetricsCalculator, MetricSummary
from evaluation.ablation_runner import AblationRunner
from evaluation.multi_drone_evaluator import MultiDroneEvaluator
from evaluation.latency_profiler import LatencyProfiler
from swarm_controller.rgb_thermal_fusion_verifier import (
    RGBThermalFusionCore,
    VerificationStatus,
    RejectionReason
)


def print_header(title: str, width: int = 80):
    print("=" * width)
    print(f" {title.upper()} ".center(width, "="))
    print("=" * width)


def format_table_row(cols: List[str], widths: List[int], align: str = "left") -> str:
    parts = []
    for c, w in zip(cols, widths):
        if align == "right":
            parts.append(str(c).rjust(w))
        elif align == "center":
            parts.append(str(c).center(w))
        else:
            parts.append(str(c).ljust(w))
    return " | ".join(parts)


def main():
    print_header("Phase 2 Remediation — Stage 5 Statistical Evaluation Benchmark")
    output_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../results/benchmark_stage5"))
    os.makedirs(output_dir, exist_ok=True)

    # -------------------------------------------------------------
    # 1. Generate Evaluation Scenarios
    # -------------------------------------------------------------
    print("\n[1/5] Generating Diverse Evaluation Scenarios (Target N >= 100)...")
    gen = ScenarioGenerator(master_seed=42)
    scenarios = gen.generate_all_scenarios(total_count=150)

    n_pos = sum(1 for s in scenarios if s.gt_is_victim)
    n_distractor = sum(1 for s in scenarios if s.category == "THERMAL_DISTRACTOR")
    n_failure = sum(1 for s in scenarios if s.category == "SENSOR_FAILURE")
    print(f"      Generated {len(scenarios)} Deterministic Scenarios (Seed 42):")
    print(f"        - Human-Positive Scenarios:     {n_pos:3d} ({n_pos/len(scenarios)*100:.1f}%)")
    print(f"        - Thermal Distractor Scenarios: {n_distractor:3d} ({n_distractor/len(scenarios)*100:.1f}%)")
    print(f"        - Sensor & Perception Failures: {n_failure:3d} ({n_failure/len(scenarios)*100:.1f}%)")

    # -------------------------------------------------------------
    # 2. Execute Full Stage 4 Multi-Feature Verification Benchmark
    # -------------------------------------------------------------
    print("\n[2/5] Evaluating Stage 4 Multi-Feature Verification Pipeline...")
    core = RGBThermalFusionCore()
    eval_records = []

    for sc in scenarios:
        res = core.verify_candidate(
            candidate_id=f"sc_{sc.scenario_id:03d}",
            drone_id=sc.drone_id,
            timestamp=1.0,
            rgb_bbox=sc.rgb_bbox,
            rgb_confidence=sc.rgb_confidence,
            depth_image=sc.depth_image,
            thermal_image_k=sc.thermal_image_k,
            drone_pose=sc.drone_pose
        )

        pred_verified = (res.status in [VerificationStatus.THERMAL_SUPPORTED.value, "VERIFIED"])
        eval_records.append({
            "scenario_id": sc.scenario_id,
            "name": sc.name,
            "category": sc.category,
            "subcategory": sc.subcategory,
            "gt_is_victim": sc.gt_is_victim,
            "pred_verified": pred_verified,
            "status": res.status,
            "rejection_reason": res.rejection_reason,
            "mvs_score": res.verification_score,
            "uncertainty_score": res.uncertainty_score,
            "depth_m": sc.depth_m,
            "rgb_conf": sc.rgb_confidence
        })

    summary = MetricsCalculator.compute_summary(eval_records)
    cat_breakdown = MetricsCalculator.breakdown_by_category(eval_records)
    subcat_breakdown = MetricsCalculator.breakdown_by_subcategory(eval_records)
    failures = MetricsCalculator.get_failure_cases(eval_records)

    # Print Summary Metrics Table
    print_header("Stage 4 Benchmark Summary Metrics (N=150 Scenarios)")
    print(f"Confusion Matrix:")
    print(f"  True Positives (TP):  {summary.tp:3d}  | False Positives (FP): {summary.fp:3d}")
    print(f"  False Negatives (FN): {summary.fn:3d}  | True Negatives (TN):  {summary.tn:3d}")
    print(f"  Total Scenarios:      {summary.total:3d}")
    print("-" * 80)
    print(f"  Precision:              {summary.precision*100:6.2f}% (95% CI: [{summary.precision_ci95[0]*100:.1f}%, {summary.precision_ci95[1]*100:.1f}%])")
    print(f"  Recall (Sensitivity):   {summary.recall*100:6.2f}% (95% CI: [{summary.recall_ci95[0]*100:.1f}%, {summary.recall_ci95[1]*100:.1f}%])")
    print(f"  Specificity:            {summary.specificity*100:6.2f}% (95% CI: [{summary.specificity_ci95[0]*100:.1f}%, {summary.specificity_ci95[1]*100:.1f}%])")
    print(f"  F1 Score:               {summary.f1_score*100:6.2f}%")
    print(f"  False Positive Rate:    {summary.fpr*100:6.2f}%")
    print(f"  Overall Accuracy:       {summary.accuracy*100:6.2f}% (95% CI: [{summary.accuracy_ci95[0]*100:.1f}%, {summary.accuracy_ci95[1]*100:.1f}%])")

    # Print Category Breakdown Table
    print("\n" + "-" * 80)
    print("Category-Level Performance Breakdown:")
    c_heads = ["Category", "Count", "TP", "FP", "TN", "FN", "Accuracy", "F1 Score"]
    c_widths = [22, 6, 4, 4, 4, 4, 10, 10]
    print(format_table_row(c_heads, c_widths))
    print("-" * 80)
    for cat_name, c_data in cat_breakdown.items():
        row = [
            cat_name,
            str(c_data["total"]),
            str(c_data["tp"]),
            str(c_data["fp"]),
            str(c_data["tn"]),
            str(c_data["fn"]),
            f"{c_data['accuracy']*100:.1f}%",
            f"{c_data['f1_score']*100:.1f}%"
        ]
        print(format_table_row(row, c_widths))

    # Print Failure Cases if any
    print("\n" + "-" * 80)
    if failures:
        print(f"Failure Cases Identified ({len(failures)} total):")
        for f in failures:
            print(f"  - [{f['type']}] Scenario {f['scenario_id']:03d} ({f['name']}): status={f['status']}, reason={f['rejection_reason']}, MVS={f['mvs_score']:.2f}")
    else:
        print("Failure Cases Identified: 0 (Perfect Separation on Stage 5 Dataset)")

    # -------------------------------------------------------------
    # 3. Systematic Ablation Study (6 Modes)
    # -------------------------------------------------------------
    print_header("Systematic Ablation Study (6 Perception Modes across N=150)")
    ablation_runner = AblationRunner()
    ablation_results = ablation_runner.run_all_ablations(scenarios)

    a_heads = ["Ablation Mode", "TP", "FP", "TN", "FN", "Precision", "Recall", "F1-Score", "FPR"]
    a_widths = [24, 4, 4, 4, 4, 10, 10, 10, 8]
    print(format_table_row(a_heads, a_widths))
    print("-" * 88)

    for mode_name, m_res in ablation_results.items():
        s = m_res["summary"]
        row = [
            mode_name,
            str(s["tp"]),
            str(s["fp"]),
            str(s["tn"]),
            str(s["fn"]),
            f"{s['precision']*100:5.1f}%",
            f"{s['recall']*100:5.1f}%",
            f"{s['f1_score']*100:5.1f}%",
            f"{s['fpr']*100:4.1f}%"
        ]
        print(format_table_row(row, a_widths))

    # -------------------------------------------------------------
    # 4. Multi-Drone Spatio-Temporal Consensus Evaluation
    # -------------------------------------------------------------
    print_header("Multi-Drone Consensus Subsystem Evaluation")
    multi_drone_results = MultiDroneEvaluator.run_all_tests()
    for t_name, t_data in multi_drone_results["test_results"].items():
        status_str = "[PASS]" if t_data["passed"] else "[FAIL]"
        print(f"  {status_str} {t_name}")
        if t_data["passed"]:
            print(f"         {t_data['details']}")
        else:
            print(f"         {t_data['error']}")

    print(f"\nMulti-Drone Consensus Test Result: {multi_drone_results['passed_tests']}/{multi_drone_results['total_tests']} PASSED")

    # -------------------------------------------------------------
    # 5. Latency & Resource Benchmarking
    # -------------------------------------------------------------
    print_header("Latency & Computational Resource Benchmark")
    lat_data = LatencyProfiler.profile_all(warmup_iters=30, bench_iters=150)

    roi_timing = lat_data["roi_feature_extraction"]
    ver_timing = lat_data["candidate_verification"]
    conf_timing = lat_data["temporal_confirmation"]

    print("Execution Timing Breakdown (Measured across 150 iterations):")
    print(f"  1. Pure ROI Thermal Feature Extraction:")
    print(f"     Mean:   {roi_timing['mean_ms']:.3f} ms (Std: {roi_timing['std_ms']:.3f} ms)")
    print(f"     Median: {roi_timing['median_ms']:.3f} ms | P95: {roi_timing['p95_ms']:.3f} ms | P99: {roi_timing['p99_ms']:.3f} ms")
    print(f"     Throughput: {roi_timing['throughput_ops_per_sec']:.0f} ROIs/sec")
    print(f"  2. Complete Candidate Verification (verify_candidate):")
    print(f"     Mean:   {ver_timing['mean_ms']:.3f} ms (Std: {ver_timing['std_ms']:.3f} ms)")
    print(f"     Median: {ver_timing['median_ms']:.3f} ms | P95: {ver_timing['p95_ms']:.3f} ms | P99: {ver_timing['p99_ms']:.3f} ms")
    print(f"     Throughput: {ver_timing['throughput_ops_per_sec']:.0f} candidates/sec")
    print(f"  3. Spatio-Temporal Confirmation (add_observation):")
    print(f"     Mean:   {conf_timing['mean_ms']:.3f} ms | Median: {conf_timing['median_ms']:.3f} ms | P95: {conf_timing['p95_ms']:.3f} ms")
    print(f"  4. Composite Post-Detection Perception Latency (Verification + Consensus):")
    print(f"     Total:  {lat_data['composite_per_candidate_ms']:.3f} ms per candidate")
    print(f"  5. Empirical Batched YOLOv8n GPU Inference Reference (RTX 3050 Laptop GPU):")
    for b_name, b_val in lat_data["yolo_gpu_inference"].items():
        print(f"     {b_name.upper()}: Mean Latency = {b_val['mean_latency_ms']:.2f} ms | P95 = {b_val['p95_ms']:.2f} ms | Total FPS = {b_val['fps_total']:.1f} | VRAM = {b_val['vram_mb']:.1f} MB")

    print(f"  6. Host Resource Consumption:")
    print(f"     Process Memory (RSS): {lat_data['resource_usage']['memory_rss_mb']:.1f} MB (Delta: {lat_data['resource_usage']['memory_delta_mb']:+.2f} MB)")
    print(f"     CPU Utilization:      {lat_data['resource_usage']['cpu_percent']:.1f}%")

    # -------------------------------------------------------------
    # 6. Export Machine-Readable Datasets
    # -------------------------------------------------------------
    print_header("Exporting Machine-Readable Benchmark Artifacts")

    # 6.1 CSV of all 150 scenario evaluations
    csv_path = os.path.join(output_dir, "scenario_eval_records.csv")
    csv_fields = ["scenario_id", "name", "category", "subcategory", "gt_is_victim",
                  "pred_verified", "status", "rejection_reason", "mvs_score",
                  "uncertainty_score", "depth_m", "rgb_conf"]
    with open(csv_path, mode="w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=csv_fields)
        writer.writeheader()
        for r in eval_records:
            writer.writerow(r)
    print(f"  - Saved scenario evaluations CSV: {csv_path}")

    # 6.2 JSON summary metrics
    json_summary_path = os.path.join(output_dir, "summary_metrics.json")
    with open(json_summary_path, "w") as f:
        json.dump({
            "overall_summary": summary.to_dict(),
            "category_breakdown": cat_breakdown,
            "subcategory_breakdown": subcat_breakdown,
            "failures": failures
        }, f, indent=2)
    print(f"  - Saved summary metrics JSON:      {json_summary_path}")

    # 6.3 JSON ablation results
    json_ablation_path = os.path.join(output_dir, "ablation_results.json")
    ablation_export = {k: v["summary"] for k, v in ablation_results.items()}
    with open(json_ablation_path, "w") as f:
        json.dump(ablation_export, f, indent=2)
    print(f"  - Saved ablation study JSON:       {json_ablation_path}")

    # 6.4 JSON multi-drone results
    json_multidrone_path = os.path.join(output_dir, "multi_drone_results.json")
    with open(json_multidrone_path, "w") as f:
        json.dump(multi_drone_results, f, indent=2)
    print(f"  - Saved multi-drone results JSON:  {json_multidrone_path}")

    # 6.5 JSON latency benchmark
    json_latency_path = os.path.join(output_dir, "latency_benchmark.json")
    with open(json_latency_path, "w") as f:
        json.dump(lat_data, f, indent=2)
    print(f"  - Saved latency benchmark JSON:    {json_latency_path}")

    print_header("Stage 5 Benchmark Execution Completed Successfully")
    return 0


if __name__ == '__main__':
    sys.exit(main())
