#!/usr/bin/env python3
"""
Controlled Gazebo Runtime Evaluation of Multispectral Victim Verification (Phase 2)
Tests:
  - Target 1: Standing Human (casual_female) -> Expected: VERIFIED (TP)
  - Target 2: Sitting Human (rescue_randy_sitting) -> Expected: VERIFIED (TP)
  - Target 3: Lying Human (casual_female_lying) -> Expected: VERIFIED (TP)
  - Target 4: Cold Shadow / Obstacle (287K) -> Expected: REJECTED (TN)
  - Target 5: Hot Non-Human Distractor (345K) -> Expected: REJECTED (TN)

Measures Precision, Recall, False Positive Rate (FPR), and Confusion Matrix.
"""

import os
import sys
import time
import math
import subprocess
import numpy as np

# Ensure workspace packages can be imported
ws_src = os.path.abspath(os.path.join(os.path.dirname(__file__), '../drone_ws/src/swarm_controller'))
if ws_src not in sys.path:
    sys.path.insert(0, ws_src)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data

from swarm_controller.rgb_thermal_fusion_verifier import (
    RGBThermalFusionCore,
    VerificationStatus,
    RejectionReason
)

class EvaluationListener(Node):
    def __init__(self):
        super().__init__('multispectral_eval_listener')
        self.depth_frames = {}
        self.thermal_frames = {}

        for i in range(5):
            self.create_subscription(
                Image,
                f'/drone_{i}/camera/depth_image',
                lambda msg, d=i: self.depth_cb(msg, d),
                qos_profile_sensor_data
            )
            self.create_subscription(
                Image,
                f'/drone_{i}/thermal/image_raw',
                lambda msg, d=i: self.thermal_cb(msg, d),
                qos_profile_sensor_data
            )

    def depth_cb(self, msg, drone_id):
        self.depth_frames[drone_id] = msg

    def thermal_cb(self, msg, drone_id):
        self.thermal_frames[drone_id] = msg


def main():
    print("=" * 75)
    print("CONTROLLED GAZEBO MULTISPECTRAL EVALUATION (PHASE 2)")
    print("Targets: 3 Humans (Standing, Sitting, Lying) + 2 Distractors (Cold, Hot)")
    print("=" * 75)

    rclpy.init()
    node = EvaluationListener()

    print("Collecting synchronized sensor frames from Gazebo simulation (5 drones)...")
    start = time.time()
    while rclpy.ok() and (len(node.thermal_frames) < 5 or len(node.depth_frames) < 5):
        if time.time() - start > 15.0:
            break
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f"Captured: {len(node.thermal_frames)}/5 thermal streams, {len(node.depth_frames)}/5 depth streams.")
    if len(node.thermal_frames) < 5:
        print("ERROR: Did not capture all 5 streams within timeout!")
        return 1

    core = RGBThermalFusionCore(
        min_rgb_confidence=0.35,
        min_victim_temp_k=296.0,
        max_human_temp_k=318.15,
        min_thermal_delta_k=3.0
    )

    targets = [
        {"id": 0, "name": "Standing Human (casual_female)", "gt_is_victim": True, "expected": "VERIFIED"},
        {"id": 1, "name": "Sitting Human (rescue_randy_sitting)", "gt_is_victim": True, "expected": "VERIFIED"},
        {"id": 2, "name": "Lying Human (casual_female_lying)", "gt_is_victim": True, "expected": "VERIFIED"},
        {"id": 3, "name": "Cold Shadow / Obstacle (287.0 K)", "gt_is_victim": False, "expected": "REJECTED"},
        {"id": 4, "name": "Hot Non-Human Distractor (345.0 K)", "gt_is_victim": False, "expected": "REJECTED"}
    ]

    tp = 0
    fp = 0
    tn = 0
    fn = 0

    results_table = []

    # Standard candidate bbox centered over the target in RGB (320, 240)
    rgb_bbox = [295, 215, 345, 265] # ~50x50 px centered
    rgb_conf = 0.80

    for t in targets:
        d_id = t["id"]
        dep_msg = node.depth_frames.get(d_id)
        th_msg = node.thermal_frames.get(d_id)

        # Parse depth
        depth_arr = np.frombuffer(dep_msg.data, dtype=np.float32).reshape((dep_msg.height, dep_msg.width))
        
        # Parse thermal
        dtype = np.dtype('>u2') if th_msg.is_bigendian else np.dtype('<u2')
        th_raw = np.frombuffer(th_msg.data, dtype=dtype).reshape((th_msg.height, th_msg.width))
        th_k = core.decode_thermal_mono16(th_raw, th_msg.is_bigendian)

        # Run verification
        res = core.verify_candidate(
            candidate_id=f"eval_target_{d_id}",
            drone_id=d_id,
            timestamp=float(time.time()),
            rgb_bbox=rgb_bbox,
            rgb_confidence=rgb_conf,
            depth_image=depth_arr,
            thermal_image_k=th_k,
            drone_pose=(0.0, 0.0, 5.0, 0.0)
        )

        is_verified = (res.status == VerificationStatus.VERIFIED.value)
        is_victim_gt = t["gt_is_victim"]

        # Confusion matrix accounting
        if is_victim_gt and is_verified:
            tp += 1
            verdict = "TRUE POSITIVE (TP)"
        elif not is_victim_gt and is_verified:
            fp += 1
            verdict = "FALSE POSITIVE (FP)"
        elif not is_victim_gt and not is_verified:
            tn += 1
            verdict = "TRUE NEGATIVE (TN)"
        else: # is_victim_gt and not is_verified
            fn += 1
            verdict = "FALSE NEGATIVE (FN)"

        th_max = res.thermal_stats['max_k'] if res.thermal_stats else 0.0
        th_delta = res.thermal_stats['delta_k'] if res.thermal_stats else 0.0

        results_table.append({
            "Target": t["name"],
            "GT": "Human" if is_victim_gt else "Distractor",
            "Result": res.status,
            "Reason": res.rejection_reason,
            "T_max": f"{th_max:.1f} K ({th_max - 273.15:.1f} °C)",
            "Delta_T": f"+{th_delta:.1f} K" if th_delta >= 0 else f"{th_delta:.1f} K",
            "Verdict": verdict
        })

    print("-" * 75)
    print(f"{'Target Name':<38} | {'GT':<10} | {'Status':<10} | {'T_max':<14} | {'Verdict'}")
    print("-" * 75)
    for r in results_table:
        print(f"{r['Target']:<38} | {r['GT']:<10} | {r['Result']:<10} | {r['T_max']:<14} | {r['Verdict']}")
    print("-" * 75)

    precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
    recall = tp / (tp + fn) if (tp + fn) > 0 else 0.0
    fpr = fp / (fp + tn) if (fp + tn) > 0 else 0.0
    accuracy = (tp + tn) / (tp + tn + fp + fn) if (tp + tn + fp + fn) > 0 else 0.0

    print("\n" + "=" * 75)
    print("QUANTITATIVE BENCHMARK PERFORMANCE METRICS")
    print("=" * 75)
    print(f"True Positives (TP):     {tp} / 3  (Standing, Sitting, Lying correctly verified)")
    print(f"True Negatives (TN):     {tn} / 2  (Cold obstacle & Hot distractor correctly rejected)")
    print(f"False Positives (FP):    {fp} / 2  (Non-human distractors accepted as victims)")
    print(f"False Negatives (FN):    {fn} / 3  (Human victims erroneously rejected)")
    print("-" * 75)
    print(f"PRECISION:               {precision * 100:.1f} %")
    print(f"RECALL:                  {recall * 100:.1f} %")
    print(f"FALSE POSITIVE RATE:     {fpr * 100:.1f} %")
    print(f"OVERALL ACCURACY:        {accuracy * 100:.1f} %")
    print("=" * 75)

    return 0

if __name__ == '__main__':
    sys.exit(main())
