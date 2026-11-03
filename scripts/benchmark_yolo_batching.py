#!/usr/bin/env python3
"""
YOLO Multi-Camera Batched Inference Benchmarking Suite (Phase 2 Remediation - Stage 2)

Measures inference latency, sustained throughput (FPS), GPU memory (VRAM),
and latency percentiles (P95, P99) across batch sizes B in {1, 2, 4, 6}
on NVIDIA RTX 3050 Laptop GPU (6 GB VRAM) using the project's yolov8n.pt model.
"""

import os
import sys
import time
import numpy as np
import torch

try:
    from ultralytics import YOLO
except ImportError:
    print("ERROR: ultralytics is required. Run with ~/yolo_venv/bin/python3")
    sys.exit(1)


def benchmark_batch_size(
    model_path: str,
    batch_size: int,
    img_shape: tuple = (480, 640, 3),
    warmup_iters: int = 30,
    bench_iters: int = 100,
    device: str = "cuda:0"
):
    print(f"\n>>> Benchmarking Batch Size B={batch_size} on {device} ({bench_iters} iterations)...")

    # Clear GPU cache before test
    if torch.cuda.is_available():
        torch.cuda.empty_cache()
        torch.cuda.reset_peak_memory_stats()

    model = YOLO(model_path)
    model.to(device)

    # Generate synthetic camera images [B, H, W, C]
    # Representing raw 640x480 BGR frames from Gazebo
    dummy_images = [
        np.random.randint(0, 256, img_shape, dtype=np.uint8)
        for _ in range(batch_size)
    ]

    # Warm-up phase
    for _ in range(warmup_iters):
        _ = model(dummy_images, verbose=False)
        if torch.cuda.is_available():
            torch.cuda.synchronize()

    # Benchmark phase
    latencies_ms = []
    t_start_total = time.perf_counter()

    for _ in range(bench_iters):
        t0 = time.perf_counter()
        _ = model(dummy_images, verbose=False)
        if torch.cuda.is_available():
            torch.cuda.synchronize()
        t1 = time.perf_counter()
        latencies_ms.append((t1 - t0) * 1000.0)

    t_end_total = time.perf_counter()
    total_time_sec = t_end_total - t_start_total

    # Compute metrics
    total_frames = batch_size * bench_iters
    total_fps = total_frames / total_time_sec
    per_drone_fps = total_fps / 6.0  # Normalized to 6 drones in the swarm

    mean_lat = float(np.mean(latencies_ms))
    std_lat = float(np.std(latencies_ms))
    p50_lat = float(np.percentile(latencies_ms, 50))
    p95_lat = float(np.percentile(latencies_ms, 95))
    p99_lat = float(np.percentile(latencies_ms, 99))

    vram_alloc_mb = torch.cuda.memory_allocated() / (1024 * 1024) if torch.cuda.is_available() else 0.0
    vram_peak_mb = torch.cuda.max_memory_allocated() / (1024 * 1024) if torch.cuda.is_available() else 0.0

    return {
        "batch_size": batch_size,
        "total_fps": total_fps,
        "per_drone_fps": per_drone_fps,
        "mean_latency_ms": mean_lat,
        "std_latency_ms": std_lat,
        "p50_latency_ms": p50_lat,
        "p95_latency_ms": p95_lat,
        "p99_latency_ms": p99_lat,
        "vram_alloc_mb": vram_alloc_mb,
        "vram_peak_mb": vram_peak_mb
    }


def main():
    print("=" * 80)
    print("PHASE 2 REMEDIATION: YOLO MULTI-CAMERA BATCHING BENCHMARK")
    print("=" * 80)

    model_path = "/home/capstone/capstone_project_antigravity/models/yolov8n.pt"
    if not os.path.exists(model_path):
        print(f"ERROR: Model not found at {model_path}")
        return 1

    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    gpu_name = torch.cuda.get_device_name(0) if torch.cuda.is_available() else "CPU"
    total_vram_gb = torch.cuda.get_device_properties(0).total_memory / (1024**3) if torch.cuda.is_available() else 0.0

    print(f"Device:           {device} ({gpu_name})")
    print(f"Available VRAM:   {total_vram_gb:.2f} GB")
    print(f"Model Path:       {model_path}")
    print(f"Camera Frame:     640x480 BGR")

    batch_sizes = [1, 2, 4, 6]
    results = []

    for b in batch_sizes:
        res = benchmark_batch_size(
            model_path=model_path,
            batch_size=b,
            img_shape=(480, 640, 3),
            warmup_iters=30,
            bench_iters=100,
            device=device
        )
        results.append(res)

    print("\n" + "=" * 80)
    print("EMPIRICAL BENCHMARK RESULTS SUMMARY (RTX 3050 6GB LAPTOP GPU)")
    print("=" * 80)
    header = (
        f"{'Batch':<6} | {'Total FPS':<10} | {'FPS/Drone':<10} | "
        f"{'Mean Lat':<10} | {'P95 Lat':<10} | {'P99 Lat':<10} | {'Peak VRAM':<10}"
    )
    print(header)
    print("-" * 80)

    for r in results:
        row = (
            f"B={r['batch_size']:<4} | "
            f"{r['total_fps']:<10.1f} | "
            f"{r['per_drone_fps']:<10.2f} | "
            f"{r['mean_latency_ms']:<8.2f}ms | "
            f"{r['p95_latency_ms']:<8.2f}ms | "
            f"{r['p99_latency_ms']:<8.2f}ms | "
            f"{r['vram_peak_mb']:<7.1f}MB"
        )
        print(row)
    print("=" * 80)

    # Operational evaluation against 5 FPS requirement
    b6 = next((r for r in results if r["batch_size"] == 6), None)
    if b6:
        print("\n[OPERATIONAL ASSESSMENT FOR 6-DRONE SWARM]")
        print(f"Full Swarm Batch (B=6) Sustainable Total Throughput: {b6['total_fps']:.1f} FPS")
        print(f"Sustained Throughput Per Drone:                    {b6['per_drone_fps']:.2f} FPS")
        print(f"Average Batch Latency:                              {b6['mean_latency_ms']:.2f} ms")
        print(f"95th-Percentile Latency:                            {b6['p95_latency_ms']:.2f} ms")
        print(f"Peak VRAM Consumption:                              {b6['vram_peak_mb']:.1f} MB ({b6['vram_peak_mb']/1024:.2f} GB / {total_vram_gb:.2f} GB)")

        if b6['per_drone_fps'] >= 5.0:
            print("\n>>> TARGET VERIFIED: B=6 achieves >= 5.0 FPS per drone on RTX 3050!")
        else:
            print(f"\n>>> WARNING: B=6 achieves {b6['per_drone_fps']:.2f} FPS per drone (< 5.0 FPS target).")

    return 0


if __name__ == '__main__':
    sys.exit(main())
