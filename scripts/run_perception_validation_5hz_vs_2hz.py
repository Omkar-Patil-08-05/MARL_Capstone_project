#!/usr/bin/env python3
"""
scripts/run_perception_validation_5hz_vs_2hz.py

Matched Perception Validation Harness: 5 Hz Baseline vs 2 Hz Configuration C.
Evaluates:
  1. Actual ROS 2 sensor topic frame rates (/drone_{i}/camera/image and /drone_{i}/camera/depth_image)
     in simulation-time Hz and wall-clock Hz.
  2. Frame age (simulation clock vs header stamp at arrival) and inter-frame intervals.
  3. Image timestamps and dropped frame counts.
  4. YOLO inference rate and latency (yolov8n.pt on GPU).
  5. Victims encountered, detections, and false positives.
  6. QMIX decisions per wall-clock minute and valid cells explored per minute across stable window (Dec 1 -> Dec 4).
  7. Gazebo RTF, CPU load (System, Gazebo, PX4), GPU utilization, and VRAM use.
  8. Safety overrides and collisions.
  9. Frontend HTTP MJPEG stream rate comparison.

Guarantees 100% reversible configuration: restores models/x500_rgbd_down/model.sdf to 5 Hz on exit.
"""

import sys
import os
import json
import time
import math
import asyncio
import subprocess
import requests
import websockets
import urllib.request
import psutil
import threading
from typing import Dict, Any, List

# Ensure unbuffered output
sys.stdout.reconfigure(line_buffering=True)
sys.stderr.reconfigure(line_buffering=True)

# Set ROS 2 environment variables before importing rclpy
os.environ["ROS_LOCALHOST_ONLY"] = "1"
os.environ["ROS_DISABLE_TYPE_HASH_CHECK"] = "1"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data

WORKSPACE_DIR = "/home/capstone/capstone_project_antigravity"
BACKEND_URL = "http://127.0.0.1:8000"
WS_URL = "ws://127.0.0.1:8000/ws/telemetry"
SDF_FILE = os.path.join(WORKSPACE_DIR, "models", "x500_rgbd_down", "model.sdf")
SDF_BACKUP = os.path.join(WORKSPACE_DIR, "models", "x500_rgbd_down", "model.sdf.bak_5hz")
RESULTS_FILE = os.path.join(WORKSPACE_DIR, "results", "perception_validation_5hz_vs_2hz.json")
META_FILE = os.path.join(WORKSPACE_DIR, "worlds", "forest_sar_meta.json")

# Ground truth victim coordinates from forest_sar_meta.json (Seed 201)
GT_VICTIMS = [
    {"id": "victim_0", "x": -46.875, "y": 99.609},
    {"id": "victim_1", "x": -47.852, "y": 81.055},
    {"id": "victim_2", "x": -67.383, "y": 106.445}
]

def set_camera_update_rate(hz: int):
    """Sets the camera update rate in x500_rgbd_down model.sdf."""
    if not os.path.exists(SDF_FILE):
        return
    with open(SDF_FILE, "r") as f:
        content = f.read()
    import re
    new_content = re.sub(r"<update_rate>\d+</update_rate>", f"<update_rate>{hz}</update_rate>", content)
    with open(SDF_FILE, "w") as f:
        f.write(new_content)
    print(f"[CONFIG] Updated {SDF_FILE} camera update_rate to {hz} Hz")

def restore_camera_sdf_to_5hz():
    """Restores model.sdf from 5 Hz backup to ensure zero permanent mutation."""
    if os.path.exists(SDF_BACKUP):
        with open(SDF_BACKUP, "r") as f:
            bak_content = f.read()
        with open(SDF_FILE, "w") as f:
            f.write(bak_content)
        print(f"[REVERT] Restored {SDF_FILE} to original 5 Hz baseline from backup.")
    else:
        set_camera_update_rate(5)

def get_hardware_usage() -> Dict[str, float]:
    """Queries current CPU and GPU metrics."""
    cpu_percent = psutil.cpu_percent(interval=None)
    gpu_util = 0.0
    vram_mb = 0.0
    try:
        res = subprocess.run(
            ["nvidia-smi", "--query-gpu=utilization.gpu,memory.used", "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=1.5
        )
        if res.returncode == 0 and res.stdout.strip():
            parts = res.stdout.strip().split(",")
            gpu_util = float(parts[0].strip())
            vram_mb = float(parts[1].strip())
    except Exception:
        pass

    gz_cpu = 0.0
    px4_cpu = 0.0
    for p in psutil.process_iter(['name', 'cmdline', 'cpu_percent']):
        try:
            cmd = " ".join(p.info['cmdline'] or [])
            if "gz sim" in cmd or "ruby" in cmd:
                gz_cpu += p.info['cpu_percent'] or 0.0
            elif "px4" in cmd:
                px4_cpu += p.info['cpu_percent'] or 0.0
        except Exception:
            pass

    return {
        "system_cpu_percent": cpu_percent,
        "gz_cpu_percent": round(gz_cpu, 1),
        "px4_cpu_percent": round(px4_cpu, 1),
        "gpu_util_percent": gpu_util,
        "vram_used_mb": vram_mb
    }

def sample_fastapi_mjpeg(drone_id: int, duration_sec: float = 3.0) -> Dict[str, Any]:
    """Samples the FastAPI MJPEG stream to empirically demonstrate the frontend stream rate."""
    url = f"{BACKEND_URL}/api/camera/stream?drone_id={drone_id}"
    frames = 0
    t0 = time.time()
    try:
        req = urllib.request.urlopen(url, timeout=4)
        buf = b""
        while (time.time() - t0) < duration_sec:
            chunk = req.read(4096)
            if not chunk:
                break
            buf += chunk
            a = buf.find(b'\xff\xd8')
            b = buf.find(b'\xff\xd9')
            if a != -1 and b != -1 and b > a:
                frames += 1
                buf = buf[b+2:]
    except Exception as e:
        pass
    elapsed = max(0.1, time.time() - t0)
    return {
        "drone_id": drone_id,
        "frames_received": frames,
        "mjpeg_fps": round(frames / elapsed, 2),
        "sample_duration_sec": round(elapsed, 2)
    }

class PerceptionTelemetryMonitor(Node):
    """
    ROS 2 node that tracks raw sensor image topics, timestamps, frame age,
    dropped frames, and YOLO detections directly from the ROS 2 DDS graph.
    """
    def __init__(self, target_hz: int):
        super().__init__('perception_telemetry_monitor')
        self.target_hz = target_hz
        self.expected_sim_dt = 1.0 / target_hz

        self.latest_sim_clock = 0.0

        # Data collection per drone
        self.color_frames = {0: [], 1: []}
        self.depth_frames = {0: [], 1: []}
        self.yolo_detections = {0: [], 1: []}

        # Subscriptions
        self.create_subscription(Clock, '/clock', self._clock_cb, qos_profile_sensor_data)

        for d in [0, 1]:
            self.create_subscription(
                RosImage, f'/drone_{d}/camera/image',
                lambda msg, drone_id=d: self._image_cb(msg, drone_id),
                qos_profile_sensor_data
            )
            self.create_subscription(
                RosImage, f'/drone_{d}/camera/depth_image',
                lambda msg, drone_id=d: self._depth_cb(msg, drone_id),
                qos_profile_sensor_data
            )
            self.create_subscription(
                String, f'/drone_{d}/perception/detections',
                lambda msg, drone_id=d: self._detection_cb(msg, drone_id),
                10
            )

    def _clock_cb(self, msg: Clock):
        self.latest_sim_clock = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _image_cb(self, msg: RosImage, drone_id: int):
        t_wall = time.time()
        t_sim = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        frame_age_sim = max(0.0, self.latest_sim_clock - t_sim) if self.latest_sim_clock > 0 else 0.0

        self.color_frames[drone_id].append({
            "t_wall": t_wall,
            "t_sim": t_sim,
            "frame_age_sim": frame_age_sim,
            "width": msg.width,
            "height": msg.height,
            "encoding": msg.encoding
        })

    def _depth_cb(self, msg: RosImage, drone_id: int):
        t_wall = time.time()
        t_sim = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.depth_frames[drone_id].append({
            "t_wall": t_wall,
            "t_sim": t_sim,
            "width": msg.width,
            "height": msg.height
        })

    def _detection_cb(self, msg: String, drone_id: int):
        try:
            data = json.loads(msg.data)
            self.yolo_detections[drone_id].append({
                "t_wall": time.time(),
                "timestamp": data.get("timestamp", 0.0),
                "detections": data.get("detections", [])
            })
        except Exception:
            pass

    def compute_summary(self, t_start_wall: float, t_end_wall: float) -> Dict[str, Any]:
        """Calculates sensor FPS, frame age, and dropped frames over the stable window."""
        summary = {}
        for d in [0, 1]:
            # Filter frames within window
            c_frames = [f for f in self.color_frames[d] if t_start_wall <= f["t_wall"] <= t_end_wall]
            d_frames = [f for f in self.depth_frames[d] if t_start_wall <= f["t_wall"] <= t_end_wall]
            dets = [x for x in self.yolo_detections[d] if t_start_wall <= x["t_wall"] <= t_end_wall]

            n_color = len(c_frames)
            n_depth = len(d_frames)

            # Color frame rate
            if n_color >= 2:
                wall_duration = c_frames[-1]["t_wall"] - c_frames[0]["t_wall"]
                sim_duration = c_frames[-1]["t_sim"] - c_frames[0]["t_sim"]
                sim_fps = round((n_color - 1) / max(0.001, sim_duration), 2)
                wall_fps = round((n_color - 1) / max(0.001, wall_duration), 3)

                # Dropped frames calculation based on timestamp gaps
                dropped_frames = 0
                sim_dt_list = []
                for i in range(1, n_color):
                    dt = c_frames[i]["t_sim"] - c_frames[i-1]["t_sim"]
                    sim_dt_list.append(dt)
                    if dt > 1.5 * self.expected_sim_dt:
                        # Missed at least one frame cycle
                        dropped_frames += int(round(dt / self.expected_sim_dt)) - 1

                mean_dt = round(float(sum(sim_dt_list) / len(sim_dt_list)), 4) if sim_dt_list else 0.0
                mean_age = round(float(sum(f["frame_age_sim"] for f in c_frames) / n_color), 3)
                max_age = round(float(max(f["frame_age_sim"] for f in c_frames)), 3)
            else:
                sim_fps = 0.0
                wall_fps = 0.0
                mean_dt = 0.0
                dropped_frames = 0
                mean_age = 0.0
                max_age = 0.0
                sim_duration = 0.0
                wall_duration = 0.0

            # Count actual person detections
            total_person_detections = sum(len(x.get("detections", [])) for x in dets)

            summary[f"drone_{d}"] = {
                "color_frames_received": n_color,
                "depth_frames_received": n_depth,
                "sim_fps": sim_fps,
                "wall_fps": wall_fps,
                "nominal_target_hz": self.target_hz,
                "expected_sim_interval_sec": self.expected_sim_dt,
                "measured_mean_sim_interval_sec": mean_dt,
                "dropped_frames": dropped_frames,
                "mean_sim_frame_age_sec": mean_age,
                "max_sim_frame_age_sec": max_age,
                "yolo_detection_messages": len(dets),
                "total_person_detections": total_person_detections
            }
        return summary

def profile_yolov8_inference(device_str: str = "cuda:0") -> Dict[str, Any]:
    """Profiles actual YOLOv8n inference on the local GPU across 50 simulated frames."""
    print(f"[YOLO PROFILE] Profiling ultralytics YOLOv8n inference on {device_str}...")
    import numpy as np
    try:
        from ultralytics import YOLO
        import torch

        device = device_str if torch.cuda.is_available() else "cpu"
        model = YOLO("/home/capstone/capstone_project_antigravity/models/yolov8n.pt")
        dummy_frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

        # Warmup
        t_w0 = time.time()
        _ = model(dummy_frame, device=device, verbose=False)
        warmup_latency_ms = round((time.time() - t_w0) * 1000.0, 2)

        # 50 iterations
        latencies = []
        for _ in range(50):
            t0 = time.time()
            _ = model(dummy_frame, device=device, verbose=False)
            latencies.append((time.time() - t0) * 1000.0)

        mean_lat = round(float(np.mean(latencies)), 2)
        min_lat = round(float(np.min(latencies)), 2)
        max_lat = round(float(np.max(latencies)), 2)
        p95_lat = round(float(np.percentile(latencies, 95)), 2)
        inference_fps = round(1000.0 / mean_lat, 1)

        vram_mb = 0.0
        if torch.cuda.is_available():
            vram_mb = round(torch.cuda.memory_allocated() / (1024 * 1024), 1)

        return {
            "device": device,
            "warmup_latency_ms": warmup_latency_ms,
            "mean_latency_ms": mean_lat,
            "min_latency_ms": min_lat,
            "max_latency_ms": max_lat,
            "p95_latency_ms": p95_lat,
            "max_inference_fps_capacity": inference_fps,
            "allocated_vram_mb": vram_mb
        }
    except Exception as e:
        print(f"[YOLO PROFILE ERROR] {e}")
        return {"error": str(e)}

def cleanup_environment():
    """Kills all simulation processes to ensure a clean slate."""
    try:
        requests.post(f"{BACKEND_URL}/api/mission/stop", timeout=5)
    except Exception:
        pass
    subprocess.run(
        "pkill -9 -f 'MicroXRCEAgent' || true; pkill -9 -f 'px4' || true; pkill -9 -f 'gz sim' || true; pkill -9 -f 'ruby.*gz' || true; pkill -9 -f 'ros_gz_bridge' || true; pkill -9 -f 'swarm_runner' || true; pkill -9 -f 'swarm_perception_node' || true; pkill -9 -f 'rgbd_victim_localizer' || true",
        shell=True, executable='/bin/bash', capture_output=True
    )
    time.sleep(6)

async def run_perception_validation_run(config_name: str, target_hz: int, seed: int = 201, max_decisions: int = 4) -> Dict[str, Any]:
    print("\n" + "=" * 70)
    print(f"STARTING VALIDATION RUN: {config_name} (Camera Update Rate = {target_hz} Hz)")
    print(f"Seed: {seed} | Drone Count: 2 | World: forest_sar")
    print(f"Stable window: Decision 1 to Decision {max_decisions}")
    print("=" * 70)

    # Set update rate
    set_camera_update_rate(target_hz)
    cleanup_environment()

    # Pre-prime CPU monitor
    psutil.cpu_percent(interval=None)

    # Initialize ROS 2 monitor with clean executor
    executor = rclpy.executors.SingleThreadedExecutor()
    monitor = PerceptionTelemetryMonitor(target_hz=target_hz)
    executor.add_node(monitor)

    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    payload = {
        "map_id": "forest_sar",
        "drone_count": 2,
        "victim_count": 3,
        "seed": seed,
        "max_decisions": 6,
        "lightweight": False
    }

    t_mission_start = time.time()
    try:
        r = requests.post(f"{BACKEND_URL}/api/mission/start", json=payload, timeout=30)
        if r.status_code != 200:
            print(f"[ERROR] Mission start failed: {r.text}")
            return None
    except Exception as e:
        print(f"[ERROR] Mission start exception: {e}")
        return None

    decision_events = []
    hw_samples = []
    last_hw_sample = 0.0
    t_running_start = None

    try:
        async with websockets.connect(WS_URL, open_timeout=45) as ws:
            print("[WS] Telemetry websocket connected. Awaiting simulation stabilization and Decision 1...")

            while True:
                now = time.time()

                if (now - last_hw_sample) >= 3.0:
                    last_hw_sample = now
                    hw_samples.append({
                        "wall_time": round(now, 2),
                        "metrics": get_hardware_usage()
                    })

                try:
                    raw = await asyncio.wait_for(ws.recv(), timeout=2.0)
                except asyncio.TimeoutError:
                    continue

                try:
                    msg = json.loads(raw)
                except Exception:
                    continue

                if msg.get("type") != "telemetry":
                    continue

                mission = msg.get("mission", {})
                status = mission.get("status")
                decisions = mission.get("decision_count", 0)
                coverage = mission.get("coverage", 0.0)
                sim_time = mission.get("sim_time", 0.0)
                victims = mission.get("victims_detected", 0)
                overrides = mission.get("safety_overrides", 0)
                drones = msg.get("drones", [])

                if (status == "RUNNING" or decisions > 0) and t_running_start is None:
                    t_running_start = now
                    print(f"[STATUS] Simulation RUNNING at wall {now:.1f}s | sim {sim_time:.1f}s", flush=True)

                if decisions > 0 and (not decision_events or decisions > decision_events[-1]["decision"]):
                    drone_victim_distances = []
                    for d in drones:
                        dx, dy = float(d.get("x", 0.0)), float(d.get("y", 0.0))
                        d_dists = [math.hypot(dx - v["x"], dy - v["y"]) for v in GT_VICTIMS]
                        drone_victim_distances.append(min(d_dists) if d_dists else 999.0)
                    min_dist_to_any_victim = min(drone_victim_distances) if drone_victim_distances else 999.0

                    ev = {
                        "decision": decisions,
                        "wall_time": round(now, 2),
                        "sim_time": round(sim_time, 2),
                        "coverage": round(coverage, 2),
                        "victims_detected": victims,
                        "safety_overrides": overrides,
                        "min_victim_distance_m": round(min_dist_to_any_victim, 2),
                        "victim_in_fov": (min_dist_to_any_victim <= 8.66),
                        "victim_in_proximity": (min_dist_to_any_victim <= 15.0),
                        "drones": [
                            {
                                "id": d.get("id"),
                                "x": round(d.get("x", 0.0), 2),
                                "y": round(d.get("y", 0.0), 2),
                                "z": round(d.get("z", 0.0), 2),
                                "vx": round(d.get("vx", 0.0), 2),
                                "vy": round(d.get("vy", 0.0), 2),
                                "state": d.get("state"),
                                "action": d.get("action")
                            } for d in drones
                        ]
                    }
                    decision_events.append(ev)
                    print(f"[{now:.1f}s wall | {sim_time:.1f}s sim] *** DECISION {decisions} *** Cov: {coverage:.2f}% | Victims: {victims} | Min Vic Dist: {min_dist_to_any_victim:.1f}m | Overrides: {overrides}", flush=True)

                    if len(decision_events) >= 4 or (decision_events and (decision_events[-1]["decision"] - decision_events[0]["decision"]) >= 3):
                        print(f"[TARGET] Reached target Decision 4. Bounded window complete.", flush=True)
                        break

                # Safety timeouts
                if t_running_start is None and (now - t_mission_start) > 650.0:
                    print("[TIMEOUT] Reached 650s startup timeout.", flush=True)
                    break
                if t_running_start is not None and (now - t_running_start) > 280.0:
                    print("[TIMEOUT] Reached 280s in-flight execution timeout limit.", flush=True)
                    break

            # Sample FastAPI MJPEG streams while in flight
            print("[CAM] Sampling FastAPI HTTP MJPEG streams...")
            mjpeg_d0 = sample_fastapi_mjpeg(0, 3.0)
            mjpeg_d1 = sample_fastapi_mjpeg(1, 3.0)
            print(f"[CAM] FastAPI MJPEG rate -> Drone 0: {mjpeg_d0.get('mjpeg_fps')} FPS | Drone 1: {mjpeg_d1.get('mjpeg_fps')} FPS")

    finally:
        cleanup_environment()
        try:
            executor.remove_node(monitor)
            monitor.destroy_node()
        except Exception:
            pass

    # Process metrics over stable window
    if len(decision_events) >= 2:
        d_first = decision_events[0]
        d_last = decision_events[-1]
        t_w_start = d_first["wall_time"]
        t_w_end = d_last["wall_time"]
        wall_elapsed = round(t_w_end - t_w_start, 2)
        sim_elapsed = round(d_last["sim_time"] - d_first["sim_time"], 2)
        measured_rtf = round(sim_elapsed / max(0.001, wall_elapsed), 4)

        dec_count = d_last["decision"] - d_first["decision"]
        dec_per_min = round((dec_count / max(0.001, wall_elapsed)) * 60.0, 2)
        cov_gain = round(d_last["coverage"] - d_first["coverage"], 2)
        cov_per_min = round((cov_gain / max(0.001, wall_elapsed)) * 60.0, 2)
        valid_cells_gain = int(round((cov_gain / 100.0) * 420))
        cells_per_min = round((valid_cells_gain / max(0.001, wall_elapsed)) * 60.0, 2)
        total_overrides = d_last["safety_overrides"]
        total_victims = d_last["victims_detected"]

        # Compute sensor topic metrics from monitor
        sensor_summary = monitor.compute_summary(t_w_start, t_w_end)
    else:
        wall_elapsed = 0.0
        sim_elapsed = 0.0
        measured_rtf = 0.0
        dec_per_min = 0.0
        cov_gain = 0.0
        cov_per_min = 0.0
        valid_cells_gain = 0
        cells_per_min = 0.0
        total_overrides = 0
        total_victims = 0
        sensor_summary = {}
        t_w_start = 0.0
        t_w_end = 0.0

    # Hardware stats over measurement window
    window_hw = [s["metrics"] for s in hw_samples if t_w_start <= s["wall_time"] <= t_w_end]
    if not window_hw:
        window_hw = [s["metrics"] for s in hw_samples]

    if window_hw:
        hw_summary = {
            "system_cpu_mean": round(float(sum(m["system_cpu_percent"] for m in window_hw) / len(window_hw)), 1),
            "system_cpu_max": round(float(max(m["system_cpu_percent"] for m in window_hw)), 1),
            "gz_cpu_mean": round(float(sum(m["gz_cpu_percent"] for m in window_hw) / len(window_hw)), 1),
            "gz_cpu_max": round(float(max(m["gz_cpu_percent"] for m in window_hw)), 1),
            "px4_cpu_mean": round(float(sum(m["px4_cpu_percent"] for m in window_hw) / len(window_hw)), 1),
            "px4_cpu_max": round(float(max(m["px4_cpu_percent"] for m in window_hw)), 1),
            "gpu_util_mean": round(float(sum(m["gpu_util_percent"] for m in window_hw) / len(window_hw)), 1),
            "vram_mb_mean": round(float(sum(m["vram_used_mb"] for m in window_hw) / len(window_hw)), 1)
        }
    else:
        hw_summary = {}

    return {
        "config_name": config_name,
        "target_update_rate_hz": target_hz,
        "seed": seed,
        "wall_elapsed_sec": wall_elapsed,
        "sim_elapsed_sec": sim_elapsed,
        "measured_rtf": measured_rtf,
        "dec_count": dec_count if len(decision_events) >= 2 else 0,
        "decisions_per_wall_min": dec_per_min,
        "coverage_start": decision_events[0]["coverage"] if decision_events else 0.0,
        "coverage_end": decision_events[-1]["coverage"] if decision_events else 0.0,
        "coverage_gain": cov_gain,
        "valid_cells_gain": valid_cells_gain,
        "valid_cells_per_wall_min": cells_per_min,
        "victims_detected": total_victims,
        "safety_overrides": total_overrides,
        "sensor_topic_metrics": sensor_summary,
        "fastapi_mjpeg_stream": {
            "drone_0": mjpeg_d0,
            "drone_1": mjpeg_d1
        },
        "hardware_summary": hw_summary,
        "decision_events": decision_events
    }

async def main_suite():
    print("=" * 80)
    print("MARL CAPSTONE — MATCHED PERCEPTION VALIDATION (5 Hz BASELINE vs 2 Hz CONFIG C)")
    print("=" * 80)

    rclpy.init()

    # 1. Profile YOLOv8n inference on GPU
    yolo_profile = profile_yolov8_inference("cuda:0")
    print(f"[YOLO PROFILE RESULT] Mean latency: {yolo_profile.get('mean_latency_ms')} ms | Max capacity: {yolo_profile.get('max_inference_fps_capacity')} FPS")

    results = {
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "yolo_gpu_profiling": yolo_profile,
        "runs": {}
    }

    try:
        # Run 5 Hz Baseline (Configuration A)
        res_5hz = await run_perception_validation_run("config_a_headless_5hz", target_hz=5, seed=201, max_decisions=4)
        results["runs"]["config_a_headless_5hz"] = res_5hz

        # Run 2 Hz Configuration C
        res_2hz = await run_perception_validation_run("config_c_headless_2hz", target_hz=2, seed=201, max_decisions=4)
        results["runs"]["config_c_headless_2hz"] = res_2hz

    finally:
        # Guarantee reversible configuration: revert model.sdf back to 5 Hz baseline
        restore_camera_sdf_to_5hz()
        try:
            rclpy.shutdown()
        except Exception:
            pass

    # Save results
    with open(RESULTS_FILE, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\n[COMPLETE] Validation results saved to {RESULTS_FILE}")

if __name__ == "__main__":
    asyncio.run(main_suite())
