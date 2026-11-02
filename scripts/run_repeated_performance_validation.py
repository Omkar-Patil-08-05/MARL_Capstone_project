#!/usr/bin/env python3
"""
Repeated Bounded Performance Validation Test Harness for MARL Swarm SAR in Gazebo Harmonic.
Executes repeated bounded runs (3 repetitions per configuration) with a fixed seed and
consistent stable measurement window (Decision 1 to Decision 4).

Configurations:
  - Config A: Headless RGB-D @ 5 Hz (Baseline)
  - Config B: 3D GUI Execution + RGB-D @ 5 Hz
  - Config C: Headless RGB-D @ 2 Hz (Optimized Camera Rate)
  - Config D: Headless Pure Physics (No Camera Sensors; Controller Throughput Baseline)
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
import argparse
from typing import Dict, Any, List

# Ensure stdout flushes immediately to background logs
sys.stdout.reconfigure(line_buffering=True)
sys.stderr.reconfigure(line_buffering=True)

BACKEND_URL = "http://127.0.0.1:8000"
WS_URL = "ws://127.0.0.1:8000/ws/telemetry"
WORKSPACE_DIR = "/home/capstone/capstone_project_antigravity"
RESULTS_FILE = os.path.join(WORKSPACE_DIR, "results", "repeated_perf_validation_results.json")
SDF_FILE = os.path.join(WORKSPACE_DIR, "models", "x500_rgbd_down", "model.sdf")

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

def sample_camera_stream(drone_id: int, sample_duration: float = 3.0) -> Dict[str, Any]:
    url = f"{BACKEND_URL}/api/camera/stream?drone_id={drone_id}"
    frames = 0
    t0 = time.time()
    try:
        req = urllib.request.urlopen(url, timeout=4)
        buf = b""
        while (time.time() - t0) < sample_duration:
            chunk = req.read(4096)
            if not chunk:
                break
            buf += chunk
            a = buf.find(b'\xff\xd8')
            b = buf.find(b'\xff\xd9')
            if a != -1 and b != -1 and b > a:
                buf = buf[b+2:]
                frames += 1
        req.close()
    except Exception:
        pass
    elapsed = max(0.1, time.time() - t0)
    return {
        "drone_id": drone_id,
        "frames": frames,
        "fps": round(frames / elapsed, 2),
        "sample_duration_sec": round(elapsed, 2)
    }

def cleanup_environment():
    """Forces termination of simulation processes to guarantee a clean slate between runs."""
    try:
        requests.post(f"{BACKEND_URL}/api/mission/stop", timeout=5)
    except Exception:
        pass
    subprocess.run(
        "pkill -9 -f 'MicroXRCEAgent' || true; pkill -9 -f 'px4' || true; pkill -9 -f 'gz sim' || true; pkill -9 -f 'ruby.*gz' || true; pkill -9 -f 'ros_gz_bridge' || true; pkill -9 -f 'swarm_runner' || true; pkill -9 -f 'swarm_perception_node' || true; pkill -9 -f 'rgbd_victim_localizer' || true",
        shell=True, executable='/bin/bash', capture_output=True
    )
    time.sleep(6)

async def run_single_repetition(config_key: str, rep_index: int, seed: int = 201, max_decisions: int = 4) -> Dict[str, Any]:
    print(f"\n=======================================================")
    print(f"RUNNING {config_key} | Repetition {rep_index}/3 | Seed: {seed}")
    print(f"Stable window: Decision 1 to Decision {max_decisions}")
    print(f"=======================================================")

    is_lightweight = (config_key == "config_d_physics_only")
    is_gui = (config_key == "config_b_gui_5hz")

    if config_key == "config_c_headless_2hz":
        set_camera_update_rate(2)
    else:
        set_camera_update_rate(5)

    cleanup_environment()

    # Pre-prime CPU
    psutil.cpu_percent(interval=None)

    payload = {
        "map_id": "forest_sar",
        "drone_count": 2,
        "victim_count": 3,
        "seed": seed,
        "max_decisions": 6,
        "lightweight": is_lightweight
    }

    t_mission_start = time.time()
    try:
        r = requests.post(f"{BACKEND_URL}/api/mission/start", json=payload, timeout=30)
        if r.status_code != 200:
            print(f"[ERROR] Failed to start mission: {r.text}")
            return None
    except Exception as e:
        print(f"[ERROR] Exception on mission start: {e}")
        return None

    decision_events = []
    hw_samples = []
    last_hw_sample = 0.0
    gui_launched = False

    t_running_start = None

    try:
        async with websockets.connect(WS_URL, open_timeout=45) as ws:
            print("[WS] Connected. Waiting for telemetry and Decision 1...")

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
                    print(f"[STATUS] Simulation entered RUNNING state at wall {now:.1f}s | sim {sim_time:.1f}s", flush=True)

                if is_gui and not gui_launched and (status in ["RUNNING", "QMIX_STARTING", "SIMULATOR_READY"] or decisions > 0):
                    print("[GUI] Triggering 3D Gazebo GUI launch...", flush=True)
                    try:
                        res = requests.post(f"{BACKEND_URL}/api/mission/view_simulation", timeout=10)
                        print(f"[GUI] Launch response: {res.json()}", flush=True)
                        gui_launched = True
                    except Exception as e:
                        print(f"[GUI] Launch error: {e}", flush=True)

                if decisions > 0 and (not decision_events or decisions > decision_events[-1]["decision"]):
                    ev = {
                        "decision": decisions,
                        "wall_time": round(now, 2),
                        "sim_time": round(sim_time, 2),
                        "coverage": round(coverage, 2),
                        "victims_detected": victims,
                        "safety_overrides": overrides,
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
                    print(f"[{now:.1f}s wall | {sim_time:.1f}s sim] *** DECISION {decisions} *** Cov: {coverage:.2f}% | Victims: {victims} | Overrides: {overrides}", flush=True)

                    # Bounded window: 4 decision points = 3 inter-decision cycles
                    if len(decision_events) >= 4 or (decision_events and (decision_events[-1]["decision"] - decision_events[0]["decision"]) >= 3):
                        print(f"[TARGET] Reached target decision window ({len(decision_events)} decisions recorded). Ending measurement window.", flush=True)
                        break

                # Safety timeouts:
                # 1. Startup timeout before RUNNING (650s for Gazebo boot + climb at RTF ~0.045)
                if t_running_start is None and (now - t_mission_start) > 650.0:
                    print("[TIMEOUT] Reached 650s startup timeout before RUNNING state.", flush=True)
                    break
                # 2. In-flight timeout after RUNNING (240s for 4 decisions at low RTF)
                if t_running_start is not None and (now - t_running_start) > 240.0:
                    print("[TIMEOUT] Reached 240s in-flight execution timeout limit.", flush=True)
                    break

            # Camera sampling while still in air
            if not is_lightweight:
                print("[CAM] Sampling live camera streams...")
                cam0 = sample_camera_stream(0, 3.0)
                cam1 = sample_camera_stream(1, 3.0)
                print(f"[CAM] Drone 0: {cam0.get('fps')} FPS | Drone 1: {cam1.get('fps')} FPS")
            else:
                cam0 = {"drone_id": 0, "status": "disabled_physics_only"}
                cam1 = {"drone_id": 1, "status": "disabled_physics_only"}

    finally:
        cleanup_environment()

    # Metric recalculation strictly across stable window (Dec 1 -> Dec 4)
    if len(decision_events) >= 2:
        d_first = decision_events[0]
        d_last = decision_events[-1]
        wall_elapsed = d_last["wall_time"] - d_first["wall_time"]
        sim_elapsed = d_last["sim_time"] - d_first["sim_time"]
        measured_rtf = round(sim_elapsed / max(0.001, wall_elapsed), 4)
        dec_count = d_last["decision"] - d_first["decision"]
        dec_per_min = round((dec_count / max(0.001, wall_elapsed)) * 60.0, 2)
        cov_gain = round(d_last["coverage"] - d_first["coverage"], 2)
        cov_per_min = round((cov_gain / max(0.001, wall_elapsed)) * 60.0, 2)
        valid_cells_gain = int(round((cov_gain / 100.0) * 420))
        cells_per_min = round((valid_cells_gain / max(0.001, wall_elapsed)) * 60.0, 2)
        total_overrides = d_last["safety_overrides"]
        total_victims = d_last["victims_detected"]
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

    # Hardware stats over measurement window
    if hw_samples and len(decision_events) >= 2:
        t0 = decision_events[0]["wall_time"]
        t1 = decision_events[-1]["wall_time"]
        window_hw = [s["metrics"] for s in hw_samples if t0 <= s["wall_time"] <= t1] or [s["metrics"] for s in hw_samples]
    else:
        window_hw = [s["metrics"] for s in hw_samples] if hw_samples else []

    if window_hw:
        sys_cpu = [h["system_cpu_percent"] for h in window_hw]
        gz_cpu = [h["gz_cpu_percent"] for h in window_hw]
        px4_cpu = [h["px4_cpu_percent"] for h in window_hw]
        gpu_u = [h["gpu_util_percent"] for h in window_hw]
        vram = [h["vram_used_mb"] for h in window_hw]
        hw_summary = {
            "system_cpu_mean": round(sum(sys_cpu) / len(sys_cpu), 1),
            "system_cpu_max": max(sys_cpu),
            "gz_cpu_mean": round(sum(gz_cpu) / len(gz_cpu), 1),
            "gz_cpu_max": max(gz_cpu),
            "px4_cpu_mean": round(sum(px4_cpu) / len(px4_cpu), 1),
            "px4_cpu_max": max(px4_cpu),
            "gpu_util_mean": round(sum(gpu_u) / len(gpu_u), 1),
            "vram_mb_mean": round(sum(vram) / len(vram), 1)
        }
    else:
        hw_summary = {}

    rep_record = {
        "config_key": config_key,
        "rep_index": rep_index,
        "seed": seed,
        "wall_elapsed_sec": round(wall_elapsed, 2),
        "sim_elapsed_sec": round(sim_elapsed, 2),
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
        "cameras": [cam0, cam1],
        "hardware_summary": hw_summary,
        "decision_events": decision_events
    }
    return rep_record

def compute_stats(values: List[float]) -> Dict[str, float]:
    if not values:
        return {"median": 0.0, "min": 0.0, "max": 0.0}
    s = sorted(values)
    n = len(s)
    mid = n // 2
    median = s[mid] if n % 2 != 0 else round((s[mid - 1] + s[mid]) / 2.0, 4)
    return {
        "median": median,
        "min": min(s),
        "max": max(s)
    }

async def run_benchmark_suite(configs: List[str], repetitions: int = 3, seed: int = 201):
    os.makedirs(os.path.dirname(RESULTS_FILE), exist_ok=True)
    all_results = {}
    if os.path.exists(RESULTS_FILE):
        try:
            with open(RESULTS_FILE, "r") as f:
                all_results = json.load(f)
        except Exception:
            all_results = {}

    total_runs = len(configs) * repetitions
    est_sec = total_runs * 85
    print("\n=======================================================")
    print(f"BENCHMARK SUITE INITIATED")
    print(f"Configurations to run: {configs}")
    print(f"Repetitions per config: {repetitions}")
    print(f"Total test runs: {total_runs}")
    print(f"Fixed World Seed: {seed}")
    print(f"Estimated Total Duration: ~{est_sec // 60}m {est_sec % 60}s")
    print(f"=======================================================\n")

    try:
        for ckey in configs:
            if ckey not in all_results:
                all_results[ckey] = []

            for rep in range(1, repetitions + 1):
                # Check if rep already recorded with successful window
                existing = [r for r in all_results[ckey] if r.get("rep_index") == rep and r.get("seed") == seed and r.get("dec_count", 0) >= 2]
                if existing:
                    print(f"[SKIP] {ckey} Rep {rep} already completed with seed {seed}.")
                    continue
                # Purge any partial/failed attempt
                all_results[ckey] = [r for r in all_results[ckey] if not (r.get("rep_index") == rep and r.get("seed") == seed)]

                record = await run_single_repetition(ckey, rep, seed=seed, max_decisions=4)
                if record:
                    all_results[ckey].append(record)
                    with open(RESULTS_FILE, "w") as f:
                        json.dump(all_results, f, indent=2)
    finally:
        set_camera_update_rate(5)
        cleanup_environment()

    # Print summary table
    print("\n\n=========================================================================================")
    print("REPEATED BENCHMARK SUMMARY (MEDIAN [MIN - MAX]) ACROSS 3 REPETITIONS (SEED 201)")
    print("=========================================================================================")
    print(f"{'Configuration':<26} | {'RTF':<18} | {'Dec/Min':<16} | {'Cells/Min':<18} | {'Cov Gain %':<12}")
    print("-" * 97)

    for ckey in configs:
        reps = all_results.get(ckey, [])
        if not reps:
            continue
        rtfs = [r["measured_rtf"] for r in reps]
        decs = [r["decisions_per_wall_min"] for r in reps]
        cells = [r["valid_cells_per_wall_min"] for r in reps]
        covs = [r["coverage_gain"] for r in reps]

        s_rtf = compute_stats(rtfs)
        s_dec = compute_stats(decs)
        s_cell = compute_stats(cells)
        s_cov = compute_stats(covs)

        rtf_str = f"{s_rtf['median']:.4f} [{s_rtf['min']:.4f}-{s_rtf['max']:.4f}]"
        dec_str = f"{s_dec['median']:.2f} [{s_dec['min']:.2f}-{s_dec['max']:.2f}]"
        cell_str = f"{s_cell['median']:.2f} [{s_cell['min']:.2f}-{s_cell['max']:.2f}]"
        cov_str = f"{s_cov['median']:.2f}% [{s_cov['min']:.2f}-{s_cov['max']:.2f}%]"

        print(f"{ckey:<26} | {rtf_str:<18} | {dec_str:<16} | {cell_str:<18} | {cov_str:<12}")
    print("=========================================================================================\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--configs", nargs="+", default=["config_a_headless_5hz", "config_b_gui_5hz", "config_c_headless_2hz", "config_d_physics_only"],
                        help="Configurations to benchmark")
    parser.add_argument("--reps", type=int, default=3, help="Number of repetitions per config")
    parser.add_argument("--seed", type=int, default=201, help="Fixed random seed for forest world")
    args = parser.parse_args()

    asyncio.run(run_benchmark_suite(args.configs, args.reps, args.seed))
