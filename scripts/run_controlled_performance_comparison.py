#!/usr/bin/env python3
"""
Controlled Performance Comparison Test Harness for MARL Swarm SAR in Gazebo Harmonic.
Measures RTF, decisions/min, coverage/min, state transitions, hardware usage, and camera stability.
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

BACKEND_URL = "http://127.0.0.1:8000"
WS_URL = "ws://127.0.0.1:8000/ws/telemetry"

def get_hardware_usage():
    """Queries CPU and GPU utilization."""
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
        
    # Check gz sim and px4 cpu
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

def sample_camera_stream(drone_id: int, sample_duration: float = 3.0):
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

async def run_controlled_test(config_label: str, max_decisions: int = 5, max_wall_sec: float = 160.0):
    print(f"\n=======================================================")
    print(f"STARTING CONTROLLED TEST: {config_label}")
    print(f"Target decisions: {max_decisions} | Max wall time: {max_wall_sec}s")
    print(f"=======================================================")

    # 1. Health check
    try:
        r = requests.get(f"{BACKEND_URL}/api/health", timeout=3)
        print(f"Backend status: {r.json()}")
    except Exception as e:
        print(f"Failed to connect to backend: {e}")
        return None

    # 2. Start mission
    is_lightweight = ("physics" in config_label.lower() or "config_d" in config_label.lower())
    payload = {
        "map_id": "forest_sar",
        "drone_count": 2,
        "victim_count": 3,
        "lightweight": is_lightweight
    }
    r = requests.post(f"{BACKEND_URL}/api/mission/start", json=payload, timeout=30)
    if r.status_code != 200:
        print(f"Failed to start mission: {r.text}")
        return None
    print(f"Mission start initiated (code {r.status_code})")

    running_start_wall = None
    running_start_sim = None
    decision_events = []
    state_transitions = []
    hw_samples = []
    last_hw_sample = 0
    
    prev_decisions = 0
    prev_global_state = None
    navigating_wall_sec = 0.0
    synchronizing_wall_sec = 0.0
    last_tick_wall = time.time()
    
    # Pre-prime CPU percent measurement
    psutil.cpu_percent(interval=None)

    try:
        async with websockets.connect(WS_URL, open_timeout=45) as ws:
            print("Connected to WebSocket telemetry. Waiting for RUNNING state...")
            
            while True:
                now = time.time()
                dt = now - last_tick_wall
                last_tick_wall = now
                
                if prev_global_state == "NAVIGATING":
                    navigating_wall_sec += dt
                elif prev_global_state == "SYNCHRONIZING":
                    synchronizing_wall_sec += dt

                if (now - last_hw_sample) >= 8.0:
                    last_hw_sample = now
                    hw_samples.append({
                        "wall_time": round(now, 1),
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
                global_state = mission.get("global_step_state", "UNKNOWN")
                drones = msg.get("drones", [])

                if status == "RUNNING" and running_start_wall is None:
                    running_start_wall = now
                    running_start_sim = sim_time
                    print(f">>> RUNNING STATE DETECTED at wall {now:.1f}s | sim {sim_time:.1f}s <<<")
                    if "gui" in config_label.lower():
                        print("Launching Gazebo GUI attached to running simulation (Configuration B)...")
                        try:
                            res = requests.post(f"{BACKEND_URL}/api/mission/view_simulation", timeout=10)
                            print(f"GUI launch response: {res.json()}")
                        except Exception as e:
                            print(f"Error launching GUI: {e}")

                if global_state != prev_global_state:
                    state_transitions.append({
                        "wall_time": round(now, 2),
                        "sim_time": round(sim_time, 2),
                        "decision": decisions,
                        "from_state": prev_global_state,
                        "to_state": global_state
                    })
                    print(f"[{now:.1f}s wall | {sim_time:.1f}s sim] STATE: {prev_global_state} -> {global_state} (Dec: {decisions}, Cov: {coverage:.2f}%)")
                    prev_global_state = global_state

                if decisions > prev_decisions:
                    decision_events.append({
                        "decision": decisions,
                        "wall_time": round(now, 2),
                        "sim_time": round(sim_time, 2),
                        "coverage": round(coverage, 2),
                        "drones": [{
                            "id": d.get("id"),
                            "x": round(d.get("x", 0.0), 2),
                            "y": round(d.get("y", 0.0), 2),
                            "z": round(d.get("z", 0.0), 2),
                            "vx": round(d.get("vx", 0.0), 2),
                            "vy": round(d.get("vy", 0.0), 2),
                            "state": d.get("state"),
                            "action": d.get("action")
                        } for d in drones]
                    })
                    print(f"[{now:.1f}s wall | {sim_time:.1f}s sim] *** DECISION {decisions} *** Cov: {coverage:.2f}%")
                    prev_decisions = decisions

                if running_start_wall is not None:
                    elapsed_wall = now - running_start_wall
                    if decisions >= max_decisions:
                        print(f"\nTarget decision count ({max_decisions}) reached in {elapsed_wall:.1f}s wall time.")
                        break
                    if elapsed_wall >= max_wall_sec:
                        print(f"\nMax wall time ({max_wall_sec}s) reached.")
                        break

            # Sample camera streams while still running
            if not is_lightweight:
                print("Sampling camera streams...")
                cam0 = sample_camera_stream(0, 3.0)
                cam1 = sample_camera_stream(1, 3.0)
                print(f"Camera 0: {cam0}")
                print(f"Camera 1: {cam1}")
            else:
                print("Cameras disabled for performance-only configuration.")
                cam0 = {"drone_id": 0, "status": "disabled_performance_baseline"}
                cam1 = {"drone_id": 1, "status": "disabled_performance_baseline"}
    finally:
        print("\nStopping mission...")
        try:
            requests.post(f"{BACKEND_URL}/api/mission/stop", timeout=10)
        except Exception:
            pass

    # Compute aggregate metrics over running window
    if len(decision_events) >= 2:
        first_d = decision_events[0]
        last_d = decision_events[-1]
        sim_elapsed = last_d["sim_time"] - first_d["sim_time"]
        wall_elapsed = last_d["wall_time"] - first_d["wall_time"]
        measured_rtf = round(sim_elapsed / max(0.001, wall_elapsed), 4)
        dec_count = last_d["decision"] - first_d["decision"]
        dec_per_min = round((dec_count / max(0.001, wall_elapsed)) * 60.0, 2)
        cov_gain = round(last_d["coverage"] - first_d["coverage"], 2)
        cov_per_min = round((cov_gain / max(0.001, wall_elapsed)) * 60.0, 2)
        valid_cells_gain = int(round((cov_gain / 100.0) * 420))
        cells_per_min = round((valid_cells_gain / max(0.001, wall_elapsed)) * 60.0, 2)
    else:
        sim_elapsed = 0.0
        wall_elapsed = 0.0
        measured_rtf = 0.0
        dec_per_min = 0.0
        cov_gain = 0.0
        cov_per_min = 0.0
        valid_cells_gain = 0
        cells_per_min = 0.0

    report = {
        "config_label": config_label,
        "timestamp": time.time(),
        "wall_elapsed_sec": round(wall_elapsed, 2),
        "sim_elapsed_sec": round(sim_elapsed, 2),
        "measured_rtf": measured_rtf,
        "decisions_completed": len(decision_events),
        "initial_decision": decision_events[0]["decision"] if decision_events else 0,
        "final_decision": decision_events[-1]["decision"] if decision_events else 0,
        "decisions_per_wall_min": dec_per_min,
        "coverage_start": decision_events[0]["coverage"] if decision_events else 0.0,
        "coverage_end": decision_events[-1]["coverage"] if decision_events else 0.0,
        "coverage_gain": cov_gain,
        "coverage_gain_per_wall_min": cov_per_min,
        "valid_cells_explored_gain": valid_cells_gain,
        "valid_cells_per_wall_min": cells_per_min,
        "navigating_wall_sec": round(navigating_wall_sec, 2),
        "synchronizing_wall_sec": round(synchronizing_wall_sec, 2),
        "decision_history": decision_events,
        "state_transitions": state_transitions,
        "cameras": [cam0, cam1],
        "hardware_usage": hw_samples
    }

    out_file = f"/home/capstone/capstone_project_antigravity/results/controlled_perf_{config_label.lower()}.json"
    os.makedirs(os.path.dirname(out_file), exist_ok=True)
    with open(out_file, "w") as f:
        json.dump(report, f, indent=2)
    print(f"\nReport written to: {out_file}")
    print(f"Summary for {config_label}:")
    print(f"  RTF: {measured_rtf}")
    print(f"  Decisions/min: {dec_per_min}")
    print(f"  Valid cells/min: {cells_per_min}")
    print(f"  Coverage gain: {cov_gain}%")
    print(f"  Navigating time: {navigating_wall_sec:.1f}s | Synchronizing time: {synchronizing_wall_sec:.1f}s")
    return report

if __name__ == "__main__":
    label = sys.argv[1] if len(sys.argv) > 1 else "TEST"
    max_dec = int(sys.argv[2]) if len(sys.argv) > 2 else 5
    max_wall = float(sys.argv[3]) if len(sys.argv) > 3 else 160.0
    asyncio.run(run_controlled_test(label, max_dec, max_wall))
