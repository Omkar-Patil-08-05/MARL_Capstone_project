#!/usr/bin/env python3
import asyncio
import json
import time
import math
import requests
import websockets
import urllib.request
import subprocess
import os

BACKEND_URL = "http://127.0.0.1:8000"
WS_URL = "ws://127.0.0.1:8000/ws/telemetry"

def get_gz_stats():
    try:
        res = subprocess.run(
            ["gz", "topic", "-e", "-t", "/world/forest_sar/stats", "-n", "1", "--json-output"],
            capture_output=True, text=True, timeout=2.5
        )
        if res.returncode == 0 and res.stdout.strip():
            d = json.loads(res.stdout)
            return float(d.get("real_time_factor", 0.0)), d.get("sim_time", {}).get("sec", 0)
    except Exception:
        pass
    return None, None

def sample_camera(drone_id: int, sample_duration: float = 3.0):
    url = f"{BACKEND_URL}/api/camera/stream?drone_id={drone_id}"
    frames = 0
    t0 = time.time()
    try:
        req = urllib.request.urlopen(url, timeout=4)
        buffer = b""
        while (time.time() - t0) < sample_duration:
            chunk = req.read(4096)
            if not chunk:
                break
            buffer += chunk
            a = buffer.find(b'\xff\xd8')
            b = buffer.find(b'\xff\xd9')
            if a != -1 and b != -1 and b > a:
                buffer = buffer[b+2:]
                frames += 1
        req.close()
    except Exception as e:
        pass
    elapsed = max(0.1, time.time() - t0)
    return {"drone_id": drone_id, "frames": frames, "fps": round(frames / elapsed, 2)}

async def run_validation():
    print("=== STARTING BOUNDED POST-FIX VALIDATION (FOREST SAR, 2 DRONES) ===")
    
    # 1. Health check
    r = requests.get(f"{BACKEND_URL}/api/health", timeout=3)
    print(f"Backend status: {r.json()}")

    # 2. Start Mission with MAX_DECISIONS=10 to keep it bounded
    os.environ["MAX_DECISIONS"] = "10"
    payload = {"map_id": "forest_sar", "drone_count": 2, "victim_count": 3}
    r = requests.post(f"{BACKEND_URL}/api/mission/start", json=payload, timeout=25)
    print(f"Mission start response ({r.status_code}): {r.json()}")
    if r.status_code != 200:
        return

    telemetry_samples = []
    decision_events = []
    state_transitions = []
    rtf_samples = []
    prev_decisions = 0
    prev_global_state = None
    navigating_time = 0.0
    synchronizing_time = 0.0
    last_tick_time = time.time()

    running_start_wall = None
    mission_stopped = False

    try:
        async with websockets.connect(WS_URL, open_timeout=45) as ws:
            print("Connected to WebSocket telemetry. Awaiting mission startup...")
            last_sample_t = time.time()

            while True:
                now = time.time()
                dt = now - last_tick_time
                last_tick_time = now

                # Track time in states
                if prev_global_state == "NAVIGATING":
                    navigating_time += dt
                elif prev_global_state == "SYNCHRONIZING":
                    synchronizing_time += dt

                # Sample RTF every 5s
                if (now - last_sample_t) >= 5.0:
                    last_sample_t = now
                    rtf_val, sim_sec = get_gz_stats()
                    if rtf_val is not None:
                        rtf_samples.append({"wall_time": round(now, 1), "rtf": rtf_val, "sim_sec": sim_sec})

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

                mission_info = msg.get("mission", {})
                status = mission_info.get("status")
                decisions = mission_info.get("decision_count", 0)
                coverage = mission_info.get("coverage", 0.0)
                sim_time = mission_info.get("sim_time", 0.0)
                global_state = mission_info.get("global_step_state", "UNKNOWN")
                drones = msg.get("drones", [])

                if status == "RUNNING" and running_start_wall is None:
                    running_start_wall = now
                    print(f"\n>>> MISSION ENTERED RUNNING STATE at wall_time {now:.1f}s, sim_time {sim_time:.1f}s <<<")

                if global_state != prev_global_state:
                    state_transitions.append({
                        "wall_time": round(now, 2),
                        "sim_time": round(sim_time, 2),
                        "decision": decisions,
                        "from_state": prev_global_state,
                        "to_state": global_state
                    })
                    print(f"[{now:.1f}s wall | {sim_time:.1f}s sim] STATE TRANSITION: {prev_global_state} -> {global_state} (Decisions: {decisions}, Coverage: {coverage:.2f}%)")
                    prev_global_state = global_state

                if decisions > prev_decisions:
                    decision_events.append({
                        "decision": decisions,
                        "wall_time": round(now, 2),
                        "sim_time": round(sim_time, 2),
                        "coverage": coverage,
                        "drones": [{
                            "id": d.get("id"),
                            "x": d.get("x"), "y": d.get("y"), "z": d.get("z"),
                            "state": d.get("state"), "action": d.get("action")
                        } for d in drones]
                    })
                    print(f"[{now:.1f}s wall | {sim_time:.1f}s sim] *** QMIX DECISION {decisions} EXECUTED *** Coverage: {coverage:.2f}%")
                    prev_decisions = decisions

                # Bounded validation criteria:
                # Once decisions progress beyond step 2 (i.e. reaches step 3 or 4) AND we have captured NAVIGATING & SYNCHRONIZING
                if running_start_wall is not None:
                    elapsed_running = now - running_start_wall
                    if decisions >= 3 and any(s["to_state"] == "NAVIGATING" for s in state_transitions) and any(s["to_state"] == "SYNCHRONIZING" for s in state_transitions):
                        print(f"\nSUCCESS CRITERIA MET: Decisions reached {decisions} (beyond step 2), verified NAVIGATING & SYNCHRONIZING transitions.")
                        break

                    # Maximum safety wall timeout for running state: 160 seconds
                    if elapsed_running > 160.0:
                        print("\nBounded validation timeout reached (160s). Stopping.")
                        break

    finally:
        print("\nStopping mission...")
        try:
            requests.post(f"{BACKEND_URL}/api/mission/stop", timeout=10)
        except Exception:
            pass

    # Sample camera streams to verify stability
    print("Sampling camera streams post-flight...")
    cam0 = sample_camera(0, sample_duration=3.0)
    cam1 = sample_camera(1, sample_duration=3.0)
    print(f"Camera 0: {cam0}")
    print(f"Camera 1: {cam1}")

    # Compile Final Report
    report = {
        "initial_decisions": decision_events[0]["decision"] if decision_events else 0,
        "final_decisions": decision_events[-1]["decision"] if decision_events else 0,
        "total_decision_events": len(decision_events),
        "decision_history": decision_events,
        "state_transitions": state_transitions,
        "navigating_wall_sec": round(navigating_time, 2),
        "synchronizing_wall_sec": round(synchronizing_time, 2),
        "rtf_samples": rtf_samples,
        "median_rtf": sorted([s["rtf"] for s in rtf_samples])[len(rtf_samples)//2] if rtf_samples else None,
        "cameras": [cam0, cam1]
    }

    report_path = "/home/capstone/capstone_project_antigravity/results/post_fix_validation_report.json"
    os.makedirs(os.path.dirname(report_path), exist_ok=True)
    with open(report_path, "w") as f:
        json.dump(report, f, indent=2)
    print(f"\nValidation Report saved to: {report_path}")

if __name__ == "__main__":
    asyncio.run(run_validation())
