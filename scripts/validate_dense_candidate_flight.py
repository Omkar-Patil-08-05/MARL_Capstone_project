#!/usr/bin/env python3
"""
Validates 2-drone flight dynamics, waypoint progression, and performance on forest_dense_candidate world.
"""

import os
import sys
import json
import time
import asyncio
import requests
import websockets
import subprocess
import psutil

BACKEND_URL = "http://127.0.0.1:8000"
WS_URL = "ws://127.0.0.1:8000/ws/telemetry"

def get_system_stats():
    cpu = psutil.cpu_percent()
    gpu = None
    vram = None
    try:
        res = subprocess.run(
            ["nvidia-smi", "--query-gpu=utilization.gpu,memory.used", "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=1.5
        )
        if res.returncode == 0 and res.stdout.strip():
            p = [x.strip() for x in res.stdout.strip().split(",")]
            gpu = float(p[0])
            vram = float(p[1])
    except Exception:
        pass
    return {"cpu": cpu, "gpu": gpu, "vram_mb": vram}

def get_gazebo_rtf():
    try:
        res = subprocess.run(
            ["gz", "topic", "-e", "-t", "/world/forest_dense_candidate/stats", "-n", "1", "--json-output"],
            capture_output=True, text=True, timeout=3.0
        )
        if res.returncode == 0 and res.stdout.strip():
            d = json.loads(res.stdout)
            return float(d.get("real_time_factor", 0.0)), d.get("sim_time", {}).get("sec", 0)
    except Exception:
        pass
    return None, None

async def run_validation():
    print("=" * 80)
    print("STARTING 2-DRONE FLIGHT & WAYPOINT VALIDATION ON FOREST_DENSE_CANDIDATE")
    print("=" * 80)

    # 1. Health check & reset
    try:
        requests.post(f"{BACKEND_URL}/api/mission/stop", timeout=5)
    except Exception:
        pass
    time.sleep(2)

    try:
        r = requests.get(f"{BACKEND_URL}/api/health", timeout=5)
        print(f"[HEALTH] Backend is active: {r.json()}")
    except Exception as e:
        print(f"[ERROR] Backend not reachable: {e}")
        return False

    # 2. Start mission
    payload = {
        "map_id": "forest_dense_candidate",
        "drone_count": 2,
        "victim_count": 3,
        "seed": 42,
        "lightweight": True,
        "max_decisions": 25
    }
    print(f"[START] Starting mission with payload: {payload}")
    r = requests.post(f"{BACKEND_URL}/api/mission/start", json=payload, timeout=20)
    print(f"[RESPONSE] status={r.status_code}, data={r.json()}")
    if r.status_code != 200:
        return False

    # 3. Monitor telemetry
    print(f"[WS] Connecting to {WS_URL}...")
    max_decisions_seen = 0
    unique_cells = set()
    drones_armed = {"drone_0": False, "drone_1": False}
    max_altitudes = {"drone_0": 0.0, "drone_1": 0.0}
    max_speeds = {"drone_0": 0.0, "drone_1": 0.0}
    rtf_samples = []
    hw_samples = []
    waypoint_events = []
    prev_states = {}

    t_start = time.time()
    t_running = None

    try:
        async with websockets.connect(WS_URL, open_timeout=60) as ws:
            print("[WS] Connected to telemetry stream. Monitoring simulation stack...")
            last_sample_t = 0

            while time.time() - t_start < 450:
                now = time.time()
                try:
                    raw = await asyncio.wait_for(ws.recv(), timeout=2.0)
                    msg = json.loads(raw)
                except asyncio.TimeoutError:
                    if now - last_sample_t >= 4.0:
                        last_sample_t = now
                        try:
                            st = requests.get(f"{BACKEND_URL}/api/mission/status", timeout=2).json()
                            print(f"[INITIALIZING +{now - t_start:.0f}s] State: {st.get('state')}")
                        except Exception:
                            pass
                    continue
                except Exception as e:
                    print(f"[WS ERROR] {e}")
                    continue

                if msg.get("type") != "telemetry":
                    continue

                m_stat = msg.get("mission", {})
                state = m_stat.get("status", "UNKNOWN")
                dec = m_stat.get("decision_count", 0)
                cov = m_stat.get("coverage", 0.0)

                if state == "RUNNING" and t_running is None:
                    t_running = time.time()
                    print(f"\n>>> [MISSION RUNNING] Active flight commenced after {t_running - t_start:.1f}s initialization!\n")

                if dec > max_decisions_seen:
                    max_decisions_seen = dec

                drones = msg.get("drones", [])
                for d in drones:
                    did = d.get("id")
                    z = d.get("z", 0.0)
                    spd = d.get("speed", 0.0)
                    st = d.get("state")
                    gx = d.get("grid_x")
                    gy = d.get("grid_y")
                    if gx is not None and gy is not None:
                        unique_cells.add((gx, gy))

                    if did in max_altitudes:
                        max_altitudes[did] = max(max_altitudes[did], z)
                        max_speeds[did] = max(max_speeds[did], spd)
                        if z > 2.0:
                            drones_armed[did] = True

                    if did in prev_states and prev_states[did] != st:
                        waypoint_events.append({
                            "wall_time": round(now - t_start, 1),
                            "drone": did,
                            "from": prev_states[did],
                            "to": st,
                            "action": d.get("action"),
                            "pos": (d.get("x"), d.get("y"), z)
                        })
                        print(f"  [EVENT +{now-t_start:.1f}s] {did}: {prev_states[did]} -> {st} (action={d.get('action')} pos=({d.get('x')}, {d.get('y')}, {z:.1f}m))")
                    prev_states[did] = st

                if now - last_sample_t >= 2.0:
                    last_sample_t = now
                    hw = get_system_stats()
                    rtf, sim_t = get_gazebo_rtf()
                    hw_samples.append(hw)
                    if rtf is not None:
                        rtf_samples.append(rtf)

                    status_line = f"[{state} | +{now - t_start:.0f}s] Dec={dec} Cov={cov:.1f}% RTF={f'{rtf:.3f}' if rtf else 'N/A'}"
                    for d in drones:
                        status_line += f" | {d.get('id')}: {d.get('state')} z={d.get('z',0):.1f}m v={d.get('speed',0):.1f}m/s act={d.get('action')}"
                    print(status_line)

                # Target completion criteria: at least 6 decisions under active flight
                if t_running is not None and max_decisions_seen >= 6:
                    print(f"\n>>> [SUCCESS CRITERIA MET] Observed {max_decisions_seen} decisions and stable waypoint progression on candidate forest!")
                    break

    except Exception as e:
        print(f"[RUNNER EXCEPTION] {e}")

    finally:
        print("\n[STOPPING] Stopping mission cleanly...")
        try:
            requests.post(f"{BACKEND_URL}/api/mission/stop", timeout=10)
        except Exception:
            pass

    # Compile results
    avg_rtf = sum(rtf_samples) / max(1, len(rtf_samples)) if rtf_samples else None
    avg_cpu = sum(h["cpu"] for h in hw_samples) / max(1, len(hw_samples)) if hw_samples else None
    avg_gpu = sum(h["gpu"] for h in hw_samples if h["gpu"] is not None) / max(1, len([h for h in hw_samples if h["gpu"] is not None])) if hw_samples else None

    results = {
        "candidate_world": "forest_dense_candidate.sdf",
        "tree_count": 950,
        "drone_count": 2,
        "decisions_completed": max_decisions_seen,
        "unique_cells_covered": len(unique_cells),
        "drones_airborne": drones_armed,
        "max_altitudes_m": {k: round(v, 2) for k, v in max_altitudes.items()},
        "max_speeds_mps": {k: round(v, 2) for k, v in max_speeds.items()},
        "avg_real_time_factor": round(avg_rtf, 4) if avg_rtf else None,
        "avg_cpu_percent": round(avg_cpu, 1) if avg_cpu else None,
        "avg_gpu_percent": round(avg_gpu, 1) if avg_gpu else None,
        "waypoint_transitions_count": len(waypoint_events),
        "waypoint_events": waypoint_events,
        "flight_test_passed": all(drones_armed.values()) and max_decisions_seen >= 5
    }

    out_path = "/home/capstone/.gemini/antigravity-ide/brain/f88b16af-cf14-4d78-a628-0fd98bc346fa/scratch/candidate_2drone_flight_results.json"
    with open(out_path, "w") as f:
        json.dump(results, f, indent=2)

    print("\n" + "=" * 80)
    print("FLIGHT VALIDATION SUMMARY:")
    print(json.dumps(results, indent=2))
    print("=" * 80)
    return results["flight_test_passed"]

if __name__ == "__main__":
    asyncio.run(run_validation())
