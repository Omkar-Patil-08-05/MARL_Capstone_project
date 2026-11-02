#!/usr/bin/env python3
"""
Step 7 & 8: Lightweight Forest Mission & Quantitative Telemetry Benchmark
- Runs forest_sar with 2 drones and 3 victims via the dashboard backend.
- Connects to /ws/telemetry via websockets and samples live mission telemetry.
- Quantitatively measures:
  * Decision count and decision interval (latency / barrier timing)
  * Drone physical speed and displacement vs reported velocity
  * Continuous coverage progression rate
  * Configured victim count (3) and detection confirmation
  * Time spent in WAYPOINT_NAVIGATION vs HOLD
"""

import os
import sys
import json
import time
import math
import asyncio
import requests
import websockets

BACKEND_URL = "http://127.0.0.1:8000"
WS_URL = "ws://127.0.0.1:8000/ws/telemetry"

async def run_benchmark():
    print("================================================================")
    print("STARTING FOREST SAR MISSION BENCHMARK (2 DRONES, 3 VICTIMS)")
    print("================================================================")

    # 1. Check backend health
    try:
        resp = requests.get(f"{BACKEND_URL}/api/health", timeout=3)
        data = resp.json()
        print(f"[BACKEND HEALTH] status={data.get('status')} ros_connected={data.get('ros_connected')}")
    except Exception as e:
        print(f"[ERROR] Backend not running on {BACKEND_URL}: {e}")
        return False

    # 2. Check Map Registry for forest_sar
    resp = requests.get(f"{BACKEND_URL}/api/maps", timeout=3)
    maps = resp.json()
    forest_meta = maps.get("forest_sar", {})
    print(f"[MAP REGISTRY] forest_sar registered victim_count: {forest_meta.get('victim_count')}")

    # 3. Connect to WebSocket FIRST
    print(f"[WEBSOCKET] Connecting to {WS_URL}...")
    telemetry_samples = []
    start_wait_time = time.time()
    mission_running_time = None
    max_duration = 320  # Up to 5.3 minutes total to allow full cold-start and evaluation

    try:
        async with websockets.connect(WS_URL, open_timeout=30) as ws:
            print("[WEBSOCKET] Connected successfully!")

            # 4. Start Mission: forest_sar, 2 drones, 3 victims
            start_payload = {
                "map_id": "forest_sar",
                "drone_count": 2,
                "victim_count": 3
            }
            print(f"[STARTING MISSION] POST /api/mission/start with payload: {start_payload}")
            resp = requests.post(f"{BACKEND_URL}/api/mission/start", json=start_payload, timeout=15)
            print(f"[START RESPONSE] status={resp.status_code} body={resp.json()}")
            if resp.status_code != 200:
                print(f"Failed to start mission: {resp.text}")
                return False

            print("[MONITORING] Waiting for mission telemetry...")
            
            while True:
                try:
                    raw_msg = await asyncio.wait_for(ws.recv(), timeout=2.0)
                except asyncio.TimeoutError:
                    if (time.time() - start_wait_time) > max_duration:
                        print("\n[TIMEOUT] Reached max duration.")
                        break
                    continue

                try:
                    data = json.loads(raw_msg)
                except json.JSONDecodeError:
                    continue

                if data.get("type") != "telemetry":
                    continue

                now = time.time()
                m_stat = data.get("mission", {})
                drones = data.get("drones", [])
                status = m_stat.get("status")
                decisions = m_stat.get("decision_count", 0)
                coverage = m_stat.get("coverage", 0.0)
                victims = data.get("victims", [])

                telemetry_samples.append({
                    "time": now,
                    "status": status,
                    "decisions": decisions,
                    "coverage": coverage,
                    "drones": drones,
                    "victims": victims,
                    "explored_count": m_stat.get("explored_count", 0)
                })

                if status == "RUNNING" and mission_running_time is None:
                    mission_running_time = now
                    print(f"\n>>> MISSION REACHED RUNNING STATE at t={now - start_wait_time:.1f}s <<<\n")

                if mission_running_time and (now - mission_running_time) % 5.0 < 0.2:
                    d_info = ", ".join([f"{d.get('id')}: {d.get('state')} v={d.get('speed', 0):.2f}m/s @({d.get('x', 0):.1f},{d.get('y', 0):.1f})" for d in drones])
                    v_det = sum(1 for v in victims if v.get("detected"))
                    print(f"[{now - mission_running_time:5.1f}s] Dec={decisions:2d} | Cov={coverage:5.1f}% | Victims Det={v_det}/{len(victims)} | {d_info}")

                # Stop after achieving at least 8 QMIX decisions in RUNNING
                if mission_running_time and decisions >= 8:
                    print(f"\n[TARGET REACHED] Achieved {decisions} QMIX decisions! Completing evaluation.")
                    break

                if (now - start_wait_time) > max_duration:
                    print(f"\n[TIMEOUT] Reached max duration {max_duration}s.")
                    break

    except Exception as e:
        print(f"[WEBSOCKET ERROR]: {e}")

    # 5. Stop Mission cleanly
    print("\n[STOPPING MISSION] Calling /api/mission/stop...")
    try:
        resp = requests.post(f"{BACKEND_URL}/api/mission/stop", timeout=10)
        print(f"[STOPPED] status={resp.status_code}")
    except Exception as e:
        print(f"[STOP ERROR]: {e}")

    # 6. Quantitative Analysis
    print("\n================================================================")
    print("QUANTITATIVE BENCHMARK ANALYSIS")
    print("================================================================")
    if not telemetry_samples:
        print("ERROR: No telemetry samples received.")
        return False

    running_samples = [s for s in telemetry_samples if s["status"] == "RUNNING"]
    if not running_samples:
        print("WARNING: Mission did not reach RUNNING status within test duration.")
        return False

    first_run = running_samples[0]
    last_run = running_samples[-1]
    run_duration = last_run["time"] - first_run["time"]

    first_cov = first_run["coverage"]
    last_cov = last_run["coverage"]
    total_cov_change = last_cov - first_cov

    first_dec = first_run["decisions"]
    last_dec = last_run["decisions"]
    total_decisions = last_dec - first_dec

    first_expl = first_run["explored_count"]
    last_expl = last_run["explored_count"]

    print(f"Active RUNNING Duration: {run_duration:.1f} s")
    print(f"QMIX Decisions Made:     {total_decisions} (interval: {run_duration / max(1, total_decisions):.2f} s/decision)")
    print(f"Coverage Progression:    {first_cov:.1f}% -> {last_cov:.1f}% (+{total_cov_change:.1f}%)")
    print(f"Explored Cells Count:    {first_expl} -> {last_expl} (+{last_expl - first_expl} cells)")

    # Drones kinematic analysis
    for drone_idx in [0, 1]:
        d_id = f"drone_{drone_idx}"
        speeds = []
        positions = []
        states = []
        for s in running_samples:
            for d in s["drones"]:
                if d.get("id") == d_id:
                    speeds.append(d.get("speed", 0.0))
                    positions.append((d.get("x", 0.0), d.get("y", 0.0), d.get("z", 0.0), s["time"]))
                    states.append(d.get("state", "UNKNOWN"))

        if positions:
            total_dist = 0.0
            for i in range(1, len(positions)):
                dx = positions[i][0] - positions[i-1][0]
                dy = positions[i][1] - positions[i-1][1]
                total_dist += math.hypot(dx, dy)

            avg_reported_speed = sum(speeds) / max(1, len(speeds))
            max_reported_speed = max(speeds) if speeds else 0.0
            avg_physical_speed = total_dist / max(0.1, run_duration)

            nav_ticks = sum(1 for st in states if st == "WAYPOINT_NAVIGATION")
            hold_ticks = sum(1 for st in states if st == "HOLD")

            print(f"\n--- {d_id.upper()} KINEMATICS ---")
            print(f"  Total Distance Travelled:    {total_dist:.1f} m")
            print(f"  Max Instantaneous Velocity:  {max_reported_speed:.2f} m/s")
            print(f"  Avg Reported Velocity:       {avg_reported_speed:.2f} m/s")
            print(f"  Physical Mean Velocity:      {avg_physical_speed:.2f} m/s")
            print(f"  Time in WAYPOINT_NAVIGATION: {nav_ticks / max(1, len(states)) * 100:.1f}%")
            print(f"  Time in HOLD / SYNC:         {hold_ticks / max(1, len(states)) * 100:.1f}%")

    # Victim configuration verification
    last_victims = last_run["victims"]
    print(f"\n--- VICTIM STATUS (CONFIGURED: 3) ---")
    print(f"  Total Victim Count in Telemetry: {len(last_victims)}")
    assert len(last_victims) == 3, f"Expected 3 victims in telemetry, got {len(last_victims)}"
    for v in last_victims:
        print(f"  - {v.get('id')}: detected={v.get('detected')} world=({v.get('world_x', 0):.1f}, {v.get('world_y', 0):.1f}) grid=({v.get('x')}, {v.get('y')})")

    print("\n================================================================")
    print("BENCHMARK COMPLETED SUCCESSFULLY!")
    print("================================================================")
    return True

if __name__ == "__main__":
    success = asyncio.run(run_benchmark())
    sys.exit(0 if success else 1)
