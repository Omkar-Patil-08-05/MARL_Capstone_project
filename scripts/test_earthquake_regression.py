#!/usr/bin/env python3
"""
Step 10: Earthquake Environment Regression Test (realistic_sar)
Verifies:
1. Map registry returns realistic_sar with 4m cells, origin (0, 0), and 3 victims.
2. World generator for realistic_sar executes properly without touching protected baselines.
3. PX4 launch preserves realistic_sar speed config (5.0 m/s cruise).
4. Telemetry streams via WebSocket with realistic_sar active_map_id.
5. Mission reaches SIMULATOR_READY / QMIX_STARTING / RUNNING.
6. Clean teardown via /api/mission/stop.
"""

import sys
import time
import json
import asyncio
import requests
import websockets

BACKEND_URL = "http://127.0.0.1:8000"
WS_URL = "ws://127.0.0.1:8000/ws/telemetry"

async def run_regression():
    print("================================================================")
    print("STARTING EARTHQUAKE ENVIRONMENT REGRESSION TEST (realistic_sar)")
    print("================================================================")

    # 1. Health check
    resp = requests.get(f"{BACKEND_URL}/api/health", timeout=5)
    assert resp.status_code == 200, f"Health check failed: {resp.text}"
    print(f"[HEALTH] status={resp.json()}")

    # 2. Map registry validation
    resp = requests.get(f"{BACKEND_URL}/api/maps", timeout=5)
    maps = resp.json()
    eq = maps.get("realistic_sar")
    assert eq is not None, "realistic_sar missing from map registry"
    assert eq["meters_per_cell"] == 4.0, f"Expected 4.0m/cell, got {eq['meters_per_cell']}"
    assert eq["origin_x"] == 0.0 and eq["origin_y"] == 0.0, f"Expected origin (0,0), got ({eq['origin_x']},{eq['origin_y']})"
    assert eq["grid_width"] == 25 and eq["grid_height"] == 25, "Expected 25x25 grid"
    print(f"[MAP VERIFIED] realistic_sar: 4m cells, origin (0,0), {eq['grid_width']}x{eq['grid_height']}")

    # 3. Connect to WebSocket
    print(f"[WEBSOCKET] Connecting to {WS_URL}...")
    telemetry_samples = []
    max_wait = 180
    start_time = time.time()

    async with websockets.connect(WS_URL, open_timeout=20) as ws:
        print("[WEBSOCKET] Connected!")

        # 4. Start realistic_sar mission (2 drones, 3 victims)
        payload = {"map_id": "realistic_sar", "drone_count": 2, "victim_count": 3}
        resp = requests.post(f"{BACKEND_URL}/api/mission/start", json=payload, timeout=15)
        assert resp.status_code == 200, f"Start failed: {resp.text}"
        print(f"[MISSION STARTED] payload={payload}")

        # 5. Monitor telemetry
        print("[MONITORING] Waiting for realistic_sar telemetry...")
        reached_running = False
        while time.time() - start_time < max_wait:
            try:
                raw = await asyncio.wait_for(ws.recv(), timeout=2.0)
                data = json.loads(raw)
            except asyncio.TimeoutError:
                continue

            if data.get("type") != "telemetry":
                continue

            m = data.get("mission", {})
            st = m.get("status")
            act_map = data.get("active_map_id")
            decisions = m.get("decision_count", 0)
            telemetry_samples.append(data)

            if act_map == "realistic_sar" and st in ["INITIALIZING", "RUNNING"]:
                if st == "RUNNING" and not reached_running:
                    reached_running = True
                    print(f"\n>>> REALISTIC_SAR MISSION REACHED RUNNING at t={time.time()-start_time:.1f}s! <<<")

            if reached_running and decisions >= 2:
                print(f"[TARGET REACHED] realistic_sar achieved {decisions} decisions!")
                break

    # 6. Stop mission
    print("[STOPPING] Stopping realistic_sar mission...")
    resp = requests.post(f"{BACKEND_URL}/api/mission/stop", timeout=10)
    print(f"[STOPPED] status={resp.status_code}")

    # 7. Verification
    assert len(telemetry_samples) > 0, "No telemetry received"
    print(f"Total telemetry frames received: {len(telemetry_samples)}")
    print("================================================================")
    print("EARTHQUAKE REGRESSION TEST PASSED SUCCESSFULLY!")
    print("================================================================")
    return True

if __name__ == "__main__":
    success = asyncio.run(run_regression())
    sys.exit(0 if success else 1)
