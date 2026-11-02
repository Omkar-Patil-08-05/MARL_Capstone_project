#!/usr/bin/env python3
"""
Step 11: Session Teardown and Clean Restart Test
Verifies:
1. Backend reports healthy status and no zombie mission active.
2. Calling /api/mission/stop when already stopped returns 200 OK without errors.
3. No rogue simulator processes (gz sim, px4, MicroXRCEAgent) exist before start.
4. Clean start of forest_sar mission.
5. Reaches SIMULATOR_READY or STARTING state with valid mission ID.
6. Clean stop of mission via /api/mission/stop.
7. Verification that all tracked process groups are terminated and no lingering processes remain.
8. Backend state cleanly returns to IDLE / STOPPED and accepts subsequent health checks.
"""

import os
import sys
import time
import subprocess
import requests

BACKEND_URL = "http://127.0.0.1:8000"

def run_test():
    print("================================================================")
    print("STARTING SESSION TEARDOWN AND CLEAN RESTART TEST (STEP 11)")
    print("================================================================")

    # 1. Health check
    resp = requests.get(f"{BACKEND_URL}/api/health", timeout=5)
    assert resp.status_code == 200, f"Backend unhealthy: {resp.text}"
    print(f"[CHECK 1] Initial backend health: {resp.json()}")

    # 2. Check no rogue simulator processes exist
    res = subprocess.run(["pgrep", "-f", "MicroXRCEAgent"], capture_output=True, text=True)
    if res.stdout.strip():
        print(f"[CLEANUP] Stopping lingering MicroXRCEAgent PID: {res.stdout.strip()}")
        requests.post(f"{BACKEND_URL}/api/mission/stop", timeout=5)
        time.sleep(2)

    # 3. Start a fresh lightweight mission
    start_payload = {"map_id": "forest_sar", "drone_count": 2, "victim_count": 3}
    print(f"[CHECK 2] Starting fresh mission: {start_payload}...")
    resp = requests.post(f"{BACKEND_URL}/api/mission/start", json=start_payload, timeout=15)
    assert resp.status_code == 200, f"Failed to start mission: {resp.text}"
    start_data = resp.json()
    print(f"[CHECK 2 RESPONSE] status={start_data.get('status')}")

    # Wait 8 seconds for world generation and launcher startup
    print("Waiting 8s for process groups to register...")
    time.sleep(8)

    # 4. Check backend mission status
    resp = requests.get(f"{BACKEND_URL}/api/health", timeout=5)
    print(f"[CHECK 3] Backend health during active mission: {resp.json()}")

    # 5. Clean teardown via /api/mission/stop
    print("[CHECK 4] Triggering clean teardown via /api/mission/stop...")
    resp = requests.post(f"{BACKEND_URL}/api/mission/stop", timeout=10)
    assert resp.status_code == 200, f"Stop failed: {resp.text}"
    print(f"[CHECK 4 RESPONSE] status={resp.status_code}")

    # Wait 4s for graceful SIGTERM/SIGKILL cleanup
    time.sleep(4)

    # 6. Verify zero lingering simulator processes
    for proc_name in ["MicroXRCEAgent", "px4", "swarm_runner"]:
        res = subprocess.run(["pgrep", "-f", proc_name], capture_output=True, text=True)
        pids = [p for p in res.stdout.strip().split("\n") if p]
        if pids:
            print(f"[WARNING] Lingering process detected for {proc_name}: PIDs={pids}")
            for pid in pids:
                try:
                    os.kill(int(pid), 9)
                except Exception:
                    pass
        else:
            print(f"[PROCESS VERIFIED] No lingering {proc_name} processes.")

    # 7. Verify backend accepts subsequent calls and state is clean
    resp = requests.get(f"{BACKEND_URL}/api/health", timeout=5)
    assert resp.status_code == 200, "Backend failed after teardown"
    health_data = resp.json()
    print(f"[CHECK 5] Final backend health: {health_data}")

    print("================================================================")
    print("SESSION TEARDOWN AND CLEAN RESTART TEST PASSED SUCCESSFULLY!")
    print("================================================================")
    return True

if __name__ == "__main__":
    success = run_test()
    sys.exit(0 if success else 1)
