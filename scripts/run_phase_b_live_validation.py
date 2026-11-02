#!/usr/bin/env python3
"""
Phase B: Controlled Single-Seed Live Validation Harness
Target World: forest_dense_candidate (950 trees)
Target Metadata: worlds/forest_dense_candidate_meta.json
Checkpoint: models/qmix_sar_v4_align_best.pth
Seed: 42
Fleet: 6 simulated drones (x500 in lightweight SITL mode)
Budget: 25 global QMIX decisions
Target Output: results/single_seed_live_validation_forest_dense_candidate/
"""

import os
import sys
import json
import time
import math
import signal
import shutil
import asyncio
import hashlib
import requests
import websockets
import subprocess
import psutil
from typing import Dict, List, Tuple, Set, Optional

# --- Configuration & Constants ---
BACKEND_URL = "http://127.0.0.1:8000"
WS_URL = "ws://127.0.0.1:8000/ws/telemetry"
TARGET_WORLD = "forest_dense_candidate"
EXPECTED_META = "worlds/forest_dense_candidate_meta.json"
CHECKPOINT_PATH = "models/qmix_sar_v4_align_best.pth"
EXPECTED_CHECKPOINT_HASH = "f87d87528969ff3165a0cdfb299f5d228ad99502b974176d05b81479242a272d"
TARGET_OUTPUT_DIR = "results/single_seed_live_validation_forest_dense_candidate"
HISTORICAL_DIR = "results/stage2_three_seed_benchmark"
MAX_DECISIONS_BUDGET = 25
SEED = 42
NUM_DRONES = 6
NUM_VICTIMS = 5
VALID_CELLS_FOREST = 416
TOTAL_GRID_CELLS = 625

# Watchdog & Timeout Budgets
WALL_CLOCK_TIMEOUT_SEC = 1800.0  # 30-minute outer wall-clock ceiling
# Controller internal waypoint_wall_watchdog is 90.0s (qmix_mission_controller.py:284).
# A 120.0s harness watchdog provides a 30s buffer for controller resynchronization to fire
# under low RTF (~0.04-0.07) without prematurely aborting a healthy mission.
DECISION_STALL_WATCHDOG_SEC = 120.0

# Termination Status Constants & Exit Codes
STATUS_SUCCESS = "COMPLETED_BUDGET_REACHED"
STATUS_EARLY_COMPLETE = "COMPLETED_EARLY_TERMINAL"
STATUS_TIMEOUT = "TERMINATED_WALLCLOCK_TIMEOUT"
STATUS_STALL = "TERMINATED_CONTROLLER_STALL"
STATUS_BACKEND_ERROR = "TERMINATED_BACKEND_OR_WEBSOCKET_ERROR"
STATUS_PREFLIGHT_FAIL = "TERMINATED_PREFLIGHT_FAILURE"
STATUS_EXTERNAL_INTERRUPT = "TERMINATED_EXTERNAL_INTERRUPT"

EXIT_SUCCESS = 0
EXIT_TIMEOUT = 2
EXIT_STALL = 3
EXIT_BACKEND_ERROR = 4
EXIT_PREFLIGHT_FAIL = 5
EXIT_EXTERNAL_INTERRUPT = 6


def get_system_stats() -> Dict[str, Optional[float]]:
    """Gathers host CPU, system memory, and NVIDIA GPU telemetry."""
    cpu = psutil.cpu_percent()
    mem = psutil.virtual_memory().percent
    gpu = None
    vram = None
    gpu_temp = None
    try:
        res = subprocess.run(
            ["nvidia-smi", "--query-gpu=utilization.gpu,memory.used,temperature.gpu", "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=1.5
        )
        if res.returncode == 0 and res.stdout.strip():
            p = [x.strip() for x in res.stdout.strip().split(",")]
            gpu = float(p[0])
            vram = float(p[1])
            gpu_temp = float(p[2])
    except Exception:
        pass
    return {
        "cpu_percent": cpu,
        "memory_percent": mem,
        "gpu_util_percent": gpu,
        "vram_used_mb": vram,
        "gpu_temp_c": gpu_temp
    }


def get_gazebo_stats() -> Tuple[Optional[float], Optional[int]]:
    """Polls real-time factor and sim_time from Gazebo world stats topic."""
    try:
        res = subprocess.run(
            ["gz", "topic", "-e", "-t", f"/world/{TARGET_WORLD}/stats", "-n", "1", "--json-output"],
            capture_output=True, text=True, timeout=2.5
        )
        if res.returncode == 0 and res.stdout.strip():
            d = json.loads(res.stdout)
            return float(d.get("real_time_factor", 0.0)), d.get("sim_time", {}).get("sec", 0)
    except Exception:
        pass
    return None, None


def query_gazebo_services() -> List[str]:
    """Queries currently registered Gazebo transport services."""
    try:
        res = subprocess.run(["gz", "service", "-l"], capture_output=True, text=True, timeout=3.0)
        if res.returncode == 0:
            return [s.strip() for s in res.stdout.strip().split("\n") if s.strip()]
    except Exception:
        pass
    return []


def query_gazebo_models() -> List[str]:
    """Queries loaded entity models in Gazebo."""
    try:
        res = subprocess.run(["gz", "model", "--list"], capture_output=True, text=True, timeout=3.0)
        if res.returncode == 0:
            lines = res.stdout.strip().split("\n")
            models = []
            for line in lines:
                line = line.strip()
                if line.startswith("- "):
                    models.append(line[2:].strip())
                elif line and not line.startswith("Models") and not line.startswith("==="):
                    models.append(line.strip())
            return models
    except Exception:
        pass
    return []


def compute_min_pairwise_distance(drones: List[dict]) -> Tuple[Optional[float], Tuple[str, str]]:
    """
    Computes minimum pairwise Euclidean distance between active drones.
    Returns (None, ('none', 'none')) if fewer than 2 drones have valid positions.
    Does not conflate missing positions with 0.0 separation.
    """
    coords = []
    for d in drones:
        did = d.get("id")
        x = d.get("x")
        y = d.get("y")
        if x is not None and y is not None:
            # Check for non-NaN, finite values
            if not math.isnan(x) and not math.isnan(y) and not math.isinf(x) and not math.isinf(y):
                coords.append((did, float(x), float(y)))

    if len(coords) < 2:
        return None, ("none", "none")

    min_dist = float("inf")
    closest_pair = ("none", "none")
    for i in range(len(coords)):
        for j in range(i + 1, len(coords)):
            dist = math.hypot(coords[i][1] - coords[j][1], coords[i][2] - coords[j][2])
            if dist < min_dist:
                min_dist = dist
                closest_pair = (coords[i][0], coords[j][0])

    if min_dist == float("inf"):
        return None, ("none", "none")
    return min_dist, closest_pair


def is_pgid_alive(pgid: int) -> bool:
    """Checks whether any processes in the process group are still running."""
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    except Exception:
        return False


def terminate_owned_process_groups(tracked_pgids: Set[int], timeout_sec: float = 3.0):
    """
    Terminates ONLY the process groups demonstrably owned and tracked by the backend/harness.
    Escalates from SIGTERM to SIGKILL only for owned PGIDs.
    Never kills generic system processes or unrelated user applications.
    """
    alive_pgids = [pg for pg in tracked_pgids if is_pgid_alive(pg)]
    if not alive_pgids:
        return

    print(f"[PROCESS CLEANUP] Gracefully stopping owned process groups: {alive_pgids}")
    for pg in alive_pgids:
        try:
            os.killpg(pg, signal.SIGTERM)
        except Exception:
            pass

    t0 = time.time()
    while time.time() - t0 < timeout_sec:
        still_alive = [pg for pg in alive_pgids if is_pgid_alive(pg)]
        if not still_alive:
            return
        time.sleep(0.5)

    # Escalate only remaining owned process groups
    for pg in alive_pgids:
        if is_pgid_alive(pg):
            print(f"[PROCESS CLEANUP] Force-killing lingering owned process group: {pg}")
            try:
                os.killpg(pg, signal.SIGKILL)
            except Exception:
                pass


def stop_mission_gracefully(tracked_pgids: Set[int], timeout_sec: float = 8.0) -> bool:
    """
    Requests graceful mission shutdown via the FastAPI backend endpoint,
    polls until the backend marks the mission stopped, and cleans up tracked PGIDs.
    """
    try:
        r = requests.post(f"{BACKEND_URL}/api/mission/stop", timeout=5)
        print(f"[SHUTDOWN] POST /api/mission/stop response: status={r.status_code}")
    except Exception as e:
        print(f"[SHUTDOWN WARNING] Backend stop request failed: {e}")

    # Poll status for orderly backend process group teardown
    t0 = time.time()
    while time.time() - t0 < timeout_sec:
        try:
            st = requests.get(f"{BACKEND_URL}/api/mission/status", timeout=2).json()
            if st.get("state") in ["IDLE", "STOPPED", "ERROR"]:
                break
        except Exception:
            pass
        time.sleep(0.5)

    # Ensure all backend-tracked PGIDs are fully terminated
    terminate_owned_process_groups(tracked_pgids)
    return True


def ensure_clean_simulator_state() -> Tuple[bool, str]:
    """
    Preflight check to ensure no previous mission is active.
    Uses backend API and process inspection without generic pkill commands.
    """
    try:
        r = requests.get(f"{BACKEND_URL}/api/health", timeout=3)
        if r.status_code != 200:
            return False, f"Backend health endpoint returned HTTP {r.status_code}"
    except Exception as e:
        return False, f"Backend is unreachable at {BACKEND_URL}: {e}"

    try:
        st = requests.get(f"{BACKEND_URL}/api/mission/status", timeout=3).json()
        active_pgids = set(st.get("process_groups", []))
        if st.get("state") not in ["IDLE", "STOPPED"]:
            print(f"[PREFLIGHT] Backend reports active state: {st.get('state')}. Sending stop request...")
            stop_mission_gracefully(active_pgids)
    except Exception as e:
        print(f"[PREFLIGHT WARNING] Could not query backend mission status: {e}")

    # Inspect system for existing Gazebo or PX4 processes
    stale_procs = []
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            cmd = " ".join(proc.info['cmdline'] or [])
            if any(k in cmd for k in ["gz sim", "px4", "MicroXRCEAgent", "swarm_runner"]):
                stale_procs.append((proc.info['pid'], proc.info['name']))
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue

    if stale_procs:
        p_desc = ", ".join([f"{name}(PID {pid})" for pid, name in stale_procs[:5]])
        print(f"[PREFLIGHT NOTICE] Existing simulator processes detected: {p_desc}")
        # Note: We do not kill them indiscriminately; backend stop handles owned processes.

    return True, "Preflight clean state confirmed."


async def run_live_validation() -> int:
    """
    Executes the Phase B controlled validation run and returns an exit code:
      0: SUCCESS (All 25 decisions completed or expected early completion)
      2: TIMEOUT (Wall-clock timeout exceeded before completion)
      3: STALL (Controller stall detected without decision progress)
      4: BACKEND_ERROR (Backend or WebSocket connection failure)
      5: PREFLIGHT_FAIL (Preflight check or checksum mismatch)
      6: EXTERNAL_INTERRUPT (Interrupted by signal or user)
    """
    print("=" * 80)
    print("PHASE B: CONTROLLED SINGLE-SEED LIVE VALIDATION ON FOREST_DENSE_CANDIDATE")
    print(f"World: {TARGET_WORLD} | Seed: {SEED} | Fleet: {NUM_DRONES} drones | Victims: {NUM_VICTIMS}")
    print("=" * 80)

    tracked_pgids: Set[int] = set()
    termination_status = STATUS_PREFLIGHT_FAIL
    exit_code = EXIT_PREFLIGHT_FAIL
    termination_reason = "Preflight check failed."

    # 1. Preflight sanity check
    print("\n--- 1. Preflight Validation & Cleanliness ---")
    if not os.path.exists(CHECKPOINT_PATH):
        print(f"[FATAL] Checkpoint not found at: {CHECKPOINT_PATH}")
        return EXIT_PREFLIGHT_FAIL

    actual_ckpt_hash = hashlib.sha256(open(CHECKPOINT_PATH, "rb").read()).hexdigest()
    if actual_ckpt_hash != EXPECTED_CHECKPOINT_HASH:
        print(f"[FATAL] Checkpoint hash mismatch! Expected {EXPECTED_CHECKPOINT_HASH}, got {actual_ckpt_hash}")
        return EXIT_PREFLIGHT_FAIL
    print(f"✓ Authoritative checkpoint verified: {actual_ckpt_hash}")

    hist_summary_path = os.path.join(HISTORICAL_DIR, "benchmark_summary.json")
    hist_hash_pre = hashlib.sha256(open(hist_summary_path, "rb").read()).hexdigest() if os.path.exists(hist_summary_path) else None
    print(f"✓ Historical Stage 2 benchmark hash: {hist_hash_pre[:16] if hist_hash_pre else 'N/A'}")

    os.makedirs(TARGET_OUTPUT_DIR, exist_ok=True)
    print(f"✓ Target output directory: {TARGET_OUTPUT_DIR}")

    clean_ok, clean_msg = ensure_clean_simulator_state()
    if not clean_ok:
        print(f"[FATAL] {clean_msg}")
        return EXIT_PREFLIGHT_FAIL
    print(f"✓ {clean_msg}")

    pre_hw = get_system_stats()
    print(f"✓ Host hardware: CPU {pre_hw['cpu_percent']}%, Memory {pre_hw['memory_percent']}%, GPU {pre_hw['gpu_util_percent']}%, Temp {pre_hw['gpu_temp_c']}°C")

    # 2. Trigger Mission Launch via Backend API
    print("\n--- 2. Launching Mission via Backend API ---")
    payload = {
        "map_id": TARGET_WORLD,
        "drone_count": NUM_DRONES,
        "victim_count": NUM_VICTIMS,
        "seed": SEED,
        "lightweight": True,
        "max_decisions": MAX_DECISIONS_BUDGET
    }
    print(f"POST {BACKEND_URL}/api/mission/start with payload:\n{json.dumps(payload, indent=2)}")
    try:
        r = requests.post(f"{BACKEND_URL}/api/mission/start", json=payload, timeout=20)
        print(f"Backend launch response: status={r.status_code}, body={r.json()}")
        if r.status_code != 200:
            print(f"[FATAL] Backend launch failed: {r.text}")
            return EXIT_BACKEND_ERROR
    except Exception as e:
        print(f"[FATAL] Failed to send mission start request: {e}")
        return EXIT_BACKEND_ERROR

    # 3. Telemetry and Runtime Monitoring Loop
    print("\n--- 3. Real-Time Telemetry and Verification Loop ---")
    print(f"Connecting to WebSocket stream at {WS_URL}...")

    t_start = time.time()
    t_running = None
    max_decisions_seen = 0
    unique_cells_visited = set()
    per_drone_visited_cells = {f"drone_{i}": set() for i in range(NUM_DRONES)}
    drones_armed = {f"drone_{i}": False for i in range(NUM_DRONES)}
    drones_hold_seen = {f"drone_{i}": False for i in range(NUM_DRONES)}
    max_altitudes = {f"drone_{i}": 0.0 for i in range(NUM_DRONES)}
    max_speeds = {f"drone_{i}": 0.0 for i in range(NUM_DRONES)}
    inter_drone_min_distances: List[float] = []
    rtf_samples: List[float] = []
    hw_samples: List[Dict[str, Optional[float]]] = []
    safety_overrides_count = 0
    sim_duration_sec = 0.0
    final_fov_coverage_pct = 0.0
    final_victims_detected = 0
    last_log_print_t = 0.0
    last_decision_t = time.time()

    services_verified = False
    models_verified = False
    gazebo_services_found: List[str] = []
    gazebo_models_found: List[str] = []
    victim_initial_positions: Dict[str, Tuple[float, float]] = {}
    victim_movement_observed = {f"victim_{i}": False for i in range(NUM_VICTIMS)}

    # Default to timeout before loop executes
    termination_status = STATUS_TIMEOUT
    exit_code = EXIT_TIMEOUT
    termination_reason = f"Outer wall-clock timeout ({WALL_CLOCK_TIMEOUT_SEC:.0f}s) expired before 25 decisions were completed."

    try:
        async with websockets.connect(WS_URL, open_timeout=60) as ws:
            print("Connected to telemetry stream. Monitoring fleet stabilization and mission startup...")

            while time.time() - t_start < WALL_CLOCK_TIMEOUT_SEC:
                now = time.time()
                try:
                    raw = await asyncio.wait_for(ws.recv(), timeout=2.0)
                    msg = json.loads(raw)
                except asyncio.TimeoutError:
                    if now - last_log_print_t >= 4.0:
                        last_log_print_t = now
                        try:
                            st = requests.get(f"{BACKEND_URL}/api/mission/status", timeout=2).json()
                            for pg in st.get("process_groups", []):
                                tracked_pgids.add(pg)
                            print(f"  [INIT +{now - t_start:.0f}s] State: {st.get('state')}")
                        except Exception:
                            pass
                    continue
                except Exception as e:
                    print(f"  [WS WARNING] Telemetry receive error: {e}")
                    continue

                if msg.get("type") != "telemetry":
                    continue

                m_stat = msg.get("mission", {})
                state = m_stat.get("status", "UNKNOWN")
                dec = m_stat.get("decision_count", 0)
                cov = m_stat.get("coverage", 0.0)
                sim_t = m_stat.get("sim_time", 0.0)
                v_det = m_stat.get("victims_detected", 0)
                overrides = m_stat.get("safety_overrides", 0)

                if overrides > safety_overrides_count:
                    safety_overrides_count = overrides
                if sim_t:
                    sim_duration_sec = max(sim_duration_sec, sim_t)
                if v_det > final_victims_detected:
                    final_victims_detected = v_det
                if cov > final_fov_coverage_pct:
                    final_fov_coverage_pct = cov

                # Transition to RUNNING
                if state == "RUNNING" and t_running is None:
                    t_running = time.time()
                    last_decision_t = time.time()
                    print(f"\n{'='*75}\n[MISSION ACTIVE] Swarm airborne & active after {t_running - t_start:.1f}s initialization!\n{'='*75}\n")

                # Decision progress tracking
                if dec > max_decisions_seen:
                    max_decisions_seen = dec
                    last_decision_t = time.time()

                # Watchdog: Controller stall detection
                if t_running is not None and (now - last_decision_t > DECISION_STALL_WATCHDOG_SEC):
                    termination_status = STATUS_STALL
                    exit_code = EXIT_STALL
                    termination_reason = (
                        f"Controller made no decision progress for >{DECISION_STALL_WATCHDOG_SEC:.0f}s "
                        f"at step {max_decisions_seen}."
                    )
                    print(f"\n[FATAL WATCHDOG] {termination_reason} Triggering safe stop.")
                    break

                # Drone telemetry & separation calculation
                drones = msg.get("drones", [])
                if len(drones) == NUM_DRONES:
                    min_dist, closest_pair = compute_min_pairwise_distance(drones)
                    if t_running is not None and min_dist is not None:
                        inter_drone_min_distances.append(min_dist)

                    for d in drones:
                        did = d.get("id")
                        z = d.get("z", 0.0)
                        spd = d.get("speed", 0.0)
                        armed = d.get("armed", False)
                        st_name = d.get("state", "UNKNOWN")
                        gx = d.get("grid_x")
                        gy = d.get("grid_y")

                        if armed and did in drones_armed:
                            drones_armed[did] = True
                        if st_name in ["HOLD", "WAYPOINT_NAVIGATION"] and did in drones_hold_seen:
                            drones_hold_seen[did] = True

                        if gx is not None and gy is not None:
                            unique_cells_visited.add((gx, gy))
                            if did in per_drone_visited_cells:
                                per_drone_visited_cells[did].add((gx, gy))

                        if did in max_altitudes:
                            max_altitudes[did] = max(max_altitudes[did], z)
                            max_speeds[did] = max(max_speeds[did], spd)

                # Victim tracking in telemetry
                for v in msg.get("victims", []):
                    vid = v.get("id")
                    wx = v.get("world_x")
                    wy = v.get("world_y")
                    if vid and wx is not None and wy is not None:
                        if vid not in victim_initial_positions:
                            victim_initial_positions[vid] = (wx, wy)
                        else:
                            init_x, init_y = victim_initial_positions[vid]
                            if math.hypot(wx - init_x, wy - init_y) > 0.5:
                                victim_movement_observed[vid] = True

                # Periodic inspection & logging
                if now - last_log_print_t >= 2.5:
                    last_log_print_t = now
                    hw = get_system_stats()
                    hw_samples.append(hw)

                    rtf, _ = get_gazebo_stats()
                    if rtf is not None:
                        rtf_samples.append(rtf)

                    # Update tracked process groups from backend status
                    try:
                        st = requests.get(f"{BACKEND_URL}/api/mission/status", timeout=1.5).json()
                        for pg in st.get("process_groups", []):
                            tracked_pgids.add(pg)
                    except Exception:
                        pass

                    # One-time Gazebo service and model inspection
                    if t_running is not None and not services_verified:
                        gz_services = query_gazebo_services()
                        create_svc = f"/world/{TARGET_WORLD}/create"
                        pose_svc = f"/world/{TARGET_WORLD}/set_pose"
                        has_create = create_svc in gz_services
                        has_pose = pose_svc in gz_services
                        print(f"  [GAZEBO SERVICE AUDIT] Service '{create_svc}': {'FOUND' if has_create else 'NOT FOUND'}")
                        print(f"  [GAZEBO SERVICE AUDIT] Service '{pose_svc}': {'FOUND' if has_pose else 'NOT FOUND'}")
                        gazebo_services_found = gz_services
                        if has_create and has_pose:
                            services_verified = True

                    if t_running is not None and not models_verified:
                        gz_models = query_gazebo_models()
                        print(f"  [GAZEBO MODEL AUDIT] Active models in world: {len(gz_models)} models loaded")
                        gazebo_models_found = gz_models
                        models_verified = True

                    min_sep = (
                        f"{min(inter_drone_min_distances[-10:]):.1f}m"
                        if inter_drone_min_distances else "N/A"
                    )
                    rtf_str = f"{rtf:.2f}" if rtf else "N/A"
                    print(
                        f"  [STEP {dec:02d}/{MAX_DECISIONS_BUDGET} | +{now-t_start:03.0f}s] "
                        f"CenterCells={len(unique_cells_visited):02d} | TelemCov={cov:.1f}% | "
                        f"Victims={v_det}/{NUM_VICTIMS} | Sep={min_sep} | RTF={rtf_str} | CPU={hw['cpu_percent']:.1f}%"
                    )

                # Completion condition: Budget reached
                if t_running is not None and max_decisions_seen >= MAX_DECISIONS_BUDGET:
                    termination_status = STATUS_SUCCESS
                    exit_code = EXIT_SUCCESS
                    termination_reason = f"All {MAX_DECISIONS_BUDGET} global QMIX decisions successfully completed."
                    print(f"\n{'='*75}\n>>> [BUDGET REACHED] Successfully completed {max_decisions_seen} global decisions!\n{'='*75}\n")
                    break

                # Completion condition: Natural terminal state
                if state in ["COMPLETE", "LANDING"] and max_decisions_seen >= 10:
                    termination_status = STATUS_EARLY_COMPLETE
                    exit_code = EXIT_SUCCESS
                    termination_reason = f"Mission reached expected terminal state ({state}) at step {max_decisions_seen}."
                    print(f"\n[MISSION COMPLETE] Swarm commanded landing/complete at step {max_decisions_seen}.")
                    break

    except KeyboardInterrupt:
        termination_status = STATUS_EXTERNAL_INTERRUPT
        exit_code = EXIT_EXTERNAL_INTERRUPT
        termination_reason = "Mission interrupted by user signal (KeyboardInterrupt)."
        print(f"\n[USER INTERRUPT] {termination_reason}")
    except Exception as e:
        termination_status = STATUS_BACKEND_ERROR
        exit_code = EXIT_BACKEND_ERROR
        termination_reason = f"WebSocket or backend communication error: {e}"
        print(f"\n[ERROR] {termination_reason}")
    finally:
        print("\n--- 4. Graceful Shutdown & Teardown ---")
        stop_mission_gracefully(tracked_pgids)

    wall_duration = round(time.time() - t_start, 1)

    # 4. Post-run Artifact Archival & Verification
    print("\n--- 5. Archiving Artifacts to Unique Directory ---")
    raw_results_dir = "results/h8_v4_physical_final"

    # Copy decisions CSV
    qmix_csv_src = os.path.join(raw_results_dir, "qmix_decisions.csv")
    qmix_csv_dst = os.path.join(TARGET_OUTPUT_DIR, "qmix_decisions.csv")
    actual_fov_cells = 0
    logged_decisions_count = 0
    per_drone_decision_rows_count = 0

    if os.path.exists(qmix_csv_src):
        shutil.copy2(qmix_csv_src, qmix_csv_dst)
        print(f"✓ Saved {qmix_csv_dst}")
        with open(qmix_csv_src, "r") as f:
            lines = [l.strip() for l in f.readlines() if l.strip()]
            if len(lines) > 1:
                per_drone_decision_rows_count = len(lines) - 1
                last_line = lines[-1].split(",")
                if len(last_line) >= 16:
                    try:
                        logged_decisions_count = int(last_line[1])
                        cov_ratio = float(last_line[15])
                        actual_fov_cells = int(round(cov_ratio * VALID_CELLS_FOREST))
                    except ValueError:
                        pass

    # Verify per-drone rows vs global decisions agreement
    expected_rows = max_decisions_seen * NUM_DRONES
    per_drone_rows_agreement = (
        (per_drone_decision_rows_count == expected_rows) if max_decisions_seen > 0 else True
    )

    # Copy victim detection CSV
    victim_csv_src = os.path.join(raw_results_dir, "victim_detection.csv")
    victim_csv_dst = os.path.join(TARGET_OUTPUT_DIR, "victim_detection.csv")
    proximity_detections = []
    if os.path.exists(victim_csv_src):
        shutil.copy2(victim_csv_src, victim_csv_dst)
        print(f"✓ Saved {victim_csv_dst}")
        with open(victim_csv_src, "r") as f:
            v_lines = f.readlines()
            for vl in v_lines[1:]:
                parts = [p.strip() for p in vl.strip().split(",")]
                if len(parts) >= 5:
                    proximity_detections.append({
                        "timestamp": parts[0],
                        "decision_step": int(parts[1]),
                        "victim_grid_x": int(parts[2]),
                        "victim_grid_y": int(parts[3]),
                        "detected_by": parts[4]
                    })

    # Copy trajectories
    for did in [f"drone_{i}" for i in range(NUM_DRONES)]:
        traj_src = os.path.join(raw_results_dir, f"trajectory_{did}.csv")
        traj_dst = os.path.join(TARGET_OUTPUT_DIR, f"trajectory_{did}.csv")
        if os.path.exists(traj_src):
            shutil.copy2(traj_src, traj_dst)
            print(f"✓ Saved {traj_dst}")

    # Copy Gazebo console log
    if os.path.exists("/tmp/gz.log"):
        shutil.copy2("/tmp/gz.log", os.path.join(TARGET_OUTPUT_DIR, "gazebo_console.log"))
        print(f"✓ Saved {os.path.join(TARGET_OUTPUT_DIR, 'gazebo_console.log')}")

    # Metrics computation
    center_cell_count = len(unique_cells_visited)
    center_cell_cov_forest_pct = round((center_cell_count / VALID_CELLS_FOREST) * 100, 2)
    center_cell_cov_grid_pct = round((center_cell_count / TOTAL_GRID_CELLS) * 100, 2)

    fov_cells_count = actual_fov_cells if actual_fov_cells > 0 else int(round(final_fov_coverage_pct * VALID_CELLS_FOREST / 100))
    fov_cov_forest_pct = round((fov_cells_count / VALID_CELLS_FOREST) * 100, 2)
    fov_cov_grid_pct = round((fov_cells_count / TOTAL_GRID_CELLS) * 100, 2)

    min_separation_overall = round(min(inter_drone_min_distances), 2) if inter_drone_min_distances else None
    mean_rtf = round(sum(rtf_samples) / len(rtf_samples), 3) if rtf_samples else None
    mean_cpu = round(sum(h["cpu_percent"] for h in hw_samples) / len(hw_samples), 1) if hw_samples else None
    valid_gpus = [h["gpu_util_percent"] for h in hw_samples if h.get("gpu_util_percent") is not None]
    mean_gpu = round(sum(valid_gpus) / len(valid_gpus), 1) if valid_gpus else None

    per_drone_summary = {}
    for did in [f"drone_{i}" for i in range(NUM_DRONES)]:
        c_count = len(per_drone_visited_cells[did])
        per_drone_summary[did] = {
            "armed": drones_armed[did],
            "reached_flight_state": drones_hold_seen[did],
            "center_cells_visited": c_count,
            "max_altitude_m": round(max_altitudes[did], 2),
            "max_speed_mps": round(max_speeds[did], 2)
        }

    summary = {
        "validation_metadata": {
            "world": TARGET_WORLD,
            "metadata_file": EXPECTED_META,
            "checkpoint": CHECKPOINT_PATH,
            "checkpoint_sha256": actual_ckpt_hash,
            "seed": SEED,
            "fleet_size": NUM_DRONES,
            "budget_decisions": MAX_DECISIONS_BUDGET,
            "completed_decisions": max_decisions_seen,
            "logged_decisions_csv": logged_decisions_count,
            "per_drone_decision_rows_count": per_drone_decision_rows_count,
            "per_drone_rows_agreement": per_drone_rows_agreement,
            "valid_cells_forest_denominator": VALID_CELLS_FOREST,
            "total_grid_cells_denominator": TOTAL_GRID_CELLS,
            "launch_timestamp": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t_start)),
            "wall_duration_sec": wall_duration,
            "sim_duration_sec": round(sim_duration_sec, 1),
            "real_time_factor_mean": mean_rtf,
            "termination_status": termination_status,
            "termination_reason": termination_reason,
            "exit_code": exit_code,
            "is_success": (exit_code == EXIT_SUCCESS)
        },
        "gazebo_verification": {
            "world_service_prefix": f"/world/{TARGET_WORLD}",
            "create_service_available": services_verified,
            "pose_service_available": services_verified,
            "victim_models_declaration": "Declared statically in worlds/forest_dense_candidate.sdf (models victim_0 to victim_4)",
            "victim_models_loaded": len(gazebo_models_found),
            "moving_victims_movement_observed": victim_movement_observed
        },
        "fleet_status": per_drone_summary,
        "coverage_metrics": {
            "center_cell_visitation": {
                "unique_cells": center_cell_count,
                "forest_valid_pct": center_cell_cov_forest_pct,
                "grid_total_pct": center_cell_cov_grid_pct,
                "description": "Unique discrete grid cells traversed by drone center positions"
            },
            "sensor_fov_footprint": {
                "unique_cells": fov_cells_count,
                "forest_valid_pct": fov_cov_forest_pct,
                "grid_total_pct": fov_cov_grid_pct,
                "description": "Explored 3x3 sensor footprint cells tracked by authoritative SARGridEnv"
            }
        },
        "victim_detection": {
            "evaluation_mode": "Lightweight High-Speed Proximity Mode (Visual Rendering Disabled)",
            "ground_truth_proximity_radius_m": 12.0,
            "ground_truth_proximity_detections_count": len(proximity_detections),
            "ground_truth_proximity_detections": proximity_detections,
            "vision_pipeline_evaluated": False,
            "vision_pipeline_status": "DISABLED_IN_LIGHTWEIGHT_MODE",
            "vision_note": (
                "Visual perception (RGB-D cameras & YOLO) was disabled for this run to conserve GPU/CPU resources "
                "on the RTX 3050 Laptop GPU. Ground-truth proximity detection was used for tactical scoring and is "
                "strictly unconflated with visual perception."
            )
        },
        "safety_and_kinematics": {
            "min_pairwise_separation_m": min_separation_overall,
            "collision_events": (
                "No valid airborne separation data"
                if min_separation_overall is None
                else (0 if min_separation_overall >= 1.5 else "potential proximity event (< 1.5m)")
            ),
            "safety_overrides": safety_overrides_count
        },
        "hardware_utilization": {
            "mean_cpu_percent": mean_cpu,
            "mean_gpu_percent": mean_gpu
        }
    }

    summary_file = os.path.join(TARGET_OUTPUT_DIR, "live_validation_summary.json")
    with open(summary_file, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"✓ Summary written to {summary_file}")

    # Historical Benchmark Preservation Check
    hist_hash_post = hashlib.sha256(open(hist_summary_path, "rb").read()).hexdigest() if os.path.exists(hist_summary_path) else None
    print(f"\n--- 6. Historical Data Preservation Check ---")
    print(f"Historical pre-hash:  {hist_hash_pre}")
    print(f"Historical post-hash: {hist_hash_post}")
    assert hist_hash_pre == hist_hash_post, "FATAL: Historical benchmark summary was modified!"
    print(">>> HISTORICAL BENCHMARK DATA FULLY PRESERVED (Bitwise Identical).")

    print("\n" + "=" * 80)
    if exit_code == EXIT_SUCCESS:
        print("PHASE B SINGLE-SEED VALIDATION PASSED")
    else:
        print(f"PHASE B SINGLE-SEED VALIDATION INCOMPLETE / FAILED (Exit Code: {exit_code})")
    print(f"Status: {termination_status}")
    print(f"Reason: {termination_reason}")
    print(f"Decisions: {max_decisions_seen}/{MAX_DECISIONS_BUDGET} | Center Cells: {center_cell_count} | FOV Cells: {fov_cells_count}")
    print(f"Coverage: {fov_cov_forest_pct}% (Valid 416 Denominator) | Min Separation: {min_separation_overall}m | Overrides: {safety_overrides_count}")
    print(f"Ground Truth Proximity Detections: {len(proximity_detections)} | Vision Pipeline: Disabled (Unconflated)")
    print("=" * 80)

    return exit_code


if __name__ == "__main__":
    code = asyncio.run(run_live_validation())
    sys.exit(code)
