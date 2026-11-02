#!/usr/bin/env python3
"""
Full Forest-World Simulation Runner with Autonomous Video Recording & Event Capture.

Orchestrates:
1. Preflight verifications:
   - Checkpoint integrity (SHA-256 matches baseline f87d87528969ff3165a0cdfb299f5d228ad99502b974176d05b81479242a272d).
   - Confirms production target of 300 decisions.
   - Verifies disk space and process tree hygiene.
2. Starts headless Gazebo with RGB-D downward cameras, PX4 SITL, YOLO perception, and victim localizer.
3. Starts the Autonomous Mission Recorder (capturing continuous H.264 MP4 video, confirmed victim snapshots, and 2D tactical map).
4. Launches the QMIX Swarm Mission Controller with the 300-decision target.
5. Monitors flight progress, telemetry, coverage, victim events, and system resources.
6. Gracefully finalizes recordings and outputs a comprehensive evaluation evidence package upon completion.
"""

import os
import sys
import time
import json
import hashlib
import signal
import asyncio
import argparse
import datetime
import subprocess
from typing import Dict, Any, List, Optional, Tuple

import requests
import psutil

# Ensure unbuffered output
sys.stdout.reconfigure(line_buffering=True)
sys.stderr.reconfigure(line_buffering=True)

WORKSPACE_DIR = "/home/capstone/capstone_project_antigravity"
BACKEND_URL = "http://127.0.0.1:8000"
CHECKPOINT_PATH = os.path.join(WORKSPACE_DIR, "models", "qmix_sar_v4_align_best.pth")
EXPECTED_SHA256 = "f87d87528969ff3165a0cdfb299f5d228ad99502b974176d05b81479242a272d"


def compute_sha256(filepath: str) -> str:
    h = hashlib.sha256()
    with open(filepath, "rb") as f:
        while chunk := f.read(65536):
            h.update(chunk)
    return h.hexdigest()


def get_hardware_telemetry() -> Dict[str, Any]:
    cpu = psutil.cpu_percent(interval=None)
    mem = psutil.virtual_memory().percent
    gpu = None
    vram = None
    temp = None
    try:
        res = subprocess.run(
            ["nvidia-smi", "--query-gpu=utilization.gpu,memory.used,temperature.gpu", "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=1.5
        )
        if res.returncode == 0 and res.stdout.strip():
            p = [x.strip() for x in res.stdout.strip().split(",")]
            gpu = float(p[0])
            vram = float(p[1])
            temp = float(p[2])
    except Exception:
        pass
    return {"cpu_percent": cpu, "memory_percent": mem, "gpu_util_percent": gpu, "vram_mb": vram, "gpu_temp_c": temp}


def get_gazebo_rtf(world_name: str = "forest_dense_candidate") -> Tuple[Optional[float], Optional[float]]:
    try:
        res = subprocess.run(
            ["gz", "topic", "-e", "-t", f"/world/{world_name}/stats", "-n", "1", "--json-output"],
            capture_output=True, text=True, timeout=3.0
        )
        if res.returncode == 0 and res.stdout.strip():
            d = json.loads(res.stdout)
            return float(d.get("real_time_factor", 0.0)), float(d.get("sim_time", {}).get("sec", 0))
    except Exception:
        pass
    return None, None


async def run_evaluation(
    map_id: str = "forest_dense_candidate",
    drone_count: int = 6,
    max_decisions: int = 300,
    seed: int = 42,
    output_dir: Optional[str] = None,
    timeout: Optional[int] = None
):
    watchdog_timeout = timeout if (timeout is not None and timeout > 0) else max(1800, int(max_decisions * 28))
    print("=" * 80)
    print("MARL SAR FOREST SIMULATION EVALUATION & RECORDING HARNESS")
    print(f"World: {map_id} | Drones: {drone_count} | Decision Target: {max_decisions} | Watchdog Timeout: {watchdog_timeout}s ({watchdog_timeout/60:.1f}m) | Seed: {seed}")
    print("=" * 80)

    # 1. Preflight Verification
    print("\n[PREFLIGHT 1/4] Verifying QMIX Checkpoint Integrity...")
    actual_hash = compute_sha256(CHECKPOINT_PATH)
    print(f"  Target Checkpoint: {CHECKPOINT_PATH}")
    print(f"  SHA-256: {actual_hash}")
    assert actual_hash == EXPECTED_SHA256, f"Checkpoint mismatch! Expected {EXPECTED_SHA256}, got {actual_hash}"
    print("  [PASS] Checkpoint integrity confirmed 100% identical.")

    print("\n[PREFLIGHT 2/4] Verifying Process State & Running Simulations...")
    chk = subprocess.run(
        "pgrep -fa 'gz sim|px4|MicroXRCEAgent|swarm_runner' || echo 'Clean'",
        shell=True, executable="/bin/bash", capture_output=True, text=True
    )
    lines = [l for l in chk.stdout.splitlines() if "pgrep" not in l and "Clean" not in l]
    if lines:
        print(f"  [WARNING] Active simulation processes detected: {len(lines)}")
        for l in lines[:5]:
            print(f"    - {l}")
        print("  Aborting to prevent interrupting an active mission.")
        return False
    print("  [PASS] System is clean. No conflicting simulation is active.")

    print("\n[PREFLIGHT 3/4] Checking Disk Space & Hardware Status...")
    usage = psutil.disk_usage(WORKSPACE_DIR)
    free_gb = usage.free / (1024 ** 3)
    print(f"  Free disk space on {WORKSPACE_DIR}: {free_gb:.1f} GB")
    assert free_gb > 2.0, "Insufficient disk space (< 2.0 GB)!"
    hw = get_hardware_telemetry()
    print(f"  Initial Hardware: CPU={hw['cpu_percent']}%, RAM={hw['memory_percent']}%, GPU={hw['gpu_util_percent']}%, VRAM={hw['vram_mb']}MB, Temp={hw['gpu_temp_c']}C")
    print("  [PASS] Hardware and storage ready.")

    # Prepare Output Directory
    if not output_dir:
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = os.path.join(WORKSPACE_DIR, "results", f"forest_simulation_recording_{ts}")
    os.makedirs(output_dir, exist_ok=True)
    print(f"\n[OUTPUT DIRECTORY] Evidence will be saved to: {output_dir}")

    # Check backend reachability
    backend_active = False
    try:
        r = requests.get(f"{BACKEND_URL}/api/health", timeout=3)
        if r.status_code == 200:
            backend_active = True
            print(f"  FastAPI Backend active at {BACKEND_URL}.")
    except Exception:
        print("  FastAPI Backend not running. Launching direct ROS 2 simulation pipeline.")

    pids_to_clean = []

    def cleanup():
        print("\n[TEARDOWN] Stopping mission and simulation stack...")
        if backend_active:
            try:
                requests.post(f"{BACKEND_URL}/api/mission/stop", timeout=5)
            except Exception:
                pass
        for p in reversed(pids_to_clean):
            try:
                os.killpg(p, signal.SIGTERM)
            except Exception:
                pass
        time.sleep(1)
        for p in reversed(pids_to_clean):
            try:
                os.killpg(p, signal.SIGKILL)
            except Exception:
                pass
        print("[TEARDOWN] Completed.")

    # 2. Launch Simulator Stack
    print("\n[1/3] Starting Gazebo Harmonic Stack with RGB-D Perception...")
    if backend_active:
        start_payload = {
            "map_id": map_id,
            "drone_count": drone_count,
            "victim_count": 5,
            "seed": seed,
            "lightweight": False,
            "max_decisions": max_decisions
        }
        resp = requests.post(f"{BACKEND_URL}/api/mission/start", json=start_payload, timeout=20)
        assert resp.status_code == 200, f"Backend mission start failed: {resp.text}"
        print(f"  Mission started via backend: status={resp.status_code}")
    else:
        # Launch directly via launch_swarm_rgbd.sh
        cmd_sim = f"bash {WORKSPACE_DIR}/scripts/launch_swarm_rgbd.sh {map_id} {drone_count}"
        proc_sim = subprocess.Popen(
            cmd_sim, shell=True, executable="/bin/bash",
            preexec_fn=os.setsid, stdout=open("/tmp/launch_swarm_rgbd.log", "w"), stderr=subprocess.STDOUT
        )
        pids_to_clean.append(proc_sim.pid)
        print("  Waiting for Gazebo and PX4 initialization (up to 90s)...")
        ready = False
        for _ in range(30):
            res = subprocess.run(
                "source /opt/ros/jazzy/setup.bash && export ROS_LOCALHOST_ONLY=1 && ros2 topic list",
                shell=True, executable="/bin/bash", capture_output=True, text=True
            )
            if all(f"/drone_{i}/fmu/out/vehicle_odometry" in res.stdout for i in range(drone_count)):
                ready = True
                break
            time.sleep(3)
        if not ready:
            print("[ERROR] Timed out waiting for drone odometry topics!")
            cleanup()
            return False
        print("  Simulator stack initialized.")

    # 3. Launch Standalone Mission Recorder
    print(f"\n[2/3] Starting Autonomous Mission Recorder targeting {output_dir}...")
    rec_cmd = (
        f"source /opt/ros/jazzy/setup.bash && "
        f"python3 {WORKSPACE_DIR}/scripts/forest_mission_recorder.py "
        f"--output-dir {output_dir} --map-id {map_id} --drone-id 0 --world-fps 5.0 --drone-fps 10.0"
    )
    rec_proc = subprocess.Popen(
        rec_cmd, shell=True, executable="/bin/bash",
        preexec_fn=os.setsid, stdout=open(os.path.join(output_dir, "recorder.log"), "w"), stderr=subprocess.STDOUT
    )
    pids_to_clean.append(rec_proc.pid)
    time.sleep(2)

    # 4. If direct execution, launch QMIX Swarm Runner
    if not backend_active:
        print(f"\n[3/3] Starting QMIX Swarm Runner (300-Decision Target)...")
        runner_cmd = (
            f"source /opt/ros/jazzy/setup.bash && source {WORKSPACE_DIR}/drone_ws/install/setup.bash && "
            f"export ROS_LOCALHOST_ONLY=1 && export MAX_DECISIONS={max_decisions} && "
            f"ros2 run swarm_controller swarm_runner --map {map_id} --drones {drone_count} --controller qmix --ros-args -p use_sim_time:=true"
        )
        runner_proc = subprocess.Popen(
            runner_cmd, shell=True, executable="/bin/bash",
            preexec_fn=os.setsid, stdout=open(os.path.join(output_dir, "swarm_runner.log"), "w"), stderr=subprocess.STDOUT
        )
        pids_to_clean.append(runner_proc.pid)

    # 5. Telemetry & Decision Monitoring Loop
    print("\n" + "=" * 80)
    print("MISSION IN PROGRESS — MONITORING RECORDING & DECISION BUDGET")
    print("=" * 80)

    t_start = time.time()
    last_log_t = 0
    hw_samples = []
    timed_out = False

    try:
        while time.time() - t_start < watchdog_timeout:
            now = time.time()
            if now - last_log_t >= 10.0:
                last_log_t = now
                hw = get_hardware_telemetry()
                hw_samples.append(hw)
                rtf, sim_s = get_gazebo_rtf(map_id)

                # Query status from decision_history.csv or recording_metadata.json
                dec_csv_file = os.path.join(output_dir, "decision_history.csv")
                meta_file = os.path.join(output_dir, "recording_metadata.json")
                dec_cnt = 0
                cov = 0.0
                vic = 0
                if os.path.exists(dec_csv_file):
                    try:
                        with open(dec_csv_file, "r") as f:
                            lines = [l.strip() for l in f.readlines() if l.strip()]
                            if len(lines) > 1:
                                last_line = lines[-1].split(",")
                                dec_cnt = int(last_line[0])
                                cov = float(last_line[3])
                                vic = int(last_line[4])
                    except Exception:
                        pass
                elif os.path.exists(meta_file):
                    try:
                        with open(meta_file, "r") as f:
                            m_dict = json.load(f).get("recording_metadata", {})
                            dec_cnt = m_dict.get("decisions_completed", 0)
                            cov = m_dict.get("final_coverage_pct", 0.0)
                            vic = m_dict.get("victims_detected", 0)
                    except Exception:
                        pass

                elapsed = now - t_start
                print(
                    f"[{elapsed:5.1f}s elapsed] Decisions: {dec_cnt}/{max_decisions} | "
                    f"Coverage: {cov:5.1f}% | Victims: {vic}/5 | "
                    f"RTF: {rtf if rtf else 'N/A'} | CPU: {hw['cpu_percent']}% | GPU: {hw['gpu_util_percent']}% (VRAM: {hw['vram_mb']}MB)"
                )

                if dec_cnt >= max_decisions:
                    print(f"\n[GOAL REACHED] Production target of {max_decisions} decisions completed!")
                    break

                if not backend_active and runner_proc.poll() is not None:
                    print(f"\n[RUNNER COMPLETE] QMIX Swarm Runner process exited with code {runner_proc.returncode}.")
                    break

            time.sleep(2.0)
        else:
            timed_out = True
            print(f"\n[WATCHDOG TIMEOUT] Exceeded safety watchdog timeout of {watchdog_timeout}s ({watchdog_timeout/60:.1f}m) before mission completion!")

    except KeyboardInterrupt:
        print("\n[USER INTERRUPT] Controlled mission stop requested.")
    finally:
        # Give recorder time to finalize gracefully before tearing down simulation
        print("\nStopping recorder gracefully...")
        try:
            os.killpg(rec_proc.pid, signal.SIGINT)
            rec_proc.wait(timeout=20)
        except Exception:
            pass
        cleanup()

    # 6. Verify Evidence Outputs
    print("\n" + "=" * 80)
    print("FINAL EVIDENCE VERIFICATION")
    print("=" * 80)
    world_video = os.path.join(output_dir, "forest_3d_world_overview.mp4")
    drone_video = os.path.join(output_dir, "forest_drone_camera.mp4")
    tac_video = os.path.join(output_dir, "forest_tactical_map_trajectory.mp4")
    snapshots_dir = os.path.join(output_dir, "snapshots")

    if os.path.exists(world_video) and os.path.getsize(world_video) > 0:
        print(f"[VERIFIED] 3D World Video: {world_video} ({os.path.getsize(world_video):,} bytes)")
    if os.path.exists(drone_video) and os.path.getsize(drone_video) > 0:
        print(f"[VERIFIED] Drone Camera Video: {drone_video} ({os.path.getsize(drone_video):,} bytes)")
    if os.path.exists(tac_video) and os.path.getsize(tac_video) > 0:
        print(f"[VERIFIED] 2D Tactical Map Video: {tac_video} ({os.path.getsize(tac_video):,} bytes)")
    if os.path.exists(snapshots_dir):
        snaps = [f for f in os.listdir(snapshots_dir) if f.endswith(".jpg")]
        print(f"[VERIFIED] Confirmed-Victim Snapshots: {len(snaps)} saved in {snapshots_dir}")
        for s in snaps:
            print(f"  - {s}")

    if timed_out:
        print(f"\n[EVALUATION FAILED] Mission terminated due to watchdog timeout ({watchdog_timeout}s).")
        return False

    return True


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Full Forest-World Simulation & Recording Runner")
    parser.add_argument("--map", type=str, default="forest_dense_candidate")
    parser.add_argument("--drones", type=int, default=6)
    parser.add_argument("--decisions", type=int, default=300)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--output-dir", type=str, default=None)
    parser.add_argument("--timeout", type=int, default=None, help="Watchdog safety timeout in seconds (default: max(1800, int(decisions * 28)))")
    args = parser.parse_args()

    asyncio.run(run_evaluation(
        map_id=args.map,
        drone_count=args.drones,
        max_decisions=args.decisions,
        seed=args.seed,
        output_dir=args.output_dir,
        timeout=args.timeout
    ))
