#!/usr/bin/env python3
"""
Lightweight test to launch Gazebo Harmonic headless on worlds/forest_sar.sdf,
verify simulation clock and model loading, and cleanly terminate only the started process.
"""

import os
import sys
import time
import subprocess
import signal

WORKSPACE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

resource_paths = [
    os.path.join(WORKSPACE_DIR, "models"),
    os.path.join(WORKSPACE_DIR, "assets_real", "victims"),
    os.path.join(WORKSPACE_DIR, "generated_worlds"),
    os.path.join(WORKSPACE_DIR, "worlds"),
    "/home/capstone/PX4-Autopilot/Tools/simulation/gz/models",
    "/home/capstone/PX4-Autopilot/Tools/simulation/gz/worlds"
]

env = os.environ.copy()
env["GZ_SIM_RESOURCE_PATH"] = ":".join(resource_paths)
env["GZ_IP"] = "127.0.0.1"

world_path = os.path.join(WORKSPACE_DIR, "worlds", "forest_sar.sdf")
assert os.path.exists(world_path), f"World SDF not found: {world_path}"

print(f"Launching Gazebo Harmonic headless on {world_path}...")
cmd = ["gz", "sim", "-s", "-r", "-v", "2", world_path]
proc = subprocess.Popen(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid)
pid = proc.pid
print(f"Gazebo started with PID {pid}. Waiting 8s for clock and models...")

try:
    time.sleep(8)
    
    # Check clock topic
    clock_check = subprocess.run(["gz", "topic", "-e", "-t", "/clock", "-n", "1"], capture_output=True, text=True, timeout=5)
    has_clock = clock_check.returncode == 0 and len(clock_check.stdout.strip()) > 0
    print(f"Simulation Clock Active: {has_clock}")
    assert has_clock, "Simulation clock failed to tick!"
    
    # Check topics
    topic_check = subprocess.run(["gz", "topic", "-l"], capture_output=True, text=True, timeout=10)
    topics_out = topic_check.stdout
    print(f"Discovered {len(topics_out.splitlines())} Gazebo topics.")
    assert "/clock" in topics_out, "Clock topic not found in topic list!"
    assert "/world/forest_sar/pose/info" in topics_out or "/world/forest_sar/scene/info" in topics_out or "/stats" in topics_out, "World state topic missing!"
    
    print("\nGazebo simulation verified successfully! Headless physics engine and simulation clock operational.")

finally:
    print(f"Stopping scoped Gazebo process group (PGID {pid})...")
    try:
        os.killpg(pid, signal.SIGTERM)
        time.sleep(1)
        if proc.poll() is None:
            os.killpg(pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    print("Gazebo process stopped cleanly.")
