#!/bin/bash
# scripts/launch_6_drone_px4.sh
# Presentation-ready launcher for the 6-Drone QMIX + PX4 System
set -e

echo "=================================================="
echo "    LAUNCHING 6-DRONE QMIX + PX4 SYSTEM           "
echo "=================================================="

# Function to cleanly tear down the entire stack
cleanup() {
    echo -e "\n[Shutdown] Cleaning up 6-drone launch stack..."
    if [ ! -z "$RUNNER_PID" ]; then
        kill -TERM $RUNNER_PID 2>/dev/null || true
    fi
    if [ ! -z "$LAUNCH_PID" ]; then
        kill -TERM $LAUNCH_PID 2>/dev/null || true
        wait $LAUNCH_PID 2>/dev/null || true
    fi
    echo "[Shutdown] Complete."
    exit 0
}
trap cleanup EXIT INT TERM

# 1. Start PX4 and Gazebo environment using launch_swarm.sh
echo "[1/2] Starting PX4 Swarm Environment (realistic_sar, 6 drones)..."
bash scripts/launch_swarm.sh realistic_sar 6 | tee /tmp/launch_swarm_6d.log &
LAUNCH_PID=$!

echo "[*] Waiting for launch_swarm.sh to complete initialization..."
echo "[*] (This spins up Gazebo and 6 PX4 SITL instances. Please wait ~100s)"

# Wait until the "All instances started" message appears indicating readiness
while ! grep -q "All instances started" /tmp/launch_swarm_6d.log 2>/dev/null; do
    if ! kill -0 $LAUNCH_PID 2>/dev/null; then
        echo "Error: launch_swarm.sh failed or exited unexpectedly."
        exit 1
    fi
    sleep 2
done

# 2. Start Swarm Runner with 6 drones in QMIX mode
echo "[2/2] PX4 Swarm Initialized! Starting QMIX Mission Controller for 6 drones..."
source /opt/ros/jazzy/setup.bash
source /home/capstone/capstone_project_antigravity/drone_ws/install/setup.bash

export PYTHONUNBUFFERED=1

ros2 run swarm_controller swarm_runner --drones 6 --controller qmix &
RUNNER_PID=$!

echo "=================================================="
echo "    SYSTEM RUNNING! Press Ctrl+C to stop.         "
echo "=================================================="

wait $RUNNER_PID
