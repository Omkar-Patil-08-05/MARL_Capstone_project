#!/bin/bash
# scripts/launch_6_drone_rgbd.sh
# Presentation-ready launcher for the N-Drone QMIX + PX4 System with RGB-D
set -e

DRONE_COUNT=${1:-6}

echo "=================================================="
echo "    LAUNCHING ${DRONE_COUNT}-DRONE QMIX + PX4 RGB-D SYSTEM"
echo "=================================================="

# Function to cleanly tear down the entire stack
cleanup() {
    echo -e "\n[Shutdown] Cleaning up launch stack..."
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

# 1. Start PX4 and Gazebo environment using launch_swarm_rgbd.sh
echo "[1/2] Starting PX4 Swarm Environment (realistic_sar, ${DRONE_COUNT} drones)..."
bash scripts/launch_swarm_rgbd.sh realistic_sar ${DRONE_COUNT} | tee /tmp/launch_swarm_rgbd.log &
LAUNCH_PID=$!

echo "[*] Waiting for launch_swarm_rgbd.sh to complete initialization..."
echo "[*] (This spins up Gazebo, PX4 SITL instances, and Perception Node. Please wait)"

# Wait until the "All instances started" message appears indicating readiness
while ! grep -q "All instances started" /tmp/launch_swarm_rgbd.log 2>/dev/null; do
    if ! kill -0 $LAUNCH_PID 2>/dev/null; then
        echo "Error: launch_swarm_rgbd.sh failed or exited unexpectedly."
        exit 1
    fi
    sleep 2
done

# 2. Start Swarm Runner with N drones in QMIX mode
echo "[2/2] PX4 Swarm Initialized! Starting QMIX Mission Controller for ${DRONE_COUNT} drones..."
source /opt/ros/jazzy/setup.bash
source /home/capstone/capstone_project_antigravity/drone_ws/install/setup.bash

export PYTHONUNBUFFERED=1

ros2 run swarm_controller swarm_runner --drones ${DRONE_COUNT} --controller qmix &
RUNNER_PID=$!

echo "=================================================="
echo "    SYSTEM RUNNING! Press Ctrl+C to stop.         "
echo "=================================================="

wait $RUNNER_PID
