#!/bin/bash
# One-command launcher for the MARL SAR Live Demonstration
set -e

export GZ_IP=127.0.0.1
export ROS_LOCALHOST_ONLY=1

echo "=================================================="
echo "MARL SAR LIVE DEMONSTRATION - STARTUP SEQUENCE"
echo "=================================================="

# 1. Clean stale processes
echo "[1/8] Cleaning up stale processes..."
pkill -f "MicroXRCEAgent" || true
pkill -f "px4" || true
pkill -9 -f "gz sim" || true
pkill -9 -f "ruby.*gz" || true
pkill -f "qmix_drone_test" || true
pkill -f "uvicorn" || true
pkill -f "vite" || true
sleep 2

# Track PIDs for cleanup
PIDS=()
cleanup() {
    echo ""
    echo "=================================================="
    echo "SHUTTING DOWN DEMONSTRATION"
    echo "=================================================="
    for pid in "${PIDS[@]}"; do
        if kill -0 $pid 2>/dev/null; then
            kill -TERM $pid 2>/dev/null || true
        fi
    done
    
    # Fallback absolute cleanup
    pkill -f "MicroXRCEAgent" || true
    pkill -f "px4" || true
    pkill -9 -f "gz sim" || true
    pkill -9 -f "ruby.*gz" || true
    pkill -f "qmix_drone_test" || true
    pkill -f "uvicorn" || true
    pkill -f "vite" || true
    echo "Shutdown complete."
}
trap cleanup EXIT INT TERM

# 3. Start Gazebo/PX4 Stack
echo "[2/8] Starting Simulator Stack (Gazebo, PX4, MicroXRCEAgent)..."
bash scripts/launch_two_drones.sh > /tmp/launch_drones.log 2>&1 &
LAUNCH_PID=$!
PIDS+=($LAUNCH_PID)

echo "Waiting for Simulator to initialize (max 90s)..."
source /opt/ros/jazzy/setup.bash

TOPICS_READY=0
for i in {1..30}; do
    if ros2 topic list | grep -q "/drone_0/fmu/out/vehicle_odometry" && \
       ros2 topic list | grep -q "/drone_1/fmu/out/vehicle_odometry"; then
        TOPICS_READY=1
        break
    fi
    sleep 3
done

if [ $TOPICS_READY -eq 0 ]; then
    echo "ERROR: Drone odometry topics are unavailable. Simulator failed to start."
    echo "Check /tmp/launch_drones.log for details."
    exit 1
fi
echo "Drone odometry topics detected. Simulator is alive."

# 4. Wait for PX4/EKF2 stabilization
echo "[3/8] Waiting 60 seconds for EKF2 stabilization to prevent Failsafe/AUTO.LAND..."
for i in {1..60}; do
    echo -ne "Stabilizing: $i/60s\r"
    sleep 1
done
echo -e "\nEKF2 Stabilization complete."

# 5. Start FastAPI
echo "[4/8] Starting FastAPI Backend..."
(
    cd dashboard/backend
    source /opt/ros/jazzy/setup.bash
    uvicorn main:app --host 0.0.0.0 --port 8000 > /tmp/fastapi.log 2>&1
) &
FASTAPI_PID=$!
PIDS+=($FASTAPI_PID)

echo "Waiting for FastAPI readiness..."
while ! curl -s http://localhost:8000/api/health > /dev/null; do
    sleep 2
done
echo "FastAPI is responding successfully."

# 6. Start React/Vite
echo "[5/8] Starting React Frontend..."
(
    cd dashboard/frontend
    npm run dev > /tmp/vite.log 2>&1
) &
VITE_PID=$!
PIDS+=($VITE_PID)

echo "Waiting for React Dashboard readiness..."
while ! curl -s http://localhost:5173 > /dev/null; do
    sleep 2
done
echo "React Dashboard is reachable."

# 7. Print ready message
echo ""
echo "=================================================="
echo "MARL SAR LIVE DEMONSTRATION READY"
echo "=================================================="
echo "Dashboard:"
echo "http://localhost:5173"
echo ""
echo "Backend:"
echo "http://localhost:8000"
echo ""
echo "Telemetry:"
echo " /swarm/telemetry"
echo ""
echo "Gazebo:"
echo "HEADLESS"
echo ""
echo "Drones:"
echo "2 × PX4 SITL"
echo ""
echo "Policy:"
echo "QMIX V4 ALIGNED"
echo "=================================================="
echo ""

# 8. Start QMIX controller
echo "[6/8] Starting QMIX Mission Controller..."
(
    source /opt/ros/jazzy/setup.bash
    source /home/capstone/capstone_project_antigravity/drone_ws/install/setup.bash
    export ROS_LOCALHOST_ONLY=1
    ros2 run swarm_controller qmix_drone_test > /tmp/qmix.log 2>&1
) &
QMIX_PID=$!
PIDS+=($QMIX_PID)

echo "[7/8] Demonstration is running! View live telemetry in your browser."
echo "Open in browser: http://localhost:5173"
echo "[8/8] Press Ctrl+C to safely shutdown all systems."
sleep infinity
