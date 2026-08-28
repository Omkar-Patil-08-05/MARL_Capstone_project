#!/bin/bash
# Script to launch two PX4 drones in Gazebo Harmonic SITL with explicit Gazebo readiness check.
set -e

# Cleanup function
export GZ_IP=127.0.0.1
export ROS_LOCALHOST_ONLY=1
cleanup() {
    echo "Stopping all processes..."
    kill $(jobs -p) 2>/dev/null || true
    pkill -f "MicroXRCEAgent" || true
    pkill -f "px4" || true
    pkill -9 -f "gz sim" || true
    pkill -9 -f "ruby.*gz" || true
    echo "Cleanup complete."
}
trap cleanup EXIT INT TERM

# Ensure clean state before starting
pkill -f "MicroXRCEAgent" || true
pkill -f "px4" || true
pkill -9 -f "gz sim" || true
pkill -9 -f "ruby.*gz" || true
sleep 2

echo "Starting MicroXRCEAgent on UDP 8888..."
MicroXRCEAgent udp4 -p 8888 &
AGENT_PID=$!
sleep 2

# Map parameterization
MAP_ID=${1:-realistic_sar}

if [ "$MAP_ID" == "realistic_sar" ]; then
    META_FILE="/home/capstone/capstone_project_antigravity/worlds/generated_world_meta.json"
    export PX4_GZ_WORLD="realistic_sar"
else
    echo "Error: Unsupported map ID: $MAP_ID"
    exit 1
fi

if [ ! -f "$META_FILE" ]; then
    echo "Error: Metadata file not found at $META_FILE"
    exit 1
fi
D0_X=$(python3 -c "import json; print(json.load(open('$META_FILE'))['drone_base']['spawns'][0]['x'])")
D0_Y=$(python3 -c "import json; print(json.load(open('$META_FILE'))['drone_base']['spawns'][0]['y'])")
D1_X=$(python3 -c "import json; print(json.load(open('$META_FILE'))['drone_base']['spawns'][1]['x'])")
D1_Y=$(python3 -c "import json; print(json.load(open('$META_FILE'))['drone_base']['spawns'][1]['y'])")

echo "Drone 0 spawn: ($D0_X, $D0_Y)"
echo "Drone 1 spawn: ($D1_X, $D1_Y)"

# Setup PX4 Gazebo environment
source /home/capstone/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh
# PX4_GZ_WORLD was exported during map selection above
export PX4_GZ_WORLDS="/home/capstone/capstone_project_antigravity/worlds"
export GZ_SIM_RESOURCE_PATH="/home/capstone/PX4-Autopilot/Tools/simulation/gz/models:/home/capstone/PX4-Autopilot/Tools/simulation/gz/worlds:/home/capstone/capstone_project_antigravity/models:/home/capstone/capstone_project_antigravity/assets_real/victims"

# 1. Start Gazebo Server (Headless)
echo "Starting Gazebo Server..."
gz sim -s -r ${PX4_GZ_WORLDS}/${PX4_GZ_WORLD}.sdf &
GZ_PID=$!

# 2. Wait for Gazebo process and clock
echo "Waiting for Gazebo simulation clock to become active..."
CLOCK_ACTIVE=0
for i in {1..60}; do
    # gz topic -e -t /clock -n 1 will exit once it receives 1 message
    if gz topic -e -t /clock -n 1 > /dev/null 2>&1; then
        CLOCK_ACTIVE=1
        break
    fi
    sleep 2
done

if [ $CLOCK_ACTIVE -eq 0 ]; then
    echo "Error: Timed out waiting for Gazebo simulation clock!"
    exit 1
fi
echo "Gazebo clock is active! Proceeding."

export PX4_GZ_STANDALONE=1
export PX4_PARAM_NAV_DLL_ACT=0
export PX4_SYS_AUTOSTART=4001
export PX4_SIMULATOR=gz

# Moderate physical speed enhancements for demonstration
export PX4_PARAM_MPC_XY_CRUISE=5.0
export PX4_PARAM_MPC_XY_VEL_MAX=8.0
export PX4_PARAM_MPC_JERK_AUTO=4.0

PX4_DIR="/home/capstone/PX4-Autopilot"
BUILD_DIR="${PX4_DIR}/build/px4_sitl_default"

rm -rf /tmp/px4_instance_0 /tmp/px4_instance_1
mkdir -p /tmp/px4_instance_0
mkdir -p /tmp/px4_instance_1

# Drone 0 Configuration
echo "Starting PX4 Instance 0..."
(
    cd /tmp/px4_instance_0
    export PX4_GZ_MODEL_POSE="$D0_X,$D0_Y,0.2,0,0,0"
    export PX4_UXRCE_DDS_NS="drone_0"
    ${BUILD_DIR}/bin/px4 -i 0 -d ${BUILD_DIR}/etc > out.log 2> err.log
) &

echo "Waiting 15 seconds for Drone 0 to initialize..."
sleep 15

# Drone 1 Configuration
echo "Starting PX4 Instance 1..."
(
    cd /tmp/px4_instance_1
    export PX4_GZ_MODEL_POSE="$D1_X,$D1_Y,0.2,0,0,0"
    export PX4_UXRCE_DDS_NS="drone_1"
    ${BUILD_DIR}/bin/px4 -i 1 -d ${BUILD_DIR}/etc > out.log 2> err.log
) &

echo "Waiting 15 seconds for Drone 1 to initialize..."
sleep 15

echo "=== EXPLICIT X500 EXISTENCE VALIDATION ==="
echo "Checking Gazebo models for x500_0 and x500_1:"
gz model --list | grep x500 || echo "NO X500 MODELS FOUND!"

echo "=== ROS 2 TELEMETRY VALIDATION ==="
source /opt/ros/jazzy/setup.bash
source /home/capstone/capstone_project_antigravity/drone_ws/install/setup.bash
echo "Checking drone_0 odometry topic:"
timeout 5 ros2 topic echo /drone_0/fmu/out/vehicle_odometry --once || echo "DRONE 0 ODOMETRY TIMEOUT!"
echo "Checking drone_1 odometry topic:"
timeout 5 ros2 topic echo /drone_1/fmu/out/vehicle_odometry --once || echo "DRONE 1 ODOMETRY TIMEOUT!"
echo "Validation complete."

echo "All instances started. Press Ctrl+C to stop."
sleep infinity
