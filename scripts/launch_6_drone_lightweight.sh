#!/bin/bash
# Script to launch 6 PX4 drones in Gazebo Harmonic SITL WITHOUT RGB-D perception.
# Used for the 6-drone smoke test to avoid hardware starvation.
set -e
echo "LAUNCH SWARM LIGHTWEIGHT CALLED WITH ARGS: MAP_ID=$1 DRONE_COUNT=$2" >> /tmp/launch_swarm_lightweight_debug.log

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
pkill -9 -f "MicroXRCEAgent" || true
pkill -9 -f "px4" || true
pkill -9 -f "gz sim" || true
pkill -9 -f "ruby.*gz" || true
sleep 2

echo "Starting MicroXRCEAgent on UDP 8888..."
bash -c "source /opt/ros/jazzy/setup.bash && source /home/capstone/capstone_project_antigravity/drone_ws/install/setup.bash && MicroXRCEAgent udp4 -p 8888" &
AGENT_PID=$!
sleep 2

# Map parameterization
MAP_ID=${1:-realistic_sar}
DRONE_COUNT=${2:-6}

if [ "$DRONE_COUNT" -lt 1 ] || [ "$DRONE_COUNT" -gt 6 ]; then
    echo "Error: Unsupported drone count: $DRONE_COUNT. Must be 1-6."
    exit 1
fi

WORKSPACE_DIR="/home/capstone/capstone_project_antigravity"

if [ "$MAP_ID" == "realistic_sar" ]; then
    META_FILE="$WORKSPACE_DIR/worlds/generated_world_meta.json"
    export PX4_GZ_WORLD="realistic_sar"
elif [ "$MAP_ID" == "earthquake_world" ]; then
    META_FILE="$WORKSPACE_DIR/worlds/earthquake_world_meta.json"
    export PX4_GZ_WORLD="earthquake_world"
else
    echo "Error: Unsupported map ID: $MAP_ID"
    exit 1
fi

if [ ! -f "$META_FILE" ]; then
    echo "Error: Metadata file not found at $META_FILE"
    exit 1
fi

# Extract spawn coordinates dynamically
declare -a DRONE_X
declare -a DRONE_Y
for ((i=0; i<$DRONE_COUNT; i++)); do
    X=$(python3 -c "import json; print(json.load(open('$META_FILE'))['drone_base']['spawns'][$i]['x'])")
    Y=$(python3 -c "import json; print(json.load(open('$META_FILE'))['drone_base']['spawns'][$i]['y'])")
    DRONE_X[$i]=$X
    DRONE_Y[$i]=$Y
    echo "Drone $i spawn: ($X, $Y)"
done

# Setup PX4 Gazebo environment
source /home/capstone/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh
export PX4_GZ_WORLDS="/home/capstone/capstone_project_antigravity/worlds"
export GZ_SIM_RESOURCE_PATH="/home/capstone/capstone_project_antigravity/models:/home/capstone/PX4-Autopilot/Tools/simulation/gz/models:/home/capstone/PX4-Autopilot/Tools/simulation/gz/worlds:/home/capstone/capstone_project_antigravity/assets_real/victims"

# 1. Start Gazebo Server (headless by default)
echo "Starting Gazebo Server with headless rendering..."
gz sim -s -v 4 -r ${PX4_GZ_WORLDS}/${PX4_GZ_WORLD}.sdf > /tmp/gz.log 2>&1 &
GZ_PID=$!

# 2. Wait for stabilization
echo "Waiting 10s for simulation to stabilize..."
sleep 10
echo "Simulator stack successfully launched!"

# 3. Wait for Gazebo process and clock
echo "Waiting for Gazebo simulation clock to become active..."
CLOCK_ACTIVE=0
for i in {1..60}; do
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
export PX4_PARAM_MPC_XY_CRUISE=5.0
export PX4_PARAM_MPC_XY_VEL_MAX=8.0
export PX4_PARAM_MPC_JERK_AUTO=4.0

export PX4_GZ_MODELS="/home/capstone/capstone_project_antigravity/models"

PX4_DIR="/home/capstone/PX4-Autopilot"
BUILD_DIR="${PX4_DIR}/build/px4_sitl_default"

for ((i=0; i<$DRONE_COUNT; i++)); do
    rm -rf /tmp/px4_instance_$i
    mkdir -p /tmp/px4_instance_$i
done

# Launch PX4 instances concurrently
for ((i=0; i<$DRONE_COUNT; i++)); do
    echo "Starting PX4 Instance $i..."
    (
        cd /tmp/px4_instance_$i
        # LIGHTWEIGHT: Use standard x500 without RGB-D camera
        export PX4_GZ_MODEL="x500"
        export PX4_GZ_MODEL_POSE="${DRONE_X[$i]},${DRONE_Y[$i]},0.2,0,0,0"
        export PX4_UXRCE_DDS_NS="drone_$i"
        ${BUILD_DIR}/bin/px4 -i $i -d ${BUILD_DIR}/etc > out.log 2> err.log
    ) &
done

echo "=== ROS 2 TELEMETRY VALIDATION ==="
source /opt/ros/jazzy/setup.bash
source /home/capstone/capstone_project_antigravity/drone_ws/install/setup.bash

for ((i=0; i<$DRONE_COUNT; i++)); do
    echo "Checking drone_$i odometry topic:"
    timeout 5 ros2 topic echo /drone_$i/fmu/out/vehicle_odometry --once || echo "DRONE $i ODOMETRY TIMEOUT!"
done
echo "Validation complete."

echo "All instances started. RGB-D perception disabled for lightweight mode. Press Ctrl+C to stop."
sleep infinity
