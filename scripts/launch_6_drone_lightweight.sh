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
    pkill -f "parameter_bridge" || true
    pkill -f "MicroXRCEAgent" || true
    pkill -f "px4" || true
    pkill -9 -f "gz sim" || true
    pkill -9 -f "ruby.*gz" || true
    echo "Cleanup complete."
}
trap cleanup EXIT INT TERM

# Ensure clean state before starting
pkill -f "parameter_bridge" || true
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
elif [ "$MAP_ID" == "forest_sar" ]; then
    META_FILE="$WORKSPACE_DIR/worlds/forest_sar_meta.json"
    export PX4_GZ_WORLD="forest_sar"
elif [ "$MAP_ID" == "forest_dense_candidate" ]; then
    META_FILE="$WORKSPACE_DIR/worlds/forest_dense_candidate_meta.json"
    export PX4_GZ_WORLD="forest_dense_candidate"
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
declare -a DRONE_Z
for ((i=0; i<$DRONE_COUNT; i++)); do
    X=$(python3 -c "import json; print(json.load(open('$META_FILE'))['drone_base']['spawns'][$i]['x'])")
    Y=$(python3 -c "import json; print(json.load(open('$META_FILE'))['drone_base']['spawns'][$i]['y'])")
    Z=$(python3 -c "import json; print(json.load(open('$META_FILE'))['drone_base']['spawns'][$i].get('z', 0.2))")
    DRONE_X[$i]=$X
    DRONE_Y[$i]=$Y
    DRONE_Z[$i]=$Z
    echo "Drone $i spawn: ($X, $Y, $Z)"
done

# Setup PX4 Gazebo environment
source /home/capstone/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh
export PX4_GZ_WORLDS="/home/capstone/capstone_project_antigravity/worlds"
export GZ_SIM_RESOURCE_PATH="/home/capstone/capstone_project_antigravity/models:/home/capstone/capstone_project_antigravity/assets_real:/home/capstone/capstone_project_antigravity/assets_real/victims:/home/capstone/PX4-Autopilot/Tools/simulation/gz/models:/home/capstone/PX4-Autopilot/Tools/simulation/gz/worlds:/home/capstone/capstone_project_antigravity/generated_worlds:/home/capstone/capstone_project_antigravity/worlds"

# 1. Start Gazebo Server (headless by default)
echo "Starting Gazebo Server with headless rendering..."
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
gz sim -s -v 4 -r --headless-rendering ${PX4_GZ_WORLDS}/${PX4_GZ_WORLD}.sdf > /tmp/gz.log 2>&1 &
GZ_PID=$!

# 2. Wait for stabilization
echo "Waiting 3s for simulation to stabilize..."
sleep 3
echo "Simulator stack successfully launched!"

# 3. Wait for Gazebo process and clock
echo "Waiting for Gazebo simulation clock to become active..."
CLOCK_ACTIVE=0
for i in {1..60}; do
    if gz topic -e -t /clock -n 1 > /dev/null 2>&1; then
        CLOCK_ACTIVE=1
        break
    fi
    sleep 1
done

if [ $CLOCK_ACTIVE -eq 0 ]; then
    echo "Error: Timed out waiting for Gazebo simulation clock!"
    exit 1
fi
echo "Gazebo clock is active! Proceeding."

# Bridge simulation clock to ROS 2 for use_sim_time nodes
echo "Starting ROS-Gazebo Bridge for /clock..."
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock > /tmp/bridge.log 2>&1 &
sleep 1

export PX4_GZ_STANDALONE=1
export PX4_PARAM_NAV_DLL_ACT=0
export PX4_SYS_AUTOSTART=4001
export PX4_SIMULATOR=gz

# High-Speed Velocity Profiling:
# Forest SAR 8m cells require rapid acceleration and high cruise to match city agility
if [ "$MAP_ID" == "forest_sar" ] || [ "$MAP_ID" == "forest_dense_candidate" ]; then
    export PX4_PARAM_MPC_XY_CRUISE=16.0
    export PX4_PARAM_MPC_XY_VEL_MAX=20.0
    export PX4_PARAM_MPC_ACC_HOR=8.0
    export PX4_PARAM_MPC_JERK_AUTO=24.0
    export PX4_PARAM_MPC_Z_VEL_MAX_UP=4.0
else
    export PX4_PARAM_MPC_XY_CRUISE=8.0
    export PX4_PARAM_MPC_XY_VEL_MAX=12.0
    export PX4_PARAM_MPC_ACC_HOR=4.0
    export PX4_PARAM_MPC_JERK_AUTO=8.0
    export PX4_PARAM_MPC_Z_VEL_MAX_UP=3.0
fi

export PX4_GZ_MODELS="/home/capstone/capstone_project_antigravity/models"

PX4_DIR="/home/capstone/PX4-Autopilot"
BUILD_DIR="${PX4_DIR}/build/px4_sitl_default"

for ((i=0; i<$DRONE_COUNT; i++)); do
    rm -rf /tmp/px4_instance_$i
    mkdir -p /tmp/px4_instance_$i
done

# Launch PX4 instances with stagger for clean EKF2 initialization
for ((i=0; i<$DRONE_COUNT; i++)); do
    echo "Starting PX4 Instance $i..."
    (
        cd /tmp/px4_instance_$i
        export PX4_GZ_WORLD="$PX4_GZ_WORLD"
        # LIGHTWEIGHT: Use standard x500 without RGB-D camera
        export PX4_GZ_MODEL="x500"
        export PX4_GZ_MODEL_POSE="${DRONE_X[$i]},${DRONE_Y[$i]},${DRONE_Z[$i]},0,0,0"
        export PX4_UXRCE_DDS_NS="drone_$i"
        ${BUILD_DIR}/bin/px4 -i $i -d ${BUILD_DIR}/etc > out.log 2> err.log
    ) &
    if [ $i -lt $((DRONE_COUNT - 1)) ]; then
        echo "Waiting 3s before launching next PX4 instance..."
        sleep 3
    fi
done

echo "All $DRONE_COUNT drone instances launched in high-speed mode!"
echo "Readiness verified. RGB-D perception disabled for lightweight high-speed mode."

echo "All instances started. RGB-D perception disabled for lightweight mode. Press Ctrl+C to stop."
sleep infinity
