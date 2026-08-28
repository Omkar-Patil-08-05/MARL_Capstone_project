#!/bin/bash
set -e

cd "$(dirname "$0")/.."
PROJECT_ROOT=$(pwd)

if pgrep -f "gz sim" > /dev/null; then
    echo "=================================================="
    echo "WARNING: A live mission is currently running!"
    echo "Stop the mission before opening a standalone Gazebo environment."
    echo "=================================================="
    exit 1
fi

if [ "$1" == "small" ]; then
    WORLD="realistic_sar"
    echo "Opening SMALL validated SAR environment..."
elif [ "$1" == "large" ]; then
    WORLD="earthquake_world"
    echo "Opening LARGE SAR environment..."
else
    echo "Usage: bash scripts/view_environment.sh [small|large]"
    exit 1
fi

export GZ_SIM_RESOURCE_PATH="${PROJECT_ROOT}/models:${PROJECT_ROOT}/assets_real/victims:/home/capstone/PX4-Autopilot/Tools/simulation/gz/models:/home/capstone/PX4-Autopilot/Tools/simulation/gz/worlds"

gz sim ${PROJECT_ROOT}/worlds/${WORLD}.sdf
