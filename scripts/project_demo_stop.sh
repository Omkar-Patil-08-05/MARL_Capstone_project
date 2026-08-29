#!/bin/bash
set -e

echo "=================================================="
echo "    STOPPING MARL SWARM PROJECT DEMONSTRATION     "
echo "=================================================="

# Kill backend and frontend
pkill -f "uvicorn main:app" || true
pkill -f "vite" || true
pkill -f "node.*vite" || true

# Kill simulation stack
pkill -f "MicroXRCEAgent" || true
pkill -f "px4" || true
pkill -9 -f "gz sim" || true
pkill -9 -f "ruby.*gz" || true

# Kill ROS 2 and control nodes
pkill -f "parameter_bridge" || true
pkill -f "yolo_human" || true
pkill -f "qmix_drone" || true
pkill -f "ros2" || true

# Just to be safe, kill any background sleep commands from the launcher
pkill -f "sleep infinity" || true

echo "=================================================="
echo "               ALL PROCESSES STOPPED              "
echo "=================================================="
