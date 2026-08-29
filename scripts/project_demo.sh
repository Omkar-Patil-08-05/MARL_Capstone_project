#!/bin/bash
set -e

# Change to project root
cd "$(dirname "$0")/.."
PROJECT_ROOT=$(pwd)

echo "=================================================="
echo "    MARL SWARM PROJECT DEMONSTRATION CONTROL      "
echo "=================================================="
echo "Initializing..."

# Stop any existing processes cleanly
bash scripts/project_demo_stop.sh
sleep 2

echo "[1/3] Starting FastAPI Backend..."
cd $PROJECT_ROOT/dashboard/backend
source /opt/ros/jazzy/setup.bash
uvicorn main:app --host 0.0.0.0 --port 8000 > /tmp/fastapi_demo.log 2>&1 &
BACKEND_PID=$!

echo "Waiting for backend health check..."
for i in {1..30}; do
    if curl -s http://localhost:8000/api/health | grep -q '"status":"ok"'; then
        break
    fi
    sleep 1
done

echo "[2/3] Starting Vite React Frontend..."
cd $PROJECT_ROOT/dashboard/frontend
npm run dev > /tmp/vite_demo.log 2>&1 &
FRONTEND_PID=$!

echo "Waiting for frontend..."
for i in {1..30}; do
    if curl -s http://localhost:5173 > /dev/null; then
        break
    fi
    sleep 1
done

echo "[3/3] Starting Simulation & Mission Pipeline (HEADLESS)..."
cd $PROJECT_ROOT
echo "[3/3] System Ready! Waiting for mission start from dashboard..."

echo "=================================================="
echo "               SYSTEM READY!                      "
echo "=================================================="
echo "Open your browser to: http://localhost:5173"
echo "To view Gazebo visually, click 'VIEW SIMULATION' in the dashboard."
echo "Press Ctrl+C in this terminal to stop all systems."
echo "=================================================="

cleanup() {
    echo -e "\nShutting down demonstration system..."
    bash $PROJECT_ROOT/scripts/project_demo_stop.sh
    exit 0
}

trap cleanup EXIT INT TERM
sleep infinity
