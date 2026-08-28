#!/bin/bash
set -e

# Change to project root
cd "$(dirname "$0")/.."
PROJECT_ROOT=$(pwd)

echo "=================================================="
echo "    MARL SWARM DEMONSTRATION CONTROL SYSTEM       "
echo "=================================================="
echo "Initializing..."

bash scripts/professor_demo_stop.sh

echo "[1/2] Starting FastAPI Backend..."
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

echo "[2/2] Starting Vite React Frontend..."
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

echo "=================================================="
echo "               SYSTEM READY!                      "
echo "=================================================="
echo "Open your browser to: http://localhost:5173"
echo "Press Ctrl+C in this terminal to stop all systems."
echo "=================================================="

cleanup() {
    echo -e "\nShutting down demonstration system..."
    kill $BACKEND_PID 2>/dev/null || true
    kill $FRONTEND_PID 2>/dev/null || true
    bash $PROJECT_ROOT/scripts/professor_demo_stop.sh
    exit 0
}

trap cleanup EXIT INT TERM
sleep infinity
