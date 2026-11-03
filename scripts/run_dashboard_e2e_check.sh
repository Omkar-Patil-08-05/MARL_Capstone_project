#!/usr/bin/env bash
set -e

PROJECT_ROOT="/home/capstone/capstone_project_antigravity"
cd "$PROJECT_ROOT"

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PROJECT_ROOT/drone_ws/src/swarm_controller:$PROJECT_ROOT/dashboard/backend:$PYTHONPATH"

BACKEND_PID=""
FRONTEND_PID=""

cleanup() {
    echo "[RUNNER] Cleaning up background processes..."
    if [ -n "$FRONTEND_PID" ]; then
        kill "$FRONTEND_PID" 2>/dev/null || true
    fi
    if [ -n "$BACKEND_PID" ]; then
        kill "$BACKEND_PID" 2>/dev/null || true
    fi
    # Also kill any leftover processes on 8000 or 5173
    fuser -k 8000/tcp 2>/dev/null || true
    fuser -k 5173/tcp 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[RUNNER] Starting backend on port 8000..."
cd "$PROJECT_ROOT/dashboard/backend"
python3 -m uvicorn main:app --host 0.0.0.0 --port 8000 > /tmp/dashboard_backend.log 2>&1 &
BACKEND_PID=$!

echo "[RUNNER] Waiting for backend to become healthy..."
for i in {1..30}; do
    if curl -s http://localhost:8000/api/health >/dev/null 2>&1; then
        echo "[RUNNER] Backend is healthy!"
        break
    fi
    sleep 0.5
done

echo "[RUNNER] Starting frontend on port 5173..."
cd "$PROJECT_ROOT/dashboard/frontend"
npm run dev -- --host 0.0.0.0 --port 5173 > /tmp/dashboard_frontend.log 2>&1 &
FRONTEND_PID=$!

echo "[RUNNER] Waiting for frontend to become available..."
for i in {1..30}; do
    if curl -s http://localhost:5173 >/dev/null 2>&1; then
        echo "[RUNNER] Frontend is available!"
        break
    fi
    sleep 0.5
done

cd "$PROJECT_ROOT/dashboard/frontend"
echo "[RUNNER] Running Puppeteer UI verification..."
node verify_ui.js "$PROJECT_ROOT/results/dashboard_demo_polish_verification.png"

echo "[RUNNER] All E2E checks completed successfully!"
