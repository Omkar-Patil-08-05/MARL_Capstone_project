#!/usr/bin/env bash
set -e

PROJECT_ROOT="/home/capstone/capstone_project_antigravity"
cd "$PROJECT_ROOT"

source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PROJECT_ROOT/drone_ws/src/swarm_controller:$PROJECT_ROOT/dashboard/backend:$PYTHONPATH"

BACKEND_PID=""
FRONTEND_PID=""
TELEMETRY_PID=""

cleanup() {
    echo "[RUNNER] Cleaning up background processes..."
    if [ -n "$TELEMETRY_PID" ]; then
        kill "$TELEMETRY_PID" 2>/dev/null || true
    fi
    if [ -n "$FRONTEND_PID" ]; then
        kill "$FRONTEND_PID" 2>/dev/null || true
    fi
    if [ -n "$BACKEND_PID" ]; then
        kill "$BACKEND_PID" 2>/dev/null || true
    fi
    fuser -k 8000/tcp 2>/dev/null || true
    fuser -k 5173/tcp 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[RUNNER] Starting backend on port 8000..."
cd "$PROJECT_ROOT/dashboard/backend"
python3 -m uvicorn main:app --host 0.0.0.0 --port 8000 > /tmp/live_backend.log 2>&1 &
BACKEND_PID=$!

for i in {1..30}; do
    if curl -s http://localhost:8000/api/health >/dev/null 2>&1; then
        echo "[RUNNER] Backend is healthy!"
        break
    fi
    sleep 0.5
done

echo "[RUNNER] Starting frontend on port 5173..."
cd "$PROJECT_ROOT/dashboard/frontend"
npm run dev -- --host 0.0.0.0 --port 5173 > /tmp/live_frontend.log 2>&1 &
FRONTEND_PID=$!

for i in {1..30}; do
    if curl -s http://localhost:5173 >/dev/null 2>&1; then
        echo "[RUNNER] Frontend is available!"
        break
    fi
    sleep 0.5
done

NUM_DRONES="${1:-6}"
if [ "$NUM_DRONES" == "2" ]; then
    OUT_PNG="$PROJECT_ROOT/results/dashboard_two_drone_polished.png"
else
    OUT_PNG="$PROJECT_ROOT/results/dashboard_six_drone_polished.png"
fi

echo "[RUNNER] Streaming $NUM_DRONES-drone telemetry in background..."
cd "$PROJECT_ROOT"
python3 scripts/test_live_dashboard_render.py --drones "$NUM_DRONES" &
TELEMETRY_PID=$!

sleep 2

echo "[RUNNER] Running Puppeteer live verification for $NUM_DRONES drones..."
cd "$PROJECT_ROOT/dashboard/frontend"
node verify_live_render.js "$OUT_PNG" "$NUM_DRONES"

echo "[RUNNER] All live checks completed!"
