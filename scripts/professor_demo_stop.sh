#!/bin/bash
echo "Stopping all Professor Demo processes..."
# Stop active mission via backend API if reachable
curl -s -m 2 -X POST http://localhost:8000/api/mission/stop >/dev/null 2>&1 || true
pkill -f "uvicorn" || true
pkill -f "vite" || true
pkill -9 -f "MicroXRCEAgent" || true
pkill -9 -f "px4" || true
pkill -9 -f "gz sim" || true
pkill -9 -f "ruby.*gz" || true
if [ -f /tmp/run_demo_controller.pid ]; then
    OLD_PID=$(cat /tmp/run_demo_controller.pid 2>/dev/null || true)
    if [ -n "$OLD_PID" ] && kill -0 $OLD_PID 2>/dev/null; then
        pkill -TERM -P $OLD_PID 2>/dev/null || true
        kill -TERM $OLD_PID 2>/dev/null || true
    fi
    rm -f /tmp/run_demo_controller.pid
fi
echo "Cleanup complete."
