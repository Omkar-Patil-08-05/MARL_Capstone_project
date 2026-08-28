#!/bin/bash
echo "Stopping all Professor Demo processes..."
pkill -f "uvicorn" || true
pkill -f "vite" || true
pkill -9 -f "MicroXRCEAgent" || true
pkill -9 -f "px4" || true
pkill -9 -f "gz sim" || true
pkill -9 -f "ruby.*gz" || true
pkill -9 -f "qmix_drone_test" || true
echo "Cleanup complete."
