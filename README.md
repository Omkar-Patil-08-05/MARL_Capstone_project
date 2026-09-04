# 🚁 Multi-Agent Reinforcement Learning for Drone Swarm Coordination in Search and Rescue using QMIX, ROS 2 Jazzy, PX4 and Gazebo Harmonic.

## 📌 PROJECT OVERVIEW

This project implements a multi-agent reinforcement learning (MARL) approach to drone swarm coordination for Search and Rescue (SAR) missions in disaster environments. The core intelligence is driven by a QMIX policy network integrated with a rule-based safety shield to guarantee collision-free flight dynamics.

**Control Pipeline:**
`QMIX` → `rule-based safety shield` → `ROS 2` → `PX4` → `Gazebo`

**Visual/Perception Pipeline:**
`RGB-D Camera` → `ROS 2` → `YOLO` → `Dashboard`

*Note: Visual perception (YOLO) is utilized for demonstration in the dashboard, but authoritative ground-truth mission evaluation relies on a 3.5m spatial proximity detector to guarantee evaluation integrity independent of computer vision artifacts.*

## 💻 HARDWARE / SOFTWARE
- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- Python 3.12
- RTX 3050 Laptop GPU 6GB
- Intel iGPU
- X11

## 📁 REPOSITORY STRUCTURE

- **`marl_drone_project/`**: Contains the core QMIX training framework and model checkpoints.
- **`worlds/`**: Gazebo Harmonic (`.sdf`) world definitions for the simulation.
- **`world_generator/`**: Python module for procedurally generating randomized disaster maps and victim placements.
- **`generated_worlds/`**: Output directory for procedurally generated world variants.
- **`assets_real/`**: 3D assets and meshes used in the simulation (e.g., Rescue Randy victims).
- **`gazebo_evaluation/`**: The authoritative raw data logs for QMIX evaluation.
- **`dashboard/`**: React/FastAPI real-time mission command center and visualization interface.
- **`scripts/`**: Shell scripts for orchestrating multi-drone launches, world generation, and testing.
- **`analysis/`**: Read-only Python module for synthesizing final evaluation plots and statistical summaries.
- **`results/`**: Detailed trajectory and decision logs from physical drone experiments.

## 🧠 QMIX

The system utilizes a **QMIX + rule-based safety shield** architecture. 
The frozen 6-agent QMIX checkpoint (`models/qmix_sar_v4_align_best.pth`) dictates optimal SAR search vectors, while the safety shield intercepts and sanitizes waypoint requests to prevent mid-air collisions before they reach the PX4 flight controller.

## 📊 EVALUATION

Authoritative evaluation was conducted across **10 physical simulation episodes** (Episodes 11–20) inside Gazebo:
- **Mean Coverage**: 90.08%
- **Victim Success**: 5/5 victims found in every episode
- **Invalid Actions**: 0
- **Timeouts**: 0
- **Mean Duration**: ~229.92 seconds

*(Note: Collision data was unavailable in the prototype evaluation logs. Collision-free performance is designed via the safety shield but cannot be statistically claimed from these logs).*

## 📈 SCALABILITY

The architecture was successfully scaled through multiple phases:
- **2 Drones**: Complete QMIX + PX4 validation.
- **3 Drones**: Hybrid deterministic/neural validation.
- **4 Drones**: Hybrid deterministic/neural validation.
- **5 Drones**: Skipped in favor of direct 6-drone integration.
- **6 Drones**: QMIX integration and scalability physically demonstrated via microXRCE. 

*Note: The 6-drone run represents integration scalability, not a completed SAR evaluation.*

## 🖥️ DASHBOARD

A real-time command center provides situational awareness:
- Dynamic victim count rendering
- Mission lifecycle control (START/STOP)
- Live telemetry streams
- 2D Drone Map & real-time Heatmap
- Victim tracking and detection logging
- Coordination panel
- RGB-D camera topic streams

## 🗄️ DATABASE

Mission data is persistently logged to SQLite at `dashboard/backend/results/antigravity.db`.
The persistence layer operates asynchronously (via an internal threaded queue) to capture experiments, missions, episodes, victims, detection events, and high-frequency telemetry without blocking the critical drone control loop.

## 🔬 ANALYSIS

A read-only analytical module (`analysis/final_analysis.py`) programmatically extracts performance metrics and trajectory visualizations from the raw physical logs to `analysis/outputs/`. It strictly prevents data fabrication or modification of source experimental logs.

## 🚀 HOW TO RUN

**Launch 6-Drone Lightweight Configuration:**
```bash
./scripts/launch_6_drone_lightweight.sh
```

**Launch the Dashboard (Backend & Frontend):**
```bash
# Start FastAPI backend (Port 8000)
cd dashboard/backend
python3 main.py

# Start React frontend (Port 3000)
cd dashboard/frontend
npm start
```

**Run Quantitative Analysis:**
```bash
python3 analysis/final_analysis.py
```
*(For other tests, refer to the `scripts/` directory).*

## 📋 CURRENT STATUS

**COMPLETED:**
- QMIX integration
- PX4 flight stack integration
- ROS 2 control integration
- Rule-based safety shield
- Mission Dashboard
- Telemetry Database
- Quantitative final analysis
- Six-drone integration

**NEXT:**
- Enhanced earthquake SAR world
- Final controlled demonstration
- Final evaluation where necessary
- Thesis
- Presentation
- Viva

## ⚠️ SCIENTIFIC LIMITATIONS

To maintain absolute academic integrity, the following limitations are explicitly stated:
1. Only 10 raw episodes are present in the authoritative CSV evaluation data (Episodes 11–20), despite earlier projections.
2. Explicit collision records are unavailable in the evaluation logs.
3. The 6-drone experiment (H8) validates scalability and system integration, but is not a completed full-duration SAR evaluation.
4. The 5-drone evaluation was bypassed.
5. Mission detections utilize 3.5m Ground-Truth spatial proximity, not visual YOLO classifications, to ensure objective evaluation untouched by rendering artifacts.
6. Real-world drone hardware deployment is not claimed; all physical logs are sourced from the Gazebo SITL simulator.