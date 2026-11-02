# MARL CAPSTONE: FINAL LIVE DEMONSTRATION CHECKLIST & RUNBOOK

This operational runbook provides a step-by-step verification checklist for conducting a live flight demonstration of the QMIX drone swarm in Gazebo Harmonic, or presenting the historical physical evaluation dataset to an examination panel.

---

## 1. Important Distinction: Live Demonstration vs. Authoritative Benchmark

> [!IMPORTANT]
> - **Authoritative Historical Dataset (`antigravity.db`):** Contains the official, 10-episode benchmark (Episodes 11–20) recorded with a 6-drone swarm, demonstrating **90.08% stable coverage** and 100% target recovery.
> - **Multi-Seed Benchmark (`results/final_research_package/`):** Contains the statistical comparison across 10 deterministic seeds, demonstrating QMIX's superiority over Frontier Search (**94.03% vs 88.09% coverage**, **93.3% vs 80.0% detection**).
> - **Live Physics Demonstration:** A real-time, online simulation run intended to visually showcase offboard PX4 flight, spatial waypoint tracking, live camera perception, and real-time telemetry streaming in the web dashboard.

---

## 2. Pre-Flight System & Dependency Check

Run these commands in a bash terminal to verify system readiness:

```bash
# 1. Source ROS 2 Jazzy workspace
source /opt/ros/jazzy/setup.bash
source ~/capstone_project_antigravity/drone_ws/install/setup.bash

# 2. Check NVIDIA GPU status
nvidia-smi

# 3. Check for lingering Gazebo, PX4, or DDS processes (must be empty before start)
pgrep -fl "gz sim|px4|micro_xrce_dds" || echo "System clean. Ready for demo."

# 4. Verify baseline world checksums
python3 scripts/validate_forest_integration.py
```

---

## 3. Step-by-Step Live Flight Execution Procedure

### Step 1: Start Dashboard Services
1. **Backend Server (FastAPI / Uvicorn):**
   ```bash
   cd ~/capstone_project_antigravity/dashboard/backend
   source /opt/ros/jazzy/setup.bash
   source ~/capstone_project_antigravity/drone_ws/install/setup.bash
   uvicorn main:app --host 0.0.0.0 --port 8000
   ```
   *Expected output:* `Application startup complete. Uvicorn running on http://0.0.0.0:8000`.

2. **Frontend UI (Vite / React):**
   ```bash
   cd ~/capstone_project_antigravity/dashboard/frontend
   npm run dev
   ```
   *Access:* Open browser at `http://localhost:5173`.

---

### Step 2: Initialize & Configure Mission in Dashboard
1. Open the web dashboard at `http://localhost:5173`.
2. In the **Map Selection** panel:
   - Select **Realistic SAR** (`realistic_sar`) for the urban disaster environment, OR **Forest SAR** (`forest_sar`) for wilderness search.
3. In the **Swarm Configuration** panel:
   - Select **2 Drones** (recommended for laptop GPU live demonstration).
   - Set **Victim Count:** `3`.
   - Set **Random Seed:** `555` (for reproducible target placement).
   - Set **Max Decisions Budget:** `150`.
4. Click **Start Mission**.

---

### Step 3: Verification of Automated Startup Lifecycle
Watch the dashboard header status and backend logs:
1. `Transition -> SIMULATOR_READY`:
   - Headless Gazebo Harmonic starts (`worlds/realistic_sar.sdf`).
   - Micro-XRCE-DDS agent establishes client participant connections.
2. `Transition -> QMIX_STARTING`:
   - PX4 SITL instances 0 and 1 boot and establish telemetry lock.
   - `px4_interface` establishes offboard position setpoint streaming at 20 Hz.
3. `Transition -> RUNNING`:
   - Drones transition from `TAKEOFF` to `HOLD` at $15.0\text{ m}$ cruise altitude.
   - QMIX begins inference, dispatching discrete spatial waypoint actions.

---

### Step 4: Live Telemetry & ROS 2 Topic Validation
In a separate terminal, confirm live topic health:
```bash
# 1. Verify telemetry stream (should publish at ~10 Hz)
ros2 topic hz /swarm/telemetry

# 2. Inspect active drone positions
ros2 topic echo /swarm/telemetry --once

# 3. Check simulation clock synchronization
ros2 topic echo /clock --once
```

---

### Step 5: Live Victim Interception & SQLite Persistence
1. As drones sweep their respective quadrants, watch for victim detection events:
   - In UI: Undetected victim markers turn green (`DETECTED`).
   - In Backend Log: Look for `[drone_0] MISSION DETECTION: victim_22 at distance 3.2m`.
2. Confirm live SQLite insertion via API:
   ```bash
   curl -s http://localhost:8000/api/db/live | python3 -m json.tool
   ```

---

### Step 6: Clean Mission Stop & Graceful Teardown
1. When search coverage reaches saturation ($>85\%$) or all victims are located, click **Stop Mission** in the dashboard, or call:
   ```bash
   curl -s -X POST http://localhost:8000/api/mission/stop
   ```
2. The backend gracefully commands vehicle disarm, terminates Gazebo and PX4 process groups, stops the ROS 2 daemon, and sets state to `STOPPED`.

---

## 4. Fallback Plan (Contingency for Presentation)

If a GPU memory spike or simulator timeout occurs during a live presentation:

### Contingency A: Present Authoritative Historical Batch
1. Open the dashboard at `http://localhost:5173`.
2. Navigate to the **Database Viewer** tab.
3. Select **Authoritative QMIX Evaluation (Episodes 11–20)**.
4. Highlight the persisted metrics:
   - **Mean Coverage:** $90.08\%$
   - **Victim Success:** $5.0 / 5.0$
   - **Invalid Actions:** $0$
   - **Timeouts:** $0$

### Contingency B: Present High-Resolution Multi-Seed Figures
Open the generated publication figures in `results/final_research_package/figures/`:
1. `fig1_mean_coverage_comparison.png`: Highlights QMIX's $94.03\%$ mean coverage vs Frontier's $88.09\%$ and Random's $31.91\%$.
2. `fig3_per_seed_city_coverage.png`: Proves QMIX stability across 10 independent seeds compared to Frontier's variance collapses ($50.93\%$ in Seed 108).
3. `fig5_forest_coverage_comparison.png`: Proves QMIX generalization in wilderness terrain ($80.57\%$ vs $36.26\%$).

---

## 5. Pre-Demo Checklist Sign-Off
- [ ] Checkpoint `models/qmix_sar_v4_align_best.pth` verified intact (`f74edd5...`).
- [ ] Protected world baselines verified (all 6 MD5 checksums passed).
- [ ] Backend (`:8000`) and Frontend (`:5173`) running.
- [ ] Ports 8000, 5173, and 8888 free from zombie processes.
- [ ] Fallback slides and figures verified and ready to display.
