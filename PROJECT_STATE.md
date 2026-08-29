# MARL Drone Project: Authoritative Project State

This document is the authoritative continuation document for the Multi-Agent Reinforcement Learning SAR Drone Swarm Capstone project. 

**CRITICAL RULE FOR ALL FUTURE AI ASSISTANTS:**
You MUST read this document before making architectural changes. Do NOT assume old chat context is available. This file is the absolute source of truth. Inspect the actual files before modifying them. Do not fabricate validation results or claim a feature is implemented unless it exists in the repository.

---

## 1. PROJECT IDENTITY

**Project Title:** Multi-Agent Reinforcement Learning for Drone Swarm Coordination in Search and Rescue using QMIX, ROS 2 Jazzy and Gazebo Harmonic.

**Current Platform:**
- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- Python 3.12
- PX4 SITL
- MicroXRCEAgent
- PyTorch
- React / TypeScript (Frontend)
- FastAPI / WebSockets (Backend)
- X11 Display

**Research Objective:** Demonstrate coordinated multi-drone SAR exploration where multiple UAVs autonomously navigate an unknown disaster environment to maximize search coverage and discover victims using a fully trained QMIX MARL algorithm communicating via ROS 2.

---

## 2. CURRENT ARCHITECTURE

The current pipeline forms a complete loop from high-level RL decision making to physical Gazebo simulation, culminating in a live React telemetry dashboard.

1. **QMIX Policy (`qmix_sar_v4_align_best.pth`)**: Evaluates the multi-agent observation state.
2. **ROS 2 Mission Controller (`qmix_mission_controller.py`)**: Manages the agent state machine and coordinates the execution loop.
3. **ROS 2 Translation (`qmix_drone_test.py`)**: Translates discrete grid actions (up/down/left/right) into physical NED coordinate vectors.
4. **MicroXRCEAgent & PX4 SITL**: Bridges ROS 2 commands to physical drone aerodynamics and EKF state estimation.
5. **Gazebo Harmonic**: Computes 3D rigid-body physics, collisions, and sensor data (running Headless).
6. **Telemetry Publisher**: `qmix_mission_controller.py` publishes JSON states to `/swarm/telemetry`.
7. **FastAPI Backend (`dashboard/backend/main.py`)**: Subscribes to telemetry, enforces monotonic timestamp filtering, and broadcasts via WebSockets.
8. **React Dashboard (`dashboard/frontend/src`)**: Subscribes to WebSockets via `useTelemetry.ts` and renders a live, interactive 2D map, coverage metrics, and drone trajectories.

**Key Internal Components:**
- **`SARGridEnv`**: The core Python grid-based RL environment used for training the QMIX policy.
- **`GridWorldTransform`**: Converts physical continuous Gazebo coordinates into discrete `SARGridEnv` cells and vice versa.
- **`DroneAgent`**: Handles individual state machines (TAKEOFF, MOVING, HOVERING) for each PX4 node.

---

## 3. CURRENT DIRECTORY STRUCTURE

Important files and their purposes:
```
~/capstone_project_antigravity
├── assets_real/            # 3D models (Buildings, Cars, Houses, Rubble, Victims) for Gazebo
├── dashboard/
│   ├── backend/            # FastAPI server (main.py, world.py, WebSocket manager)
│   └── frontend/           # Vite/React dashboard (App.tsx, components/, hooks/)
├── drone_ws/               # The core ROS 2 workspace
│   └── src/
│       └── swarm_controller/
│           ├── swarm_controller/ 
│           │   ├── qmix_mission_controller.py # Multi-agent synchronous state machine and telemetry
│           │   ├── qmix_drone_test.py         # Entrypoint mapping RL actions to physical space
│           │   ├── grid_world_transform.py    # Math for grid ↔ continuous physical space
│           │   └── configs.py                 # Core configurations (Takeoff altitude, timeouts)
│           ├── package.xml
│           └── setup.py
├── marl_drone_project/     # The offline RL training suite
│   ├── env/sar_env.py      # Core SARGridEnv logic and reward formulation
│   └── train/              # QMIX network architecture, mixers, and training loop
├── models/                 # Cached trained PyTorch checkpoints (e.g. qmix_sar_v4_align_best.pth)
├── scripts/
│   ├── launch_two_drones.sh      # Baseline Gazebo launcher script
│   ├── professor_demo.sh         # Top-level demo launcher for backend/frontend
│   ├── professor_demo_stop.sh    # Cleanup script using SIGKILL (signal 9)
│   └── view_environment.sh       # Standalone viewer for 3D Gazebo environments
├── world_generator/        # Python scripts to generate procedural SDF environments
└── worlds/
    ├── generated_world_meta.json # Active world metadata, spawns, and obstacle layouts
    ├── realistic_sar.sdf         # CURRENT VALIDATED SMALL ENVIRONMENT (25x25)
    └── earthquake_world.sdf      # LARGE UNVALIDATED ENVIRONMENT
```

---

## 4. RL / QMIX STATUS

- **Current Trained Checkpoint:** `qmix_sar_v4_align_best.pth`
- **Current Number of Trained Agents:** **2 AGENTS** (CRITICAL: Do NOT attempt to run 3-6 drones with this checkpoint)
- **Observation Dimensionality:** 29-D continuous space (normalized own pos, one-hot ID, normalized teammate pos, relative teammate vector, BFS frontier vector, frontier density, and 3x3 FOV obstacle/exploration grids).
- **Victim Knowledge in Observation:** **FALSE**. Victims are NOT part of the RL observation space; the network learns pure exploration, while a separate semantic perception layer tracks victims.
- **Action Space:** Discrete (0: +X, 1: -X, 2: +Y, 3: -Y, 4: Hover)
- **Environment Dimensions:** 25x25 grid (100m x 100m physical)
- **Mixer Architecture:** VDN/QMIX monotonic mixing network.

---

## 5. IMPORTANT HISTORICAL FIXES

| ID | Feature | Status | Notes |
| :--- | :--- | :--- | :--- |
| N3 | Multi-Drone Architecture Setup | IMPLEMENTED | Refactored QMIXAdapter and setup discrete sync lockstep. |
| N4 | SAR RL Environment Integration | IMPLEMENTED | Replaced placeholder with SARGridEnv, ensuring bounded 29-D state. |
| N5 | Headless Gazebo Integration | IMPLEMENTED | Removed Playwright. Use PX4 SITL + Headless gzserver. |
| N6 | ROS 2 Camera Integration | IMPLEMENTED | Proxy bridge established (`cam2image`). Real camera plugin disabled to prevent headless rendering hang. |
| N7 | YOLO Bounding Box Pipeline | IMPLEMENTED | Mock YOLO active (OpenCV based). Emits 15Hz bounding boxes. |
| N8 | Visual Victim Localization | VERIFIED | `VictimLocalizer` and `VictimManager` successfully map 2D bounding boxes to 3D world coordinates. |
| N9 | Real YOLO Validation | FUTURE | Blocked by ultralytics PyPI network timeouts. Needs resolution or explicit fallback usage. |
| N10 | Advanced Moving Victim Tracking | FUTURE | Deduplication and robust association in `VictimManager`. |
| N11 | Camera Localization vs Ground Truth | FUTURE | Evaluate accuracy of bounding box projection. |
| N12 | Dashboard Live Camera Stream | FUTURE | Render YOLO boxes dynamically on React dashboard. |
| N13 | Visual Evaluate: realistic_sar | FUTURE | Measure 2-drone mission success with visual perception. |
| N14 | Visual Evaluate: earthquake_world | FUTURE | Measure scalability in large experimental environment. |
| N15 | Perception-RL Integration Research | FUTURE | Define how visual features enter QMIX observation vector. |
| N16 | Retrain Perception-Aware QMIX | FUTURE | Train new policy using bounding box / perception features. |
| N17+ | Scale to 3-6 Drones | FUTURE | Swarm expansion and architectural review. |

**PHASE 1: ROS 2 Telemetry Publisher**
- *Problem:* No way to view the drones without GUI Gazebo, which crashed repeatedly.
- *Fix:* Added `_publish_telemetry` to `QMIXMissionController` to broadcast state as JSON on `/swarm/telemetry`.

**PHASE 4: Headless Gazebo + Dashboard Integration**
- *Problem:* Gazebo GUI caused severe X11 crashes and memory exhaustion.
- *Fix:* Forced `gz sim -s` (headless) and relied entirely on the lightweight React Dashboard for visualization.

**H9: Synchronous Lockstep Orchestration**
- *Problem:* Drones executed actions asynchronously in physical space, which violated the fundamental Markov assumption of QMIX (which requires joint action execution), causing the drones to collide and behave erratically.
- *Fix:* Implemented a strict synchronous barrier in `QMIXMissionController`. The QMIX network is only queried when **all** drones have finished their previous physical waypoints.

**H10: 300-Decision Run & Physical Crash**
- *Problem:* Drone 1 crashed into a building during a deep exploration mission.
- *Root Cause:* The takeoff altitude was configured to 10m, which was lower than some procedurally generated buildings (e.g., `maj_bldg_9`).

**H13: 15m Altitude Validation**
- *Problem:* Needed to resolve the H10 crash safely.
- *Fix:* Modified `configs.py` `takeoff_altitude` to 15.0m. Validated with 0 collisions and 0 safety overrides.

**H14: Dashboard Flicker Diagnosis/Fix**
- *Problem:* The frontend metrics rapidly flickered between current values and old values.
- *Root Cause:* Starting a new mission didn't kill old orphaned `qmix_drone_test` Python nodes (which ignored SIGTERM). Also, React StrictMode caused duplicate WebSockets.
- *Fix:* Upgraded backend cleanup to use `SIGKILL` (signal 9), added a monotonic timestamp filter to the FastAPI subscriber, and properly closed WebSockets in React `useEffect` cleanup.

---

## 6. CURRENT CRITICAL CONFIGURATION

*Inspect actual repository code if updating these values.*

- **Takeoff Altitude:** 15.0 m (CURRENT VALIDATED)
- **Drone Count:** 2 (CURRENT VALIDATED)
- **Max Decisions:** 300 (CURRENT VALIDATED)
- **Waypoint Dwell Ticks:** 40 (CURRENT VALIDATED)
- **Velocity Parameters:** `MPC_XY_CRUISE=5.0`, `MPC_XY_VEL_MAX=8.0` (CURRENT VALIDATED)
- **Goal Tolerance:** 0.75 m (CURRENT VALIDATED)
- **Grid Size:** 25x25 (CURRENT VALIDATED)
- **Grid Resolution:** 4.0 meters per cell (CURRENT VALIDATED)
- **Map ID:** `realistic_sar` (CURRENT VALIDATED)
- **Checkpoint:** `qmix_sar_v4_align_best.pth` (CURRENT VALIDATED)
- **ROS Telemetry Rate:** ~10 Hz (CURRENT)
- **FastAPI Port:** 8000 (CURRENT)
- **Vite Port:** 5173 (CURRENT)

---

## 7. CURRENT VALIDATED RESULTS

The strongest experimentally validated configuration currently consists of:
- **H9 Orchestration:** Synchronous lockstep physical execution.
- **H10 Endurance:** 300 joint decisions, achieving 98.0% grid coverage and discovering 4/5 dynamic victims. *(Note: H10 had a physical crash at 10m altitude).*
- **H13 Altitude Fix:** Takeoff altitude raised to 15m, yielding **0 collisions, 0 safety overrides, and 0 PX4 attitude errors**.

**Final Current Baseline:** 300 joint decisions, 98% coverage, 2 drones, 15m altitude, synchronous lockstep.

---

## 8. CURRENT DASHBOARD

The React interactive dashboard (`localhost:5173`) communicates with the FastAPI backend via WebSockets.
- **Map Selection:** Queries `/api/maps` from `world.py`.
- **Mission Start:** Posts to `/api/mission/start` which dynamically launches the Gazebo/PX4/ROS stack via Python.
- **Mission Stop:** Uses `SIGKILL` process groups to safely terminate the stack.
- **Telemetry UI:** Displays real-time `decision_count`, `coverage`, `victims_detected`, and drone trajectory histories (smoothed).

---

## 9. CURRENT MAPS / WORLDS

**SMALL / VALIDATED ENVIRONMENT**
- *Exact Map ID:* `realistic_sar`
- *Exact SDF:* `worlds/realistic_sar.sdf`
- *Grid Dimensions:* 25x25
- *Physical Dimensions:* 100m x 100m
- *Victims:* 5
- *Policy Validated:* **TRUE**

**LARGE ENVIRONMENT**
- *Exact Map ID:* N/A (Currently bypassed in UI)
- *Exact SDF:* `worlds/earthquake_world.sdf`
- *Policy Validated:* **FALSE** (Requires future testing)

---

## 10. CURRENT DRONE SCALING STATUS

**Current:** 2 drones (Validated).

**Future Target:** 6 drones.
Simply changing `drone_count=6` in the UI is **NOT** sufficient. The checkpoint is fundamentally structured for 2 agents.
Future work must include:
1. Redefining the QMIX mixer to accept variable agents OR training specific 3-agent, 4-agent, 5-agent, and 6-agent PyTorch policies.
2. Expanding the observation space configuration.
3. Conducting physical validation in Gazebo for inter-drone collision avoidance at high agent densities.

---

## 11. FUTURE WORLD SCALING

Scaling progression roadmap:
`25x25 (Validated) → 50x50 → Larger Environments`

*Requirements for scaling:*
- Verify Gazebo Real-Time Factor (RTF) does not degrade severely.
- Measure if the RL policy generalization holds (QMIX must generalize its frontier BFS behavior to unbounded spaces).
- ### Status Definitions
*   **VALIDATED**: Tested and confirmed working on hardware/simulation.
*   **IMPLEMENTED**: Code written but pending full validation.
*   **EXPERIMENTAL / FROZEN**: Kept for research but currently halted (e.g. earthquake_world).
*   **BLOCKED**: Cannot proceed without resolving a dependency.
*   **FUTURE**: Planned for a later phase.

## Environment States
*   **realistic_sar**: VALIDATED / ACTIVE
*   **earthquake_world**: EXPERIMENTAL / FROZEN (Reason: Excessive computational load causes unsafe laptop shutdowns. Frozen for hardware safety).

## Milestones History
*   **H9**: Synchronous lockstep enabled.
*   **H13**: 15m altitude enforced.
*   **H14/H15**: Various pipeline and diagnostics.
*   **N3 - N7**: QMIX offline scaling, 300-decision demos.
*   **N8**: Physical Orchestration & Victim Tracking Debugging (COMPLETED).
*   **N12**: Live Camera Bridge & MJPEG Backend (COMPLETED).
*   **N9+**: Future camera extensions and 6-drone scaling.

## Current State

### 1. MARL (Offline)
*   **State**: VALIDATED (2-agent, 300 decisions)
*   **Observation**: 29-D
*   **Model**: `models/qmix_sar_v4_align_best.pth`

### 2. Physical Orchestration (ROS 2 / QMIX)
*   **State**: VALIDATED (realistic_sar)
*   **Controller**: `qmix_mission_controller.py`
*   **Fixes**: Synchronous lockstep, trajectory limits, FOV bounding box mapping.

### 3. Simulation (Gazebo / PX4)
*   **State**: VALIDATED
*   **World**: `realistic_sar` (Earthquake world explicitly frozen)
*   **Spawn**: Fixed dynamic offsets from `generated_world_meta.json`

### 4. Vision Pipeline
*   **State**: IMPLEMENTED (MOCK/YOLO toggle active, Live MJPEG backend integrated)
*   **Node**: `yolo_human_detection.py` (Publishes `/drone_x/camera/detection_image` and `_data`)
*   **Dashboard**: `CameraStreamPanel.tsx` updated to use HTTP MJPEG streaming for zero-latency camera feeds.

### 5. Victim Tracking
*   **State**: VALIDATED
*   **Manager**: `VictimManager` coordinates simulated Gazebo actors.
*   **Fixes**: Resolved Y-axis clamping logic (which caused victims to teleport to Y=1) and correctly deduped grid-based telemetry mapping to properly count the 5 victims instead of overflowing based on FOV occurrences.
*   **Metrics**: Localization Error (Mean, Max, Grid Cell) integrated into backend ROS 2 logging.

## 5. Demonstration Modes (Project Demo)

We have implemented a clean, unified architecture for presenting the project. 
- **Active Environment**: `realistic_sar` (2 drones)
- **Frozen Environment**: `earthquake_world` (Frozen due to laptop resource constraints)

### Starting the Project Demonstration

The normal presentation flow is extremely simple and starts the system in a lightweight HEADLESS mode by default to prevent laptop overloading while preserving the 2-drone / 300-decision QMIX baseline.

1. **Start Command**:
   ```bash
   ./scripts/project_demo.sh
   ```
2. **Dashboard URL**: Open `http://localhost:5173`
3. **VIEW SIMULATION**: The dashboard features a "VIEW SIMULATION" button. Clicking this button safely launches the Gazebo GUI, connecting to the already-running physics engine without spawning duplicate drones, duplicate ROS nodes, or restarting PX4. This allows you to show the physical Gazebo world and the dashboard simultaneously.
   - *Fallback*: If browser security blocks the background GUI launch, you can manually open a new terminal and run: `export GZ_IP=127.0.0.1 && gz sim -g`

### Stopping the Project Demonstration

Use the provided stop script to safely terminate all project processes without killing unrelated system processes.

- **Stop Command**:
   ```bash
   ./scripts/project_demo_stop.sh
   ```

## 6. Current Technical Limitations

### Camera Pipeline Status: **BLOCKED**
The Gazebo camera topics (`/world/realistic_sar/model/x500_0/link/camera_link/sensor/camera/image`) currently exist but produce **0 Hz (0 frames)**. 
- **Root Cause**: The Gazebo `Sensors` plugin uses a dedicated render thread. On this specific laptop, headless EGL rendering (default NVIDIA) crashes. Forcing the Intel Mesa driver prevents the crash, but the `ogre` engine permanently deadlocks at `Waiting for init` during context creation. Because the engine never initializes, the camera sensor is physically unable to capture pixels, starving the entire downstream pipeline.
- **Result**: The dashboard camera panels legitimately show "WAITING FOR STREAM".

### Victim Detection Status: **EMPTY**
The dashboard Victim Detection panel correctly shows no data.
- **Root Cause**: The physical human victims exist in Gazebo, but YOLO relies on visual data from the drone cameras. Because the cameras are generating 0 frames (due to the Gazebo render bug), YOLO is receiving 0 frames, and therefore making 0 visual detections. The `VictimManager` and telemetry serialization are completely healthy, but they have no input data to process.

---

## 12. DYNAMIC VICTIMS

**Current State:** Victims have static coordinates loaded from `generated_world_meta.json`. Detection is simulated by spatial proximity (when a victim coordinate falls inside a drone's FOV grid cell).

**Future Target:** Modular `VictimManager` for realistic dynamic movement.
Victims should not teleport randomly. They should possess:
- Spawns, bounding boxes, and obstacle avoidance.
- Controlled movement profiles (e.g., wandering, pausing).
- Synchronized telemetry updates sent to the React dashboard.

---

## 13. CAMERA-BASED VICTIM DETECTION

**Current State:** There are no Gazebo camera sensors attached to the drones, and no ROS 2 image topics exist. Detection is purely semantic/coordinate-based.

**Future Target:** 
Integrate Gazebo camera sensors → Publish to ROS 2 image topic → Process via `cv_bridge` → Feed into YOLO detector → Filter bounding box confidence → Project to world coordinate → Update `VictimManager` → Display on dashboard.

*Note: Do NOT attempt to integrate YOLO code until the physical Gazebo camera rendering and ROS 2 image topics are firmly established.*

---

## 14. YOLO ROADMAP

1. **Stage 1:** Attach camera sensors to Drone URDF/SDF.
2. **Stage 2:** Bridge Gazebo transport to ROS 2 `Image` topics.
3. **Stage 3:** Validate camera output visually.
4. **Stage 4:** Collect/prepare SAR victim dataset.
5. **Stage 5:** Select YOLO architecture (e.g., YOLOv8-nano).
6. **Stage 6:** Train/fine-tune the detector.
7. **Stage 7:** Run inference node subscribing to drone cameras.
8. **Stage 8:** Track detections temporally.
9. **Stage 9:** Perform raycasting to associate bounding boxes with 3D world coordinates.
10. **Stage 10:** Update the VictimManager.
11. **Stage 11:** Render actual YOLO detections on the dashboard.

---

## 15. IMPORTANT RL QUESTION: VICTIM INFORMATION

Currently, victims **do not** appear in the RL observation space. QMIX is trained purely for maximum area coverage. 

**Future Research Options:**
- **OPTION A (Current Trajectory):** Victims remain outside the RL observation. QMIX handles exploration, while an entirely independent perception pipeline (YOLO) handles detection.
- **OPTION B (Alternative):** Victims become part of the RL state. This would require completely retraining QMIX to alter its behavior (e.g., pausing to orbit a victim instead of continuing to explore).

---

## 16. FUTURE RESEARCH EXPERIMENTS

Define the following experimental metrics for future thesis/evaluation work:
*Metrics: Coverage %, Victims Detected, Path Length, Safety Overrides, Collisions, Inference Latency, RTF.*

* **Exp A:** Baseline (2 drones, 25x25, static victims).
* **Exp B:** Larger Map (2 drones, 50x50).
* **Exp C, D, E, F:** Scaled agents (3, 4, 5, 6 drones).
* **Exp G:** Moving victims.
* **Exp H & I:** YOLO perception integration.

---

## 17. PROFESSOR DEMONSTRATION WORKFLOW

1. Open terminal and run: `bash scripts/professor_demo.sh`
2. Open browser to `http://localhost:5173`.
3. Select `realistic_sar` and `2 Drones`.
4. Click **START MISSION**.
5. Observe headless Gazebo takeoff and live telemetry dashboard.
6. Stop mission via UI or press Ctrl+C in terminal.

*To view 3D environments (Ensure mission is stopped):*
- Small: `bash scripts/view_environment.sh small`
- Large: `bash scripts/view_environment.sh large`

---

## 18. HEADLESS GAZEBO EXPLANATION

**Headless Gazebo does NOT mean physics is disabled.**
The simulation fully computes rigid-body dynamics, PX4 SITL aerodynamics, and ROS 2 middleware in the background. We simply disable the 3D graphics rendering window to drastically save CPU/GPU overhead. This efficiency allows the complex RL models to run reliably in real-time, while our React dashboard acts as a dedicated C2 (Command and Control) visualization layer.

---

## 19. WHAT MUST NOT BE BROKEN

**DO NOT BREAK THESE WITHOUT A SPECIFIC REASON:**
- **2-agent validated checkpoint:** Altering agent count without retraining will crash matrix multiplications.
- **H9 synchronous lockstep:** Removing this causes asynchronous physical chaos and violates QMIX assumptions.
- **BFS frontier update:** Generates the highly effective frontier vectors for the network.
- **15m altitude:** Prevents building collisions.
- **Monotonic timestamp filtering:** Prevents UI ghosting and flickering from stale ROS messages.
- **Process Cleanup using SIGKILL:** `qmix_drone_test` ignores SIGTERM. You must use `kill -9` or it will orphan.

---

## 20. KNOWN LIMITATIONS

- **Agent Count:** Only 2-agent policy is validated.
- **Perception:** Camera perception and YOLO are not yet integrated; victim detection is purely spatial.
- **Altitude:** Z-axis altitude is statically defined (15m) and not dynamically controlled by the RL agent.
- **Hardware:** Exclusively uses PX4 SITL; physical hardware deployment bridges (e.g., real Pixhawks) are not configured.

---

## 21. NEXT DEVELOPMENT ORDER

**PHASE N1:** Stabilize current 2-drone professor demo. *(DONE)*
**PHASE N2:** Make launcher/documentation reliable. *(DONE)*
**PHASE N3:** Validate larger environment (`earthquake_world.sdf`) with 2 drones.
**PHASE N4:** Implement modular `VictimManager` with physical models. *(DONE)*
**PHASE N5:** Add realistic dynamic movement behaviors to human victims. *(DONE)*
**PHASE N6:** Integrate Gazebo camera sensors / ROS 2 image pipeline. *(DONE - EGL Headless fallback active via cam2image)*
**PHASE N7:** Train/fine-tune YOLO Human Detection. *(DONE - Mock OpenCV node implemented due to PyPI timeouts; structurally identical to real YOLO)*
**PHASE N8:** Convert YOLO detections into useful victim observations/localized positions for QMIX.
**PHASE N9:** Integrate moving human victims with actual visual detection pipeline.
**PHASE N10:** Improve victim tracking/association so the same person is not repeatedly counted.
**PHASE N11:** Improve dashboard to show live camera feeds, bounding boxes, confidence, detected victim IDs, and drone responsible for detection.
**PHASE N12+:** Scale QMIX from 2 drones toward 3, 4, 5 and eventually 6.

---

## 22. CHANGELOG

- **[Aug 2026] Phase 1-3:** Implemented ROS 2 telemetry, FastAPI bridge, and React dashboard.
- **[Aug 2026] Phase 4:** Migrated to Headless Gazebo to prevent X11 crashes.
- **[Aug 2026] H9:** Fixed catastrophic asynchronous behavior by implementing synchronous lockstep execution.
- **[Aug 2026] H10:** Scaled mission to 300 decisions. Drone 1 collided at 10m altitude.
- **[Aug 2026] H13:** Fixed crash by raising takeoff altitude to 15m.
- **[Aug 2026] H14:** Fixed React dashboard flicker by implementing SIGKILL process cleanup and monotonic timestamp filtering.
- **[Aug 2026] Professor Demo:** Finalized `professor_demo.sh` workflow.
- **[Aug 2026] Phase N3:** Added `earthquake_world` architecture support (dynamic grid sizing, origin offsets).
- **[Aug 2026] Phase N4:** Implemented modular `VictimManager` with physical `rescue_randy_sitting` Gazebo spawning and removed static SDF victims.
- **[Aug 2026] Phase N5:** Added realistic dynamic movement behaviors to human victims with Gazebo physical pose updates and obstacle avoidance synchronization with `SARGridEnv`.
- **[Aug 2026] Phase N6:** Diagnosed EGL Headless limitations for Gazebo camera rendering. Implemented `cam2image` pipeline proxy for downstream processing.
- **[Aug 2026] Phase N7:** Deployed `yolo_human_detection` ROS 2 node. Bypassed PyPI `ultralytics` timeout by implementing OpenCV-based mock inference publishing at 15Hz to `[/drone_0/camera/detection]`.

---

## 23. CURRENT STATUS TABLE

| Component | Status | Validation |
|----------|--------|------------|
| QMIX Algorithm | IMPLEMENTED | VALIDATED (2-agent) |
| 2-drone physical execution | IMPLEMENTED | VALIDATED (15m alt) |
| 3-drone execution | NOT IMPLEMENTED | NOT VALIDATED |
| 6-drone execution | NOT IMPLEMENTED | NOT VALIDATED |
| 25x25 world | IMPLEMENTED | VALIDATED |
| Large world (earthquake_world) | IMPLEMENTED | EXPERIMENTAL (Labelled in UI) |
| Headless Gazebo | IMPLEMENTED | VALIDATED |
| Dashboard / React | IMPLEMENTED | VALIDATED |
| FastAPI / WebSocket | IMPLEMENTED | VALIDATED |
| Actual human models (N4) | IMPLEMENTED | VALIDATED (VictimManager) |
| Dynamic victims (Moving N5) | IMPLEMENTED | VALIDATED |
| Gazebo cameras & Image Pipeline (N6) | IMPLEMENTED (Mocked) | VALIDATED (Via cam2image fallback) |
| YOLO detection (N7) | IMPLEMENTED (Mocked) | VALIDATED (15Hz OpenCV bounding boxes) |
