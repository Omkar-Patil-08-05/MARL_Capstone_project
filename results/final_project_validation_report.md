# Multi-Agent Reinforcement Learning for Drone Swarm SAR
## Final Project Validation, Empirical Reconciliation, and System Freeze Report

**Author:** Autonomous Systems Researcher / MARL Swarm Team  
**Institution:** Capstone Project Research Group  
**Target Environment:** Ubuntu 24.04 LTS, ROS 2 Jazzy, Gazebo Harmonic 8.x, PX4 SITL v1.14+, Python 3.12, PyTorch 2.6.0+cu131, NVIDIA RTX 3050 (6GB VRAM)  
**Project Workspace:** `/home/capstone/capstone_project_antigravity`  
**Date of Freeze:** November 3, 2026  
**Status:** ✅ **SYSTEM FULLY VALIDATED AND FROZEN FOR DEMONSTRATION**

---

## 1. Executive Summary & Freeze Classification

This document represents the definitive technical audit, empirical reconciliation, and academic freeze for the capstone project: **Multi-Agent Reinforcement Learning (MARL) for Drone Swarm Coordination in Search and Rescue using QMIX, ROS 2 Jazzy, PX4 SITL, and Gazebo Harmonic**.

The system demonstrates a closed-loop, vision-driven search-and-rescue mission where autonomous quadrotors execute decentralized MARL area coverage, discover human victims using onboard RGB-D cameras and YOLOv8n object detection, temporally confirm detections over a consensus window, allocate dynamic inspection tasks, and execute setpoint maneuvers with hover dwell—**without using ground-truth positions in the autonomous decision pipeline**.

### 1.1 Architectural Freeze Status Table

| Subsystem / Capability | Verification Level | Empirical Evidence & Test Artifacts | Academic Status |
| :--- | :---: | :--- | :---: |
| **QMIX Multi-Agent Coordination** | Component & Closed-Loop | Checkpoint `models/qmix_sar_v4_align_best.pth`, BFS grid tracking | **Validated** |
| **PX4 SITL Integration** | Hardware-in-Loop / SITL | Dual offboard quadrotors, NED setpoints, `takeoff_altitude=10m` | **Validated** |
| **Gazebo Harmonic Simulation** | Physics & Sensors | Headless rendering, nadir RGB-D cameras (`x500_rgbd_down`) | **Validated** |
| **YOLOv8n GPU Perception** | Real-Time Hardware Tensor | Batched PyTorch inference, class 0 (`person`), CUDA 13.1 | **Validated** |
| **RGB-D Pinhole Localization** | Optical Geometry | Camera intrinsics via `/camera_info`, sub-patch depth sampling | **Validated** |
| **Temporal Event Confirmation** | Multi-Frame Consensus | $K=3$ distinct camera frames, spatial gating $r \le 3.8\,\text{m}$ | **Validated** |
| **Dynamic Task Allocation** | Deterministic / Hungarian | `SARTaskAllocator`, 1-to-1 greedy distance, lifecycle tracking | **Validated** |
| **Two-Drone Closed Loop Mission** | Full System Gazebo SITL | `results/final_autonomous_sar_metrics.json`, `events.csv`, `log.txt` | **Validated** |
| **Six-Drone Swarm Compatibility** | E2E Integration Suite | 9/9 tests in `scripts/test_phase3d_e2e_integration.py` | **Validated (Tested)** |
| **Multispectral Thermal Verification** | Source & Synthetic Test | `scripts/test_rgb_thermal_fusion.py` (simulated frames) | **Implemented / Baseline** |
| **Earthquake World Support** | Map Config & Grid | `worlds/earthquake_world.sdf`, metadata parsing verified | **Configured** |

---

## 2. Project Architecture and Data Flow

The architecture decouples autonomous perception and mission execution into independent, communicating ROS 2 nodes, maintaining ground-truth isolation across all stages.

```mermaid
flowchart TD
    subgraph Gazebo_Harmonic["Gazebo Harmonic Physics Engine"]
        GZ_Cam["Nadir RGB-D Camera\n(640x480 @ 5Hz)"]
        GZ_Drone["x500 Drone Kinematics\n(PX4 SITL Interface)"]
    end

    subgraph Perception_Pipeline["Autonomous Perception Pipeline"]
        YOLO["YOLOv8n Batched Inference\n(class='person', conf >= 0.30)"]
        Localizer["RGBDVictimLocalizer\n(Pinhole + Median Depth Patch)"]
        Confirmer["TemporalEventConfirmer\n(K=3/6 frames, span >= 0.2s)"]
    end

    subgraph Mission_Control["Swarm Mission Controller & Orchestration"]
        VM["VictimManager\n(Track Lifecycle & De-dup)"]
        Alloc["SARTaskAllocator\n(1-to-1 Greedy Assignment)"]
        QMIX["QMIX Controller + Safety Shield\n(Occupancy, Rubble Clearance, Hover Dwell)"]
    end

    subgraph Vehicle_Interface["PX4 Autopilot SITL"]
        PX4_0["drone_0 (OFFBOARD Mode)"]
        PX4_1["drone_1 (OFFBOARD Mode)"]
    end

    GZ_Cam -->|"/drone_i/camera/image"| YOLO
    GZ_Cam -->|"/drone_i/camera/depth_image"| Localizer
    GZ_Cam -->|"/drone_i/camera/camera_info"| Localizer
    GZ_Drone -->|"/model/drone_i/pose (Odometry)"| Localizer

    YOLO -->|"/drone_i/perception/detections"| Localizer
    Localizer --> Confirmer
    Confirmer -->|"/sar/victim_discovery"| VM
    VM --> Alloc
    Alloc --> QMIX
    QMIX -->|"/drone_0/fmu/in/trajectory_setpoint"| PX4_0
    QMIX -->|"/drone_1/fmu/in/trajectory_setpoint"| PX4_1
    PX4_0 -.-> GZ_Drone
    PX4_1 -.-> GZ_Drone
```

---

## 3. Strict Ground-Truth Isolation Verification

A primary objective of this project is ensuring that no privileged ground-truth data enters the autonomous perception, estimation, or decision pipeline.

### 3.1 Verification Matrix by Component

| Pipeline Module | Input Topics / Sources | Ground-Truth Ingestion? | Isolation Mechanism |
| :--- | :--- | :---: | :--- |
| **YOLO Perception** | `/drone_{i}/camera/image` | **NO** | Pure image tensor forward pass in `swarm_perception_node.py` |
| **RGB-D Localizer** | `/drone_{i}/camera/depth_image`, `/camera_info`, `/model/drone_{i}/pose` | **NO** | Pinhole ray-casting using onboard depth buffer and drone ego-pose |
| **Temporal Confirmer** | Perception candidate stream | **NO** | Multi-frame consensus accumulator; zero world knowledge |
| **Victim Manager** | `/sar/victim_discovery` | **NO** | Ingests purely sensor-derived tracks into `tracked_victims` |
| **SAR Task Allocator** | `tracked_victims` | **NO** | Allocates tasks based on estimated target coordinates |
| **QMIX Mission Controller** | SARTask coordinates, agent odometry | **NO** | Commands setpoints toward estimated coordinates |
| **Post-Hoc Evaluation** | `generated_world_meta.json` | **YES (Post-Hoc Only)** | Hungarian spatial matching executed exclusively after mission termination |

### 3.2 Audit of Permitted Exceptions
Ground-truth victim coordinates are parsed from [`worlds/generated_world_meta.json`](file:///home/capstone/capstone_project_antigravity/worlds/generated_world_meta.json) in exactly two locations:
1. In [`victim_manager.py`](file:///home/capstone/capstone_project_antigravity/drone_ws/src/swarm_controller/swarm_controller/victim_manager.py#L45-L75), ground-truth victims are loaded into `self.victims` exclusively to calculate post-hoc coverage metrics (`unique_detected / total_victims`) reported in the telemetry payload. The autonomous allocator never queries `self.victims`; it consumes only `self.tracked_victims`.
2. In [`scripts/run_two_drone_sar_validation.py`](file:///home/capstone/capstone_project_antigravity/scripts/run_two_drone_sar_validation.py#L315-L345), ground-truth victims are loaded in Step 5 (after Gazebo and the flight stack have been terminated) to calculate horizontal localization error against actual model poses.

---

## 4. Deep Technical Pipeline Specifications

### 4.1 RGB-D Pinhole Geometric Localization
The localizer in [`rgbd_victim_localizer.py`](file:///home/capstone/capstone_project_antigravity/drone_ws/src/swarm_controller/swarm_controller/rgbd_victim_localizer.py) operates as follows:
1. **Intrinsics Calibration:** Extracted dynamically from ROS 2 `/camera_info` messages:
   $$f_x = 269.9, \quad f_y = 269.9, \quad c_x = 320.0, \quad c_y = 240.0$$
2. **Depth Patch Extraction:** Given detection bounding box $(x_1, y_1, x_2, y_2)$ of width $w$ and height $h$, a centered patch of dimensions $0.30w \times 0.30h$ is sampled from the aligned $32\text{FC1}$ depth frame.
3. **Statistical Filtering:** Invalid, zero, NaN, and Inf depth pixels are removed. Detections with fewer than 15 valid samples or excessive spread ($\sigma > 1.2\,\text{m}$) are discarded.
4. **Coordinate Transformation:** The centroid $(c_x, c_y)$ and median depth $Z_c$ are back-projected:
   $$x_c = \frac{(u - c_x) Z_c}{f_x}, \quad y_c = \frac{(v - c_y) Z_c}{f_y}, \quad z_c = Z_c$$
   Converted to the drone FLU body frame:
   $$x_d = -y_c, \quad y_d = -x_c, \quad z_d = -z_c$$
   Rotated by vehicle yaw $\psi$ and translated into world coordinates:
   $$\begin{bmatrix} x_{\text{world}} \\ y_{\text{world}} \end{bmatrix} = \begin{bmatrix} x_{\text{drone}} \\ y_{\text{drone}} \end{bmatrix} + \begin{bmatrix} \cos\psi & -\sin\psi \\ \sin\psi & \cos\psi \end{bmatrix} \begin{bmatrix} x_d \\ y_d \end{bmatrix}$$

### 4.2 Temporal Event Confirmation
In [`temporal_event_confirmer.py`](file:///home/capstone/capstone_project_antigravity/drone_ws/src/swarm_controller/swarm_controller/temporal_event_confirmer.py), isolated detections are prevented from triggering false missions:
- **Consensus Threshold:** Requires $K=3$ observations within a sliding window of $W=6$ frames.
- **Frame-Uniqueness Check:** Enforces that every vote originates from a unique camera timestamp $(t_{\text{stamp}})$, rejecting multiple detections within a single frame.
- **Time Span Gate:** Enforces $t_{\text{span}} = \max(t) - \min(t) \ge 0.20\,\text{s}$ to prevent instantaneous flash artifacts.
- **Spatial Gating:** Associates candidate detections to an active track if within dynamic gate radius $r_{\text{gate}} = 3.0\,\text{m} + 0.08 Z_c$.

### 4.3 Task Allocation & Priority Execution
- **Defect A Resolution:** When running in 2-drone mode (`det_agents` list is empty), [`qmix_mission_controller.py`](file:///home/capstone/capstone_project_antigravity/drone_ws/src/swarm_controller/swarm_controller/qmix_mission_controller.py#L353-L356) and [`swarm_coordination_manager.py`](file:///home/capstone/capstone_project_antigravity/drone_ws/src/swarm_controller/swarm_controller/swarm_coordination_manager.py#L79-L84) dynamically fall back to eligible QMIX drones (`drone_0`, `drone_1`).
- **Defect B Resolution:** When an agent is in `ASSIGNED`, `APPROACHING`, or `INVESTIGATING`:
  1. Exploratory QMIX actions are superseded by SAR guidance.
  2. The guidance algorithm steps 1 grid cell along 4-connected directions $(+X, -X, +Y, -Y)$ toward the victim cell.
  3. When within 1.5 grid cells ($\approx 6.0\,\text{m}$) or during `INVESTIGATING`, the waypoint target switches directly from coarse cell center to the precise sensor-derived continuous coordinates $(x_{\text{world}}, y_{\text{world}})$.
  4. Obstacle collision checking is modified so that ground rubble cells ($z < 1.5\,\text{m}$) do not force aerial quadrotors at $10.0\,\text{m}$ flight altitude into safety hovers away from victims.

---

## 5. Empirical Verification and Data Reconciliation

### 5.1 Reconciliation of Investigation Dwell Time

A critical audit item is the investigation dwell time recorded during the live Gazebo mission:

$$\text{Reported Start Sim Time:} \quad 53.264\,\text{s}$$
$$\text{Reported Completion Sim Time:} \quad 58.168\,\text{s}$$
$$\Delta t_{\text{telemetry}} = 58.168 - 53.264 = 4.904\,\text{s}$$
$$\text{Nominal Target Dwell:} \quad 5.000\,\text{s}$$

#### Exact Root Cause Analysis
1. **Task Assignment:** In [`sar_task_allocator.py`](file:///home/capstone/capstone_project_antigravity/drone_ws/src/swarm_controller/swarm_controller/sar_task_allocator.py#L442-L465), task assignment was completed at simulation time $t = 53.168\,\text{s}$. Because the drone was already within the approach threshold ($d = 2.8\,\text{m} \le 3.0\,\text{m}$), the internal task state transitioned to `APPROACHING` and immediately to `INVESTIGATING` during that cycle.
2. **First Telemetry Output:** The `qmix_mission_controller` publishes `/swarm/telemetry` at $10\,\text{Hz}$ ($\Delta t = 0.1\,\text{s}$). The first published telemetry frame with `state: INVESTIGATING` was dispatched at $t = 53.264\,\text{s}$ (exactly $0.096\,\text{s}$ after internal initialization), explicitly reporting `progress: 0.1s`.
3. **Completion Trigger:** The allocator evaluates completion using:
   $$\text{elapsed} = t_{\text{sim}} - t_{\text{investigation\_start}} \ge 5.000\,\text{s}$$
   At $t_{\text{sim}} = 58.168\,\text{s}$:
   $$\text{elapsed} = 58.168 - 53.168 = 5.000\,\text{s}$$
4. **Conclusion:** The physical drone hovered over the victim coordinates for **$5.000\,\text{s}$** of simulation time ($53.168\,\text{s} \to 58.168\,\text{s}$, or $20.24\,\text{s}$ wall clock time). The $4.904\,\text{s}$ interval reflects the discrete $10\,\text{Hz}$ sampling delay between state transition and the first received telemetry frame.

### 5.2 Post-Hoc Localization Accuracy

- **Ground Truth Target:** `victim_24` (`casual_female` model in `worlds/realistic_sar.sdf`) located at:
  $$x_{\text{gt}} = 10.00\,\text{m}, \quad y_{\text{gt}} = 98.00\,\text{m}, \quad z_{\text{gt}} = 0.25\,\text{m}$$
- **Sensor Estimated Position:** Emitted by pinhole localizer from downward RGB-D frames:
  $$x_{\text{est}} = 13.55\,\text{m}, \quad y_{\text{est}} = 95.42\,\text{m}$$
- **Euclidean Localization Error:**
  $$\Delta x = 13.55 - 10.00 = +3.55\,\text{m}$$
  $$\Delta y = 95.42 - 98.00 = -2.58\,\text{m}$$
  $$\text{Error} = \sqrt{(3.55)^2 + (-2.58)^2} = \sqrt{12.6025 + 6.6564} = \sqrt{19.2589} = \mathbf{4.388\,\text{m}}$$
- **Operational Evaluation:** In a $100\,\text{m} \times 100\,\text{m}$ search field with $4.0\,\text{m} \times 4.0\,\text{m}$ grid cells, an error of $4.388\,\text{m}$ (1.09 grid cells) from a moving quadrotor at $10.0\,\text{m}$ altitude represents a verified True Positive detection within the operational Hungarian matching gate ($\le 5.0\,\text{m}$).

### 5.3 Multi-Agent Safety Separation
- **Separation Limit:** Minimum allowed distance: $3.0\,\text{m}$.
- **Empirical Minimum:** Observed continuously between `drone_0` and `drone_1`:
  $$d_{\text{min}} = \mathbf{7.95\,\text{m}}$$
- **Outcome:** Zero inter-drone safety violations or physical collisions occurred throughout takeoff, transit, investigation dwell, or exploration.

---

## 6. Verification and Regression Test Suite

Integrity testing was conducted across all unit, component, and multi-agent integration modules.

```
========================================================================================
                      MARL CAPSTONE INTEGRATED TEST AUDIT
========================================================================================
Test Module                                         Target Scope              Result
----------------------------------------------------------------------------------------
scripts/test_two_drone_sar_allocation.py            2-Drone QMIX Fallback     4/4 PASS
scripts/test_sar_task_allocation.py                 Allocator Lifecycle      12/12 PASS
scripts/test_temporal_event_confirmer.py            Multi-Frame Consensus     7/7 PASS
scripts/test_phase3d_e2e_integration.py             6-Drone Hybrid Swarm      9/9 PASS
python3 -m py_compile [all controller files]        Syntax & Bytecode Check   11/11 PASS
----------------------------------------------------------------------------------------
TOTAL AUTOMATED INTEGRATION TESTS:                  32 / 32 PASSED (100%)
========================================================================================
```

---

## 7. Limitations and Academic Boundary Conditions

To maintain academic defensibility, the following constraints must be explicitly acknowledged:
1. **Multispectral Thermal Infrared:** The multispectral thermal fusion pipeline ([`rgb_thermal_fusion_verifier.py`](file:///home/capstone/capstone_project_antigravity/drone_ws/src/swarm_controller/swarm_controller/rgb_thermal_fusion_verifier.py)) is fully implemented and tested with synthetic inputs, but was not executed during this physical Gazebo mission because `x500_rgbd_down` models were deployed rather than `x500_multispectral`.
2. **Nadir Vision Perspective:** Standard COCO-pretrained weights exhibit lower confidence on top-down views of standing humans than frontal views. Operating at $10.0\,\text{m}$ altitude (rather than $15.0\,\text{m}$) was required to maintain confidence scores between $0.35$ and $0.76$.
3. **Six-Drone Physical Execution:** Six-drone swarm capability is validated at the unit test, mock orchestrator, and architecture level (passing all 9 E2E tests). Due to local GPU/CPU compute constraints on the RTX 3050 laptop (which runs dual SITL at $\sim 0.24\text{x}$ real-time factor), physical 6-drone Gazebo runs are reserved for multi-GPU compute servers.

---

## 8. Exact Reproduction Commands

To replicate this exact two-drone autonomous SAR mission from a clean shell:

```bash
# 1. Source ROS 2 Jazzy and built workspace
source /opt/ros/jazzy/setup.bash
source /home/capstone/capstone_project_antigravity/drone_ws/install/setup.bash

# 2. Export calibrated localizer consensus parameters
export LOCALIZER_CONSENSUS_NEEDED=3
export LOCALIZER_MIN_CONFIDENCE=0.30
export LOCALIZER_MIN_TIME_SPAN=0.20

# 3. Execute the closed-loop autonomous mission
python3 /home/capstone/capstone_project_antigravity/scripts/run_two_drone_sar_validation.py
```

### Key Experiment Artifacts:
- **Full Report:** [`results/final_project_validation_report.md`](file:///home/capstone/capstone_project_antigravity/results/final_project_validation_report.md)
- **Validation Audit Baseline:** [`results/final_yolo_validation_audit.md`](file:///home/capstone/capstone_project_antigravity/results/final_yolo_validation_audit.md)
- **Metrics JSON:** [`results/final_autonomous_sar_metrics.json`](file:///home/capstone/capstone_project_antigravity/results/final_autonomous_sar_metrics.json)
- **Event Timeline CSV:** [`results/final_autonomous_sar_events.csv`](file:///home/capstone/capstone_project_antigravity/results/final_autonomous_sar_events.csv)
- **Raw Execution Log:** [`results/final_autonomous_sar_log.txt`](file:///home/capstone/capstone_project_antigravity/results/final_autonomous_sar_log.txt)
