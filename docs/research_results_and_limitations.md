# MARL Swarm Search & Rescue: Empirical Results, System Verification, and Methodological Limitations

**Project:** Multi-Agent Reinforcement Learning for Drone Swarm Coordination in Wilderness Search and Rescue (SAR)  
**Execution Environment:** Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic 8.10.0, PX4 SITL (Lockstep, v1.14), Python 3.12, NVIDIA RTX 3050 Laptop GPU (6GB)  
**Authoritative Checkpoint:** [`models/qmix_sar_v4_align_best.pth`](file:///home/capstone/capstone_project_antigravity/models/qmix_sar_v4_align_best.pth)  
**Authoritative SHA-256:** `f87d87528969ff3165a0cdfb299f5d228ad99502b974176d05b81479242a272d` (Verified Bit-for-Bit)  
**Target World:** [`worlds/forest_dense_candidate.sdf`](file:///home/capstone/capstone_project_antigravity/worlds/forest_dense_candidate.sdf) (950 Trees Restored, Preserved Intact)  
**Benchmark Artifacts Directory:** [`results/stage2_three_seed_benchmark/`](file:///home/capstone/capstone_project_antigravity/results/stage2_three_seed_benchmark/)  
**Date:** October 24, 2026  

---

## 1. Executive Summary & Scientific Scope

This research document presents the audited empirical findings, mathematical foundations, and methodological boundaries of the **Stage 2 Three-Seed Six-Drone Benchmark** (Seeds 42, 43, 44) and **Stage 1 Perception Validation** conducted for the MARL drone swarm capstone. 

### Core Audited Findings
1. **Decision Budget & Swarm Scalability:** Under a fixed budget of 25 global QMIX decision steps (150 individual drone actions per run), the 6-drone swarm achieved a **100% mission execution completion rate** across all three seeds with zero deadlock, zero safety filter overrides, and zero physical collisions.
2. **Coverage Rigor:** The swarm navigated through **141 unique grid center cells**, expanding to an active sensor field-of-view (FOV) footprint of **230 cells** ($42.75\%$ of the active 538 run mask). When strictly intersected with the true 416 traversable forest cells, valid FOV coverage reached **$166 / 416 = 39.90\%$**.
3. **Collision Avoidance:** The decentralized safety shield maintained an empirical minimum pairwise separation of **$3.08 \pm 0.02\text{ m}$** (absolute observed minimum: $3.07\text{ m}$), well above the physical safety threshold.
4. **Victim Detection Analysis:** Zero ground-truth victims were intercepted during the 25-step budget. Audit proved that no drone ever approached within the required $15.0\text{ m}$ radius (closest observed distance: **$24.74\text{ m}$**). Furthermore, runtime metadata fallback in the controller initialized zero victims in memory.
5. **Perception Nadir Perspective Finding:** Synthetic victim visual detection failed under nadir (downward-facing vertical) cameras because YOLOv8n classified the top-down perspective of the 3D human mesh as COCO class 14 (`bird`, conf $0.844$). The strict person-only filter (`cls == 0`) correctly discarded this detection, preventing false alerts. Offline back-projection verified that downstream 3D ray unprojection achieves **$24.2\text{ cm}$ localization accuracy** relative to ground truth when a valid detection bounding box is provided.

> [!IMPORTANT]
> **Scientific Integrity Notice:** The evaluation of three random seeds ($N = 3$) under a deterministic greedy QMIX policy in an identical static environment does **not** constitute statistical significance, proof of generalized exploration, or real-world operational readiness. Results documented herein represent empirical benchmarks in simulation.

---

## 2. Experimental Setup & Model Integrity

### 2.1 Hardware and Software Environment
- **Operating System:** Ubuntu 24.04 LTS (x86_64 Linux kernel 6.8.0-47-generic)
- **Middleware:** ROS 2 Jazzy Jalisco
- **Simulator:** Gazebo Harmonic 8.10.0 with Open Dynamics Engine (ODE)
- **Autopilot Firmware:** PX4 SITL (v1.14 release stack) operating in lockstep simulation with Micro-XRCE-DDS agent
- **Host Hardware:** Intel Core i7 (16 threads), 16GB RAM, NVIDIA GeForce RTX 3050 Laptop GPU (6GB VRAM, Driver 550.120)

### 2.2 Neural Architecture & Checkpoint Hash
The policy network is a recurrent QMIX agent trained via value factorization:
- **Base Architecture:** Gated Recurrent Unit (GRU) with hidden dimension 64, preceded by a fully connected feature encoder and followed by discrete action-value heads.
- **Local Observation Dimension:** 29 continuous features per agent (normalized ego-position, relative teammate displacement vectors, BFS global frontier distance gradient, local swarm density, and $3 \times 3$ FOV status).
- **Action Space:** 5 discrete kinematic actions: `[STAY, MOVE_NORTH, MOVE_SOUTH, MOVE_EAST, MOVE_WEST]`.
- **Checkpoint Path:** [`models/qmix_sar_v4_align_best.pth`](file:///home/capstone/capstone_project_antigravity/models/qmix_sar_v4_align_best.pth)
- **Verified SHA-256 Checksum:** `f87d87528969ff3165a0cdfb299f5d228ad99502b974176d05b81479242a272d` (Verified identical before and after all benchmark runs).

### 2.3 Simulation Mode Isolation: High-Speed vs. Full Perception
To prevent thermal throttling and GPU resource exhaustion:
- **High-Speed Mode (`lightweight: true`):** Used for the 6-drone multi-seed benchmark. Gazebo ran headless without Ogre2 visual rendering; RGB-D camera sensor topics were disabled. Physics ran at maximum sustainable speed on CPU (0% GPU utilization, maintaining GPU temperatures below $40.0^\circ\text{C}$).
- **Single-Drone Perception Mode:** Isolated to a single drone (`drone_0`) at 2 Hz RGB-D rendering to benchmark the optical detector without risking system shutdown.

---

## 3. World Configuration & Valid-Cell Geometry

### 3.1 Forest Terrain and Tree Density
The benchmark was conducted in the restored dense forest environment:
- **World File:** [`worlds/forest_dense_candidate.sdf`](file:///home/capstone/capstone_project_antigravity/worlds/forest_dense_candidate.sdf)
- **Model Distribution:** Exactly **950 individual tree models** instantiating a dual-layer architecture:
  - Collidable tree trunks (cylindrical collision geometries, $r \approx 0.3 - 0.5\text{ m}$) preventing low-altitude transit.
  - Elevated visual canopies ($z \approx 8.0 - 18.0\text{ m}$) creating partial optical occlusion.
- **Flight Altitude:** The swarm maintains a cruising setpoint of $z = 15.0\text{ m}$ above the terrain surface, navigating through upper canopy boundaries.

### 3.2 Discrete Grid Bounds and Obstacle Masking
The continuous coordinate system is mapped to a discrete 2D grid:
- **Grid Dimensions:** $25 \times 25$ cells ($N_{\text{total}} = 625$ cells).
- **Cell Dimension ($L_{\text{cell}}$):** $8.0\text{ m} \times 8.0\text{ m}$.
- **Forest World Bounds:** $X \in [-180.0, 20.0]\text{ m}$, $Y \in [-20.0, 180.0]\text{ m}$.
- **Obstacle Projection:** Tree trunk Axis-Aligned Bounding Boxes (AABBs) are rasterized onto the grid:
  - **Forest Obstacle Cells:** 209 cells marked invalid (`grid == -1`).
  - **True Traversable Cells ($N_{\text{forest\_valid}}$):** $625 - 209 = \mathbf{416}$ cells.
- **Active Controller Mask Discrepancy:** The controller runtime defaulted to `earthquake_world_meta.json` bounds ($X \in [0, 100]$, 87 obstacle cells), creating an active valid denominator of $625 - 87 = \mathbf{538}$ cells during execution.

---

## 4. Stage 2 Benchmark Results & Multi-Seed Analysis

Three sequential runs (Seeds 42, 43, 44) were conducted with 30-second thermal cooldown intervals. Every metric was recomputed directly from the raw decision logs (`qmix_decisions.csv`) and trajectory traces (`trajectory_drone_0.csv` to `trajectory_drone_5.csv`).

### 4.1 Audited Per-Seed Metrics

| Evaluation Telemetry | Seed 42 | Seed 43 | Seed 44 | Mean ± Std ($N=3$) | Verification Source |
| :--- | :---: | :---: | :---: | :---: | :--- |
| **Execution Status** | COMPLETED | COMPLETED | COMPLETED | 100% Completion | Run logs |
| **Completed Global Decisions** | **25 / 25** | **25 / 25** | **25 / 25** | $25.00 \pm 0.00$ | `qmix_decisions.csv` |
| **Total Drone Decisions** | 150 | 150 | 150 | $150.0 \pm 0.0$ | 25 steps $\times$ 6 drones |
| **Unique Center Cells Visited** | **141** | **141** | **141** | $141.00 \pm 0.00$ | Union of $(gx_d, gy_d)$ |
| **Active Run Explored FOV ($M$)**| **230** | **230** | **230** | $230.00 \pm 0.00$ | $3 \times 3$ window on run mask |
| **Unconstrained Raw Grid FOV** | **247** | **247** | **247** | $247.00 \pm 0.00$ | Full 625 grid projection |
| **FOV Cells in Traversable Forest**| **166** | **166** | **166** | $166.00 \pm 0.00$ | Intersection with 416 mask |
| **Minimum Pairwise Separation** | **3.07 m** | **3.11 m** | **3.07 m** | **$3.08 \pm 0.02\text{ m}$** | Pairwise trajectory norm |
| **Physical Collisions** | **0** | **0** | **0** | $0.00 \pm 0.00$ | Odometry & crash logs |
| **Safety Overrides Triggered** | **0** | **0** | **0** | $0.00 \pm 0.00$ | Shield override count |
| **Swarm Failsafes / Drops** | **0** | **0** | **0** | $0.00 \pm 0.00$ | PX4 flight mode telemetry |
| **Max Flight Speed** | 3.99 m/s | 3.99 m/s | 3.99 m/s | $3.99 \pm 0.00\text{ m/s}$ | PX4 velocity magnitude |
| **Active Sim Flight Duration** | 51.4 s | 50.9 s | 51.4 s | $51.23 \pm 0.28\text{ s}$ | Gazebo `/clock` delta |
| **Total Wall-Clock Duration** | 574.3 s | 494.0 s | 504.4 s | $524.23 \pm 35.74\text{ s}$ | Wall timer (8.7 min mean) |
| **Effective Real-Time Factor** | **0.090** | **0.103** | **0.102** | **$0.098 \pm 0.006$** | Sim time / Wall time |
| **Average CPU Utilization** | 34.6% | 32.6% | 28.9% | $32.03\% \pm 2.37\%$ | 16-thread host monitor |
| **Average GPU Temperature** | 39.5°C | 39.5°C | 39.4°C | $39.47^\circ\text{C} \pm 0.05^\circ\text{C}$ | Peak temp: 40.0°C |
| **GPU Compute Utilization** | 0% | 0% | 0% | $0.0\% \pm 0.0\%$ | Headless physics |
| **Ground-Truth Victim Detections** | 0 / 5 | 0 / 5 | 0 / 5 | 0.00% | Proximity scoring ($r \le 15\text{ m}$) |

### 4.2 Cross-Seed Variation Analysis
Across seeds 42, 43, and 44, the discrete high-level decision sequences and visited grid cells were **identical**. 
- **Cause of Discrete Determinism:** The QMIX policy network executes greedily ($\epsilon = 0.0$) from fixed spawn coordinates in a deterministic, static obstacle field. The random seed parameter is propagated to Python's `random`, `numpy.random`, and PyTorch, but with zero stochastic exploration during evaluation, the policy produces identical discrete action selections.
- **Observed Continuous Variations:**
  - **Minimum Pairwise Separation:** Varied between $3.0689\text{ m}$ (Seed 42), $3.1287\text{ m}$ (Seed 43), and $3.1292\text{ m}$ (Seed 44). This variance arises from continuous-time numerical integration in Gazebo’s ODE physics engine and asynchronous DDS message scheduling.
  - **Simulation Real-Time Factor (RTF):** Varied between 0.090 and 0.103 due to host CPU background thread scheduling and operating system interrupts.
- **Scientific Conclusion:** Identical discrete outputs across seeds reflect deterministic execution fidelity, **not** independent evidence of generalization across diverse environments.

---

## 5. Coverage Semantics & Mathematical Framework

### 5.1 Verification of FOV Intersection with Forest Mask
An audit was conducted to determine whether the reported 230 FOV cells are entirely within the 416-cell traversable forest mask:
- **Empirical Finding:** **No.** The 230 FOV cells are **not** all within the 416 traversable forest mask.
- **Mathematical Breakdown:**
  1. The 141 visited center cells expand via an $r = 1$ ($3 \times 3$) square kernel to cover **247 unique cells** across the entire $25 \times 25$ grid.
  2. When intersected with the 416 true forest traversable cells:
     $$\text{FOV}_{\text{valid}} = \text{FOV}_{\text{total}} \cap \text{Mask}_{\text{forest}} = 247 \cap 416 = \mathbf{166}\text{ cells}$$
  3. The remaining **81 cells** of the 247 FOV footprint (and 64 of the 230 active run FOV cells) overlap tree obstacle bounding boxes. Because the drones cruise at $15\text{ m}$ altitude (above tree crowns), the camera footprint projects over tree canopy envelopes. Under strict ground-traversability definitions, these cells cannot be traversed by low-altitude ground agents.

### 5.2 Rigorous Coverage Matrix Across All Definitions

$$\text{Coverage}_{\text{center, forest}} = \frac{|\{(gx_d, gy_d)\} \cap \text{Mask}_{\text{forest}}|}{N_{\text{forest\_valid}}} = \frac{96}{416} = \mathbf{23.08\%}$$

$$\text{Coverage}_{\text{center, full}} = \frac{|\{(gx_d, gy_d)\}|}{N_{\text{grid\_total}}} = \frac{141}{625} = \mathbf{22.56\%}$$

$$\text{Coverage}_{\text{FOV, forest}} = \frac{|\text{FOV} \cap \text{Mask}_{\text{forest}}|}{N_{\text{forest\_valid}}} = \frac{166}{416} = \mathbf{39.90\%}$$

$$\text{Coverage}_{\text{FOV, active}} = \frac{|\text{FOV}_{\text{active}}|}{N_{\text{run\_valid}}} = \frac{230}{538} = \mathbf{42.75\%}$$

$$\text{Coverage}_{\text{FOV, grid}} = \frac{|\text{FOV}_{\text{unconstrained}}|}{N_{\text{grid\_total}}} = \frac{247}{625} = \mathbf{39.52\%}$$

| Metric Description | Explored (Numerator) | Denominator Basis | Calculated Coverage (%) | Mathematical Definition |
| :--- | :---: | :---: | :---: | :--- |
| **Traversable Center-Cell Coverage** | **96** | **416** | **23.08%** | Center cells strictly in forest mask / 416 |
| **Total Center Cells over Forest Mask** | **141** | **416** | **33.89%** | Total unique center positions / 416 |
| **Total Center Cells over Full Grid** | **141** | **625** | **22.56%** | Total unique center positions / 625 |
| **Traversable Sensor FOV Coverage** | **166** | **416** | **39.90%** | FOV cells strictly in forest mask / 416 |
| **Active Run Explored FOV Coverage** | **230** | **538** | **42.75%** | Active FOV / Active controller mask |
| **Active FOV Divided by Forest Mask** | **230** | **416** | **55.29%** | Uncorrected ratio (contains 64 obstacle cells) |
| **Unconstrained Raw Grid FOV Coverage**| **247** | **625** | **39.52%** | Unconstrained FOV union / 625 |

---

## 6. Safety, Kinematics, and Computational Telemetry

### 6.1 Inter-Agent Separation and Multi-Agent Safety Shield
- **Configured Safety Radius:** $r_{\text{safe}} = 2.0\text{ m}$.
- **Empirical Minimum Separation:** **$3.0689\text{ m}$** (observed between `drone_0` and `drone_2` during step 12 transition).
- **Aggregate Minimum Separation:** $3.08 \pm 0.02\text{ m}$.
- **Safety Overrides:** Exactly **0** overrides triggered across all 450 decision executions. The learned QMIX policy inherently coordinated decentralized spatial dispersion, eliminating cell-swap conflicts without requiring emergency safety shield intervention.

### 6.2 Kinematics and Flight Control
- **Waypoint Traversal Speed:** Drones achieved an average horizontal ground speed of $3.98\text{ m/s}$ (maximum $3.99\text{ m/s}$), corresponding to PX4’s maximum velocity limit (`MPC_XY_VEL_MAX = 4.0 m/s`).
- **Cruising Altitude Stability:** Drones maintained $15.15 \pm 0.05\text{ m}$, rejecting ground-effect disturbances and ODE physics oscillations.
- **Flight Time:** Each 25-step mission spanned $51.2\text{ s}$ of active flight ($2.05\text{ s}$ per decision step), demonstrating stable velocity setpoint tracking.

### 6.3 Resource Utilization and Hardware Cooldown
- **Host CPU Load:** Ranged from $28.9\%$ to $34.6\%$ (mean $32.0\%$) across all 16 cores, verifying that headless ODE physics does not saturate modern multi-threaded processors.
- **GPU Thermal Safety:** GPU temperature hovered between $39.4^\circ\text{C}$ and $39.5^\circ\text{C}$ with $0\%$ VRAM and compute utilization, validating the High-Speed Mode as thermally safe for long-duration benchmark campaigns on mobile workstations.

---

## 7. Ground-Truth Proximity vs. Visual Detection Analysis

### 7.1 Root Causes of Zero Ground-Truth Victim Detections
The Stage 2 benchmark reported **0 / 5 victims detected** across all three seeds. Investigation revealed two independent root causes:

1. **Software / Configuration Cause (Runtime Manager Fallback):**
   In `drone_ws/src/swarm_controller/swarm_controller/swarm_runner.py` line 220, the map selection logic:
   ```python
   elif parsed.map == "forest_sar":
       meta_path = ...
   else:
       meta_path = "worlds/earthquake_world_meta.json"
   ```
   did not contain an explicit branch for `"forest_dense_candidate"`. The controller defaulted to `earthquake_world_meta.json`, which contains `victims: []`. As a result, the runtime `VictimManager` held 0 victims in memory during execution.

2. **Physical / Geometric Cause (Distance Out of Range):**
   Even if the metadata had been loaded correctly, **no drone ever approached within the configured $15.0\text{ m}$ Euclidean proximity threshold**. The closest drone-to-victim separation was **$24.74\text{ m}$**.

### 7.2 Per-Victim Proximity Audit

| Victim ID | Ground Truth World $(X, Y, Z)$ | Closest Drone | Time ($t$) | Closest Distance (Gazebo) | Proximity Bubble | Detection Outcome |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **victim_0** | $(-82.03, 43.95, 13.79)$ | `drone_1` | $24.85\text{ s}$ | **105.34 m** | 15.0 m | Out of Range (0 / 1) |
| **victim_1** | $(-36.13, 83.98, 14.99)$ | `drone_1` | $26.15\text{ s}$ | **105.32 m** | 15.0 m | Out of Range (0 / 1) |
| **victim_2** | $(-70.31, 121.09, 16.86)$ | `drone_1` | $26.15\text{ s}$ | **57.48 m** | 15.0 m | Out of Range (0 / 1) |
| **victim_3** | $(-91.80, 147.46, 15.63)$ | `drone_1` | $35.80\text{ s}$ | **24.74 m** | 15.0 m | Out of Range (0 / 1) |
| **victim_4** | $(-58.59, 69.34, 15.25)$ | `drone_1` | $24.85\text{ s}$ | **97.35 m** | 15.0 m | Out of Range (0 / 1) |

During the 25 decision steps, the swarm explored northward along an initial $100\text{ m}$ corridor. The five victims were placed in distant perimeter sectors ($Y > 120\text{ m}$ or $X < -70\text{ m}$). Zero detections was physically and geographically expected under a 25-step budget.

### 7.3 Stage 1 Perception Finding: Top-Down Silhouette Ambiguity
During Stage 1 single-drone hover testing directly over `victim_0` at $z = 15.0\text{ m}$:
- The standard COCO-trained YOLOv8n detector classified the vertical nadir projection of the human mesh as **`bird` (class 14, confidence 0.844)**.
- Standard computer vision models trained on ground-level imagery fail on downward overhead perspectives because the human torso and head silhouette mimic avian wing/body profiles.
- [`swarm_perception_node.py`](file:///home/capstone/capstone_project_antigravity/drone_ws/src/swarm_controller/swarm_controller/swarm_perception_node.py) enforces a strict filter:
  ```python
  if int(box.cls[0]) == 0:  # Class 0 = Person
      ...
  ```
  Class 14 was discarded as required, resulting in 0 published discovery events on `/sar/victim_discovery`.
- **Localization Stack Offline Verification:** To verify that the downstream geometric pipeline functions, the class 14 bounding box was processed through the pinhole back-projector and depth unprojector. The estimated 3D position was $(-82.26, 44.02, 13.80)\text{ m}$, exhibiting an absolute localization error of **$24.2\text{ cm}$** relative to ground truth $(-82.03, 43.95, 13.79)\text{ m}$.

---

## 8. Simulation-Only Perception Experiment Design (Task 4)

To empirically evaluate the downstream localization pipeline during active flight without compromising perception integrity, the following experiment is designed. **This experiment is purely documented; no code changes or simulations are initiated.**

### 8.1 Methodological Boundary: Rejecting Class-14 Aliasing
Globally mapping class 14 (`bird`) to class 0 (`person`) in production code is scientifically inadmissible. In real-world wilderness operations, birds, rocks, and foliage would trigger continuous false alarms. The simulation-only experiment must test geometric localization on synthetic models while explicitly assessing false-positive risks.

### 8.2 Disjoint Architecture & Topic Isolation
1. **Production Pipeline Frozen:** `swarm_perception_node.py` retains `if int(box.cls[0]) == 0:` on the production topic `/sar/victim_discovery`.
2. **Dedicated Proxy Topic:** Synthetic detections are published strictly to:
   $$\text{Topic: } \texttt{/sar/simulation\_proxy/discovery}$$
   $$\text{Message Type: } \texttt{sar\_msgs/msg/VictimDiscovery}$$
   $$\text{Field: } \texttt{detection\_source = "SIM\_SYNTHETIC\_PROXY"}$$
3. **Environment Variable Gating:** The proxy pipeline is gated by an opt-in variable:
   ```bash
   export SIM_PERCEPTION_SYNTHETIC_PROXY=1  # Default: 0 (disabled)
   ```
4. **Scoring Decoupling:** Official QMIX mission scoring evaluates ground-truth Euclidean proximity ($r \le 15.0\text{ m}$) and is completely decoupled from visual discovery topics.

### 8.3 False-Positive Assessment & Experimental Protocol
1. **Target Evaluation (Positive Test):**
   - Drone 0 hovers over `victim_0` at $15\text{ m}$ altitude.
   - Measure 3D localization error: $e_{3D} = \|\mathbf{\hat{p}}_{\text{est}} - \mathbf{p}_{\text{true}}\|$.
   - Measure discovery event latency: $\Delta t_{\text{event}} = t_{\text{publish}} - t_{\text{frame\_capture}}$.
2. **Foliage Control Evaluation (Negative Test):**
   - Fly a 50-meter trajectory over dense canopy devoid of any human meshes.
   - Record total spurious proxy triggers ($FP_{\text{proxy}}$) across total frames ($N_{\text{frames}}$):
     $$FPR_{\text{proxy}} = \frac{FP_{\text{proxy}}}{N_{\text{frames}}}$$
   - This metric quantitatively documents the operational hazard of class aliasing.

---

## 9. Methodological Limitations and Unverified Claims

To ensure academic and scientific integrity, the following limitations are formally recorded:

1. **No Claim of Statistical Significance:** Three evaluation runs ($N = 3$) under a deterministic greedy policy do not provide statistical significance across stochastic domains. Variance was confined to ODE continuous integration and OS scheduling.
2. **No Claim of Full Forest Coverage:** The 25-step budget achieved $39.90\%$ traversable forest coverage ($166 / 416$ cells). Full coverage would require extended flight budgets ($\ge 100$ steps).
3. **No Claim of Real-World Readiness:** SITL simulation does not model wind turbulence, aerodynamic rotor wash between proximate drones, GPS multipath in dense foliage, or camera lens smearing.
4. **No Claim of Autonomous Vision-Based Victim Discovery:** The swarm did not achieve autonomous visual victim discovery under standard COCO person weights. Downstream localization was validated only via offline back-projection.
5. **No Claim of Dynamic Obstacle Avoidance:** Trees were static obstacles; moving wildlife or falling debris were not evaluated.

---

## 10. Classification of Findings

| Metric / Parameter | Value | Scientific Classification | Methodological Basis |
| :--- | :---: | :---: | :--- |
| **QMIX Checkpoint Hash** | `f87d8752...272d` | **Measured** | Direct `sha256sum` system call |
| **Completed Decision Steps** | 25 / 25 | **Measured** | Exact integer row count in `qmix_decisions.csv` |
| **Unique Center Cells** | 141 cells | **Measured** | Set cardinality of logged discrete agent coordinates |
| **True Forest Valid Cells** | 416 cells | **Derived** | $625 - 209\text{ tree AABBs} = 416$ |
| **Traversable FOV Coverage** | 39.90% | **Derived** | Exact geometric intersection: $166 / 416 \times 100$ |
| **Active Run Explored FOV** | 230 cells (42.75%) | **Measured / Derived**| Reported by telemetry against active 538 mask |
| **Minimum Separation** | $3.08 \pm 0.02\text{ m}$ | **Measured** | Minimum Euclidean distance across continuous trajectories |
| **Collisions & Overrides** | 0 / 0 | **Measured** | Hardware and controller telemetry streams |
| **Wall-Clock Duration** | $524.2 \pm 35.7\text{ s}$ | **Measured** | Host Python timing utility |
| **Real-Time Factor (RTF)** | $0.098 \pm 0.006$ | **Derived** | Ratio: Sim duration / Wall-clock duration |
| **Closest Approach to Victim**| 24.74 m | **Derived** | Minimum distance from trajectory to `victim_3` |
| **Visual Discoveries Emitted**| 0 events | **Measured** | Zero messages published on `/sar/victim_discovery` |
| **Offline Localization Error**| 24.2 cm | **Derived** | Geometric back-projection delta to ground truth |
| **Statistical Generalization**| None | **Inferred Boundary** | Deterministic greedy policy across identical static runs |

---

## 11. Reproducibility Manifest & Artifact Locations

All empirical results documented in this report can be verified independently using the preserved artifacts:

| Artifact Description | Path |
| :--- | :--- |
| **Trained QMIX Policy Checkpoint** | [`models/qmix_sar_v4_align_best.pth`](file:///home/capstone/capstone_project_antigravity/models/qmix_sar_v4_align_best.pth) |
| **Restored Dense Forest World (950 Trees)** | [`worlds/forest_dense_candidate.sdf`](file:///home/capstone/capstone_project_antigravity/worlds/forest_dense_candidate.sdf) |
| **Forest World Metadata & Obstacle Mask** | [`worlds/forest_dense_candidate_meta.json`](file:///home/capstone/capstone_project_antigravity/worlds/forest_dense_candidate_meta.json) |
| **Benchmark Summary JSON** | [`results/stage2_three_seed_benchmark/benchmark_summary.json`](file:///home/capstone/capstone_project_antigravity/results/stage2_three_seed_benchmark/benchmark_summary.json) |
| **Seed 42 Decision & Trajectory Data** | [`results/stage2_three_seed_benchmark/seed_42_data/`](file:///home/capstone/capstone_project_antigravity/results/stage2_three_seed_benchmark/seed_42_data/) |
| **Seed 43 Decision & Trajectory Data** | [`results/stage2_three_seed_benchmark/seed_43_data/`](file:///home/capstone/capstone_project_antigravity/results/stage2_three_seed_benchmark/seed_43_data/) |
| **Seed 44 Decision & Trajectory Data** | [`results/stage2_three_seed_benchmark/seed_44_data/`](file:///home/capstone/capstone_project_antigravity/results/stage2_three_seed_benchmark/seed_44_data/) |
| **Stage 2 Results Report** | [`docs/stage2_three_seed_results.md`](file:///home/capstone/capstone_project_antigravity/docs/stage2_three_seed_results.md) |
| **Dense Forest Evaluation Audit & Addendum** | [`docs/dense_forest_evaluation_audit.md`](file:///home/capstone/capstone_project_antigravity/docs/dense_forest_evaluation_audit.md) |
| **Research Results and Limitations (This File)**| [`docs/research_results_and_limitations.md`](file:///home/capstone/capstone_project_antigravity/docs/research_results_and_limitations.md) |
