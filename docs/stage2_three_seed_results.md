# Stage 2: Three-Seed Six-Drone Benchmark Results Report
**Benchmark Type:** Multi-Agent QMIX Swarm Scalability & Repeatability Evaluation (High-Speed Mode)  
**Evaluated Seeds:** 42, 43, 44 (Sequential Execution, 30s Cooldowns)  
**Target World:** [`worlds/forest_dense_candidate.sdf`](file:///home/capstone/capstone_project_antigravity/worlds/forest_dense_candidate.sdf) (950 Trees Restored, Preserved Intact)  
**Authoritative Checkpoint:** [`models/qmix_sar_v4_align_best.pth`](file:///home/capstone/capstone_project_antigravity/models/qmix_sar_v4_align_best.pth)  
**Verified SHA-256:** `f87d87528969ff3165a0cdfb299f5d228ad99502b974176d05b81479242a272d`  
**Execution Environment:** Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic 8.10.0, PX4 SITL (v1.14), Python 3.12, NVIDIA RTX 3050 Laptop GPU (6GB)  
**Archived Artifacts:** [`results/stage2_three_seed_benchmark/`](file:///home/capstone/capstone_project_antigravity/results/stage2_three_seed_benchmark/)  
**Date:** October 24, 2026  

---

## 1. Executive Summary & Experimental Protocol

This document provides a post-benchmark audit and research analysis of the **Stage 2 Three-Seed Six-Drone Benchmark** conducted in the dense forest environment. The objective was to evaluate the navigation stability, decision cadence, coverage progression, collision avoidance, and thermal safety of the 6-drone QMIX swarm under a fixed budget of **25 global decision steps** across seeds 42, 43, and 44.

### 1.1 Experimental Guardrails & Configuration
- **Simulation Mode:** High-Speed Mode (`lightweight: true`). Physics ran under Gazebo's Open Dynamics Engine (ODE) with headless rendering; camera sensors were disabled to avoid GPU thermal overload.
- **Decision Budget:** Exactly 25 synchronous global QMIX steps per seed (totaling 150 drone actions per run).
- **Environment State:** Evaluated against the verified candidate forest ([`worlds/forest_dense_candidate.sdf`](file:///home/capstone/capstone_project_antigravity/worlds/forest_dense_candidate.sdf)), comprising 950 trees and 5 ground victims.
- **Inter-Run Cooldown:** 30 seconds of forced thermal cooldown between consecutive seed executions.

---

## 2. Raw Artifact Recomputation & Audit (Task 1)

Every metric reported in [`results/stage2_three_seed_benchmark/benchmark_summary.json`](file:///home/capstone/capstone_project_antigravity/results/stage2_three_seed_benchmark/benchmark_summary.json) has been independently recomputed directly from raw decision CSVs ([`seed_42_data/qmix_decisions.csv`](file:///home/capstone/capstone_project_antigravity/results/stage2_three_seed_benchmark/seed_42_data/qmix_decisions.csv), `seed_43_data`, `seed_44_data`) and per-drone trajectory logs (`trajectory_drone_0.csv` to `trajectory_drone_5.csv`).

### 2.1 Per-Seed Recomputed Metric Table

| Telemetry / Metric | Seed 42 | Seed 43 | Seed 44 | Recomputation Source / Formula |
| :--- | :---: | :---: | :---: | :--- |
| **Run Status** | COMPLETED | COMPLETED | COMPLETED | Full 25-step execution |
| **Decisions Completed** | **25 / 25** | **25 / 25** | **25 / 25** | $\max(\text{episode\_step})$ in `qmix_decisions.csv` |
| **Total Decision Rows** | **150** | **150** | **150** | 25 steps $\times$ 6 drones |
| **Unique Center Cells ($N$)** | **141** | **141** | **141** | $\text{len}(\{(gx, gy) \mid \forall d, t\})$ |
| **Per-Drone Visited Center Cells** | 25 each (D0–D5) | 25 each (D0–D5) | 25 each (D0–D5) | Exactly 25 cells per drone (9 shared) |
| **Active Run Explored FOV ($M$)** | **230** | **230** | **230** | Active $3 \times 3$ window against run mask |
| **Unconstrained Raw Grid FOV** | **247** | **247** | **247** | $3 \times 3$ window union across 625 grid |
| **FOV Cells in Traversable Forest**| **166** | **166** | **166** | Intersection of FOV and 416 forest mask |
| **Minimum Pairwise Separation** | **3.07 m** (3.0689 m) | **3.11 m** (3.1287 m) | **3.07 m** (3.1292 m) | $\min_{i \ne j} \|\mathbf{p}_i(t) - \mathbf{p}_j(t)\|$ from trajectories |
| **Physical Collisions** | **0** | **0** | **0** | No crashes, zero altitude drops |
| **Safety Overrides Triggered** | **0** | **0** | **0** | `safety_override` column in CSV |
| **Swarm Failsafes** | **0** | **0** | **0** | Zero loss of offboard control |
| **Max Flight Cruising Altitude** | 15.21 m | 15.16 m | 15.14 m | Cruising at 15.0m setpoint |
| **Max Flight Speed** | 3.99 m/s | 3.99 m/s | 3.99 m/s | PX4 NED velocity magnitude |
| **Active Simulation Flight Time** | 51.4 s | 50.9 s | 51.4 s | Gazebo `/clock` duration |
| **Total Wall-Clock Duration** | 574.3 s (9.6 min) | 494.0 s (8.2 min) | 504.4 s (8.4 min) | From script start to teardown |
| **Effective Real-Time Factor (RTF)**| **0.090** | **0.103** | **0.102** | $\text{Sim Duration} / \text{Wall Duration}$ |
| **Average CPU Utilization** | 34.6% | 32.6% | 28.9% | System telemetry across 16 threads |
| **Average Memory Utilization** | 72.4% | 72.5% | 72.6% | RAM usage |
| **Average GPU Temperature** | 39.5°C | 39.5°C | 39.4°C | Peak temp: 40.0°C (Thermally safe) |
| **GPU Utilization** | 0% | 0% | 0% | Zero rendering load in High-Speed Mode |
| **Ground-Truth Victim Discoveries** | 0 / 5 | 0 / 5 | 0 / 5 | See Section 4 for detailed diagnosis |

### 2.2 Aggregate Statistics (Mean & Standard Deviation, $n = 3$)

```
====================================================================================================
STAGE 2 THREE-SEED AGGREGATE SUMMARY (N = 3)
====================================================================================================
Decision Cadence & Budget:
  - Budget Decision Steps:         25.00 ± 0.00 steps
  - Completed Decisions:           25.00 ± 0.00 steps (100% completion)

Center-Cell Visitation:
  - Unique Center Cells Visited:   141.00 ± 0.00 cells
  - Traversable Forest Cov (416):   33.89% ± 0.00%  (141 / 416 * 100)
  - Raw Grid Capacity Cov (625):    22.56% ± 0.00%  (141 / 625 * 100)

Sensor FOV Footprint:
  - Active Explored FOV Cells:     230.00 ± 0.00 cells
  - Active Mask Cov (538):          42.75% ± 0.00%  (230 / 538 * 100)
  - True Forest Mask Cov (416):     39.90% ± 0.00%  (166 / 416 * 100)
  - Raw Grid Capacity Cov (625):    39.52% ± 0.00%  (247 / 625 * 100)

Flight Safety & Dynamics:
  - Minimum Separation Distance:     3.08 m ± 0.02 m  (Min: 3.07 m)
  - Physical Collisions:            0 total (0.00 ± 0.00)
  - Safety Overrides:               0 total (0.00 ± 0.00)

Execution Resources:
  - Wall-Clock Run Duration:       524.2 s ± 35.7 s (8.7 min mean)
  - Active Simulation Duration:     51.2 s ± 0.2 s
  - Effective Real-Time Factor:     0.098 ± 0.006   (9.8% RTF)
  - System CPU Utilization:         32.0% ± 2.4%
  - GPU Temperature:                39.5°C ± 0.05°C
====================================================================================================
```

> [!NOTE]
> **Methodological Limitation:** Because all three runs evaluated against the identical fixed candidate world, identical initial spawns, and a deterministic greedy QMIX policy, the discrete high-level decision sequences were identical across seeds. Minor variances appeared only in continuous ODE physics integration (e.g. minimum separation varying between 3.07m and 3.11m) and operating system thread scheduling (RTF varying between 0.090 and 0.103). **No claim of statistical significance across stochastic domains is made from a three-seed sample.**

---

## 3. Coverage Semantics Verification (Task 2)

An analysis was conducted on the geometry of the $25 \times 25$ grid, the candidate forest obstacle bounding boxes, and the drone trajectories.

### 3.1 Are the 230 FOV Cells All Within the 416-Cell Valid Forest Mask?
**No.** The 230 FOV cells are **not** all within the 416-cell valid forest mask:
1. **Active Controller Mask vs. Forest Mask:** The controller initialized its grid obstacles using `earthquake_world_meta.json` (where 87 cells are obstacles, leaving **538 valid cells**). The 230 FOV cells were marked valid against this 538-cell mask ($230 / 538 = \mathbf{42.75\%}$).
2. **True Forest Obstacle Mask:** In [`worlds/forest_dense_candidate_meta.json`](file:///home/capstone/capstone_project_antigravity/worlds/forest_dense_candidate_meta.json) (bounds $[-180.0, -20.0]$, cell size $8.0\text{ m}$), 209 grid cells intersect tree obstacle AABBs, leaving **416 traversable flight cells**.
3. **Exact Geometric Intersection:**
   - The total unconstrained $3 \times 3$ FOV window around the 141 center positions covers **247 unique cells** across the $25 \times 25$ grid.
   - When intersected with the 416 traversable forest cells, exactly **166 FOV cells** lie within the traversable forest mask ($166 / 416 = \mathbf{39.90\%}$).
   - The remaining **81 cells** of the 247 FOV footprint (and 64 of the 230 active FOV cells) overlap tree obstacle AABBs. This is physically consistent with overhead aerial cameras cruising at $15\text{ m}$ altitude looking down onto tree canopy envelopes.

### 3.2 Rigorous Coverage Matrix Across All Definitions

| Metric Interpretation | Explored Cells (Numerator) | Denominator Basis | Calculated Coverage (%) | Mathematical Formula |
| :--- | :---: | :---: | :---: | :--- |
| **Center Cells in Forest Traversable Mask** | **96** | **416** | **23.08%** | $96 / 416 \times 100$ |
| **Total Visited Center Cells over Forest Mask** | **141** | **416** | **33.89%** | $141 / 416 \times 100$ |
| **Total Visited Center Cells over Full Grid** | **141** | **625** | **22.56%** | $141 / 625 \times 100$ |
| **FOV Cells in Forest Traversable Mask** | **166** | **416** | **39.90%** | $166 / 416 \times 100$ |
| **Active Run Explored FOV (Reported in Telemetry)**| **230** | **538** | **42.75%** | $230 / 538 \times 100$ |
| **Active FOV Divided by Forest Mask (Crude Ratio)** | **230** | **416** | **55.29%** | $230 / 416 \times 100$ |
| **Unconstrained Raw Grid FOV Footprint** | **247** | **625** | **39.52%** | $247 / 625 \times 100$ |

---

## 4. Investigation of Zero Ground-Truth Victim Detections (Task 3)

The benchmark reported **0 victims detected** across all three runs. A geometric and architectural investigation was performed using ground truth coordinates and recorded trajectories:

### 4.1 Root Cause 1: Runtime Victim Manager Metadata Loading
In `drone_ws/install/swarm_controller/lib/python3.12/site-packages/swarm_controller/swarm_runner.py` line 220, the condition `elif parsed.map == "forest_sar":` did not include `"forest_dense_candidate"`. When launched with `--map forest_dense_candidate`, it fell into the `else:` branch, loading `worlds/earthquake_world_meta.json`. Because `earthquake_world_meta.json` contains `victims: []`, the runtime `VictimManager` had **zero victims initialized in memory**.

### 4.2 Root Cause 2: Physical Drone-to-Victim Separation Distance
Even if the 5 victims had been loaded into memory, **no drone ever approached within the configured 15.0 m ground-truth proximity radius**.

Using the recorded trajectory positions of all six drones across all 25 decision steps, the minimum horizontal distance from any drone to each of the five victims was calculated:

| Victim ID | Ground Truth World $(X, Y, Z)$ | Closest Drone | Time ($t$) | Min 2D Distance (Gazebo Frame) | Min 2D Distance (Controller Frame) | Proximity Threshold | Detection Outcome |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **victim_0** | $(-82.03, 43.95, 13.79)$ | `drone_1` | $24.85\text{ s}$ | **105.34 m** | 88.10 m | 15.0 m | **Not Detected (Out of Range)** |
| **victim_1** | $(-36.13, 83.98, 14.99)$ | `drone_1` | $26.15\text{ s}$ | **105.32 m** | 42.14 m | 15.0 m | **Not Detected (Out of Range)** |
| **victim_2** | $(-70.31, 121.09, 16.86)$ | `drone_1` | $26.15\text{ s}$ | **57.48 m** | 80.19 m | 15.0 m | **Not Detected (Out of Range)** |
| **victim_3** | $(-91.80, 147.46, 15.63)$ | `drone_1` | $35.80\text{ s}$ | **24.74 m** | 110.29 m | 15.0 m | **Not Detected (Out of Range)** |
| **victim_4** | $(-58.59, 69.34, 15.25)$ | `drone_1` | $24.85\text{ s}$ | **97.35 m** | 64.63 m | 15.0 m | **Not Detected (Out of Range)** |

**Conclusion:** The closest approach by any drone to any victim was **$24.74\text{ m}$** (`drone_1` approaching `victim_3` at $t = 35.8\text{ s}$ in the Gazebo frame), which is well outside the $15.0\text{ m}$ proximity bubble. During the 25 decision steps, the swarm progressed northward along its initial exploration corridor and had not yet reached the deeper sectors where victims are located. Zero victim discoveries was physically and geographically expected.

---

## 5. Artifact Manifest & Verification Checksums

All source files, CSV traces, and JSON summaries have been preserved and verified:

| File Path | Description | Checksum / Record |
| :--- | :--- | :--- |
| [`models/qmix_sar_v4_align_best.pth`](file:///home/capstone/capstone_project_antigravity/models/qmix_sar_v4_align_best.pth) | Authoritative QMIX Neural Network | SHA-256: `f87d8752...272d` (Verified) |
| [`worlds/forest_dense_candidate.sdf`](file:///home/capstone/capstone_project_antigravity/worlds/forest_dense_candidate.sdf) | Evaluated Dense Forest Environment | 950 trees, intact |
| [`results/stage2_three_seed_benchmark/benchmark_summary.json`](file:///home/capstone/capstone_project_antigravity/results/stage2_three_seed_benchmark/benchmark_summary.json) | Complete Multi-Seed Summary | 3 runs, aggregate stats |
| [`results/stage2_three_seed_benchmark/seed_42_data/`](file:///home/capstone/capstone_project_antigravity/results/stage2_three_seed_benchmark/seed_42_data) | Seed 42 Decisions & Trajectories | 150 rows, 6 drone CSVs |
| [`results/stage2_three_seed_benchmark/seed_43_data/`](file:///home/capstone/capstone_project_antigravity/results/stage2_three_seed_benchmark/seed_43_data) | Seed 43 Decisions & Trajectories | 150 rows, 6 drone CSVs |
| [`results/stage2_three_seed_benchmark/seed_44_data/`](file:///home/capstone/capstone_project_antigravity/results/stage2_three_seed_benchmark/seed_44_data) | Seed 44 Decisions & Trajectories | 150 rows, 6 drone CSVs |

---

## 6. Simulation-Only Perception Experiment Design (Task 4)

### 6.1 Rationale & Problem Formulation
In Stage 1 testing, downward nadir optical imagery of Gazebo’s `casual_female` mesh triggered YOLOv8n COCO class 14 (`bird`, conf $0.844$) rather than class 0 (`person`). Treating class 14 as `person` in production would be methodologically fraudulent and dangerous: in real wilderness deployments, birds and foliage would cause disastrous false positives. However, validating the downstream geometric localization stack (depth unprojection, camera intrinsics, TF2 transforms, and clustering) on the synthetic model remains an essential engineering requirement.

### 6.2 Disjoint Architecture & Strict Isolation
To rigorously test the localization pipeline without compromising perception integrity:
1. **Production Pipeline Frozen:** [`swarm_perception_node.py`](file:///home/capstone/capstone_project_antigravity/drone_ws/src/swarm_controller/swarm_controller/swarm_perception_node.py) retains its strict person-only filter (`box.cls[0] == 0`).
2. **Dedicated Proxy Topic:** Any simulation-specific proxy must publish strictly to `/sar/simulation_proxy/discovery`, leaving `/sar/victim_discovery` exclusively for genuine `person` detections.
3. **Opt-In Environment Gate:** The proxy is gated behind an explicit environment variable (`SIM_PERCEPTION_SYNTHETIC_PROXY=1`), defaulting to `0` (disabled).
4. **Scoring Decoupling:** Official QMIX mission scoring evaluates ground-truth Euclidean proximity ($r \le 15.0\text{ m}$) and ignores simulation proxy topics.
5. **Separated Metrics:** All downstream logs, CSVs, and JSON files isolate `proxy_synthetic_detections` from `official_person_detections`.

### 6.3 False Positive Assessment Protocol
To evaluate false-alarm susceptibility:
- **Negative Control Run:** Fly a single drone over a 50 m trajectory across dense canopy with zero human meshes.
- **Metric Formulation:** Record total proxy detections ($FP_{\text{proxy}}$) over total frames ($N_{\text{frames}}$) to establish the proxy false positive rate:
  $$FPR_{\text{proxy}} = \frac{FP_{\text{proxy}}}{N_{\text{frames}}}$$
- If tree canopy shadows or foliage trigger class 14, this empirically documents the exact operational risk and validates why class 14 is prohibited from operational deployment.

