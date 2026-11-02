# MARL CAPSTONE: AUTHORITATIVE BASELINE MANIFEST

## 1. Provenance & Identification
- **Manifest Name:** `MARL_SAR_QMIX_BASELINE_FREEZE`
- **Creation Timestamp:** `2026-10-21T16:58:00+05:30`
- **Git Revision:** `4ca781c5b35db897a33f3cfa2cbce10ce45f98ae`
- **Git Branch:** `main`
- **Authoritative Checkpoint Path:** `models/qmix_sar_v4_align_best.pth`
- **Checkpoint MD5:** `f74edd5763cb5e895836bbc58448fe2d`
- **Checkpoint SHA256:** `f87d87528969ff3165a0cdfb299f5d228ad99502b974176d05b81479242a272d`

---

## 2. Environment & Problem Geometry
- **Map Identifier:** `realistic_sar`
- **World File:** `worlds/realistic_sar.sdf`
- **Metadata File:** `worlds/generated_world_meta.json`
- **Grid Geometry:** $25 \times 25$ cells ($100.0\text{ m} \times 100.0\text{ m}$)
- **Cell Resolution:** $4.0\text{ m}$ per cell
- **Total Discrete Cells:** 625
- **Obstacle Building Cells:** 248 (derived from metadata bounding boxes)
- **Valid Searchable Cells ($N_{\text{valid}}$):** 377
- **Swarm Composition:** 2 PX4 SITL multirotors (`drone_0` @ $(21.0, 25.0)$, `drone_1` @ $(29.0, 25.0)$)
- **Victim Count:** 3 (1 static, 2 dynamic)
- **Random Seed:** 555
- **Cruise Altitude:** $15.0\text{ m}$
- **Detection Sensor Semantics:** Isotropic ground-truth proximity $r = 1.5 \times L_{\text{cell}} = 6.0\text{ m}$
- **Coverage Metric:** $\frac{\text{Explored Valid Cells}}{377} \times 100\%$

---

## 3. Operational Software Stack
- **OS:** Ubuntu 24.04 LTS (Kernel 6.8.0)
- **ROS 2:** Jazzy Jalisco (`/opt/ros/jazzy`)
- **Simulator:** Gazebo Harmonic (gz-sim 8)
- **Flight Controller:** PX4 Autopilot SITL v1.14+
- **Deep Learning Framework:** PyTorch 2.4.0 (CUDA 12.1 enabled)
- **Target GPU:** NVIDIA GeForce RTX 3050 Laptop GPU (6 GB VRAM)
- **Communication Bridge:** Micro-XRCE-DDS-Agent + `ros_gz_bridge`

---

## 4. Authoritative Frozen Baseline Run
- **Mission ID:** `398731d3-6e33-4b9e-ae2b-d33788d05806`
- **QMIX Decisions Executed:** 95 decisions
- **Search Duration:** 136.5 s (Sim) / 561.4 s (Wall)
- **Final Valid Cell Coverage:** **89.1247%** (336 / 377 cells)
- **Victim Detection Rate:** **100% (3 / 3 detected)**
- **Safety Overrides:** 0
- **Collisions:** 0
- **Persisted Telemetry Records:** 2,480 rows in SQLite
- **Detection Events:** Exactly 3 rows (unique per victim)

---

## 5. Artifact Paths
- **SQLite Database:** `dashboard/backend/results/antigravity.db`
- **QMIX Decision Log:** `dashboard/backend/results/h8_v4_physical_final/qmix_decisions.csv`
- **Victim Detection Log:** `dashboard/backend/results/h8_v4_physical_final/victim_detection.csv`
- **Drone Trajectories:** `dashboard/backend/results/h8_v4_physical_final/trajectory_drone_0.csv`, `trajectory_drone_1.csv`
- **Baseline JSON Manifest:** `baseline_manifest.json`
