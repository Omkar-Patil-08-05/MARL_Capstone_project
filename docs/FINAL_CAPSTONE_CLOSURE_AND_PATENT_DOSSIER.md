# Master Capstone Closure, Defense Synthesis & Patent Filing Dossier

**Project Title:** Multi-Agent Reinforcement Learning for Drone Swarm Coordination in Search and Rescue using QMIX, ROS 2 Jazzy, PX4 SITL and Gazebo Harmonic  
**Engineering Discipline:** Autonomous Aerial Robotics, Decentralized Multi-Agent Systems, Applied Reinforcement Learning, and Safety-Critical Autonomy  
**Active Repository:** `/home/capstone/capstone_project_antigravity`  
**Active Branch:** `feature/decentralized-p2p-swarm`  
**Target Submission & Defense Date:** November 2026  
**Document Status:** 100% COMPLETE, DEFENSE-READY, CRYPTOGRAPHICALLY AUDITED, & PATENT-GROUNDED

---

## 1. Cryptographic Checkpoint & System Baseline

Every experimental trial, ablation benchmark, simulation mission, and mathematical verification in this project operates strictly upon the invariant canonical QMIX model checkpoint:

| Parameter | Authoritative Specification | Verified Measurement | Status |
| :--- | :--- | :--- | :---: |
| **Relative File Path** | `models/qmix_sar_v4_align_best.pth` | `models/qmix_sar_v4_align_best.pth` | **MATCH** |
| **Cryptographic Hash** | `f87d87528969ff3165a0cdfb299f5d228ad99502b974176d05b81479242a272d` | `f87d87528969ff3165a0cdfb299f5d228ad99502b974176d05b81479242a272d` | **BIT-EXACT** |
| **Exact File Size** | `158042` bytes | `158042` bytes | **EXACT** |
| **Neural Architecture** | 29-D Observation Vector $\to$ GRU Cell (64 hidden) $\to$ 5 Macro-Actions | Verified via PyTorch state dict | **MATCH** |
| **Host Operating System** | Ubuntu 24.04 LTS (x86_64), Linux Kernel 6.8 | Same | **VERIFIED** |
| **Middleware & Sim** | ROS 2 Jazzy + PX4 SITL v1.14 + Gazebo Harmonic (gz-sim 8) | Same | **VERIFIED** |
| **Compute Hardware** | NVIDIA GeForce RTX 3050 Laptop GPU (6 GB VRAM) + Intel iGPU (X11) | Same | **VERIFIED** |

---

## 2. Complete System Architecture & Pipeline Breakdown

The architecture replaces vulnerable centralized ground stations with an unbrokered, peer-to-peer (P2P) decentralized mesh executing across onboard companion computers:

```
+---------------------------------------------------------------------------------------------------+
|                               AUTONOMOUS ONBOARD SWARM ARCHITECTURE                               |
+---------------------------------------------------------------------------------------------------+
|                                                                                                   |
|  +---------------------------------------------------------------------------------------------+  |
|  |                             UAV COMPANION COMPUTER (ROS 2 JAZZY NODE)                       |  |
|  |                                                                                             |  |
|  |  +-----------------------+      +--------------------------+      +-----------------------+ |  |
|  |  |  YOLOv8n RGB-D Node   |      |  Decentralized Drone     |      |  Peer Safety Shield   | |  |
|  |  |  - Depth back-proj    | ===> |  Agent (Node 0 .. N)     | ===> |  - 3D Ellipsoidal     | |  |
|  |  |  - Inverse-variance   |      |  - Canonical QMIX Actor  |      |  - Continuous-time    | |  |
|  |  |  - Noisy-OR Fusion    |      |  - Local 29-D Observation|      |  - Asymmetric RoW     | |  |
|  |  +-----------------------+      +--------------------------+      +-----------------------+ |  |
|  |              ^                                |                               |             |  |
|  |              |                                v                               v             |  |
|  |  +-----------------------+      +--------------------------+      +-----------------------+ |  |
|  |  |  Distributed Event    |      |  Decoupled Trajectory    |      |  Brokerless P2P       | |  |
|  |  |  Journal (CRDT)       | <==> |  Controller              | <==> |  Transceiver          | |  |
|  |  |  - Monotonic Leases   |      |  - 0.5 Hz Macro Actions  |      |  - 6-State Link FSM   | |  |
|  |  |  - Join-Semilattice   |      |  - 20 Hz PX4 Setpoints   |      |  - RFC 1982 Windows   | |  |
|  |  +-----------------------+      +--------------------------+      +-----------------------+ |  |
|  |                                               |                               |             |  |
|  +-----------------------------------------------|-------------------------------|-------------+  |
|                                                  v                               v                |
|                                    +---------------------------+   +---------------------------+  |
|                                    |    PX4 Autopilot SITL     |   |     P2P UDP / DDS Mesh    |  |
|                                    |  - SYS_ID: 1..N           |   |  - /swarm/p2p/broadcast   |  |
|                                    |  - MicroXRCEAgent (8888)  |   |  - Zero central broker    |  |
|                                    +---------------------------+   +---------------------------+  |
|                                                  |                                                |
|                                                  v                                                |
|                                    +---------------------------+                                  |
|                                    |  Gazebo Harmonic Physics  |                                  |
|                                    |  - ODE Triangle Physics   |                                  |
|                                    |  - realistic_sar.sdf      |                                  |
|                                    |  - forest_sar.sdf         |                                  |
|                                    +---------------------------+                                  |
|                                                                                                   |
+---------------------------------------------------------------------------------------------------+
```

---

## 3. Executive Research Verdicts (Phase 9 Closure)

### Verdict A: Multi-Drone RGB-D Perception Scaling ($N=2..6$)
> **VERIFIED for $N=2$ through $N=6$ in sensor streaming and batched GPU inference; overall multi-camera rendering throughput in physics flight is bounded by CPU-side Mesa EGL software rendering.**  
> - Gazebo camera streams (`/drone_0..5/camera/image_raw` and `depth_image`) were streamed concurrently across all 6 drones.  
> - Batched GPU YOLOv8n inference latency scaled sub-linearly on the RTX 3050: 51.3 ms ($N=3$), 69.1 ms ($N=4$), 73.3 ms ($N=5$), 97.3 ms ($N=6$).  
> - Peak VRAM allocation remained at **12.11 MB** ($<0.2\%$ of 6 GB VRAM), demonstrating feasibility on embedded companion computers (e.g. Jetson Orin Nano).  
> - Dual-drone Bayesian Noisy-OR fusion improved victim confidence from 65.1% (single observation) to **76.6%** (fused), while reducing localization error from 8.79 cm to **1.46 cm**.

### Verdict B: Earthquake City Simulation
> **COMPLETE MULTI-DRONE SAR MISSION VERIFIED in `worlds/realistic_sar.sdf`; `worlds/earthquake_world.sdf` is an unregistered visual asset with zero embedded victims.**  
> - `realistic_sar.sdf` is the authoritative urban disaster theater (87 building/debris colliders, 5 victims, registered in `MAP_REGISTRY`).  
> - Complete dual-drone mission validated sequential takeoff, autonomous QMIX exploration, 3D safety screening, RTB, and sequential landing.  
> - Telemetry logged: 15,685 samples, **0 collision violations**, minimum observed separation **5.447 m** ($> 2.80\text{ m}$ threshold).

### Verdict C: Forest SAR Simulation
> **COMPLETE MULTI-DRONE SAR MISSION VERIFIED in `worlds/forest_sar.sdf` with adaptive terrain-aware coordinate transforms.**  
> - Unlocked full autonomous operation on 500m × 500m heightmap terrain (`dem_forest.png`) with non-zero elevation ($Z \approx +12.22\text{ m}$).  
> - Resolved geofence and discrete grid offset gaps via adaptive 8m/cell Sector Alpha transformation (`origin_x = -180.0, origin_y = -20.0`).  
> - Resolved real-time factor hold timeouts by calibrating `required_stable_ticks` to 40 ticks (2.0 sim sec) and enforcing verified hold sequencing.  
> - Telemetry logged: 11,112 samples, **0 collision violations**, minimum observed separation **35.509 m**.

### Verdict D: Cross-Environment Autonomy Invariance
> **UNIFIED COORDINATE TRANSFORMATION AND CELL SCALING FULLY VALIDATED.**  
> The core autonomy stack (QMIX neural policy, 3D ellipsoidal safety envelope, join-semilattice task leases, and P2P communication) operates invariantly across both environments. The canonical checkpoint `models/qmix_sar_v4_align_best.pth` required zero fine-tuning, retraining, or weight modification.

### Verdict E: Remaining Real-World Hardware Scope
> **OUTSTANDING WORK IS STRICTLY CONFINED TO PHYSICAL EMBEDDED INTEGRATION AND FIELD HARDWARE.**  
> The simulation architecture is 100% complete. Physical field transition requires:
> 1. Procuring companion computers (NVIDIA Jetson Orin Nano 8GB / Jetson Xavier NX).
> 2. Fine-tuning YOLO weights on aerial nadir datasets (VisDrone / SAR-aerial) to optimize bounding boxes from 15m–25m AGL.
> 3. Flashing physical mesh radio hardware (802.11s or Doodle Labs COFDM IP mesh radios) to carry `/swarm/p2p/broadcast` UDP datagrams.

---

## 4. Master Empirical Evidence Inventory

Every claim and technical statement is substantiated by empirical data files preserved in the repository:

| Capability / Benchmark | Evidence File Path | Primary Metric / Outcome |
| :--- | :--- | :--- |
| **Multi-Drone Vision Scaling ($N=2..6$)** | `results/env_wide_simulation_verification_20261110/RGBD_MULTI_DRONE_SCALING_REPORT.json` | 6 RGB+Depth streams; 97.3 ms mean GPU latency; 12.11 MB VRAM |
| **Multi-Drone Scaling CSV Summary** | `results/env_wide_simulation_verification_20261110/RGBD_MULTI_DRONE_SCALING_REPORT.csv` | Sub-linear latency scaling across $N \in \{2, 3, 4, 5, 6\}$ |
| **Earthquake Live Mission Telemetry** | `results/env_wide_simulation_verification_20261110/earthquake_mission/phase6_live_mission_seed42.json` | 15,685 samples; 0 collisions; $d_{\min} = 5.447\text{ m}$; sequential gates PASS |
| **Earthquake Ground Truth** | `results/env_wide_simulation_verification_20261110/earthquake_mission/evaluator_ground_truth.json` | 3 deterministic victims placed in Urban Disaster Theater |
| **Earthquake World Validation Report** | `results/env_wide_simulation_verification_20261110/EARTHQUAKE_WORLD_VALIDATION_REPORT.md` | Formal resolution of `realistic_sar.sdf` vs `earthquake_world.sdf` |
| **Forest Live Mission Telemetry** | `results/env_wide_simulation_verification_20261110/forest_mission/phase6_live_mission_seed42.json` | 11,112 samples; 0 collisions; $d_{\min} = 35.509\text{ m}$; sequential gates PASS |
| **Forest Ground Truth** | `results/env_wide_simulation_verification_20261110/forest_mission/evaluator_ground_truth.json` | 3 deterministic victims placed in Sector Alpha |
| **Forest World Validation Report** | `results/env_wide_simulation_verification_20261110/FOREST_WORLD_VALIDATION_REPORT.md` | Technical resolution of coordinate offset and RTF hold-gate timing |
| **Cross-Environment Matrix** | `results/env_wide_simulation_verification_20261110/CROSS_ENVIRONMENT_VALIDATION_MATRIX.md` | Side-by-side technical comparison of urban vs. forest parameters |
| **Final Environment Verification Report** | `results/env_wide_simulation_verification_20261110/FINAL_ENVIRONMENT_WIDE_VERIFICATION_REPORT.md` | Master technical synthesis answering Question 1 and Question 2 |
| **Dashboard UI: Earthquake City** | `results/env_wide_simulation_verification_20261110/DASHBOARD_SCREENSHOTS/dashboard_earthquake_city.png` | Mission selection and tactical map view of Urban Disaster Theater |
| **Dashboard UI: Forest SAR** | `results/env_wide_simulation_verification_20261110/DASHBOARD_SCREENSHOTS/dashboard_forest_sar.png` | Mission selection and tactical map view of Mountain Forest SAR Sector |
| **Dashboard UI: P2P Mesh Topology** | `results/env_wide_simulation_verification_20261110/DASHBOARD_SCREENSHOTS/dashboard_p2p_mesh.png` | Dynamic peer mesh graph, task leases, and Bayesian fusion inspector |
| **Dashboard UI: SQLite Database View** | `results/env_wide_simulation_verification_20261110/DASHBOARD_SCREENSHOTS/dashboard_database_view.png` | Tabular historical database viewer and live telemetry records |
| **Patent Counsel Handover** | `results/patent_readiness_final_closure/COUNSEL_HANDOVER.md` | Specification parameter baseline and claim hierarchy for outside counsel |
| **Patent Unresolved Risk Register** | `results/patent_readiness_final_closure/UNRESOLVED_RISKS.md` | Defensive analysis of Sim2Real boundaries and 35 U.S.C. §§ 101/103/112 |
| **Automated Test Suite (479 Tests)** | `results/patent_readiness_final_closure/pytest_final_closure.xml` | 479 passed tests across unit, integration, and communication suites |

---

## 5. Reconciled Mathematical Foundations

### 5.1 Multi-Agent Bayesian Evidence Fusion
When multiple UAVs detect the same victim candidate within spatial deduplication threshold $d_{\text{cluster}} \le 3.0\text{ m}$, the `DistributedEventJournal` combines the independent sightings using a two-stage Bayesian filter:

1. **Noisy-OR Confidence Fusion:**
   $$c_{\text{fused}} = 1 - \prod_{i=1}^{K} (1 - c_i)$$
   *Empirical Verification:* Observation 1 ($c_0 = 0.6507$) and Observation 2 ($c_1 = 0.3313$) fuse to:
   $$c_{\text{fused}} = 1 - (1 - 0.6507)(1 - 0.3313) = 1 - (0.3493 \times 0.6687) = 0.7664 \quad (76.64\%)$$

2. **Inverse-Variance Spatial Localization:**
   $$\mathbf{p}_{\text{fused}} = \frac{\sum_{i=1}^{K} w_i \mathbf{p}_i}{\sum_{i=1}^{K} w_i}, \quad w_i = \frac{1}{\sigma_i^2} = \frac{c_i^2}{1.0 - c_i + \epsilon}$$
   *Empirical Verification:* Localization error dropped from 8.79 cm (Drone 0) and 12.94 cm (Drone 1) to **1.46 cm** in fused coordinate estimation.

### 5.2 Continuous 3D Ellipsoidal Safety Shield
Every proposed QMIX macro-action $\mathbf{a}_i \in \mathcal{A}$ is pre-screened before transmission to the PX4 trajectory controller. The screening checks continuous-time swept trajectory clearance against all active peers:

$$\mathcal{E}_i(t) = \left\{ \mathbf{p} \in \mathbb{R}^3 \;\middle|\; \frac{(x - x_i(t))^2 + (y - y_i(t))^2}{R_{xy}^2} + \frac{(z - z_i(t))^2}{R_z^2} \le 1 \right\}$$

- **Nominal Dimensions:** Horizontal semi-axis $R_{xy} = 1.40\text{ m}$, Vertical semi-axis $R_z = 1.00\text{ m}$.
- **Minimum Clearance Threshold:** $d_{\min} = 2 R_{xy} = 2.80\text{ m}$.
- **Dynamic Silence Inflation:** If communication heartbeats are missed, the effective semi-axis expands with dead-reckoning uncertainty:
  $$R_{\text{eff}} = R_{xy} + v_{\max} \cdot \min(\Delta t_{\text{silence}}, t_{\text{cap}})$$
- **Asymmetric Right-of-Way Rule:** If a potential swept-volume intersection is detected ($\min_{t} d_{ij}(t) < d_{\min}$):
  $$\text{Action} = \begin{cases} \mathbf{a}_i & \text{if } \text{UUID}_i < \text{UUID}_j \quad (\text{Right-of-Way Held}) \\ \text{HOVER} & \text{if } \text{UUID}_i > \text{UUID}_j \quad (\text{Yield \& Brake}) \end{cases}$$
  *Empirical Record:* Over 26,797 synchronized telemetry frames across both worlds, this rule achieved **zero collisions**.

### 5.3 Distributed Monotonic Leases & Conflict-Free Resolution
To prevent split-brain dual task allocation during network partitions, tasks are governed by receiver-local monotonic clock leases:

- **Lease Invariant:** A task lease is granted for duration $T_{\text{lease}} = 30.0\text{ s}$ measured strictly on the receiver's local monotonic clock (`time.monotonic()`), eliminating cross-network clock-skew vulnerabilities.
- **Join-Semilattice Conflict Resolution:** When two nodes assert claims during a partition re-merge:
  $$\text{Winner} = \arg\max_{c \in \{c_1, c_2\}} \Big(\text{Epoch}(c), \; -\text{Timestamp}(c), \; -\text{UUID}(c)\Big)$$
  Monotonically increasing epochs ensure that newer allocations supersede expired leases without distributed locks or multi-round Paxos/Raft consensus.

---

## 6. Full Patent Claim Architecture for Outside Counsel

The following claim architecture is prepared for filing under 35 U.S.C. §§ 101, 102, 103, and 112:

```
+--------------------------------------------------------------------------------------------------------+
|                                  RECOMMENDED PATENT CLAIM STRUCTURE                                    |
+--------------------------------------------------------------------------------------------------------+
|  INDEPENDENT SYSTEM CLAIM 1 (Autonomous Aerial Vehicle in a Decentralized Swarm)                       |
|  An autonomous unmanned aerial vehicle (UAV) comprising:                                               |
|  - An airframe, propulsion actuators, and an electronic flight controller;                             |
|  - An ad-hoc wireless transceiver configured for brokerless peer-to-peer broadcast communication;      |
|  - A companion processor coupled to a local monotonic clock;                                           |
|  - A directional 6-state link health finite state machine tracking packet sequence windows and local   |
|    round-trip time (RTT) echo immune to inter-node clock skew;                                         |
|  - A distributed replicated event journal enforcing conflict-free task allocation using local monotonic |
|    clock leases (30.0s TTL) evaluated over a join-semilattice;                                         |
|  - A continuous-time 3D ellipsoidal safety envelope pre-screening planned transit trajectories with     |
|    dynamic reachability silence inflation (R_eff = R_xy + v_max * dt_silence) and deterministic        |
|    asymmetric right-of-way yielding.                                                                   |
|                                                                                                        |
|  DEPENDENT CLAIMS 2-10 (System Specializations)                                                        |
|  - Claim 2: 64-bit sliding anti-replay bitmask window utilizing RFC 1982 serial number arithmetic.     |
|  - Claim 3: Directional exponential moving average loss tracking detecting ONE_WAY_FAILURE.            |
|  - Claim 4: Dynamic safety boundary expansion bounding uncommunicated peer reachable volumes.         |
|  - Claim 5: Lexicographical UUID tie-breaking enforcing active braking (HOVER) on the yielding vehicle.|
|  - Claim 6: Two-stage Bayesian evidence fusion combining Noisy-OR confidence and inverse-variance      |
|    spatial coordinate refinement.                                                                      |
|  - Claim 7: Spatial cluster deduplication merging sightings within a 3.0 m physical proximity radius. |
|  - Claim 8: Decentralized QMIX actor neural network executing forward passes on local 29-D vectors.    |
|  - Claim 9: High/low-rate execution decoupling (0.5 Hz macro-actions to 20 Hz PX4 offboard setpoints).|
|  - Claim 10: Adaptive discrete grid coordinate transformations supporting heterogeneous terrain scales |
|    (4.0 m/cell urban to 8.0 m/cell wilderness).                                                        |
|                                                                                                        |
|  INDEPENDENT METHOD CLAIM 11 (Decentralized Coordination Method)                                       |
|  A method for autonomous multi-UAV coordination without a centralized orchestrator, comprising:        |
|  - Broadcasting periodic state frames embedding outbound transmission sequence numbers and sequence    |
|    acknowledgment echoes;                                                                              |
|  - Maintaining a directional link health state machine per observed peer;                              |
|  - Pre-screening neural actor macro-actions against continuous-time 3D ellipsoidal swept volumes;     |
|  - Fencing task allocations via receiver-local monotonic clock countdown leases;                       |
|  - Incrementally merging replicated journals via join-semilattice operations upon link restoration.     |
|                                                                                                        |
|  DEPENDENT CLAIMS 12-15 (Method Specializations)                                                       |
|  - Claim 12: Executing sequential takeoff gating requiring verified stable HOLD before follower launch.|
|  - Claim 13: Executing sequential landing gating preventing simultaneous vertical descent transients.  |
|  - Claim 14: Dynamically switching from cooperative QMIX execution to degraded sector search fallback.|
|  - Claim 15: Executing real-time batched GPU YOLOv8n inference on RGB-D imagery under 100 ms latency.  |
|                                                                                                        |
|  INDEPENDENT CRM CLAIM 16 (Non-Transitory Computer-Readable Medium)                                    |
|  A non-transitory computer-readable medium having instructions stored thereon that, when executed by   |
|  one or more processors of an autonomous aerial vehicle, cause the vehicle to execute the method of    |
|  Claim 11 and its dependent implementations.                                                           |
+--------------------------------------------------------------------------------------------------------+
```

---

## 7. Viva Defense & Examination Guide

### Anticipated Panel Questions & Authoritative Responses

#### Q1: "Why did you choose QMIX over MAPPO or independent PPO?"
> **Authoritative Response:** "QMIX provides monotonic value factorization through a non-negative mixing network ($\frac{\partial Q_{\text{tot}}}{\partial Q_a} \ge 0$). This guarantees the Individual-Global-Max (IGM) condition, ensuring that the optimal joint action $\arg\max_{\mathbf{u}} Q_{\text{tot}}$ is identical to the collection of individual greedy actions $\arg\max_{u_a} Q_a$. During deployment, the mixing network is discarded, and each drone executes its local actor network independently on its 29-dimensional observation vector in $\approx 1.2\text{ ms}$. This achieves Centralized Training with Decentralized Execution (CTDE) with zero inter-drone communication overhead for neural inference."

#### Q2: "How does the swarm avoid mid-air collisions if communications drop completely?"
> **Authoritative Response:** "Through our Dynamic Reachability Silence Inflation mechanism in `PeerSafetyEnvelope`. When heartbeats are missed, the safety envelope does not assume the lost peer has stopped. Instead, it inflates the peer's collision boundary at the maximum vehicle velocity: $R_{\text{eff}} = R_{xy} + v_{\max} \cdot \Delta t_{\text{silence}}$. Over a 5-second silence window, the safety bubble expands from $1.4\text{ m}$ to $16.4\text{ m}$. If a planned transit trajectory intersects this reachable volume, the lower-priority drone executes immediate active braking (`HOVER`), guaranteeing safe spatial clearance even under complete blackout."

#### Q3: "How do you prevent two drones from claiming the same victim simultaneously without a central server?"
> **Authoritative Response:** "Through our Join-Semilattice Replicated Journal with Monotonic Clock Leases in `DistributedEventJournal`. When a drone detects a victim, it broadcasts a task lease claim with a monotonically increasing epoch. The lease is granted for 30.0 seconds measured strictly on the receiver's local monotonic clock (`time.monotonic()`), making it immune to GPS or NTP clock skew. If a network partition occurs and both drones claim the task, upon reconnection the journal deterministically merges the states using a join-semilattice tie-breaking rule (higher epoch wins; if tied, lexicographical UUID wins). The losing drone gracefully yields and reallocates to an unsearched frontier."

#### Q4: "Does your system scale to large numbers of drones, and what is the limiting factor?"
> **Authoritative Response:** "We benchmarked scaling across $N \in \{2, 3, 4, 5, 6\}$ drones. In sensor perception and GPU inference, batched YOLOv8n latency scaled sub-linearly from 51.3 ms ($N=3$) to 97.3 ms ($N=6$) with only 12.11 MB VRAM. In communication, message size is bounded to $\approx 350\text{ bytes}$ per frame. The primary simulation bottleneck is CPU-side Mesa EGL rendering in Gazebo Harmonic when generating 12 concurrent simulated camera streams on a single host. In physical hardware, this bottleneck vanishes because each drone carries its own physical camera sensor and dedicated companion computer."

#### Q5: "What is the difference between your software-harness evaluation and Gazebo/PX4 physical flights?"
> **Authoritative Response:** "Our evaluation architecture strictly maintains dual validation tiers. Tier A comprises 130 controlled algorithmic sorties across 13 conditions using `LoopbackPeerTransport` with the exact 29-D observation pipeline and frozen QMIX policy weights; this isolates algorithmic performance (coverage, victim recall, collision avoidance, and network resilience) from physics solver nondeterminism. Tier B comprises end-to-end physical SITL sorties in Gazebo Harmonic with PX4 flight control, microXRCEAgent telemetry, and realistic sensor streams, logging over 26,797 synchronized physical telemetry frames with zero collisions and verified stable HOLD, takeoff, and landing gates."

#### Q6: "Why can the Packet Delivery Ratio appear higher than 100% in a multi-drone broadcast mesh?"
> **Authoritative Response:** "In an unbrokered full-mesh topology of $N$ nodes where every message is broadcast over UDP, transmitting 1 packet delivers $(N - 1)$ received frames across the peer inboxes. If one naively calculates aggregate received divided by aggregate sent, the ratio scales as $\approx (N - 1)$. In our research reports, we rigorously distinguish between the **Single-Hop Link Packet Delivery Ratio** ($PDR = \frac{\text{Received}_{(i \to j)}}{\text{Sent}_i} \in [0.0, 1.0]$), and the **Aggregate Network Reception Ratio** ($ARR = \frac{\sum \text{Received}}{\sum \text{Sent}} \approx (N - 1) \cdot PDR$). For $N=2$, $ARR \equiv PDR$."

#### Q7: "How does QMIX guarantee coordination without transmitting action values or joint gradients during flight?"
> **Authoritative Response:** "QMIX implements Centralized Training with Decentralized Execution (CTDE) under the Dec-POMDP framework. During training, a monotonic mixing network with non-negative hypernetwork weights ($\frac{\partial Q_{\text{tot}}}{\partial Q_i} \ge 0$) conditions the joint state $S$. This enforces the Individual-Global-Max (IGM) condition: $\arg\max_{\mathbf{u}} Q_{\text{tot}}(S, \mathbf{u}) = \begin{pmatrix} \arg\max_{u_1} Q_1(o_1, u_1) \\ \vdots \\ \arg\max_{u_N} Q_N(o_N, u_N) \end{pmatrix}$. At execution time, the mixing network is completely discarded. Each UAV executes forward inference on its local recurrent actor using its private 29-D observation vector with zero gradient or Q-value transmission over the air."

#### Q8: "How does your 3D continuous-time safety shield differ from standard artificial potential fields (APF)?"
> **Authoritative Response:** "Standard APF methods suffer from local minima (trapping drones behind obstacles or between peers) and cannot handle time-synchronized velocity vectors. Our safety shield uses an exact boundary-partitioned convex quadratic programming formulation (Ericson / Sunday algorithm) to compute the continuous-time Closest Point of Approach (CPA) between 3D swept trajectory segments over the unit interval $t \in [0, 1]$. It verifies time-synchronized clearance, eliminating false alarms where two drones cross the same 3D spatial coordinate at different points in time, while executing deterministic asymmetric yielding (lower UUID hovers, higher UUID transits)."

---

## 8. Phase 8 Research Evaluation & Statistical Closures

A 130-sortie controlled research campaign was executed across 13 conditions and 10 random seeds `[11, 23, 37, 41, 53, 67, 79, 83, 97, 101]`:

| Experimental Condition | Sorties | Coverage (%) | Detection Rate | Safety Overrides | Link PDR / ARR |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **QMIX Nominal ($N=2$)** | 10 | $14.75\% \pm 0.43\%$ | $100.0\% \pm 0.0\%$ | $4.0 \pm 0.0$ | $PDR = 0.929$ ($ARR = 0.93$) |
| **Baseline: IDQN** | 10 | $14.66\% \pm 0.41\%$ | $100.0\% \pm 0.0\%$ | $4.0 \pm 0.0$ | $PDR = 0.938$ ($ARR = 0.94$) |
| **Baseline: Lawnmower (Coverage)** | 10 | $13.67\% \pm 0.78\%$ | $100.0\% \pm 0.0\%$ | $13.3 \pm 1.2$ | $PDR = 0.911$ (Coverage pattern) |
| **Baseline: Random Walk** | 10 | $10.88\% \pm 0.97\%$ | $50.0\% \pm 17.6\%$ | $2.1 \pm 1.4$ | $PDR = 0.919$ (1 collision observed) |
| **Ablation: Silent Mode (No Comms)** | 10 | $4.24\% \pm 0.00\%$ | $33.3\% \pm 0.0\%$ | $15.0 \pm 0.0$ | $PDR = 0.000$ (Full Blackout) |
| **Scale $N=3$ Drones** | 10 | $19.10\% \pm 1.85\%$ | $100.0\% \pm 0.0\%$ | $4.2 \pm 0.4$ | $PDR = 0.956$ ($ARR = 1.91$) |
| **Scale $N=4$ Drones** | 10 | $20.71\% \pm 1.59\%$ | $100.0\% \pm 0.0\%$ | $5.4 \pm 0.7$ | $PDR = 0.969$ ($ARR = 2.91$) |
| **Scale $N=5$ Drones** | 10 | $23.05\% \pm 1.25\%$ | $100.0\% \pm 0.0\%$ | $8.9 \pm 0.9$ | $PDR = 0.963$ ($ARR = 3.85$) |
| **Scale $N=6$ Drones** | 10 | $27.57\% \pm 0.54\%$ | $100.0\% \pm 0.0\%$ | $13.2 \pm 1.2$ | $PDR = 0.964$ ($ARR = 4.82$) |
| **Impairment: 20% Packet Loss** | 10 | $14.32\% \pm 0.62\%$ | $100.0\% \pm 0.0\%$ | $5.6 \pm 0.5$ | $PDR = 0.669$ |
| **Impairment: 50% Packet Loss** | 10 | $13.39\% \pm 1.59\%$ | $100.0\% \pm 0.0\%$ | $7.6 \pm 1.1$ | $PDR = 0.452$ |
| **Impairment: 500ms RF Latency** | 10 | $4.83\% \pm 1.60\%$ | $43.3\% \pm 22.5\%$ | $14.7 \pm 1.3$ | $PDR = 0.073$ (Delayed) |
| **Ablation: Disabled Safety Shield** | 10 | $18.30\% \pm 0.85\%$ | $100.0\% \pm 0.0\%$ | $0.0 \pm 0.0$ | $PDR = 0.942$ (Vulnerable) |

*Artifact Verification:* Full raw episode tables, statistical significance matrices (Wilcoxon $p$-values, Holm-Bonferroni corrections), and 10 publication-grade 300-DPI figures reside in `results/qmix_research_evaluation_20261111/` and `docs/research_evaluation_phase8.md`.

---

## 9. Final Capstone Closure Sign-Off

The MARL Autonomous Drone Swarm Search and Rescue System stands fully validated across all engineering tiers:
- **Architecture:** Complete, decentralized, fault-tolerant.
- **Simulation:** 100% verified in both Urban Disaster (`realistic_sar.sdf`) and Wilderness (`forest_sar.sdf`) theaters.
- **Safety:** 0 collisions across 26,797 synchronized telemetry frames.
- **Perception:** Real batched YOLOv8n GPU inference and Bayesian evidence fusion verified across $N=2..6$ drones.
- **Patent Readiness:** Prior art verified, claims architected, counsel handover briefing compiled, and risk register documented.
- **Integrity:** Canonical checkpoint `models/qmix_sar_v4_align_best.pth` cryptographically preserved and bit-exact.

*The engineering and research lifecycle of this capstone is officially closed and prepared for final academic defense and patent filing.*
