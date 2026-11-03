# MARL CAPSTONE: METHODOLOGY, EXPERIMENTAL CONTROLS & RESEARCH LIMITATIONS

## 1. Research Methodology & System Formulation

### A. Cooperative MARL Architecture
The search-and-rescue problem is formulated as a Decentralized Partially Observable Markov Decision Process (Dec-POMDP) $\langle S, U, P, r, Z, O, N, \gamma \rangle$:
- **Agent Count:** $N = 2$ autonomous multirotors navigating at fixed cruise altitude ($15.0\text{ m}$).
- **State Space:** Discrete 2D occupancy grid where cells are categorized into:
  - Unexplored ($0$)
  - Explored ($1$)
  - Static Obstacle ($-1$)
- **Action Space:** $U = \{0: +X, 1: -X, 2: +Y, 3: -Y, 4: \text{Hover}\}$.
- **Observation Representation:** Each drone receives a 29-dimensional local observation vector $\mathbf{o}_i \in \mathbb{R}^{29}$:
  - Normalized agent position: $(x / W, y / H) \in [0, 1]^2$ ($2$ features).
  - One-hot agent identification vector ($2$ features).
  - Normalized teammate position ($2$ features).
  - Teammate relative displacement vector $\Delta \mathbf{p} / \|\Delta \mathbf{p}\|_1$ ($2$ features).
  - Global Breadth-First Search (BFS) gradient vector towards the nearest reachable unexplored cell ($2$ features).
  - Local unexplored frontier cell density in a $5 \times 5$ window ($1$ feature).
  - Local $3 \times 3$ Obstacle FOV occupancy mask ($9$ features).
  - Local $3 \times 3$ Explored FOV status mask ($9$ features).
- **QMIX Value Factorization:**
  - Individual agent Q-values $Q_i(\tau_i, u_i)$ are modeled using a Gated Recurrent Unit (GRU) with 64 hidden units.
  - Joint action-value $Q_{\text{tot}}(\mathbf{s}, \mathbf{u})$ is approximated through a state-dependent mixing hypernetwork enforcing monotonicity:
    $$\frac{\partial Q_{\text{tot}}}{\partial Q_i} \ge 0, \quad \forall i \in \{1, \dots, N\}$$

---

## 2. Experimental Controls & Baseline Implementations

### A. Non-Learning Heuristic Baseline: Frontier Search
To establish a rigorous, fair baseline, an identical environment interface was provided to an obstacle-aware Frontier Search policy:
1. **Target Selection:** For each drone, the algorithm evaluates all currently unexplored cells ($\text{grid}[x, y] == 0$) and identifies the nearest frontier cell using Manhattan distance:
   $$\mathbf{t}^* = \arg\min_{\mathbf{t} \in \mathcal{U}} \|\mathbf{p}_i - \mathbf{t}\|_1$$
2. **Target Masking (Inter-Drone Dispersion):** When drone $i$ selects target $\mathbf{t}^*$, the target and its surrounding $3 \times 3$ footprint are added to a dynamic exclusion mask $\mathcal{M}$. Drone $j$ ($j \neq i$) is prohibited from targeting any cell within $\mathcal{M}$, enforcing cooperative spatial dispersion without neural communication.
3. **Obstacle-Aware Local Routing:** The selected action $a \in \{0, 1, 2, 3\}$ advances the drone towards $\mathbf{t}^*$ while strictly filtering out building or tree obstacles. If completely blocked, the drone hovers (Action 4).

### B. Lower-Bound Baseline: Random Walk
A uniform stochastic policy that samples randomly from feasible, collision-free moves at each step. This establishes the absolute performance floor attributable purely to Brownian spatial diffusion.

### C. Sensor Detection Semantics
- **Urban Environment (`realistic_sar`):** Cell size $L_{\text{cell}} = 4.0\text{ m}$. Isotropic proximity detection radius $r = 1.5 \times 4.0\text{ m} = \mathbf{6.0\text{ m}}$.
- **Wilderness Environment (`forest_sar`):** Cell size $L_{\text{cell}} = 8.0\text{ m}$. Isotropic proximity detection radius $r = 1.5 \times 8.0\text{ m} = \mathbf{12.0\text{ m}}$.
- **Detection Mechanism:** Continuous $20\text{ Hz}$ Euclidean distance monitoring:
  $$d_{\text{victim}} = \sqrt{(x_{\text{drone}} - x_{\text{victim}})^2 + (y_{\text{drone}} - y_{\text{victim}})^2} \le r$$
- **Deduplication:** A single detection event is persisted per victim using in-memory set tracking and SQLite `UNIQUE(mission_id, victim_id, detection_source)` constraints.

---

## 3. Methodological Clarifications & Contextual Nuances

### A. Forest Search Coverage Determinism
In the wilderness validation benchmark (`forest_sar`), both QMIX and Frontier Search yielded identical search coverage across all 5 seeds ($80.57\%$ and $36.26\%$, respectively, with sample standard deviation $s = 0.00\%$).
- **Explanation:** In `forest_sar`, both drones are initialized at the fixed physical base pads in Sector Alpha @ `(1, 20)` and `(6, 20)`. The tree obstacle layout is static. The local observation vector provided to the policy encodes explored status, obstacles, and frontier vectors—**it does not encode hidden victim locations**. Because the policy inference is fully deterministic, the drones execute the exact same spatial trajectory regardless of the random seed.
- **Why Seeds Still Matter:** The random seed governed stochastic victim generation and dynamic victim wandering (simulating disoriented survivors moving at $0.8\text{ m/s}$). Consequently, victim detection counts varied dynamically across seeds ($2/3$ to $3/3$ for QMIX; $0/3$ to $2/3$ for Frontier), validating discovery robustness against stochastic targets.

### B. City Multi-Seed Random Spawns vs. Fixed Live Staging Pad
- **Multi-Seed Benchmark:** To evaluate policy generalization across arbitrary spatial configurations, the benchmark initialized drone spawns uniformly at random across valid non-obstacle cells for each seed. Crucially, for any given seed, **all three policies (QMIX, Frontier, Random) started from identical initial coordinates**, guaranteeing complete comparative fairness.
- **Live Physics Mission:** In the live Gazebo/PX4 execution (Mission `398731d3-6e33-4b9e-ae2b-d33788d05806`), the drones launched from fixed physical takeoff pads @ $(21.0, 25.0)$ and $(29.0, 25.0)$, achieving $89.12\%$ coverage and $100\%$ victim recovery.

### C. Safety Overrides vs. Physical Collisions
- **Safety Overrides (Proactive Shield):** The multi-agent safety shield continuously intercepts proposed actions that would result in boundary escape, obstacle impact, or cell-swap crossing, substituting Action 4 (Hover).
- **Physical Collisions:** Physical impacts between vehicles or obstacles. In both the multi-seed benchmark and live SITL flight, **zero physical collisions occurred**. The 3 overrides logged per episode in `forest_sar` represent successful software interventions that prevented tree collisions.

---

## 4. Remaining Research Limitations

1. **Dead-End Alcove Avoidance in Urban Canyons:**
   In `realistic_sar`, QMIX achieves a mean coverage of $94.03\%$ rather than $100\%$. The unsearched $5.97\%$ ($\sim 22$ cells) consists of narrow, 1-cell-wide dead-ends between tall building colliders. The policy has learned that entering these high-penalty alcoves carries a severe risk of boundary or obstacle clipping that outweighs the marginal exploration reward.
2. **Circular Proximity vs. Square Cell Corners:**
   While an isotropic radius $r = 1.5 \times L_{\text{cell}}$ fully covers all 4 adjacent cardinal and ordinal cell midpoints, the outer diagonal corners of a $3 \times 3$ grid block extend to $d_{\text{corner}} = \sqrt{2} \times 1.5 \times L_{\text{cell}} \approx 2.12 \times L_{\text{cell}}$ ($8.49\text{ m}$ in city; $16.97\text{ m}$ in forest). A victim positioned in the extreme corner of an adjacent cell requires a closer approach to trigger detection.
3. **Concurrent Rendering Hardware Constraints:**
   Running simultaneous Gazebo physics, dual PX4 flight controllers, ROS 2 bridges, and multi-camera RGB-D image rendering on a laptop GPU (NVIDIA RTX 3050, 6GB VRAM) degrades the simulation real-time factor (RTF) to $\sim 0.25 - 0.30\times$. Headless physics execution with ground-truth spatial proximity monitoring remains essential for reproducible, non-stochastic control benchmarking on edge compute hardware.
