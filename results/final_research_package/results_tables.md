# MARL CAPSTONE: VERIFIED THESIS RESULTS TABLES

This document provides the authoritative, thesis-ready comparative performance tables derived directly from the raw evaluation records in `verified_results.json`.

---

## Table 1: Urban SAR Multi-Seed Benchmark (`realistic_sar`)
- **Map:** `realistic_sar` ($100.0\text{ m} \times 100.0\text{ m}$, $L_{\text{cell}} = 4.0\text{ m}$).
- **Valid Searchable Cells ($N_{\text{valid}}$):** $377$ cells (248 building obstacles excluded).
- **Proximity Sensor Radius:** $r = 1.5 \times L_{\text{cell}} = 6.0\text{ m}$.
- **Decision Budget:** 150 decisions maximum per episode.
- **Sample Size:** $N = 10$ independent deterministic random seeds ($101 \dots 110$).
- **Targets:** 3 victims per episode ($30$ total targets evaluated per policy).

| Performance Metric | QMIX (Trained Policy) | Frontier Search (Heuristic) | Random Walk (Lower Bound) | Definition & Verification Status |
| :--- | :---: | :---: | :---: | :--- |
| **Mean Search Coverage (%) [Sample $s$]** | **94.03% ± 4.31%** | 88.09% ± 16.09% | 31.91% ± 6.55% | Sample Std. Dev. ($ddof=1$, $N-1=9$) |
| **Mean Search Coverage (%) [Pop. $\sigma$]** | **94.03% ± 4.09%** | 88.09% ± 15.26% | 31.91% ± 6.21% | Population Std. Dev. ($ddof=0$, $N=10$) |
| **Coverage Range [Min, Max] (%)** | **[83.82%, 97.88%]** | [50.93%, 98.94%] | [15.92%, 40.58%] | Bounded dispersion across all 10 seeds |
| **Mean Victims Located (/ 3)** | **2.80 ± 0.63** | 2.40 ± 0.84 | 0.90 ± 0.57 | Targets detected per episode |
| **Victim Detection Rate (%)** | **93.33% (28 / 30)** | 80.00% (24 / 30) | 30.00% (9 / 30) | Denominator: 30 total targets |
| **All-Victims Intercepted Rate** | **90.00% (9 / 10)** | 60.00% (6 / 10) | 0.00% (0 / 10) | Frequency of locating 100% of victims |
| **Mean Decisions to 1st Victim** | 31.5 steps ($N=10$) | **24.0 steps ($N=10$)** | 51.4 steps ($N=8$) | Steps to initial target intercept |
| **Mean Decisions to All Victims** | 104.1 steps ($N=9$) | 78.2 steps ($N=6$)* | N/A ($N=0$) | *Conditioned only on successful episodes |
| **Mission Success Rate (%)** | **90.00% (9 / 10)** | 60.00% (6 / 10) | 0.00% (0 / 10) | Defined as: $\ge 85\%$ coverage AND 3/3 victims |
| **Mean Cumulative Path Length (m)** | 1177.2 m | 1110.4 m | 941.6 m | Sum of continuous waypoint displacement |
| **Mean Min Inter-Drone Distance** | 14.23 m | 12.42 m | 39.25 m | Minimum pairwise clearance |
| **Safety Overrides** | **0** | **0** | **0** | Proactive shield interventions |
| **Physical Collisions** | **0** | **0** | **0** | Inter-drone or obstacle impacts |
| **Mean Decision Latency (ms)** | **3.32 ms** (GPU) | 5.95 ms (CPU BFS) | 0.05 ms | Time per joint action computation |

*\*Note on Frontier Search Time-to-All-Victims: The 78.2-step metric is conditioned only on the 6 episodes where Frontier Search located all targets. In 40% of episodes (Seeds 103, 107, 108, 109), Frontier Search failed to locate all victims due to local minima trapping.*

---

## Table 2: Urban SAR Per-Seed Breakdown (`realistic_sar`)
*Detailed breakdown across all 10 independent evaluation seeds.*

| Random Seed | QMIX Coverage (%) | QMIX Victims (/3) | Frontier Coverage (%) | Frontier Victims (/3) | Random Coverage (%) | Random Victims (/3) |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **101** | 83.82% | 1 / 3 | **98.94%** | 3 / 3 | 32.63% | 1 / 3 |
| **102** | 95.76% | 3 / 3 | **96.82%** | 3 / 3 | 31.30% | 0 / 3 |
| **103** | **96.82%** | 3 / 3 | 79.31% | 2 / 3 | 15.92% | 0 / 3 |
| **104** | 93.90% | 3 / 3 | **97.88%** | 3 / 3 | 34.48% | 1 / 3 |
| **105** | 91.25% | 3 / 3 | **97.61%** | 3 / 3 | 37.93% | 1 / 3 |
| **106** | 96.82% | 3 / 3 | **98.67%** | 3 / 3 | 29.71% | 1 / 3 |
| **107** | **91.25%** | 3 / 3 | 90.19% | 2 / 3 | 33.95% | 1 / 3 |
| **108** | **97.88%** | 3 / 3 | 50.93% | 1 / 3 | 40.58% | 1 / 3 |
| **109** | **97.61%** | 3 / 3 | 71.88% | 1 / 3 | 32.10% | 1 / 3 |
| **110** | 95.23% | 3 / 3 | **98.67%** | 3 / 3 | 30.50% | 2 / 3 |
| **Mean** | **94.03%** | **2.80 / 3** | 88.09% | 2.40 / 3 | 31.91% | 0.90 / 3 |

---

## Table 3: Wilderness Cross-Environment Validation (`forest_sar`)
- **Map:** `forest_sar` ($200.0\text{ m} \times 200.0\text{ m}$, $L_{\text{cell}} = 8.0\text{ m}$, origin $[-180.0, -20.0]$).
- **Valid Searchable Cells ($N_{\text{valid}}$):** $422$ cells (203 dense tree/rock obstacles excluded).
- **Proximity Sensor Radius:** $r = 1.5 \times L_{\text{cell}} = 12.0\text{ m}$.
- **Decision Budget:** 150 decisions maximum per episode.
- **Sample Size:** $N = 5$ independent deterministic random seeds ($201 \dots 205$).
- **Launch Staging:** Fixed pads in Sector Alpha @ `(1, 20)` and `(6, 20)`.

| Validation Metric | QMIX (Trained Policy) | Frontier Search (Heuristic) | Difference / Advantage | Methodological Context |
| :--- | :---: | :---: | :---: | :--- |
| **Mean Coverage (%) [Sample $s$]** | **80.57% ± 0.00%** | 36.26% ± 0.00% | **+44.31% (QMIX)** | *Deterministic spatial search path |
| **Explored Cells Count** | **340 / 422 cells** | 153 / 422 cells | **+187 cells** | Fixed start pad & static obstacles |
| **Victim Detection Rate (%)** | **80.00% (12 / 15)** | 33.33% (5 / 15) | **+46.67% (QMIX)** | Stochastic dynamic victim wander |
| **Mean Victims Located (/ 3)** | **2.40 ± 0.55** | 1.00 ± 0.71 | **+1.40 targets** | Varying victim discoverability |
| **All-Victims Intercepted Rate** | **40.00% (2 / 5)** | 0.00% (0 / 5) | **+40.00% (QMIX)** | Intercepting all 3 forest victims |
| **Mean Min Inter-Drone Distance** | 48.7 m | 24.0 m | +24.7 m | Broader multi-sector dispersion |
| **Safety Overrides (/ Episode)** | 3.0 / ep | 0.0 / ep | Tree obstacle shield | Proactive Hover replacement |
| **Physical Collisions** | **0** | **0** | **0** | Zero vehicle or obstacle impacts |

*\*Note on Forest Coverage Path Determinism: Because both drones take off from the exact same staging pads `(1, 20)` and `(6, 20)`, the tree obstacle layout is fixed, and the policy observation vector does not encode hidden victim locations, both QMIX and Frontier Search follow the exact same deterministic waypoint paths across all seeds. Coverage is identical across seeds ($340$ cells vs $153$ cells). The random seed governs victim generation and dynamic movement, leading to varying detection outcomes ($2/3$ to $3/3$ for QMIX, $0/3$ to $2/3$ for Frontier).*

---

## Table 4: Wilderness SAR Per-Seed Breakdown (`forest_sar`)
*Detailed breakdown across all 5 cross-environment validation seeds.*

| Random Seed | QMIX Explored Cells | QMIX Coverage (%) | QMIX Victims Detected | Frontier Explored Cells | Frontier Coverage (%) | Frontier Victims Detected |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **201** | 340 / 422 | 80.57% | 3 / 3 | 153 / 422 | 36.26% | 2 / 3 |
| **202** | 340 / 422 | 80.57% | 3 / 3 | 153 / 422 | 36.26% | 1 / 3 |
| **203** | 340 / 422 | 80.57% | 2 / 3 | 153 / 422 | 36.26% | 1 / 3 |
| **204** | 340 / 422 | 80.57% | 2 / 3 | 153 / 422 | 36.26% | 1 / 3 |
| **205** | 340 / 422 | 80.57% | 2 / 3 | 153 / 422 | 36.26% | 0 / 3 |
| **Mean** | **340 / 422** | **80.57%** | **2.40 / 3** | 153 / 422 | 36.26% | 1.00 / 3 |
