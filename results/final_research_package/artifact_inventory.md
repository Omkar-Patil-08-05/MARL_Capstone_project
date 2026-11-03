# MARL CAPSTONE: FINAL RESEARCH ARTIFACT INVENTORY

This document catalogs all verified research artifacts, data tables, manifests, and publication figures generated for the capstone project.

---

## 1. Verified Data & Results Artifacts
| Filename | Format | Description | Target Use Case |
| :--- | :---: | :--- | :--- |
| [`verified_results.json`](file:///home/capstone/capstone_project_antigravity/results/final_research_package/verified_results.json) | JSON | Machine-readable dataset containing recomputed sample and population statistics, raw episode arrays, and metadata. | Primary data source for analysis and validation. |
| [`results_tables.md`](file:///home/capstone/capstone_project_antigravity/results/final_research_package/results_tables.md) | Markdown | Thesis-ready comparative performance tables for `realistic_sar` and `forest_sar` with explicit sample standard deviations ($ddof=1$). | Copy-paste tables for thesis chapter 4 and paper submission. |
| [`methodology_and_limitations.md`](file:///home/capstone/capstone_project_antigravity/results/final_research_package/methodology_and_limitations.md) | Markdown | Rigorous documentation of Dec-POMDP formulation, observation space, baseline algorithms, methodological nuances, and limitations. | Thesis methodology chapter and defense preparation. |
| [`reproducibility_manifest.json`](file:///home/capstone/capstone_project_antigravity/results/final_research_package/reproducibility_manifest.json) | JSON | Complete system environment specification, checkpoint hashes, protected file checksums, and execution commands. | Artifact reproducibility badge and open-source audit. |

---

## 2. Publication-Quality Figures (300 DPI)
All figures are saved in [`results/final_research_package/figures/`](file:///home/capstone/capstone_project_antigravity/results/final_research_package/figures/) at 300 DPI resolution with clean styling, explicit sample sizes, error bars, and units:

| Figure File | Resolution | Key Contents | Intended Placement |
| :--- | :---: | :--- | :--- |
| `fig1_mean_coverage_comparison.png` | 2100x1500 (300 DPI) | Bar plot of mean search coverage across QMIX ($94.03\% \pm 4.31\%$), Frontier ($88.09\% \pm 16.09\%$), and Random Walk ($31.91\% \pm 6.55\%$) with sample std error bars. | Thesis: Results Overview / Slide: Swarm Performance |
| `fig2_victim_detection_rate.png` | 2100x1500 (300 DPI) | Overall victim discovery rate comparing QMIX ($93.3\%$, $28/30$), Frontier ($80.0\%$, $24/30$), and Random ($30.0\%$, $9/30$). | Thesis: Target Acquisition / Slide: Mission Efficacy |
| `fig3_per_seed_city_coverage.png` | 3000x1500 (300 DPI) | Grouped bar chart showing search coverage across all 10 independent seeds in `realistic_sar`, illustrating QMIX consistency vs Frontier variance collapses. | Thesis: Robustness & Variance Analysis |
| `fig4_per_seed_victim_detections.png` | 3000x1500 (300 DPI) | Target recovery counts per seed (out of 3), demonstrating that QMIX recovered 100% of victims in 9 out of 10 episodes. | Thesis: Per-Episode Deep-Dive |
| `fig5_forest_coverage_comparison.png` | 2100x1500 (300 DPI) | Wilderness SAR performance in complex organic terrain (`forest_sar`), highlighting QMIX's $80.57\%$ coverage vs Frontier's $36.26\%$ local-minima trapping. | Thesis: Generalization Chapter |
| `fig6_mission_success_rate.png` | 2100x1500 (300 DPI) | Mission success rate ($\ge 85\%$ coverage AND $3/3$ victims located) across policies: QMIX ($90.0\%$), Frontier ($60.0\%$), Random ($0.0\%$). | Thesis: Overall Evaluation Summary |

---

## 3. Underlying Raw Evaluation Datasets (Read-Only)
| Dataset Path | Rows / Episodes | Provenance | Integrity |
| :--- | :---: | :--- | :---: |
| `results/multi_seed_evaluation/multi_seed_raw_episodes.json` | 30 episodes | 10 seeds $\times$ 3 policies on `realistic_sar` | Frozen / Unmodified |
| `results/forest_sar_validation/forest_sar_validation_results.json` | 10 episodes | 5 seeds $\times$ 2 policies on `forest_sar` | Frozen / Unmodified |
| `dashboard/backend/results/antigravity.db` | 10 episodes | Authoritative historical physical simulation batch | Frozen / Unmodified |
| `models/qmix_sar_v4_align_best.pth` | 39,429 params | Primary trained neural checkpoint | Verified `f74edd5...` |
