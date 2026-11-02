# PROJECT MASTER TECHNICAL RECORD

**Project:** Multi-Agent Reinforcement Learning for Drone Swarm Coordination in Search and Rescue using QMIX, ROS 2 Jazzy, PX4 and Gazebo Harmonic.  
**Repository:** `~/capstone_project_antigravity`  
**Classification:** INTERNAL PROJECT ENCYCLOPEDIA (HONEST / HIGH-DETAIL)

---

## I. PROJECT IDENTITY AND MOTIVATION

### 1. Project Identity
This project serves as the capstone technical deliverable for multi-agent reinforcement learning applied to swarm robotics. It is housed in the `~/capstone_project_antigravity` repository, unifying Python-based ML frameworks, ROS 2 robotics middleware, PX4 flight controllers, and full-stack web technologies.

### 2. Original Motivation
The project originated from the challenge of translating abstract, grid-based Reinforcement Learning (RL) policies into continuous, physics-bound real-world robotic execution. While many papers present MARL in grid-worlds, few successfully implement these policies on a complete Software-In-The-Loop (SITL) stack using vision as the primary sensory modality.

### 3. Research Question
How can multiple autonomous UAVs coordinate efficiently for disaster search-and-rescue using cooperative multi-agent reinforcement learning while utilizing onboard visual perception to locate victims?

---

## II. REINFORCEMENT LEARNING DEVELOPMENT

### 4. Initial Architecture
The initial architecture was a pure Python simulation to prove out the RL algorithms without the overhead of Gazebo physics. It focused solely on the algorithmic challenge of cooperative exploration.

### 5. Initial Training Environment
Initial training began on a trivial 10x10 obstacle-free grid. The sole objective was to train a simple Q-Learning agent to explore the space efficiently. This established the baseline for the reward structure.

### 6. Training Progression
The curriculum advanced from 10x10 empty grids to 25x25 grids with static obstacles. Swarm sizes were stepped from 2 agents up to the final 6 agents.

### 7. SARGridEnv
The custom Python environment built using OpenAI Gym interfaces. It handles discrete movements, obstacle collision checks, and tracks the global coverage metric used for QMIX rewards.

### 8. 25×25×3 Representation
The final training state space is a 25x25 spatial grid consisting of 3 distinct channels:
1. Agent locations (1-hot encoding of drone presence).
2. Obstacle map (static binary masking).
3. Explored coverage map (binary mask of previously visited cells).

### 9. Observation Space
A localized Field of View (FOV). Drones do not receive the global state; they only receive a slice of the 3-channel state corresponding to their immediate surroundings.

### 10. Action Space
Discrete 5-action space: North, South, East, West, Hover.

### 11. Reward Function
The global reward signal ($r_t$) is driven purely by the marginal increase in global coverage ($\Delta C_t$). Heavy negative penalties are applied if an agent attempts an invalid action (e.g., crashing into a boundary or obstacle), discouraging reckless policies.

### 12. QMIX Implementation
Decentralized agent utility networks (RNNs) feed into a centralized monotonic mixing network. The system leverages `pymarl` mechanics adapted for grid-based continuous exploration.

### 13. Training Configuration
Trained using an experience replay buffer with $\epsilon$-greedy exploration decaying over time. Centralized Training with Decentralized Execution (CTDE) allows the hypernetwork to utilize the global state during training.

### 14. Checkpoint History
Multiple models were saved. The authoritative checkpoint successfully exported and utilized for physics execution is `qmix_sar_v4_align_best.pth`.

---

## III. SCALABILITY AND TRANSITION

### 15. Transition from Grid to Gazebo
The jump from a 25x25 discrete grid to a continuous physics simulator required a translation layer. We adopted a strict 4m/cell mapping. The discrete actions (e.g., "North") are converted into velocity setpoints aimed at the center of the adjacent 4x4 meter world block.

### 16. 2-Drone Development
The foundational physical integration phase. Proved the viability of connecting the QMIX policy to PX4 setpoints over ROS 2. Validated with 2-drone trajectory evaluations in RViz.

### 17. 3-Drone Development
Explored intermediate scaling using hybrid orchestrator configurations to validate the safety shield and trajectory execution logic before full decentralized rollout.

### 18. 4-Drone Hybrid
Continued hybrid configuration validation for 4 drones, stressing the ROS 2 message queues and DDS routing.

### 19. Skipped 5-Drone Stage
Explicitly **not evaluated**. Development moved directly from N=4 to N=6 to rapidly test the hardware limit of the laptop.

### 20. 6-Drone QMIX
The final QMIX target size. Replaced the hybrid coordinator with the full decentralized QMIX policy for 6 drones. Scalability and integration testing was successful, validating communication routing.

---

## IV. ROBOTICS & SIMULATION STACK

### 21. ROS 2
ROS 2 Jazzy serves as the middleware backbone, providing publisher/subscriber interfaces for telemetry, commands, and camera streams.

### 22. PX4
PX4 Autopilot SITL executes the actual drone flight dynamics in Offboard mode.

### 23. Namespaces
Strict namespacing (`/drone_0`, `/drone_1`) isolates odometry, status, setpoints, and camera topics, completely eliminating cross-talk between the 6 agents.

### 24. Gazebo
Gazebo Harmonic simulates the rigid body physics, gravity, wind, and sensor rendering.

### 25. World Generator
A Python-based procedural generation script (`generate_world.py`) that constructs the urban disaster scene.

### 26. realistic_sar
The authoritative 3D simulated disaster environment generated by the script, saved as `realistic_sar.sdf`.

### 27. Obstacles
The world generator produces structural rubble and walls parsed directly into both the SDF file and a JSON metadata file.

### 28. Coordinate System
The global world origin (0,0) maps to the grid origin (0,0). All drone starting positions and victim locations are strictly aligned to the 4m/cell grid bounds.

### 29. Victims
3D human models (e.g., `casual_female`) placed into the Gazebo world to serve as visual targets for the YOLO network.

### 30. Dynamic Victim Generation
The world generator dynamically injects a specified number of victims into random valid (non-obstacle) coordinates, writing these coordinates to the JSON metadata to support randomized SAR evaluations.

### 31. Drone Starting Positions
Drones are spawned at specific, conflict-free base coordinates designated in the JSON metadata.

### 32. Landing/Search Zones
The world encompasses a 100x100 meter search zone. Safe landing protocols trigger when the mission completes.

### 33. Mission Controller
The central ROS 2 node bridging the QMIX policy inference, safety shield evaluation, and PX4 publisher loop.

### 34. Safety Shield
A deterministic, rule-based temporal occupancy grid that intercepts discrete QMIX actions, preventing continuous-space collisions. Unsafe actions are overridden with a "Hover" command. The evaluation logs recorded a mean of 1290 hover commands and zero invalid actions.

---

## V. FULL-STACK DASHBOARD

### 35. Dashboard
A full-stack interface (React frontend, FastAPI backend) providing real-time telemetry rendering and mission control.

### 36. Telemetry
ROS 2 topics are aggregated by the backend and streamed via WebSockets to the frontend at high frequencies.

### 37. Heatmap
The frontend dynamically renders a coverage heatmap, visually indicating which 4x4m cells have been explored by the swarm's FOV.

### 38. Database
SQLite implementation using Write-Ahead Logging (WAL) and background thread queueing to ensure high-frequency telemetry doesn't lock the database.

### 39. Experiment Storage
The schema tracks `experiments`, `missions`, `episodes`, `drones`, `victims`, `detection_events`, `telemetry`, `performance_samples`, and `safety_events` for full historical playback.

---

## VI. PERCEPTION SUBSYSTEM

### 40. Camera
Simulated downward-facing cameras attached to the `x500` drone models.

### 41. RGB
Provides synchronized RGB arrays for object detection.

### 42. RGB-D
Provides depth maps (16-bit float) for geometric localization.

### 43. YOLO
Runs on the RGB stream to identify victims. Detection triggers bounding box coordinates and confidence scores.

### 44. Victim Localization
Projects pixel bounding boxes onto a 3D coordinate frame using the median depth-map value to filter ground noise.

### 45. Perception Experiments
Due to hardware limits on end-to-end multi-drone rendering, perception was validated via isolated controlled experiments ensuring the RGB-D bounding box geometric projection functions correctly.

### 46. Six-Camera Tests
Encountered immediate hardware throttling. The RTX 3050 cannot maintain 6 high-resolution concurrent RGB-D Gazebo renders + YOLO inference + Physics.

### 47. Computational Limitations
Real-Time Factor (RTF) dropped significantly with multiple cameras. Python DDS overhead spiked, causing synchronization timeouts.

---

## VII. EXPERIMENTAL RECORD

### 48. Physical Experiments
Missions were physically evaluated in Gazebo utilizing a ground-truth proximity detector (3.5m radius) to bypass the laptop's rendering limitations and rigorously isolate the QMIX coverage efficiency.

### 49. H4
Historical hybrid run (2 drones long duration).

### 50. H6
Historical hybrid run (2 drones).

### 51. H7
Historical hybrid run.

### 52. H7 final
Historical hybrid run.

### 53. H8
Historical 6-drone run.

### 54. H8 final
The 6-drone lightweight scalability integration test. Aborted at 29.44% coverage with a max inference latency of 74.6 seconds. This was NOT a complete SAR mission.

### 55. QMIX Episodes 11–20
The authoritative, CSV-backed evaluation dataset proving the 90.08% stable coverage of the final QMIX 6-drone architecture.

### 56. Historical Episodes 1–10 Discrepancy
Episodes 1-10 are referenced in older state files but are NOT fully backed by the authoritative raw CSVs. They must not be fabricated or claimed as rigorous evidence.

### 57. Failed Experiments
Attempts to run the full 6-drone RGB-D pipeline end-to-end resulted in massive simulation slowdowns and DDS timeout crashes.

### 58. Invalid Experiments
The temporal grid-voting continuous-error metric was determined to have a metric-definition problem. This must not be presented as a validated positive result.

### 59. Discarded Approaches
Collision counts are unavailable in the authoritative CSV and thus cannot be rigorously claimed as a metric. We rely on the "zero invalid actions" logging metric.

### 60. Current Validated Baseline
- 10 evaluated physical simulation episodes (Episodes 11-20).
- Mean coverage: 90.08%.
- Victims found: 5/5 across all evaluated runs using 3.5m proximity detection.
- Invalid actions/timeouts: 0.

---

## VIII. REPOSITORY HYGIENE

### 61. Git Checkpoints
Repository is synchronized. The `gazebo_evaluation_clean/` artifacts are intentionally `.gitignore`'d.

### 62. Current Repository Structure
Clean and fully modularized. PX4 submodule (`px4_msgs`) is clean.

### 63. Current Repository State
Zero tracked modifications. Unrelated generated artifacts ignored.

---

## IX. ACADEMIC & PRESENTATION PLANNING

### 64. Current Research Claims
Demonstrated scalable transition of grid-trained MARL policies to continuous robotics physics utilizing safety middleware. Validated YOLO/RGB-D geometry through controlled perception experiments.

### 65. Forbidden Claims
- Do NOT claim a five-drone evaluation occurred.
- Do NOT claim 20 raw episodes (only Episodes 11-20 are backed by CSV).
- Do NOT claim the final six-drone lightweight run was a complete SAR mission.
- Do NOT claim that the 5/5 victims found in the final batch were detected end-to-end by YOLO (they were proximity-evaluated to bypass rendering bottlenecks).
- Do NOT claim "collision-free flight" (authoritative collision data is unavailable in the CSV). Claim "zero invalid actions" instead.
- Do NOT treat the invalid temporal grid-voting metric as a validated positive result.

### 66. Thesis-Ready Results
- **Coverage Stability:** 90.08% mean coverage across 10 distinct, heavily varied physical simulation runs.
- **Safety Reliability:** 0 invalid actions and 0 timeouts logged by the temporal occupancy shield.
- **Architectural Scalability:** Successfully bridging discrete grid RL (QMIX) to continuous-physics ROS 2/PX4 setpoints up to 6 drones.
- **Vision Pipeline:** Accurate geometric translation of 2D bounding boxes and depth maps into 3D world coordinates.

### 67. Presentation-Ready Results
- The conceptual architecture diagram showing the exact flow from Camera → YOLO → Localization → QMIX → Gazebo.
- The Dashboard UI demonstrating real-time telemetry, trajectory mapping, and SQLite database persistence.
- Graphing the 90.08% coverage plateau over the 229.9s mean duration to show exploration efficiency.

### 68. Viva Preparation
Be prepared to defend the use of proximity evaluation for the QMIX coverage metrics as a necessary, completely valid hardware compromise (RTX 3050 limits). Emphasize that the *architecture* for vision is fully implemented and mathematically validated through the isolated perception experiments.

### 69. Earthquake-World Plan
**FUTURE TASK.** Expand the world generator to produce more complex, non-uniform rubble structures mimicking severe seismic damage. This environment is NOT currently validated.

### 70. Final Evaluation Plan
**FUTURE TASK.** Conduct a massive final batch of headless physical simulations in the new earthquake world to finalize the dataset.

### 71. Thesis Roadmap
Structure the thesis sequentially: Grid MARL Theory $\rightarrow$ ROS 2/PX4 Integration $\rightarrow$ Vision Perception Validation $\rightarrow$ Full Swarm Proximity Evaluation.

### 72. Demonstration Roadmap
Record high-quality videos of 2-drone and 3-drone fully visual runs, and present the 6-drone run via the Dashboard telemetry replay to bypass rendering limits during live demos.

### 73. Remaining Risks
The transition to a highly complex earthquake environment may expose navigation limitations in the discrete QMIX grid tracking.

---

## X. DETAILED PROGRESSION TABLES

### TABLE A — COMPLETE PROJECT TIMELINE
| Date/Phase | Development | Environment | Swarm | Result | Problem | Next Improvement |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| Phase 1 | Initial QMIX Setup | 10x10 Grid | 2 | Convergence | Policy too simplistic | Increase grid size |
| Phase 2 | Complex Grid RL | 25x25x3 | 6 | 90%+ Coverage | Abstracted physics | ROS 2 Integration |
| Phase 3 | PX4 Integration | Gazebo | 2 | Flight Success | Single namespace | Namespacing / DDS |
| Phase 4 | Hybrid Scaling | Gazebo | 3 & 4 | Route Success | Centralized limit | Decentralize to QMIX |
| Phase 5 | 6-Drone QMIX | Gazebo | 6 | Boot Success | Severe RTF drop | Hardware offloading |
| Phase 6 | Dashboard/DB | React/SQLite | N/A | Telemetry logging | N/A | N/A |
| Phase 7 | Perception Tests | Gazebo | 1 | YOLO Math Valid | N/A | Full Mission Batch |
| Phase 8 | Physical Batch | Gazebo | 6 | 90.08% Coverage | Missing CSV data | Final Audit |

### TABLE B — RL EVOLUTION
| Version | Environment | Agents | Observation | Actions | Reward | Result |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| v1 | 10x10 Empty | 2 | Global | 4-way | Absolute Coverage | Baselines set |
| v2 | 25x25 Obstacles | 4 | Local FOV | 4-way | Marginal Coverage | Failed edge cases |
| v3 | 25x25x3 | 6 | Local FOV | 5-way (Hover) | $\Delta C_t - Penalty$ | Strong policy |
| v4 (Best) | 25x25x3 | 6 | Local FOV | 5-way (Hover) | $\Delta C_t - Penalty$ | Exported to PX4 |

### TABLE C — SWARM EVOLUTION
| Stage | Drones | Controller | Architecture | Validation | Limitation |
| :--- | :--- | :--- | :--- | :--- | :--- |
| 1 | 2 | QMIX | Decentralized | Physical Gazebo | Too small for heavy SAR |
| 2 | 3 & 4 | Hybrid | Centralized | Physical Gazebo | Not using final MARL |
| 3 | 5 | N/A | N/A | Skipped | Hardware stress test needed |
| 4 | 6 | QMIX | Decentralized | Physical (Proximity) | Heavy rendering bottleneck |

### TABLE D — PERCEPTION EVOLUTION
| Stage | Camera | Detector | Localization | Result | Limitation |
| :--- | :--- | :--- | :--- | :--- | :--- |
| 1 | Ground Truth | Grid Coordinates | None | Perfect routing | Unrealistic |
| 2 | RGB (Monocular) | YOLO | Center-pixel raycast | Bounding box found | No depth scale |
| 3 | RGB-D | YOLO | Median depth extraction | Accurate 3D World Coord | Extreme GPU load |

### TABLE E — WORLD EVOLUTION
| Version | World | Obstacles | Victims | Generator | Purpose |
| :--- | :--- | :--- | :--- | :--- | :--- |
| v1 | empty.sdf | None | Static | Manual XML | Drone flight tuning |
| v2 | simple_city | Static Blocks | Static | Manual XML | Basic obstacle avoidance |
| v3 | realistic_sar | Rubble/Walls | Dynamic | Python script | QMIX authoritative eval |
| v4 | earthquake | Non-uniform | Dynamic | Complex Generator | FUTURE Validation |

### TABLE F — SAFETY EVOLUTION
| Version | Safety Method | Result | Limitation |
| :--- | :--- | :--- | :--- |
| v1 | None | Drones crashed into walls | No translation layer |
| v2 | Basic Distance Check | Prevented head-on | Ignored diagonal overlaps |
| v3 | Temporal Occupancy Shield | 0 Invalid Actions logged | Forces hover state |

### TABLE G — DASHBOARD EVOLUTION
| Version | Features | Result |
| :--- | :--- | :--- |
| v1 | Terminal output | Hard to read |
| v2 | RViz markers | Better, but no history |
| v3 | React/FastAPI + Heatmap | Professional visualization |

### TABLE H — DATABASE EVOLUTION
| Stage | Storage | Capability |
| :--- | :--- | :--- |
| 1 | Text files | Basic debug |
| 2 | Summary CSVs | Fast pandas parsing |
| 3 | SQLite + WAL + Queue | High-freq telemetry persistence |

### TABLE I — EXPERIMENT HISTORY
| Experiment | Date | Drones | Records | Coverage | Victims | Perception | Status |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| QMIX Batch (Ep 11-20) | Recorded | 6 | 10 | 90.08% | 5/5 | Proximity | Validated |
| 6-Drone Scalability Test | Recorded | 6 | 1 | 29.44% | 0 | Proximity | Aborted Early |
| Perception Accuracy | Recorded | 1 | Varies | N/A | 1 | YOLO+Depth | Validated Math |

### TABLE J — VALIDATED VS NOT VALIDATED
| Capability | Status | Evidence | Notes |
| :--- | :--- | :--- | :--- |
| 6-Drone QMIX Navigation | Validated | Episodes 11-20 CSV | 90.08% stable coverage. |
| Safety Shield Interception | Validated | 0 Invalid Actions | Hover overrides intercepted collisions. |
| YOLO + RGB-D Math | Validated | Perception Reports | Geometric localization operates correctly. |
| End-to-End 6-Camera Run | NOT Validated | Failed Experiments | Hardware bottleneck (RTX 3050). |
| 5-Drone Operation | NOT Validated | N/A | Skipped entirely. |
| Earthquake Environment | NOT Validated | N/A | Future work target. |

### TABLE K — RESEARCH CLAIMS
| Claim | Evidence | Safe to State? | Correct Wording |
| :--- | :--- | :--- | :--- |
| 100% Victim Recovery | Ep 11-20 CSV | YES (with caveat) | "The swarm recovered 5/5 victims across 10 evaluation episodes using a 3.5m criterion." |
| YOLO Vision Pipeline | Codebase / Logs | YES (with caveat) | "The vision architecture was validated via controlled perception experiments." |
| QMIX Scalability | Ep 11-20 / Integrations | YES | "The architecture scaled to 6 drones executing decentralized QMIX policies." |
| Collision-Free Flight | None | NO | "The safety shield produced 0 invalid actions/timeouts." |
| 20 Evaluation Runs | Only 10 in CSV | NO | "Evaluated across 10 distinct physical simulation runs." |

### TABLE L — KNOWN LIMITATIONS
| Limitation | Impact | Mitigation Strategy |
| :--- | :--- | :--- |
| RTX 3050 VRAM Limit | Prevents 6-camera rendering | Evaluate perception/navigation separately |
| Python DDS Overhead | Drops ROS 2 telemetry | Implement background thread queueing |
| Missing CSV Collision Data | Cannot claim collision-free | Claim "0 Invalid Actions" instead |

### TABLE M — FAILED EXPERIMENTS
| Experiment | Goal | Failure Reason | Lesson Learned |
| :--- | :--- | :--- | :--- |
| End-to-End 6-Camera Mission | Full visual SAR | Gazebo RTF crashed to 0.1 | Hardware limits require decoupled testing |
| Continuous Grid Metric | Track localization error | Metric definition flawed | Abandon metric for discrete grid hits |

### TABLE N — GIT CHECKPOINTS
| Hash | Branch | Description | Status |
| :--- | :--- | :--- | :--- |
| 4ca781c | main | Ignore local cleaned evaluation artifacts | Pushed (Latest Clean) |
| 7ae05e7 | main | Clean up diagnostics and unused declarations | Pushed |
| 0ef1d2e | main | Finalize research artifacts and repository cleanup | Pushed |

### TABLE O — CURRENT TODO
| Task | Owner | Priority | Status |
| :--- | :--- | :--- | :--- |
| Implement Earthquake World | Capstone Team | High | Pending |
| Conduct Final Simulation Batch | Capstone Team | High | Pending |
| Finalize Thesis Document | Capstone Team | High | Pending |
| Prepare Live Dashboard Demo | Capstone Team | Medium | Pending |
