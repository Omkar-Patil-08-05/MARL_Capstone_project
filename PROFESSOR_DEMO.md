# MARL Swarm Command: Professor Demonstration Guide

This document provides the exact workflow and technical details required to conduct the final professor demonstration for the Multi-Agent Reinforcement Learning SAR (Search and Rescue) Drone Swarm project.

## A. Fresh Laptop Startup
After booting the laptop, you **do not** need to open Antigravity. Open a standard terminal (e.g., standard Ubuntu Terminal) and proceed to the next step.

## B. Exact ONE-Command Startup
In your terminal, navigate to the project directory and run the launcher script:

```bash
cd ~/capstone_project_antigravity
bash scripts/professor_demo.sh
```

This script will safely clean up any leftover processes, start the FastAPI backend, start the Vite frontend, perform health checks, and wait. 
Leave this terminal open.

## C. Browser URL
Once the terminal prints `SYSTEM READY!`, open your web browser and navigate to:

[http://localhost:5173](http://localhost:5173)

## D. Exact Professor Interaction Sequence
To showcase the system, follow these steps with the professor:
1. **Open dashboard:** Show the React-based Mission Command interface at `http://localhost:5173`.
2. **Select environment:** Click on **Earthquake City (Realistic SAR)**.
3. **Select drones:** Choose **2 Drones**.
4. **Verify policy:** Mention that the selected map explicitly checks metadata to ensure it is "Policy Compatible" with the currently loaded 2-agent QMIX checkpoint.
5. **Click START MISSION:** The backend will spawn the simulator stack headlessly.
6. **Wait for simulator readiness:** The UI will progress from `STARTING` to `SIMULATOR_READY` to `QMIX_STARTING` to `RUNNING`.
7. **Observe takeoff:** Once running, the drones will simultaneously take off to a validated altitude of 15m.
8. **Observe live exploration:** The drones will begin synchronously executing joint QMIX decisions and navigating the grid.
9. **Show coverage:** Highlight the Search Coverage metric increasing as drones discover new frontier cells.
10. **Show victims:** Point out that detected victims appear on the map and the UI counter increments.
11. **Show trajectories:** The 2D map tracks the real-time smoothed physical trajectories of the drones.
12. **Show QMIX decision counter:** Emphasize that the mission runs for 300 joint decisions.

## E. How to Stop Everything
To safely shut down the mission and the entire dashboard:
Press **Ctrl+C** in the terminal where you ran `bash scripts/professor_demo.sh`.

Alternatively, if you want to stop only the active mission while keeping the dashboard open, click the **Stop Mission** button on the UI.

## F. How to Open the Small Gazebo Environment
If you want to visually show the 3D physics environment (the 25x25 validated small environment), use the dedicated viewer script. **Make sure no live mission is running first.**

Open a terminal and run:
```bash
cd ~/capstone_project_antigravity
bash scripts/view_environment.sh small
```

## G. How to Open the Large Gazebo Environment
To show the larger, procedurally generated scale environment:
```bash
cd ~/capstone_project_antigravity
bash scripts/view_environment.sh large
```

## H. What to Say About Headless Gazebo
**Why don't we see the 3D graphics during the mission?**
The live mission uses "Headless Gazebo". This does **not** mean the physics simulation is disabled. The core physics engine, rigid body dynamics, collision detection, PX4 SITL (Software In The Loop) aerodynamics, and ROS 2 middleware are fully active. 

We simply disable the 3D graphics rendering window to drastically reduce CPU/GPU overhead. This allows the computationally intensive QMIX inference and multi-agent RL loops to run highly efficiently, while our lightweight React dashboard provides a superior real-time 2D visualization layer designed specifically for Command & Control. 

*(You can use the `view_environment.sh` commands above to prove the 3D environments exist natively if asked).*

## I. Current Validated Results
The current configuration is a carefully balanced culmination of our experiments:
* **H9 Baseline:** Validated the synchronous lockstep architecture (40 decisions, 43.0% coverage, 1.2% safety overrides).
* **H10 Endurance:** Validated long-horizon exploration (300 decisions, 98.0% coverage, 4/5 victims). However, a physical collision occurred due to the drones clipping a 10m tall building roof.
* **H13 Altitude Fix:** Raised the takeoff altitude to 15m. This strictly eliminated the collisions, yielding **0 collisions, 0 safety overrides, and 0 PX4 attitude errors**.

**The Final Demo combines all of these:** 300 decisions, synchronous lockstep, 15m safe altitude, BFS semantic exploration, and a 2-agent QMIX checkpoint.

## J. Future Work and Roadmap
Be scientifically transparent about our current limits and future plans:
1. **Scale Agent Count:** Scale validated QMIX execution from 2 → 3 → 4 → 5 → 6 drones. The current trained checkpoint is mathematically constrained to 2 agents.
2. **Multi-Agent Training:** Train and validate compatible multi-agent policies for these additional agent counts.
3. **Environment Scalability:** Quantitatively validate the larger generated environments with the scaled policies.
4. **Dynamic Victim Behavior:** Improve dynamic victim movement and behavior models.
5. **Vision-based Detection:** Add Gazebo camera sensors, bridge camera/image ROS 2 topics, and eventually integrate YOLO (or similar) for true vision-based victim detection rather than semantic boundary proximity.
6. **Performance Evaluation:** Compare centralized vs. decentralized coordination metrics on larger swarms.
