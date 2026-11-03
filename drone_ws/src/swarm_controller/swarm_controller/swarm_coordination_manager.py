import math
import time
from typing import Optional, Dict, Tuple, List, Set

from swarm_controller.grid_world_transform import GridWorldTransform
from swarm_controller.drone_agent import FlightState
from swarm_controller.victim_localizer import VictimLocalizer
from swarm_controller.sar_task_allocator import TaskState, SARTaskAllocator, SARTask


class SwarmCoordinationManager:
    """
    Manages the shared global state and coordinated frontier search and autonomous victim
    task allocation for deterministic agents (D2/D3/D4/D5) while preserving the isolated
    QMIX state for D0/D1.

    CRITICAL DESIGN PRINCIPLES:
    - Coverage is ONLY marked from actual physical drone position + FOV geometry.
    - Planning state (targets, reservations, corridors) NEVER marks coverage.
    - Clean Separation: Mission execution and task allocation use autonomously discovered
      victims from VictimManager.tracked_victims via SARTaskAllocator; ground-truth victims
      are strictly isolated for evaluation metrics.
    - One-Task / One-Drone: Autonomous victim inspection tasks are allocated 1-to-1 without
      multi-drone duplication.
    """
    # Safety radius in world meters (horizontal only)
    SAFETY_RADIUS_M = 3.0

    # MOCK perception: FOV half-widths at 15m altitude
    MOCK_DETECTION_RADIUS = 6.0

    # Coverage logging interval in ticks (every ~5 seconds at 20Hz)
    COVERAGE_LOG_INTERVAL = 100

    # Perception check interval in ticks (every ~2 seconds at 20Hz to avoid spam)
    PERCEPTION_CHECK_INTERVAL = 40

    def __init__(self, orchestrator):
        self.orchestrator = orchestrator
        self.env = orchestrator.env
        self.qmix_agents = orchestrator.agents
        self.det_agents = orchestrator.det_agents

        self.fov_radius = self.env.fov_radius
        self.x_size = self.env.x_size
        self.y_size = self.env.y_size

        self.det_coverage = set()

        # Explicit target reservations: {drone_id: {"target_cell": (x, y), "status": str, "time": float, "task_id": Optional[str]}}
        self.reserved_targets = {}

        # Track safety state to log RESUME transitions {drone_id: conflict_drone_id}
        self.safety_holds = {}

        # Track targets that a drone has tried and failed to reach, to avoid infinite loops
        self.failed_targets = {}

        # Tick counters for throttled logging
        self.tick_count = 0
        self.perception_tick = 0

        # Localizer for MOCK perception
        self.localizer = VictimLocalizer()

    def get_sim_time(self) -> float:
        """Returns the authoritative simulation time from ROS 2 clock, or falls back to wall time."""
        if self.orchestrator is not None and hasattr(self.orchestrator, 'get_clock'):
            try:
                now_ns = self.orchestrator.get_clock().now().nanoseconds
                if now_ns > 0:
                    return now_ns * 1e-9
            except Exception:
                pass
        return time.time()

    def dist(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def update_global_coverage(self):
        """
        Projects the FOV of deterministic agents (D2, D3, etc.) onto the shared environment grid.
        ONLY uses actual physical drone position. Never marks planned/reserved cells.
        """
        new_cells = 0
        for agent in self.det_agents:
            if agent.get_state() not in [FlightState.WAYPOINT_NAVIGATION, FlightState.HOLD]:
                continue

            local_north, local_east, _ = agent.px4.current_position
            wx, wy = GridWorldTransform.px4_ned_to_world_enu(
                local_north, local_east, agent.config.world_spawn_x, agent.config.world_spawn_y
            )
            gx, gy = GridWorldTransform.world_to_grid(wx, wy)

            drone_new = 0
            for i in range(-self.fov_radius, self.fov_radius + 1):
                for j in range(-self.fov_radius, self.fov_radius + 1):
                    cx, cy = gx + i, gy + j
                    if 0 <= cx < self.x_size and 0 <= cy < self.y_size:
                        if self.env.grid[cx, cy] == 0:
                            self.env.grid[cx, cy] = 1
                            self.det_coverage.add((cx, cy))
                            drone_new += 1
                            new_cells += 1

            # Periodic coverage diagnostics
            if self.tick_count % self.COVERAGE_LOG_INTERVAL == 0 and drone_new > 0:
                self.orchestrator.get_logger().info(
                    f"[Coverage] {agent.config.drone_id} pose=({wx:.1f},{wy:.1f}) grid=({gx},{gy}) newly_searched={drone_new}"
                )

        if new_cells > 0:
            self.env._update_global_bfs()

        # Periodic global coverage diagnostic
        if self.tick_count % self.COVERAGE_LOG_INTERVAL == 0:
            valid_cells = int((self.env.grid != -1).sum())
            explored = int((self.env.grid == 1).sum())
            pct = (explored / valid_cells * 100) if valid_cells > 0 else 0
            self.orchestrator.get_logger().info(f"[Coverage] searched={explored}/{valid_cells} percentage={pct:.1f}%")

    def run_det_perception(self):
        """
        Runs MOCK perception for D2/D3 independently of the QMIX decision cycle.
        Feeds visual detection to VictimManager when running without autonomous ROS 2 perception topics.
        """
        self.perception_tick += 1
        if self.perception_tick % self.PERCEPTION_CHECK_INTERVAL != 0:
            return

        for agent in self.det_agents:
            if agent.get_state() not in [FlightState.WAYPOINT_NAVIGATION, FlightState.HOLD]:
                continue

            local_north, local_east, _ = agent.px4.current_position
            wx, wy = GridWorldTransform.px4_ned_to_world_enu(
                local_north, local_east, agent.config.world_spawn_x, agent.config.world_spawn_y
            )
            drone_id = agent.config.drone_id

            for v_id, v_obj in self.orchestrator.victim_manager.victims.items():
                dx = v_obj.world_x - wx
                dy = v_obj.world_y - wy
                dist = math.hypot(dx, dy)

                if dist < self.MOCK_DETECTION_RADIUS:
                    self.orchestrator.get_logger().info(f"[PERCEPTION] {drone_id} candidate {v_id}")

                    # Camera geometry: alt=15m, fov_h=60deg, fov_v=45deg
                    alt = 15.0
                    fov_h = math.radians(60.0)
                    fov_v = math.radians(45.0)

                    # Project victim position into simulated camera image
                    nx = dy / (alt * math.tan(fov_h / 2.0))
                    ny = -dx / (alt * math.tan(fov_v / 2.0))

                    cx_img = nx * 320.0 + 320.0
                    cy_img = ny * 240.0 + 240.0

                    # Check if projected position is within image bounds
                    if not (0 <= cx_img <= 640 and 0 <= cy_img <= 480):
                        continue

                    bbox = [int(cx_img - 50), int(cy_img - 50), int(cx_img + 50), int(cy_img + 50)]

                    loc_res = self.localizer.localize(
                        drone_world_x=wx,
                        drone_world_y=wy,
                        drone_world_z=15.0,
                        drone_yaw=0.0,
                        img_w=640,
                        img_h=480,
                        bbox=bbox
                    )

                    self.orchestrator.victim_manager.process_visual_detection(
                        world_x=loc_res['world_x'],
                        world_y=loc_res['world_y'],
                        source="MOCK",
                        confidence=0.99,
                        drone_id=drone_id
                    )

    def get_world_targets(self):
        """Returns list of (drone_id, wx, wy) for all drones' current active waypoints"""
        targets = []
        for a in self.qmix_agents + self.det_agents:
            if hasattr(a, 'mission_goal_local') and a.mission_goal_local:
                goal_north = a.mission_goal_local[0]
                goal_east = a.mission_goal_local[1]
                wx, wy = GridWorldTransform.px4_ned_to_world_enu(
                    goal_north, goal_east, a.config.world_spawn_x, a.config.world_spawn_y
                )
                targets.append((a.config.drone_id, wx, wy))
        return targets

    def tick_coordination(self):
        """
        Coordinates deterministic agents (D2..D5):
        1. Updates physical coverage from FOV.
        2. Syncs and updates autonomous victim task allocation.
        3. Assigns pending victim inspection tasks to available drones.
        4. Allocates frontier exploration to idle drones.
        5. Enforces hard predictive safety collision interlocks.
        """
        if any(a.initial_pos is None for a in self.det_agents):
            return

        startup_states = [
            FlightState.WAIT_FOR_CONNECTION,
            FlightState.WAIT_FOR_ESTIMATOR,
            FlightState.STREAM_SETPOINTS,
            FlightState.REQUEST_OFFBOARD,
            FlightState.REQUEST_ARM,
            FlightState.TAKEOFF
        ]
        if any(a.get_state() in startup_states for a in self.det_agents):
            return

        self.tick_count += 1
        sim_time = self.get_sim_time()

        # 1. Update coverage from ACTUAL physical positions
        self.update_global_coverage()

        # 2. Run MOCK perception for D2/D3 independently of QMIX sync
        self.run_det_perception()

        # 3. Collect current world positions and flight states for all drones
        current_world_pos = {}
        drone_states = {}
        for a in self.qmix_agents + self.det_agents:
            local_north, local_east, _ = a.px4.current_position
            wx, wy = GridWorldTransform.px4_ned_to_world_enu(
                local_north, local_east, a.config.world_spawn_x, a.config.world_spawn_y
            )
            d_id = a.config.drone_id
            current_world_pos[d_id] = (wx, wy)
            drone_states[d_id] = a.state.name if hasattr(a, 'state') else "UNKNOWN"

        target_world_pos = self.get_world_targets()

        # 4. Integrate SARTaskAllocator (Autonomous Discoveries -> Mission Tasks)
        task_allocator: Optional[SARTaskAllocator] = getattr(self.orchestrator, 'task_allocator', None)
        if task_allocator is not None:
            # Sync autonomous tracked victims into task allocator
            tracked_victims = getattr(self.orchestrator.victim_manager, 'tracked_victims', {})
            task_allocator.sync_from_tracked_victims(tracked_victims, sim_time)

            # Update lifecycle of active tasks (approach distance, investigation dwell, failure handling)
            task_allocator.update_task_lifecycle(current_world_pos, drone_states, sim_time)

            # Determine available drones eligible for task allocation (prefer det_agents, fallback to qmix_agents)
            available_det_drones = {}
            candidate_pool = self.det_agents if self.det_agents else self.qmix_agents
            for a in candidate_pool:
                d_id = a.config.drone_id
                if a.get_state() in [FlightState.WAYPOINT_NAVIGATION, FlightState.HOLD]:
                    if d_id not in task_allocator.drone_assignments:
                        if d_id in current_world_pos:
                            available_det_drones[d_id] = current_world_pos[d_id]

            # Allocate tasks to closest available drones
            if available_det_drones:
                task_allocator.allocate_tasks(available_det_drones, sim_time)

        # 5. Update reservation statuses — release reached/expired reservations
        for idx, a in enumerate(self.det_agents):
            drone_id = a.config.drone_id

            if drone_id in self.reserved_targets:
                res = self.reserved_targets[drone_id]
                rtx, rty = res["target_cell"]
                task_id = res.get("task_id")

                if task_id and task_allocator is not None:
                    # Task reservation: release when task is completed or failed
                    t = task_allocator.tasks.get(task_id)
                    if t is None or t.state in (TaskState.COMPLETED, TaskState.FAILED) or t.assigned_drone_id != drone_id:
                        del self.reserved_targets[drone_id]
                        self.orchestrator.get_logger().info(f"[COORD] {drone_id} task {task_id} concluded/released.")
                else:
                    # Frontier reservation: release when cell is explored or times out
                    if self.env.grid[rtx, rty] == 1:
                        res["status"] = "REACHED"
                        del self.reserved_targets[drone_id]
                        self.orchestrator.get_logger().info(f"[COORD] {drone_id} reservation ({rtx},{rty}) fulfilled/claimed.")
                    elif time.time() - res["time"] > 300.0:
                        del self.reserved_targets[drone_id]

        # 6. For each deterministic drone, plan step toward assigned task or frontier
        for idx, a in enumerate(self.det_agents):
            if a.get_state() not in [FlightState.WAYPOINT_NAVIGATION, FlightState.HOLD]:
                continue

            drone_id = a.config.drone_id
            curr_wx, curr_wy = current_world_pos[drone_id]
            cgx, cgy = GridWorldTransform.world_to_grid(curr_wx, curr_wy)

            # Check if this drone has an active autonomous SAR task
            assigned_task: Optional[SARTask] = None
            if task_allocator is not None:
                assigned_task = task_allocator.get_assigned_task_for_drone(drone_id)

            if assigned_task is not None:
                # Direct drone to the autonomous victim coordinates
                tgt_cell = (assigned_task.grid_x, assigned_task.grid_y)
                self.reserved_targets[drone_id] = {
                    "target_cell": tgt_cell,
                    "status": assigned_task.state.value,
                    "task_id": assigned_task.task_id,
                    "time": time.time()
                }
            elif drone_id not in self.reserved_targets:
                # Frontier Assignment — only when drone has no active task and no active reservation
                best_score = -999999.0
                best_target = None

                scan_radius = 30
                min_x = max(0, cgx - scan_radius)
                max_x = min(self.x_size, cgx + scan_radius)
                min_y = max(0, cgy - scan_radius)
                max_y = min(self.y_size, cgy + scan_radius)

                active_reservations = [res["target_cell"] for res in self.reserved_targets.values()]

                for x in range(min_x, max_x):
                    for y in range(min_y, max_y):
                        if self.env.grid[x, y] == 0:
                            if (x, y) in active_reservations:
                                continue

                            score = 100.0
                            if drone_id in self.failed_targets and (x, y) in self.failed_targets[drone_id]:
                                score -= 1000.0

                            d = self.dist(cgx, cgy, x, y)
                            score -= d * 1.5

                            for other_id, (owx, owy) in current_world_pos.items():
                                if other_id != drone_id:
                                    ogx, ogy = GridWorldTransform.world_to_grid(owx, owy)
                                    od = self.dist(ogx, ogy, x, y)
                                    if "drone_0" in str(other_id) or "drone_1" in str(other_id):
                                        if od < 6.0: score -= 500.0
                                        elif od < 12.0: score -= 100.0
                                    else:
                                        if od < 8.0: score -= 200.0

                            for other_id, twx, twy in target_world_pos:
                                if other_id != drone_id:
                                    tgx, tgy = GridWorldTransform.world_to_grid(twx, twy)
                                    td = self.dist(tgx, tgy, x, y)
                                    if "drone_0" in str(other_id) or "drone_1" in str(other_id):
                                        if td < 8.0: score -= 300.0

                            # Prioritize unassigned autonomous discoveries if any exist
                            if task_allocator is not None:
                                for t in task_allocator.tasks.values():
                                    if t.state == TaskState.DISCOVERED:
                                        if self.dist(t.grid_x, t.grid_y, x, y) < 4.0:
                                            score += 150.0

                            if score > best_score:
                                best_score = score
                                best_target = (x, y)

                if best_target:
                    self.reserved_targets[drone_id] = {
                        "target_cell": best_target,
                        "status": "ASSIGNED",
                        "task_id": None,
                        "time": time.time()
                    }
                    self.orchestrator.get_logger().info(f"[COORD] {drone_id} assigned frontier {best_target} with score {best_score:.1f}")
                else:
                    if a.get_state() != FlightState.HOLD:
                        self.orchestrator.get_logger().info(f"[COORD] {drone_id} HOLD: no valid frontier")
                        a._log_transition(FlightState.HOLD)
                    continue

            self.reserved_targets[drone_id]["status"] = "ACTIVE"
            rtx, rty = self.reserved_targets[drone_id]["target_cell"]

            # If investigating an autonomous victim, maintain hover dwell at victim location
            if assigned_task is not None and assigned_task.state == TaskState.INVESTIGATING:
                step_gx, step_gy = cgx, cgy
            else:
                # Calculate next 1-cell step toward reserved target
                step_gx, step_gy = cgx, cgy
                if rtx > cgx: step_gx += 1
                elif rtx < cgx: step_gx -= 1
                if rty > cgy: step_gy += 1
                elif rty < cgy: step_gy -= 1

            safe_gx, safe_gy, is_valid = GridWorldTransform.clamp_grid(step_gx, step_gy)

            # Obstacle sliding: if direct diagonal is blocked, try axis-aligned
            if not is_valid or self.env.grid[safe_gx, safe_gy] == -1:
                test_gx, test_gy = step_gx, cgy
                _, _, val_x = GridWorldTransform.clamp_grid(test_gx, test_gy)

                test_gx2, test_gy2 = cgx, step_gy
                _, _, val_y = GridWorldTransform.clamp_grid(test_gx2, test_gy2)

                if val_x and self.env.grid[test_gx, test_gy] != -1 and test_gx != cgx:
                    safe_gx, safe_gy = test_gx, test_gy
                elif val_y and self.env.grid[test_gx2, test_gy2] != -1 and test_gy2 != cgy:
                    safe_gx, safe_gy = test_gx2, test_gy2
                else:
                    # Blocked: release reservation
                    if drone_id not in self.failed_targets:
                        self.failed_targets[drone_id] = set()
                    self.failed_targets[drone_id].add((rtx, rty))

                    del self.reserved_targets[drone_id]
                    self.orchestrator.get_logger().info(f"[COORD] {drone_id} HOLD: target ({rtx},{rty}) unreachable, releasing.")
                    if a.get_state() != FlightState.HOLD:
                        a._log_transition(FlightState.HOLD)
                    continue

            twx, twy = GridWorldTransform.grid_to_world_center(safe_gx, safe_gy)

            # --- HARD PREDICTIVE SAFETY INTERLOCK ---
            is_unsafe = False
            conflict_drone = None

            for other_id, (owx, owy) in current_world_pos.items():
                if other_id != drone_id:
                    if self.dist(twx, twy, owx, owy) < self.SAFETY_RADIUS_M:
                        is_unsafe = True
                        conflict_drone = other_id
                        break

            if not is_unsafe:
                for other_id, otwx, otwy in target_world_pos:
                    if other_id != drone_id:
                        if self.dist(twx, twy, otwx, otwy) < self.SAFETY_RADIUS_M:
                            is_unsafe = True
                            conflict_drone = other_id
                            break

                        owx, owy = current_world_pos[other_id]
                        mid_wx, mid_wy = (owx + otwx) / 2.0, (owy + otwy) / 2.0
                        if self.dist(twx, twy, mid_wx, mid_wy) < self.SAFETY_RADIUS_M:
                            is_unsafe = True
                            conflict_drone = other_id
                            break

            if is_unsafe:
                if self.safety_holds.get(drone_id) != conflict_drone:
                    self.orchestrator.get_logger().info(f"[COORD] {drone_id} HOLD: safety conflict with {conflict_drone}")
                    self.safety_holds[drone_id] = conflict_drone

                if a.get_state() != FlightState.HOLD:
                    a._log_transition(FlightState.HOLD)
                continue
            else:
                if drone_id in self.safety_holds:
                    self.orchestrator.get_logger().info(f"[COORD] {drone_id} RESUME: safety conflict cleared")
                    del self.safety_holds[drone_id]

            # Safe to proceed with the step
            local_north, local_east = GridWorldTransform.world_enu_to_px4_ned(
                twx, twy, a.config.world_spawn_x, a.config.world_spawn_y
            )

            is_hover = (safe_gx == cgx and safe_gy == cgy) or (assigned_task is not None and assigned_task.state == TaskState.INVESTIGATING)
            a.set_mission_goal_local(float(local_north), float(local_east), a.initial_pos[2] - a.mission.takeoff_altitude, require_min_dwell=is_hover)

            if a.get_state() != FlightState.WAYPOINT_NAVIGATION:
                a._log_transition(FlightState.WAYPOINT_NAVIGATION)
