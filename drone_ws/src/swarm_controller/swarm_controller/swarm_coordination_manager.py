import math
import time
from swarm_controller.grid_world_transform import GridWorldTransform
from swarm_controller.drone_agent import FlightState
from swarm_controller.victim_localizer import VictimLocalizer

class SwarmCoordinationManager:
    """
    Manages the shared global state and coordinated frontier search for deterministic agents (D2/D3)
    while preserving the isolated QMIX state for D0/D1.

    CRITICAL DESIGN PRINCIPLES:
    - Coverage is ONLY marked from actual physical drone position + FOV geometry.
    - Planning state (targets, reservations, corridors) NEVER marks coverage.
    - D2/D3 MOCK perception runs here continuously, not gated by QMIX decisions.
    """
    # Safety radius in world meters (horizontal only)
    SAFETY_RADIUS_M = 3.0

    # MOCK perception: FOV half-widths at 15m altitude
    # fov_h=60deg, fov_v=45deg => ground footprint = 2*15*tan(30)=17.3m x 2*15*tan(22.5)=12.4m
    # But we use the simulated downward camera: detection radius = min(half_width) = ~6.2m
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

        # Explicit target reservations: {drone_id: {"target_cell": (x, y), "status": str, "time": float}}
        self.reserved_targets = {}

        # Track safety state to log RESUME transitions {drone_id: conflict_drone_id}
        self.safety_holds = {}

        # Track targets that a drone has tried and failed to reach, to avoid infinite loops
        self.failed_targets = {}

        # Tick counters for throttled logging
        self.tick_count = 0
        self.perception_tick = 0

        # Localizer for MOCK perception (same as used in qmix_mission_controller)
        self.localizer = VictimLocalizer()

    def dist(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def update_global_coverage(self):
        """
        Projects the FOV of deterministic agents (D2, D3) onto the shared environment grid.
        ONLY uses actual physical drone position. Never marks planned/reserved cells.
        """
        new_cells = 0
        for agent in self.det_agents:
            if agent.get_state() not in [FlightState.WAYPOINT_NAVIGATION, FlightState.HOLD]:
                continue

            lx, ly, lz = agent.px4.current_position
            wx = lx + agent.config.world_spawn_x
            wy = ly + agent.config.world_spawn_y
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
        This ensures victims are detected whenever a deterministic drone is physically near them,
        not only when D0/D1 happen to synchronize.
        """
        self.perception_tick += 1
        if self.perception_tick % self.PERCEPTION_CHECK_INTERVAL != 0:
            return

        for agent in self.det_agents:
            if agent.get_state() not in [FlightState.WAYPOINT_NAVIGATION, FlightState.HOLD]:
                continue

            lx, ly, wz = agent.px4.current_position
            wx = lx + agent.config.world_spawn_x
            wy = ly + agent.config.world_spawn_y
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

                    # Check if projected position is actually within image bounds
                    if not (0 <= cx_img <= 640 and 0 <= cy_img <= 480):
                        self.orchestrator.get_logger().info(f"[PERCEPTION] {drone_id} {v_id} OUT_OF_FOV (projected=({cx_img:.0f},{cy_img:.0f}))")
                        continue

                    bbox = [int(cx_img - 50), int(cy_img - 50), int(cx_img + 50), int(cy_img + 50)]
                    self.orchestrator.get_logger().info(f"[PERCEPTION] {drone_id} bbox={bbox}")

                    loc_res = self.localizer.localize(
                        drone_world_x=wx,
                        drone_world_y=wy,
                        drone_world_z=15.0,
                        drone_yaw=0.0,
                        img_w=640,
                        img_h=480,
                        bbox=bbox
                    )

                    self.orchestrator.get_logger().info(
                        f"[PERCEPTION] {drone_id} localized=({loc_res['world_x']:.1f}, {loc_res['world_y']:.1f})"
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
                wx = a.mission_goal_local[0] + a.config.world_spawn_x
                wy = a.mission_goal_local[1] + a.config.world_spawn_y
                targets.append((a.config.drone_id, wx, wy))
        return targets

    def tick_coordination(self):
        if any(a.initial_pos is None for a in self.det_agents):
            return

        all_ready = all(a.get_state() in [FlightState.WAYPOINT_NAVIGATION, FlightState.HOLD] for a in self.det_agents)
        if not all_ready:
            return

        self.tick_count += 1

        # 1. Update coverage from ACTUAL physical positions
        self.update_global_coverage()

        # 2. Run MOCK perception for D2/D3 independently of QMIX sync
        self.run_det_perception()

        # 3. Collect current world positions for all drones
        current_world_pos = {}
        for a in self.qmix_agents + self.det_agents:
            lx, ly, lz = a.px4.current_position
            wx = lx + a.config.world_spawn_x
            wy = ly + a.config.world_spawn_y
            current_world_pos[a.config.drone_id] = (wx, wy)

        target_world_pos = self.get_world_targets()

        # 4. Update reservation statuses — release reached/expired reservations
        for idx, a in enumerate(self.det_agents):
            drone_id = a.config.drone_id
            cgx, cgy = GridWorldTransform.world_to_grid(*current_world_pos[drone_id])

            if drone_id in self.reserved_targets:
                res = self.reserved_targets[drone_id]
                rtx, rty = res["target_cell"]

                if self.env.grid[rtx, rty] == 1:
                    res["status"] = "REACHED"
                    del self.reserved_targets[drone_id]
                    self.orchestrator.get_logger().info(f"[COORD] {drone_id} reservation ({rtx},{rty}) fulfilled/claimed.")
                elif time.time() - res["time"] > 300.0:
                    del self.reserved_targets[drone_id]

        # 5. For each deterministic drone, plan or step
        for idx, a in enumerate(self.det_agents):
            if a.get_state() not in [FlightState.WAYPOINT_NAVIGATION, FlightState.HOLD]:
                continue

            drone_id = a.config.drone_id
            curr_wx, curr_wy = current_world_pos[drone_id]
            cgx, cgy = GridWorldTransform.world_to_grid(curr_wx, curr_wy)

            # Frontier Assignment — only when drone has no active reservation
            if drone_id not in self.reserved_targets:
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

                            for (vx, vy), status in self.env.victims.items():
                                if self.dist(vx, vy, x, y) < 5.0:
                                    score += 150.0

                            if score > best_score:
                                best_score = score
                                best_target = (x, y)

                if best_target:
                    self.reserved_targets[drone_id] = {
                        "target_cell": best_target,
                        "status": "ASSIGNED",
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

            # Calculate next 1-cell step toward reserved target
            step_gx, step_gy = cgx, cgy
            if rtx > cgx: step_gx += 1
            elif rtx < cgx: step_gx -= 1
            if rty > cgy: step_gy += 1
            elif rty < cgy: step_gy -= 1

            safe_gx, safe_gy, is_valid = GridWorldTransform.clamp_grid(step_gx, step_gy)

            # Obstacle sliding: if direct diagonal is blocked, try axis-aligned
            if not is_valid or self.env.grid[safe_gx, safe_gy] == -1:
                # Try X-only
                test_gx, test_gy = step_gx, cgy
                _, _, val_x = GridWorldTransform.clamp_grid(test_gx, test_gy)

                # Try Y-only
                test_gx2, test_gy2 = cgx, step_gy
                _, _, val_y = GridWorldTransform.clamp_grid(test_gx2, test_gy2)

                if val_x and self.env.grid[test_gx, test_gy] != -1 and test_gx != cgx:
                    safe_gx, safe_gy = test_gx, test_gy
                elif val_y and self.env.grid[test_gx2, test_gy2] != -1 and test_gy2 != cgy:
                    safe_gx, safe_gy = test_gx2, test_gy2
                else:
                    # Completely blocked. Drop reservation but do NOT mark the target as explored.
                    # The target remains unexplored — another drone approaching from a different
                    # angle may be able to reach it. This drone will pick a different frontier.
                    if drone_id not in self.failed_targets:
                        self.failed_targets[drone_id] = set()
                    self.failed_targets[drone_id].add((rtx, rty))

                    del self.reserved_targets[drone_id]
                    self.orchestrator.get_logger().info(f"[COORD] {drone_id} HOLD: target ({rtx},{rty}) unreachable from current position, releasing.")
                    if a.get_state() != FlightState.HOLD:
                        a._log_transition(FlightState.HOLD)
                    continue

            twx, twy = GridWorldTransform.grid_to_world_center(safe_gx, safe_gy)

            # --- HARD PREDICTIVE SAFETY INTERLOCK ---
            is_unsafe = False
            conflict_drone = None

            # Check proposed step against every other drone's current position
            for other_id, (owx, owy) in current_world_pos.items():
                if other_id != drone_id:
                    if self.dist(twx, twy, owx, owy) < self.SAFETY_RADIUS_M:
                        is_unsafe = True
                        conflict_drone = other_id
                        break

            # Check proposed step against every other drone's target position and corridor midpoint
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
            tlx = twx - a.config.world_spawn_x
            tly = twy - a.config.world_spawn_y

            is_hover = (safe_gx == cgx and safe_gy == cgy)
            a.set_mission_goal_local(float(tlx), float(tly), a.initial_pos[2] - a.mission.takeoff_altitude, require_min_dwell=is_hover)

            if a.get_state() != FlightState.WAYPOINT_NAVIGATION:
                a._log_transition(FlightState.WAYPOINT_NAVIGATION)
