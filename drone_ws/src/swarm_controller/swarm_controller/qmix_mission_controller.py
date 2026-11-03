import os
import csv
import time
import math
import rclpy
from rclpy.node import Node
import sys
from std_msgs.msg import String
import json

from swarm_controller.drone_agent import DroneAgent, FlightState
from swarm_controller.grid_world_transform import GridWorldTransform
from swarm_controller.qmix_ros2_adapter import QMIXAdapter
from swarm_controller.victim_manager import VictimManager
from swarm_controller.sar_task_allocator import SARTaskAllocator, TaskState

sys.path.append('/home/capstone/capstone_project_antigravity')
from marl_drone_project.env.sar_env import SARGridEnv

class QMIXMissionController(Node):
    def __init__(self, mission_config, drone_configs, checkpoint_path, max_decisions=30, meta_file=None):
        super().__init__('qmix_mission_controller')
        self.mission_config = mission_config
        self.drone_configs = drone_configs
        self.max_decisions = int(os.environ.get("MAX_DECISIONS", str(max_decisions)))
        self.meta_file = meta_file

        # Instantiate drones
        self.agents = [DroneAgent(self, config, mission_config) for config in drone_configs]

        # Initialize internal SARGridEnv to track deployment grid state and observations
        self.env = SARGridEnv(
            num_drones=len(self.agents),
            max_steps=1000,
            x_size=GridWorldTransform.GRID_SIZE_X,
            y_size=GridWorldTransform.GRID_SIZE_Y
        )
        self.env.reset()

        # Override initial grid with static obstacles from metadata
        self._initialize_grid_obstacles(meta_file=self.meta_file)

        self.unspawned_victims = list(self.env.victims.keys())

        # QMIX inference bridge
        self.qmix = QMIXAdapter(checkpoint_path, num_agents=len(self.agents))

        # Logging
        self.results_dir = os.path.join(os.getcwd(), "results", "h8_v4_physical_final")
        os.makedirs(self.results_dir, exist_ok=True)
        self.log_file = open(os.path.join(self.results_dir, "qmix_decisions.csv"), "w", newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow([
            "timestamp", "episode_step", "drone_id",
            "grid_x", "grid_y", "current_world_x", "current_world_y", "world_z",
            "action", "action_name",
            "target_grid_x", "target_grid_y",
            "target_world_x", "target_world_y",
            "distance_to_target", "coverage", "victims_detected", "inference_latency_ms",
            "safety_override"
        ])

        self.trajectory_files = {}
        self.trajectory_writers = {}
        for agent in self.agents:
            d_id_str = str(agent.config.drone_id)
            f = open(os.path.join(self.results_dir, f"trajectory_{d_id_str}.csv"), "w", newline='')
            w = csv.writer(f)
            w.writerow(["timestamp", "world_x", "world_y", "world_z", "grid_x", "grid_y"])
            self.trajectory_files[d_id_str] = f
            self.trajectory_writers[d_id_str] = w

        self.victim_file = open(os.path.join(self.results_dir, "victim_detection.csv"), "w", newline='')
        self.victim_writer = csv.writer(self.victim_file)
        self.victim_writer.writerow(["timestamp", "decision_step", "victim_grid_x", "victim_grid_y", "detected_by"])

        # State tracking
        self.decisions_made = 0
        self.mission_active = False
        self.land_commanded = False
        self.detected_victims = set()
        self.detection_radius = float(os.environ.get("DETECTION_RADIUS", str(1.5 * GridWorldTransform.METERS_PER_CELL)))
        self.last_decision_time = time.time()
        self.last_heartbeat_time = time.time()
        self.natural_hover_count = 0
        self.safety_forced_hover_count = 0
        self.safety_overrides_count = 0
        self.start_time = time.time()
        self.global_step_state = "INITIALIZING"

        # Modular SAR Task Allocator for autonomous discoveries
        self.task_allocator = SARTaskAllocator(node=self)

        self.setup_telemetry()

        # Create control loop timer
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.tick)

    def _check_proximity_detection(self):
        """Continuous proximity-based mission detection.
        
        Runs every tick (20Hz) to ensure victims are detected the instant
        a drone passes within the sensor detection radius, regardless of QMIX decision timing.
        This is the authoritative detection mechanism.
        """
        for agent in self.agents:
            if agent.state not in [FlightState.WAYPOINT_NAVIGATION, FlightState.HOLD]:
                continue
            local_north, local_east, _ = agent.px4.current_position
            wx, wy = GridWorldTransform.px4_ned_to_world_enu(
                local_north, local_east, agent.config.world_spawn_x, agent.config.world_spawn_y
            )

            for v_id, victim in self.victim_manager.victims.items():
                if victim.state.value == "UNDETECTED":
                    dist = math.hypot(victim.world_x - wx, victim.world_y - wy)
                    if dist <= self.detection_radius:
                        if self.victim_manager.mark_detected(v_id, detected_by=agent.config.drone_id, detection_distance=dist):
                            self.detected_victims.add(v_id)
                            t_sec = self.get_clock().now().nanoseconds / 1e9
                            self.victim_writer.writerow([t_sec, self.decisions_made, victim.grid_x, victim.grid_y, agent.config.drone_id])
                            self.victim_file.flush()
                            self.get_logger().info(
                                f"[{agent.config.drone_id}] MISSION DETECTION: "
                                f"{v_id} at distance {dist:.1f}m (radius={self.detection_radius:.1f}m) "
                                f"(drone@({wx:.1f},{wy:.1f}) victim@({victim.world_x:.1f},{victim.world_y:.1f}))"
                            )


    def sync_victims_to_env(self):
        """Synchronizes dynamic victims from VictimManager to SARGridEnv for FOV detection."""
        self.env.victims.clear()
        for v_obj in self.victim_manager.victims.values():
            gx = v_obj.grid_x
            gy = v_obj.grid_y

            # Bound check gracefully
            if 0 <= gx < self.env.x_size and 0 <= gy < self.env.y_size:
                status = 1 if v_obj.state.value == "DETECTED" or v_obj.state.value == "RESCUED" else 0
                self.env.victims[(gx, gy)] = status

    def setup_telemetry(self):

        # Telemetry
        self.telemetry_pub = self.create_publisher(String, '/swarm/telemetry', 10)
        self.telemetry_tick_counter = 0
        self.last_actions = {a.config.drone_id: "None" for a in self.agents}
        self.last_safety_overrides = {a.config.drone_id: False for a in self.agents}

        # Autonomous victim discovery ingestion
        self.discovery_sub = self.create_subscription(
            String,
            '/sar/victim_discovery',
            self.discovery_callback,
            10
        )

        self.get_logger().info("QMIX Mission Controller initialized.")

    def discovery_callback(self, msg: String):
        """Callback for /sar/victim_discovery events from autonomous perception.
        
        Enforces mandatory publisher fields:
          - event_type == 'VICTIM_DISCOVERED'
          - timestamp (finite float >= 0.0)
          - victim_track_id (non-negative integer)
          - confirming_drone_id (non-negative integer)
          - world_x (finite float)
          - world_y (finite float)
          - confidence (finite float in [0.0, 1.0])
        
        Rejects missing or invalid mandatory fields with diagnostic warnings.
        Validates optional metadata strictly (finite checks, non-negative constraints).
        Preserves source, source_track_id, and verification_mode independently.
        Uses UNKNOWN_AUTONOMOUS_SOURCE when source is absent from publisher payload.
        Synchronizes confirmed discoveries into SARTaskAllocator.
        """
        try:
            if not msg.data:
                return
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                self.get_logger().warn("[QMIX Discovery] Non-dict payload received on /sar/victim_discovery.")
                return

            is_valid, parsed_data, reason = self.victim_manager.validate_event_payload(data)
            if not is_valid or parsed_data is None:
                self.get_logger().warn(f"[QMIX Discovery] Rejected invalid discovery event: {reason}")
                return

            # Ingest validated event into VictimManager
            self.victim_manager.process_discovery_event(
                world_x=parsed_data["world_x"],
                world_y=parsed_data["world_y"],
                source=parsed_data["source"],
                confidence=parsed_data["confidence"],
                drone_id=parsed_data["drone_id"],
                sim_time=parsed_data["sim_time"],
                source_track_id=parsed_data["source_track_id"],
                verification_mode=parsed_data["verification_mode"],
                evidence_state=parsed_data["evidence_state"],
                thermal_peak_k=parsed_data["thermal_peak_k"],
                thermal_delta_k=parsed_data["thermal_delta_k"],
                confirmation_count=parsed_data["confirmation_count"],
                confirmation_window=parsed_data["confirmation_window"],
                consensus_span_sec=parsed_data["consensus_span_sec"],
                median_depth_estimate=parsed_data["median_depth_estimate"],
                depth_valid_samples=parsed_data["depth_valid_samples"],
                depth_spread=parsed_data["depth_spread"]
            )

            # Sync autonomous discoveries into Task Allocator
            self.task_allocator.sync_from_tracked_victims(
                self.victim_manager.tracked_victims,
                parsed_data["sim_time"]
            )
        except json.JSONDecodeError as err:
            self.get_logger().warn(f"[QMIX Discovery] Malformed JSON message on /sar/victim_discovery: {err}")
        except Exception as e:
            self.get_logger().error(f"[QMIX Discovery] Error processing discovery event: {e}")


    def _initialize_grid_obstacles(self, meta_file=None):
        """Populates the grid with static obstacles from metadata."""
        project_root = os.path.expanduser('~/capstone_project_antigravity')
        if meta_file and os.path.exists(meta_file):
            meta_path = meta_file
        else:
            meta_path = os.path.join(project_root, 'worlds', 'generated_world_meta.json')

        if not os.path.exists(meta_path):
            raise RuntimeError(f"CRITICAL: Metadata file not found at {meta_path}.")

        try:
            with open(meta_path, 'r') as f:
                meta = json.load(f)
        except json.JSONDecodeError:
            raise RuntimeError(f"CRITICAL: Metadata file {meta_path} is malformed.")

        obstacles = meta.get("obstacles", [])
        drone_spawns = meta.get("drone_base", {}).get("spawns", [])

        METERS_PER_CELL = GridWorldTransform.METERS_PER_CELL
        ORIGIN_X = GridWorldTransform.GRID_ORIGIN_X
        ORIGIN_Y = GridWorldTransform.GRID_ORIGIN_Y

        # 1. Clear training obstacles and project geometric AABBs
        self.env.grid.fill(0)
        for gx in range(self.env.x_size):
            for gy in range(self.env.y_size):
                cell_min_x = ORIGIN_X + gx * METERS_PER_CELL
                cell_max_x = ORIGIN_X + (gx + 1) * METERS_PER_CELL
                cell_min_y = ORIGIN_Y + gy * METERS_PER_CELL
                cell_max_y = ORIGIN_Y + (gy + 1) * METERS_PER_CELL

                for obs in obstacles:
                    aabb = obs.get("aabb")
                    if not aabb:
                        continue
                    intersect_x = (aabb["max_x"] > cell_min_x) and (aabb["min_x"] < cell_max_x)
                    intersect_y = (aabb["max_y"] > cell_min_y) and (aabb["min_y"] < cell_max_y)

                    if intersect_x and intersect_y:
                        self.env.grid[gx, gy] = -1
                        break

        # 2. Populate and strictly validate victims using VictimManager
        if self.meta_file and "forest_dense_candidate" in self.meta_file:
            map_id = "forest_dense_candidate"
        elif self.meta_file and "forest" in self.meta_file:
            map_id = "forest_sar"
        elif self.meta_file and "earthquake" in self.meta_file:
            map_id = "earthquake_world"
        else:
            map_id = os.environ.get("PX4_GZ_WORLD", "realistic_sar")
        victim_seed = int(os.environ.get("VICTIM_SEED", "42"))
        self.victim_manager = VictimManager(self, meta, map_id, self.env, seed=victim_seed)

        self.sync_victims_to_env()

        # Spawn the victims into Gazebo
        self.victim_manager.spawn_all()

        # 3. Synchronize drone spawn metadata
        for agent in self.agents:
            drone_id_str = str(agent.config.drone_id)
            spawn_meta = next((s for s in drone_spawns if drone_id_str in str(s.get("id", ""))), None)
            if spawn_meta:
                agent.config.world_spawn_x = spawn_meta["x"]
                agent.config.world_spawn_y = spawn_meta["y"]
                self.get_logger().info(f"Dynamically loaded spawn for Drone {drone_id_str}: ({spawn_meta['x']}, {spawn_meta['y']})")
            else:
                self.get_logger().warn(f"No drone_base spawn metadata found for Drone {drone_id_str}. Relying on launch config.")

    def tick(self):
        current_time = time.time()
        if self.mission_active and not self.land_commanded:
            if current_time - self.last_heartbeat_time >= 60.0:
                self.last_heartbeat_time = current_time
                elapsed = current_time - self.start_time
                valid_cells = (self.env.grid != -1).sum()
                explored = (self.env.grid == 1).sum()
                coverage = (explored / valid_cells) * 100 if valid_cells > 0 else 0
                rate = self.decisions_made / elapsed if elapsed > 0 else 0.0
                state_str = ", ".join([f"{a.config.drone_id}: {a.state.name}" for a in self.agents])
                self.get_logger().info(
                    f"\n==================================================\n"
                    f"Elapsed: {elapsed:.1f}s | Decisions: {self.decisions_made} / {self.max_decisions} ({rate:.2f} dec/s)\n"
                    f"Coverage: {coverage:.2f}% | Victims: {len(self.detected_victims)}/5\n"
                    f"Hover (Nat): {self.natural_hover_count} | Hover (Safe): {self.safety_forced_hover_count} | Overrides: {self.safety_overrides_count}\n"
                    f"States: {state_str}\n"
                    f"Last decision: {self.last_decision_time:.2f} | Time since prev: {current_time - self.last_decision_time:.2f}s\n"
                    f"=================================================="
                )

            if current_time - self.last_decision_time >= 120.0:
                self.get_logger().error("H6 WARNING: NO DECISION PROGRESS FOR 120 SECONDS")
                self.last_decision_time = current_time

        for agent in self.agents:
            agent.tick()

        # Continuous real-time FOV and proximity-based mission detection (runs every tick at 20Hz)
        if self.mission_active:
            self.sync_env_state()
            self.victim_manager.update()
            self.sync_victims_to_env()
            new_cells, _ = self.env._update_fov_and_victims()
            if sum(new_cells) > 0:
                self.env._update_global_bfs()
            self._check_proximity_detection()

            # Update autonomous task allocator lifecycle
            current_sim_time = self.agents[0].get_current_sim_time() if self.agents else 0.0
            self.task_allocator.sync_from_tracked_victims(self.victim_manager.tracked_victims, current_sim_time)
            drone_positions = {}
            drone_states = {}
            # Track all active agents (both QMIX and deterministic if present)
            all_tracked_agents = list(self.agents) + getattr(self, 'det_agents', [])
            for a in all_tracked_agents:
                local_north, local_east, _ = a.px4.current_position
                wx, wy = GridWorldTransform.px4_ned_to_world_enu(
                    local_north, local_east, a.config.world_spawn_x, a.config.world_spawn_y
                )
                drone_positions[str(a.config.drone_id)] = (wx, wy)
                drone_states[str(a.config.drone_id)] = a.state.name
            self.task_allocator.update_task_lifecycle(drone_positions, drone_states, current_sim_time)

            # Allocate unassigned tasks to available drones
            # In 6-drone mode (det_agents present), det_agents are preferred for SAR tasks.
            # In 2-drone mode (det_agents empty), QMIX agents are eligible.
            det_agents = getattr(self, 'det_agents', [])
            eligible_pool = det_agents if det_agents else self.agents
            available_drones = {}
            for a in eligible_pool:
                d_id = str(a.config.drone_id)
                if a.get_state() in [FlightState.WAYPOINT_NAVIGATION, FlightState.HOLD]:
                    if d_id not in self.task_allocator.drone_assignments:
                        if d_id in drone_positions:
                            available_drones[d_id] = drone_positions[d_id]
            if available_drones:
                self.task_allocator.allocate_tasks(available_drones, current_sim_time)

        self.telemetry_tick_counter += 1
        if self.telemetry_tick_counter % 2 == 0:
            self._publish_telemetry(current_time)

        if not self.mission_active:
            if all(a.state == FlightState.HOLD for a in self.agents):
                self.get_logger().info("All drones airborne. Starting QMIX inference.")
                self.mission_active = True
                self.global_step_state = "EXECUTING"
                self.execute_global_step()
            return

        if self.decisions_made >= self.max_decisions:
            if not self.land_commanded:
                self.get_logger().info("Max decisions reached. Commanding landing.")
                self.land_commanded = True
                self.global_step_state = "LANDING"
                for agent in self.agents:
                    agent.px4.publish_vehicle_command(21)
                    agent._log_transition(FlightState.LAND)
            return

        # Simulation-Time Synchronization & Watchdog Block
        all_ready = True
        active_agents = [a for a in self.agents if a.state not in (FlightState.FAILSAFE, FlightState.COMPLETE, FlightState.DISARM)]

        sim_timeout = getattr(self.mission_config, 'waypoint_sim_timeout', 20.0)
        wall_watchdog = getattr(self.mission_config, 'waypoint_wall_watchdog', 90.0)

        for agent in active_agents:
            if agent.state == FlightState.WAYPOINT_NAVIGATION:
                # 1. Stall detection: zero distance progress over threshold
                if agent.is_stalled():
                    self.get_logger().warn(
                        f"[{agent.config.drone_id}] Drone stalled without forward progress! Forcing HOLD for resync."
                    )
                    agent._log_transition(FlightState.HOLD)
                # 2. Genuine simulation-time waypoint timeout
                elif agent.elapsed_sim_time() >= sim_timeout:
                    self.get_logger().warn(
                        f"[{agent.config.drone_id}] Waypoint simulation timeout reached "
                        f"({agent.elapsed_sim_time():.1f}s sim >= {sim_timeout:.1f}s)! Forcing HOLD for resync."
                    )
                    agent._log_transition(FlightState.HOLD)
                # 3. Outer safety wall-clock watchdog to prevent deadlock
                elif agent.elapsed_wall_time() >= wall_watchdog:
                    self.get_logger().warn(
                        f"[{agent.config.drone_id}] Waypoint wall-clock watchdog expired "
                        f"({agent.elapsed_wall_time():.1f}s wall >= {wall_watchdog:.1f}s)! Forcing HOLD for resync."
                    )
                    agent._log_transition(FlightState.HOLD)

            if agent.state != FlightState.HOLD:
                all_ready = False

        if active_agents and all_ready:
            self.global_step_state = "READY_FOR_NEXT_STEP"
            self.execute_global_step()
            self.global_step_state = "EXECUTING"
        elif not active_agents:
            if not self.land_commanded:
                self.get_logger().info("All drones in terminal/failsafe states. Commanding landing.")
                self.land_commanded = True
                self.global_step_state = "COMPLETE"
        else:
            if any(a.state == FlightState.WAYPOINT_NAVIGATION for a in active_agents):
                self.global_step_state = "NAVIGATING"
            elif any(a.state == FlightState.HOLD for a in active_agents):
                self.global_step_state = "SYNCHRONIZING"
            else:
                self.global_step_state = "WAITING_FOR_AGENTS"

    def _publish_telemetry(self, current_time):
        try:
            mission_status = "INITIALIZING"
            if self.mission_active:
                mission_status = "RUNNING"
            if self.land_commanded:
                mission_status = "LANDING"
            if any(a.state == FlightState.FAILSAFE for a in self.agents):
                mission_status = "FAILSAFE"
            if all(a.state == FlightState.COMPLETE for a in self.agents):
                mission_status = "COMPLETE"

            valid_cells = (self.env.grid != -1).sum()
            explored = (self.env.grid == 1).sum()
            coverage = (explored / valid_cells) * 100 if valid_cells > 0 else 0

            drones_data = []
            for agent in self.agents:
                local_north, local_east, local_down = agent.px4.current_position
                wx, wy = GridWorldTransform.px4_ned_to_world_enu(
                    local_north, local_east, agent.config.world_spawn_x, agent.config.world_spawn_y
                )
                wz = -local_down + getattr(agent.config, 'world_spawn_z', 0.2)
                gx, gy = GridWorldTransform.world_to_grid(wx, wy)

                drone_id = agent.config.drone_id

                # Actual runtime velocity from PX4 odometry (converted to ENU: vx=East, vy=North, vz=Up)
                vn, ve, vd = agent.px4.current_velocity
                vx_enu = ve
                vy_enu = vn
                vz_enu = -vd
                speed = math.sqrt(vn**2 + ve**2 + vd**2)

                drones_data.append({
                    "id": str(drone_id),
                    "state": agent.state.name,
                    "x": round(float(wx), 2),
                    "y": round(float(wy), 2),
                    "z": round(float(wz), 2),
                    "grid_x": int(gx),
                    "grid_y": int(gy),
                    "action": self.last_actions.get(drone_id, "None"),
                    "safety_override": self.last_safety_overrides.get(drone_id, False),
                    "vx": round(float(vx_enu), 3),
                    "vy": round(float(vy_enu), 3),
                    "vz": round(float(vz_enu), 3),
                    "speed": round(float(speed), 2)
                })

            # Build explored cells list from authoritative SARGridEnv
            explored_cells = []
            for gx in range(self.env.x_size):
                for gy in range(self.env.y_size):
                    if self.env.grid[gx, gy] == 1:
                        explored_cells.append({"x": int(gx), "y": int(gy)})

            # Online Ground-Truth Evaluation (Strict one-to-one Hungarian matching)
            eval_metrics = {}
            if hasattr(self.victim_manager, 'get_evaluation_metrics'):
                try:
                    eval_metrics = self.victim_manager.get_evaluation_metrics()
                except Exception as e:
                    self.get_logger().debug(f"Evaluation metrics calculation error: {e}")

            # Build victim state list from VictimManager
            victims_state = [v.get_dict() for v in self.victim_manager.victims.values()]
            tracked_victims_state = [t.get_dict() for t in self.victim_manager.tracked_victims.values()]

            # Calculate unique detections based on mission state
            unique_detected = sum(1 for v in self.victim_manager.victims.values() if v.state.value == "DETECTED" or v.state.value == "RESCUED")

            # Calculate simulation time
            sim_time = self.agents[0].get_current_sim_time() if self.agents else 0.0

            # Task allocation state
            task_alloc_data = self.task_allocator.get_telemetry()
            tasks_list = [t.get_dict() for t in self.task_allocator.tasks.values()]

            msg_dict = {
                "type": "telemetry",
                "timestamp": current_time,
                "mission": {
                    "status": mission_status,
                    "decision_count": self.decisions_made,
                    "max_decisions": self.max_decisions,
                    "global_step_state": getattr(self, 'global_step_state', "UNKNOWN"),
                    "sim_time": round(float(sim_time), 2),
                    "coverage": round(float(coverage), 2),
                    "victims_detected": unique_detected,
                    "total_victims": len(self.victim_manager.victims),
                    "explored_count": int(explored),
                    "valid_count": int(valid_cells),
                    "safety_overrides": self.safety_overrides_count
                },
                "drones": drones_data,
                "explored_cells": explored_cells,
                "victims": victims_state,
                "tracked_victims": tracked_victims_state,
                "task_allocation": task_alloc_data,
                "tasks": tasks_list,
                "evaluation": eval_metrics,
                "coordination": {
                    "qmix_drones": len(self.agents),
                    "coord_drones": 0,
                    "active_frontiers": {},
                    "safety_holds": {str(a.config.drone_id): "HOLD" for a in self.agents if getattr(a, 'state', None) and a.state.name == "HOLD"},
                    "task_assignments": dict(self.task_allocator.drone_assignments),
                    "tasks": tasks_list
                }
            }

            msg = String()
            msg.data = json.dumps(msg_dict)
            self.telemetry_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing telemetry: {e}")

    def sync_env_state(self):
        """Authoritatively synchronizes the SARGridEnv state from physical PX4 drone locations."""
        for i, agent in enumerate(self.agents):
            local_north, local_east, _ = agent.px4.current_position

            wx, wy = GridWorldTransform.px4_ned_to_world_enu(
                local_north, local_east, agent.config.world_spawn_x, agent.config.world_spawn_y
            )

            gx, gy = GridWorldTransform.world_to_grid(wx, wy)

            gx = max(0, min(gx, self.env.x_size - 1))
            gy = max(0, min(gy, self.env.y_size - 1))

            self.env.drone_positions[i] = [gx, gy]

    def execute_global_step(self):
        """Executes a fully synchronous RL step for all active agents simultaneously."""
        self.sync_env_state()

        # Update moving victims
        self.victim_manager.update()

        self.sync_victims_to_env()

        new_cells, _ = self.env._update_fov_and_victims()

        if sum(new_cells) > 0:
            self.env._update_global_bfs()

        # 1. Collect all observations synchronously
        t_start = time.time()
        actions = []
        latencies = []
        for idx, agent in enumerate(self.agents):
            obs = self.env.get_agent_state(idx)
            action = self.qmix.select_action(idx, obs)
            actions.append(action)
            latencies.append((time.time() - t_start) * 1000.0)

        self.decisions_made += 1
        self.last_decision_time = time.time()

        action_names = {0: "+X", 1: "-X", 2: "+Y", 3: "-Y", 4: "Hover"}

        # ============================================================
        # QMIX + Rule-Based Safety Shield (Multi-Agent Occupancy)
        # ============================================================
        proposed = []  # List of (curr_gx, curr_gy, target_gx, target_gy, action)
        for idx, agent in enumerate(self.agents):
            curr_gx, curr_gy = self.env.drone_positions[idx]
            target_gx, target_gy = curr_gx, curr_gy

            # Check if this agent is assigned an active autonomous SAR task
            assigned_task = self.task_allocator.get_assigned_task_for_drone(agent.config.drone_id)
            if assigned_task is not None and assigned_task.state in (TaskState.ASSIGNED, TaskState.APPROACHING, TaskState.INVESTIGATING):
                if assigned_task.state == TaskState.INVESTIGATING:
                    # Hover dwell over victim
                    target_gx, target_gy = curr_gx, curr_gy
                    action = 4
                else:
                    # Steer 1-cell step toward autonomous victim grid coordinates (4-connected)
                    tgt_gx, tgt_gy = assigned_task.grid_x, assigned_task.grid_y
                    step_gx, step_gy = curr_gx, curr_gy
                    dx = tgt_gx - curr_gx
                    dy = tgt_gy - curr_gy
                    if abs(dx) >= abs(dy) and dx != 0:
                        if dx > 0: action = 0; step_gx += 1
                        else: action = 1; step_gx -= 1
                    elif dy != 0:
                        if dy > 0: action = 2; step_gy += 1
                        else: action = 3; step_gy -= 1
                    else:
                        action = 4
                    target_gx, target_gy = step_gx, step_gy
            else:
                action = actions[idx]
                if action == 0: target_gx += 1
                elif action == 1: target_gx -= 1
                elif action == 2: target_gy += 1
                elif action == 3: target_gy -= 1

            # Clamp to grid bounds
            safe_gx, safe_gy, is_valid = GridWorldTransform.clamp_grid(target_gx, target_gy)
            if not is_valid:
                safe_gx, safe_gy = curr_gx, curr_gy
                action = 4  # Override to hover on boundary violation

            # Obstacle check
            if self.env.grid[safe_gx, safe_gy] == -1:
                if assigned_task is not None and assigned_task.state in (TaskState.ASSIGNED, TaskState.APPROACHING, TaskState.INVESTIGATING):
                    # Aerial drones at flight altitude (10m+) can fly over ground rubble to investigate victims
                    pass
                else:
                    safe_gx, safe_gy = curr_gx, curr_gy
                    action = 4  # Override to hover on obstacle

            proposed.append((curr_gx, curr_gy, safe_gx, safe_gy, action))

        # Detect cell-swap conflicts
        swap_overrides = set()
        for i in range(len(proposed)):
            for j in range(i + 1, len(proposed)):
                ci, cj = proposed[i], proposed[j]
                if (ci[2], ci[3]) == (cj[0], cj[1]) and (cj[2], cj[3]) == (ci[0], ci[1]):
                    swap_overrides.add(i)
                    swap_overrides.add(j)

        # Apply occupancy claims with deterministic priority (lower index first)
        claimed_cells = set()
        final_actions = []  # (safe_gx, safe_gy, action, safety_override)

        # First pass: apply swap overrides (force hover at current position)
        resolved_proposed = list(proposed)
        for idx in swap_overrides:
            curr_gx, curr_gy, _, _, _ = resolved_proposed[idx]
            resolved_proposed[idx] = (curr_gx, curr_gy, curr_gx, curr_gy, 4)

        # Second pass: claim cells in deterministic order
        for idx in range(len(resolved_proposed)):
            curr_gx, curr_gy, safe_gx, safe_gy, action = resolved_proposed[idx]
            safety_override = False

            # Check original vs resolved to detect if bounds/obstacle already overrode
            orig_action = actions[idx]
            orig_curr_gx, orig_curr_gy = self.env.drone_positions[idx]
            orig_target_gx, orig_target_gy = orig_curr_gx, orig_curr_gy
            if orig_action == 0: orig_target_gx += 1
            elif orig_action == 1: orig_target_gx -= 1
            elif orig_action == 2: orig_target_gy += 1
            elif orig_action == 3: orig_target_gy -= 1

            # Boundary violation
            _, _, is_valid = GridWorldTransform.clamp_grid(orig_target_gx, orig_target_gy)
            if not is_valid:
                self.get_logger().warn(f"[{self.agents[idx].config.drone_id}] Boundary violation prevented!")
                safety_override = True

            # Obstacle collision
            clamped_gx, clamped_gy, _ = GridWorldTransform.clamp_grid(orig_target_gx, orig_target_gy)
            if self.env.grid[clamped_gx, clamped_gy] == -1:
                self.get_logger().warn(f"[{self.agents[idx].config.drone_id}] Obstacle collision prevented! Falling back to hover.")
                safety_override = True

            # Swap override
            if idx in swap_overrides:
                self.get_logger().warn(f"[{self.agents[idx].config.drone_id}] Cell-swap collision prevented! Falling back to hover.")
                safety_override = True

            # Multi-agent occupancy check
            if (safe_gx, safe_gy) in claimed_cells:
                self.get_logger().warn(f"[{self.agents[idx].config.drone_id}] Occupancy conflict at ({safe_gx},{safe_gy})! Falling back to hover.")
                safe_gx, safe_gy = curr_gx, curr_gy
                action = 4
                safety_override = True
                if (safe_gx, safe_gy) in claimed_cells:
                    pass

            claimed_cells.add((safe_gx, safe_gy))
            final_actions.append((safe_gx, safe_gy, action, safety_override))

        # 2. Dispatch all finalized actions
        for idx, agent in enumerate(self.agents):
            safe_gx, safe_gy, action, safety_override = final_actions[idx]
            curr_gx, curr_gy = self.env.drone_positions[idx]

            if safety_override:
                self.safety_overrides_count += 1
                if action == 4:
                    self.safety_forced_hover_count += 1
            else:
                if action == 4:
                    self.natural_hover_count += 1

            assigned_task = self.task_allocator.get_assigned_task_for_drone(agent.config.drone_id)
            if assigned_task is not None and assigned_task.state in (TaskState.ASSIGNED, TaskState.APPROACHING, TaskState.INVESTIGATING):
                dist_grid = math.hypot(curr_gx - assigned_task.grid_x, curr_gy - assigned_task.grid_y)
                if dist_grid <= 1.5 or (safe_gx == assigned_task.grid_x and safe_gy == assigned_task.grid_y) or assigned_task.state == TaskState.INVESTIGATING:
                    target_wx, target_wy = assigned_task.world_x, assigned_task.world_y
                else:
                    target_wx, target_wy = GridWorldTransform.grid_to_world_center(safe_gx, safe_gy)
                is_hover = (action == 4) or (assigned_task.state == TaskState.INVESTIGATING)
            else:
                target_wx, target_wy = GridWorldTransform.grid_to_world_center(safe_gx, safe_gy)
                is_hover = (action == 4)

            local_north, local_east = GridWorldTransform.world_enu_to_px4_ned(
                target_wx, target_wy, agent.config.world_spawn_x, agent.config.world_spawn_y
            )
            lz = agent.initial_pos[2] - agent.mission.takeoff_altitude

            if agent.state not in (FlightState.FAILSAFE, FlightState.COMPLETE, FlightState.DISARM):
                agent.set_mission_goal_local(local_north, local_east, lz, require_min_dwell=is_hover)
                agent.state_timer = 0
                agent._log_transition(FlightState.WAYPOINT_NAVIGATION)

            log_action_name = f"SAR_{assigned_task.state.value}:{action_names[action]}" if (assigned_task is not None and assigned_task.state in (TaskState.ASSIGNED, TaskState.APPROACHING, TaskState.INVESTIGATING)) else action_names[action]
            self.last_actions[agent.config.drone_id] = log_action_name
            self.last_safety_overrides[agent.config.drone_id] = safety_override

            # Use original QMIX target for logging (before safety override)
            orig_action = actions[idx]
            orig_tgt_gx, orig_tgt_gy = curr_gx, curr_gy
            if orig_action == 0: orig_tgt_gx += 1
            elif orig_action == 1: orig_tgt_gx -= 1
            elif orig_action == 2: orig_tgt_gy += 1
            elif orig_action == 3: orig_tgt_gy -= 1

            self.log_decision(agent, curr_gx, curr_gy, action, log_action_name, orig_tgt_gx, orig_tgt_gy, target_wx, target_wy, latencies[idx], safety_override)
            self.get_logger().info(f"[{agent.config.drone_id}] Action: {log_action_name} -> Grid ({safe_gx},{safe_gy}) -> World ({target_wx:.1f}, {target_wy:.1f})")

    def log_decision(self, agent, gx, gy, action, action_name, tgt_gx, tgt_gy, tgt_wx, tgt_wy, inf_time, safety):
        local_north, local_east, local_down = agent.px4.current_position
        curr_wx, curr_wy = GridWorldTransform.px4_ned_to_world_enu(
            local_north, local_east, agent.config.world_spawn_x, agent.config.world_spawn_y
        )
        wz = -local_down + getattr(agent.config, 'world_spawn_z', 0.2)

        valid_cells = (self.env.grid != -1).sum()
        explored = (self.env.grid == 1).sum()
        coverage = explored / valid_cells if valid_cells > 0 else 0
        victims = sum(self.env.victims.values())

        dx = curr_wx - tgt_wx
        dy = curr_wy - tgt_wy
        dist = (dx**2 + dy**2)**0.5

        t = self.get_clock().now().nanoseconds / 1e9

        self.csv_writer.writerow([
            t, self.decisions_made, agent.config.drone_id,
            gx, gy, curr_wx, curr_wy, wz,
            action, action_name,
            tgt_gx, tgt_gy, tgt_wx, tgt_wy,
            dist, coverage, victims, inf_time, safety
        ])
        self.log_file.flush()

        # Log Trajectories
        drone_id_str = str(agent.config.drone_id)
        if drone_id_str in self.trajectory_writers:
            self.trajectory_writers[drone_id_str].writerow([t, curr_wx, curr_wy, wz, gx, gy])
            self.trajectory_files[drone_id_str].flush()

    def destroy_node(self):
        self.log_file.close()
        for f in self.trajectory_files.values():
            f.close()
        self.victim_file.close()
        super().destroy_node()


def main(args=None):
    """
    ROS 2 executable entry point for QMIXMissionController.
    Initializes rclpy, spins the controller node, and cleanly shuts down.
    """
    rclpy.init(args=args)
    node = None
    try:
        from swarm_controller.configs import DroneConfig, MissionConfig
        project_root = os.path.expanduser('~/capstone_project_antigravity')
        meta_file = os.path.join(project_root, 'worlds', 'earthquake_world_meta.json')
        checkpoint_path = os.path.join(project_root, 'models', 'qmix_sar_v4_align_best.pth')
        mission_config = MissionConfig()
        drone_configs = [DroneConfig(
            drone_id="drone_0",
            namespace="drone_0",
            system_id=1,
            world_spawn_x=0.0,
            world_spawn_y=0.0,
            world_spawn_z=0.2,
            world_yaw=0.0
        )]
        node = QMIXMissionController(
            mission_config=mission_config,
            drone_configs=drone_configs,
            checkpoint_path=checkpoint_path,
            max_decisions=1,
            meta_file=meta_file
        )
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
