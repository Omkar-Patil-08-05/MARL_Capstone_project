"""
Decentralized Autonomous Drone Node (Corrected Architecture)
Protocol Version: 2.1-P2P

Key Corrections:
- Strict 2-Agent 29-Dimensional Observation Vector strictly preserving checkpoint input shape.
- Teammate Observation Staleness & Explicit Isolated-Mode Fallback (zeroed relative vector + BFS frontier exploration).
- High-level discrete QMIX policy decision separated from low-level continuous 20 Hz setpoint stream.
- Ingestion of explicit ACK, Journal Sync Request/Response, and Task Claim messages.
- Evasion trajectory execution for CONFLICT_RESOLVED_LATERAL_ESCAPE (hover is not treated as universally safe).
- Continuous wire bytes and payload bytes accounting.
"""

import os
import sys
import math
import time
import json
import random
import numpy as np
from typing import Dict, List, Optional, Tuple, Set, Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from swarm_controller.configs import DroneConfig, MissionConfig
from swarm_controller.drone_agent import DroneAgent, FlightState
from swarm_controller.grid_world_transform import GridWorldTransform
from swarm_controller.px4_interface import PX4Interface
from .qmix_adapter import QMIXAdapter

from .peer_message_schema import (
    PeerMessage,
    MessageType,
    MessagePriority
)
from .network_impairment import NetworkImpairmentConfig, NetworkImpairmentSimulator
from .peer_transport import BasePeerTransport, ROSTopicPeerTransport, LoopbackPeerTransport
from .peer_state_manager import PeerStateManager, CommsHealthState
from .peer_safety_envelope import PeerSafetyEnvelope, SafetyVerdict
from .risk_triggered_scheduler import RiskTriggeredScheduler
from .distributed_event_journal import DistributedEventJournal

class DecentralizedDroneNode(Node):
    def __init__(
        self,
        drone_id: str,
        drone_index: int,
        drone_config: DroneConfig,
        mission_config: MissionConfig,
        checkpoint_path: str,
        meta_file: Optional[str] = None,
        peer_ids: Optional[List[str]] = None,
        max_decisions: int = 30,
        impairment_config: Optional[NetworkImpairmentConfig] = None,
        use_loopback_transport: bool = False,
        mock_px4: bool = False,
        use_sim_time: Optional[bool] = None,
        telemetry_only: bool = False,
        takeoff_hold_only: bool = False,
        enable_phase5_fallback: bool = False,
        fault_scenario: Optional[str] = None,
        fault_seed: Optional[int] = None,
        candidate_world_victims: Optional[List[Dict[str, Any]]] = None,
        enable_safety_envelope: bool = True,
        enable_task_claims: bool = True,
        silent_mode: bool = False,
        policy_type: str = "qmix",
        extra_obstacle_cells: Optional[Set[Tuple[int, int]]] = None,
        gps_noise_std_m: float = 0.0,
        gps_bias_m: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        gps_seed: Optional[int] = None,
        camera_noise_std_m: Optional[float] = None,
        camera_min_confidence: Optional[float] = None,
        camera_occlusion_rate: float = 0.0,
        initial_battery_pct: float = 100.0,
        battery_drain_rate_per_m: float = 0.35,
        battery_safety_margin_pct: float = 15.0,
        enable_battery_rtb: bool = False,
        enable_adaptive_reallocation: bool = False,
        sensor_seed: Optional[int] = None
    ):
        from rclpy.parameter import Parameter
        param_overrides = []
        if use_sim_time is not None:
            param_overrides.append(Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time))

        super().__init__(f'decentralized_drone_{drone_id}', parameter_overrides=param_overrides)
        self.drone_id = drone_id
        self.drone_index = drone_index
        self.drone_config = drone_config
        self.mission_config = mission_config
        self.checkpoint_path = checkpoint_path
        self.meta_file = meta_file
        self.peer_ids = [p for p in (peer_ids or []) if p != drone_id]
        self.max_decisions = max_decisions
        self.mock_px4 = mock_px4
        self.telemetry_only = telemetry_only
        self.takeoff_hold_only = takeoff_hold_only
        self.enable_phase5_fallback = enable_phase5_fallback
        self.fault_scenario = fault_scenario
        self.fault_seed = fault_seed
        self.candidate_world_victims = list(candidate_world_victims or [])
        self.enable_safety_envelope = enable_safety_envelope
        self.enable_task_claims = enable_task_claims
        self.silent_mode = silent_mode
        self.policy_type = policy_type
        self.extra_obstacle_cells = set(extra_obstacle_cells or [])
        self.gps_noise_std_m = gps_noise_std_m
        self.gps_bias_m = gps_bias_m
        self.gps_rng = random.Random(gps_seed) if gps_seed is not None else random.Random(42 + drone_index)
        self.camera_noise_std_m = camera_noise_std_m if camera_noise_std_m is not None else 0.35
        self.camera_min_confidence = camera_min_confidence if camera_min_confidence is not None else 0.70
        self.camera_occlusion_rate = camera_occlusion_rate
        self.battery_pct = float(initial_battery_pct)
        self.battery_drain_rate_per_m = float(battery_drain_rate_per_m)
        self.battery_safety_margin_pct = float(battery_safety_margin_pct)
        self.enable_battery_rtb = enable_battery_rtb
        self.enable_adaptive_reallocation = enable_adaptive_reallocation
        self.last_step_world_pos: Optional[Tuple[float, float, float]] = None
        self._lawnmower_direction = 1
        self.sensor_seed = sensor_seed if sensor_seed is not None else (42 + drone_index)
        from .victim_perception import SimulatedVictimSensor, PerceptionTrackletTracker
        self.victim_sensor = SimulatedVictimSensor(
            drone_id=self.drone_id,
            noise_std_m=self.camera_noise_std_m,
            min_detection_confidence=self.camera_min_confidence,
            occlusion_rate=self.camera_occlusion_rate,
            seed=self.sensor_seed
        )
        self.tracklet_tracker = PerceptionTrackletTracker(
            drone_id=self.drone_id,
            tracklet_timeout_sec=4.0
        )
        self.passive_safety_diagnostics: Dict[str, Any] = {}
        self.last_passive_log_wall_time: float = 0.0
        self.last_flight_log_wall_time: float = 0.0

        # Read use_sim_time parameter (already declared automatically by rclpy in ROS 2 Jazzy)
        self.is_sim_time = self.get_parameter('use_sim_time').value
        self.get_logger().info(
            f"[{self.drone_id}] Initializing DecentralizedDroneNode "
            f"(use_sim_time={self.is_sim_time}, telemetry_only={self.telemetry_only})"
        )

        # Low-level flight agent managing PX4 offboard interface
        self.agent = DroneAgent(self, self.drone_config, self.mission_config, telemetry_only=self.telemetry_only)
        if self.mock_px4:
            self.agent.px4.is_connected = lambda: True
            self.agent.px4.is_offboard = lambda: True
            self.agent.px4.is_estimator_ready = lambda: True
            if not self.telemetry_only:
                self.agent.state = FlightState.HOLD
                self.agent.initial_pos = [0.0, 0.0, -15.0]
            else:
                self.agent.state = FlightState.WAIT_FOR_ESTIMATOR
                self.agent.initial_pos = [0.0, 0.0, 0.0]
            self.get_logger().info(f"[{self.drone_id}] Mock PX4 state enabled for dry-run verification.")

        # Peer state manager with silence-age FSM
        self.state_manager = PeerStateManager(
            local_drone_id=self.drone_id,
            degraded_timeout_sec=1.5,
            isolated_timeout_sec=5.0,
            unknown_timeout_sec=20.0
        )
        for peer_id in self.peer_ids:
            self.state_manager.register_peer(peer_id)

        # Candidate A: 3D Ellipsoidal Safety Envelope
        self.safety_envelope = PeerSafetyEnvelope(
            local_drone_id=self.drone_id,
            state_manager=self.state_manager,
            base_radius_xy_m=1.40,
            base_radius_z_m=1.00,
            max_accel_xy_m_s2=3.0,
            max_accel_z_up_m_s2=4.0,
            max_accel_z_down_m_s2=3.0,
            max_vel_xy_m_s=3.0,
            max_vel_z_m_s=3.0,
            operational_limit_radius_m=16.4,
            collision_buffer_m=1.05
        )

        # Candidate B: Risk-Triggered Scheduler
        self.scheduler = RiskTriggeredScheduler(
            local_drone_id=self.drone_id,
            mission_id="p2p_sar_mission",
            heartbeat_hz=1.0
        )

        # Candidate C: Partition-Resilient Event Journal
        self.event_journal = DistributedEventJournal(
            local_drone_id=self.drone_id,
            spatial_merge_threshold_m=6.0,
            default_lease_duration_sec=30.0
        )

        # Transport layer
        self.use_loopback_transport: bool = use_loopback_transport
        self.impairment_sim = NetworkImpairmentSimulator(impairment_config or NetworkImpairmentConfig())
        if self.fault_scenario is not None:
            self.impairment_sim.set_scenario(self.fault_scenario, seed=self.fault_seed)
        if self.silent_mode:
            self.impairment_sim.config.packet_loss_rate = 1.0
        if use_loopback_transport:
            self.transport: BasePeerTransport = LoopbackPeerTransport(self.drone_id, self.impairment_sim)
        else:
            self.transport = ROSTopicPeerTransport(self, self.drone_id, self.impairment_sim)
            # Subscribe to external sensor discovery (Gazebo Harmonic + YOLOv8n + RGB-D Localizer)
            self.discovery_sub = self.create_subscription(
                String,
                '/sar/victim_discovery',
                self._external_victim_discovery_cb,
                10
            )

        # Private QMIX instance retaining 2-agent, 29-dim checkpoint
        self.qmix = QMIXAdapter(self.checkpoint_path, num_agents=2)

        # Configure GridWorldTransform bounds based on meta_file
        if self.meta_file and os.path.exists(self.meta_file):
            if "forest" in os.path.basename(self.meta_file).lower():
                GridWorldTransform.set_bounds(width=25, height=25, meters_per_cell=8.0, origin_x=-180.0, origin_y=-20.0)
                self.agent.required_stable_ticks = 40  # 2.0s sim time (calibrated for heightmap physics RTF)
            else:
                GridWorldTransform.set_bounds(width=25, height=25, meters_per_cell=4.0, origin_x=0.0, origin_y=0.0)

        # Local grid state
        self.grid_x_size = GridWorldTransform.GRID_SIZE_X
        self.grid_y_size = GridWorldTransform.GRID_SIZE_Y
        self.local_grid = np.zeros((self.grid_x_size, self.grid_y_size), dtype=np.int8)
        self.explored_cells: Set[Tuple[int, int]] = set()
        self.static_obstacles: Set[Tuple[int, int]] = set()
        self._load_static_obstacles()

        # Operational counters
        self.decisions_made: int = 0
        self.safety_overrides_count: int = 0
        self.active_task_id: Optional[str] = None
        self.last_target_grid: Optional[Tuple[int, int]] = None
        self.isolated_fallback_active: bool = False
        self.sar_mission_authorized: bool = not takeoff_hold_only
        self.mission_completed_logged: bool = False
        self.decision_records: List[Dict[str, Any]] = []
        self.unique_explored_cells: Set[Tuple[int, int]] = set()

        # Flight authorization service (std_srvs/srv/Trigger)
        self.auth_srv = self.create_service(
            Trigger,
            f'/{self.drone_id}/authorize_takeoff',
            self._handle_authorize_takeoff
        )

        # SAR Mission authorization service (std_srvs/srv/Trigger)
        self.sar_auth_srv = self.create_service(
            Trigger,
            f'/{self.drone_id}/authorize_sar_mission',
            self._handle_authorize_sar_mission
        )

        # Return to base service (std_srvs/srv/Trigger)
        self.rtb_srv = self.create_service(
            Trigger,
            f'/{self.drone_id}/trigger_rtb',
            self._handle_trigger_rtb
        )

        # Landing authorization service (std_srvs/srv/Trigger)
        self.landing_auth_srv = self.create_service(
            Trigger,
            f'/{self.drone_id}/authorize_landing',
            self._handle_authorize_landing
        )

        # 20 Hz primary control timer
        self.timer = self.create_timer(0.05, self._control_loop_tick)
        self.get_logger().info(f"[{self.drone_id}] Decentralized P2P Drone Node initialized successfully.")

    def _handle_trigger_rtb(self, request, response):
        self.get_logger().info(f"[{self.drone_id}] Return-to-Base (RTB) requested via service.")
        self.agent.trigger_return_to_base()
        response.success = True
        response.message = f"[{self.drone_id}] RTB initiated. Returning to spawn base."
        return response

    def _handle_authorize_landing(self, request, response):
        self.get_logger().info(f"[{self.drone_id}] Landing authorization requested via service.")
        self.agent.authorize_landing()
        response.success = True
        response.message = f"[{self.drone_id}] Landing descent authorized."
        return response

    def _handle_authorize_takeoff(self, request, response):
        """
        Validates all 12 preflight gates before authorizing flight actuation.
        Enforces sequential takeoff gate: drone_1 requires drone_0 airborne in ALTITUDE_ACQUISITION or HOLD.
        """
        now = time.monotonic()
        px4 = self.agent.px4

        # 1. PX4 Interface connected
        if not px4.is_connected():
            response.success = False
            response.message = f"Preflight FAIL: {self.drone_id} PX4 interface not connected (odometry_count={px4.odometry_count})"
            self.get_logger().error(response.message)
            return response

        # 2. Telemetry freshness (< 1.0s)
        if self.mock_px4:
            telem_age = 0.0
        else:
            telem_age = (time.time() - px4.last_odometry_wall_time) if px4.odometry_count > 0 else float('inf')
        if telem_age > 1.0:
            response.success = False
            response.message = f"Preflight FAIL: {self.drone_id} odometry stale (age={telem_age:.2f}s > 1.0s)"
            self.get_logger().error(response.message)
            return response

        # 3. Telemetry position finite
        if not all(math.isfinite(x) for x in px4.current_position):
            response.success = False
            response.message = f"Preflight FAIL: {self.drone_id} non-finite position {px4.current_position}"
            self.get_logger().error(response.message)
            return response

        # 4. Telemetry velocity finite
        if not all(math.isfinite(x) for x in px4.current_velocity):
            response.success = False
            response.message = f"Preflight FAIL: {self.drone_id} non-finite velocity {px4.current_velocity}"
            self.get_logger().error(response.message)
            return response

        # 5. Estimator ready
        if not px4.is_estimator_ready():
            response.success = False
            response.message = f"Preflight FAIL: {self.drone_id} estimator not ready (pre_flight_checks_pass={px4.pre_flight_checks_pass})"
            self.get_logger().error(response.message)
            return response

        # 6. Vehicle disarmed
        if px4.is_armed():
            response.success = False
            response.message = f"Preflight FAIL: {self.drone_id} vehicle already armed"
            self.get_logger().error(response.message)
            return response

        # 7. Simulation clock advancing
        curr_sim = self.agent.get_current_sim_time()
        if self.is_sim_time and not self.mock_px4 and curr_sim <= 0.0 and px4.px4_timestamp_sec <= 0.0:
            response.success = False
            response.message = f"Preflight FAIL: {self.drone_id} simulation clock not advancing"
            self.get_logger().error(response.message)
            return response

        # 8. Vertical takeoff column clear of static obstacles
        curr_gx, curr_gy = self._get_current_grid_cell()
        if (curr_gx, curr_gy) in self.static_obstacles:
            response.success = False
            response.message = f"Preflight FAIL: {self.drone_id} spawn cell ({curr_gx}, {curr_gy}) inside static obstacle"
            self.get_logger().error(response.message)
            return response

        # 9. Peer registered in state manager
        for p_id in self.peer_ids:
            if p_id not in self.state_manager.peers:
                response.success = False
                response.message = f"Preflight FAIL: Peer {p_id} not registered in state manager"
                self.get_logger().error(response.message)
                return response

        # 10. Peer position valid and finite
        curr_wx, curr_wy, curr_wz = self._get_current_world_enu()
        for p_id in self.peer_ids:
            peer = self.state_manager.peers[p_id]
            if not self.mock_px4 and peer.total_packets_received == 0:
                response.success = False
                response.message = f"Preflight FAIL: Peer {p_id} telemetry not yet received (total_packets=0)"
                self.get_logger().error(response.message)
                return response
            if not all(math.isfinite(x) for x in peer.last_position_enu):
                response.success = False
                response.message = f"Preflight FAIL: Peer {p_id} position has non-finite values"
                self.get_logger().error(response.message)
                return response

        # 11. Peer separation safe (>= 5.0m)
        for p_id in self.peer_ids:
            peer = self.state_manager.peers[p_id]
            dx = curr_wx - peer.last_position_enu[0]
            dy = curr_wy - peer.last_position_enu[1]
            dz = curr_wz - peer.last_position_enu[2]
            sep = math.hypot(dx, dy, dz)
            if sep < 5.0 and peer.total_packets_received > 0:
                response.success = False
                response.message = f"Preflight FAIL: Peer {p_id} separation {sep:.2f}m < 5.0m safety threshold"
                self.get_logger().error(response.message)
                return response

        # 12. Peer safety verdict is SAFE
        verdict_res = self.safety_envelope.screen_trajectory_3d(
            current_grid=[curr_gx, curr_gy],
            candidate_grid=[curr_gx, curr_gy],
            current_world_enu=[curr_wx, curr_wy, curr_wz],
            candidate_world_enu=[curr_wx, curr_wy, curr_wz],
            candidate_action=4,
            grid_obstacles=self.static_obstacles,
            grid_size=self.grid_x_size,
            current_time=now
        )
        if verdict_res.verdict != SafetyVerdict.SAFE:
            response.success = False
            response.message = f"Preflight FAIL: Peer safety verdict is {verdict_res.verdict.name} (not SAFE)"
            self.get_logger().error(response.message)
            return response

        # Sequential Takeoff Gate: If this node is drone_1, ensure drone_0 is already airborne in verified stable HOLD
        if self.drone_id == "drone_1" and "drone_0" in self.state_manager.peers:
            peer_0 = self.state_manager.peers["drone_0"]
            silence_0 = self.state_manager.get_silence_age_sec("drone_0", now)
            if silence_0 > 2.0:
                response.success = False
                response.message = f"Sequential Gate FAIL: drone_0 telemetry stale (silence_age={silence_0:.2f}s > 2.0s)"
                self.get_logger().warn(response.message)
                return response
            if peer_0.flight_state != "HOLD":
                response.success = False
                response.message = (
                    f"Sequential Gate FAIL: drone_0 flight state is '{peer_0.flight_state}', "
                    f"must be verified stable HOLD before drone_1 takeoff"
                )
                self.get_logger().warn(response.message)
                return response
            if not getattr(peer_0, "stability_verified", False):
                response.success = False
                response.message = "Sequential Gate FAIL: drone_0 has not verified stable hold window"
                self.get_logger().warn(response.message)
                return response
            if not getattr(peer_0, "is_armed", False):
                response.success = False
                response.message = "Sequential Gate FAIL: drone_0 is not armed"
                self.get_logger().warn(response.message)
                return response
            if not getattr(peer_0, "is_offboard", False):
                response.success = False
                response.message = "Sequential Gate FAIL: drone_0 is not in offboard mode"
                self.get_logger().warn(response.message)
                return response

        # All checks passed! Authorize flight actuation
        self.agent.authorize_takeoff()
        response.success = True
        response.message = f"Preflight validation PASSED. Takeoff authorized for {self.drone_id}."
        self.get_logger().info(f"[{self.drone_id}] *** {response.message} ***")
        return response

    def _handle_authorize_sar_mission(self, request, response):
        """
        Validates all 14 mission preconditions before releasing autonomous SAR exploration.
        Preconditions:
        1. Local PX4 connected
        2. Telemetry fresh (< 1.0s)
        3. Local position and velocity finite
        4. Estimator ready
        5. Vehicle armed
        6. Vehicle in offboard mode (nav_state == 14)
        7. Local flight state == HOLD
        8. Stability window complete (stable_hold_ticks >= required_stable_ticks)
        9. Peer registered in state manager
        10. Peer telemetry fresh (< 2.0s)
        11. Peer reports verified stable HOLD (is_armed, is_offboard, stability_verified)
        12. World ENU separation safe (>= 5.0m)
        13. Safety envelope verdict is SAFE
        14. Simulation time advancing correctly
        """
        now = time.monotonic()
        px4 = self.agent.px4

        # 1. PX4 connected
        if not px4.is_connected():
            response.success = False
            response.message = f"SAR Auth FAIL: {self.drone_id} PX4 interface not connected"
            self.get_logger().error(response.message)
            return response

        # 2. Telemetry fresh (< 1.0s)
        telem_age = 0.0 if self.mock_px4 else ((time.time() - px4.last_odometry_wall_time) if px4.odometry_count > 0 else float('inf'))
        if telem_age > 1.0:
            response.success = False
            response.message = f"SAR Auth FAIL: {self.drone_id} odometry stale (age={telem_age:.2f}s > 1.0s)"
            self.get_logger().error(response.message)
            return response

        # 3. Position and velocity finite
        if not all(math.isfinite(x) for x in px4.current_position):
            response.success = False
            response.message = f"SAR Auth FAIL: {self.drone_id} non-finite position {px4.current_position}"
            self.get_logger().error(response.message)
            return response
        if not all(math.isfinite(x) for x in px4.current_velocity):
            response.success = False
            response.message = f"SAR Auth FAIL: {self.drone_id} non-finite velocity {px4.current_velocity}"
            self.get_logger().error(response.message)
            return response

        # 4. Estimator ready
        if not px4.is_estimator_ready():
            response.success = False
            response.message = f"SAR Auth FAIL: {self.drone_id} estimator not ready"
            self.get_logger().error(response.message)
            return response

        # 5. Vehicle armed
        if not px4.is_armed():
            response.success = False
            response.message = f"SAR Auth FAIL: {self.drone_id} vehicle not armed"
            self.get_logger().error(response.message)
            return response

        # 6. Offboard mode active (nav_state == 14)
        if not px4.is_offboard():
            response.success = False
            response.message = f"SAR Auth FAIL: {self.drone_id} not in offboard mode (nav_state={px4.nav_state})"
            self.get_logger().error(response.message)
            return response

        # 7. Local flight state == HOLD
        if self.agent.state != FlightState.HOLD:
            response.success = False
            response.message = f"SAR Auth FAIL: {self.drone_id} local flight state is {self.agent.state.name} (must be HOLD)"
            self.get_logger().error(response.message)
            return response

        # 8. Stability window complete
        if self.agent.stable_hold_ticks < self.agent.required_stable_ticks:
            response.success = False
            response.message = (
                f"SAR Auth FAIL: {self.drone_id} stability window incomplete "
                f"({self.agent.stable_hold_ticks}/{self.agent.required_stable_ticks} ticks)"
            )
            self.get_logger().error(response.message)
            return response

        # 9. Peer registered
        for p_id in self.peer_ids:
            if p_id not in self.state_manager.peers:
                response.success = False
                response.message = f"SAR Auth FAIL: Peer {p_id} not registered"
                self.get_logger().error(response.message)
                return response

        # 10. Peer telemetry fresh (< 2.0s)
        for p_id in self.peer_ids:
            peer = self.state_manager.peers[p_id]
            if not self.mock_px4 and peer.total_packets_received == 0:
                response.success = False
                response.message = f"SAR Auth FAIL: Peer {p_id} telemetry not received"
                self.get_logger().error(response.message)
                return response
            silence = self.state_manager.get_silence_age_sec(p_id, now)
            if silence > 2.0:
                response.success = False
                response.message = f"SAR Auth FAIL: Peer {p_id} telemetry stale (silence={silence:.2f}s > 2.0s)"
                self.get_logger().error(response.message)
                return response

        # 11. Peer reports verified stable HOLD
        for p_id in self.peer_ids:
            peer = self.state_manager.peers[p_id]
            if peer.flight_state != "HOLD":
                response.success = False
                response.message = f"SAR Auth FAIL: Peer {p_id} flight state is '{peer.flight_state}' (must be HOLD)"
                self.get_logger().error(response.message)
                return response
            if not getattr(peer, "stability_verified", False):
                response.success = False
                response.message = f"SAR Auth FAIL: Peer {p_id} has not verified stable hold window"
                self.get_logger().error(response.message)
                return response
            if not getattr(peer, "is_armed", False):
                response.success = False
                response.message = f"SAR Auth FAIL: Peer {p_id} is not armed"
                self.get_logger().error(response.message)
                return response
            if not getattr(peer, "is_offboard", False):
                response.success = False
                response.message = f"SAR Auth FAIL: Peer {p_id} is not in offboard mode"
                self.get_logger().error(response.message)
                return response

        # 12. World ENU separation safe (>= 5.0m)
        curr_wx, curr_wy, curr_wz = self._get_current_world_enu()
        for p_id in self.peer_ids:
            peer = self.state_manager.peers[p_id]
            dx = curr_wx - peer.last_position_enu[0]
            dy = curr_wy - peer.last_position_enu[1]
            dz = curr_wz - peer.last_position_enu[2]
            sep = math.hypot(dx, dy, dz)
            if sep < 5.0:
                response.success = False
                response.message = f"SAR Auth FAIL: Peer {p_id} separation {sep:.2f}m < 5.0m"
                self.get_logger().error(response.message)
                return response

        # 13. Safety envelope reports SAFE
        curr_gx, curr_gy = self._get_current_grid_cell()
        verdict_res = self.safety_envelope.screen_trajectory_3d(
            current_grid=[curr_gx, curr_gy],
            candidate_grid=[curr_gx, curr_gy],
            current_world_enu=[curr_wx, curr_wy, curr_wz],
            candidate_world_enu=[curr_wx, curr_wy, curr_wz],
            candidate_action=4,
            grid_obstacles=self.static_obstacles,
            grid_size=self.grid_x_size,
            current_time=now
        )
        if verdict_res.verdict != SafetyVerdict.SAFE:
            response.success = False
            response.message = f"SAR Auth FAIL: Safety envelope reports {verdict_res.verdict.name} (reason: {verdict_res.reason})"
            self.get_logger().error(response.message)
            return response

        # 14. Simulation clock advancing
        curr_sim = self.agent.get_current_sim_time()
        if self.is_sim_time and not self.mock_px4 and curr_sim <= 0.0 and px4.px4_timestamp_sec <= 0.0:
            response.success = False
            response.message = f"SAR Auth FAIL: Simulation clock not advancing"
            self.get_logger().error(response.message)
            return response

        # All 14 preconditions passed! Authorize SAR exploration
        self.sar_mission_authorized = True
        self.takeoff_hold_only = False
        response.success = True
        response.message = f"SAR mission authorization PASSED. Autonomous SAR exploration enabled for {self.drone_id}."
        self.get_logger().info(f"[{self.drone_id}] *** {response.message} ***")
        return response

    def _load_static_obstacles(self):
        """Loads static obstacles from metadata into local grid."""
        if self.meta_file and os.path.exists(self.meta_file):
            try:
                with open(self.meta_file, 'r') as f:
                    meta = json.load(f)
                for obs in meta.get("obstacles", []):
                    aabb = obs.get("aabb", {})
                    min_x, max_x = aabb.get("min_x", 0.0), aabb.get("max_x", 0.0)
                    min_y, max_y = aabb.get("min_y", 0.0), aabb.get("max_y", 0.0)
                    gx1, gy1 = GridWorldTransform.world_to_grid(min_x, min_y)
                    gx2, gy2 = GridWorldTransform.world_to_grid(max_x, max_y)
                    for gx in range(min(gx1, gx2), max(gx1, gx2) + 1):
                        for gy in range(min(gy1, gy2), max(gy1, gy2) + 1):
                            if 0 <= gx < self.grid_x_size and 0 <= gy < self.grid_y_size:
                                self.local_grid[gx, gy] = -1
                                self.static_obstacles.add((gx, gy))
            except Exception as e:
                self.get_logger().warn(f"[{self.drone_id}] Failed loading metadata obstacles: {e}")

        # Ingest extra environmental clutter/obstacle cells if configured
        if self.extra_obstacle_cells:
            for gx, gy in self.extra_obstacle_cells:
                if 0 <= gx < self.grid_x_size and 0 <= gy < self.grid_y_size:
                    self.local_grid[gx, gy] = -1
                    self.static_obstacles.add((gx, gy))

        self.safety_envelope.set_obstacle_cells(self.static_obstacles)
        self.get_logger().info(f"[{self.drone_id}] Loaded {len(self.static_obstacles)} obstacle cells (meta + extra).")

    def _get_ground_truth_world_enu(self) -> Tuple[float, float, float]:
        """Returns true physical world ENU position without sensor corruption."""
        local_north, local_east, local_down = self.agent.px4.current_position
        wx, wy = GridWorldTransform.px4_ned_to_world_enu(
            local_north, local_east, self.drone_config.world_spawn_x, self.drone_config.world_spawn_y
        )
        wz = -local_down + getattr(self.drone_config, 'world_spawn_z', 0.2)
        return (wx, wy, wz)

    def _get_current_world_enu(self) -> Tuple[float, float, float]:
        """Returns estimated world ENU position subject to GPS sensor noise and bias."""
        wx, wy, wz = self._get_ground_truth_world_enu()
        if self.gps_noise_std_m > 0.0 or any(abs(b) > 1e-4 for b in self.gps_bias_m):
            noisy_x = wx + self.gps_rng.gauss(0.0, self.gps_noise_std_m) + self.gps_bias_m[0]
            noisy_y = wy + self.gps_rng.gauss(0.0, self.gps_noise_std_m) + self.gps_bias_m[1]
            noisy_z = wz + self.gps_rng.gauss(0.0, self.gps_noise_std_m * 0.5) + self.gps_bias_m[2]
            return (noisy_x, noisy_y, noisy_z)
        return (wx, wy, wz)

    def _get_current_grid_cell(self) -> Tuple[int, int]:
        wx, wy, _ = self._get_current_world_enu()
        return GridWorldTransform.world_to_grid(wx, wy)

    def _bfs_frontier_direction(self, start_cell: Tuple[int, int]) -> Tuple[float, float]:
        """Computes unit vector toward nearest unexplored non-obstacle cell using BFS."""
        from collections import deque
        visited = set()
        queue = deque([(start_cell[0], start_cell[1], 0)])
        visited.add(start_cell)

        while queue:
            cx, cy, dist = queue.popleft()
            if self.local_grid[cx, cy] == 0:
                dx = cx - start_cell[0]
                dy = cy - start_cell[1]
                norm = math.hypot(dx, dy)
                if norm > 1e-4:
                    return (dx / norm, dy / norm)
                return (0.0, 0.0)

            for nx, ny in [(cx+1, cy), (cx-1, cy), (cx, cy+1), (cx, cy-1)]:
                if 0 <= nx < self.grid_x_size and 0 <= ny < self.grid_y_size:
                    if (nx, ny) not in visited and self.local_grid[nx, ny] != -1:
                        visited.add((nx, ny))
                        queue.append((nx, ny, dist + 1))
        return (0.0, 0.0)

    def _get_lawnmower_action(self, curr_gx: int, curr_gy: int) -> int:
        """
        Deterministic Boustrophedon (lawnmower) coverage path planner.
        Partitions the grid:
          Drone 0: Western sector gx in [2, 11]
          Drone 1: Eastern sector gx in [13, 22]
        """
        min_gx = 2 if self.drone_index == 0 else 13
        max_gx = 11 if self.drone_index == 0 else 22
        min_gy, max_gy = 2, 22

        # If not yet in assigned sector, navigate horizontally
        if curr_gx < min_gx:
            return 0  # +X
        if curr_gx > max_gx:
            return 1  # -X

        # Standard vertical boustrophedon sweep
        if self._lawnmower_direction > 0:
            # Moving +Y
            if curr_gy < max_gy and (curr_gx, curr_gy + 1) not in self.static_obstacles:
                return 2  # +Y
            else:
                # Top boundary reached; step +X if possible, and flip direction
                if curr_gx < max_gx and (curr_gx + 1, curr_gy) not in self.static_obstacles:
                    self._lawnmower_direction = -1
                    return 0  # +X
                elif (curr_gx, curr_gy - 1) not in self.static_obstacles:
                    self._lawnmower_direction = -1
                    return 3  # -Y
                return 4  # Hover
        else:
            # Moving -Y
            if curr_gy > min_gy and (curr_gx, curr_gy - 1) not in self.static_obstacles:
                return 3  # -Y
            else:
                # Bottom boundary reached; step +X if possible, and flip direction
                if curr_gx < max_gx and (curr_gx + 1, curr_gy) not in self.static_obstacles:
                    self._lawnmower_direction = 1
                    return 0  # +X
                elif (curr_gx, curr_gy + 1) not in self.static_obstacles:
                    self._lawnmower_direction = 1
                    return 2  # +Y
                return 4  # Hover

    def _build_local_observation(self, curr_gx: int, curr_gy: int) -> np.ndarray:
        """
        Constructs the strict 29-dimensional observation vector for QMIX.
        Dimensions:
        - Own pos normalized (2)
        - Agent one-hot (2)
        - Peer pos normalized (2)
        - Teammate relative vector (2)
        - BFS frontier vector (2)
        - Local frontier density (1)
        - Local 3x3 FOV obstacles (9)
        - Local 3x3 FOV explored (9)
        """
        own_pos = [curr_gx / self.grid_x_size, curr_gy / self.grid_y_size]

        onehot = [0.0, 0.0]
        idx = min(self.drone_index, 1)
        onehot[idx] = 1.0

        peer_pos = [0.0, 0.0]
        team_vec = [0.0, 0.0]

        if self.peer_ids and not self.silent_mode and self.policy_type != "idqn":
            # Multi-drone nearest-neighbor active peer selection for 29-dim observation
            best_peer_id = None
            best_peer_dist = float('inf')

            for pid in self.peer_ids:
                if pid in self.state_manager.peers:
                    prec = self.state_manager.peers[pid]
                    if prec.comms_health in (CommsHealthState.CONNECTED, CommsHealthState.DEGRADED):
                        pgx, pgy = prec.last_grid_cell
                        d = abs(pgx - curr_gx) + abs(pgy - curr_gy)
                        if d < best_peer_dist:
                            best_peer_dist = d
                            best_peer_id = pid

            p_id = best_peer_id if best_peer_id is not None else self.peer_ids[0]
            if p_id in self.state_manager.peers:
                peer_record = self.state_manager.peers[p_id]
                health = peer_record.comms_health

                if health == CommsHealthState.CONNECTED:
                    self.isolated_fallback_active = False
                    pgx, pgy = peer_record.last_grid_cell
                    peer_pos = [pgx / self.grid_x_size, pgy / self.grid_y_size]
                    dx = pgx - curr_gx
                    dy = pgy - curr_gy
                    norm = max(abs(dx) + abs(dy), 1.0)
                    team_vec = [dx / norm, dy / norm]

                elif health == CommsHealthState.DEGRADED:
                    self.isolated_fallback_active = False
                    # Dead-reckoning peer position
                    silence = self.state_manager.get_silence_age_sec(p_id)
                    vx, vy, _ = peer_record.last_velocity_enu
                    est_wx = peer_record.last_position_enu[0] + vx * silence
                    est_wy = peer_record.last_position_enu[1] + vy * silence
                    pgx, pgy = GridWorldTransform.world_to_grid(est_wx, est_wy)
                    pgx = max(0, min(self.grid_x_size - 1, pgx))
                    pgy = max(0, min(self.grid_y_size - 1, pgy))
                    peer_pos = [pgx / self.grid_x_size, pgy / self.grid_y_size]
                    dx = pgx - curr_gx
                    dy = pgy - curr_gy
                    norm = max(abs(dx) + abs(dy), 1.0)
                    team_vec = [dx / norm, dy / norm]

                else:
                    # ISOLATED or UNKNOWN: Explicit Isolated-Mode Fallback
                    # Zero out teammate relative vector so GRU does not receive misleading cues
                    self.isolated_fallback_active = True
                    peer_pos = [0.0, 0.0]
                    team_vec = [0.0, 0.0]

        # BFS Frontier vector
        fx, fy = self._bfs_frontier_direction((curr_gx, curr_gy))
        frontier_vec = [fx, fy]

        # Local Frontier density in radius 5
        r = 5
        min_x, max_x = max(0, curr_gx - r), min(self.grid_x_size, curr_gx + r + 1)
        min_y, max_y = max(0, curr_gy - r), min(self.grid_y_size, curr_gy + r + 1)
        window = self.local_grid[min_x:max_x, min_y:max_y]
        density = [float(np.sum(window == 0) / max(1, window.size))]

        # Local 3x3 FOV
        local_obs = []
        local_exp = []
        for i in range(-1, 2):
            for j in range(-1, 2):
                cx, cy = curr_gx + i, curr_gy + j
                if 0 <= cx < self.grid_x_size and 0 <= cy < self.grid_y_size:
                    val = self.local_grid[cx, cy]
                    if val == -1:
                        local_obs.append(1.0)
                        local_exp.append(0.0)
                    else:
                        local_obs.append(0.0)
                        local_exp.append(float(val))
                else:
                    local_obs.append(1.0)
                    local_exp.append(0.0)

        obs = np.array(own_pos + onehot + peer_pos + team_vec + frontier_vec + density + local_obs + local_exp, dtype=np.float32)
        return obs

    def _external_victim_discovery_cb(self, msg: String):
        """Ingests confirmed victim detections from RGB-D Localizer / YOLO pipeline."""
        try:
            data = json.loads(msg.data)
            v_id = data.get("id") or data.get("victim_id") or "victim_yolo"
            wx = float(data.get("world_x", 0.0))
            wy = float(data.get("world_y", 0.0))
            wz = float(data.get("world_z", 0.0))
            gx = int(data.get("grid_x", 0))
            gy = int(data.get("grid_y", 0))
            conf = float(data.get("confidence", 0.95))

            # Proximity check or airframe origin attribution
            reporting_drone = data.get("drone_id") or data.get("detected_by")
            px4 = self.agent.px4
            curr_wx, curr_wy, _ = px4.current_position_enu
            dist = math.hypot(wx - curr_wx, wy - curr_wy)

            if reporting_drone == self.drone_id or (reporting_drone is None and dist < 15.0):
                now_tick_utc = time.time()
                t_id, is_fresh_epoch, f_conf, f_unc, f_coords = self.tracklet_tracker.process_detection(
                    raw_target_id=v_id,
                    measured_coords=[wx, wy, wz],
                    uncertainty_m=0.15,
                    confidence=conf,
                    timestamp_utc=now_tick_utc
                )
                rec_id, is_new = self.event_journal.record_local_victim(
                    world_coords=f_coords,
                    grid_cell=[gx, gy],
                    uncertainty_m=f_unc,
                    confidence=f_conf,
                    victim_event_id=v_id,
                    observation_id=t_id
                )
                if is_new or is_fresh_epoch:
                    self.get_logger().info(
                        f"[{self.drone_id}] SENSOR PERCEPTION (YOLO/RGB-D): Ingested victim {rec_id} at {f_coords}"
                    )
                    self.scheduler.trigger_victim_confirmed(
                        victim_event_id=rec_id,
                        world_coords=f_coords,
                        grid_cell=[gx, gy],
                        uncertainty_m=f_unc,
                        confidence=f_conf,
                        observation_id=t_id
                    )
                    task_id = f"task_{rec_id}"
                    if self.enable_task_claims and not self.active_task_id and self.event_journal.claim_task(task_id):
                        self.active_task_id = task_id
                        self.scheduler.trigger_task_claim(
                            task_id=task_id,
                            victim_event_id=rec_id,
                            epoch=self.event_journal.tasks[task_id].ownership_epoch
                        )
        except Exception as e:
            self.get_logger().debug(f"[{self.drone_id}] Error in external victim discovery callback: {e}")

    def _process_inbound_messages(self):
        """Drains incoming peer messages, updates state manager, journal, and dispatches ACKs."""
        messages = self.transport.poll_incoming()
        for msg in messages:
            self.scheduler.record_received_message(msg)
            sender_id = msg.header.sender_drone_id
            prev_health = self.state_manager.peers[sender_id].comms_health if sender_id in self.state_manager.peers else None
            accepted = self.state_manager.record_incoming_message(msg)
            if not accepted:
                continue

            curr_health = self.state_manager.peers[sender_id].comms_health if sender_id in self.state_manager.peers else None
            if prev_health != curr_health and curr_health is not None:
                self.get_logger().info(
                    f"[{self.drone_id}] Peer {sender_id} comms health: {prev_health.name if prev_health else 'NONE'} -> {curr_health.name}"
                )

            msg_type = msg.header.message_type

            if msg_type == MessageType.EMERGENCY_SAFETY:
                action = msg.payload.get("emergency_action", "HOVER_IMMEDIATE")
                if msg.payload.get("conflicting_peer_id") == self.drone_id:
                    self.get_logger().warn(f"[{self.drone_id}] Emergency warning from {sender_id}: {action}")
                    if action == "HOVER_IMMEDIATE" and self.agent.state == FlightState.WAYPOINT_NAVIGATION:
                        self.agent.mission_goal_local = list(self.agent.px4.current_position)
                        self.agent.state = FlightState.HOLD
                        self.agent.state_timer = 0

            elif msg_type == MessageType.VICTIM_CONFIRMED:
                self.event_journal.apply_remote_victim_payload(sender_id, msg.payload)
                self.scheduler.trigger_ack(
                    acked_seq=msg.header.sequence_number,
                    acked_msg_type=msg_type,
                    acked_session_id=msg.header.session_id
                )

            elif msg_type == MessageType.TASK_CLAIM:
                t_id = msg.payload.get("task_id", "")
                epoch = msg.payload.get("ownership_epoch", 1)
                self.event_journal.apply_remote_task_state(sender_id, {
                    "task_id": t_id,
                    "victim_event_id": msg.payload.get("victim_event_id", ""),
                    "owner_drone_id": sender_id,
                    "task_state": "CLAIMED",
                    "ownership_epoch": epoch,
                    "lease_duration_sec": msg.payload.get("lease_duration_sec", 30.0)
                })
                self.scheduler.trigger_ack(
                    acked_seq=msg.header.sequence_number,
                    acked_msg_type=msg_type,
                    acked_session_id=msg.header.session_id
                )

            elif msg_type == MessageType.TASK_STATE:
                self.event_journal.apply_remote_task_state(sender_id, msg.payload)

            elif msg_type == MessageType.JOURNAL_SYNC_REQ:
                summary = self.event_journal.export_state_summary()
                self.scheduler.trigger_journal_sync_resp(
                    victims_delta=list(summary["victims"].values()),
                    tasks_delta=list(summary["tasks"].values())
                )

            elif msg_type == MessageType.JOURNAL_SYNC_RESP:
                for v in msg.payload.get("victims_delta", []):
                    self.event_journal.apply_remote_victim_payload(sender_id, v)
                for t in msg.payload.get("tasks_delta", []):
                    self.event_journal.apply_remote_task_state(sender_id, t)

            elif msg_type == MessageType.INTENT_TRAJECTORY:
                tgt = msg.payload.get("target_grid")
                if tgt and sender_id in self.state_manager.peers:
                    self.state_manager.peers[sender_id].target_grid_cell = list(tgt)

    def _control_loop_tick(self):
        """20 Hz periodic control loop."""
        # 1. Update PX4 flight state
        if hasattr(self.agent, 'tick'):
            self.agent.tick()

        # 2. Update health FSM based on silence age
        self.state_manager.tick_health_fsm()

        # 3. Process incoming peer messages
        self._process_inbound_messages()

        # 4. Schedule periodic 1 Hz heartbeat
        curr_wx, curr_wy, curr_wz = self._get_current_world_enu()
        pv = self.agent.px4.current_velocity
        curr_vel_enu = [float(pv[1]), float(pv[0]), -float(pv[2])] if self.agent.px4.is_connected() else [0.0, 0.0, 0.0]

        is_armed = self.agent.px4.is_armed()
        is_offboard = self.agent.px4.is_offboard()
        stability_verified = (self.agent.state == FlightState.HOLD and self.agent.stable_hold_ticks >= self.agent.required_stable_ticks)

        curr_state = self.agent.state.name
        force_hb = (curr_state != getattr(self, '_prev_flight_state', None))
        self._prev_flight_state = curr_state

        # Synchronize local sequence and gather peer sequence acknowledgments
        self.state_manager.set_local_sequence(self.scheduler._sequence_counter)
        last_seen_seqs = {
            p_id: p_rec.last_seen_sequence
            for p_id, p_rec in self.state_manager.peers.items()
        }

        self.scheduler.schedule_heartbeat(
            flight_state=curr_state,
            position_enu=[curr_wx, curr_wy, curr_wz],
            velocity_enu=curr_vel_enu,
            battery_pct=round(self.battery_pct, 1),
            cells_explored=len(self.explored_cells),
            comms_health=self.state_manager.overall_comms_state.name,
            force=force_hb,
            is_armed=is_armed,
            is_offboard=is_offboard,
            stability_verified=stability_verified,
            last_seen_peer_sequences=last_seen_seqs
        )

        # 5. Flush outgoing scheduler queue
        self.scheduler.flush_outbox(self.transport)

        # 6. Autonomous decision making (strictly gated by telemetry_only and takeoff_hold_only)
        if self.telemetry_only:
            self._run_passive_safety_diagnostics(curr_wx, curr_wy, curr_wz)
            return

        # Active flight periodic diagnostic logging (1 Hz)
        px4 = self.agent.px4
        now_wall = time.time()
        if (now_wall - self.last_flight_log_wall_time) >= 1.0:
            self.last_flight_log_wall_time = now_wall
            arm_str = f"{PX4Interface.get_arming_state_name(px4.arming_state)}({px4.arming_state})"
            nav_str = f"{PX4Interface.get_nav_state_name(px4.nav_state)}({px4.nav_state})"
            telem_age = (now_wall - px4.last_odometry_wall_time) if px4.odometry_count > 0 else float('inf')

            peer_parts = []
            for p_id in self.peer_ids:
                if p_id in self.state_manager.peers:
                    peer = self.state_manager.peers[p_id]
                    peer_wx, peer_wy, peer_wz = peer.last_position_enu
                    sep = math.hypot(curr_wx - peer_wx, curr_wy - peer_wy, curr_wz - peer_wz)
                    peer_parts.append(f"{p_id}: state={peer.flight_state}, sep={sep:.2f}m, health={peer.comms_health.name}")
            peer_str = "; ".join(peer_parts) or "No peers"

            self.get_logger().info(
                f"[{self.drone_id}] State={self.agent.state.name} NavState={nav_str} Arming={arm_str} "
                f"Pos=({curr_wx:.2f}, {curr_wy:.2f}, {curr_wz:.2f}) AltErr={self.agent.current_alt_error:.2f}m "
                f"DriftXY={self.agent.current_drift_xy:.2f}m StableTicks={self.agent.stable_hold_ticks}/{self.agent.required_stable_ticks} "
                f"SimTime={px4.current_sim_time:.1f}s TelemAge={telem_age:.2f}s | {peer_str}"
            )

        # Section 5b: Simulated Onboard Visual Perception
        if self.candidate_world_victims and self.agent.state in (FlightState.HOLD, FlightState.WAYPOINT_NAVIGATION):
            heading_fn = getattr(px4, 'is_heading_valid', None)
            is_valid_heading = heading_fn() if callable(heading_fn) else (getattr(px4, 'heading_valid', False))
            
            if not is_valid_heading:
                # Critical Task B: Do not fabricate zero yaw when heading is invalid or stale
                inv_reason = getattr(px4, 'heading_invalid_reason', 'TELEMETRY_UNAVAILABLE')
                self.get_logger().warn(
                    f"[{self.drone_id}] PERCEPTION: Bypassing visual scan - vehicle heading unavailable/stale: {inv_reason}",
                    throttle_duration_sec=5.0
                )
            else:
                curr_yaw = float(getattr(px4, 'current_yaw_enu', 0.0))
                detections = self.victim_sensor.scan_environment(
                    drone_world_enu=[curr_wx, curr_wy, curr_wz],
                    drone_yaw_rad=curr_yaw,
                    candidate_world_victims=self.candidate_world_victims
                )
                now_tick_utc = time.time()
                self.tracklet_tracker.prune_stale_tracklets(now_tick_utc)
                for det in detections:
                    t_id, is_fresh_epoch, f_conf, f_unc, f_coords = self.tracklet_tracker.process_detection(
                        raw_target_id=det.victim_event_id,
                        measured_coords=det.estimated_world_enu,
                        uncertainty_m=det.position_uncertainty_m,
                        confidence=det.confidence,
                        timestamp_utc=now_tick_utc
                    )
                    v_id, is_new = self.event_journal.record_local_victim(
                        world_coords=f_coords,
                        grid_cell=det.estimated_grid_cell,
                        uncertainty_m=f_unc,
                        confidence=f_conf,
                        victim_event_id=det.victim_event_id,
                        observation_id=t_id
                    )
                    if is_new or is_fresh_epoch:
                        self.get_logger().info(
                            f"[{self.drone_id}] PERCEPTION: Detected victim {v_id} (tracklet={t_id}) at {f_coords} "
                            f"(Grid={det.estimated_grid_cell}, Conf={f_conf:.2f}, Unc={f_unc:.2f}m)"
                        )
                        self.scheduler.trigger_victim_confirmed(
                            victim_event_id=v_id,
                            world_coords=f_coords,
                            grid_cell=det.estimated_grid_cell,
                            uncertainty_m=f_unc,
                            confidence=f_conf,
                            observation_id=t_id
                        )
                        task_id = f"task_{v_id}"
                        if self.enable_task_claims and not self.active_task_id and self.event_journal.claim_task(task_id):
                            self.active_task_id = task_id
                            self.scheduler.trigger_task_claim(
                                task_id=task_id,
                                victim_event_id=v_id,
                                epoch=self.event_journal.tasks[task_id].ownership_epoch
                            )

        if self.takeoff_hold_only or not self.sar_mission_authorized or self.agent.state in (
            FlightState.RETURN_TO_BASE, FlightState.HOME_APPROACH, FlightState.LANDING_AUTHORIZED,
            FlightState.LAND, FlightState.DISARM, FlightState.COMPLETE
        ):
            # Inhibit autonomous SAR exploration, maintain stable position hold or execute RTB/landing
            return

        # Section 12: Communication-loss behavior during live exploration (Phase 4 & Phase 5)
        # - Standalone mode (no peers): proceed normally
        # - Loopback mode: if not connected, activate isolated fallback
        # - Live ROS mode with peers:
        #   - CONNECTED: normal decentralized policy operation
        #   - DEGRADED, UNKNOWN, ONE_WAY_FAILURE, RECONNECTING: pause new QMIX decisions and hold safe
        #   - ISOLATED: if enable_phase5_fallback, allow sector-bounded fallback exploration; otherwise hold safe
        if self.peer_ids:
            if not self.use_loopback_transport:
                comms_state = self.state_manager.overall_comms_state
                if comms_state != CommsHealthState.CONNECTED:
                    if comms_state == CommsHealthState.ISOLATED and self.enable_phase5_fallback:
                        self.isolated_fallback_active = True
                    else:
                        self.isolated_fallback_active = False
                        if comms_state == CommsHealthState.RECONNECTING:
                            if not getattr(self, '_reconnecting_sync_sent', False):
                                self._reconnecting_sync_sent = True
                                self.scheduler.trigger_journal_sync_req(
                                    known_victim_ids=list(self.event_journal.victims.keys()),
                                    known_task_epochs={t_id: t.ownership_epoch for t_id, t in self.event_journal.tasks.items()}
                                )
                        # Pause decision making and maintain safe position hold
                        return
                else:
                    self.isolated_fallback_active = False
                    self._reconnecting_sync_sent = False
            else:
                comms_state = self.state_manager.overall_comms_state
                if comms_state != CommsHealthState.CONNECTED:
                    self.isolated_fallback_active = True
                else:
                    self.isolated_fallback_active = False
        else:
            self.isolated_fallback_active = False

        if self.agent.state == FlightState.HOLD:
            if self.decisions_made >= self.max_decisions:
                if not self.mission_completed_logged:
                    self.mission_completed_logged = True
                    self.get_logger().info(
                        f"[{self.drone_id}] Max decisions ({self.max_decisions}) reached. "
                        f"Maintaining safe HOLD setpoint for mission completion."
                    )
                return

            # Section 11: Waypoint navigation arrival and dwell requirement
            # Wait until drone is settled stably in HOLD and residual velocity is low before next decision
            if not self.use_loopback_transport and not self.mock_px4:
                speed_xy = math.hypot(self.agent.px4.current_velocity[0], self.agent.px4.current_velocity[1])
                if self.agent.state_timer < 10 or speed_xy > 0.35:
                    return

            self._execute_autonomous_step()

    def _run_passive_safety_diagnostics(self, curr_wx: float, curr_wy: float, curr_wz: float):
        """Runs passive safety-envelope calculations and diagnostics without commanding flight actuation."""
        now = time.monotonic()
        px4 = self.agent.px4
        telem_age = (time.time() - px4.last_odometry_wall_time) if px4.odometry_count > 0 else float('inf')

        # Run passive safety checks against tracked peers
        for peer_id in self.peer_ids:
            if peer_id in self.state_manager.peers:
                peer = self.state_manager.peers[peer_id]
                center, r_xy, r_z, silence_age, is_unknown = self.safety_envelope.compute_peer_reachable_ellipsoid(peer_id, now)

                # Compute World ENU Euclidean separation
                peer_wx, peer_wy, peer_wz = peer.last_position_enu
                dx = curr_wx - peer_wx
                dy = curr_wy - peer_wy
                dz = curr_wz - peer_wz
                separation_m = math.sqrt(dx * dx + dy * dy + dz * dz)

                # Passive trajectory screening for stationary hover
                curr_gx, curr_gy = self._get_current_grid_cell()
                verdict_res = self.safety_envelope.screen_trajectory_3d(
                    current_grid=[curr_gx, curr_gy],
                    candidate_grid=[curr_gx, curr_gy],
                    current_world_enu=[curr_wx, curr_wy, curr_wz],
                    candidate_world_enu=[curr_wx, curr_wy, curr_wz],
                    candidate_action=4,  # HOVER
                    grid_obstacles=self.static_obstacles,
                    grid_size=self.grid_x_size,
                    current_time=now
                )

                self.passive_safety_diagnostics[peer_id] = {
                    "local_world_enu": [curr_wx, curr_wy, curr_wz],
                    "peer_world_enu": [peer_wx, peer_wy, peer_wz],
                    "separation_m": separation_m,
                    "peer_silence_age_sec": silence_age,
                    "reachable_radius_xy_m": r_xy,
                    "reachable_radius_z_m": r_z,
                    "is_unknown": is_unknown,
                    "safety_verdict": verdict_res.verdict.name,
                    "comms_health": peer.comms_health.name
                }

        # Periodic logging once per second
        if (time.time() - self.last_passive_log_wall_time) >= 1.0:
            self.last_passive_log_wall_time = time.time()
            arm_str = f"{PX4Interface.get_arming_state_name(px4.arming_state)}({px4.arming_state})"
            nav_str = f"{PX4Interface.get_nav_state_name(px4.nav_state)}({px4.nav_state})"
            peer_summary = "; ".join(
                f"{p}: sep={d['separation_m']:.2f}m, health={d['comms_health']}, verdict={d['safety_verdict']}"
                for p, d in self.passive_safety_diagnostics.items()
            ) or "No peers"

            self.get_logger().info(
                f"[{self.drone_id}] [PX4: {self.drone_config.namespace}] "
                f"Pos=({curr_wx:.2f}, {curr_wy:.2f}, {curr_wz:.2f}) "
                f"Vel=({px4.current_velocity[0]:.2f}, {px4.current_velocity[1]:.2f}, {px4.current_velocity[2]:.2f}) "
                f"Arming={arm_str} NavState={nav_str} SimTime={px4.current_sim_time:.1f}s "
                f"TelemAge={telem_age:.2f}s | {peer_summary}"
            )

    def _execute_autonomous_step(self):
        """Executes a single autonomous SAR decision step without waiting for central coordination."""
        curr_gx, curr_gy = self._get_current_grid_cell()

        # Mark current cell and local FOV as explored
        for i in range(-1, 2):
            for j in range(-1, 2):
                cx, cy = curr_gx + i, curr_gy + j
                if 0 <= cx < self.grid_x_size and 0 <= cy < self.grid_y_size:
                    if self.local_grid[cx, cy] != -1:
                        self.local_grid[cx, cy] = 1
                        self.explored_cells.add((cx, cy))

        # Check and expire overdue task leases
        self.event_journal.check_and_expire_leases()

        # Check for unassigned tasks to claim
        if self.enable_task_claims:
            unassigned_tasks = self.event_journal.get_unassigned_tasks()
            if not self.active_task_id and unassigned_tasks:
                t = unassigned_tasks[0]
                if self.event_journal.claim_task(t.task_id):
                    self.active_task_id = t.task_id
                    self.scheduler.trigger_task_claim(
                        task_id=t.task_id,
                        victim_event_id=t.victim_event_id,
                        epoch=self.event_journal.tasks[t.task_id].ownership_epoch
                    )
                    self.get_logger().info(f"[{self.drone_id}] Claimed SAR task: {t.task_id}")

        # Build observation and select action based on policy type
        if self.policy_type == "lawnmower":
            raw_action = self._get_lawnmower_action(curr_gx, curr_gy)
        elif self.policy_type == "random":
            raw_action = random.randint(0, 4)
        else:
            # Build 29-dim observation and select action
            obs = self._build_local_observation(curr_gx, curr_gy)
            raw_action = self.qmix.select_action(agent_id=self.drone_index, obs_np=obs)
        action_names = ["+X", "-X", "+Y", "-Y", "Hover"]

        # Compute tentative target grid cell
        tgt_gx, tgt_gy = curr_gx, curr_gy
        if raw_action == 0: tgt_gx += 1
        elif raw_action == 1: tgt_gx -= 1
        elif raw_action == 2: tgt_gy += 1
        elif raw_action == 3: tgt_gy -= 1

        # Check safety with 3D Swept-Trajectory Safety Envelope
        curr_wx, curr_wy, curr_wz = self._get_current_world_enu()
        target_wx, target_wy = GridWorldTransform.grid_to_world_center(tgt_gx, tgt_gy)

        # Battery tracking and dynamic RTB evaluation
        if self.last_step_world_pos is not None:
            step_dist = math.hypot(curr_wx - self.last_step_world_pos[0], curr_wy - self.last_step_world_pos[1])
            consumed = step_dist * self.battery_drain_rate_per_m + 0.15
            self.battery_pct = max(0.0, self.battery_pct - consumed)
        self.last_step_world_pos = (curr_wx, curr_wy, curr_wz)

        dist_to_home = math.hypot(curr_wx - self.drone_config.world_spawn_x, curr_wy - self.drone_config.world_spawn_y)
        rtb_battery_required = dist_to_home * self.battery_drain_rate_per_m + self.battery_safety_margin_pct

        if self.enable_battery_rtb and self.battery_pct <= rtb_battery_required:
            self.get_logger().warn(
                f"[{self.drone_id}] Dynamic Battery RTB triggered: battery={self.battery_pct:.1f}% <= "
                f"required={rtb_battery_required:.1f}% (dist_home={dist_to_home:.1f}m). Aborting exploration."
            )
            if self.active_task_id:
                curr_task = self.event_journal.tasks.get(self.active_task_id)
                v_id = curr_task.victim_event_id if curr_task else ""
                epoch = curr_task.ownership_epoch if curr_task else 1
                self.event_journal.release_task(self.active_task_id)
                self.scheduler.trigger_task_state(
                    task_id=self.active_task_id,
                    victim_event_id=v_id,
                    task_state="UNASSIGNED",
                    ownership_epoch=epoch + 1
                )
                self.active_task_id = None
                self.scheduler.flush_outbox(self.transport)

            self.agent.trigger_return_to_base()
            return

        # Sector Fallback bounds check when isolated
        if self.isolated_fallback_active and self.enable_phase5_fallback:
            peer_departed = any(
                p.flight_state in ("RETURN_TO_BASE", "HOME_APPROACH", "LANDING_AUTHORIZED", "LAND", "DISARM", "COMPLETE")
                for p in self.state_manager.peers.values()
            )
            if self.enable_adaptive_reallocation and peer_departed:
                min_gx = 0
                max_gx = self.grid_x_size - 1
                is_out_of_bounds = (tgt_gx < min_gx or tgt_gx > max_gx)
                self.get_logger().info(
                    f"[{self.drone_id}] Adaptive search reallocation active: peer departed, expanding sector to [{min_gx}, {max_gx}]."
                )
            else:
                num_drones = len(self.peer_ids) + 1
                if num_drones <= 2:
                    min_gx = 0 if self.drone_index == 0 else 7
                    max_gx = 5 if self.drone_index == 0 else (self.grid_x_size - 1)
                    is_out_of_bounds = (tgt_gx < min_gx or tgt_gx > max_gx or tgt_gx == 6)
                else:
                    sec_width = self.grid_x_size // num_drones
                    min_gx = self.drone_index * sec_width
                    if self.drone_index == num_drones - 1:
                        max_gx = self.grid_x_size - 1
                    else:
                        max_gx = min(self.grid_x_size - 1, (self.drone_index + 1) * sec_width - 1)
                    is_out_of_bounds = (tgt_gx < min_gx or tgt_gx > max_gx)

            if is_out_of_bounds:
                self.get_logger().warn(
                    f"[{self.drone_id}] Sector fallback active: tentative target gx={tgt_gx} outside "
                    f"assigned sector [{min_gx}, {max_gx}]. Overriding to HOVER."
                )
                raw_action = 4
                tgt_gx, tgt_gy = curr_gx, curr_gy
                target_wx, target_wy = curr_wx, curr_wy

        if self.enable_safety_envelope:
            safety_check = self.safety_envelope.screen_trajectory_3d(
                current_grid=[curr_gx, curr_gy],
                candidate_grid=[tgt_gx, tgt_gy],
                current_world_enu=[curr_wx, curr_wy, curr_wz],
                candidate_world_enu=[target_wx, target_wy, curr_wz],
                candidate_action=raw_action,
                grid_obstacles=self.static_obstacles,
                grid_size=self.grid_x_size
            )

            safe_action = safety_check.override_action
            safe_gx, safe_gy = safety_check.target_grid
            is_override = (safety_check.verdict != SafetyVerdict.SAFE)

            if is_override:
                self.safety_overrides_count += 1
                self.get_logger().warn(
                    f"[{self.drone_id}] Safety envelope override: {action_names[raw_action]} -> "
                    f"{action_names[safe_action]} ({safety_check.reason})"
                )

                # If lateral escape required, use escape waypoint; otherwise update world target to safe grid cell
                if safety_check.verdict == SafetyVerdict.CONFLICT_RESOLVED_LATERAL_ESCAPE and safety_check.evasion_vector:
                    target_wx = safety_check.target_pos[0]
                    target_wy = safety_check.target_pos[1]
                    self.get_logger().warn(f"[{self.drone_id}] Executing ACTIVE LATERAL ESCAPE: evasion_vec={safety_check.evasion_vector}")
                else:
                    target_wx, target_wy = GridWorldTransform.grid_to_world_center(safe_gx, safe_gy)

                if safety_check.verdict == SafetyVerdict.CONFLICT_RESOLVED_LATERAL_ESCAPE and safety_check.conflicting_peer_id is not None:
                    self.scheduler.trigger_emergency_safety(
                        emergency_code="TRAJECTORY_INTERSECTION_PREDICTED",
                        conflicting_peer_id=safety_check.conflicting_peer_id,
                        collision_time_sec=1.5,
                        current_pos=[curr_wx, curr_wy, curr_wz],
                        emergency_action="HOVER_IMMEDIATE",
                        evasion_vector=safety_check.evasion_vector
                    )
            else:
                target_wx, target_wy = GridWorldTransform.grid_to_world_center(safe_gx, safe_gy)
        else:
            # Safety envelope screening disabled (ablation mode)
            is_override = False
            safe_action = raw_action
            safe_gx = max(0, min(self.grid_x_size - 1, tgt_gx))
            safe_gy = max(0, min(self.grid_y_size - 1, tgt_gy))
            target_wx, target_wy = GridWorldTransform.grid_to_world_center(safe_gx, safe_gy)
            from .peer_safety_envelope import SafetyCheckResult
            safety_check = SafetyCheckResult(
                verdict=SafetyVerdict.SAFE,
                override_action=safe_action,
                target_grid=[safe_gx, safe_gy],
                target_pos=[target_wx, target_wy, curr_wz],
                reason="Safety envelope screening bypassed (ablation mode)"
            )

        # Broadcast trajectory intent
        self.scheduler.trigger_trajectory_intent(
            current_grid=[curr_gx, curr_gy],
            target_grid=[safe_gx, safe_gy],
            start_world_enu=[curr_wx, curr_wy, curr_wz],
            target_world_enu=[target_wx, target_wy, curr_wz],
            velocity_vector=[0.0, 0.0, 0.0],
            macro_action=action_names[safe_action]
        )
        self.scheduler.flush_outbox(self.transport)

        # Dispatch setpoint to PX4 flight agent
        local_north, local_east = GridWorldTransform.world_enu_to_px4_ned(
            target_wx, target_wy, self.drone_config.world_spawn_x, self.drone_config.world_spawn_y
        )
        lz = self.agent.initial_pos[2] - self.agent.mission.takeoff_altitude if self.agent.initial_pos else -self.agent.mission.takeoff_altitude

        self.agent.set_mission_goal_local(local_north, local_east, lz, require_min_dwell=(safe_action == 4))
        self.agent.state_timer = 0
        self.agent.state = FlightState.WAYPOINT_NAVIGATION
        self.decisions_made += 1
        self.last_target_grid = (safe_gx, safe_gy)
        self.unique_explored_cells.add((safe_gx, safe_gy))

        # Record decision for mission observability
        decision_record = {
            "decision_index": self.decisions_made,
            "timestamp_wall": time.time(),
            "timestamp_sim": self.agent.px4.current_sim_time,
            "raw_action": raw_action,
            "raw_action_name": action_names[raw_action],
            "safe_action": safe_action,
            "safe_action_name": action_names[safe_action],
            "start_grid": [curr_gx, curr_gy],
            "tentative_grid": [tgt_gx, tgt_gy],
            "dispatched_grid": [safe_gx, safe_gy],
            "start_world_enu": [curr_wx, curr_wy, curr_wz],
            "dispatched_world_enu": [target_wx, target_wy, curr_wz],
            "dispatched_local_ned": [local_north, local_east, lz],
            "safety_verdict": safety_check.verdict.name,
            "safety_reason": safety_check.reason,
            "is_override": is_override,
            "conflicting_peer": safety_check.conflicting_peer_id,
            "unique_cells_count": len(self.explored_cells)
        }
        self.decision_records.append(decision_record)

        self.get_logger().info(
            f"[{self.drone_id}] Step {self.decisions_made}/{self.max_decisions}: Action {action_names[safe_action]} "
            f"Target Grid ({safe_gx},{safe_gy}) -> World ({target_wx:.1f},{target_wy:.1f})"
        )

    def destroy_node(self):
        """Cancels periodic control timer, destroys services, and cleanly releases ROS 2 node resources."""
        if hasattr(self, 'auth_srv') and self.auth_srv is not None:
            self.destroy_service(self.auth_srv)
            self.auth_srv = None
        if hasattr(self, 'sar_auth_srv') and self.sar_auth_srv is not None:
            self.destroy_service(self.sar_auth_srv)
            self.sar_auth_srv = None
        if hasattr(self, 'rtb_srv') and self.rtb_srv is not None:
            self.destroy_service(self.rtb_srv)
            self.rtb_srv = None
        if hasattr(self, 'landing_auth_srv') and self.landing_auth_srv is not None:
            self.destroy_service(self.landing_auth_srv)
            self.landing_auth_srv = None
        if hasattr(self, 'timer') and self.timer is not None:
            self.destroy_timer(self.timer)
            self.timer = None
        return super().destroy_node()

def main(args=None):
    import argparse
    import signal

    # Convert SIGTERM into KeyboardInterrupt for clean node destruction
    def _sigterm_handler(signum, frame):
        raise KeyboardInterrupt

    try:
        signal.signal(signal.SIGTERM, _sigterm_handler)
    except Exception:
        pass

    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description="Decentralized P2P Drone Autonomous Node")
    parser.add_argument("--drone-id", type=str, default="drone_0", help="Drone unique ID")
    parser.add_argument("--drone-idx", type=int, default=0, help="Agent index in swarm")
    parser.add_argument("--system-id", type=int, default=1, help="PX4 MAVLink system ID")
    parser.add_argument("--spawn-x", type=float, default=12.0, help="Initial spawn X in ENU")
    parser.add_argument("--spawn-y", type=float, default=12.0, help="Initial spawn Y in ENU")
    parser.add_argument("--spawn-z", type=float, default=0.2, help="Initial spawn Z in ENU")
    parser.add_argument("--peer-ids", type=str, default="drone_1", help="Comma-separated peer drone IDs")
    parser.add_argument("--max-decisions", type=int, default=30, help="Max SAR exploration steps")
    parser.add_argument("--checkpoint", type=str, default=None, help="Path to QMIX .pth checkpoint")
    parser.add_argument("--meta-file", type=str, default=None, help="Path to world metadata JSON")
    parser.add_argument("--packet-loss", type=float, default=0.0, help="Simulated packet loss probability [0.0 - 1.0]")
    parser.add_argument("--latency-ms", type=float, default=0.0, help="Simulated network latency in milliseconds")
    parser.add_argument("--mock-px4", action="store_true", default=False, help="Enable mock PX4 state for headless/dry-run verification")
    parser.add_argument("--telemetry-only", action="store_true", default=False, help="Enable telemetry-only passive mode (strict no-flight-command interlock)")
    parser.add_argument("--takeoff-hold-only", action="store_true", default=False, help="Inhibit autonomous SAR exploration after takeoff and maintain stable position hold")
    parser.add_argument("--enable-phase5-fallback", action="store_true", default=False, help="Enable sector-bounded deterministic fallback exploration when isolated")
    parser.add_argument("--enable-battery-rtb", action="store_true", default=False, help="Enable dynamic energy-aware RTB thresholding")
    parser.add_argument("--enable-adaptive-reallocation", action="store_true", default=False, help="Enable dynamic sector expansion upon peer departure")
    parser.add_argument("--initial-battery", type=float, default=100.0, help="Initial battery percentage [0.0 - 100.0]")
    parser.add_argument("--fault-scenario", type=str, default=None, help="Phase 5 fault scenario name")
    parser.add_argument("--fault-seed", type=int, default=None, help="Deterministic seed for fault scenario")
    parser.add_argument("--sensor-seed", type=int, default=None, help="Deterministic seed for victim perception sensor simulation")
    parser.add_argument("--victims-file", type=str, default=None, help="Path to ground truth victims file for sensor simulation")

    parsed_args, unknown = parser.parse_known_args()

    project_root = os.path.expanduser('~/capstone_project_antigravity')
    checkpoint = parsed_args.checkpoint or os.path.join(project_root, 'models', 'qmix_sar_v4_align_best.pth')
    meta = parsed_args.meta_file or os.path.join(project_root, 'worlds', 'earthquake_world_meta.json')
    peers = [p.strip() for p in parsed_args.peer_ids.split(",") if p.strip()]

    victims_list = []
    if parsed_args.victims_file and os.path.exists(parsed_args.victims_file):
        try:
            with open(parsed_args.victims_file, 'r') as vf:
                vdata = json.load(vf)
                victims_list = vdata.get("victims", [])
        except Exception:
            pass
    elif meta and os.path.exists(meta):
        try:
            with open(meta, 'r') as mf:
                mdata = json.load(mf)
                victims_list = mdata.get("victims", [])
        except Exception:
            pass

    drone_cfg = DroneConfig(
        drone_id=parsed_args.drone_id,
        namespace=parsed_args.drone_id,
        system_id=parsed_args.system_id,
        world_spawn_x=parsed_args.spawn_x,
        world_spawn_y=parsed_args.spawn_y,
        world_spawn_z=parsed_args.spawn_z,
        world_yaw=0.0
    )
    mission_cfg = MissionConfig(execution_mode="peer_to_peer")

    impairment_cfg = NetworkImpairmentConfig(
        packet_loss_rate=parsed_args.packet_loss,
        mean_latency_ms=parsed_args.latency_ms
    )

    node = DecentralizedDroneNode(
        drone_id=parsed_args.drone_id,
        drone_index=parsed_args.drone_idx,
        drone_config=drone_cfg,
        mission_config=mission_cfg,
        checkpoint_path=checkpoint,
        meta_file=meta,
        peer_ids=peers,
        max_decisions=parsed_args.max_decisions,
        impairment_config=impairment_cfg,
        mock_px4=parsed_args.mock_px4,
        telemetry_only=parsed_args.telemetry_only,
        takeoff_hold_only=parsed_args.takeoff_hold_only,
        enable_phase5_fallback=parsed_args.enable_phase5_fallback,
        enable_battery_rtb=parsed_args.enable_battery_rtb,
        enable_adaptive_reallocation=parsed_args.enable_adaptive_reallocation,
        initial_battery_pct=parsed_args.initial_battery,
        fault_scenario=parsed_args.fault_scenario,
        fault_seed=parsed_args.fault_seed,
        sensor_seed=parsed_args.sensor_seed,
        candidate_world_victims=victims_list
    )

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0)

if __name__ == '__main__':
    main()
