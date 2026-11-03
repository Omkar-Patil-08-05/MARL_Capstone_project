import os
import subprocess
from enum import Enum
import time
import random
import math
from typing import Optional, Dict, Tuple, Any

try:
    from .grid_world_transform import GridWorldTransform
except ImportError:
    try:
        from swarm_controller.grid_world_transform import GridWorldTransform
    except ImportError:
        from grid_world_transform import GridWorldTransform

try:
    from .ground_truth_matcher import GroundTruthMatcher, GroundTruthMatchResult
except ImportError:
    try:
        from swarm_controller.ground_truth_matcher import GroundTruthMatcher, GroundTruthMatchResult
    except ImportError:
        from ground_truth_matcher import GroundTruthMatcher, GroundTruthMatchResult

class VictimState(Enum):
    UNDETECTED = "UNDETECTED"
    DETECTED = "DETECTED"
    RESCUED = "RESCUED"

class MovementState(Enum):
    RESTING = "RESTING"
    MOVING = "MOVING"

class Victim:
    def __init__(self, v_id, grid_x, grid_y, world_x, world_y, profile_config, random_gen, world_z=0.1):
        self.id = v_id
        self.grid_x = grid_x
        self.grid_y = grid_y
        self.world_x = world_x
        self.world_y = world_y
        self.world_z = world_z
        
        self.state = VictimState.UNDETECTED
        self.movement_state = MovementState.RESTING
        
        # Detection metadata (populated by mark_detected)
        self.detected_by = None
        self.detection_distance = None
        self.detection_time = None
        
        self.profile = profile_config
        self.rng = random_gen
        
        self.speed = self.rng.uniform(self.profile['min_speed'], self.profile['max_speed'])
        self.rest_timer = self.rng.uniform(0.0, self.profile['max_rest_duration'])
        
        self.dest_world_x = self.world_x
        self.dest_world_y = self.world_y
        
        self.last_gz_update_time = time.time()
        
    def get_dict(self):
        return {
            "id": self.id,
            "x": self.grid_x,
            "y": self.grid_y,
            "world_x": self.world_x,
            "world_y": self.world_y,
            "world_z": self.world_z,
            "detected": self.state == VictimState.DETECTED or self.state == VictimState.RESCUED,
            "state": self.state.value,
            "movement_state": self.movement_state.value,
            "detected_by": self.detected_by,
            "detection_distance": round(self.detection_distance, 2) if self.detection_distance is not None else None,
            "detection_time": self.detection_time
        }

UNKNOWN_AUTONOMOUS_SOURCE = "UNKNOWN_AUTONOMOUS_SOURCE"
"""
Limitation Documentation:
The /sar/victim_discovery publisher payload emitted by TemporalEventConfirmer
specifies 'verification_mode' ('RGBD' or 'MULTISPECTRAL_RGB_THERMAL') but does NOT
contain an explicit publisher node or detector source identifier key ('source').
When 'source' is omitted or cannot be established from the payload, it is explicitly
assigned UNKNOWN_AUTONOMOUS_SOURCE rather than fabricating a specific detector provenance
(e.g., never defaulting to 'multispectral_verifier').
"""


def _validate_strict_int(val: Any, field_name: str, min_val: Optional[int] = None) -> int:
    """Validates that val is a strict integer (not bool, not fractional float) and >= min_val."""
    if isinstance(val, bool) or val is None:
        raise ValueError(f"Field '{field_name}' must be an integer, got {type(val).__name__} ({val})")
    if isinstance(val, int):
        parsed = val
    elif isinstance(val, float):
        if not math.isfinite(val) or not val.is_integer():
            raise ValueError(f"Field '{field_name}' cannot have fractional or non-finite value: {val}")
        parsed = int(val)
    elif isinstance(val, str):
        try:
            parsed = int(val)
        except ValueError:
            raise ValueError(f"Field '{field_name}' invalid integer string: {val}")
    else:
        raise ValueError(f"Field '{field_name}' unsupported type: {type(val).__name__}")
    
    if min_val is not None and parsed < min_val:
        raise ValueError(f"Field '{field_name}' must be >= {min_val}, got {parsed}")
    return parsed


def _validate_strict_float(val: Any, field_name: str, min_val: Optional[float] = None, max_val: Optional[float] = None) -> float:
    """Validates that val is a finite float (not bool) within [min_val, max_val]."""
    if isinstance(val, bool) or val is None:
        raise ValueError(f"Field '{field_name}' must be a float, got {type(val).__name__} ({val})")
    try:
        parsed = float(val)
    except (ValueError, TypeError):
        raise ValueError(f"Field '{field_name}' cannot be converted to float: {val}")
    if not math.isfinite(parsed):
        raise ValueError(f"Field '{field_name}' must be finite, got {parsed}")
    if min_val is not None and parsed < min_val:
        raise ValueError(f"Field '{field_name}' must be >= {min_val}, got {parsed}")
    if max_val is not None and parsed > max_val:
        raise ValueError(f"Field '{field_name}' must be <= {max_val}, got {parsed}")
    return parsed


def _validate_optional_int(val: Any, field_name: str, min_val: Optional[int] = None) -> Optional[int]:
    """Validates an optional integer. Preserves None (JSON null). Rejects bools, fractional floats, and values < min_val."""
    if val is None:
        return None
    return _validate_strict_int(val, field_name, min_val=min_val)


def _validate_optional_float(val: Any, field_name: str, min_val: Optional[float] = None, max_val: Optional[float] = None) -> Optional[float]:
    """Validates an optional float. Preserves None (JSON null). Rejects non-finite values and values outside [min_val, max_val]."""
    if val is None:
        return None
    return _validate_strict_float(val, field_name, min_val=min_val, max_val=max_val)


class TrackedVictim:
    def __init__(
        self,
        t_id: str,
        world_x: float,
        world_y: float,
        source: Optional[str] = None,
        confidence: float = 1.0,
        drone_id: int = 0,
        sim_time: float = 0.0,
        source_track_id: Optional[int] = None,
        verification_mode: Optional[str] = None,
        evidence_state: str = "TEMPORALLY_CONFIRMED",
        thermal_peak_k: Optional[float] = None,
        thermal_delta_k: Optional[float] = None,
        confirmation_count: Optional[int] = None,
        confirmation_window: Optional[int] = None,
        consensus_span_sec: Optional[float] = None,
        median_depth_estimate: Optional[float] = None,
        depth_valid_samples: Optional[int] = None,
        depth_spread: Optional[float] = None
    ):
        self.id = str(t_id)
        self.world_x = _validate_strict_float(world_x, "world_x")
        self.world_y = _validate_strict_float(world_y, "world_y")
        self.grid_x, self.grid_y = GridWorldTransform.world_to_grid(self.world_x, self.world_y)
        
        # Source validation: never default to multispectral_verifier
        if source is None or not str(source).strip():
            self.source = UNKNOWN_AUTONOMOUS_SOURCE
        else:
            self.source = str(source).strip()
            
        self.confidence = _validate_strict_float(confidence, "confidence", min_val=0.0, max_val=1.0)
        self.detected_by = _validate_strict_int(drone_id, "drone_id", min_val=0)
        self.sim_time = _validate_strict_float(sim_time, "sim_time", min_val=0.0)
        
        # Timing separation: wall-clock timestamps vs simulation clock timestamps
        now_wall = time.time()
        self.first_seen = now_wall       # Backward-compatible wall clock (seconds)
        self.last_seen = now_wall        # Backward-compatible wall clock (seconds)
        self.first_seen_sim = self.sim_time
        self.last_seen_sim = self.sim_time

        self.status = "DETECTED"
        self.gt_error = -1.0 # -1 means unknown or not calculated
        self.observations = 1

        # Traceability and metadata
        self.source_track_id = _validate_optional_int(source_track_id, "source_track_id", min_val=0)
        self.verification_mode = str(verification_mode).strip() if (verification_mode is not None and str(verification_mode).strip()) else None
        self.evidence_state = str(evidence_state) if evidence_state is not None else "TEMPORALLY_CONFIRMED"
        
        # Optional metadata fields (strict validation, finite checks, non-negative where required)
        self.thermal_peak_k = _validate_optional_float(thermal_peak_k, "thermal_peak_k", min_val=0.0)
        self.thermal_delta_k = _validate_optional_float(thermal_delta_k, "thermal_delta_k")
        self.confirmation_count = _validate_optional_int(confirmation_count, "confirmation_count", min_val=0)
        self.confirmation_window = _validate_optional_int(confirmation_window, "confirmation_window", min_val=0)
        self.consensus_span_sec = _validate_optional_float(consensus_span_sec, "consensus_span_sec", min_val=0.0)
        self.median_depth_estimate = _validate_optional_float(median_depth_estimate, "median_depth_estimate", min_val=0.0)
        self.depth_valid_samples = _validate_optional_int(depth_valid_samples, "depth_valid_samples", min_val=0)
        self.depth_spread = _validate_optional_float(depth_spread, "depth_spread", min_val=0.0)

    def update(
        self,
        world_x: float,
        world_y: float,
        source: Optional[str] = None,
        confidence: float = 1.0,
        drone_id: int = 0,
        sim_time: float = 0.0,
        source_track_id: Optional[int] = None,
        verification_mode: Optional[str] = None,
        evidence_state: Optional[str] = None,
        thermal_peak_k: Optional[float] = None,
        thermal_delta_k: Optional[float] = None,
        confirmation_count: Optional[int] = None,
        confirmation_window: Optional[int] = None,
        consensus_span_sec: Optional[float] = None,
        median_depth_estimate: Optional[float] = None,
        depth_valid_samples: Optional[int] = None,
        depth_spread: Optional[float] = None
    ):
        """
        Atomically updates the TrackedVictim with a new observation.

        Atomicity Guarantee:
          All incoming mandatory and optional fields are completely validated and prepared in local
          variables before ANY attribute of self is modified. If any validation fails (e.g. ValueError),
          an exception is raised and every attribute of self remains completely unchanged.

        Out-of-Order Simulation Timestamp Policy:
          - last_seen_sim and sim_time are strictly monotonic non-decreasing:
            An older event (sim_time < last_seen_sim) will NOT regress last_seen_sim or sim_time.
          - first_seen_sim records the earliest simulation sighting:
            If sim_time < first_seen_sim, first_seen_sim is updated to min(first_seen_sim, sim_time).
          - Wall-clock time (last_seen) records reception wall-clock time and is never mixed with sim_time.
          - Observation count (observations) increments by 1 for every valid confirmed sighting.
          - Position smoothing (world_x, world_y via EMA alpha=0.7) is applied for all valid spatial observations
            regardless of temporal order, as observations are spatial measurements of the stationary/quasi-stationary victim.
          - Confidence is monotonically maximized: confidence = max(self.confidence, new_confidence).
          - Active metadata (detected_by, source, verification_mode, evidence_state, etc.):
            Updated when sim_time >= last_seen_sim (in-order or equal timestamp).
            When sim_time < last_seen_sim (out-of-order event), existing non-None metadata is retained
            to prevent newer evidence from being overwritten by older observations, while any currently
            None metadata fields are backfilled from the older event.
          - Equal timestamps (sim_time == last_seen_sim) are handled deterministically under the
            sim_time >= last_seen_sim branch.
        """
        # Step 1: Strict validation of all mandatory fields into local variables
        new_wx = _validate_strict_float(world_x, "world_x")
        new_wy = _validate_strict_float(world_y, "world_y")
        new_conf = _validate_strict_float(confidence, "confidence", min_val=0.0, max_val=1.0)
        new_drone_id = _validate_strict_int(drone_id, "drone_id", min_val=0)
        st = _validate_strict_float(sim_time, "sim_time", min_val=0.0)

        # Step 2: Strict validation of all optional metadata fields into local variables
        new_source = str(source).strip() if (source is not None and str(source).strip()) else None
        new_source_track_id = _validate_optional_int(source_track_id, "source_track_id", min_val=0)
        new_verif_mode = str(verification_mode).strip() if (verification_mode is not None and str(verification_mode).strip()) else None
        new_evidence_state = str(evidence_state) if evidence_state is not None else None
        new_th_peak = _validate_optional_float(thermal_peak_k, "thermal_peak_k", min_val=0.0)
        new_th_delta = _validate_optional_float(thermal_delta_k, "thermal_delta_k")
        new_conf_count = _validate_optional_int(confirmation_count, "confirmation_count", min_val=0)
        new_conf_win = _validate_optional_int(confirmation_window, "confirmation_window", min_val=0)
        new_consensus_span = _validate_optional_float(consensus_span_sec, "consensus_span_sec", min_val=0.0)
        new_median_depth = _validate_optional_float(median_depth_estimate, "median_depth_estimate", min_val=0.0)
        new_depth_samples = _validate_optional_int(depth_valid_samples, "depth_valid_samples", min_val=0)
        new_depth_spread = _validate_optional_float(depth_spread, "depth_spread", min_val=0.0)

        # Step 3: Compute proposed new state in local variables
        alpha = 0.7
        smoothed_wx = self.world_x * (1.0 - alpha) + new_wx * alpha
        smoothed_wy = self.world_y * (1.0 - alpha) + new_wy * alpha
        smoothed_gx, smoothed_gy = GridWorldTransform.world_to_grid(smoothed_wx, smoothed_wy)

        updated_confidence = max(self.confidence, new_conf)
        updated_observations = self.observations + 1
        now_wall = time.time()

        # Timestamp policy
        updated_first_seen_sim = min(self.first_seen_sim, st)
        updated_last_seen_sim = max(self.last_seen_sim, st)
        updated_sim_time = max(self.sim_time, st)

        # Metadata policy based on simulation time order
        is_newer_or_equal = (st >= self.last_seen_sim)

        if is_newer_or_equal:
            target_detected_by = new_drone_id
            target_source = new_source if new_source is not None else self.source
            target_source_track_id = new_source_track_id if new_source_track_id is not None else self.source_track_id
            target_verif_mode = new_verif_mode if new_verif_mode is not None else self.verification_mode
            target_evidence_state = new_evidence_state if new_evidence_state is not None else self.evidence_state
            target_th_peak = new_th_peak if new_th_peak is not None else self.thermal_peak_k
            target_th_delta = new_th_delta if new_th_delta is not None else self.thermal_delta_k
            target_conf_count = new_conf_count if new_conf_count is not None else self.confirmation_count
            target_conf_win = new_conf_win if new_conf_win is not None else self.confirmation_window
            target_consensus_span = new_consensus_span if new_consensus_span is not None else self.consensus_span_sec
            target_median_depth = new_median_depth if new_median_depth is not None else self.median_depth_estimate
            target_depth_samples = new_depth_samples if new_depth_samples is not None else self.depth_valid_samples
            target_depth_spread = new_depth_spread if new_depth_spread is not None else self.depth_spread
        else:
            # Out-of-order event: Retain active metadata from the newer event;
            # only backfill fields that are currently None on self to prevent newer evidence regression.
            target_detected_by = self.detected_by
            target_source = self.source if self.source != UNKNOWN_AUTONOMOUS_SOURCE else (new_source if new_source is not None else self.source)
            target_source_track_id = self.source_track_id if self.source_track_id is not None else new_source_track_id
            target_verif_mode = self.verification_mode if self.verification_mode is not None else new_verif_mode
            target_evidence_state = self.evidence_state if self.evidence_state is not None else (new_evidence_state if new_evidence_state is not None else "TEMPORALLY_CONFIRMED")
            target_th_peak = self.thermal_peak_k if self.thermal_peak_k is not None else new_th_peak
            target_th_delta = self.thermal_delta_k if self.thermal_delta_k is not None else new_th_delta
            target_conf_count = self.confirmation_count if self.confirmation_count is not None else new_conf_count
            target_conf_win = self.confirmation_window if self.confirmation_window is not None else new_conf_win
            target_consensus_span = self.consensus_span_sec if self.consensus_span_sec is not None else new_consensus_span
            target_median_depth = self.median_depth_estimate if self.median_depth_estimate is not None else new_median_depth
            target_depth_samples = self.depth_valid_samples if self.depth_valid_samples is not None else new_depth_samples
            target_depth_spread = self.depth_spread if self.depth_spread is not None else new_depth_spread

        # Step 4: Atomic application of all validated and computed values
        self.world_x = smoothed_wx
        self.world_y = smoothed_wy
        self.grid_x = smoothed_gx
        self.grid_y = smoothed_gy
        self.confidence = updated_confidence
        self.observations = updated_observations
        self.last_seen = now_wall
        self.first_seen_sim = updated_first_seen_sim
        self.last_seen_sim = updated_last_seen_sim
        self.sim_time = updated_sim_time
        self.detected_by = target_detected_by
        self.source = target_source
        self.source_track_id = target_source_track_id
        self.verification_mode = target_verif_mode
        self.evidence_state = target_evidence_state
        self.thermal_peak_k = target_th_peak
        self.thermal_delta_k = target_th_delta
        self.confirmation_count = target_conf_count
        self.confirmation_window = target_conf_win
        self.consensus_span_sec = target_consensus_span
        self.median_depth_estimate = target_median_depth
        self.depth_valid_samples = target_depth_samples
        self.depth_spread = target_depth_spread
        self.status = "DETECTED"
        
    def check_status(self, current_time):
        # The user requested that we NEVER reduce the historical detected count.
        # Once detected, it remains DETECTED permanently.
        self.status = "DETECTED"

    def get_dict(self):
        return {
            # 100% backward-compatible fields
            "id": self.id,
            "x": self.grid_x,
            "y": self.grid_y,
            "world_x": round(self.world_x, 2),
            "world_y": round(self.world_y, 2),
            "detected": True,
            "state": self.status,
            "source": self.source,
            "confidence": round(self.confidence, 3),
            "detected_by": self.detected_by,
            "last_seen_sec_ago": round(time.time() - self.last_seen, 1),
            "gt_error": round(self.gt_error, 2) if self.gt_error >= 0 else -1.0,
            "observations": self.observations,
            # Autonomous multispectral metadata extensions
            "sim_time": round(self.sim_time, 3),
            "first_seen_sim": round(self.first_seen_sim, 3),
            "last_seen_sim": round(self.last_seen_sim, 3),
            "source_track_id": self.source_track_id,
            "verification_mode": self.verification_mode,
            "evidence_state": self.evidence_state,
            "thermal_peak_k": round(self.thermal_peak_k, 2) if self.thermal_peak_k is not None else None,
            "thermal_delta_k": round(self.thermal_delta_k, 2) if self.thermal_delta_k is not None else None,
            "confirmation_count": self.confirmation_count,
            "confirmation_window": self.confirmation_window,
            "consensus_span_sec": round(self.consensus_span_sec, 3) if self.consensus_span_sec is not None else None,
            "median_depth_estimate": round(self.median_depth_estimate, 2) if self.median_depth_estimate is not None else None,
            "depth_valid_samples": self.depth_valid_samples,
            "depth_spread": round(self.depth_spread, 2) if self.depth_spread is not None else None
        }

class VictimManager:
    def __init__(self, node, metadata, world_name, env, seed=42, autonomous_association_radius=None):
        self.node = node
        self.world_name = world_name
        self.env = env # SARGridEnv reference for obstacle checking
        self.victims = {} # Ground truth victims
        self.tracked_victims = {} # Camera-detected tracked victims
        self.next_track_id = 0
        
        # Existing generic association radius preserved without modification
        self.association_radius = 12.0 # Meters
        
        # Autonomous discovery association threshold (default 6.0m, positive & finite)
        if autonomous_association_radius is not None:
            raw_auto_rad = autonomous_association_radius
        else:
            raw_auto_rad = os.environ.get("AUTONOMOUS_ASSOCIATION_RADIUS", "6.0")
            
        try:
            parsed_rad = float(raw_auto_rad)
            if math.isfinite(parsed_rad) and parsed_rad > 0.0:
                self.autonomous_association_radius = parsed_rad
            else:
                if self.node is not None and hasattr(self.node, 'get_logger'):
                    self.node.get_logger().warn(
                        f"[VictimManager] Invalid AUTONOMOUS_ASSOCIATION_RADIUS '{raw_auto_rad}'. Must be finite positive float. Fallback to 6.0m."
                    )
                self.autonomous_association_radius = 6.0
        except (ValueError, TypeError):
            if self.node is not None and hasattr(self.node, 'get_logger'):
                self.node.get_logger().warn(
                    f"[VictimManager] Cannot parse AUTONOMOUS_ASSOCIATION_RADIUS '{raw_auto_rad}'. Fallback to 6.0m."
                )
            self.autonomous_association_radius = 6.0

        # Ambiguity margin to prevent merging when multiple distinct tracks are in close contention
        self.ambiguity_margin = 1.0 # Meters

        self.rng = random.Random(seed)
        
        # Victim Profiles
        self.profiles = {
            "stationary": {"movement_enabled": False, "min_speed": 0.0, "max_speed": 0.0, "min_rest_duration": 9999, "max_rest_duration": 9999, "radius": 0},
            "slow": {"movement_enabled": True, "min_speed": 0.2, "max_speed": 0.4, "min_rest_duration": 5.0, "max_rest_duration": 15.0, "radius": 12.0},
            "intermittent": {"movement_enabled": True, "min_speed": 0.4, "max_speed": 0.8, "min_rest_duration": 10.0, "max_rest_duration": 30.0, "radius": 20.0},
            "moderate": {"movement_enabled": True, "min_speed": 0.6, "max_speed": 1.2, "min_rest_duration": 3.0, "max_rest_duration": 10.0, "radius": 30.0},
            "wanderer": {"movement_enabled": True, "min_speed": 0.3, "max_speed": 0.7, "min_rest_duration": 2.0, "max_rest_duration": 8.0, "radius": 40.0},
        }
        
        self.parse_metadata(metadata)
        
        self.waypoint_tolerance = 0.5 # meters
        self.gz_update_interval = 1.0 # Throttle Gazebo service calls to 1 Hz to prevent performance drop
        self.last_update_time = None

    def _log_info(self, msg: str):
        if self.node is not None and hasattr(self.node, 'get_logger'):
            self.node.get_logger().info(msg)

    def _log_warn(self, msg: str):
        if self.node is not None and hasattr(self.node, 'get_logger'):
            self.node.get_logger().warn(msg)

    def _get_current_time(self) -> float:
        """Returns the current simulation time in seconds from ROS clock, or falls back to wall clock."""
        if self.node is not None:
            try:
                now_ns = self.node.get_clock().now().nanoseconds
                if now_ns > 0:
                    return now_ns * 1e-9
            except Exception:
                pass
        return time.time()

    def parse_metadata(self, metadata):
        victims_meta = metadata.get("victims", [])
        profile_names = list(self.profiles.keys())
        
        for idx, v_meta in enumerate(victims_meta):
            v_id = v_meta.get("id", f"V{idx}")
            grid_x = v_meta["grid"]["x"]
            grid_y = v_meta["grid"]["y"]
            world_x = v_meta["world"]["x"]
            world_y = v_meta["world"]["y"]
            world_z = v_meta["world"].get("z", 0.1)
            
            # Assign varied behavior profiles based on index for variety
            if idx == 0:
                p_name = "stationary"
            elif idx == 1:
                p_name = "slow"
            elif idx == 2:
                p_name = "intermittent"
            elif idx == 3:
                p_name = "moderate"
            else:
                p_name = "wanderer"
                
            self.victims[v_id] = Victim(v_id, grid_x, grid_y, world_x, world_y, self.profiles[p_name], self.rng, world_z=world_z)

    def spawn_all(self):
        """Spawns all victims into Gazebo using the rescue_randy_sitting model."""
        for v_id, victim in self.victims.items():
            self.node.get_logger().info(f"[VictimManager] Spawning {v_id} at world({victim.world_x:.1f}, {victim.world_y:.1f}, {victim.world_z:.1f}) with profile {victim.profile['movement_enabled']}")
            
            # Using the casual_female standing human model
            model_path = "/home/capstone/capstone_project_antigravity/assets_real/victims/casual_female/model.sdf"
            cmd = [
                'gz', 'service', '-s', f'/world/{self.world_name}/create',
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--req', f'sdf_filename: "{model_path}", name: "{v_id}", pose: {{position: {{x: {victim.world_x}, y: {victim.world_y}, z: {victim.world_z}}}}}'
            ]
            subprocess.Popen(cmd)
            time.sleep(0.1)

    @staticmethod
    def validate_event_payload(payload: Any) -> Tuple[bool, Optional[Dict[str, Any]], Optional[str]]:
        """
        Validates an incoming /sar/victim_discovery JSON payload against the publisher contract.
        
        Enforces mandatory publisher fields:
          - event_type: == 'VICTIM_DISCOVERED'
          - timestamp: finite float >= 0.0 (0.0 is explicitly valid)
          - victim_track_id: strict integer >= 0 (cannot be fractional float or bool)
          - confirming_drone_id: strict integer >= 0 (cannot be fractional float or bool)
          - world_x: finite float
          - world_y: finite float
          - confidence: finite float in [0.0, 1.0]
        
        Validates optional metadata:
          - Validates every optional numeric field with math.isfinite()
          - Preserves JSON null as Python None
          - Validates optional integer fields without silently truncating fractional values
          - Rejects negative values where field semantics require non-negative values
          - Sets source to UNKNOWN_AUTONOMOUS_SOURCE if absent (never defaults to multispectral_verifier)
          
        Returns:
          (is_valid: bool, parsed_dict: Optional[Dict], error_reason: Optional[str])
        """
        if not isinstance(payload, dict):
            return False, None, f"Payload must be a dict, got {type(payload).__name__}"
            
        # 1. Mandatory event_type
        if "event_type" not in payload:
            return False, None, "Missing mandatory field 'event_type'"
        if payload["event_type"] != "VICTIM_DISCOVERED":
            return False, None, f"Invalid event_type '{payload['event_type']}', expected 'VICTIM_DISCOVERED'"
            
        # 2. Mandatory field presence
        mandatory_fields = [
            "timestamp", "victim_track_id", "confirming_drone_id",
            "world_x", "world_y", "confidence"
        ]
        for f in mandatory_fields:
            if f not in payload:
                if f == "victim_track_id" and "source_track_id" in payload:
                    continue
                if f == "confirming_drone_id" and "drone_id" in payload:
                    continue
                return False, None, f"Missing mandatory field '{f}'"
                
        # 3. Mandatory value validation
        try:
            ts = _validate_strict_float(payload["timestamp"], "timestamp", min_val=0.0)
            
            raw_v_tid = payload.get("victim_track_id", payload.get("source_track_id"))
            v_tid = _validate_strict_int(raw_v_tid, "victim_track_id", min_val=0)
            
            raw_drone = payload.get("confirming_drone_id", payload.get("drone_id"))
            d_id = _validate_strict_int(raw_drone, "confirming_drone_id", min_val=0)
            
            wx = _validate_strict_float(payload["world_x"], "world_x")
            wy = _validate_strict_float(payload["world_y"], "world_y")
            conf = _validate_strict_float(payload["confidence"], "confidence", min_val=0.0, max_val=1.0)
        except (ValueError, TypeError) as e:
            return False, None, f"Mandatory field validation error: {e}"
            
        # 4. Detection source correctness
        # Never default missing source to "multispectral_verifier"
        raw_source = payload.get("source")
        if raw_source is not None and str(raw_source).strip():
            source = str(raw_source).strip()
        else:
            source = UNKNOWN_AUTONOMOUS_SOURCE
            
        raw_mode = payload.get("verification_mode")
        verification_mode = str(raw_mode).strip() if (raw_mode is not None and str(raw_mode).strip()) else None
        evidence_state = str(payload.get("evidence_state", "TEMPORALLY_CONFIRMED"))
        
        # 5. Optional metadata validation (preserve None, validate isfinite, reject fractional ints, reject negative)
        try:
            conf_count = _validate_optional_int(payload.get("confirmation_count"), "confirmation_count", min_val=0)
            conf_win = _validate_optional_int(payload.get("confirmation_window"), "confirmation_window", min_val=0)
            depth_samples = _validate_optional_int(payload.get("depth_valid_samples"), "depth_valid_samples", min_val=0)
            
            median_depth = _validate_optional_float(payload.get("median_depth_estimate"), "median_depth_estimate", min_val=0.0)
            consensus_span = _validate_optional_float(payload.get("consensus_span_sec"), "consensus_span_sec", min_val=0.0)
            depth_spread = _validate_optional_float(payload.get("depth_spread"), "depth_spread", min_val=0.0)
            th_peak = _validate_optional_float(payload.get("thermal_peak_k"), "thermal_peak_k", min_val=0.0)
            th_delta = _validate_optional_float(payload.get("thermal_delta_k"), "thermal_delta_k")
            
            gx = _validate_optional_int(payload.get("grid_x"), "grid_x", min_val=0)
            gy = _validate_optional_int(payload.get("grid_y"), "grid_y", min_val=0)
            cell_size = _validate_optional_float(payload.get("grid_cell_size_m"), "grid_cell_size_m", min_val=0.0)
            if cell_size is not None and cell_size <= 0.0:
                raise ValueError("Field 'grid_cell_size_m' must be strictly positive")
        except (ValueError, TypeError) as e:
            return False, None, f"Optional metadata validation error: {e}"
            
        parsed = {
            "event_type": "VICTIM_DISCOVERED",
            "sim_time": ts,
            "timestamp": ts,
            "source_track_id": v_tid,
            "victim_track_id": v_tid,
            "drone_id": d_id,
            "confirming_drone_id": d_id,
            "world_x": wx,
            "world_y": wy,
            "confidence": conf,
            "source": source,
            "verification_mode": verification_mode,
            "evidence_state": evidence_state,
            "confirmation_count": conf_count,
            "confirmation_window": conf_win,
            "depth_valid_samples": depth_samples,
            "median_depth_estimate": median_depth,
            "consensus_span_sec": consensus_span,
            "depth_spread": depth_spread,
            "thermal_peak_k": th_peak,
            "thermal_delta_k": th_delta,
            "grid_x": gx,
            "grid_y": gy,
            "grid_cell_size_m": cell_size
        }
        return True, parsed, None

    def process_discovery_event(
        self,
        *args,
        world_x: Optional[float] = None,
        world_y: Optional[float] = None,
        source: Optional[str] = None,
        confidence: Optional[float] = None,
        drone_id: Optional[int] = None,
        sim_time: Optional[float] = None,
        source_track_id: Optional[int] = None,
        verification_mode: Optional[str] = None,
        evidence_state: str = "TEMPORALLY_CONFIRMED",
        thermal_peak_k: Optional[float] = None,
        thermal_delta_k: Optional[float] = None,
        confirmation_count: Optional[int] = None,
        confirmation_window: Optional[int] = None,
        consensus_span_sec: Optional[float] = None,
        median_depth_estimate: Optional[float] = None,
        depth_valid_samples: Optional[int] = None,
        depth_spread: Optional[float] = None,
        **kwargs
    ) -> Tuple[Optional[str], bool]:
        """Processes an incoming visual or multispectral detection from autonomous perception.
        
        Enforces mandatory publisher fields:
          - world_x (finite float)
          - world_y (finite float)
          - confidence (finite float in [0.0, 1.0])
          - drone_id / confirming_drone_id (strict integer >= 0)
          - sim_time / timestamp (finite float >= 0.0)
          
        Validates optional metadata strictly (isfinite, non-fractional integers, non-negative bounds).
        Preserves source, source_track_id, and verification_mode independently.
        Filters candidate tracks to those within autonomous_association_radius before ambiguity comparison.
        Only eligible candidates may cause ambiguous association.
        Enforces conservative handling when multiple eligible candidates are in contention.

        Out-of-Order Simulation Timestamp Policy:
          - Delayed or reordered simulation events (sim_time < last_seen_sim) do not cause last_seen_sim
            or sim_time to regress.
          - Earliest simulation timestamp (first_seen_sim) updates to min(first_seen_sim, sim_time).
          - Wall-clock time (last_seen) tracks receipt time and is strictly decoupled from simulation time.
          - Observation count increments for every valid sighting.
          - Spatial smoothing via EMA (alpha=0.7) and monotonic confidence maximization are applied.
          - Active metadata is preserved from the latest simulation observation, with backfilling for
            any unpopulated optional fields.
          - Equal simulation timestamps are handled deterministically without regression.
        """
        # Legacy positional compatibility:
        # process_discovery_event(track_id, world_x, world_y, source, confidence, drone_id)
        if len(args) >= 5 and isinstance(args[3], str):
            legacy_track_id = args[0]
            world_x = float(args[1]) if world_x is None else world_x
            world_y = float(args[2]) if world_y is None else world_y
            source = str(args[3]) if source is None else source
            confidence = float(args[4]) if confidence is None else confidence
            if len(args) >= 6:
                drone_id = int(args[5]) if drone_id is None else drone_id
            if source_track_id is None:
                try:
                    source_track_id = int(legacy_track_id)
                except (ValueError, TypeError):
                    pass
            if sim_time is None:
                sim_time = 0.0
        elif len(args) >= 2:
            world_x = float(args[0]) if world_x is None else world_x
            world_y = float(args[1]) if world_y is None else world_y
            if len(args) >= 3 and source is None:
                source = str(args[2])
            if len(args) >= 4 and confidence is None:
                confidence = float(args[3])
            if len(args) >= 5 and drone_id is None:
                drone_id = int(args[4])

        # Extract keyword fallbacks
        if source_track_id is None:
            if "victim_track_id" in kwargs:
                source_track_id = kwargs["victim_track_id"]
            elif "track_id" in kwargs:
                source_track_id = kwargs["track_id"]

        if drone_id is None:
            if "confirming_drone_id" in kwargs:
                drone_id = kwargs["confirming_drone_id"]

        if sim_time is None:
            if "timestamp" in kwargs:
                sim_time = kwargs["timestamp"]

        # Validate mandatory fields: reject missing values with diagnostic warnings
        if world_x is None or world_y is None:
            self._log_warn("[VictimTracker] Missing mandatory world_x or world_y coordinates.")
            return None, False
        if confidence is None:
            self._log_warn("[VictimTracker] Missing mandatory confidence field.")
            return None, False
        if drone_id is None:
            self._log_warn("[VictimTracker] Missing mandatory drone_id/confirming_drone_id field.")
            return None, False
        if sim_time is None:
            self._log_warn("[VictimTracker] Missing mandatory simulation timestamp (sim_time/timestamp) field.")
            return None, False

        try:
            wx = _validate_strict_float(world_x, "world_x")
            wy = _validate_strict_float(world_y, "world_y")
            conf = _validate_strict_float(confidence, "confidence", min_val=0.0, max_val=1.0)
            st = _validate_strict_float(sim_time, "sim_time", min_val=0.0)
            d_id = _validate_strict_int(drone_id, "drone_id", min_val=0)
        except (ValueError, TypeError) as err:
            self._log_warn(f"[VictimTracker] Mandatory field validation failure: {err}")
            return None, False

        # Source provenance handling: never default to multispectral_verifier
        if source is not None and str(source).strip():
            src = str(source).strip()
        else:
            src = UNKNOWN_AUTONOMOUS_SOURCE

        # Validate optional metadata fields
        try:
            s_tid = _validate_optional_int(source_track_id, "source_track_id", min_val=0)
            c_count = _validate_optional_int(confirmation_count, "confirmation_count", min_val=0)
            c_win = _validate_optional_int(confirmation_window, "confirmation_window", min_val=0)
            d_samples = _validate_optional_int(depth_valid_samples, "depth_valid_samples", min_val=0)
            
            m_depth = _validate_optional_float(median_depth_estimate, "median_depth_estimate", min_val=0.0)
            c_span = _validate_optional_float(consensus_span_sec, "consensus_span_sec", min_val=0.0)
            d_spread = _validate_optional_float(depth_spread, "depth_spread", min_val=0.0)
            t_peak = _validate_optional_float(thermal_peak_k, "thermal_peak_k", min_val=0.0)
            t_delta = _validate_optional_float(thermal_delta_k, "thermal_delta_k")
        except (ValueError, TypeError) as err:
            self._log_warn(f"[VictimTracker] Optional metadata validation failure: {err}")
            return None, False

        # Spatial association against existing tracks
        all_candidates = []
        for tid, track in self.tracked_victims.items():
            dist = math.hypot(track.world_x - wx, track.world_y - wy)
            all_candidates.append((dist, tid, track))

        # Filter candidate tracks to those within autonomous_association_radius before ambiguity comparison.
        # Only eligible candidates may cause ambiguous association.
        eligible_candidates = [
            c for c in all_candidates
            if c[0] <= self.autonomous_association_radius
        ]
        eligible_candidates.sort(key=lambda item: item[0])

        is_new = False
        target_track = None
        target_tid = None

        if not eligible_candidates:
            is_new = True
        elif len(eligible_candidates) == 1:
            target_dist, target_tid, target_track = eligible_candidates[0]
            is_new = False
        else:
            # Multiple ELIGIBLE candidates within radius: check contention between top two
            best_dist, best_tid, best_track = eligible_candidates[0]
            second_dist, second_tid, second_track = eligible_candidates[1]
            if (second_dist - best_dist) < self.ambiguity_margin:
                self._log_warn(
                    f"[VictimTracker] Ambiguous association: detection at ({wx:.2f}, {wy:.2f}) "
                    f"is in contention between eligible tracks {best_tid} ({best_dist:.2f}m) and "
                    f"{second_tid} ({second_dist:.2f}m) with delta {(second_dist - best_dist):.2f}m < {self.ambiguity_margin}m. "
                    f"Conservative handling: creating distinct track."
                )
                is_new = True
            else:
                target_tid = best_tid
                target_track = best_track
                is_new = False

        if is_new:
            self.next_track_id += 1
            internal_track_id = f"victim_{self.next_track_id}"
            new_track = TrackedVictim(
                t_id=internal_track_id,
                world_x=wx,
                world_y=wy,
                source=src,
                confidence=conf,
                drone_id=d_id,
                sim_time=st,
                source_track_id=s_tid,
                verification_mode=verification_mode,
                evidence_state=evidence_state,
                thermal_peak_k=t_peak,
                thermal_delta_k=t_delta,
                confirmation_count=c_count,
                confirmation_window=c_win,
                consensus_span_sec=c_span,
                median_depth_estimate=m_depth,
                depth_valid_samples=d_samples,
                depth_spread=d_spread
            )
            self.tracked_victims[internal_track_id] = new_track
            self._log_info(
                f"[VictimTracker] New autonomous track {internal_track_id} initialized at ({wx:.1f}, {wy:.1f}) "
                f"by D{d_id} (source={src}, mode={verification_mode}, local_id={s_tid})"
            )
            return internal_track_id, True
        else:
            target_track.update(
                world_x=wx,
                world_y=wy,
                source=src,
                confidence=conf,
                drone_id=d_id,
                sim_time=st,
                source_track_id=s_tid,
                verification_mode=verification_mode,
                evidence_state=evidence_state,
                thermal_peak_k=t_peak,
                thermal_delta_k=t_delta,
                confirmation_count=c_count,
                confirmation_window=c_win,
                consensus_span_sec=c_span,
                median_depth_estimate=m_depth,
                depth_valid_samples=d_samples,
                depth_spread=d_spread
            )
            self._log_info(
                f"[VictimTracker] Updated autonomous track {target_tid} at ({target_track.world_x:.1f}, {target_track.world_y:.1f}) "
                f"by D{d_id} (obs={target_track.observations})"
            )
            return target_tid, False

    def mark_detected(self, v_id, detected_by=None, detection_distance=None):
        if v_id in self.victims and self.victims[v_id].state == VictimState.UNDETECTED:
            self.victims[v_id].state = VictimState.DETECTED
            self.victims[v_id].detected_by = detected_by
            self.victims[v_id].detection_distance = detection_distance
            self.victims[v_id].detection_time = self._get_current_time()
            self.node.get_logger().info(f"[VictimManager] Ground truth {v_id} marked as DETECTED by {detected_by} at {detection_distance:.1f}m!")
            return True
        return False
        
    def mark_detected_by_grid(self, grid_x, grid_y):
        for v_id, victim in self.victims.items():
            if victim.grid_x == grid_x and victim.grid_y == grid_y:
                return self.mark_detected(v_id)
        return False

    def is_valid_destination(self, wx, wy):
        """Checks if a continuous world coordinate is a valid non-obstacle location."""
        gx, gy = GridWorldTransform.world_to_grid(wx, wy)
        gx, gy, is_valid = GridWorldTransform.clamp_grid(gx, gy)
        
        if not is_valid:
            return False
            
        if self.env.grid[gx, gy] == -1: # -1 is obstacle in SARGridEnv
            return False
            
        return True

    def choose_new_destination(self, victim):
        """Chooses a valid random destination within the victim's movement radius."""
        if not victim.profile["movement_enabled"]:
            return False
            
        radius = victim.profile["radius"]
        max_attempts = 10
        
        for _ in range(max_attempts):
            angle = self.rng.uniform(0, 2 * math.pi)
            dist = self.rng.uniform(1.0, radius)
            
            target_wx = victim.world_x + math.cos(angle) * dist
            target_wy = victim.world_y + math.sin(angle) * dist
            
            if self.is_valid_destination(target_wx, target_wy):
                victim.dest_world_x = target_wx
                victim.dest_world_y = target_wy
                return True
                
        return False

    def update(self):
        """Main update loop. Should be called periodically from the mission controller."""
        current_time = self._get_current_time()
        
        # Guard against uninitialized clock
        if current_time <= 0.0:
            return
            
        # First update initializes baseline without stepping
        if self.last_update_time is None or self.last_update_time <= 0.0:
            self.last_update_time = current_time
            for victim in self.victims.values():
                victim.last_gz_update_time = current_time
            return
            
        # Clock reset or rewind: resynchronize baseline without advancing
        if current_time < self.last_update_time:
            self.last_update_time = current_time
            return

        dt = current_time - self.last_update_time
        
        # Simulation is paused or zero elapsed time: do not advance victims
        if dt == 0.0:
            return
            
        # Large discontinuous time jump (>2.0s):
        # Resync time baseline without applying a teleporting delta step
        if dt > 2.0:
            self.last_update_time = current_time
            return
            
        self.last_update_time = current_time
        
        # Update track statuses
        for t_id, track in self.tracked_victims.items():
            track.check_status(current_time)
        
        for v_id, victim in self.victims.items():
            if victim.state == VictimState.RESCUED:
                continue
                
            if not victim.profile["movement_enabled"]:
                continue
                
            if victim.movement_state == MovementState.RESTING:
                victim.rest_timer -= dt
                if victim.rest_timer <= 0:
                    if self.choose_new_destination(victim):
                        victim.movement_state = MovementState.MOVING
                        victim.speed = self.rng.uniform(victim.profile['min_speed'], victim.profile['max_speed'])
                    else:
                        # Failed to find valid destination, reset timer
                        victim.rest_timer = self.rng.uniform(victim.profile['min_rest_duration'], victim.profile['max_rest_duration'])
                        
            elif victim.movement_state == MovementState.MOVING:
                dx = victim.dest_world_x - victim.world_x
                dy = victim.dest_world_y - victim.world_y
                dist = math.hypot(dx, dy)
                
                if dist < self.waypoint_tolerance:
                    victim.movement_state = MovementState.RESTING
                    victim.rest_timer = self.rng.uniform(victim.profile['min_rest_duration'], victim.profile['max_rest_duration'])
                else:
                    # Move towards destination
                    move_dist = min(victim.speed * dt, dist)
                    victim.world_x += (dx / dist) * move_dist
                    victim.world_y += (dy / dist) * move_dist
                    
                    # Update grid coordinates
                    gx, gy = GridWorldTransform.world_to_grid(victim.world_x, victim.world_y)
                    victim.grid_x, victim.grid_y, _ = GridWorldTransform.clamp_grid(gx, gy)
                    
                    # Update Gazebo model at a throttled rate (1 Hz)
                    if current_time - victim.last_gz_update_time > self.gz_update_interval:
                        self.update_gazebo_pose(victim)
                        victim.last_gz_update_time = current_time

    def update_gazebo_pose(self, victim):
        """Updates the physical Gazebo model position using the set_pose service."""
        cmd = [
            'gz', 'service', '-s', f'/world/{self.world_name}/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--req', f'name: "{victim.id}", position: {{x: {victim.world_x}, y: {victim.world_y}, z: 0.1}}'
        ]
        # Run in background to avoid blocking Python loop
        subprocess.Popen(cmd)

    def get_dashboard_state(self):
        """Returns the serialized state for telemetry output (using mission victims)."""
        return [v.get_dict() for v in self.victims.values()]

    def evaluate_autonomous_tracks(self, threshold_m: float = 3.0) -> Any:
        """
        Executes strict one-to-one spatial matching between autonomous victim tracks
        and ground truth victims for evaluation purposes ONLY.
        Updates self.gt_error on each tracked victim with its matched ground-truth distance.
        """
        matcher = GroundTruthMatcher(default_threshold_m=threshold_m)
        gt_list = [v.get_dict() for v in self.victims.values()]
        track_list = [t.get_dict() for t in self.tracked_victims.values()]
        
        result = matcher.match(gt_list, track_list, threshold_m=threshold_m)
        
        # Update gt_error on tracked victims
        matched_track_errors = {str(p.track_id): p.distance_error_m for p in result.matched_pairs}
        for tid, t in self.tracked_victims.items():
            str_tid = str(tid)
            if str_tid in matched_track_errors:
                t.gt_error = matched_track_errors[str_tid]
            else:
                t.gt_error = -1.0
                
        return result

    def get_evaluation_metrics(self, threshold_m: float = 3.0) -> Dict[str, Any]:
        """Returns online evaluation KPIs comparing autonomous tracks against ground truth."""
        res = self.evaluate_autonomous_tracks(threshold_m=threshold_m)
        return res.to_dict()

