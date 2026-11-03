"""
Shared Temporal Event Confirmer & Multi-Drone Tracker Module (Phase 2 Remediation)

Provides robust multi-drone spatio-temporal consensus, frame-uniqueness validation,
dynamic spatial gating with localization uncertainty, mobile victim trajectory tracking,
and duplicate discovery suppression.

Shared across:
  - rgbd_victim_localizer.py (RGB-D baseline localization)
  - rgb_thermal_fusion_verifier.py (Multispectral RGB-Thermal verification)
"""

import math
from collections import deque
from typing import Dict, List, Optional, Tuple, Any

try:
    from swarm_controller.grid_world_transform import GridWorldTransform
except ImportError:
    try:
        from .grid_world_transform import GridWorldTransform
    except ImportError:
        from grid_world_transform import GridWorldTransform


class TemporalEventConfirmer:
    """
    Robust spatio-temporal consensus engine for multi-drone search and rescue.

    Key Architectural Guarantees:
      1. Frame Uniqueness: Each camera frame (identified by (drone_id, timestamp))
         contributes AT MOST one observation to a track. Multiple bounding boxes
         from the same image cannot satisfy temporal consensus.
      2. Dynamic Gating: Spatial association gate adapts to observation depth
         uncertainty and elapsed time for mobile victims.
      3. Cross-Drone Fusion: Observations from multiple drones seamlessly fuse
         into a unified victim track if spatially and temporally consistent.
      4. Mobile Victim Tracking: Track centroids adapt as victims move, preventing
         spurious duplicate IDs when a confirmed victim relocates.
      5. Deduplication: Confirmed discoveries suppress duplicate events within
         dedup_radius, dynamically following the victim's updated location.
    """

    def __init__(
        self,
        window_size: int = 6,
        consensus_needed: int = 4,
        proximity_threshold: float = 3.0,
        grid_cell_size: float = 8.0,
        min_confidence: float = 0.35,
        min_time_span: float = 0.4,
        min_depth: float = 3.0,
        max_depth: float = 35.0,
        min_valid_samples: int = 10,
        max_depth_spread: float = 3.0,
        dedup_radius: float = 6.0,
        max_victim_speed_m_s: float = 1.5,
        max_track_coasting_sec: float = 20.0
    ):
        self.window_size = window_size
        self.consensus_needed = consensus_needed
        self.proximity_threshold = proximity_threshold
        self.grid_cell_size = grid_cell_size
        self.min_confidence = min_confidence
        self.min_time_span = min_time_span
        self.min_depth = min_depth
        self.max_depth = max_depth
        self.min_valid_samples = min_valid_samples
        self.max_depth_spread = max_depth_spread
        self.dedup_radius = dedup_radius
        self.max_victim_speed = max_victim_speed_m_s
        self.max_track_coasting_sec = max_track_coasting_sec

        self.tracks: Dict[int, Dict[str, Any]] = {}
        self.next_track_id: int = 1
        self.confirmed_victims: List[Dict[str, Any]] = []

    def is_valid_observation(
        self,
        world_x: float,
        world_y: float,
        confidence: float,
        median_depth: float,
        valid_samples: int,
        depth_spread: float,
        timestamp: Optional[float] = None
    ) -> Tuple[bool, str]:
        """Validates a single observation against coordinate bounds, confidence, depth, and timestamp."""
        # 1. Timestamp validity (simulation clock if provided must be non-negative and finite)
        if timestamp is not None and (timestamp < 0.0 or math.isnan(timestamp) or math.isinf(timestamp)):
            return False, "INVALID_TIMESTAMP"

        # 2. Search-grid / world-coordinate validity
        if not GridWorldTransform.is_within_world_bounds(world_x, world_y):
            return False, "OUT_OF_BOUNDS"

        # 3. Confidence threshold
        if confidence < self.min_confidence:
            return False, "LOW_CONFIDENCE"

        # 4. Depth plausibility & sample quality
        if not (self.min_depth <= median_depth <= self.max_depth):
            return False, "INVALID_DEPTH_RANGE"
        if valid_samples < self.min_valid_samples:
            return False, "INSUFFICIENT_DEPTH_SAMPLES"
        if depth_spread > self.max_depth_spread:
            return False, "HIGH_DEPTH_SPREAD"

        return True, "VALID"

    def _compute_gate_radius(self, median_depth: float, dt: float) -> float:
        """
        Computes dynamic spatial association gate accounting for:
          - Nominal proximity threshold (3.0m)
          - Lateral localization uncertainty scaling with depth (~8% of depth)
          - Potential victim displacement over elapsed time dt (up to 5s window)
        """
        depth_uncertainty = max(0.0, 0.08 * median_depth)
        motion_slack = self.max_victim_speed * min(max(0.0, dt), 5.0)
        return self.proximity_threshold + depth_uncertainty + motion_slack

    def find_associated_track(
        self,
        world_x: float,
        world_y: float,
        timestamp: float,
        median_depth: float
    ) -> Optional[int]:
        """
        Associates an incoming observation with the best active or confirmed track
        using spatial gating, depth uncertainty, and kinematic motion bounds.
        """
        best_track_id = None
        min_distance = float('inf')

        # Check all existing tracks
        for tid, track in self.tracks.items():
            if not track['history']:
                continue

            last_obs = track['history'][-1]
            dt = timestamp - last_obs['ts']

            # If track has been inactive longer than max coasting window, skip association
            if dt > self.max_track_coasting_sec:
                continue

            # Calculate distance to current estimated position (or last observation)
            est_x = track.get('est_x', last_obs['x'])
            est_y = track.get('est_y', last_obs['y'])
            dist = math.hypot(world_x - est_x, world_y - est_y)

            gate_radius = self._compute_gate_radius(median_depth, dt)

            if dist <= gate_radius and dist < min_distance:
                min_distance = dist
                best_track_id = tid

        return best_track_id

    def add_observation(
        self,
        drone_id: int,
        timestamp: float,
        world_x: float,
        world_y: float,
        confidence: float,
        median_depth: float,
        valid_samples: int = 15,
        depth_spread: float = 1.0,
        thermal_stats: Optional[Dict[str, Any]] = None,
        verification_mode: str = "RGBD"
    ) -> Tuple[Optional[int], Optional[Dict[str, Any]]]:
        """
        Processes an incoming detection observation.

        Returns:
            (track_id, discovery_event)
            - track_id: ID of the associated or newly created track, or None if invalid
            - discovery_event: Dict containing VICTIM_DISCOVERED payload if consensus reached, else None
        """
        # 1. Validate observation
        is_valid, reason = self.is_valid_observation(
            world_x, world_y, confidence, median_depth, valid_samples, depth_spread, timestamp
        )
        if not is_valid:
            return None, None

        # 2. Associate with existing track or allocate new track
        best_track_id = self.find_associated_track(world_x, world_y, timestamp, median_depth)

        if best_track_id is None:
            # Check if this observation is near an already confirmed victim
            # If so, associate with the confirmed victim's track to track motion without new ID
            for cv in self.confirmed_victims:
                d_conf = math.hypot(world_x - cv['x'], world_y - cv['y'])
                dt_conf = timestamp - cv.get('ts', timestamp)
                if d_conf <= self._compute_gate_radius(median_depth, dt_conf):
                    best_track_id = cv['track_id']
                    break

        if best_track_id is None:
            # Allocate a new candidate track
            best_track_id = self.next_track_id
            self.next_track_id += 1
            self.tracks[best_track_id] = {
                'history': deque(maxlen=self.window_size),
                'seen_frames': set(),
                'discovered': False,
                'est_x': world_x,
                'est_y': world_y
            }

        track = self.tracks[best_track_id]

        # 3. Frame-Uniqueness Enforcement
        # Identify each camera frame uniquely by (drone_id, timestamp rounded to 0.1ms)
        frame_key = (drone_id, round(float(timestamp), 4))

        if frame_key in track['seen_frames']:
            # Another detection from the EXACT SAME camera frame was already processed
            # Update the observation if this new detection has higher confidence, but do not increment consensus
            for obs in track['history']:
                if (obs['drone'], round(float(obs['ts']), 4)) == frame_key:
                    if confidence > obs['conf']:
                        obs['conf'] = confidence
                        obs['x'] = world_x
                        obs['y'] = world_y
                        obs['depth'] = median_depth
                    break
            return best_track_id, None

        # Record this unique frame
        track['seen_frames'].add(frame_key)
        # Keep seen_frames bounded to window size
        if len(track['seen_frames']) > self.window_size * 2:
            oldest_ts = min(k[1] for k in track['seen_frames'])
            track['seen_frames'] = {k for k in track['seen_frames'] if k[1] > oldest_ts}

        grid_x, grid_y = GridWorldTransform.world_to_grid(world_x, world_y)

        # Update running position estimate (exponential smoothing for mobile victims)
        alpha = 0.35
        track['est_x'] = (1.0 - alpha) * track.get('est_x', world_x) + alpha * world_x
        track['est_y'] = (1.0 - alpha) * track.get('est_y', world_y) + alpha * world_y

        obs_record = {
            'x': world_x,
            'y': world_y,
            'gx': grid_x,
            'gy': grid_y,
            'ts': float(timestamp),
            'conf': float(confidence),
            'depth': float(median_depth),
            'samples': int(valid_samples),
            'spread': float(depth_spread),
            'drone': int(drone_id),
            'thermal_stats': thermal_stats,
            'verification_mode': verification_mode
        }
        track['history'].append(obs_record)

        # If already discovered, update confirmed victim position to track mobile targets
        if track['discovered']:
            for cv in self.confirmed_victims:
                if cv['track_id'] == best_track_id:
                    cv['x'] = track['est_x']
                    cv['y'] = track['est_y']
                    cv['ts'] = float(timestamp)
                    break
            return best_track_id, None

        # 4. Check Temporal Consensus Requirements
        discovery_event = None

        if len(track['history']) >= self.consensus_needed:
            recent_obs = list(track['history'])[-self.consensus_needed:]

            # 4a. Requirement: Observations must originate from distinct camera frames
            unique_frames_in_consensus = {(o['drone'], round(o['ts'], 4)) for o in recent_obs}
            if len(unique_frames_in_consensus) < self.consensus_needed:
                return best_track_id, None

            # 4b. Requirement: Consensus time span >= min_time_span (sorted to handle multi-drone arrival jitter)
            sorted_ts = sorted(o['ts'] for o in recent_obs)
            time_span = sorted_ts[-1] - sorted_ts[0]
            if time_span < self.min_time_span:
                return best_track_id, None

            # 4c. Requirement: Average confidence across consensus observations >= min_confidence
            avg_conf = sum(o['conf'] for o in recent_obs) / len(recent_obs)
            if avg_conf < self.min_confidence:
                return best_track_id, None

            # 4d. Compute robust centroid across consensus observations
            mean_x = sum(o['x'] for o in recent_obs) / len(recent_obs)
            mean_y = sum(o['y'] for o in recent_obs) / len(recent_obs)

            # 4e. Deduplication check against already confirmed victims
            for cv in self.confirmed_victims:
                dist = math.hypot(mean_x - cv['x'], mean_y - cv['y'])
                if dist < self.dedup_radius:
                    track['discovered'] = True
                    return best_track_id, None

            # 4f. Consensus verified: emit discovery event
            winning_obs = recent_obs[-1]
            rep_gx, rep_gy = GridWorldTransform.world_to_grid(mean_x, mean_y)

            track['discovered'] = True
            self.confirmed_victims.append({
                'track_id': best_track_id,
                'x': mean_x,
                'y': mean_y,
                'ts': float(timestamp),
                'drone': int(drone_id)
            })

            th_stat = winning_obs.get('thermal_stats') or {}
            th_peak = round(float(th_stat['max_k']), 2) if 'max_k' in th_stat else None
            th_delta = round(float(th_stat['delta_k']), 2) if 'delta_k' in th_stat else None

            discovery_event = {
                "event_type": "VICTIM_DISCOVERED",
                "timestamp": float(timestamp),
                "victim_track_id": int(best_track_id),
                "confirming_drone_id": int(drone_id),
                "world_x": round(float(mean_x), 2),
                "world_y": round(float(mean_y), 2),
                "grid_x": int(rep_gx),
                "grid_y": int(rep_gy),
                "grid_cell_size_m": float(self.grid_cell_size),
                "confidence": round(float(avg_conf), 3),
                "median_depth_estimate": round(float(winning_obs['depth']), 2),
                "confirmation_count": int(self.consensus_needed),
                "confirmation_window": int(self.window_size),
                "consensus_span_sec": round(float(time_span), 3),
                "depth_valid_samples": int(winning_obs['samples']),
                "depth_spread": round(float(winning_obs['spread']), 2),
                "verification_mode": str(verification_mode),
                "thermal_peak_k": th_peak,
                "thermal_delta_k": th_delta
            }

        return best_track_id, discovery_event

    def add_multispectral_observation(
        self,
        candidate_result: Any
    ) -> Tuple[Optional[int], Optional[Dict[str, Any]]]:
        """
        Convenience adapter for CandidateVerificationResult objects
        produced by RGBThermalFusionCore.
        """
        if candidate_result.world_pos is None:
            return None, None

        wx, wy = candidate_result.world_pos
        return self.add_observation(
            drone_id=candidate_result.drone_id,
            timestamp=candidate_result.timestamp,
            world_x=wx,
            world_y=wy,
            confidence=candidate_result.rgb_confidence,
            median_depth=candidate_result.depth_m,
            valid_samples=15,
            depth_spread=1.0,
            thermal_stats=candidate_result.thermal_stats,
            verification_mode="MULTISPECTRAL_RGB_THERMAL"
        )
