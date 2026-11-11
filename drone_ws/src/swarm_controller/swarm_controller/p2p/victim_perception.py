"""
Perception and Victim Detection Simulation Module
Tier 3 Phase 6: Sensor-Based Perception, Coordinate Estimation, and Temporal Confirmation.
Models a downward-facing visual/depth sensor payload with realistic FOV footprint,
stochastic noise, and spatial uncertainty modeling without leaking ground truth.
"""

import math
import random
import time
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass, field

from ..grid_world_transform import GridWorldTransform


@dataclass
class PerceivedVictimDetection:
    detection_id: str
    victim_event_id: str
    reporting_drone_id: str
    estimated_world_enu: List[float]
    estimated_grid_cell: List[int]
    position_uncertainty_m: float
    confidence: float
    timestamp_utc: float = field(default_factory=time.time)
    sensor_modality: str = "geometric_camera_fov_frustum"
    observation_id: str = ""


class PerceptionTrackletTracker:
    """
    Perception Tracklet Manager for SAR Drone Swarm.
    Maintains temporal tracklet continuity for visual detections.

    Guarantees:
    1. Continuous sightings of the same target within tracking timeout share
       the same stable observation_id: f"track_{drone_id}_{target_id}_ep{track_epoch}".
    2. Consecutive frames from the same tracklet do NOT generate new independent
       evidence in the distributed journal's Noisy-OR fusion.
    3. When tracking is lost for longer than `tracklet_timeout_sec` (e.g. 4.0s)
       and later reacquired, a new tracklet epoch is incremented, treating the
       new pass as an independent observation event.
    4. Deterministic expiration and reacquisition.
    5. Tracklet identity survives serialization and network gossip.
    """
    def __init__(self, drone_id: str, tracklet_timeout_sec: float = 4.0):
        self.drone_id = drone_id
        self.tracklet_timeout_sec = float(tracklet_timeout_sec)
        self.active_tracklets: Dict[str, Dict[str, Any]] = {}
        self.target_epochs: Dict[str, int] = {}

    def process_detection(
        self,
        target_id: str = "",
        coords: Optional[List[float]] = None,
        uncertainty_m: float = 0.5,
        confidence: float = 0.8,
        timestamp_utc: float = 0.0,
        raw_target_id: Optional[str] = None,
        measured_coords: Optional[List[float]] = None
    ) -> Tuple[str, bool, float, float, List[float]]:
        """
        Processes a raw frame detection and associates it with a stable tracklet.
        Returns: (tracklet_id, is_new_tracklet, fused_confidence, fused_uncertainty, fused_coords)
        """
        eff_target_id = raw_target_id if raw_target_id is not None else target_id
        eff_coords = measured_coords if measured_coords is not None else (coords if coords is not None else [0.0, 0.0, 0.0])
        now = float(timestamp_utc)
        track = self.active_tracklets.get(eff_target_id)

        if track is None or (now - track["last_seen_utc"]) > self.tracklet_timeout_sec:
            # New tracklet episode (either first time or reacquired after timeout)
            self.target_epochs[eff_target_id] = self.target_epochs.get(eff_target_id, 0) + 1
            new_epoch = self.target_epochs[eff_target_id]
            tracklet_id = f"track_{self.drone_id}_{eff_target_id}_ep{new_epoch}"
            self.active_tracklets[eff_target_id] = {
                "track_epoch": new_epoch,
                "first_seen_utc": now,
                "last_seen_utc": now,
                "frame_count": 1,
                "tracklet_id": tracklet_id,
                "best_confidence": float(confidence),
                "fused_coords": [float(c) for c in eff_coords],
                "fused_uncertainty": float(uncertainty_m)
            }
            return tracklet_id, True, float(confidence), float(uncertainty_m), list(eff_coords)
        else:
            # Continuing existing tracklet: update temporal stats without creating new evidence
            track["last_seen_utc"] = now
            track["frame_count"] += 1
            # Running confidence: best confidence observed in tracklet
            track["best_confidence"] = max(track["best_confidence"], float(confidence))
            # Refine coordinates via running inverse-variance weighted average
            old_w = 1.0 / max(0.01, track["fused_uncertainty"] ** 2)
            new_w = 1.0 / max(0.01, float(uncertainty_m) ** 2)
            total_w = old_w + new_w
            track["fused_coords"] = [
                (track["fused_coords"][i] * old_w + float(eff_coords[i]) * new_w) / total_w
                for i in range(len(eff_coords))
            ]
            track["fused_uncertainty"] = math.sqrt(1.0 / total_w)
            return (
                track["tracklet_id"],
                False,
                track["best_confidence"],
                track["fused_uncertainty"],
                list(track["fused_coords"])
            )

    def prune_expired_tracklets(self, now_utc: float) -> List[str]:
        """Prunes tracklets that have been inactive longer than 2x timeout."""
        expired = []
        for tid, track in list(self.active_tracklets.items()):
            if (now_utc - track["last_seen_utc"]) > (2.0 * self.tracklet_timeout_sec):
                expired.append(tid)
                del self.active_tracklets[tid]
        return expired

    prune_stale_tracklets = prune_expired_tracklets


class SimulatedVictimSensor:
    """
    Simulates onboard camera perception payload (e.g. downward-facing RGB/RGB-D camera).
    Evaluates physical ground intersection against world victim targets, computes
    perspective projection with realistic sensor noise, and outputs perceived detections.
    """
    def __init__(
        self,
        drone_id: str,
        fov_horizontal_deg: float = 60.0,
        fov_vertical_deg: float = 45.0,
        noise_std_m: float = 0.35,
        min_detection_confidence: float = 0.70,
        occlusion_rate: float = 0.0,
        seed: Optional[int] = None
    ):
        self.drone_id = drone_id
        self.fov_h = math.radians(fov_horizontal_deg)
        self.fov_v = math.radians(fov_vertical_deg)
        self.noise_std = noise_std_m
        self.min_confidence = min_detection_confidence
        self.occlusion_rate = occlusion_rate
        self.rng = random.Random(seed) if seed is not None else random.Random()
        self.sighting_history: Dict[str, List[PerceivedVictimDetection]] = {}

    def set_seed(self, seed: int):
        self.rng = random.Random(seed)

    def scan_environment(
        self,
        drone_world_enu: List[float],
        drone_yaw_rad: float,
        candidate_world_victims: List[Dict[str, Any]]
    ) -> List[PerceivedVictimDetection]:
        """
        Scans ground plane from drone's current pose using downward-facing geometric camera frustum.
        
        Camera Mounting Geometry (from models/x500_rgb_down/model.sdf):
          - Pitch = +90 degrees (+1.5707 rad about body +Y)
          - Optical axis points down (-Z_body) towards ground plane
          - Camera image width (HFOV = 60 deg, 1.047 rad) spans drone lateral axis (+/- Y_body, Left-Right)
          - Camera image height (VFOV = 45 deg, 0.785 rad) spans drone longitudinal axis (+/- X_body, Forward-Back)
        
        Coverage Half-Dimensions at altitude h:
          - half_forward (X_body) = h * tan(VFOV / 2) = h * tan(22.5 deg) = 0.4142 * h (6.213m at 15m)
          - half_lateral (Y_body) = h * tan(HFOV / 2) = h * tan(30.0 deg) = 0.5774 * h (8.660m at 15m)
        """
        drone_x, drone_y, drone_z = drone_world_enu
        
        # Altitude gate: drone must be airborne (>= 1.0m) for downward camera ground projection
        if drone_z < 1.0 or not math.isfinite(drone_z):
            return []

        alt = float(drone_z)

        # Ground plane coverage half-dimensions derived from physical camera mounting
        half_forward = alt * math.tan(self.fov_v / 2.0)  # Longitudinal span along body X
        half_lateral = alt * math.tan(self.fov_h / 2.0)  # Lateral span along body Y
        max_ground_radius = math.hypot(half_forward, half_lateral)

        detections: List[PerceivedVictimDetection] = []

        # Sort candidate victims deterministically by victim ID to isolate PRNG sequences
        sorted_victims = sorted(
            candidate_world_victims,
            key=lambda v: str(v.get("victim_id") or v.get("id", "unknown_victim"))
        )

        # Hoist loop-invariant trigonometric yaw rotation once per scan
        cos_yaw = math.cos(-drone_yaw_rad)
        sin_yaw = math.sin(-drone_yaw_rad)

        for victim in sorted_victims:
            if "world_enu" in victim and isinstance(victim["world_enu"], (list, tuple)):
                vx = float(victim["world_enu"][0])
                vy = float(victim["world_enu"][1])
            elif "world" in victim and isinstance(victim["world"], dict):
                vx = float(victim["world"].get("x", 0.0))
                vy = float(victim["world"].get("y", 0.0))
            else:
                vx = float(victim.get("world_x", 0.0))
                vy = float(victim.get("world_y", 0.0))
            v_id = str(victim.get("victim_id") or victim.get("id", "unknown_victim"))

            dx_world = vx - drone_x
            dy_world = vy - drone_y

            # Rotate horizontal displacement into drone FLU body frame
            # cos(-yaw) = cos(yaw), sin(-yaw) = -sin(yaw)
            dx_body = dx_world * cos_yaw - dy_world * sin_yaw  # Forward-backward offset
            dy_body = dx_world * sin_yaw + dy_world * cos_yaw  # Left-right lateral offset

            # Check if within ground FOV rectangular bounds
            if abs(dx_body) <= half_forward and abs(dy_body) <= half_lateral:
                # Stochastic visual occlusion / low-light drop
                if self.occlusion_rate > 0.0 and self.rng.random() < self.occlusion_rate:
                    continue

                dist_center = math.hypot(dx_body, dy_body)
                norm_dist = min(1.0, dist_center / max(0.1, max_ground_radius))

                # Sensor confidence: highest at image nadir, decays towards edge
                conf = max(self.min_confidence, 0.98 - 0.25 * norm_dist)

                # Add zero-mean Gaussian sensor noise
                noise_x = self.rng.gauss(0.0, self.noise_std)
                noise_y = self.rng.gauss(0.0, self.noise_std)

                est_x = vx + noise_x
                est_y = vy + noise_y
                est_gx, est_gy = GridWorldTransform.world_to_grid(est_x, est_y)

                # 1-sigma uncertainty radius grows with radial distance from nadir
                unc = math.sqrt(self.noise_std ** 2 + 0.05 * (dist_center ** 2))

                det = PerceivedVictimDetection(
                    detection_id=f"det_{v_id}_{int(time.time()*1000)}",
                    victim_event_id=v_id,
                    reporting_drone_id=self.drone_id,
                    estimated_world_enu=[round(est_x, 3), round(est_y, 3), 0.2],
                    estimated_grid_cell=[est_gx, est_gy],
                    position_uncertainty_m=round(unc, 3),
                    confidence=round(conf, 4)
                )
                detections.append(det)

                if v_id not in self.sighting_history:
                    self.sighting_history[v_id] = []
                self.sighting_history[v_id].append(det)

        return detections
