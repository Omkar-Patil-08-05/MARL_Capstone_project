#!/usr/bin/env python3
"""
RGB-Thermal Multispectral Victim Fusion & Verification Module (Phase 2 Remediation - Stage 4)
Combines RGB YOLO candidate detections, radiometric 16-bit thermal evidence,
and metric depth information to verify search and rescue victims.

Stage 4 Multi-Feature Thermal Analysis:
  1. Temperature Percentiles (P10, P50, P75, P90, P95, min, max, mean, std)
  2. Maximum and Mean Temperature
  3. Local Background Annulus Temperature (mean, median, std)
  4. Thermal Contrast (Delta_T, Delta_Mean, SNR)
  5. Spatial Temperature Gradients (Sobel magnitude mean and 90th percentile)
  6. Core-to-Periphery Distribution (central thoracic/cranial core vs outer ROI ring)
  7. Connected-Component Hotspot Analysis (largest component area, eccentricity, centroid offset)
  8. Thermal ROI Occupancy (fraction of elevated pixels in ROI)
  9. Physical Metric Dimensions and Aspect Ratio from RGB-D
  10. RGB Detector Confidence & Multispectral Verification Score (MVS)

Evidence States:
  - RGB_CANDIDATE: RGB detection candidate; thermal unverified or pending
  - THERMAL_SUPPORTED: Radiometric and morphological features support candidate
  - TEMPORALLY_CONFIRMED: Multi-frame spatio-temporal consensus confirmed across unique frames
  - REJECTED: Candidate rejected by geometric, thermal, or morphological criteria

Preserves:
  - Backward compatibility with VERIFIED and CANDIDATE_UNVERIFIED aliases.
  - Multi-drone (drones 0..5) independent asynchronous stream processing.
  - Integration with shared TemporalEventConfirmer.
"""

import os
import sys
import math
import time
import json
from enum import Enum
from collections import deque
from dataclasses import dataclass, asdict, field
from typing import Optional, Tuple, Dict, List, Any

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2

try:
    from swarm_controller.grid_world_transform import GridWorldTransform
except ImportError:
    class GridWorldTransform:
        METERS_PER_CELL = 8.0
        WORLD_MIN_X = -180.0
        WORLD_MAX_X = 20.0
        WORLD_MIN_Y = -20.0
        WORLD_MAX_Y = 180.0
        ORIGIN_X = -180.0
        ORIGIN_Y = -20.0

        @classmethod
        def is_within_world_bounds(cls, x, y):
            return (cls.WORLD_MIN_X <= x <= cls.WORLD_MAX_X and
                    cls.WORLD_MIN_Y <= y <= cls.WORLD_MAX_Y)

        @classmethod
        def world_to_grid(cls, x, y):
            gx = int((x - cls.ORIGIN_X) / cls.METERS_PER_CELL)
            gy = int((y - cls.ORIGIN_Y) / cls.METERS_PER_CELL)
            return gx, gy

try:
    from swarm_controller.temporal_event_confirmer import TemporalEventConfirmer
except ImportError:
    try:
        from .temporal_event_confirmer import TemporalEventConfirmer
    except ImportError:
        from temporal_event_confirmer import TemporalEventConfirmer


def quat2euler(q):
    w, x, y, z = q
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [roll, pitch, yaw]


class VerificationStatus(str, Enum):
    RGB_CANDIDATE = "RGB_CANDIDATE"
    THERMAL_SUPPORTED = "THERMAL_SUPPORTED"
    TEMPORALLY_CONFIRMED = "TEMPORALLY_CONFIRMED"
    REJECTED = "REJECTED"

    # Backward-compatible aliases
    VERIFIED = "THERMAL_SUPPORTED"
    CANDIDATE_UNVERIFIED = "RGB_CANDIDATE"


class RejectionReason(str, Enum):
    NONE = "NONE"
    OUT_OF_THERMAL_FOV = "OUT_OF_THERMAL_FOV"
    COLD_SHADOW_OR_OBSTACLE = "COLD_SHADOW_OR_OBSTACLE"
    HOT_NON_HUMAN_CLUTTER = "HOT_NON_HUMAN_CLUTTER"
    LOW_THERMAL_CONTRAST = "LOW_THERMAL_CONTRAST"
    INVALID_DEPTH = "INVALID_DEPTH"
    LOW_RGB_CONFIDENCE = "LOW_RGB_CONFIDENCE"
    GEOMETRIC_IMPLAUSIBLE = "GEOMETRIC_IMPLAUSIBLE"
    OUT_OF_WORLD_BOUNDS = "OUT_OF_WORLD_BOUNDS"
    DUPLICATE_NEAR_CONFIRMED = "DUPLICATE_NEAR_CONFIRMED"
    MISSING_SENSOR_DATA = "MISSING_SENSOR_DATA"
    UNIFORM_DIFFUSE_HEAT = "UNIFORM_DIFFUSE_HEAT"
    NON_HUMAN_THERMAL_MORPHOLOGY = "NON_HUMAN_THERMAL_MORPHOLOGY"
    IMPLAUSIBLE_THERMAL_OCCUPANCY = "IMPLAUSIBLE_THERMAL_OCCUPANCY"
    SATURATED_OR_CORRUPT_FRAME = "SATURATED_OR_CORRUPT_FRAME"
    LOW_MULTISPECTRAL_SCORE = "LOW_MULTISPECTRAL_SCORE"
    HIGH_UNCERTAINTY = "HIGH_UNCERTAINTY"


@dataclass
class ThermalFeatureConfig:
    """Configurable thresholds for multi-feature thermal verification."""
    min_rgb_confidence: float = 0.35
    min_depth_m: float = 3.0
    max_depth_m: float = 35.0
    min_victim_temp_k: float = 296.0        # ~22.85 °C (well above ambient 288.15 K)
    max_human_temp_k: float = 318.15        # 45.0 °C (reject fire, exhaust, boiling radiators)
    max_sensor_saturation_k: float = 323.0  # Sensor saturation limit check
    min_thermal_delta_k: float = 3.0        # Minimum thermal contrast above local background
    min_physical_size_m: float = 0.30       # Min human feature dimension (prone head/torso)
    max_physical_size_m: float = 2.40       # Max human dimension (standing adult with margin)
    min_core_periphery_delta_k: float = 0.30 # Minimum core hotspot concentration above periphery
    min_spatial_gradient: float = 0.20      # Minimum spatial temperature gradient (K/px)
    max_thermal_occupancy: float = 0.90     # Max fraction of hot pixels in ROI (reject uniform hot slabs)
    min_thermal_occupancy: float = 0.05     # Min fraction of hot pixels in ROI (reject tiny specks/noise)
    min_mvs_score: float = 0.45             # Minimum Multispectral Verification Score
    max_uncertainty_score: float = 0.70     # Maximum tolerable uncertainty for THERMAL_SUPPORTED


@dataclass
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    hfov_rad: float

    @classmethod
    def from_fov(cls, width: int, height: int, hfov_rad: float):
        fx = (width / 2.0) / math.tan(hfov_rad / 2.0)
        fy = fx
        cx = width / 2.0
        cy = height / 2.0
        return cls(fx=fx, fy=fy, cx=cx, cy=cy, width=width, height=height, hfov_rad=hfov_rad)


@dataclass
class ThermalStats:
    # 1. Percentiles & Base Temperature
    min_k: float
    max_k: float
    mean_k: float
    median_k: float
    std_k: float
    p90_k: float
    bg_mean_k: float
    bg_std_k: float
    delta_k: float
    snr: float
    p10_k: float = 0.0
    p50_k: float = 0.0
    p75_k: float = 0.0
    p95_k: float = 0.0

    # 2. Local Background
    bg_median_k: float = 0.0
    delta_mean_k: float = 0.0

    # 3. Spatial Temperature Gradients
    grad_mean: float = 0.0
    grad_p90: float = 0.0

    # 4. Core-to-Periphery Distribution
    core_mean_k: float = 0.0
    periphery_mean_k: float = 0.0
    core_periphery_delta_k: float = 0.0
    core_periphery_ratio: float = 0.0

    # 5. Connected-Component & Occupancy
    hot_pixel_fraction: float = 0.0
    primary_component_fraction: float = 0.0
    centroid_offset_norm: float = 0.0

    # 6. Physical Dimensions & Geometry
    physical_width_m: float = 0.0
    physical_height_m: float = 0.0
    aspect_ratio: float = 0.0

    # 7. Composite Scoring & Uncertainty
    rgb_confidence: float = 0.0
    mvs_score: float = 0.0
    uncertainty_score: float = 0.0
    uncertainty_flags: List[str] = field(default_factory=list)


@dataclass
class CandidateVerificationResult:
    candidate_id: str
    drone_id: int
    timestamp: float
    rgb_confidence: float
    rgb_bbox: List[int]
    projected_thermal_bbox: Optional[List[int]]
    depth_m: float
    physical_dims_m: Tuple[float, float]
    thermal_stats: Optional[Dict]
    status: str
    rejection_reason: str
    world_pos: Optional[Tuple[float, float]] = None
    grid_pos: Optional[Tuple[int, int]] = None
    evidence_state: Optional[str] = None
    verification_score: float = 0.0
    uncertainty_score: float = 0.0

    def to_dict(self):
        return {
            'candidate_id': self.candidate_id,
            'drone_id': self.drone_id,
            'timestamp': round(self.timestamp, 4),
            'rgb_confidence': round(self.rgb_confidence, 3),
            'rgb_bbox': self.rgb_bbox,
            'projected_thermal_bbox': self.projected_thermal_bbox,
            'depth_m': round(self.depth_m, 2),
            'physical_dims_m': [round(x, 2) for x in self.physical_dims_m],
            'thermal_stats': self.thermal_stats,
            'status': self.status,
            'evidence_state': self.evidence_state or self.status,
            'verification_score': round(self.verification_score, 3),
            'uncertainty_score': round(self.uncertainty_score, 3),
            'rejection_reason': self.rejection_reason,
            'world_pos': [round(p, 2) for p in self.world_pos] if self.world_pos else None,
            'grid_pos': list(self.grid_pos) if self.grid_pos else None
        }


class RGBThermalFusionCore:
    """
    Core mathematical and algorithmic engine for RGB-Thermal projective
    alignment, radiometric temperature decoding, multi-feature analysis,
    and multispectral candidate verification.
    """

    def __init__(
        self,
        rgb_intrinsics: Optional[CameraIntrinsics] = None,
        thermal_intrinsics: Optional[CameraIntrinsics] = None,
        translation_rgb_to_th: Tuple[float, float, float] = (-0.030, 0.0, 0.0),
        rotation_rgb_to_th: Optional[np.ndarray] = None,
        config: Optional[ThermalFeatureConfig] = None,
        min_rgb_confidence: Optional[float] = None,
        min_depth_m: Optional[float] = None,
        max_depth_m: Optional[float] = None,
        min_victim_temp_k: Optional[float] = None,
        max_human_temp_k: Optional[float] = None,
        min_thermal_delta_k: Optional[float] = None,
        min_physical_size_m: Optional[float] = None,
        max_physical_size_m: Optional[float] = None,
    ):
        self.rgb_cam = rgb_intrinsics or CameraIntrinsics.from_fov(640, 480, 1.74)
        self.th_cam = thermal_intrinsics or CameraIntrinsics.from_fov(320, 240, 1.047)
        self.t_rgb_to_th = translation_rgb_to_th
        self.R_rgb_to_th = rotation_rgb_to_th

        # Configuration initialization with keyword overrides for backward compatibility
        self.config = config or ThermalFeatureConfig()
        if min_rgb_confidence is not None:
            self.config.min_rgb_confidence = min_rgb_confidence
        if min_depth_m is not None:
            self.config.min_depth_m = min_depth_m
        if max_depth_m is not None:
            self.config.max_depth_m = max_depth_m
        if min_victim_temp_k is not None:
            self.config.min_victim_temp_k = min_victim_temp_k
        if max_human_temp_k is not None:
            self.config.max_human_temp_k = max_human_temp_k
        if min_thermal_delta_k is not None:
            self.config.min_thermal_delta_k = min_thermal_delta_k
        if min_physical_size_m is not None:
            self.config.min_physical_size_m = min_physical_size_m
        if max_physical_size_m is not None:
            self.config.max_physical_size_m = max_physical_size_m

        # Convenience aliases
        self.min_rgb_confidence = self.config.min_rgb_confidence
        self.min_depth_m = self.config.min_depth_m
        self.max_depth_m = self.config.max_depth_m
        self.min_victim_temp_k = self.config.min_victim_temp_k
        self.max_human_temp_k = self.config.max_human_temp_k
        self.min_thermal_delta_k = self.config.min_thermal_delta_k
        self.min_physical_size_m = self.config.min_physical_size_m
        self.max_physical_size_m = self.config.max_physical_size_m

    def update_rgb_intrinsics(self, fx: float, fy: float, cx: float, cy: float, w: int = 640, h: int = 480):
        self.rgb_cam = CameraIntrinsics(fx=fx, fy=fy, cx=cx, cy=cy, width=w, height=h, hfov_rad=self.rgb_cam.hfov_rad)

    def update_thermal_intrinsics(self, fx: float, fy: float, cx: float, cy: float, w: int = 320, h: int = 240):
        self.th_cam = CameraIntrinsics(fx=fx, fy=fy, cx=cx, cy=cy, width=w, height=h, hfov_rad=self.th_cam.hfov_rad)

    @staticmethod
    def decode_thermal_mono16(raw_buffer: np.ndarray, is_bigendian: bool = False) -> np.ndarray:
        """
        Decodes a 16-bit radiometric thermal image buffer into Kelvin temperatures.
        Gazebo Harmonic format L16: Raw Count = 100 * Temperature_K.
        """
        if raw_buffer.dtype not in [np.uint16, np.dtype('>u2'), np.dtype('<u2')]:
            dtype = np.dtype('>u2') if is_bigendian else np.dtype('<u2')
            arr = np.frombuffer(raw_buffer, dtype=dtype)
        else:
            arr = raw_buffer
        return arr.astype(np.float32) / 100.0

    def project_rgb_point_to_thermal(self, u_rgb: float, v_rgb: float, depth_m: float) -> Tuple[float, float]:
        """
        Rigorous 3D pinhole backprojection from RGB to Thermal camera coordinates.
        Accounts for focal lengths, principal points, depth, and complete rigid-body transformation:
            P_th = R_rgb_to_th * (P_rgb - t_rgb_to_th)
        """
        if depth_m <= 0 or math.isnan(depth_m) or math.isinf(depth_m):
            raise ValueError(f"Depth must be positive finite number, got {depth_m}")

        x_c = (u_rgb - self.rgb_cam.cx) * depth_m / self.rgb_cam.fx
        y_c = (v_rgb - self.rgb_cam.cy) * depth_m / self.rgb_cam.fy
        z_c = depth_m

        dx, dy, dz = self.t_rgb_to_th
        p_shifted = np.array([x_c - dx, y_c - dy, z_c - dz], dtype=np.float64)

        if self.R_rgb_to_th is not None:
            p_th = self.R_rgb_to_th @ p_shifted
        else:
            p_th = p_shifted

        x_th, y_th, z_th = float(p_th[0]), float(p_th[1]), float(p_th[2])
        if z_th <= 0:
            raise ValueError(f"Projected thermal point has non-positive depth: {z_th}")

        u_th = (self.th_cam.fx * x_th / z_th) + self.th_cam.cx
        v_th = (self.th_cam.fy * y_th / z_th) + self.th_cam.cy
        return float(u_th), float(v_th)

    def project_rgb_bbox_to_thermal(
        self,
        bbox_rgb: List[int],
        depth_m: float
    ) -> Tuple[Optional[List[int]], bool, bool]:
        """
        Projects an RGB bounding box [x1, y1, x2, y2] into thermal pixel coordinates.
        Returns:
            (projected_bbox_th, is_fully_inside_fov, is_partially_inside_fov)
        """
        if depth_m <= 0:
            return None, False, False

        x1, y1, x2, y2 = bbox_rgb
        corners_rgb = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
        th_pts = [self.project_rgb_point_to_thermal(u, v, depth_m) for u, v in corners_rgb]

        th_xs = [p[0] for p in th_pts]
        th_ys = [p[1] for p in th_pts]

        u_min = int(round(min(th_xs)))
        u_max = int(round(max(th_xs)))
        v_min = int(round(min(th_ys)))
        v_max = int(round(max(th_ys)))

        w_th = self.th_cam.width
        h_th = self.th_cam.height

        fully_inside = (0 <= u_min < w_th and 0 <= u_max < w_th and
                        0 <= v_min < h_th and 0 <= v_max < h_th)
        partially_inside = (u_max >= 0 and u_min < w_th and
                            v_max >= 0 and v_min < h_th)

        clamped_bbox = [
            max(0, min(w_th - 1, u_min)),
            max(0, min(h_th - 1, v_min)),
            max(0, min(w_th, u_max)),
            max(0, min(h_th, v_max))
        ]

        if clamped_bbox[2] <= clamped_bbox[0] or clamped_bbox[3] <= clamped_bbox[1]:
            return None, False, False

        return clamped_bbox, fully_inside, partially_inside

    def analyze_thermal_roi(
        self,
        temp_k_image: np.ndarray,
        bbox_th: List[int],
        phys_dims_m: Tuple[float, float] = (0.0, 0.0),
        rgb_confidence: float = 0.80,
        annulus_margin_fraction: float = 0.5
    ) -> Optional[ThermalStats]:
        """
        Executes comprehensive multi-feature thermal analysis on projected ROI:
          1. Temperature percentiles (P10, P50, P75, P90, P95) & stats (min, max, mean, std)
          2. Surrounding background annulus statistics (mean, median, std)
          3. Contrast & SNR (delta_k, delta_mean, snr)
          4. Spatial temperature gradients (Sobel magnitude mean & 90th percentile)
          5. Core-to-periphery distribution (core delta & contrast ratio)
          6. Connected-component analysis (occupancy, primary hotspot area fraction, centroid offset)
          7. Physical dimension aspect ratio
          8. Multispectral Verification Score (MVS) & explicit uncertainty quantification
        """
        u1, v1, u2, v2 = bbox_th
        h_img, w_img = temp_k_image.shape[:2]

        if u1 >= u2 or v1 >= v2 or u1 >= w_img or v1 >= h_img or u2 <= 0 or v2 <= 0:
            return None

        target_roi = temp_k_image[v1:v2, u1:u2]
        if target_roi.size == 0:
            return None

        # Filter NaNs if any
        valid_pixels = target_roi[~np.isnan(target_roi)]
        if valid_pixels.size == 0:
            return None

        min_k = float(np.min(valid_pixels))
        max_k = float(np.max(valid_pixels))
        mean_k = float(np.mean(valid_pixels))
        median_k = float(np.median(valid_pixels))
        std_k = float(np.std(valid_pixels))

        p_vals = np.percentile(valid_pixels, [10, 50, 75, 90, 95])
        p10_k = float(p_vals[0])
        p50_k = float(p_vals[1])
        p75_k = float(p_vals[2])
        p90_k = float(p_vals[3])
        p95_k = float(p_vals[4])

        # Local background annular ring around target
        bw = u2 - u1
        bh = v2 - v1
        mw = max(2, int(bw * annulus_margin_fraction))
        mh = max(2, int(bh * annulus_margin_fraction))

        bg_u1 = max(0, u1 - mw)
        bg_u2 = min(w_img, u2 + mw)
        bg_v1 = max(0, v1 - mh)
        bg_v2 = min(h_img, v2 + mh)

        expanded_roi = temp_k_image[bg_v1:bg_v2, bg_u1:bg_u2]
        mask = np.ones(expanded_roi.shape, dtype=bool)
        inner_v1 = v1 - bg_v1
        inner_v2 = v2 - bg_v1
        inner_u1 = u1 - bg_u1
        inner_u2 = u2 - bg_u1
        mask[inner_v1:inner_v2, inner_u1:inner_u2] = False

        bg_pixels = expanded_roi[mask]
        bg_pixels = bg_pixels[~np.isnan(bg_pixels)]
        if bg_pixels.size > 0:
            bg_mean_k = float(np.mean(bg_pixels))
            bg_median_k = float(np.median(bg_pixels))
            bg_std_k = float(np.std(bg_pixels))
        else:
            bg_mean_k = float(np.nanmedian(temp_k_image))
            bg_median_k = bg_mean_k
            bg_std_k = 0.5

        delta_k = p90_k - bg_mean_k
        delta_mean_k = mean_k - bg_mean_k
        snr = delta_k / (bg_std_k + 1e-4)

        # Spatial temperature gradient
        if target_roi.shape[0] >= 3 and target_roi.shape[1] >= 3:
            gx = cv2.Sobel(target_roi.astype(np.float32), cv2.CV_32F, 1, 0, ksize=3)
            gy = cv2.Sobel(target_roi.astype(np.float32), cv2.CV_32F, 0, 1, ksize=3)
            grad_mag = np.sqrt(gx**2 + gy**2)
            grad_mean = float(np.mean(grad_mag))
            grad_p90 = float(np.percentile(grad_mag, 90))
        else:
            grad_mean = float(std_k)
            grad_p90 = float(std_k)

        # Core-to-periphery distribution (central 50% box)
        cw = max(2, int(bw * 0.50))
        ch = max(2, int(bh * 0.50))
        cu1 = (bw - cw) // 2
        cv1 = (bh - ch) // 2
        core_roi = target_roi[cv1:cv1+ch, cu1:cu1+cw]
        core_mask = np.zeros(target_roi.shape, dtype=bool)
        core_mask[cv1:cv1+ch, cu1:cu1+cw] = True
        periph_pixels = target_roi[~core_mask]
        periph_pixels = periph_pixels[~np.isnan(periph_pixels)]

        core_valid = core_roi[~np.isnan(core_roi)]
        core_mean_k = float(np.mean(core_valid)) if core_valid.size > 0 else mean_k
        periph_mean_k = float(np.mean(periph_pixels)) if periph_pixels.size > 0 else core_mean_k
        core_periphery_delta_k = core_mean_k - periph_mean_k
        core_periphery_ratio = (core_mean_k - bg_mean_k) / max(0.1, periph_mean_k - bg_mean_k)

        # Connected-component & occupancy
        hot_thresh = bg_mean_k + max(1.5, 0.35 * max(0.0, delta_k))
        hot_mask = (target_roi >= hot_thresh).astype(np.uint8)
        hot_pixel_fraction = float(np.mean(hot_mask))

        primary_comp_fraction = 0.0
        centroid_offset_norm = 1.0

        if np.any(hot_mask > 0):
            n_labels, labels, cc_stats, centroids = cv2.connectedComponentsWithStats(hot_mask, connectivity=8)
            if n_labels > 1:
                areas = cc_stats[1:, cv2.CC_STAT_AREA]
                largest_idx = 1 + int(np.argmax(areas))
                largest_area = cc_stats[largest_idx, cv2.CC_STAT_AREA]
                primary_comp_fraction = float(largest_area / target_roi.size)
                cx_comp, cy_comp = centroids[largest_idx]
                dist_c = math.hypot(cx_comp - bw / 2.0, cy_comp - bh / 2.0)
                diag = max(1.0, math.hypot(bw, bh) / 2.0)
                centroid_offset_norm = float(dist_c / diag)

        # Physical dimensions & aspect ratio
        phys_w, phys_h = phys_dims_m
        aspect_ratio = max(phys_w, phys_h) / max(0.01, min(phys_w, phys_h)) if (phys_w > 0 and phys_h > 0) else 0.0

        # Multispectral Verification Score (MVS)
        s_rgb = min(1.0, max(0.0, rgb_confidence))
        s_temp = math.exp(-((p90_k - 308.15)**2) / (2.0 * (6.0**2)))
        s_contrast = min(1.0, max(0.0, delta_k / 10.0))
        s_core = min(1.0, max(0.0, core_periphery_delta_k / 2.0))
        s_occ = 1.0 - min(1.0, abs(hot_pixel_fraction - 0.45) / 0.45)

        mvs_score = float(0.30 * s_rgb + 0.25 * s_temp + 0.20 * s_contrast + 0.15 * s_core + 0.10 * max(0.0, s_occ))

        # Explicit uncertainty quantification
        uncertainty_flags = []
        u_penalty = 0.0
        if delta_k < 4.5:
            uncertainty_flags.append("LOW_THERMAL_CONTRAST")
            u_penalty += 0.25
        if p90_k < 298.0:
            uncertainty_flags.append("NEAR_AMBIENT_TEMPERATURE")
            u_penalty += 0.25
        if grad_mean < 0.25:
            uncertainty_flags.append("DIFFUSE_BOUNDARY_GRADIENT")
            u_penalty += 0.20
        if core_periphery_delta_k < 0.3:
            uncertainty_flags.append("WEAK_CORE_CONCENTRATION")
            u_penalty += 0.20
        if hot_pixel_fraction > 0.85:
            uncertainty_flags.append("DIFFUSE_UNIFORM_HEATING")
            u_penalty += 0.30
        if rgb_confidence < 0.55:
            uncertainty_flags.append("MARGINAL_RGB_CONFIDENCE")
            u_penalty += 0.20

        is_saturated = (max_k >= self.config.max_sensor_saturation_k and std_k < 0.05)
        if is_saturated:
            uncertainty_flags.append("SENSOR_SATURATION")
            u_penalty += 0.50

        uncertainty_score = float(min(1.0, u_penalty))

        return ThermalStats(
            min_k=min_k,
            max_k=max_k,
            mean_k=mean_k,
            median_k=median_k,
            std_k=std_k,
            p90_k=p90_k,
            bg_mean_k=bg_mean_k,
            bg_std_k=bg_std_k,
            delta_k=delta_k,
            snr=snr,
            p10_k=p10_k,
            p50_k=p50_k,
            p75_k=p75_k,
            p95_k=p95_k,
            bg_median_k=bg_median_k,
            delta_mean_k=delta_mean_k,
            grad_mean=grad_mean,
            grad_p90=grad_p90,
            core_mean_k=core_mean_k,
            periphery_mean_k=periph_mean_k,
            core_periphery_delta_k=core_periphery_delta_k,
            core_periphery_ratio=core_periphery_ratio,
            hot_pixel_fraction=hot_pixel_fraction,
            primary_component_fraction=primary_comp_fraction,
            centroid_offset_norm=centroid_offset_norm,
            physical_width_m=phys_w,
            physical_height_m=phys_h,
            aspect_ratio=aspect_ratio,
            rgb_confidence=rgb_confidence,
            mvs_score=mvs_score,
            uncertainty_score=uncertainty_score,
            uncertainty_flags=uncertainty_flags
        )

    def verify_candidate(
        self,
        candidate_id: str,
        drone_id: int,
        timestamp: float,
        rgb_bbox: List[int],
        rgb_confidence: float,
        depth_image: Optional[np.ndarray],
        thermal_image_k: Optional[np.ndarray],
        drone_pose: Optional[Tuple[float, float, float, float]] = None
    ) -> CandidateVerificationResult:
        """
        Executes multi-feature multispectral candidate verification pipeline.
        Returns structured verification result with distinct evidence states.
        """
        x1, y1, x2, y2 = rgb_bbox
        w_px = x2 - x1
        h_px = y2 - y1

        # 1. RGB Confidence Check
        if rgb_confidence < self.config.min_rgb_confidence:
            return CandidateVerificationResult(
                candidate_id=candidate_id,
                drone_id=drone_id,
                timestamp=timestamp,
                rgb_confidence=rgb_confidence,
                rgb_bbox=rgb_bbox,
                projected_thermal_bbox=None,
                depth_m=-1.0,
                physical_dims_m=(0.0, 0.0),
                thermal_stats=None,
                status=VerificationStatus.REJECTED.value,
                rejection_reason=RejectionReason.LOW_RGB_CONFIDENCE.value,
                evidence_state=VerificationStatus.REJECTED.value
            )

        # 2. Depth Sensor Presence Check
        if depth_image is None or depth_image.size == 0:
            return CandidateVerificationResult(
                candidate_id=candidate_id,
                drone_id=drone_id,
                timestamp=timestamp,
                rgb_confidence=rgb_confidence,
                rgb_bbox=rgb_bbox,
                projected_thermal_bbox=None,
                depth_m=-1.0,
                physical_dims_m=(0.0, 0.0),
                thermal_stats=None,
                status=VerificationStatus.REJECTED.value,
                rejection_reason=RejectionReason.MISSING_SENSOR_DATA.value,
                evidence_state=VerificationStatus.REJECTED.value
            )

        # Central 30% patch for robust depth
        cx_px = int(x1 + w_px / 2.0)
        cy_px = int(y1 + h_px / 2.0)
        pw = max(2, int(w_px * 0.30))
        ph = max(2, int(h_px * 0.30))
        px1 = max(0, cx_px - pw // 2)
        px2 = min(depth_image.shape[1], cx_px + pw // 2)
        py1 = max(0, cy_px - ph // 2)
        py2 = min(depth_image.shape[0], cy_px + ph // 2)

        depth_patch = depth_image[py1:py2, px1:px2]
        valid_mask = (depth_patch > 0) & (~np.isnan(depth_patch)) & (~np.isinf(depth_patch))
        valid_samples = depth_patch[valid_mask]

        if len(valid_samples) < 10:
            return CandidateVerificationResult(
                candidate_id=candidate_id,
                drone_id=drone_id,
                timestamp=timestamp,
                rgb_confidence=rgb_confidence,
                rgb_bbox=rgb_bbox,
                projected_thermal_bbox=None,
                depth_m=-1.0,
                physical_dims_m=(0.0, 0.0),
                thermal_stats=None,
                status=VerificationStatus.REJECTED.value,
                rejection_reason=RejectionReason.INVALID_DEPTH.value,
                evidence_state=VerificationStatus.REJECTED.value
            )

        median_depth = float(np.median(valid_samples))
        depth_spread = float(np.std(valid_samples))

        if not (self.config.min_depth_m <= median_depth <= self.config.max_depth_m) or depth_spread > 3.0:
            return CandidateVerificationResult(
                candidate_id=candidate_id,
                drone_id=drone_id,
                timestamp=timestamp,
                rgb_confidence=rgb_confidence,
                rgb_bbox=rgb_bbox,
                projected_thermal_bbox=None,
                depth_m=median_depth,
                physical_dims_m=(0.0, 0.0),
                thermal_stats=None,
                status=VerificationStatus.REJECTED.value,
                rejection_reason=RejectionReason.INVALID_DEPTH.value,
                evidence_state=VerificationStatus.REJECTED.value
            )

        # 3. Geometric Plausibility Check (metric dimensions on ground)
        phys_w = (w_px / self.rgb_cam.fx) * median_depth
        phys_h = (h_px / self.rgb_cam.fy) * median_depth
        max_dim = max(phys_w, phys_h)
        min_dim = min(phys_w, phys_h)

        if max_dim > self.config.max_physical_size_m or min_dim < self.config.min_physical_size_m:
            return CandidateVerificationResult(
                candidate_id=candidate_id,
                drone_id=drone_id,
                timestamp=timestamp,
                rgb_confidence=rgb_confidence,
                rgb_bbox=rgb_bbox,
                projected_thermal_bbox=None,
                depth_m=median_depth,
                physical_dims_m=(phys_w, phys_h),
                thermal_stats=None,
                status=VerificationStatus.REJECTED.value,
                rejection_reason=RejectionReason.GEOMETRIC_IMPLAUSIBLE.value,
                evidence_state=VerificationStatus.REJECTED.value
            )

        # 4. Fallback Mode: Thermal image absent (RGB-D-only baseline)
        if thermal_image_k is None or thermal_image_k.size == 0:
            return CandidateVerificationResult(
                candidate_id=candidate_id,
                drone_id=drone_id,
                timestamp=timestamp,
                rgb_confidence=rgb_confidence,
                rgb_bbox=rgb_bbox,
                projected_thermal_bbox=None,
                depth_m=median_depth,
                physical_dims_m=(phys_w, phys_h),
                thermal_stats=None,
                status=VerificationStatus.RGB_CANDIDATE.value,
                rejection_reason=RejectionReason.MISSING_SENSOR_DATA.value,
                evidence_state=VerificationStatus.RGB_CANDIDATE.value
            )

        # 5. Project RGB BBox into Thermal Image Coordinates
        bbox_th, fully_inside, partially_inside = self.project_rgb_bbox_to_thermal(rgb_bbox, median_depth)
        if not partially_inside or bbox_th is None:
            return CandidateVerificationResult(
                candidate_id=candidate_id,
                drone_id=drone_id,
                timestamp=timestamp,
                rgb_confidence=rgb_confidence,
                rgb_bbox=rgb_bbox,
                projected_thermal_bbox=None,
                depth_m=median_depth,
                physical_dims_m=(phys_w, phys_h),
                thermal_stats=None,
                status=VerificationStatus.RGB_CANDIDATE.value,
                rejection_reason=RejectionReason.OUT_OF_THERMAL_FOV.value,
                evidence_state=VerificationStatus.RGB_CANDIDATE.value
            )

        # 6. Multi-Feature Thermal Analysis
        stats = self.analyze_thermal_roi(
            thermal_image_k,
            bbox_th,
            phys_dims_m=(phys_w, phys_h),
            rgb_confidence=rgb_confidence
        )
        if stats is None:
            reason = (
                RejectionReason.SATURATED_OR_CORRUPT_FRAME.value
                if (thermal_image_k is not None and np.all(np.isnan(thermal_image_k)))
                else RejectionReason.OUT_OF_THERMAL_FOV.value
            )
            return CandidateVerificationResult(
                candidate_id=candidate_id,
                drone_id=drone_id,
                timestamp=timestamp,
                rgb_confidence=rgb_confidence,
                rgb_bbox=rgb_bbox,
                projected_thermal_bbox=bbox_th,
                depth_m=median_depth,
                physical_dims_m=(phys_w, phys_h),
                thermal_stats=None,
                status=VerificationStatus.REJECTED.value,
                rejection_reason=reason,
                evidence_state=VerificationStatus.REJECTED.value
            )

        # 7. Saturated or Corrupt Thermal Frame Check
        if "SENSOR_SATURATION" in stats.uncertainty_flags:
            return CandidateVerificationResult(
                candidate_id=candidate_id,
                drone_id=drone_id,
                timestamp=timestamp,
                rgb_confidence=rgb_confidence,
                rgb_bbox=rgb_bbox,
                projected_thermal_bbox=bbox_th,
                depth_m=median_depth,
                physical_dims_m=(phys_w, phys_h),
                thermal_stats=asdict(stats),
                status=VerificationStatus.REJECTED.value,
                rejection_reason=RejectionReason.SATURATED_OR_CORRUPT_FRAME.value,
                evidence_state=VerificationStatus.REJECTED.value,
                verification_score=stats.mvs_score,
                uncertainty_score=stats.uncertainty_score
            )

        # 8. Hot Clutter Rejection (> 45°C = 318.15 K)
        if stats.max_k > self.config.max_human_temp_k:
            return CandidateVerificationResult(
                candidate_id=candidate_id,
                drone_id=drone_id,
                timestamp=timestamp,
                rgb_confidence=rgb_confidence,
                rgb_bbox=rgb_bbox,
                projected_thermal_bbox=bbox_th,
                depth_m=median_depth,
                physical_dims_m=(phys_w, phys_h),
                thermal_stats=asdict(stats),
                status=VerificationStatus.REJECTED.value,
                rejection_reason=RejectionReason.HOT_NON_HUMAN_CLUTTER.value,
                evidence_state=VerificationStatus.REJECTED.value,
                verification_score=stats.mvs_score,
                uncertainty_score=stats.uncertainty_score
            )

        # 9. Cold Shadow & Obstacle Rejection
        if stats.max_k < self.config.min_victim_temp_k or stats.delta_k < self.config.min_thermal_delta_k:
            return CandidateVerificationResult(
                candidate_id=candidate_id,
                drone_id=drone_id,
                timestamp=timestamp,
                rgb_confidence=rgb_confidence,
                rgb_bbox=rgb_bbox,
                projected_thermal_bbox=bbox_th,
                depth_m=median_depth,
                physical_dims_m=(phys_w, phys_h),
                thermal_stats=asdict(stats),
                status=VerificationStatus.REJECTED.value,
                rejection_reason=RejectionReason.COLD_SHADOW_OR_OBSTACLE.value,
                evidence_state=VerificationStatus.REJECTED.value,
                verification_score=stats.mvs_score,
                uncertainty_score=stats.uncertainty_score
            )

        # 10. Uniform Diffuse Heat Rejection (sun-heated concrete, road asphalt)
        if (stats.hot_pixel_fraction > self.config.max_thermal_occupancy and
                stats.core_periphery_delta_k < self.config.min_core_periphery_delta_k and
                stats.grad_mean < self.config.min_spatial_gradient):
            return CandidateVerificationResult(
                candidate_id=candidate_id,
                drone_id=drone_id,
                timestamp=timestamp,
                rgb_confidence=rgb_confidence,
                rgb_bbox=rgb_bbox,
                projected_thermal_bbox=bbox_th,
                depth_m=median_depth,
                physical_dims_m=(phys_w, phys_h),
                thermal_stats=asdict(stats),
                status=VerificationStatus.REJECTED.value,
                rejection_reason=RejectionReason.UNIFORM_DIFFUSE_HEAT.value,
                evidence_state=VerificationStatus.REJECTED.value,
                verification_score=stats.mvs_score,
                uncertainty_score=stats.uncertainty_score
            )

        # 11. Non-Human Thermal Morphology (isothermal sun-heated rocks or warm objects)
        if (stats.core_periphery_delta_k <= 0.0 or
                (stats.core_periphery_delta_k < 0.20 and stats.p90_k < 304.0 and rgb_confidence < 0.65)):
            return CandidateVerificationResult(
                candidate_id=candidate_id,
                drone_id=drone_id,
                timestamp=timestamp,
                rgb_confidence=rgb_confidence,
                rgb_bbox=rgb_bbox,
                projected_thermal_bbox=bbox_th,
                depth_m=median_depth,
                physical_dims_m=(phys_w, phys_h),
                thermal_stats=asdict(stats),
                status=VerificationStatus.REJECTED.value,
                rejection_reason=RejectionReason.NON_HUMAN_THERMAL_MORPHOLOGY.value,
                evidence_state=VerificationStatus.REJECTED.value,
                verification_score=stats.mvs_score,
                uncertainty_score=stats.uncertainty_score
            )

        # 12. Implausible Thermal Occupancy (specks / noise)
        if stats.hot_pixel_fraction < self.config.min_thermal_occupancy:
            return CandidateVerificationResult(
                candidate_id=candidate_id,
                drone_id=drone_id,
                timestamp=timestamp,
                rgb_confidence=rgb_confidence,
                rgb_bbox=rgb_bbox,
                projected_thermal_bbox=bbox_th,
                depth_m=median_depth,
                physical_dims_m=(phys_w, phys_h),
                thermal_stats=asdict(stats),
                status=VerificationStatus.REJECTED.value,
                rejection_reason=RejectionReason.IMPLAUSIBLE_THERMAL_OCCUPANCY.value,
                evidence_state=VerificationStatus.REJECTED.value,
                verification_score=stats.mvs_score,
                uncertainty_score=stats.uncertainty_score
            )

        # 13. Multispectral Verification Score (MVS) & Uncertainty Gate
        if stats.mvs_score < self.config.min_mvs_score or stats.uncertainty_score > self.config.max_uncertainty_score:
            return CandidateVerificationResult(
                candidate_id=candidate_id,
                drone_id=drone_id,
                timestamp=timestamp,
                rgb_confidence=rgb_confidence,
                rgb_bbox=rgb_bbox,
                projected_thermal_bbox=bbox_th,
                depth_m=median_depth,
                physical_dims_m=(phys_w, phys_h),
                thermal_stats=asdict(stats),
                status=VerificationStatus.REJECTED.value,
                rejection_reason=RejectionReason.LOW_MULTISPECTRAL_SCORE.value,
                evidence_state=VerificationStatus.REJECTED.value,
                verification_score=stats.mvs_score,
                uncertainty_score=stats.uncertainty_score
            )

        # 14. World Coordinates and Search Bounds
        world_x, world_y = None, None
        grid_pos = None
        if drone_pose is not None:
            dx, dy, dz, yaw = drone_pose
            x_c = (cx_px - self.rgb_cam.cx) * median_depth / self.rgb_cam.fx
            y_c = (cy_px - self.rgb_cam.cy) * median_depth / self.rgb_cam.fy

            # Downward facing camera frame rotation
            x_d = -y_c
            y_d = -x_c
            cos_y = math.cos(yaw)
            sin_y = math.sin(yaw)
            world_x = dx + (x_d * cos_y - y_d * sin_y)
            world_y = dy + (x_d * sin_y + y_d * cos_y)

            if not GridWorldTransform.is_within_world_bounds(world_x, world_y):
                return CandidateVerificationResult(
                    candidate_id=candidate_id,
                    drone_id=drone_id,
                    timestamp=timestamp,
                    rgb_confidence=rgb_confidence,
                    rgb_bbox=rgb_bbox,
                    projected_thermal_bbox=bbox_th,
                    depth_m=median_depth,
                    physical_dims_m=(phys_w, phys_h),
                    thermal_stats=asdict(stats),
                    status=VerificationStatus.REJECTED.value,
                    rejection_reason=RejectionReason.OUT_OF_WORLD_BOUNDS.value,
                    evidence_state=VerificationStatus.REJECTED.value,
                    world_pos=(world_x, world_y),
                    verification_score=stats.mvs_score,
                    uncertainty_score=stats.uncertainty_score
                )
            grid_pos = GridWorldTransform.world_to_grid(world_x, world_y)

        # 15. SUCCESS: Multispectral criteria verified -> THERMAL_SUPPORTED
        return CandidateVerificationResult(
            candidate_id=candidate_id,
            drone_id=drone_id,
            timestamp=timestamp,
            rgb_confidence=rgb_confidence,
            rgb_bbox=rgb_bbox,
            projected_thermal_bbox=bbox_th,
            depth_m=median_depth,
            physical_dims_m=(phys_w, phys_h),
            thermal_stats=asdict(stats),
            status=VerificationStatus.THERMAL_SUPPORTED.value,
            rejection_reason=RejectionReason.NONE.value,
            world_pos=(world_x, world_y) if world_x is not None else None,
            grid_pos=grid_pos,
            evidence_state=VerificationStatus.THERMAL_SUPPORTED.value,
            verification_score=stats.mvs_score,
            uncertainty_score=stats.uncertainty_score
        )


class RGBThermalFusionVerifierNode(Node):
    """
    ROS 2 Perception Node implementing multi-drone RGB-Thermal verification.
    Subscribes to 6-drone perception streams without round-robin throttling.
    """

    def __init__(self):
        super().__init__('rgb_thermal_fusion_verifier')
        self.bridge = CvBridge()
        self.num_drones = 6

        # Configurable Parameters
        self.declare_parameter('min_confidence', 0.35)
        self.declare_parameter('min_victim_temp_k', 296.0)
        self.declare_parameter('max_human_temp_k', 318.15)
        self.declare_parameter('min_thermal_delta_k', 3.0)
        self.declare_parameter('publish_debug_images', True)
        self.declare_parameter('sync_tolerance_sec', 0.15)
        self.declare_parameter('min_core_periphery_delta_k', 0.30)
        self.declare_parameter('min_spatial_gradient', 0.20)
        self.declare_parameter('max_thermal_occupancy', 0.90)
        self.declare_parameter('min_thermal_occupancy', 0.05)
        self.declare_parameter('min_mvs_score', 0.45)

        feature_config = ThermalFeatureConfig(
            min_rgb_confidence=float(self.get_parameter('min_confidence').value),
            min_victim_temp_k=float(self.get_parameter('min_victim_temp_k').value),
            max_human_temp_k=float(self.get_parameter('max_human_temp_k').value),
            min_thermal_delta_k=float(self.get_parameter('min_thermal_delta_k').value),
            min_core_periphery_delta_k=float(self.get_parameter('min_core_periphery_delta_k').value),
            min_spatial_gradient=float(self.get_parameter('min_spatial_gradient').value),
            max_thermal_occupancy=float(self.get_parameter('max_thermal_occupancy').value),
            min_thermal_occupancy=float(self.get_parameter('min_thermal_occupancy').value),
            min_mvs_score=float(self.get_parameter('min_mvs_score').value)
        )

        self.publish_debug = self.get_parameter('publish_debug_images').value
        self.sync_tolerance_sec = float(self.get_parameter('sync_tolerance_sec').value)

        self.core = RGBThermalFusionCore(config=feature_config)

        # Rolling buffers per drone (keeps last 50 frames = ~3.3s of 15 Hz thermal history)
        self.depth_buffers: Dict[int, deque] = {i: deque(maxlen=50) for i in range(self.num_drones)}
        self.thermal_buffers: Dict[int, deque] = {i: deque(maxlen=50) for i in range(self.num_drones)}
        self.drone_poses: Dict[int, Optional[Tuple[float, float, float, float]]] = {i: None for i in range(self.num_drones)}

        # Publishers
        self.verified_pubs: Dict[int, rclpy.publisher.Publisher] = {}
        self.debug_image_pubs: Dict[int, rclpy.publisher.Publisher] = {}
        self.discovery_pub = self.create_publisher(String, '/sar/victim_discovery', 10)
        # Configure grid-world transform from metadata if provided
        meta_file = os.environ.get("WORLD_META_PATH")
        if meta_file and os.path.exists(meta_file):
            try:
                with open(meta_file, 'r') as f:
                    meta = json.load(f)
                grid_meta = meta.get('grid', {})
                world_meta = meta.get('world', {})
                width = grid_meta.get('width', 25)
                height = grid_meta.get('height', 25)
                meters_per_cell = grid_meta.get('meters_per_cell', 8.0)
                origin_x = world_meta.get('origin_x', -180.0)
                origin_y = world_meta.get('origin_y', -20.0)
                GridWorldTransform.set_bounds(
                    width=width,
                    height=height,
                    meters_per_cell=meters_per_cell,
                    origin_x=origin_x,
                    origin_y=origin_y
                )
                self.get_logger().info(f"Configured GridWorldTransform from {meta_file}: origin=({origin_x},{origin_y}) cell={meters_per_cell}m dims={width}x{height}")
            except Exception as e:
                self.get_logger().warn(f"Failed to load world bounds from {meta_file}: {e}")

        # Temporal consensus tracker (canonical TemporalEventConfirmer)
        self.confirmer = TemporalEventConfirmer(
            window_size=6,
            consensus_needed=4,
            proximity_threshold=3.0,
            grid_cell_size=GridWorldTransform.METERS_PER_CELL,
            min_confidence=feature_config.min_rgb_confidence,
            min_time_span=0.4,
            dedup_radius=6.0
        )
        self.confirmed_victims = self.confirmer.confirmed_victims
        self.tracks = self.confirmer.tracks
        self.next_track_id = self.confirmer.next_track_id
        self.consensus_needed = self.confirmer.consensus_needed
        self.window_size = self.confirmer.window_size
        self.min_time_span = self.confirmer.min_time_span
        self.proximity_threshold = self.confirmer.proximity_threshold
        self.dedup_radius = self.confirmer.dedup_radius

        # Subscriptions for all 6 drones
        self.subs = []
        for i in range(self.num_drones):
            s_det = self.create_subscription(
                String,
                f'/drone_{i}/perception/detections',
                lambda msg, d=i: self.detection_callback(msg, d),
                10
            )
            s_dep = self.create_subscription(
                Image,
                f'/drone_{i}/camera/depth_image',
                lambda msg, d=i: self.depth_callback(msg, d),
                qos_profile_sensor_data
            )
            s_th = self.create_subscription(
                Image,
                f'/drone_{i}/thermal/image_raw',
                lambda msg, d=i: self.thermal_callback(msg, d),
                qos_profile_sensor_data
            )
            s_ci = self.create_subscription(
                CameraInfo,
                f'/drone_{i}/camera/camera_info',
                lambda msg, d=i: self.rgb_info_callback(msg, d),
                qos_profile_sensor_data
            )
            s_ti = self.create_subscription(
                CameraInfo,
                f'/drone_{i}/thermal/camera_info',
                lambda msg, d=i: self.thermal_info_callback(msg, d),
                qos_profile_sensor_data
            )
            s_pose = self.create_subscription(
                PoseStamped,
                f'/model/drone_{i}/pose',
                lambda msg, d=i: self.pose_callback(msg, d),
                qos_profile_sensor_data
            )

            self.subs.extend([s_det, s_dep, s_th, s_ci, s_ti, s_pose])

            self.verified_pubs[i] = self.create_publisher(
                String,
                f'/drone_{i}/perception/verified_detections',
                10
            )
            self.debug_image_pubs[i] = self.create_publisher(
                Image,
                f'/drone_{i}/thermal/detection_image',
                qos_profile_sensor_data
            )

        self.get_logger().info(
            f"RGB-Thermal Fusion Verifier initialized for {self.num_drones} drones with Multi-Feature Thermal Analysis. "
            f"Thresholds: min_T={feature_config.min_victim_temp_k}K, max_T={feature_config.max_human_temp_k}K, "
            f"min_delta={feature_config.min_thermal_delta_k}K, min_mvs={feature_config.min_mvs_score}"
        )

    def rgb_info_callback(self, msg: CameraInfo, drone_id: int):
        self.core.update_rgb_intrinsics(msg.k[0], msg.k[4], msg.k[2], msg.k[5], msg.width, msg.height)

    def thermal_info_callback(self, msg: CameraInfo, drone_id: int):
        self.core.update_thermal_intrinsics(msg.k[0], msg.k[4], msg.k[2], msg.k[5], msg.width, msg.height)

    def pose_callback(self, msg: PoseStamped, drone_id: int):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        q = msg.pose.orientation
        euler = quat2euler([q.w, q.x, q.y, q.z])
        self.drone_poses[drone_id] = (x, y, z, euler[2])

    def depth_callback(self, msg: Image, drone_id: int):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.depth_buffers[drone_id].append((ts, msg))

    def thermal_callback(self, msg: Image, drone_id: int):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.thermal_buffers[drone_id].append((ts, msg))

    def _get_synced_frames(self, drone_id: int, target_ts: float, max_tol: Optional[float] = None):
        tol = max_tol if max_tol is not None else self.sync_tolerance_sec
        depth_msg = None
        thermal_msg = None

        if self.depth_buffers[drone_id]:
            closest_d = min(self.depth_buffers[drone_id], key=lambda x: abs(x[0] - target_ts))
            diff_d = abs(closest_d[0] - target_ts)
            if diff_d <= tol:
                depth_msg = closest_d[1]
            else:
                self.get_logger().debug(f"Drone {drone_id} depth sync delta {diff_d:.3f}s > {tol:.3f}s tol")

        if self.thermal_buffers[drone_id]:
            closest_th = min(self.thermal_buffers[drone_id], key=lambda x: abs(x[0] - target_ts))
            diff_th = abs(closest_th[0] - target_ts)
            if diff_th <= tol:
                thermal_msg = closest_th[1]
            else:
                self.get_logger().debug(f"Drone {drone_id} thermal sync delta {diff_th:.3f}s > {tol:.3f}s tol")

        return depth_msg, thermal_msg

    def detection_callback(self, msg: String, drone_id: int):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Failed to parse detection JSON: {e}")
            return

        det_ts = data.get('timestamp')
        if det_ts is None or det_ts <= 0.0 or math.isnan(det_ts):
            self.get_logger().warn(f"Drone {drone_id}: Rejected detection with invalid simulation timestamp: {det_ts}")
            return
        detections = data.get('detections', [])
        if not detections:
            return

        depth_msg, thermal_msg = self._get_synced_frames(drone_id, det_ts)

        cv_depth = None
        if depth_msg is not None:
            try:
                cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            except Exception as e:
                self.get_logger().warn(f"Depth cv2 conversion error: {e}")

        cv_thermal_k = None
        cv_thermal_raw = None
        if thermal_msg is not None:
            try:
                dtype = np.dtype('>u2') if thermal_msg.is_bigendian else np.dtype('<u2')
                cv_thermal_raw = np.frombuffer(thermal_msg.data, dtype=dtype).reshape((thermal_msg.height, thermal_msg.width))
                cv_thermal_k = self.core.decode_thermal_mono16(cv_thermal_raw, thermal_msg.is_bigendian)
            except Exception as e:
                self.get_logger().warn(f"Thermal cv2 conversion error: {e}")

        drone_pose = self.drone_poses.get(drone_id)

        results = []
        for idx, det in enumerate(detections):
            det_cls = str(det.get('class', 'person')).strip().lower()
            if det_cls != 'person':
                self.get_logger().warn(f"Drone {drone_id}: Ignoring non-person detection class='{det_cls}'")
                continue

            cid = f"d{drone_id}_t{int(det_ts*100)}_{idx}"
            bbox = det['bbox']
            conf = det['confidence']

            res = self.core.verify_candidate(
                candidate_id=cid,
                drone_id=drone_id,
                timestamp=det_ts,
                rgb_bbox=bbox,
                rgb_confidence=conf,
                depth_image=cv_depth,
                thermal_image_k=cv_thermal_k,
                drone_pose=drone_pose
            )
            results.append(res)

            if res.status in [VerificationStatus.THERMAL_SUPPORTED.value, "VERIFIED"]:
                th_stat = res.thermal_stats or {}
                self.get_logger().info(
                    f"[FUSION THERMAL_SUPPORTED Drone {drone_id}] Person conf={conf:.3f} | "
                    f"T_max={th_stat.get('max_k', 0.0):.1f}K ({th_stat.get('max_k', 273.15)-273.15:.1f}°C), "
                    f"Delta_T=+{th_stat.get('delta_k', 0.0):.1f}K | MVS={res.verification_score:.2f} | "
                    f"depth={res.depth_m:.1f}m | world={res.world_pos}"
                )
                self._update_temporal_tracker(res)
            elif res.status == VerificationStatus.REJECTED.value:
                self.get_logger().info(
                    f"[FUSION REJECTED Drone {drone_id}] Reason: {res.rejection_reason} | "
                    f"conf={conf:.3f} | depth={res.depth_m:.1f}m"
                )

        out_msg = String()
        out_msg.data = json.dumps({
            'drone_id': drone_id,
            'timestamp': det_ts,
            'results': [r.to_dict() for r in results]
        })
        self.verified_pubs[drone_id].publish(out_msg)

        if self.publish_debug and cv_thermal_k is not None:
            self._publish_debug_overlay(drone_id, cv_thermal_k, results, thermal_msg.header)

    def _update_temporal_tracker(self, res: CandidateVerificationResult):
        best_tid, event = self.confirmer.add_multispectral_observation(res)
        if event is not None:
            evt_msg = String()
            evt_msg.data = json.dumps(event)
            self.discovery_pub.publish(evt_msg)
            self.get_logger().info(
                f"*** MULTISPECTRAL VICTIM CONFIRMED *** track={best_tid} at ({event['world_x']}, {event['world_y']}) "
                f"grid=({event['grid_x']}, {event['grid_y']}) conf={event['confidence']} "
                f"depth={event['median_depth_estimate']}m consensus={event['confirmation_count']}/{event['confirmation_window']} "
                f"span={event['consensus_span_sec']}s T_peak={event['thermal_peak_k']}K, "
                f"Delta_T=+{event['thermal_delta_k']}K"
            )

    def _publish_debug_overlay(self, drone_id: int, temp_k: np.ndarray, results: List[CandidateVerificationResult], header):
        t_min = float(np.nanmin(temp_k))
        t_max = float(np.nanmax(temp_k))
        norm = ((temp_k - t_min) / (t_max - t_min + 1e-5) * 255.0).astype(np.uint8)
        color = cv2.applyColorMap(norm, cv2.COLORMAP_INFERNO)

        for r in results:
            if r.projected_thermal_bbox is not None:
                u1, v1, u2, v2 = r.projected_thermal_bbox
                if r.status in [VerificationStatus.THERMAL_SUPPORTED.value, "VERIFIED"]:
                    box_color = (0, 255, 0)
                    t_val = r.thermal_stats['max_k'] if r.thermal_stats else 0.0
                    label = f"SUPP: {t_val:.1f}K MVS={r.verification_score:.2f}"
                elif r.status == VerificationStatus.RGB_CANDIDATE.value:
                    box_color = (255, 165, 0)
                    label = f"CAND: {r.rejection_reason[:8]}"
                else:
                    box_color = (0, 0, 255)
                    label = f"REJ: {r.rejection_reason[:8]}"

                cv2.rectangle(color, (u1, v1), (u2, v2), box_color, 2)
                cv2.putText(color, label, (u1, max(12, v1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, box_color, 1)

        out = self.bridge.cv2_to_imgmsg(color, encoding='bgr8')
        out.header = header
        self.debug_image_pubs[drone_id].publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = RGBThermalFusionVerifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
