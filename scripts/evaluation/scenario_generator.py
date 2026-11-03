#!/usr/bin/env python3
"""
Evaluation Scenario Generator (Phase 2 Remediation - Stage 5)

Generates 150 diverse, reproducible, and deterministic evaluation scenarios
with recorded random seeds covering:
  1. Human-Positive Cases (N=60): Standing, sitting, lying, prone, occluded, low-contrast, clustered.
  2. Non-Human Thermal Distractors (N=60): Sun-heated concrete, rocks, rubble, asphalt, vehicle panels,
     warm engine parts, fire/exhaust, human-sized warm non-human objects.
  3. Sensor & Perception Failures (N=30): Cold ground/shadows, missing sensors, NaNs, invalid depths,
     saturated sensors, misaligned projections.
"""

from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple, Any
import numpy as np
import math

from swarm_controller.rgb_thermal_fusion_verifier import (
    RGBThermalFusionCore,
    CameraIntrinsics
)


@dataclass
class EvaluationScenario:
    scenario_id: int
    name: str
    category: str         # HUMAN_POSITIVE, THERMAL_DISTRACTOR, SENSOR_FAILURE
    subcategory: str      # standing, concrete, etc.
    gt_is_victim: bool    # True for humans, False for distractors/failures
    drone_id: int
    drone_pose: Tuple[float, float, float, float]  # (x, y, z, yaw)
    depth_m: float
    rgb_bbox: List[int]   # [x1, y1, x2, y2]
    rgb_confidence: float
    depth_image: Optional[np.ndarray]
    thermal_image_k: Optional[np.ndarray]
    metadata: Dict[str, Any] = field(default_factory=dict)


class ScenarioGenerator:
    """Deterministic generator for Stage 5 benchmark scenarios."""

    def __init__(self, master_seed: int = 42):
        self.master_seed = master_seed
        self.rng = np.random.RandomState(master_seed)
        self.core = RGBThermalFusionCore()

    def generate_all_scenarios(self, total_count: int = 150) -> List[EvaluationScenario]:
        """
        Generates exactly `total_count` scenarios (default 150: 60 positive, 60 negative, 30 failures).
        """
        scenarios: List[EvaluationScenario] = []
        s_id = 1

        # -------------------------------------------------------------
        # 1. HUMAN-POSITIVE SCENARIOS (N=60)
        # -------------------------------------------------------------
        # 1.1 Standing human victims (15 scenarios)
        for i in range(15):
            depth = float(self.rng.uniform(5.0, 25.0))
            phys_h = float(self.rng.uniform(1.60, 1.95))
            phys_w = float(self.rng.uniform(0.45, 0.65))
            ambient_k = float(self.rng.uniform(278.15, 292.15))
            core_k = float(self.rng.uniform(306.5, 310.5))
            conf = float(self.rng.uniform(0.75, 0.95))
            sc = self._create_human_scenario(
                s_id, f"Standing Human #{i+1}", "standing", depth,
                phys_w, phys_h, ambient_k, core_k, conf, occlusion=0.0
            )
            scenarios.append(sc)
            s_id += 1

        # 1.2 Sitting human victims (12 scenarios)
        for i in range(12):
            depth = float(self.rng.uniform(5.0, 20.0))
            phys_h = float(self.rng.uniform(0.95, 1.30))
            phys_w = float(self.rng.uniform(0.55, 0.80))
            ambient_k = float(self.rng.uniform(278.15, 292.15))
            core_k = float(self.rng.uniform(306.0, 309.5))
            conf = float(self.rng.uniform(0.70, 0.92))
            sc = self._create_human_scenario(
                s_id, f"Sitting Human #{i+1}", "sitting", depth,
                phys_w, phys_h, ambient_k, core_k, conf, occlusion=0.0
            )
            scenarios.append(sc)
            s_id += 1

        # 1.3 Lying / prone human victims (13 scenarios)
        for i in range(13):
            depth = float(self.rng.uniform(5.0, 20.0))
            phys_w = float(self.rng.uniform(1.50, 1.95))  # Elongated
            phys_h = float(self.rng.uniform(0.45, 0.65))
            ambient_k = float(self.rng.uniform(278.15, 292.15))
            core_k = float(self.rng.uniform(306.0, 309.8))
            conf = float(self.rng.uniform(0.68, 0.90))
            sc = self._create_human_scenario(
                s_id, f"Lying Prone Human #{i+1}", "lying_prone", depth,
                phys_w, phys_h, ambient_k, core_k, conf, occlusion=0.0
            )
            scenarios.append(sc)
            s_id += 1

        # 1.4 Partially occluded human victims (8 scenarios)
        for i in range(8):
            depth = float(self.rng.uniform(6.0, 18.0))
            phys_h = float(self.rng.uniform(1.60, 1.85))
            phys_w = float(self.rng.uniform(0.45, 0.60))
            ambient_k = float(self.rng.uniform(280.15, 290.15))
            core_k = float(self.rng.uniform(306.5, 309.5))
            conf = float(self.rng.uniform(0.62, 0.82))
            occlusion = float(self.rng.uniform(0.15, 0.45))
            sc = self._create_human_scenario(
                s_id, f"Partially Occluded Human #{i+1} ({int(occlusion*100)}% occl)",
                "occluded_human", depth, phys_w, phys_h, ambient_k, core_k, conf, occlusion=occlusion
            )
            scenarios.append(sc)
            s_id += 1

        # 1.5 Insulating clothing / reduced thermal contrast (7 scenarios)
        for i in range(7):
            depth = float(self.rng.uniform(6.0, 16.0))
            phys_h = float(self.rng.uniform(1.65, 1.90))
            phys_w = float(self.rng.uniform(0.50, 0.70))
            ambient_k = float(self.rng.uniform(285.15, 292.15))
            core_k = ambient_k + float(self.rng.uniform(3.8, 6.5))  # Modest contrast above threshold
            conf = float(self.rng.uniform(0.72, 0.88))
            sc = self._create_human_scenario(
                s_id, f"Insulated Clothing Human #{i+1} (+{core_k-ambient_k:.1f}K)",
                "insulated_clothing", depth, phys_w, phys_h, ambient_k, core_k, conf, occlusion=0.0
            )
            scenarios.append(sc)
            s_id += 1

        # 1.6 Clustered nearby victims (5 scenarios)
        for i in range(5):
            depth = float(self.rng.uniform(7.0, 15.0))
            phys_h = float(self.rng.uniform(1.65, 1.85))
            phys_w = float(self.rng.uniform(0.50, 0.65))
            ambient_k = 288.15
            core_k = 308.2
            conf = float(self.rng.uniform(0.80, 0.92))
            sc = self._create_human_scenario(
                s_id, f"Clustered Victims #{i+1}", "clustered_human", depth,
                phys_w, phys_h, ambient_k, core_k, conf, occlusion=0.0, secondary_target=True
            )
            scenarios.append(sc)
            s_id += 1

        # -------------------------------------------------------------
        # 2. NON-HUMAN THERMAL DISTRACTORS (N=60)
        # -------------------------------------------------------------
        # 2.1 Sun-heated concrete slabs (10 scenarios)
        for i in range(10):
            depth = float(self.rng.uniform(6.0, 18.0))
            temp_k = float(self.rng.uniform(302.0, 312.0))
            sc = self._create_distractor_scenario(
                s_id, f"Sun-Heated Concrete Slab #{i+1}", "concrete_slab",
                depth=depth, temp_k=temp_k, is_diffuse=True, conf=float(self.rng.uniform(0.55, 0.78))
            )
            scenarios.append(sc)
            s_id += 1

        # 2.2 Sun-heated rocks / boulders (10 scenarios)
        for i in range(10):
            depth = float(self.rng.uniform(6.0, 18.0))
            temp_k = float(self.rng.uniform(299.0, 307.0))
            sc = self._create_distractor_scenario(
                s_id, f"Sun-Heated Rock #{i+1}", "heated_rock",
                depth=depth, temp_k=temp_k, is_diffuse=False, is_edge_heated=True,
                conf=float(self.rng.uniform(0.42, 0.62))
            )
            scenarios.append(sc)
            s_id += 1

        # 2.3 Concrete rubble piles (6 scenarios)
        for i in range(6):
            depth = float(self.rng.uniform(6.0, 15.0))
            temp_k = float(self.rng.uniform(298.5, 306.0))
            sc = self._create_distractor_scenario(
                s_id, f"Rubble Pile #{i+1}", "rubble_pile",
                depth=depth, temp_k=temp_k, is_diffuse=True, conf=float(self.rng.uniform(0.40, 0.58))
            )
            scenarios.append(sc)
            s_id += 1

        # 2.4 Asphalt road / pathway patches (6 scenarios)
        for i in range(6):
            depth = float(self.rng.uniform(8.0, 20.0))
            temp_k = float(self.rng.uniform(305.0, 315.0))
            sc = self._create_distractor_scenario(
                s_id, f"Asphalt Patch #{i+1}", "asphalt",
                depth=depth, temp_k=temp_k, is_diffuse=True, conf=float(self.rng.uniform(0.45, 0.65))
            )
            scenarios.append(sc)
            s_id += 1

        # 2.5 Vehicle panels / hoods (8 scenarios)
        for i in range(8):
            depth = float(self.rng.uniform(6.0, 16.0))
            temp_k = float(self.rng.uniform(308.0, 316.0))
            # Oversized geometric dimension (> 2.4m)
            sc = self._create_distractor_scenario(
                s_id, f"Vehicle Hood/Panel #{i+1}", "vehicle_panel",
                depth=depth, temp_k=temp_k, oversized=True, conf=float(self.rng.uniform(0.60, 0.82))
            )
            scenarios.append(sc)
            s_id += 1

        # 2.6 Warm engine components (6 scenarios)
        for i in range(6):
            depth = float(self.rng.uniform(5.0, 14.0))
            temp_k = float(self.rng.uniform(314.0, 317.8))
            sc = self._create_distractor_scenario(
                s_id, f"Warm Engine Component #{i+1}", "engine_component",
                depth=depth, temp_k=temp_k, is_diffuse=True, conf=float(self.rng.uniform(0.45, 0.65))
            )
            scenarios.append(sc)
            s_id += 1

        # 2.7 Hot exhaust and fire clutter (8 scenarios)
        for i in range(8):
            depth = float(self.rng.uniform(5.0, 20.0))
            temp_k = float(self.rng.uniform(325.0, 360.0))  # > 45°C
            sc = self._create_distractor_scenario(
                s_id, f"Hot Exhaust / Campfire #{i+1} ({temp_k-273.15:.0f}°C)", "exhaust_fire",
                depth=depth, temp_k=temp_k, is_diffuse=False, conf=float(self.rng.uniform(0.50, 0.85))
            )
            scenarios.append(sc)
            s_id += 1

        # 2.8 Human-sized warm non-human objects (6 scenarios)
        for i in range(6):
            depth = float(self.rng.uniform(6.0, 15.0))
            temp_k = float(self.rng.uniform(301.5, 306.5))
            sc = self._create_distractor_scenario(
                s_id, f"Warm Box/Barrel #{i+1}", "warm_box",
                depth=depth, temp_k=temp_k, is_isothermal_box=True, conf=float(self.rng.uniform(0.42, 0.58))
            )
            scenarios.append(sc)
            s_id += 1

        # -------------------------------------------------------------
        # 3. SENSOR & PERCEPTION FAILURES (N=30)
        # -------------------------------------------------------------
        # 3.1 Cold ambient ground & tree shadows (6 scenarios)
        for i in range(6):
            depth = float(self.rng.uniform(6.0, 16.0))
            ambient_k = 288.15
            temp_k = ambient_k - float(self.rng.uniform(0.0, 3.0))  # Cold or shadow
            sc = self._create_failure_scenario(
                s_id, f"Cold Shadow / Ambient Ground #{i+1}", "cold_shadow",
                depth=depth, failure_type="cold_shadow", temp_k=temp_k
            )
            scenarios.append(sc)
            s_id += 1

        # 3.2 Missing sensor streams (6 scenarios)
        for i in range(6):
            depth = float(self.rng.uniform(6.0, 15.0))
            f_type = "missing_thermal" if (i % 2 == 0) else "missing_depth"
            sc = self._create_failure_scenario(
                s_id, f"Missing Stream ({f_type}) #{i+1}", f_type,
                depth=depth, failure_type=f_type
            )
            scenarios.append(sc)
            s_id += 1

        # 3.3 Corrupted / NaN frames (6 scenarios)
        for i in range(6):
            depth = float(self.rng.uniform(6.0, 15.0))
            f_type = "nan_thermal" if (i % 2 == 0) else "nan_depth"
            sc = self._create_failure_scenario(
                s_id, f"Corrupted Frame ({f_type}) #{i+1}", f_type,
                depth=depth, failure_type=f_type
            )
            scenarios.append(sc)
            s_id += 1

        # 3.4 Invalid depth range (4 scenarios)
        for i in range(4):
            depth = 1.8 if (i % 2 == 0) else 45.0  # < 3m or > 35m
            sc = self._create_failure_scenario(
                s_id, f"Invalid Depth Range ({depth:.1f}m) #{i+1}", "invalid_depth",
                depth=depth, failure_type="invalid_depth"
            )
            scenarios.append(sc)
            s_id += 1

        # 3.5 Saturated thermal sensor (4 scenarios)
        for i in range(4):
            depth = float(self.rng.uniform(6.0, 16.0))
            sc = self._create_failure_scenario(
                s_id, f"Saturated Thermal Sensor (323.15K) #{i+1}", "saturated_sensor",
                depth=depth, failure_type="saturated_sensor"
            )
            scenarios.append(sc)
            s_id += 1

        # 3.6 Misaligned RGB-thermal projection (4 scenarios)
        for i in range(4):
            depth = float(self.rng.uniform(6.0, 16.0))
            sc = self._create_failure_scenario(
                s_id, f"Misaligned RGB-Thermal Projection #{i+1}", "misaligned_projection",
                depth=depth, failure_type="misaligned_projection"
            )
            scenarios.append(sc)
            s_id += 1

        return scenarios

    # -------------------------------------------------------------
    # Helper builders
    # -------------------------------------------------------------
    def _create_human_scenario(
        self, s_id: int, name: str, subcategory: str, depth_m: float,
        phys_w_m: float, phys_h_m: float, ambient_k: float, core_k: float,
        conf: float, occlusion: float = 0.0, secondary_target: bool = False
    ) -> EvaluationScenario:
        fx = self.core.rgb_cam.fx
        fy = self.core.rgb_cam.fy
        w_px = max(6, int(round((phys_w_m / depth_m) * fx)))
        h_px = max(10, int(round((phys_h_m / depth_m) * fy)))

        # Bounding box center near image center with small random jitter
        u_c = int(self.rng.uniform(280, 360))
        v_c = int(self.rng.uniform(200, 280))
        x1 = max(0, u_c - w_px // 2)
        y1 = max(0, v_c - h_px // 2)
        x2 = min(640, x1 + w_px)
        y2 = min(480, y1 + h_px)
        rgb_bbox = [x1, y1, x2, y2]

        depth_img = np.full((480, 640), depth_m, dtype=np.float32)
        th_img = np.full((240, 320), ambient_k, dtype=np.float32)

        # Thermal hotspot projection
        u_th, v_th = self.core.project_rgb_point_to_thermal(float(u_c), float(v_c), depth_m)
        # Thermal hotspot projection based on true projected bounding box
        bbox_th, fully, part = self.core.project_rgb_bbox_to_thermal(rgb_bbox, depth_m)
        if bbox_th is not None:
            b_w = bbox_th[2] - bbox_th[0]
            b_h = bbox_th[3] - bbox_th[1]
            u_t = (bbox_th[0] + bbox_th[2]) // 2
            v_t = (bbox_th[1] + bbox_th[3]) // 2
        else:
            b_w = max(4, int(round((phys_w_m / depth_m) * self.core.th_cam.fx)))
            b_h = max(6, int(round((phys_h_m / depth_m) * self.core.th_cam.fy)))
            u_t = int(round(u_th))
            v_t = int(round(v_th))

        # Outer body envelope covering ~70% width, ~85% height
        th_w = max(3, int(round(b_w * 0.70)))
        th_h = max(5, int(round(b_h * 0.85)))
        th_y1 = max(0, v_t - th_h // 2)
        th_y2 = min(240, v_t + th_h // 2)
        th_x1 = max(0, u_t - th_w // 2)
        th_x2 = min(320, u_t + th_w // 2)
        th_img[th_y1:th_y2, th_x1:th_x2] = ambient_k + 0.65 * (core_k - ambient_k)

        # Concentrated thoracic core covering ~40% width, ~45% height
        cw = max(2, int(round(b_w * 0.40)))
        ch = max(3, int(round(b_h * 0.45)))
        c_y1 = max(0, v_t - ch // 2)
        c_y2 = min(240, v_t + ch // 2)
        c_x1 = max(0, u_t - cw // 2)
        c_x2 = min(320, u_t + cw // 2)
        th_img[c_y1:c_y2, c_x1:c_x2] = core_k

        # Apply occlusion if specified
        if occlusion > 0.0:
            occ_h = int(th_h * occlusion)
            th_img[th_y2 - occ_h:th_y2, th_x1:th_x2] = ambient_k

        # Secondary distractor target nearby if clustered
        if secondary_target:
            th_img[max(0, v_t - 10):min(240, v_t + 10), min(319, u_t + 25):min(320, u_t + 35)] = core_k - 3.0

        world_x = 5.0 + float((s_id * 7) % 85)
        world_y = 5.0 + float((s_id * 11) % 85)

        return EvaluationScenario(
            scenario_id=s_id,
            name=name,
            category="HUMAN_POSITIVE",
            subcategory=subcategory,
            gt_is_victim=True,
            drone_id=0,
            drone_pose=(world_x, world_y, depth_m, 0.0),
            depth_m=depth_m,
            rgb_bbox=rgb_bbox,
            rgb_confidence=conf,
            depth_image=depth_img,
            thermal_image_k=th_img,
            metadata={
                "phys_dims_m": (phys_w_m, phys_h_m),
                "ambient_k": ambient_k,
                "core_k": core_k,
                "contrast_k": core_k - ambient_k,
                "occlusion": occlusion
            }
        )

    def _create_distractor_scenario(
        self, s_id: int, name: str, subcategory: str, depth_m: float = 10.0,
        temp_k: float = 300.0, is_diffuse: bool = False, is_edge_heated: bool = False,
        oversized: bool = False, is_isothermal_box: bool = False, conf: float = 0.60,
        depth: Optional[float] = None
    ) -> EvaluationScenario:
        if depth is not None:
            depth_m = depth
        ambient_k = 288.15
        fx = self.core.rgb_cam.fx
        fy = self.core.rgb_cam.fy

        if oversized:
            phys_w_m, phys_h_m = 2.80, 1.80  # Exceeds 2.40m
        elif is_isothermal_box:
            phys_w_m, phys_h_m = 0.60, 1.65  # Human-sized
        else:
            phys_w_m, phys_h_m = 1.20, 1.20

        w_px = max(10, int(round((phys_w_m / depth_m) * fx)))
        h_px = max(10, int(round((phys_h_m / depth_m) * fy)))

        u_c = int(self.rng.uniform(300, 340))
        v_c = int(self.rng.uniform(220, 260))
        x1 = max(0, u_c - w_px // 2)
        y1 = max(0, v_c - h_px // 2)
        x2 = min(640, x1 + w_px)
        y2 = min(480, y1 + h_px)
        rgb_bbox = [x1, y1, x2, y2]

        depth_img = np.full((480, 640), depth_m, dtype=np.float32)
        th_img = np.full((240, 320), ambient_k, dtype=np.float32)

        u_th, v_th = self.core.project_rgb_point_to_thermal(float(u_c), float(v_c), depth_m)
        bbox_th, fully, part = self.core.project_rgb_bbox_to_thermal(rgb_bbox, depth_m)

        if bbox_th is not None:
            b_w = bbox_th[2] - bbox_th[0]
            b_h = bbox_th[3] - bbox_th[1]
            u_t = (bbox_th[0] + bbox_th[2]) // 2
            v_t = (bbox_th[1] + bbox_th[3]) // 2
        else:
            b_w = max(6, int(round((phys_w_m / depth_m) * self.core.th_cam.fx)))
            b_h = max(6, int(round((phys_h_m / depth_m) * self.core.th_cam.fy)))
            u_t = int(round(u_th))
            v_t = int(round(v_th))

        if is_diffuse:
            # Slabs (concrete, asphalt, rubble) extend across ROI and into background
            th_y1 = max(0, v_t - int(b_h * 0.70))
            th_y2 = min(240, v_t + int(b_h * 0.70))
            th_x1 = max(0, u_t - int(b_w * 0.70))
            th_x2 = min(320, u_t + int(b_w * 0.70))
            th_img[th_y1:th_y2, th_x1:th_x2] = temp_k
        elif is_edge_heated:
            # Rock filling the bounding box, core cooler than edges
            th_y1 = max(0, v_t - b_h // 2)
            th_y2 = min(240, v_t + b_h // 2)
            th_x1 = max(0, u_t - b_w // 2)
            th_x2 = min(320, u_t + b_w // 2)
            th_img[th_y1:th_y2, th_x1:th_x2] = temp_k
            cw = max(2, b_w // 2)
            ch = max(2, b_h // 2)
            th_img[v_t - ch // 2: v_t + ch // 2, u_t - cw // 2: u_t + cw // 2] = temp_k - 1.2
        elif is_isothermal_box or oversized:
            th_y1 = max(0, v_t - b_h // 2)
            th_y2 = min(240, v_t + b_h // 2)
            th_x1 = max(0, u_t - b_w // 2)
            th_x2 = min(320, u_t + b_w // 2)
            th_img[th_y1:th_y2, th_x1:th_x2] = temp_k
        else:
            th_y1 = max(0, v_t - b_h // 2)
            th_y2 = min(240, v_t + b_h // 2)
            th_x1 = max(0, u_t - b_w // 2)
            th_x2 = min(320, u_t + b_w // 2)
            th_img[th_y1:th_y2, th_x1:th_x2] = temp_k

        world_x = 5.0 + float((s_id * 7) % 85)
        world_y = 5.0 + float((s_id * 11) % 85)

        return EvaluationScenario(
            scenario_id=s_id,
            name=name,
            category="THERMAL_DISTRACTOR",
            subcategory=subcategory,
            gt_is_victim=False,
            drone_id=0,
            drone_pose=(world_x, world_y, depth_m, 0.0),
            depth_m=depth_m,
            rgb_bbox=rgb_bbox,
            rgb_confidence=conf,
            depth_image=depth_img,
            thermal_image_k=th_img,
            metadata={
                "phys_dims_m": (phys_w_m, phys_h_m),
                "temp_k": temp_k,
                "is_diffuse": is_diffuse,
                "oversized": oversized
            }
        )

    def _create_failure_scenario(
        self, s_id: int, name: str, subcategory: str, depth_m: float = 10.0,
        failure_type: str = "cold_shadow", temp_k: float = 288.15,
        depth: Optional[float] = None
    ) -> EvaluationScenario:
        if depth is not None:
            depth_m = depth
        rgb_bbox = [310, 220, 330, 260]
        conf = 0.80

        if failure_type == "missing_depth":
            depth_img = None
            th_img = np.full((240, 320), 288.15, dtype=np.float32)
        elif failure_type == "missing_thermal":
            depth_img = np.full((480, 640), depth_m, dtype=np.float32)
            th_img = None
        elif failure_type == "nan_depth":
            depth_img = np.full((480, 640), np.nan, dtype=np.float32)
            th_img = np.full((240, 320), 288.15, dtype=np.float32)
        elif failure_type == "nan_thermal":
            depth_img = np.full((480, 640), depth_m, dtype=np.float32)
            th_img = np.full((240, 320), np.nan, dtype=np.float32)
        elif failure_type == "invalid_depth":
            depth_img = np.full((480, 640), depth_m, dtype=np.float32)
            th_img = np.full((240, 320), 308.0, dtype=np.float32)
        elif failure_type == "saturated_sensor":
            depth_img = np.full((480, 640), depth_m, dtype=np.float32)
            th_img = np.full((240, 320), 323.15, dtype=np.float32)  # Pegged at sensor maximum
        elif failure_type == "misaligned_projection":
            depth_img = np.full((480, 640), depth_m, dtype=np.float32)
            th_img = np.full((240, 320), 288.15, dtype=np.float32)
            # Hotspot at (160, 120), but bbox is offset to (100, 120)
            th_img[110:130, 155:165] = 308.4
            rgb_bbox = [180, 220, 200, 260]  # Far from hotspot
        elif failure_type == "cold_shadow":
            depth_img = np.full((480, 640), depth_m, dtype=np.float32)
            th_img = np.full((240, 320), temp_k, dtype=np.float32)
        else:
            depth_img = np.full((480, 640), depth_m, dtype=np.float32)
            th_img = np.full((240, 320), 288.15, dtype=np.float32)

        world_x = 5.0 + float((s_id * 7) % 85)
        world_y = 5.0 + float((s_id * 11) % 85)

        return EvaluationScenario(
            scenario_id=s_id,
            name=name,
            category="SENSOR_FAILURE",
            subcategory=subcategory,
            gt_is_victim=False,
            drone_id=0,
            drone_pose=(world_x, world_y, depth_m if depth_m > 0 else 10.0, 0.0),
            depth_m=depth_m,
            rgb_bbox=rgb_bbox,
            rgb_confidence=conf,
            depth_image=depth_img,
            thermal_image_k=th_img,
            metadata={"failure_type": failure_type}
        )
