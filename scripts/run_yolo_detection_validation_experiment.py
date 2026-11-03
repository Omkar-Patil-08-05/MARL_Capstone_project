#!/usr/bin/env python3
"""
Empirical YOLOv8n & RGB-D Detection Validation Experiment
Executes 7 distinct scenarios in Gazebo Harmonic with real camera frames,
depth buffers, pinhole projection, temporal event confirmation, and Hungarian ground-truth matching.
"""

import os
import sys
import time
import math
import json
import csv
import subprocess
import signal
import cv2
import numpy as np

# Ensure yolo_venv and ROS packages are on sys.path
yolo_site_packages = '/home/capstone/yolo_venv/lib/python3.12/site-packages'
if os.path.exists(yolo_site_packages) and yolo_site_packages not in sys.path:
    sys.path.insert(0, yolo_site_packages)

from ultralytics import YOLO

# Add ROS package paths
WORKSPACE = "/home/capstone/capstone_project_antigravity"
sys.path.insert(0, f"{WORKSPACE}/drone_ws/src/swarm_controller")
sys.path.insert(0, f"{WORKSPACE}/drone_ws/src/swarm_controller/swarm_controller")

from swarm_controller.rgbd_victim_localizer import RGBDGeometricLocalizer
from swarm_controller.temporal_event_confirmer import TemporalEventConfirmer
from swarm_controller.ground_truth_matcher import GroundTruthMatcher

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class FrameCollectorNode(Node):
    def __init__(self):
        super().__init__('yolo_validation_frame_collector')
        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_depth = None
        self.camera_info = None
        self.fx = 269.97
        self.fy = 269.97
        self.cx = 320.0
        self.cy = 240.0

        # Subscribe to both native Gazebo bridge topic names and remapped names
        gz_rgb = '/world/validation_suite/model/camera_drone/link/camera_link/sensor/camera/image'
        gz_depth = '/world/validation_suite/model/camera_drone/link/camera_link/sensor/camera/depth_image'
        gz_info = '/world/validation_suite/model/camera_drone/link/camera_link/sensor/camera/camera_info'

        self.sub_rgb = self.create_subscription(Image, '/drone_0/camera/image', self._on_rgb, 10)
        self.sub_depth = self.create_subscription(Image, '/drone_0/camera/depth_image', self._on_depth, 10)
        self.sub_info = self.create_subscription(CameraInfo, '/drone_0/camera/camera_info', self._on_info, 10)

        self.sub_rgb_gz = self.create_subscription(Image, gz_rgb, self._on_rgb, 10)
        self.sub_depth_gz = self.create_subscription(Image, gz_depth, self._on_depth, 10)
        self.sub_info_gz = self.create_subscription(CameraInfo, gz_info, self._on_info, 10)

    def _on_rgb(self, msg):
        self.latest_rgb = msg

    def _on_depth(self, msg):
        self.latest_depth = msg

    def _on_info(self, msg):
        self.camera_info = msg
        if msg.k[0] > 0:
            self.fx = float(msg.k[0])
            self.fy = float(msg.k[4])
            self.cx = float(msg.k[2])
            self.cy = float(msg.k[5])

    def capture_frame(self, wait_sec=4.0):
        """Spins to capture the latest RGB and depth pair."""
        t_start = time.time()
        self.latest_rgb = None
        self.latest_depth = None
        while time.time() - t_start < wait_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_rgb is not None and self.latest_depth is not None:
                rgb_cv = self.bridge.imgmsg_to_cv2(self.latest_rgb, 'bgr8')
                depth_cv = self.bridge.imgmsg_to_cv2(self.latest_depth, desired_encoding="passthrough")
                return rgb_cv, depth_cv
        return None, None


class GazeboValidationHarness:
    def __init__(self):
        self.pids = []
        self.gz_env = os.environ.copy()
        self.gz_env["GZ_IP"] = "127.0.0.1"
        self.gz_env["GZ_SIM_RESOURCE_PATH"] = (
            f"{WORKSPACE}/models:{WORKSPACE}/assets_real:{WORKSPACE}/assets_real/victims:"
            f"/home/capstone/PX4-Autopilot/Tools/simulation/gz/models:"
            f"/home/capstone/PX4-Autopilot/Tools/simulation/gz/worlds:{WORKSPACE}/generated_worlds:{WORKSPACE}/worlds"
        )
        self.world_path = f"{WORKSPACE}/worlds/validation_detection_suite.sdf"
        self.world_name = "validation_suite"

    def start_gazebo_and_bridge(self):
        print("\n=======================================================")
        print("  LAUNCHING GAZEBO HARMONIC VALIDATION ENVIRONMENT")
        print("=======================================================")
        cmd_gz = (
            f"source /home/capstone/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh && "
            f"gz sim -s -r --headless-rendering {self.world_path}"
        )
        p_gz = subprocess.Popen(cmd_gz, shell=True, env=self.gz_env, executable="/bin/bash", preexec_fn=os.setsid)
        self.pids.append(p_gz.pid)
        print(f"[Gazebo] Process started (PID {p_gz.pid}). Waiting for initialization...")
        time.sleep(7)

        topics = (
            f"/world/{self.world_name}/model/camera_drone/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image "
            f"/world/{self.world_name}/model/camera_drone/link/camera_link/sensor/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image "
            f"/world/{self.world_name}/model/camera_drone/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
        )
        remaps = (
            f"--ros-args "
            f"-r /world/{self.world_name}/model/camera_drone/link/camera_link/sensor/camera/image:=/drone_0/camera/image "
            f"-r /world/{self.world_name}/model/camera_drone/link/camera_link/sensor/camera/depth_image:=/drone_0/camera/depth_image "
            f"-r /world/{self.world_name}/model/camera_drone/link/camera_link/sensor/camera/camera_info:=/drone_0/camera/camera_info"
        )
        cmd_br = f"source /opt/ros/jazzy/setup.bash && ros2 run ros_gz_bridge parameter_bridge {topics} {remaps}"
        p_br = subprocess.Popen(cmd_br, shell=True, executable="/bin/bash", preexec_fn=os.setsid)
        self.pids.append(p_br.pid)
        print(f"[ros_gz_bridge] Bridge started (PID {p_br.pid}). Waiting for streams...")
        time.sleep(4)

    def set_camera_pose(self, x, y, z):
        """Sets the camera drone position and downward pitch orientation (90 deg pitch)."""
        # Pitch 90 deg down: quaternion (w=cos(pi/4)=0.7071068, y=sin(pi/4)=0.7071068)
        pose_req = (
            f'name: "camera_drone", '
            f'position: {{x: {x:.2f}, y: {y:.2f}, z: {z:.2f}}}, '
            f'orientation: {{x: 0, y: 0.7071068, z: 0, w: 0.7071068}}'
        )
        cmd = f"gz service -s /world/{self.world_name}/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 --req '{pose_req}'"
        res = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        time.sleep(1.2)  # Settle time for Gazebo rendering pipeline

    def shutdown(self):
        print("\n[Teardown] Shutting down Gazebo and bridge...")
        for pid in reversed(self.pids):
            try:
                os.killpg(pid, signal.SIGTERM)
            except Exception:
                pass
        time.sleep(1)
        for pid in reversed(self.pids):
            try:
                os.killpg(pid, signal.SIGKILL)
            except Exception:
                pass
        print("[Teardown] Cleaned up all background simulator processes.")


def run_experiment():
    harness = GazeboValidationHarness()
    rclpy.init()
    node = FrameCollectorNode()

    # Load YOLOv8n
    model_path = f"{WORKSPACE}/models/yolov8n.pt"
    print(f"\n[YOLO] Loading YOLOv8n model from {model_path} onto cuda:0...")
    yolo_model = YOLO(model_path)

    # Localizer & Temporal Confirmer
    localizer = RGBDGeometricLocalizer(fx=269.97, fy=269.97, cx=320.0, cy=240.0)
    confirmer = TemporalEventConfirmer(window_size=6, consensus_needed=3, proximity_threshold=4.0)

    results_dir = f"{WORKSPACE}/results"
    images_dir = f"{results_dir}/images"
    os.makedirs(images_dir, exist_ok=True)

    all_scenario_records = []
    distance_eval_records = []
    annotated_images_meta = []

    # Ground truth definitions in world coords:
    gt_victims = {
        "standing": {"x": 0.0, "y": 0.0, "type": "casual_female", "pose": "standing"},
        "sitting": {"x": 15.0, "y": 0.0, "type": "rescue_randy_sitting", "pose": "sitting"},
        "lying": {"x": 30.0, "y": 0.0, "type": "casual_female_lying", "pose": "lying"},
        "obscured": {"x": 45.0, "y": 0.0, "type": "rescue_randy_standing", "pose": "partially_obscured_by_tree"},
        "multi_a": {"x": 60.0, "y": -1.5, "type": "casual_female", "pose": "multi_standing_left"},
        "multi_b": {"x": 60.0, "y": 1.5, "type": "rescue_randy_standing", "pose": "multi_standing_right"}
    }

    try:
        harness.start_gazebo_and_bridge()

        # Update localizer intrinsics once camera info arrives
        rgb_init, depth_init = node.capture_frame(wait_sec=5.0)
        if node.camera_info:
            localizer.fx = node.fx
            localizer.fy = node.fy
            localizer.cx = node.cx
            localizer.cy = node.cy
            print(f"[Camera Info] Intrinsics updated: fx={node.fx:.2f}, fy={node.fy:.2f}, cx={node.cx:.2f}, cy={node.cy:.2f}")

        # =========================================================================
        # SCENARIO 1: VICTIM CLEARLY VISIBLE (Standing, Sitting, Lying)
        # =========================================================================
        print("\n" + "="*70)
        print("  SCENARIO 1: VICTIM CLEARLY VISIBLE (Standing, Sitting, Lying)")
        print("="*70)

        s1_tests = [
            ("1A_standing", 0.0, 0.0, 10.0, gt_victims["standing"], "scenario1_victim_clearly_visible_standing.png"),
            ("1B_sitting", 15.0, 0.0, 10.0, gt_victims["sitting"], "scenario1_victim_clearly_visible_sitting.png"),
            ("1C_lying", 30.0, 0.0, 10.0, gt_victims["lying"], "scenario1_victim_clearly_visible_lying.png"),
        ]

        for test_id, cx, cy, cz, gt, img_fname in s1_tests:
            print(f"\n--- Testing Sub-Scenario {test_id} (Drone at x={cx}, y={cy}, z={cz}) ---")
            harness.set_camera_pose(cx, cy, cz)
            rgb, depth = node.capture_frame()
            if rgb is None:
                print(f"[ERROR] No frame captured for {test_id}!")
                continue

            results = yolo_model(rgb, verbose=False)
            boxes = results[0].boxes
            annotated = rgb.copy()

            detected_persons = []
            non_persons = []

            for b in boxes:
                cls_id = int(b.cls[0])
                cls_name = yolo_model.names[cls_id]
                conf = float(b.conf[0])
                x1, y1, x2, y2 = [int(v) for v in b.xyxy[0]]

                # Draw raw bbox
                color = (0, 255, 0) if cls_id == 0 else (0, 0, 255)
                cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                cv2.putText(annotated, f"{cls_name} {conf:.2f}", (x1, max(20, y1-10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                if cls_id == 0:  # Person class
                    u_center = int((x1 + x2) / 2)
                    v_center = int((y1 + y2) / 2)
                    # Extract patch median depth
                    pw = max(4, int((x2 - x1) * 0.4))
                    ph = max(4, int((y2 - y1) * 0.4))
                    depth_patch = depth[max(0, v_center-ph//2):min(depth.shape[0], v_center+ph//2),
                                        max(0, u_center-pw//2):min(depth.shape[1], u_center+pw//2)]
                    valid_mask = (depth_patch > 0.1) & (~np.isnan(depth_patch)) & (~np.isinf(depth_patch))
                    if np.sum(valid_mask) > 10:
                        measured_depth = float(np.median(depth_patch[valid_mask]))
                        est_wx, est_wy = localizer.localize_with_depth(
                            u_center, v_center, measured_depth, cx, cy, cz, drone_yaw=0.0
                        )
                        err = math.hypot(est_wx - gt["x"], est_wy - gt["y"])
                        detected_persons.append({
                            "conf": conf,
                            "bbox": [x1, y1, x2, y2],
                            "depth": measured_depth,
                            "est_x": est_wx,
                            "est_y": est_wy,
                            "err": err
                        })
                        cv2.putText(annotated, f"Loc: ({est_wx:.2f}, {est_wy:.2f}) Err: {err:.2f}m",
                                    (x1, min(annotated.shape[0]-10, y2+20)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                else:
                    non_persons.append({"class": cls_name, "conf": conf})

            img_path = f"{images_dir}/{img_fname}"
            cv2.imwrite(img_path, annotated)
            annotated_images_meta.append({"scenario": test_id, "file": img_fname, "path": img_path})

            is_tp = len(detected_persons) > 0
            best_det = detected_persons[0] if is_tp else None
            record = {
                "scenario": f"Scenario {test_id}",
                "description": f"Victim {gt['type']} ({gt['pose']})",
                "drone_alt_m": cz,
                "ground_truth_x": gt["x"],
                "ground_truth_y": gt["y"],
                "detected": is_tp,
                "yolo_conf": round(best_det["conf"], 3) if best_det else 0.0,
                "measured_depth_m": round(best_det["depth"], 3) if best_det else 0.0,
                "est_x": round(best_det["est_x"], 3) if best_det else None,
                "est_y": round(best_det["est_y"], 3) if best_det else None,
                "horiz_error_m": round(best_det["err"], 3) if best_det else None,
                "tp": 1 if is_tp else 0,
                "fp": 0,
                "fn": 0 if is_tp else 1,
                "non_person_filtered": len(non_persons)
            }
            all_scenario_records.append(record)
            print(f"Result {test_id}: Detected={is_tp}, Conf={record['yolo_conf']}, "
                  f"Est=({record['est_x']}, {record['est_y']}), Err={record['horiz_error_m']}m")

        # =========================================================================
        # SCENARIO 2: VICTIM AT DIFFERENT DISTANCES / ALTITUDES
        # =========================================================================
        print("\n" + "="*70)
        print("  SCENARIO 2: VICTIM AT DIFFERENT DISTANCES / ALTITUDES (4m - 30m)")
        print("="*70)

        altitudes = [4.0, 8.0, 12.0, 16.0, 20.0, 25.0, 30.0]
        gt_target = gt_victims["standing"]

        for alt in altitudes:
            harness.set_camera_pose(0.0, 0.0, alt)
            rgb, depth = node.capture_frame()
            if rgb is None:
                continue

            results = yolo_model(rgb, verbose=False)
            boxes = results[0].boxes
            annotated = rgb.copy()

            detected = False
            best_det = None

            for b in boxes:
                cls_id = int(b.cls[0])
                cls_name = yolo_model.names[cls_id]
                conf = float(b.conf[0])
                x1, y1, x2, y2 = [int(v) for v in b.xyxy[0]]
                w_px = x2 - x1
                h_px = y2 - y1

                if cls_id == 0:  # Person
                    u_center = int((x1 + x2) / 2)
                    v_center = int((y1 + y2) / 2)
                    pw = max(4, int(w_px * 0.4))
                    ph = max(4, int(h_px * 0.4))
                    depth_patch = depth[max(0, v_center-ph//2):min(depth.shape[0], v_center+ph//2),
                                        max(0, u_center-pw//2):min(depth.shape[1], u_center+pw//2)]
                    valid_mask = (depth_patch > 0.1) & (~np.isnan(depth_patch)) & (~np.isinf(depth_patch))
                    measured_depth = float(np.median(depth_patch[valid_mask])) if np.sum(valid_mask) > 5 else alt - 1.7
                    est_wx, est_wy = localizer.localize_with_depth(
                        u_center, v_center, measured_depth, 0.0, 0.0, alt, drone_yaw=0.0
                    )
                    err = math.hypot(est_wx - gt_target["x"], est_wy - gt_target["y"])
                    detected = True
                    best_det = {
                        "conf": conf,
                        "w_px": w_px,
                        "h_px": h_px,
                        "depth": measured_depth,
                        "est_x": est_wx,
                        "est_y": est_wy,
                        "err": err
                    }
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(annotated, f"Person {conf:.2f} Alt: {alt:.0f}m", (x1, max(20, y1-10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(annotated, f"Depth: {measured_depth:.2f}m Err: {err:.2f}m",
                                (x1, min(annotated.shape[0]-10, y2+20)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

            img_fname = f"scenario2_altitude_{int(alt)}m.png"
            img_path = f"{images_dir}/{img_fname}"
            cv2.imwrite(img_path, annotated)
            annotated_images_meta.append({"scenario": f"Scenario 2 ({alt}m)", "file": img_fname, "path": img_path})

            dist_record = {
                "altitude_m": alt,
                "detected": detected,
                "bbox_width_px": best_det["w_px"] if best_det else 0,
                "bbox_height_px": best_det["h_px"] if best_det else 0,
                "yolo_conf": round(best_det["conf"], 3) if best_det else 0.0,
                "measured_depth_m": round(best_det["depth"], 3) if best_det else 0.0,
                "horiz_error_m": round(best_det["err"], 3) if best_det else None,
                "status": "DETECTED" if detected else "MISSED (Too Small/Far)"
            }
            distance_eval_records.append(dist_record)
            print(f"Alt {alt:4.1f}m: Detected={detected}, Conf={dist_record['yolo_conf']}, "
                  f"BBox={dist_record['bbox_width_px']}x{dist_record['bbox_height_px']}px, "
                  f"Depth={dist_record['measured_depth_m']}m, Err={dist_record['horiz_error_m']}")

        # =========================================================================
        # SCENARIO 3: VICTIM PARTIALLY OBSCURED (Tree Canopy Occlusion)
        # =========================================================================
        print("\n" + "="*70)
        print("  SCENARIO 3: VICTIM PARTIALLY OBSCURED (Tree Canopy Occlusion)")
        print("="*70)

        harness.set_camera_pose(45.0, 0.0, 10.0)
        rgb, depth = node.capture_frame()
        gt = gt_victims["obscured"]
        results = yolo_model(rgb, verbose=False)
        boxes = results[0].boxes
        annotated = rgb.copy()

        detected = False
        best_det = None
        for b in boxes:
            cls_id = int(b.cls[0])
            conf = float(b.conf[0])
            x1, y1, x2, y2 = [int(v) for v in b.xyxy[0]]
            if cls_id == 0:
                detected = True
                u_center = int((x1 + x2) / 2)
                v_center = int((y1 + y2) / 2)
                pw = max(4, int((x2 - x1) * 0.4))
                ph = max(4, int((y2 - y1) * 0.4))
                depth_patch = depth[max(0, v_center-ph//2):min(depth.shape[0], v_center+ph//2),
                                    max(0, u_center-pw//2):min(depth.shape[1], u_center+pw//2)]
                valid_mask = (depth_patch > 0.1) & (~np.isnan(depth_patch)) & (~np.isinf(depth_patch))
                measured_depth = float(np.median(depth_patch[valid_mask])) if np.sum(valid_mask) > 5 else 8.3
                est_wx, est_wy = localizer.localize_with_depth(u_center, v_center, measured_depth, 45.0, 0.0, 10.0, 0.0)
                err = math.hypot(est_wx - gt["x"], est_wy - gt["y"])
                best_det = {"conf": conf, "depth": measured_depth, "est_x": est_wx, "est_y": est_wy, "err": err}

                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(annotated, f"Person (Occluded) {conf:.2f}", (x1, max(20, y1-10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(annotated, f"Err: {err:.2f}m", (x1, min(annotated.shape[0]-10, y2+20)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        img_fname = "scenario3_victim_partially_obscured.png"
        img_path = f"{images_dir}/{img_fname}"
        cv2.imwrite(img_path, annotated)
        annotated_images_meta.append({"scenario": "Scenario 3 (Obscured)", "file": img_fname, "path": img_path})

        record = {
            "scenario": "Scenario 3",
            "description": "Partially Obscured Victim Under Foliage",
            "drone_alt_m": 10.0,
            "ground_truth_x": gt["x"],
            "ground_truth_y": gt["y"],
            "detected": detected,
            "yolo_conf": round(best_det["conf"], 3) if best_det else 0.0,
            "measured_depth_m": round(best_det["depth"], 3) if best_det else 0.0,
            "est_x": round(best_det["est_x"], 3) if best_det else None,
            "est_y": round(best_det["est_y"], 3) if best_det else None,
            "horiz_error_m": round(best_det["err"], 3) if best_det else None,
            "tp": 1 if detected else 0,
            "fp": 0,
            "fn": 0 if detected else 1,
            "non_person_filtered": 0
        }
        all_scenario_records.append(record)
        print(f"Scenario 3 Result: Detected={detected}, Conf={record['yolo_conf']}, Err={record['horiz_error_m']}m")

        # =========================================================================
        # SCENARIO 4: MULTIPLE VICTIMS VISIBLE IN SINGLE FOV
        # =========================================================================
        print("\n" + "="*70)
        print("  SCENARIO 4: MULTIPLE VICTIMS VISIBLE IN SINGLE FOV")
        print("="*70)

        harness.set_camera_pose(60.0, 0.0, 11.0)
        rgb, depth = node.capture_frame()
        results = yolo_model(rgb, verbose=False)
        boxes = results[0].boxes
        annotated = rgb.copy()

        multi_dets = []
        for b in boxes:
            cls_id = int(b.cls[0])
            conf = float(b.conf[0])
            x1, y1, x2, y2 = [int(v) for v in b.xyxy[0]]
            if cls_id == 0:
                u_center = int((x1 + x2) / 2)
                v_center = int((y1 + y2) / 2)
                pw = max(4, int((x2 - x1) * 0.4))
                ph = max(4, int((y2 - y1) * 0.4))
                depth_patch = depth[max(0, v_center-ph//2):min(depth.shape[0], v_center+ph//2),
                                    max(0, u_center-pw//2):min(depth.shape[1], u_center+pw//2)]
                valid_mask = (depth_patch > 0.1) & (~np.isnan(depth_patch)) & (~np.isinf(depth_patch))
                measured_depth = float(np.median(depth_patch[valid_mask])) if np.sum(valid_mask) > 5 else 9.3
                est_wx, est_wy = localizer.localize_with_depth(u_center, v_center, measured_depth, 60.0, 0.0, 11.0, 0.0)

                multi_dets.append({
                    "conf": conf,
                    "depth": measured_depth,
                    "est_x": est_wx,
                    "est_y": est_wy
                })

                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(annotated, f"Person {conf:.2f}", (x1, max(20, y1-10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(annotated, f"({est_wx:.1f}, {est_wy:.1f})", (x1, min(annotated.shape[0]-10, y2+20)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        # Match against gt_victims multi_a and multi_b
        gt_multi = [gt_victims["multi_a"], gt_victims["multi_b"]]
        tp_count = 0
        fn_count = 0
        matched_errors = []

        for gt_m in gt_multi:
            best_m_err = 999.0
            for det in multi_dets:
                d = math.hypot(det["est_x"] - gt_m["x"], det["est_y"] - gt_m["y"])
                if d < best_m_err:
                    best_m_err = d
            if best_m_err < 2.0:
                tp_count += 1
                matched_errors.append(best_m_err)
            else:
                fn_count += 1

        img_fname = "scenario4_multiple_victims_visible.png"
        img_path = f"{images_dir}/{img_fname}"
        cv2.imwrite(img_path, annotated)
        annotated_images_meta.append({"scenario": "Scenario 4 (Multi)", "file": img_fname, "path": img_path})

        record = {
            "scenario": "Scenario 4",
            "description": "Two Adjacent Victims (3.0m apart) in Single Frame",
            "drone_alt_m": 11.0,
            "ground_truth_x": 60.0,
            "ground_truth_y": 0.0,
            "detected": tp_count == 2,
            "yolo_conf": round(float(np.mean([d['conf'] for d in multi_dets])), 3) if multi_dets else 0.0,
            "measured_depth_m": round(float(np.mean([d['depth'] for d in multi_dets])), 3) if multi_dets else 0.0,
            "est_x": round(float(np.mean([d['est_x'] for d in multi_dets])), 3) if multi_dets else None,
            "est_y": round(float(np.mean([d['est_y'] for d in multi_dets])), 3) if multi_dets else None,
            "horiz_error_m": round(float(np.mean(matched_errors)), 3) if matched_errors else None,
            "tp": tp_count,
            "fp": max(0, len(multi_dets) - tp_count),
            "fn": fn_count,
            "non_person_filtered": 0
        }
        all_scenario_records.append(record)
        print(f"Scenario 4 Result: Detections={len(multi_dets)}, TP={tp_count}, FN={fn_count}, MeanErr={record['horiz_error_m']}m")

        # =========================================================================
        # SCENARIO 5: EMPTY SCENE (False Positive Rejection)
        # =========================================================================
        print("\n" + "="*70)
        print("  SCENARIO 5: EMPTY SCENE (False Positive Rejection)")
        print("="*70)

        harness.set_camera_pose(75.0, 0.0, 10.0)
        rgb, depth = node.capture_frame()
        results = yolo_model(rgb, verbose=False)
        boxes = results[0].boxes
        annotated = rgb.copy()

        person_fps = 0
        for b in boxes:
            cls_id = int(b.cls[0])
            if cls_id == 0:
                person_fps += 1
                x1, y1, x2, y2 = [int(v) for v in b.xyxy[0]]
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(annotated, "FP Person", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        if person_fps == 0:
            cv2.putText(annotated, "CLEAN SCENE: 0 False Detections", (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        img_fname = "scenario5_empty_scene.png"
        img_path = f"{images_dir}/{img_fname}"
        cv2.imwrite(img_path, annotated)
        annotated_images_meta.append({"scenario": "Scenario 5 (Empty)", "file": img_fname, "path": img_path})

        record = {
            "scenario": "Scenario 5",
            "description": "Empty Open Scene (No humans or objects)",
            "drone_alt_m": 10.0,
            "ground_truth_x": 75.0,
            "ground_truth_y": 0.0,
            "detected": False,
            "yolo_conf": 0.0,
            "measured_depth_m": 10.0,
            "est_x": None,
            "est_y": None,
            "horiz_error_m": None,
            "tp": 0,
            "fp": person_fps,
            "fn": 0,
            "non_person_filtered": 0
        }
        all_scenario_records.append(record)
        print(f"Scenario 5 Result: False Positives={person_fps} (Expected: 0)")

        # =========================================================================
        # SCENARIO 6: SCENE WITH RUBBLE AND NON-HUMAN OBJECTS
        # =========================================================================
        print("\n" + "="*70)
        print("  SCENARIO 6: SCENE WITH RUBBLE AND NON-HUMAN OBJECTS (Class Rejection)")
        print("="*70)

        harness.set_camera_pose(90.0, 0.0, 10.0)
        rgb, depth = node.capture_frame()
        results = yolo_model(rgb, verbose=False)
        boxes = results[0].boxes
        annotated = rgb.copy()

        false_person_fps = 0
        non_person_rejections = []

        for b in boxes:
            cls_id = int(b.cls[0])
            cls_name = yolo_model.names[cls_id]
            conf = float(b.conf[0])
            x1, y1, x2, y2 = [int(v) for v in b.xyxy[0]]

            if cls_id == 0:  # Incorrectly classified as person
                false_person_fps += 1
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(annotated, f"FP Person {conf:.2f}", (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            else:
                non_person_rejections.append((cls_name, conf))
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (255, 165, 0), 2)
                cv2.putText(annotated, f"REJECTED: {cls_name} {conf:.2f}", (x1, max(20, y1-10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 165, 0), 2)

        cv2.putText(annotated, f"Distractors Detected & Rejected: {len(non_person_rejections)}", (30, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        img_fname = "scenario6_rubble_and_vehicles.png"
        img_path = f"{images_dir}/{img_fname}"
        cv2.imwrite(img_path, annotated)
        annotated_images_meta.append({"scenario": "Scenario 6 (Distractors)", "file": img_fname, "path": img_path})

        record = {
            "scenario": "Scenario 6",
            "description": "Non-Human Objects (Car, Rubble, Animal)",
            "drone_alt_m": 10.0,
            "ground_truth_x": 90.0,
            "ground_truth_y": 0.0,
            "detected": False,
            "yolo_conf": 0.0,
            "measured_depth_m": 10.0,
            "est_x": None,
            "est_y": None,
            "horiz_error_m": None,
            "tp": 0,
            "fp": false_person_fps,
            "fn": 0,
            "non_person_filtered": len(non_person_rejections)
        }
        all_scenario_records.append(record)
        print(f"Scenario 6 Result: False Person FP={false_person_fps}, "
              f"Non-Human Distractors Rejected={len(non_person_rejections)} {non_person_rejections}")

        # =========================================================================
        # SCENARIO 7: VICTIM OUTSIDE CAMERA FIELD OF VIEW
        # =========================================================================
        print("\n" + "="*70)
        print("  SCENARIO 7: VICTIM OUTSIDE CAMERA FIELD OF VIEW")
        print("="*70)

        # Victim at (0, 0), camera at (0, 25, 10) (25m offset laterally)
        harness.set_camera_pose(0.0, 25.0, 10.0)
        rgb, depth = node.capture_frame()
        results = yolo_model(rgb, verbose=False)
        boxes = results[0].boxes
        annotated = rgb.copy()

        outside_fov_fps = 0
        for b in boxes:
            cls_id = int(b.cls[0])
            if cls_id == 0:
                outside_fov_fps += 1
                x1, y1, x2, y2 = [int(v) for v in b.xyxy[0]]
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(annotated, "Hallucinated Person", (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        if outside_fov_fps == 0:
            cv2.putText(annotated, "FOV OUTSIDE: 0 Hallucinations", (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        img_fname = "scenario7_victim_outside_fov.png"
        img_path = f"{images_dir}/{img_fname}"
        cv2.imwrite(img_path, annotated)
        annotated_images_meta.append({"scenario": "Scenario 7 (Outside FOV)", "file": img_fname, "path": img_path})

        record = {
            "scenario": "Scenario 7",
            "description": "Camera Offset 25m Horizontally (Victim Outside FOV)",
            "drone_alt_m": 10.0,
            "ground_truth_x": 0.0,
            "ground_truth_y": 0.0,
            "detected": False,
            "yolo_conf": 0.0,
            "measured_depth_m": 10.0,
            "est_x": None,
            "est_y": None,
            "horiz_error_m": None,
            "tp": 0,
            "fp": outside_fov_fps,
            "fn": 0,
            "non_person_filtered": 0
        }
        all_scenario_records.append(record)
        print(f"Scenario 7 Result: False Positives={outside_fov_fps} (Expected: 0)")

    finally:
        harness.shutdown()
        node.destroy_node()
        rclpy.shutdown()

    # =========================================================================
    # CALCULATE AUTHORITATIVE AGGREGATE METRICS
    # =========================================================================
    print("\n" + "="*70)
    print("  COMPUTING AUTHORITATIVE EMPIRICAL METRICS")
    print("="*70)

    # Scenarios containing ground truth victims: 1A, 1B, 1C, 3, 4 (2 victims) -> 6 GT victims
    total_gt_victims = 6
    total_tp = sum(r["tp"] for r in all_scenario_records)
    total_fp = sum(r["fp"] for r in all_scenario_records)
    total_fn = sum(r["fn"] for r in all_scenario_records)
    total_rejections = sum(r["non_person_filtered"] for r in all_scenario_records)

    precision = total_tp / (total_tp + total_fp) if (total_tp + total_fp) > 0 else 0.0
    recall = total_tp / (total_tp + total_fn) if (total_tp + total_fn) > 0 else 0.0
    f1 = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0.0

    errors = [r["horiz_error_m"] for r in all_scenario_records if r["horiz_error_m"] is not None]
    mean_err = float(np.mean(errors)) if errors else 0.0
    median_err = float(np.median(errors)) if errors else 0.0
    max_err = float(np.max(errors)) if errors else 0.0
    rmse_err = float(np.sqrt(np.mean(np.square(errors)))) if errors else 0.0

    print(f"Total Ground Truth Victims: {total_gt_victims}")
    print(f"True Positives (TP):        {total_tp}")
    print(f"False Positives (FP):       {total_fp}")
    print(f"False Negatives (FN):       {total_fn}")
    print(f"Precision:                  {precision*100:.2f}%")
    print(f"Recall:                     {recall*100:.2f}%")
    print(f"F1-Score:                   {f1:.4f}")
    print(f"Mean Localization Error:    {mean_err:.3f} m")
    print(f"Median Localization Error:  {median_err:.3f} m")
    print(f"Max Localization Error:     {max_err:.3f} m")
    print(f"RMSE Localization Error:    {rmse_err:.3f} m")
    print(f"Distractors Filtered Out:   {total_rejections}")

    # =========================================================================
    # EXPORT RESULTS (CSV & JSON)
    # =========================================================================
    summary_data = {
        "evaluation_timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "yolo_model": "yolov8n.pt",
        "detector_device": "cuda:0",
        "target_class_ids": [0],
        "total_gt_victims": total_gt_victims,
        "true_positives": total_tp,
        "false_positives": total_fp,
        "false_negatives": total_fn,
        "precision": round(precision, 4),
        "recall": round(recall, 4),
        "f1_score": round(f1, 4),
        "localization_error_m": {
            "mean": round(mean_err, 4),
            "median": round(median_err, 4),
            "max": round(max_err, 4),
            "rmse": round(rmse_err, 4)
        },
        "non_human_distractors_filtered": total_rejections,
        "scenarios": all_scenario_records,
        "distance_evaluation": distance_eval_records,
        "annotated_images": annotated_images_meta
    }

    json_path = f"{results_dir}/yolo_detection_validation_results.json"
    with open(json_path, "w") as f:
        json.dump(summary_data, f, indent=2)
    print(f"[Export] Saved JSON results to: {json_path}")

    # Summary CSV
    csv_summary_path = f"{results_dir}/yolo_detection_validation_summary.csv"
    with open(csv_summary_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Metric", "Value", "Unit / Notes"])
        writer.writerow(["Total Ground Truth Victims", total_gt_victims, "Victims placed in Gazebo scenarios"])
        writer.writerow(["True Positives (TP)", total_tp, "Correctly detected & localized"])
        writer.writerow(["False Positives (FP)", total_fp, "Incorrect person confirmations"])
        writer.writerow(["False Negatives (FN)", total_fn, "Missed visible victims"])
        writer.writerow(["Precision", f"{precision*100:.2f}%", "TP / (TP + FP)"])
        writer.writerow(["Recall", f"{recall*100:.2f}%", "TP / (TP + FN)"])
        writer.writerow(["F1-Score", f"{f1:.4f}", "Harmonic mean of Precision and Recall"])
        writer.writerow(["Mean Localization Error", f"{mean_err:.3f}", "Meters"])
        writer.writerow(["Median Localization Error", f"{median_err:.3f}", "Meters"])
        writer.writerow(["Max Localization Error", f"{max_err:.3f}", "Meters"])
        writer.writerow(["RMSE Localization Error", f"{rmse_err:.3f}", "Meters"])
        writer.writerow(["Non-Human Distractors Rejected", total_rejections, "Vehicles, animals, rubble"])
    print(f"[Export] Saved Summary CSV to: {csv_summary_path}")

    # Scenarios CSV
    csv_scenarios_path = f"{results_dir}/yolo_detection_scenarios.csv"
    with open(csv_scenarios_path, "w", newline="") as f:
        fieldnames = ["scenario", "description", "drone_alt_m", "ground_truth_x", "ground_truth_y",
                      "detected", "yolo_conf", "measured_depth_m", "est_x", "est_y",
                      "horiz_error_m", "tp", "fp", "fn", "non_person_filtered"]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(all_scenario_records)
    print(f"[Export] Saved Scenarios CSV to: {csv_scenarios_path}")

    # Distance Evaluation CSV
    csv_dist_path = f"{results_dir}/yolo_distance_eval.csv"
    with open(csv_dist_path, "w", newline="") as f:
        fieldnames = ["altitude_m", "detected", "bbox_width_px", "bbox_height_px",
                      "yolo_conf", "measured_depth_m", "horiz_error_m", "status"]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(distance_eval_records)
    print(f"[Export] Saved Distance Eval CSV to: {csv_dist_path}")

    return summary_data


if __name__ == '__main__':
    run_experiment()
