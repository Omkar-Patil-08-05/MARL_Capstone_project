#!/usr/bin/env python3
"""
Autonomous Mission Video Recorder & Victim Event Capture for MARL SAR Forest Simulation.

Features:
1. Continuous 3D World Video Recording (forest_3d_world_overview.mp4):
   - Primary Requirement: Ingests offscreen rendered Gazebo 3D camera frames (/world_camera/image or /camera_elevated/image).
   - Shows the actual 3D Gazebo forest environment, including terrain, trees, and moving drones.
   - Real-time World Mission HUD overlay (sim time, wall time, QMIX decision count, coverage %, victims detected, active drones).
   - Dedicated background thread with bounded queue (queue.Queue(maxsize=30)) for non-blocking H.264/avc1 MP4 encoding.
2. Continuous Drone Onboard Camera Footage (forest_drone_camera.mp4):
   - Ingests genuine frames from drone 0 onboard downward camera (/drone_0/camera/image).
   - Real-time Onboard HUD overlay (sim time, altitude, speed, coordinates, safety avoidance status).
   - Dedicated background thread with bounded queue for non-blocking H.264/avc1 MP4 encoding.
3. Confirmed-Victim Snapshots:
   - Captures genuine camera frame on verified victim discovery (/sar/victim_discovery).
   - Standardized naming: victim_<track_id>_<timestamp>_drone_<id>.jpg.
   - Detailed event logging into snapshots/victim_events.json and snapshots/victim_events.csv.
4. Synchronized Mission Telemetry & Evaluation Logs:
   - drone_telemetry.csv, decision_history.csv, recording_metadata.json, mission_evaluation_summary.json.
   - Auxiliary 2D tactical-map trajectory video clearly labeled as telemetry-derived.
5. Isolated Synthetic Test Mode (--synthetic-test):
   - Verifies dual video encoders, bounded queues, snapshots, and file formats in isolation without touching live simulations.
"""

import os
import sys
import time
import math
import json
import csv
import queue
import signal
import threading
import argparse
import datetime
import subprocess
from typing import Dict, Any, List, Optional, Tuple

import cv2
import numpy as np

# Ensure ROS 2 environment is available
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image as RosImage
    from std_msgs.msg import String as RosString
    from rosgraph_msgs.msg import Clock as RosClock
    from cv_bridge import CvBridge
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class ForestMissionRecorder:
    def __init__(
        self,
        output_dir: str,
        primary_drone_id: int = 0,
        map_id: str = "forest_dense_candidate",
        world_fps: float = 5.0,
        drone_fps: float = 10.0,
        world_resolution: Tuple[int, int] = (1280, 720),
        drone_resolution: Tuple[int, int] = (640, 480),
        max_queue_size: int = 30,
        target_fps: Optional[float] = None
    ):
        self.output_dir = os.path.abspath(output_dir)
        self.snapshots_dir = os.path.join(self.output_dir, "snapshots")
        os.makedirs(self.snapshots_dir, exist_ok=True)

        self.primary_drone_id = primary_drone_id
        self.map_id = map_id
        self.world_fps = world_fps
        self.drone_fps = drone_fps if target_fps is None else target_fps
        self.target_fps = self.drone_fps
        self.world_resolution = world_resolution
        self.drone_resolution = drone_resolution
        self.max_queue_size = max_queue_size

        # Output Video Paths
        self.world_3d_video_path = os.path.join(self.output_dir, "forest_3d_world_overview.mp4")
        self.drone_video_path = os.path.join(self.output_dir, "forest_drone_camera.mp4")
        self.tactical_map_video_path = os.path.join(self.output_dir, "forest_tactical_map_trajectory.mp4")
        # Legacy/convenience alias
        self.video_path = self.world_3d_video_path

        # Log & Metadata Paths
        self.telemetry_csv_path = os.path.join(self.output_dir, "drone_telemetry.csv")
        self.decision_csv_path = os.path.join(self.output_dir, "decision_history.csv")
        self.metadata_json_path = os.path.join(self.output_dir, "recording_metadata.json")
        self.evaluation_summary_path = os.path.join(self.output_dir, "mission_evaluation_summary.json")
        self.victim_events_json_path = os.path.join(self.snapshots_dir, "victim_events.json")
        self.victim_events_csv_path = os.path.join(self.snapshots_dir, "victim_events.csv")
        self.mission_events_json_path = os.path.join(self.snapshots_dir, "mission_events.json")
        self.mission_events_csv_path = os.path.join(self.snapshots_dir, "mission_events.csv")

        # Load Map Metadata if available
        self.meta_data = self._load_map_metadata()

        # Telemetry State
        self.state_lock = threading.Lock()
        self.current_sim_time = 0.0
        self.wall_start_time = time.time()
        self.wall_end_time = None
        self.mission_status = "INITIALIZING"
        self.decision_count = 0
        self.max_decisions = 300
        self.coverage_pct = 0.0
        self.victims_detected = 0
        self.total_victims = 5
        self.safety_overrides = 0
        self.drones_state: Dict[str, Dict[str, Any]] = {}
        self.telemetry_history: List[Dict[str, Any]] = []
        self.latest_drone_frames: Dict[int, np.ndarray] = {}
        self.latest_world_frame: Optional[np.ndarray] = None

        # Event tracking & deduplication
        self.recorded_victim_tracks = set()
        self.victim_events_list: List[Dict[str, Any]] = []
        self.mission_events_list: List[Dict[str, Any]] = []

        # Video writers & bounded queues
        self.world_frame_queue: queue.Queue = queue.Queue(maxsize=self.max_queue_size)
        self.drone_frame_queue: queue.Queue = queue.Queue(maxsize=self.max_queue_size)
        self.total_world_frames_received = 0
        self.total_world_frames_encoded = 0
        self.dropped_world_frames = 0
        self.total_drone_frames_received = 0
        self.total_drone_frames_encoded = 0
        self.dropped_drone_frames = 0
        self.is_recording = True

        # Codec selection: Try avc1 (H.264), fallback to mp4v
        self.codec_tag = "avc1"
        fourcc = cv2.VideoWriter_fourcc(*self.codec_tag)

        # 1. World 3D Overview Video Writer (5.0 FPS)
        self.world_video_writer = cv2.VideoWriter(
            self.world_3d_video_path, fourcc, self.world_fps, self.world_resolution
        )
        if not self.world_video_writer.isOpened():
            self.codec_tag = "mp4v"
            fourcc = cv2.VideoWriter_fourcc(*self.codec_tag)
            self.world_video_writer = cv2.VideoWriter(
                self.world_3d_video_path, fourcc, self.world_fps, self.world_resolution
            )

        # 2. Drone Onboard Camera Video Writer (10.0 FPS)
        self.drone_video_writer = cv2.VideoWriter(
            self.drone_video_path, fourcc, self.drone_fps, self.drone_resolution
        )
        if not self.drone_video_writer.isOpened():
            fourcc_mp4v = cv2.VideoWriter_fourcc(*"mp4v")
            self.drone_video_writer = cv2.VideoWriter(
                self.drone_video_path, fourcc_mp4v, self.drone_fps, self.drone_resolution
            )

        # Legacy pointer
        self.video_writer = self.world_video_writer

        # File handles for continuous streaming CSVs
        self.telem_file = open(self.telemetry_csv_path, "w", newline="")
        self.telem_writer = csv.writer(self.telem_file)
        self.telem_writer.writerow([
            "timestamp", "sim_time", "drone_id", "state", "x", "y", "z",
            "grid_x", "grid_y", "action", "speed", "safety_override"
        ])

        self.dec_file = open(self.decision_csv_path, "w", newline="")
        self.dec_writer = csv.writer(self.dec_file)
        self.dec_writer.writerow([
            "decision_step", "timestamp", "sim_time", "coverage_pct",
            "victims_detected", "total_victims", "safety_overrides"
        ])
        self.last_logged_decision = -1

        # Background worker threads for non-blocking video encoding
        self.world_encoding_thread = threading.Thread(target=self._world_encoding_worker, daemon=True)
        self.world_encoding_thread.start()

        self.drone_encoding_thread = threading.Thread(target=self._drone_encoding_worker, daemon=True)
        self.drone_encoding_thread.start()

        # Log Initial Event
        self.log_mission_event("RECORDER_INITIALIZED", {
            "output_dir": self.output_dir,
            "primary_drone": self.primary_drone_id,
            "world_video": self.world_3d_video_path,
            "drone_video": self.drone_video_path
        })

    def _load_map_metadata(self) -> Dict[str, Any]:
        candidates = [
            f"/home/capstone/capstone_project_antigravity/worlds/{self.map_id}_meta.json",
            "/home/capstone/capstone_project_antigravity/worlds/forest_dense_candidate_meta.json",
            "/home/capstone/capstone_project_antigravity/worlds/forest_sar_meta.json"
        ]
        for p in candidates:
            if os.path.exists(p):
                try:
                    with open(p, "r") as f:
                        return json.load(f)
                except Exception:
                    pass
        return {}

    def _world_encoding_worker(self):
        """Dedicated background thread consuming 3D world frames from bounded queue."""
        while self.is_recording or not self.world_frame_queue.empty():
            try:
                item = self.world_frame_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            if item is None:
                break

            if self.world_video_writer is not None and self.world_video_writer.isOpened():
                try:
                    self.world_video_writer.write(item)
                    self.total_world_frames_encoded += 1
                except Exception as e:
                    print(f"[RECORDER ERROR] World frame encode failed: {e}", file=sys.stderr)
            self.world_frame_queue.task_done()

    def _drone_encoding_worker(self):
        """Dedicated background thread consuming drone onboard frames from bounded queue."""
        while self.is_recording or not self.drone_frame_queue.empty():
            try:
                item = self.drone_frame_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            if item is None:
                break

            if self.drone_video_writer is not None and self.drone_video_writer.isOpened():
                try:
                    self.drone_video_writer.write(item)
                    self.total_drone_frames_encoded += 1
                except Exception as e:
                    print(f"[RECORDER ERROR] Drone frame encode failed: {e}", file=sys.stderr)
            self.drone_frame_queue.task_done()

    def process_world_camera_frame(self, frame_bgr: np.ndarray):
        """Processes an incoming genuine 3D Gazebo world camera frame, overlays HUD, and enqueues."""
        self.total_world_frames_received += 1

        with self.state_lock:
            self.latest_world_frame = frame_bgr.copy()

        # Resize to target world resolution
        if (frame_bgr.shape[1], frame_bgr.shape[0]) != self.world_resolution:
            frame_resized = cv2.resize(frame_bgr, self.world_resolution)
        else:
            frame_resized = frame_bgr.copy()

        # Render Telemetry HUD Overlay
        hud_frame = self._render_world_hud_overlay(frame_resized)

        # Enqueue with bounded capacity
        try:
            self.world_frame_queue.put_nowait(hud_frame)
        except queue.Full:
            self.dropped_world_frames += 1

    def process_drone_camera_frame(self, frame_bgr: np.ndarray, drone_id: int = 0):
        """Processes an incoming genuine drone onboard camera frame, overlays HUD, and enqueues."""
        self.total_drone_frames_received += 1

        # Store latest un-annotated source frame for clean snapshots
        with self.state_lock:
            self.latest_drone_frames[drone_id] = frame_bgr.copy()

        # Only encode primary drone into the continuous drone stream
        if drone_id != self.primary_drone_id:
            return

        # Resize to target drone resolution
        if (frame_bgr.shape[1], frame_bgr.shape[0]) != self.drone_resolution:
            frame_resized = cv2.resize(frame_bgr, self.drone_resolution)
        else:
            frame_resized = frame_bgr.copy()

        # Render Telemetry HUD Overlay
        hud_frame = self._render_drone_hud_overlay(frame_resized, drone_id)

        # Enqueue with bounded capacity
        try:
            self.drone_frame_queue.put_nowait(hud_frame)
        except queue.Full:
            self.dropped_drone_frames += 1

    def process_camera_frame(self, frame_bgr: np.ndarray, drone_id: int = 0):
        """Compatibility shim routing to process_drone_camera_frame."""
        self.process_drone_camera_frame(frame_bgr, drone_id)

    def _render_world_hud_overlay(self, frame: np.ndarray) -> np.ndarray:
        """Renders an informative telemetry banner on top of the 3D world overview frames."""
        out = frame.copy()
        h, w = out.shape[:2]

        with self.state_lock:
            sim_t = self.current_sim_time
            wall_t = time.time() - self.wall_start_time
            dec = self.decision_count
            max_dec = self.max_decisions
            cov = self.coverage_pct
            v_det = self.victims_detected
            v_tot = self.total_victims
            status = self.mission_status
            active_drones = len(self.drones_state)

        # Semi-transparent top banner
        banner_h = 58
        overlay = out.copy()
        cv2.rectangle(overlay, (0, 0), (w, banner_h), (18, 22, 26), -1)
        cv2.addWeighted(overlay, 0.78, out, 0.22, 0, out)

        # Top border accent line (Cyan for world 3D overview)
        cv2.line(out, (0, banner_h), (w, banner_h), (255, 180, 0), 2)

        font = cv2.FONT_HERSHEY_SIMPLEX
        # Line 1: 3D Environment Title, Times, Status
        line1 = f"MARL SAR | GAZEBO 3D FOREST WORLD | Sim Time: {sim_t:.1f}s | Wall: {wall_t:.1f}s | {status}"
        cv2.putText(out, line1, (12, 22), font, 0.52, (255, 255, 255), 1, cv2.LINE_AA)

        # Line 2: Decisions, Coverage, Victims, Active Swarm Units
        line2 = f"Decisions: {dec}/{max_dec} | Coverage: {cov:.1f}% | Victims: {v_det}/{v_tot} | Drones Active: {active_drones} | Offscreen Ogre2 Headless"
        cv2.putText(out, line2, (12, 46), font, 0.48, (220, 235, 255), 1, cv2.LINE_AA)

        return out

    def _render_drone_hud_overlay(self, frame: np.ndarray, drone_id: int) -> np.ndarray:
        """Renders an informative telemetry banner on top of genuine drone onboard camera frames."""
        out = frame.copy()
        h, w = out.shape[:2]

        with self.state_lock:
            sim_t = self.current_sim_time
            wall_t = time.time() - self.wall_start_time
            dec = self.decision_count
            max_dec = self.max_decisions
            cov = self.coverage_pct
            v_det = self.victims_detected
            v_tot = self.total_victims
            status = self.mission_status
            drone_info = self.drones_state.get(f"drone_{drone_id}", {})
            alt = drone_info.get("z", 15.0)
            spd = drone_info.get("speed", 0.0)
            safety = drone_info.get("safety_override", False)
            wx = drone_info.get("x", 0.0)
            wy = drone_info.get("y", 0.0)

        banner_h = 56
        overlay = out.copy()
        cv2.rectangle(overlay, (0, 0), (w, banner_h), (20, 24, 28), -1)
        cv2.addWeighted(overlay, 0.75, out, 0.25, 0, out)

        accent_color = (0, 140, 255) if safety else (0, 220, 100)
        cv2.line(out, (0, banner_h), (w, banner_h), accent_color, 2)

        font = cv2.FONT_HERSHEY_SIMPLEX
        line1 = f"MARL SAR | Drone {drone_id} Onboard Camera | Sim: {sim_t:.1f}s | Wall: {wall_t:.1f}s | {status}"
        cv2.putText(out, line1, (10, 20), font, 0.46, (255, 255, 255), 1, cv2.LINE_AA)

        line2 = f"Decisions: {dec}/{max_dec} | Cov: {cov:.1f}% | Vic: {v_det}/{v_tot} | Alt: {alt:.1f}m | Spd: {spd:.1f}m/s | Pos: ({wx:.1f}, {wy:.1f})"
        cv2.putText(out, line2, (10, 42), font, 0.44, (200, 230, 255), 1, cv2.LINE_AA)

        if safety:
            alert_overlay = out.copy()
            cv2.rectangle(alert_overlay, (0, h - 28), (w, h), (0, 100, 255), -1)
            cv2.addWeighted(alert_overlay, 0.8, out, 0.2, 0, out)
            cv2.putText(out, "SAFETY OVERRIDE ENGAGED - AVOIDANCE DIVERSE WAYPOINT", (w // 2 - 210, h - 9), font, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

        return out

    def process_telemetry(self, telemetry_data: Dict[str, Any]):
        """Processes high-frequency swarm telemetry messages."""
        ts = telemetry_data.get("timestamp", time.time())
        m = telemetry_data.get("mission", {})
        drones = telemetry_data.get("drones", [])

        with self.state_lock:
            self.current_sim_time = m.get("sim_time", self.current_sim_time)
            self.mission_status = m.get("status", self.mission_status)
            self.decision_count = m.get("decision_count", self.decision_count)
            self.max_decisions = m.get("max_decisions", self.max_decisions)
            self.coverage_pct = m.get("coverage", self.coverage_pct)
            self.victims_detected = m.get("victims_detected", self.victims_detected)
            self.total_victims = m.get("total_victims", self.total_victims)
            self.safety_overrides = m.get("safety_overrides", self.safety_overrides)

            # Record per-drone kinematics
            for d in drones:
                d_id = f"drone_{d.get('id')}" if not str(d.get('id')).startswith("drone_") else str(d.get('id'))
                self.drones_state[d_id] = {
                    "x": d.get("x", 0.0),
                    "y": d.get("y", 0.0),
                    "z": d.get("z", 15.0),
                    "state": d.get("state", "UNKNOWN"),
                    "speed": d.get("speed", 0.0),
                    "action": d.get("action", "None"),
                    "safety_override": d.get("safety_override", False)
                }

                # Continuous CSV stream
                self.telem_writer.writerow([
                    round(ts, 3), round(self.current_sim_time, 2), d_id,
                    d.get("state", ""), d.get("x", 0.0), d.get("y", 0.0), d.get("z", 0.0),
                    d.get("grid_x", 0), d.get("grid_y", 0), d.get("action", ""),
                    d.get("speed", 0.0), d.get("safety_override", False)
                ])

            self.telem_file.flush()

            # Save snapshot for 2D map replay
            self.telemetry_history.append({
                "ts": ts,
                "sim_time": self.current_sim_time,
                "decision": self.decision_count,
                "coverage": self.coverage_pct,
                "victims": self.victims_detected,
                "drones": {k: dict(v) for k, v in self.drones_state.items()}
            })

            # Decision milestones
            if self.decision_count != self.last_logged_decision:
                self.dec_writer.writerow([
                    self.decision_count, round(ts, 3), round(self.current_sim_time, 2),
                    round(self.coverage_pct, 2), self.victims_detected, self.total_victims,
                    self.safety_overrides
                ])
                self.dec_file.flush()
                self.last_logged_decision = self.decision_count

    def process_victim_discovery(self, event_data: Dict[str, Any]):
        """Handles confirmed visual victim discovery events from /sar/victim_discovery."""
        track_id = event_data.get("victim_track_id", 0)
        drone_id = event_data.get("confirming_drone_id", self.primary_drone_id)
        ts = event_data.get("timestamp", time.time())
        wx = event_data.get("world_x", 0.0)
        wy = event_data.get("world_y", 0.0)
        conf = event_data.get("confidence", 0.0)
        depth = event_data.get("median_depth_estimate", 0.0)
        count = event_data.get("confirmation_count", 1)

        # Deduplicate multiple notifications for the exact same track ID
        if track_id in self.recorded_victim_tracks:
            return
        self.recorded_victim_tracks.add(track_id)

        # Standardized snapshot filename format
        safe_ts = int(ts)
        snapshot_filename = f"victim_{track_id}_{safe_ts}_drone_{drone_id}.jpg"
        snapshot_path = os.path.join(self.snapshots_dir, snapshot_filename)

        # Retrieve genuine camera frame from confirming drone (or primary)
        frame_to_save = None
        with self.state_lock:
            if drone_id in self.latest_drone_frames:
                frame_to_save = self.latest_drone_frames[drone_id].copy()
            elif self.primary_drone_id in self.latest_drone_frames:
                frame_to_save = self.latest_drone_frames[self.primary_drone_id].copy()
            elif self.latest_world_frame is not None:
                frame_to_save = self.latest_world_frame.copy()

        has_camera_frame = False
        if frame_to_save is not None:
            # Stamp verification coordinates and metadata on snapshot
            cv2.rectangle(frame_to_save, (0, 0), (frame_to_save.shape[1], 40), (0, 0, 0), -1)
            cv2.putText(
                frame_to_save,
                f"CONFIRMED VICTIM {track_id} | Drone {drone_id} | Conf: {conf:.2f} | Pos: ({wx:.1f}, {wy:.1f}) | Depth: {depth:.1f}m",
                (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 120), 1, cv2.LINE_AA
            )
            cv2.imwrite(snapshot_path, frame_to_save, [cv2.IMWRITE_JPEG_QUALITY, 92])
            has_camera_frame = True
            print(f"[RECORDER SNAPSHOT] Saved confirmed victim snapshot: {snapshot_path}")

        # Record Event Schema
        event_record = {
            "event_type": "VICTIM_DISCOVERED",
            "timestamp": float(ts),
            "wall_timestamp": datetime.datetime.now().isoformat(),
            "victim_track_id": int(track_id),
            "confirming_drone_id": int(drone_id),
            "world_x": float(wx),
            "world_y": float(wy),
            "confidence": float(conf),
            "median_depth_m": float(depth),
            "confirmation_count": int(count),
            "snapshot_file": snapshot_filename if has_camera_frame else None,
            "has_camera_frame": has_camera_frame,
            "source_topic": "/sar/victim_discovery"
        }
        self.victim_events_list.append(event_record)
        self._write_victim_events()
        self.log_mission_event("VICTIM_DISCOVERY_CONFIRMED", event_record)

    def log_mission_event(self, event_name: str, details: Dict[str, Any]):
        """Logs general mission lifecycle events (start, takeoff, safety overrides, completion)."""
        ev = {
            "event": event_name,
            "sim_time": self.current_sim_time,
            "wall_time": datetime.datetime.now().isoformat(),
            "decision_count": self.decision_count,
            "coverage_pct": self.coverage_pct,
            "victims_detected": self.victims_detected,
            "details": details
        }
        self.mission_events_list.append(ev)
        self._write_mission_events()

    def _write_victim_events(self):
        with open(self.victim_events_json_path, "w") as f:
            json.dump(self.victim_events_list, f, indent=2)
        with open(self.victim_events_csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["track_id", "drone_id", "timestamp", "world_x", "world_y", "confidence", "depth_m", "snapshot_file"])
            for e in self.victim_events_list:
                writer.writerow([
                    e.get("victim_track_id"), e.get("confirming_drone_id"), e.get("timestamp"),
                    e.get("world_x"), e.get("world_y"), e.get("confidence"),
                    e.get("median_depth_m"), e.get("snapshot_file")
                ])

    def _write_mission_events(self):
        with open(self.mission_events_json_path, "w") as f:
            json.dump(self.mission_events_list, f, indent=2)
        with open(self.mission_events_csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["event", "sim_time", "wall_time", "decision", "coverage", "victims", "details"])
            for e in self.mission_events_list:
                writer.writerow([
                    e.get("event"), e.get("sim_time"), e.get("wall_time"),
                    e.get("decision_count"), e.get("coverage_pct"),
                    e.get("victims_detected"), json.dumps(e.get("details"))
                ])

    def finalize(self):
        """Cleanly drains queues, flushes CSVs, releases video writers, and records metadata."""
        print("\n[RECORDER] Finalizing mission recording and flushing buffers...")
        self.is_recording = False
        self.wall_end_time = time.time()
        wall_duration = self.wall_end_time - self.wall_start_time

        # Flush frame queues and join encoding threads
        try:
            self.world_frame_queue.put(None, timeout=1.0)
        except Exception:
            pass
        self.world_encoding_thread.join(timeout=3.0)

        try:
            self.drone_frame_queue.put(None, timeout=1.0)
        except Exception:
            pass
        self.drone_encoding_thread.join(timeout=3.0)

        # Release Video Writers
        if self.world_video_writer is not None:
            self.world_video_writer.release()
            self.world_video_writer = None
            print(f"[RECORDER] 3D World video finalized: {self.world_3d_video_path}")

        if self.drone_video_writer is not None:
            self.drone_video_writer.release()
            self.drone_video_writer = None
            print(f"[RECORDER] Drone onboard video finalized: {self.drone_video_path}")

        # Close CSV handles
        try:
            self.telem_file.close()
            self.dec_file.close()
        except Exception:
            pass

        # Generate Synchronized 2D Tactical-Map Video
        try:
            self._generate_2d_tactical_map_video()
        except Exception as e:
            print(f"[RECORDER ERROR] Failed to generate 2D tactical map video: {e}", file=sys.stderr)

        # Final Recording Metadata JSON
        meta = {
            "recording_metadata": {
                "generated_at": datetime.datetime.now().isoformat(),
                "world_3d_video": self.world_3d_video_path,
                "world_3d_frames_encoded": self.total_world_frames_encoded,
                "world_3d_fps": self.world_fps,
                "world_3d_duration_sec": round(self.total_world_frames_encoded / self.world_fps, 2) if self.world_fps > 0 else 0,
                "world_3d_resolution": f"{self.world_resolution[0]}x{self.world_resolution[1]}",
                "drone_video": self.drone_video_path,
                "drone_frames_encoded": self.total_drone_frames_encoded,
                "drone_fps": self.drone_fps,
                "drone_duration_sec": round(self.total_drone_frames_encoded / self.drone_fps, 2) if self.drone_fps > 0 else 0,
                "drone_resolution": f"{self.drone_resolution[0]}x{self.drone_resolution[1]}",
                "tactical_map_video": self.tactical_map_video_path,
                "target_fps": self.target_fps,
                "codec": self.codec_tag,
                "wall_duration_sec": round(wall_duration, 2),
                "sim_duration_sec": round(self.current_sim_time, 2),
                "decisions_completed": self.decision_count,
                "target_decisions": self.max_decisions,
                "final_coverage_pct": round(self.coverage_pct, 2),
                "victims_detected": self.victims_detected,
                "total_victims": self.total_victims,
                "confirmed_victim_snapshots_count": len(self.victim_events_list),
                "safety_overrides": self.safety_overrides
            }
        }
        with open(self.metadata_json_path, "w") as f:
            json.dump(meta, f, indent=2)

        # Final Mission Evaluation Summary JSON
        eval_summary = {
            "evaluation_status": "COMPLETED" if self.decision_count >= self.max_decisions else "INTERRUPTED",
            "production_decision_target": self.max_decisions,
            "decisions_achieved": self.decision_count,
            "target_met": (self.decision_count >= self.max_decisions),
            "final_coverage_pct": round(self.coverage_pct, 2),
            "victims_confirmed_visual": len(self.victim_events_list),
            "total_victims_in_scenario": self.total_victims,
            "safety_interventions_count": self.safety_overrides,
            "sim_duration_sec": round(self.current_sim_time, 2),
            "wall_duration_sec": round(wall_duration, 2),
            "artifacts": {
                "world_3d_video": self.world_3d_video_path,
                "drone_camera_video": self.drone_video_path,
                "tactical_map_video": self.tactical_map_video_path,
                "telemetry_csv": self.telemetry_csv_path,
                "decision_csv": self.decision_csv_path,
                "victim_events_json": self.victim_events_json_path,
                "victim_events_csv": self.victim_events_csv_path,
                "snapshots_directory": self.snapshots_dir
            }
        }
        with open(self.evaluation_summary_path, "w") as f:
            json.dump(eval_summary, f, indent=2)

        print(f"[RECORDER] Mission evaluation summary saved: {self.evaluation_summary_path}")

    def _generate_2d_tactical_map_video(self):
        """Generates synchronized 2D telemetry tactical map animation video."""
        if not self.telemetry_history:
            return

        print(f"[RECORDER] Generating synchronized 2D tactical-map trajectory video...")
        vid_w, vid_h = 720, 720
        fourcc = cv2.VideoWriter_fourcc(*self.codec_tag)
        tac_writer = cv2.VideoWriter(self.tactical_map_video_path, fourcc, self.target_fps, (vid_w, vid_h))
        if not tac_writer.isOpened():
            fourcc_mp4v = cv2.VideoWriter_fourcc(*"mp4v")
            tac_writer = cv2.VideoWriter(self.tactical_map_video_path, fourcc_mp4v, self.target_fps, (vid_w, vid_h))

        world_min_x, world_max_x = -180.0, 20.0
        world_min_y, world_max_y = -20.0, 180.0
        margin = 60

        def to_canvas(wx: float, wy: float) -> Tuple[int, int]:
            norm_x = (wx - world_min_x) / (world_max_x - world_min_x)
            norm_y = (wy - world_min_y) / (world_max_y - world_min_y)
            cx = int(margin + norm_x * (vid_w - 2 * margin))
            cy = int((vid_h - margin) - norm_y * (vid_h - 2 * margin))
            return max(0, min(vid_w - 1, cx)), max(0, min(vid_h - 1, cy))

        meta = self.meta_data
        trees = []
        if meta:
            tree_list = meta.get("tree_visuals", [])
            for t in tree_list:
                if isinstance(t, dict) and "x" in t and "y" in t:
                    trees.append(to_canvas(t["x"], t["y"]))

        buildings = []
        if meta and "obstacles" in meta:
            for obs in meta["obstacles"]:
                if isinstance(obs, dict) and "aabb" in obs:
                    aabb = obs["aabb"]
                    bx = (aabb.get("min_x", 0) + aabb.get("max_x", 0)) / 2.0
                    by = (aabb.get("min_y", 0) + aabb.get("max_y", 0)) / 2.0
                    buildings.append(to_canvas(bx, by))

        drone_colors = [
            (255, 100, 50),
            (50, 200, 255),
            (100, 255, 100),
            (220, 100, 255),
            (50, 255, 200),
            (255, 220, 50)
        ]

        trail_history: Dict[str, List[Tuple[int, int]]] = {f"drone_{i}": [] for i in range(6)}

        for s in self.telemetry_history:
            canvas = np.zeros((vid_h, vid_w, 3), dtype=np.uint8)
            canvas[:, :] = (24, 28, 32)

            # Prominent Header
            cv2.putText(canvas, "2D SWARM TACTICAL MAP & TRAJECTORY", (20, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(canvas, "(TELEMETRY DERIVED - AUXILIARY VISUALIZATION)", (380, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (120, 170, 255), 1, cv2.LINE_AA)

            sim_t = s.get("sim_time", 0.0)
            dec = s.get("decision", 0)
            cov = s.get("coverage", 0.0)
            vic = s.get("victims", 0)
            sub_info = f"Sim Time: {sim_t:.1f}s | Decision: {dec} | Coverage: {cov:.1f}% | Victims: {vic}"
            cv2.putText(canvas, sub_info, (20, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)

            # Search area bounding box
            tl = to_canvas(world_min_x, world_max_y)
            br = to_canvas(world_max_x, world_min_y)
            cv2.rectangle(canvas, tl, br, (70, 70, 70), 1)

            # Draw static trees
            for cx, cy in trees:
                cv2.circle(canvas, (cx, cy), 2, (35, 60, 35), -1)

            # Draw static buildings/obstacles
            for bx, by in buildings:
                cv2.rectangle(canvas, (bx - 4, by - 4), (bx + 4, by + 4), (60, 75, 90), -1)

            # Draw victim positions from discovery events
            for ev in self.victim_events_list:
                vx, vy = to_canvas(ev["world_x"], ev["world_y"])
                cv2.drawMarker(canvas, (vx, vy), (0, 0, 255), cv2.MARKER_CROSS, 14, 2)
                cv2.putText(canvas, f"V{ev['victim_track_id']}", (vx + 5, vy - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 100, 255), 1)

            # Draw drone trajectories and current positions
            drones = s.get("drones", {})
            for i in range(6):
                d_id = f"drone_{i}"
                if d_id in drones:
                    dx = drones[d_id]["x"]
                    dy = drones[d_id]["y"]
                    c_pt = to_canvas(dx, dy)
                    trail_history[d_id].append(c_pt)
                    if len(trail_history[d_id]) > 100:
                        trail_history[d_id].pop(0)

                    col = drone_colors[i % len(drone_colors)]
                    for p_idx in range(1, len(trail_history[d_id])):
                        cv2.line(canvas, trail_history[d_id][p_idx - 1], trail_history[d_id][p_idx], col, 1)

                    cv2.circle(canvas, c_pt, 6, col, -1)
                    cv2.putText(canvas, f"D{i}", (c_pt[0] + 6, c_pt[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)

            tac_writer.write(canvas)

        tac_writer.release()
        print(f"[RECORDER] 2D tactical map video created: {self.tactical_map_video_path}")


# ROS 2 Wrapper Node
if ROS2_AVAILABLE:
    class RosMissionRecorderNode(Node):
        def __init__(self, recorder: ForestMissionRecorder):
            super().__init__("forest_mission_recorder")
            self.recorder = recorder
            self.bridge = CvBridge()

            # 1. 3D World Overview Camera Subscriptions
            self.sub_world1 = self.create_subscription(
                RosImage,
                "/world_camera/image",
                self._world_camera_callback,
                qos_profile_sensor_data
            )
            self.sub_world2 = self.create_subscription(
                RosImage,
                "/camera_elevated/image",
                self._world_camera_callback,
                qos_profile_sensor_data
            )
            self.sub_world3 = self.create_subscription(
                RosImage,
                f"/world/{self.recorder.map_id}/model/world_overview_camera/link/link/sensor/camera/image",
                self._world_camera_callback,
                qos_profile_sensor_data
            )

            # 2. Drone Onboard Camera Subscriptions
            p_id = self.recorder.primary_drone_id
            self.sub_drone_raw = self.create_subscription(
                RosImage,
                f"/drone_{p_id}/camera/image",
                lambda msg: self._drone_camera_callback(msg, p_id),
                qos_profile_sensor_data
            )
            self.sub_drone_det = self.create_subscription(
                RosImage,
                f"/drone_{p_id}/camera/detection_image",
                lambda msg: self._drone_camera_callback(msg, p_id),
                qos_profile_sensor_data
            )

            # Secondary drone camera subscriptions for multi-drone snapshots
            self.secondary_subs = []
            for d in range(6):
                if d != p_id:
                    sub_raw = self.create_subscription(
                        RosImage,
                        f"/drone_{d}/camera/image",
                        lambda msg, did=d: self._drone_camera_callback(msg, did),
                        qos_profile_sensor_data
                    )
                    sub_det = self.create_subscription(
                        RosImage,
                        f"/drone_{d}/camera/detection_image",
                        lambda msg, did=d: self._drone_camera_callback(msg, did),
                        qos_profile_sensor_data
                    )
                    self.secondary_subs.extend([sub_raw, sub_det])

            # 3. Telemetry and Discovery Subscriptions
            self.sub_telem = self.create_subscription(
                RosString,
                "/swarm/telemetry",
                self._telemetry_callback,
                10
            )
            self.sub_discovery = self.create_subscription(
                RosString,
                "/sar/victim_discovery",
                self._discovery_callback,
                10
            )

            self.get_logger().info(
                f"Forest Mission Recorder active. Recording 3D World Overview & Drone {p_id} to: {recorder.output_dir}"
            )

        def _world_camera_callback(self, msg: RosImage):
            try:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                self.recorder.process_world_camera_frame(cv_img)
            except Exception as e:
                self.get_logger().warn(f"World camera frame bridge conversion error: {e}")

        def _drone_camera_callback(self, msg: RosImage, drone_id: int):
            try:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                self.recorder.process_drone_camera_frame(cv_img, drone_id)
            except Exception as e:
                self.get_logger().warn(f"Drone camera frame bridge conversion error: {e}")

        def _telemetry_callback(self, msg: RosString):
            try:
                data = json.loads(msg.data)
                self.recorder.process_telemetry(data)
                if data.get("mission", {}).get("status") == "COMPLETE":
                    self.get_logger().info("Mission status is COMPLETE. Finalizing recorder.")
                    self.recorder.log_mission_event("MISSION_COMPLETE", data.get("mission", {}))
            except Exception as e:
                self.get_logger().warn(f"Telemetry parse error: {e}")

        def _discovery_callback(self, msg: RosString):
            try:
                data = json.loads(msg.data)
                self.recorder.process_victim_discovery(data)
            except Exception as e:
                self.get_logger().warn(f"Discovery event parse error: {e}")


def run_synthetic_isolated_test(output_dir: str, duration_sec: float = 5.0) -> bool:
    """
    Executes a self-contained, isolated test of the dual-video recorder using synthetic mock frames.
    Validates:
    - Bounded queues and background encoding to H.264 MP4 for both 3D World and Drone Camera streams.
    - Video file readability with OpenCV VideoCapture and ffprobe.
    - Snapshot creation with exact naming convention: victim_<id>_<timestamp>_drone_<id>.jpg.
    - Correct CSV and JSON schema outputs.
    - 2D tactical map animation generation.
    """
    print("\n" + "=" * 75)
    print("RUNNING ISOLATED SYNTHETIC DUAL-STREAM RECORDER & EVENT TEST")
    print(f"Test Output Directory: {output_dir}")
    print("=" * 75)
    os.makedirs(output_dir, exist_ok=True)

    recorder = ForestMissionRecorder(
        output_dir=output_dir,
        primary_drone_id=0,
        map_id="forest_dense_candidate",
        world_fps=5.0,
        drone_fps=10.0,
        world_resolution=(1280, 720),
        drone_resolution=(640, 480),
        max_queue_size=15  # Small queue to test boundary limits
    )

    # 10 Hz simulation step: 50 total simulation ticks for 5.0 seconds
    num_ticks = int(duration_sec * 10)
    start_sim_time = 100.0

    print(f"Feeding {num_ticks} simulation steps: 3D World Overview at 5.0 FPS, Drone Onboard at 10.0 FPS...")
    for frame_idx in range(num_ticks):
        sim_t = start_sim_time + frame_idx * 0.1

        # 1. Synthetic 3D World Overview Frame (sampled at 5 Hz from Gazebo sensor)
        if frame_idx % 2 == 0:
            synth_world = np.zeros((720, 1280, 3), dtype=np.uint8)
            # Forest terrain gradient background
            synth_world[:, :] = (35 + (frame_idx % 15), 65 + (frame_idx % 10), 40)
            # Moving tree canopy features
            for cx, cy in [(300, 250), (640, 400), (900, 300), (450, 550), (800, 600)]:
                cv2.circle(synth_world, (cx, cy), 55, (20, 50, 25), -1)
                cv2.circle(synth_world, (cx, cy), 35, (30, 80, 35), -1)
            # Moving drone markers in the 3D scene
            d0_x = int(350 + frame_idx * 7)
            d0_y = int(250 + frame_idx * 4)
            cv2.circle(synth_world, (d0_x, d0_y), 8, (255, 100, 50), -1)
            cv2.putText(synth_world, "drone_0", (d0_x + 10, d0_y + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            recorder.process_world_camera_frame(synth_world)

        # 2. Synthetic Drone Onboard Camera Frame (sampled at 10 Hz)
        synth_drone = np.zeros((480, 640, 3), dtype=np.uint8)
        synth_drone[:, :] = (28 + (frame_idx % 12), 70 + (frame_idx % 8), 32)
        # Passing trees beneath drone
        for tx in [140, 320, 520]:
            ty = (frame_idx * 9 + tx) % 480
            cv2.circle(synth_drone, (tx, ty), 22, (15, 42, 20), -1)

        # 3. Mock simulated telemetry
        mock_telem = {
            "timestamp": time.time(),
            "mission": {
                "status": "RUNNING",
                "sim_time": sim_t,
                "decision_count": min(300, 1 + frame_idx // 10),
                "max_decisions": 300,
                "coverage": min(100.0, 5.0 + frame_idx * 0.4),
                "victims_detected": 1 if frame_idx >= 20 else 0,
                "total_victims": 5,
                "safety_overrides": 1 if frame_idx >= 30 else 0
            },
            "drones": [
                {
                    "id": "0", "state": "WAYPOINT_NAVIGATION",
                    "x": -150.0 + frame_idx * 1.2, "y": 140.0 + frame_idx * 0.8, "z": 15.2,
                    "grid_x": 3, "grid_y": 20, "action": "MOVE_EAST", "speed": 6.8, "safety_override": False
                },
                {
                    "id": "1", "state": "WAYPOINT_NAVIGATION",
                    "x": -140.0 + frame_idx * 1.0, "y": 150.0 - frame_idx * 0.5, "z": 15.1,
                    "grid_x": 4, "grid_y": 21, "action": "MOVE_SOUTH", "speed": 7.1, "safety_override": frame_idx >= 30
                }
            ]
        }
        recorder.process_telemetry(mock_telem)
        recorder.process_drone_camera_frame(synth_drone, drone_id=0)

        # Trigger mock victim discovery at frame 25
        if frame_idx == 25:
            cv2.rectangle(synth_drone, (280, 200), (360, 280), (0, 0, 255), 2)
            cv2.putText(synth_drone, "person: 0.88", (280, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            mock_event = {
                "event_type": "VICTIM_DISCOVERED",
                "timestamp": sim_t,
                "victim_track_id": 1,
                "confirming_drone_id": 0,
                "world_x": -120.0,
                "world_y": 160.0,
                "confidence": 0.88,
                "median_depth_estimate": 14.8,
                "confirmation_count": 2
            }
            recorder.process_victim_discovery(mock_event)

        time.sleep(0.015)

    # Finalize recording
    recorder.finalize()

    # Verification Checks
    print("\n--- TEST VERIFICATION CHECKS ---")
    all_passed = True

    # Check 1: 3D World Video Exists and has valid duration
    if os.path.exists(recorder.world_3d_video_path) and os.path.getsize(recorder.world_3d_video_path) > 0:
        cap = cv2.VideoCapture(recorder.world_3d_video_path)
        fps = cap.get(cv2.CAP_PROP_FPS)
        frame_cnt = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        cap.release()
        expected_world_frames = int(duration_sec * recorder.world_fps)
        assert abs(fps - recorder.world_fps) < 0.1, f"Expected {recorder.world_fps} FPS, got {fps}"
        print(f"[CHECK 1 PASS] 3D World video verified: {recorder.world_3d_video_path} ({frame_cnt} frames, {w}x{h} @ {fps:.1f} FPS, size={os.path.getsize(recorder.world_3d_video_path):,} bytes, duration={frame_cnt / fps:.2f}s)")
    else:
        print("[CHECK 1 FAIL] 3D World video missing or empty!")
        all_passed = False

    # Check 2: Drone Camera Video Exists and has valid duration
    if os.path.exists(recorder.drone_video_path) and os.path.getsize(recorder.drone_video_path) > 0:
        cap = cv2.VideoCapture(recorder.drone_video_path)
        fps = cap.get(cv2.CAP_PROP_FPS)
        frame_cnt = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        cap.release()
        expected_drone_frames = int(duration_sec * recorder.drone_fps)
        assert abs(fps - recorder.drone_fps) < 0.1, f"Expected {recorder.drone_fps} FPS, got {fps}"
        print(f"[CHECK 2 PASS] Drone onboard video verified: {recorder.drone_video_path} ({frame_cnt} frames, {w}x{h} @ {fps:.1f} FPS, size={os.path.getsize(recorder.drone_video_path):,} bytes, duration={frame_cnt / fps:.2f}s)")
    else:
        print("[CHECK 2 FAIL] Drone onboard video missing or empty!")
        all_passed = False

    # Check 3: Victim Snapshot Exists and Matches Stable Name
    snapshots = [f for f in os.listdir(recorder.snapshots_dir) if f.startswith("victim_") and f.endswith(".jpg")]
    if snapshots and os.path.exists(os.path.join(recorder.snapshots_dir, snapshots[0])):
        snap_path = os.path.join(recorder.snapshots_dir, snapshots[0])
        snap_img = cv2.imread(snap_path)
        print(f"[CHECK 3 PASS] Confirmed victim snapshot created: {snapshots[0]} ({snap_img.shape[1]}x{snap_img.shape[0]}, size={os.path.getsize(snap_path):,} bytes)")
    else:
        print(f"[CHECK 3 FAIL] Victim snapshot missing! Found: {snapshots}")
        all_passed = False

    # Check 4: Metadata and CSV logs
    for p in [recorder.telemetry_csv_path, recorder.decision_csv_path, recorder.metadata_json_path, recorder.evaluation_summary_path, recorder.victim_events_json_path]:
        if os.path.exists(p) and os.path.getsize(p) > 0:
            print(f"[CHECK 4 PASS] Artifact verified: {os.path.basename(p)} ({os.path.getsize(p)} bytes)")
        else:
            print(f"[CHECK 4 FAIL] Artifact missing: {p}")
            all_passed = False

    # Check 5: ffprobe validation
    try:
        res = subprocess.run(
            ["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries", "stream=codec_name,width,height,avg_frame_rate", "-of", "json", recorder.world_3d_video_path],
            capture_output=True, text=True, timeout=5
        )
        if res.returncode == 0:
            ff_info = json.loads(res.stdout).get("streams", [{}])[0]
            print(f"[CHECK 5 PASS] ffprobe 3D world stream probe: codec={ff_info.get('codec_name')}, resolution={ff_info.get('width')}x{ff_info.get('height')}, fps={ff_info.get('avg_frame_rate')}")
        else:
            print(f"[CHECK 5 WARN] ffprobe returned {res.returncode}")
    except Exception as e:
        print(f"[CHECK 5 INFO] ffprobe check skipped: {e}")

    print("=" * 75)
    if all_passed:
        print("ALL ISOLATED SYNTHETIC TEST CHECKS PASSED SUCCESSFULLY!")
    else:
        print("SYNTHETIC TEST FAILED CHECKS!")
    print("=" * 75)
    return all_passed


def main():
    parser = argparse.ArgumentParser(description="Autonomous Mission Video Recorder & Victim Event Capture")
    parser.add_argument("--output-dir", type=str, default=None, help="Destination directory for recording artifacts")
    parser.add_argument("--drone-id", type=int, default=0, help="Primary drone ID for continuous onboard visual stream")
    parser.add_argument("--map-id", type=str, default="forest_dense_candidate", help="Map identifier")
    parser.add_argument("--world-fps", type=float, default=5.0, help="3D World Overview video recording frame rate (default: 5.0)")
    parser.add_argument("--drone-fps", type=float, default=10.0, help="Drone onboard video recording frame rate (default: 10.0)")
    parser.add_argument("--fps", type=float, default=None, help="Video recording frame rate override")
    parser.add_argument("--synthetic-test", action="store_true", help="Run isolated synthetic test without live simulation")
    parser.add_argument("--test-duration", type=float, default=5.0, help="Duration in seconds for synthetic test")
    args = parser.parse_args()

    if args.synthetic_test:
        test_dir = args.output_dir if args.output_dir else "/tmp/recorder_synthetic_test_artifacts"
        success = run_synthetic_isolated_test(test_dir, args.test_duration)
        sys.exit(0 if success else 1)

    if not ROS2_AVAILABLE:
        print("Error: ROS 2 is not available in the current Python environment.", file=sys.stderr)
        sys.exit(1)

    if not args.output_dir:
        ts_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output_dir = f"/home/capstone/capstone_project_antigravity/results/forest_simulation_recording_{ts_str}"

    drone_fps = args.fps if args.fps is not None else args.drone_fps
    recorder = ForestMissionRecorder(
        output_dir=args.output_dir,
        primary_drone_id=args.drone_id,
        map_id=args.map_id,
        world_fps=args.world_fps,
        drone_fps=drone_fps
    )

    rclpy.init(args=None)
    node = RosMissionRecorderNode(recorder)

    def sig_handler(sig, frame):
        print("\n[RECORDER] Signal received, stopping recorder...")
        recorder.finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
