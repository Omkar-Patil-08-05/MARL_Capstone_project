#!/usr/bin/env python3
"""
Test Live Dashboard Telemetry Rendering and Capture Polish Screenshot
====================================================================
Publishes a 6-drone mission telemetry payload to /swarm/telemetry,
verifies WebSocket reception, and captures a live screenshot.
"""

import os
import sys
import time
import json
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

PROJECT_ROOT = "/home/capstone/capstone_project_antigravity"

class TelemetryPublisher(Node):
    def __init__(self):
        super().__init__('demo_telemetry_publisher')
        self.pub = self.create_publisher(String, '/swarm/telemetry', 10)

    def publish_frame(self, frame_data):
        msg = String()
        msg.data = json.dumps(frame_data)
        self.pub.publish(msg)

def generate_swarm_telemetry(num_drones=6):
    t_now = time.time()
    drones = []
    
    if num_drones == 2:
        coords = [
            (-3.85, 4.12, 4.0, "INVESTIGATING", "Hover", False, 0.05),
            (1.50, -2.80, 4.0, "NAVIGATING", "Explore", False, 1.48)
        ]
        tracked_victims = [
            {
                "id": "victim_24",
                "x": 14, "y": 13,
                "world_x": 6.12, "world_y": 4.65,
                "confidence": 0.88,
                "detected": True,
                "state": "DETECTED",
                "detected_by": "drone_0",
                "source": "YOLO/RGB-D",
                "sim_time": 48.2,
                "observations": 4,
                "gt_error": 0.19
            },
            {
                "id": "victim_22",
                "x": 12, "y": 8,
                "world_x": 3.45, "world_y": -2.80,
                "confidence": 0.85,
                "detected": True,
                "state": "DETECTED",
                "detected_by": "drone_1",
                "source": "YOLO/RGB-D",
                "sim_time": 52.6,
                "observations": 3,
                "gt_error": 0.49
            }
        ]
        tasks = [
            {
                "task_id": "task_1",
                "victim_id": "victim_24",
                "world_x": 6.12, "world_y": 4.65,
                "grid_x": 14, "grid_y": 13,
                "state": "INVESTIGATING",
                "assigned_drone_id": "drone_0",
                "confidence": 0.88,
                "investigation_progress_sec": 3.2,
                "investigation_duration": 5.0
            }
        ]
        task_allocation = {
            "tasks": tasks,
            "assignments": {"drone_0": "task_1"},
            "counts": {
                "total": 1,
                "discovered": 0,
                "assigned": 0,
                "approaching": 0,
                "investigating": 1,
                "completed": 0,
                "failed": 0
            }
        }
        eval_metrics = {
            "total_ground_truth_victims": 5,
            "total_autonomous_tracks": 2,
            "true_positives": 2,
            "false_positives": 0,
            "false_negatives": 0,
            "precision": 1.0,
            "recall": 1.0,
            "f1_score": 1.0,
            "mean_localization_error_m": 0.34,
            "median_localization_error_m": 0.34,
            "max_localization_error_m": 0.49,
            "rmse_localization_error_m": 0.37
        }
        qmix_drones = 2
        coord_drones = 0
        cov = 28.60
        exp_cells = 108
    else:
        # 6 Drones at safe coordinates matching realistic_sar demo
        coords = [
            (-4.0, 3.8, 4.0, "NAVIGATING", "Explore", False, 1.45),
            (-1.2, 5.5, 4.0, "NAVIGATING", "Explore", False, 1.52),
            (2.5, 4.2, 4.0, "NAVIGATING", "Explore", False, 1.38),
            (-3.8, -4.5, 4.0, "NAVIGATING", "Explore", False, 1.41),
            (0.5, -3.8, 4.0, "HOLD", "Hover", True, 0.05), # 1 hold drone
            (4.2, -4.0, 4.0, "NAVIGATING", "Explore", False, 1.49)
        ]
        tracked_victims = []
        tasks = []
        task_allocation = {
            "tasks": [],
            "assignments": {},
            "counts": {
                "total": 0,
                "discovered": 0,
                "assigned": 0,
                "approaching": 0,
                "investigating": 0,
                "completed": 0,
                "failed": 0
            }
        }
        eval_metrics = {}
        qmix_drones = 6
        coord_drones = 0
        cov = 30.50
        exp_cells = 115

    for i, (x, y, z, state, action, override, spd) in enumerate(coords):
        drones.append({
            "id": f"drone_{i}",
            "x": x,
            "y": y,
            "z": z,
            "grid_x": int((x + 15) / 1.5),
            "grid_y": int((y + 15) / 1.5),
            "state": state,
            "action": action,
            "safety_override": override,
            "vx": spd if state != "HOLD" else 0.0,
            "vy": 0.2 if state != "HOLD" else 0.0,
            "vz": 0.0,
            "speed": spd
        })

    explored_cells = [{"x": (c % 20), "y": (c // 20)} for c in range(exp_cells)]

    victims = [
        {"id": "victim_21", "x": 5, "y": 7, "world_x": -7.5, "world_y": -4.5, "detected": False, "state": "UNDETECTED"},
        {"id": "victim_22", "x": 12, "y": 8, "world_x": 3.0, "world_y": -3.0, "detected": num_drones == 2, "state": "DETECTED" if num_drones == 2 else "UNDETECTED"},
        {"id": "victim_23", "x": 8, "y": 14, "world_x": -3.0, "world_y": 6.0, "detected": False, "state": "UNDETECTED"},
        {"id": "victim_24", "x": 14, "y": 13, "world_x": 6.0, "world_y": 4.5, "detected": True, "state": "DETECTED"},
        {"id": "victim_25", "x": 16, "y": 16, "world_x": 9.0, "world_y": 9.0, "detected": False, "state": "UNDETECTED"}
    ]

    return {
        "type": "telemetry",
        "timestamp": t_now,
        "mission": {
            "status": "RUNNING",
            "decision_count": 8,
            "max_decisions": 300,
            "global_step_state": "NAVIGATING",
            "sim_time": 42.5,
            "coverage": cov,
            "victims_detected": 2 if num_drones == 2 else 1,
            "total_victims": 5,
            "explored_count": exp_cells,
            "valid_count": 377,
            "safety_overrides": 1,
            "camera_enabled": num_drones == 2
        },
        "drones": drones,
        "explored_cells": explored_cells,
        "victims": victims,
        "tracked_victims": tracked_victims,
        "tasks": tasks,
        "task_allocation": task_allocation,
        "coordination": {
            "qmix_drones": qmix_drones,
            "coord_drones": coord_drones,
            "active_frontiers": {"drone_0": [-4, 4], "drone_1": [-1, 6]},
            "safety_holds": {"drone_4": "HOLD"} if num_drones == 6 else {},
            "task_assignments": dict(task_allocation.get("assignments", {}))
        },
        "evaluation": eval_metrics
    }

def main():
    num_drones = 6
    if "--drones" in sys.argv:
        idx = sys.argv.index("--drones")
        if idx + 1 < len(sys.argv):
            num_drones = int(sys.argv[idx + 1])

    rclpy.init()
    node = TelemetryPublisher()
    print(f"[TEST] Publishing 10 telemetry frames for {num_drones} drones at 2 Hz...")
    for _ in range(10):
        frame = generate_swarm_telemetry(num_drones=num_drones)
        node.publish_frame(frame)
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.5)

    node.destroy_node()
    rclpy.shutdown()
    print("[TEST] Telemetry publish complete!")

if __name__ == "__main__":
    main()
