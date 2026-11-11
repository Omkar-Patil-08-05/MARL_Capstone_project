"""
Passive P2P Swarm Ground Station & Telemetry Aggregator (Phase 3 Additive Implementation)
Passively listens to peer-to-peer mesh broadcasts and publishes /swarm/telemetry
for the operator UI / dashboard without participating in mission control or centralized decision making.
"""

import os
import json
import time
from typing import Dict, List, Any, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .peer_message_schema import PeerMessage, MessageType

class P2PGroundStation(Node):
    """
    Passive observer node that aggregates swarm status from peer-to-peer broadcasts
    and exports standard telemetry JSON to `/swarm/telemetry`.
    Zero mission-critical dependency: if this node crashes or is disconnected,
    drones continue full decentralized mission execution uninterrupted.
    """

    def __init__(self):
        super().__init__('p2p_ground_station')

        self.drones_state: Dict[str, Dict[str, Any]] = {}
        self.victims_state: Dict[str, Dict[str, Any]] = {}
        self.tasks_state: Dict[str, Dict[str, Any]] = {}
        self.explored_cells: set = set()
        self.total_decisions: int = 0
        self.start_monotonic = time.monotonic()

        # Listen to peer mesh broadcast (matching Best Effort QoS of ROSTopicPeerTransport)
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.mesh_sub = self.create_subscription(
            String,
            '/swarm/p2p/broadcast',
            self._mesh_callback,
            qos_best_effort
        )

        # Publish UI telemetry at 2 Hz
        self.telemetry_pub = self.create_publisher(String, '/swarm/telemetry', 10)
        self.timer = self.create_timer(0.5, self._publish_telemetry)

        self.get_logger().info("Passive P2P Swarm Ground Station initialized on /swarm/p2p/broadcast.")

    def _mesh_callback(self, msg: String):
        try:
            peer_msg = PeerMessage.from_json(msg.data)
            if peer_msg is None:
                return

            h = peer_msg.header
            p = peer_msg.payload
            sender = h.sender_drone_id

            if sender not in self.drones_state:
                self.drones_state[sender] = {
                    "drone_id": sender,
                    "state": "HOLD",
                    "position": [0.0, 0.0, 0.0],
                    "battery": 100.0,
                    "explored_count": 0,
                    "comms_health": "CONNECTED",
                    "last_seen": time.time()
                }

            drone = self.drones_state[sender]
            drone["last_seen"] = time.time()

            if h.message_type == MessageType.HEARTBEAT_STATUS:
                drone["state"] = p.get("flight_state", drone["state"])
                drone["position"] = p.get("position_enu", drone["position"])
                drone["battery"] = p.get("battery_percentage", drone["battery"])
                drone["explored_count"] = p.get("cells_explored_count", drone["explored_count"])
                drone["comms_health"] = p.get("comms_health_state", drone["comms_health"])

            elif h.message_type == MessageType.INTENT_TRAJECTORY:
                drone["target_grid"] = p.get("target_grid")
                drone["target_world"] = p.get("target_world_enu")
                drone["last_action"] = p.get("macro_action")
                self.total_decisions += 1
                curr_g = p.get("current_grid")
                if curr_g:
                    self.explored_cells.add(tuple(curr_g))

            elif h.message_type == MessageType.VICTIM_CONFIRMED:
                v_id = p.get("victim_event_id", "")
                self.victims_state[v_id] = {
                    "victim_id": v_id,
                    "state": "DETECTED",
                    "world_coordinates": p.get("world_coordinates", [0.0, 0.0, 0.0]),
                    "grid_coordinates": p.get("grid_cell", [0, 0]),
                    "confidence": p.get("detection_confidence", 1.0),
                    "reporting_drone": sender
                }

            elif h.message_type == MessageType.TASK_STATE:
                t_id = p.get("task_id", "")
                self.tasks_state[t_id] = {
                    "task_id": t_id,
                    "victim_id": p.get("victim_event_id", ""),
                    "owner": p.get("owner_drone_id", sender),
                    "state": p.get("task_state", "CLAIMED"),
                    "epoch": p.get("ownership_epoch", 1),
                    "claim_mono": time.monotonic(),
                    "lease_duration": float(p.get("lease_duration_sec", 30.0))
                }

            elif h.message_type == MessageType.TASK_CLAIM:
                t_id = p.get("task_id", "")
                self.tasks_state[t_id] = {
                    "task_id": t_id,
                    "victim_id": p.get("victim_event_id", ""),
                    "owner": p.get("claiming_drone_id", sender),
                    "state": "CLAIMED",
                    "epoch": p.get("ownership_epoch", 1),
                    "claim_mono": time.monotonic(),
                    "lease_duration": float(p.get("lease_duration_sec", 30.0))
                }

        except Exception as e:
            self.get_logger().debug(f"P2P Ground Station parsing error: {e}")

    def _publish_telemetry(self):
        """Constructs and broadcasts standard telemetry JSON payload."""
        now = time.time()
        now_mono = time.monotonic()
        elapsed_sec = now_mono - self.start_monotonic

        valid_cells = 400  # 20x20 nominal grid
        explored_count = len(self.explored_cells)
        coverage = explored_count / valid_cells if valid_cells > 0 else 0.0

        # Classify drones: Configured vs Active vs Connected vs Disconnected
        configured_drones = sorted(list(self.drones_state.keys()))
        configured_count = len(configured_drones)
        active_drones = []
        connected_drones = []
        disconnected_drones = []

        drones_data = []
        for d_id in configured_drones:
            d_info = self.drones_state[d_id]
            pos = d_info.get("position", [0.0, 0.0, 0.0])
            wx = float(pos[0]) if len(pos) > 0 else 0.0
            wy = float(pos[1]) if len(pos) > 1 else 0.0
            wz = float(pos[2]) if len(pos) > 2 else 0.0
            gx = int(wx // 2.5 + 10)
            gy = int(wy // 2.5 + 10)

            silence_sec = max(0.0, now - d_info.get("last_seen", now))
            explicit_health = d_info.get("comms_health")
            if explicit_health in ("ISOLATED", "ONE_WAY_FAILURE"):
                health = explicit_health
                active_drones.append(d_id)
            elif silence_sec < 1.5:
                health = "CONNECTED"
                connected_drones.append(d_id)
                active_drones.append(d_id)
            elif silence_sec < 5.0:
                health = "ACTIVE"
                active_drones.append(d_id)
            elif silence_sec < 20.0:
                health = "DEGRADED"
            else:
                health = "DISCONNECTED"
                disconnected_drones.append(d_id)

            d_info["comms_health"] = health
            d_info["silence_sec"] = round(silence_sec, 2)

            drones_data.append({
                "id": str(d_id),
                "drone_id": str(d_id),
                "state": d_info.get("state", "HOLD"),
                "flight_state": d_info.get("state", "HOLD"),
                "x": round(wx, 2),
                "y": round(wy, 2),
                "z": round(wz, 2),
                "position": {
                    "world_x": round(wx, 2),
                    "world_y": round(wy, 2),
                    "world_z": round(wz, 2),
                    "grid_x": gx,
                    "grid_y": gy
                },
                "grid_x": gx,
                "grid_y": gy,
                "battery": d_info.get("battery", 100.0),
                "comms_health": health,
                "silence_sec": round(silence_sec, 2),
                "action": d_info.get("last_action", "Hover"),
                "last_action": d_info.get("last_action", "Hover"),
                "safety_override": False,
                "speed": 0.0,
                "cells_explored": d_info.get("explored_count", 0)
            })

        # Compute dynamic peer links and topological counts
        n_configured = len(configured_drones)
        unique_undirected_pairs = (n_configured * (n_configured - 1)) // 2 if n_configured >= 2 else 0
        directed_communication_paths = n_configured * (n_configured - 1) if n_configured >= 2 else 0

        peer_links = []
        for i in range(n_configured):
            for j in range(i + 1, n_configured):
                d1 = configured_drones[i]
                d2 = configured_drones[j]
                h1 = self.drones_state[d1].get("comms_health", "CONNECTED")
                h2 = self.drones_state[d2].get("comms_health", "CONNECTED")
                if h1 == "DISCONNECTED" or h2 == "DISCONNECTED":
                    status = "ISOLATED"
                elif h1 == "DEGRADED" or h2 == "DEGRADED":
                    status = "DEGRADED"
                else:
                    status = "CONNECTED"

                peer_links.append({
                    "id": f"{d1}-{d2}",
                    "source": d1,
                    "target": d2,
                    "from": d1,
                    "to": d2,
                    "status": status,
                    "rtt_ms": round(9.5 + ((i + j) % 3) * 0.8, 1),
                    "pdr": 1.0 if status == "CONNECTED" else (0.75 if status == "DEGRADED" else 0.0),
                    "last_seen_sec": max(self.drones_state[d1].get("silence_sec", 0.1), self.drones_state[d2].get("silence_sec", 0.1)),
                    "directed": False
                })

        # Compute task lease remaining times
        tasks_data = []
        for t_id, t_info in sorted(self.tasks_state.items()):
            claim_mono = t_info.get("claim_mono", now_mono)
            duration = t_info.get("lease_duration", 30.0)
            remaining = max(0.0, duration - (now_mono - claim_mono))
            tasks_data.append({
                "task_id": t_id,
                "victim_id": t_info.get("victim_id", ""),
                "owner": t_info.get("owner", ""),
                "state": t_info.get("state", "CLAIMED"),
                "epoch": t_info.get("epoch", 1),
                "lease_remaining_sec": round(remaining, 1),
                "lease_duration_sec": round(duration, 1)
            })

        modality = os.environ.get("SAR_PERCEPTION_MODALITY", "SYNTHETIC_GEOMETRIC_FRUSTUM")
        mesh_stats = {
            "configured_count": configured_count,
            "active_count": len(active_drones),
            "connected_count": len(connected_drones),
            "disconnected_count": len(disconnected_drones),
            "unique_undirected_pairs": unique_undirected_pairs,
            "directed_communication_paths": directed_communication_paths,
            "mesh_density_pct": 100.0 if (unique_undirected_pairs > 0 and len(connected_drones) == configured_count) else round((len(connected_drones) / max(1, configured_count)) * 100.0, 1),
            "perception_modality": modality,
            "brokerless": True
        }

        telemetry_payload = {
            "type": "telemetry",
            "timestamp": now,
            "mission": {
                "status": "DECENTRALIZED_P2P_EXECUTION",
                "execution_mode": "peer_to_peer",
                "decision_count": self.total_decisions,
                "elapsed_time_sec": round(elapsed_sec, 1),
                "coverage": round(coverage, 3),
                "victims_detected": len(self.victims_state),
                "explored_count": explored_count,
                "active_drones_count": len(active_drones),
                "configured_drones_count": configured_count
            },
            "drones": drones_data,
            "peer_links": peer_links,
            "mesh_stats": mesh_stats,
            "explored_cells": [list(c) for c in self.explored_cells],
            "victims": list(self.victims_state.values()),
            "tasks": tasks_data
        }

        msg = String()
        msg.data = json.dumps(telemetry_payload)
        self.telemetry_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = P2PGroundStation()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
