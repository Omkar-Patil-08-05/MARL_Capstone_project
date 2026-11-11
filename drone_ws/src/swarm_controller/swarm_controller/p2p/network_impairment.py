"""
Network Impairment Simulator Module (Tier 3 Phase 5 Fault-Tolerance Engine)
Simulates realistic, deterministic RF degradation on the P2P messaging path:
- Random packet loss (low, medium, severe)
- Network latency & latency jitter
- Burst packet loss
- Symmetric network partitions
- Asymmetric (directed one-way) communication failure
- Communication recovery

Critical Isolation Guarantee:
This module operates strictly at the P2P application messaging layer.
It never impairs PX4 Offboard setpoints, PX4 commands, vehicle telemetry,
Gazebo clock topics, or emergency flight-termination mechanisms.
"""

import time
import random
from dataclasses import dataclass, field
from typing import Set, Tuple, List, Optional, Any, Dict

@dataclass
class NetworkImpairmentConfig:
    scenario_name: str = "nominal"
    seed: Optional[int] = None
    packet_loss_rate: float = 0.0          # 0.0 to 1.0 (probability of drop)
    min_latency_sec: float = 0.005         # Baseline transport delay (5 ms)
    max_latency_sec: float = 0.015         # Max transport delay (15 ms)
    jitter_sec: float = 0.002              # Latency jitter
    out_of_order_prob: float = 0.0         # Probability of out-of-order delivery
    burst_loss_length: int = 0             # Number of consecutive packets dropped in a burst
    burst_loss_prob: float = 0.0           # Probability of triggering burst drop
    partitioned_pairs: Set[Tuple[str, str]] = field(default_factory=set)  # Symmetric: neither can talk
    asymmetric_partitioned_directed: Set[Tuple[str, str]] = field(default_factory=set)  # Directed: (sender, receiver) blocked
    mean_latency_ms: Optional[float] = None
    jitter_ms: Optional[float] = None

    def __post_init__(self):
        if self.mean_latency_ms is not None:
            self.min_latency_sec = self.mean_latency_ms / 1000.0
            self.max_latency_sec = self.mean_latency_ms / 1000.0
        if self.jitter_ms is not None:
            self.jitter_sec = self.jitter_ms / 1000.0


class NetworkImpairmentSimulator:
    """
    Deterministic application-level P2P fault injector.
    Maintains detailed transmission logging for reproducibility and audits.
    """
    def __init__(self, config: Optional[NetworkImpairmentConfig] = None):
        self.config = config or NetworkImpairmentConfig()
        self.rng = random.Random(self.config.seed) if self.config.seed is not None else random.Random()
        self.packets_transmitted: int = 0
        self.packets_dropped: int = 0
        self.packets_delayed: int = 0
        self.packets_delivered: int = 0
        self.bytes_transmitted: int = 0
        self.bytes_dropped: int = 0
        self._current_burst_remaining: int = 0
        self._delayed_queue: List[Tuple[float, str, str, Any]] = []  # (delivery_time, sender, receiver, msg)
        self.transmission_log: List[Dict[str, Any]] = []

    def set_seed(self, seed: int):
        """Sets random seed for deterministic reproduction."""
        self.config.seed = seed
        self.rng = random.Random(seed)

    def is_partitioned(self, sender: str, receiver: str) -> bool:
        """
        Checks if communication between sender and receiver is partitioned.
        Evaluates both symmetric bidirectional partitions and directed asymmetric partitions.
        """
        pair = (sender, receiver)
        rev_pair = (receiver, sender)
        if pair in self.config.partitioned_pairs or rev_pair in self.config.partitioned_pairs:
            return True
        if pair in self.config.asymmetric_partitioned_directed:
            return True
        return False

    def is_asymmetric_failing(self, sender: str, receiver: str) -> bool:
        """Checks if there is a directed asymmetric failure from sender to receiver."""
        return (sender, receiver) in self.config.asymmetric_partitioned_directed

    def set_asymmetric_partition(self, sender: str, receiver: str):
        """Blocks transmissions from sender to receiver while allowing the reverse direction."""
        self.config.asymmetric_partitioned_directed.add((sender, receiver))

    def clear_asymmetric_partitions(self):
        """Clears all directed asymmetric partitions."""
        self.config.asymmetric_partitioned_directed.clear()

    def set_partition(self, partitions: List[Set[str]]):
        """Sets symmetric communication partitions: drones in different sets cannot communicate."""
        self.config.partitioned_pairs.clear()
        for i, group_a in enumerate(partitions):
            for j, group_b in enumerate(partitions):
                if i != j:
                    for da in group_a:
                        for db in group_b:
                            self.config.partitioned_pairs.add((da, db))

    def clear_partitions(self):
        """Removes all symmetric and asymmetric partitions, restoring full swarm connectivity."""
        self.config.partitioned_pairs.clear()
        self.config.asymmetric_partitioned_directed.clear()

    def set_scenario(self, scenario_name: str, seed: Optional[int] = None):
        """
        Configures one of the 11 Phase 5 fault scenarios deterministically.
        """
        if seed is not None:
            self.set_seed(seed)
        self.config.scenario_name = scenario_name

        # Reset baseline impairment parameters
        self.config.packet_loss_rate = 0.0
        self.config.min_latency_sec = 0.005
        self.config.max_latency_sec = 0.015
        self.config.jitter_sec = 0.002
        self.config.out_of_order_prob = 0.0
        self.config.burst_loss_length = 0
        self.config.burst_loss_prob = 0.0
        self.clear_partitions()

        name = scenario_name.lower().replace("-", "_").replace(" ", "_")
        if name in ("nominal", "baseline"):
            self.config.min_latency_sec = 0.0
            self.config.max_latency_sec = 0.0
            self.config.jitter_sec = 0.0

        elif name == "low_loss":
            self.config.packet_loss_rate = 0.05  # 5%

        elif name == "medium_loss":
            self.config.packet_loss_rate = 0.10  # 10%

        elif name == "severe_loss":
            self.config.packet_loss_rate = 0.20  # 20%

        elif name == "latency":
            self.config.min_latency_sec = 0.100  # 100 ms added delay
            self.config.max_latency_sec = 0.100
            self.config.jitter_sec = 0.0

        elif name == "high_latency":
            self.config.min_latency_sec = 0.250  # 250 ms added delay
            self.config.max_latency_sec = 0.250
            self.config.jitter_sec = 0.0

        elif name == "jitter":
            self.config.min_latency_sec = 0.050  # 50 - 150 ms
            self.config.max_latency_sec = 0.150
            self.config.jitter_sec = 0.040

        elif name == "burst_loss":
            self.config.burst_loss_length = 5
            self.config.burst_loss_prob = 0.20

        elif name == "partition":
            # Bidirectional 3-second blackout
            self.config.partitioned_pairs.add(("drone_0", "drone_1"))

        elif name == "asymmetric_partition":
            # Directed drop: drone_0 -> drone_1 blocked, drone_1 -> drone_0 delivered
            self.set_asymmetric_partition("drone_0", "drone_1")

        elif name == "recovery":
            # All partitions cleared, zero loss
            self.clear_partitions()
            self.config.min_latency_sec = 0.0
            self.config.max_latency_sec = 0.0
            self.config.jitter_sec = 0.0
        else:
            raise ValueError(f"Unknown fault scenario: {scenario_name}")

    def process_transmission(
        self,
        sender: str,
        receiver: str,
        payload_bytes: bytes,
        current_time: Optional[float] = None,
        msg_type: int = 0,
        sequence_number: int = 0
    ) -> Optional[float]:
        """
        Determines whether a P2P packet survives transmission and calculates scheduled delivery time.
        Returns delivery_timestamp if delivered, or None if dropped/partitioned.
        Logs every event with exact metadata.
        """
        now = current_time if current_time is not None else time.monotonic()
        size = len(payload_bytes)
        self.packets_transmitted += 1
        self.bytes_transmitted += size

        # 1. Asymmetric / Directed Partition Check
        if (sender, receiver) in self.config.asymmetric_partitioned_directed:
            self.packets_dropped += 1
            self.bytes_dropped += size
            self._log_event(now, sender, receiver, msg_type, sequence_number, "DROPPED_ASYMMETRIC_PARTITION", 0.0, size)
            return None

        # 2. Symmetric Complete Network Partition Check
        pair = (sender, receiver)
        rev_pair = (receiver, sender)
        if pair in self.config.partitioned_pairs or rev_pair in self.config.partitioned_pairs:
            self.packets_dropped += 1
            self.bytes_dropped += size
            self._log_event(now, sender, receiver, msg_type, sequence_number, "DROPPED_SYMMETRIC_PARTITION", 0.0, size)
            return None

        # 3. Burst Loss Active Check
        if self._current_burst_remaining > 0:
            self._current_burst_remaining -= 1
            self.packets_dropped += 1
            self.bytes_dropped += size
            self._log_event(now, sender, receiver, msg_type, sequence_number, "DROPPED_BURST_ACTIVE", 0.0, size)
            return None

        # 4. Burst Loss Trigger Check
        if self.config.burst_loss_prob > 0.0 and self.rng.random() < self.config.burst_loss_prob:
            self._current_burst_remaining = max(1, self.config.burst_loss_length - 1)
            self.packets_dropped += 1
            self.bytes_dropped += size
            self._log_event(now, sender, receiver, msg_type, sequence_number, "DROPPED_BURST_TRIGGER", 0.0, size)
            return None

        # 5. Independent Random Packet Loss
        if self.config.packet_loss_rate > 0.0 and self.rng.random() < self.config.packet_loss_rate:
            self.packets_dropped += 1
            self.bytes_dropped += size
            self._log_event(now, sender, receiver, msg_type, sequence_number, "DROPPED_RANDOM", 0.0, size)
            return None

        # 6. Calculate Simulated Latency + Jitter
        if self.config.max_latency_sec <= self.config.min_latency_sec:
            latency = self.config.min_latency_sec
        else:
            latency = self.rng.uniform(self.config.min_latency_sec, self.config.max_latency_sec)

        if self.config.jitter_sec > 0.0:
            latency += self.rng.uniform(-self.config.jitter_sec, self.config.jitter_sec)
        latency = max(0.0, latency)

        # 7. Out-of-Order Delay Perturbation
        if self.config.out_of_order_prob > 0.0 and self.rng.random() < self.config.out_of_order_prob:
            latency += self.rng.uniform(0.05, 0.20)

        delivery_time = now + latency
        self.packets_delivered += 1
        status = "DELIVERED_DELAYED" if latency > 0.010 else "DELIVERED_IMMEDIATE"
        if latency > 0.010:
            self.packets_delayed += 1
        self._log_event(now, sender, receiver, msg_type, sequence_number, status, latency * 1000.0, size)
        return delivery_time

    def _log_event(
        self,
        timestamp: float,
        sender: str,
        receiver: str,
        msg_type: int,
        seq: int,
        status: str,
        latency_ms: float,
        size_bytes: int
    ):
        """Records packet transmission attempt in memory."""
        self.transmission_log.append({
            "timestamp": round(timestamp, 4),
            "scenario": self.config.scenario_name,
            "sender": sender,
            "receiver": receiver,
            "msg_type": msg_type,
            "sequence": seq,
            "status": status,
            "latency_ms": round(latency_ms, 2),
            "size_bytes": size_bytes
        })

    def get_metrics(self) -> dict:
        total = self.packets_transmitted
        loss_pct = (self.packets_dropped / total * 100.0) if total > 0 else 0.0
        return {
            "scenario": self.config.scenario_name,
            "packets_transmitted": self.packets_transmitted,
            "packets_dropped": self.packets_dropped,
            "packets_delayed": self.packets_delayed,
            "packets_delivered": self.packets_delivered,
            "bytes_transmitted": self.bytes_transmitted,
            "bytes_dropped": self.bytes_dropped,
            "measured_packet_loss_pct": round(loss_pct, 2),
            "total_logged_events": len(self.transmission_log)
        }

    @property
    def metrics(self):
        from types import SimpleNamespace
        return SimpleNamespace(**self.get_metrics())
