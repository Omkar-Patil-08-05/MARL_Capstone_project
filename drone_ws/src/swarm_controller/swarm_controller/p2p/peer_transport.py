"""
Modular Peer Transport Abstraction Module (Phase 3 Additive Implementation)
Supports Direct ROS 2 DDS Topics and In-Memory Loopback with Network Impairment Injection.
"""

import time
from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Callable
from collections import deque
from .peer_message_schema import PeerMessage
from .network_impairment import NetworkImpairmentSimulator, NetworkImpairmentConfig

class BasePeerTransport(ABC):
    def __init__(self, drone_id: str, impairment: Optional[NetworkImpairmentSimulator] = None):
        self.drone_id = drone_id
        self.impairment = impairment or NetworkImpairmentSimulator()
        self.payload_bytes_sent: int = 0
        self.wire_bytes_sent: int = 0
        self.payload_bytes_received: int = 0
        self.wire_bytes_received: int = 0
        self.messages_sent: int = 0
        self.messages_received: int = 0
        self.packet_callback: Optional[Callable[[str, str, PeerMessage, bool], None]] = None

    @property
    def bytes_sent(self) -> int:
        return self.wire_bytes_sent

    @bytes_sent.setter
    def bytes_sent(self, val: int):
        self.wire_bytes_sent = val

    @property
    def bytes_received(self) -> int:
        return self.wire_bytes_received

    @bytes_received.setter
    def bytes_received(self, val: int):
        self.wire_bytes_received = val

    @abstractmethod
    def broadcast(self, msg: PeerMessage) -> bool:
        """Broadcasts a peer message to all reachable drones."""
        pass

    @abstractmethod
    def send_to(self, receiver_drone_id: str, msg: PeerMessage) -> bool:
        """Sends a peer message directly to a specific target drone."""
        pass

    @abstractmethod
    def poll_incoming(self) -> List[PeerMessage]:
        """Polls and returns all newly arrived and validated peer messages."""
        pass

    def send_message(self, msg: PeerMessage, receiver_drone_id: Optional[str] = None) -> bool:
        """Convenience dispatch method for scheduler and node outbox flushes."""
        if receiver_drone_id is not None:
            return self.send_to(receiver_drone_id, msg)
        return self.broadcast(msg)

    def get_transport_metrics(self) -> dict:
        metrics = {
            "drone_id": self.drone_id,
            "bytes_sent": self.bytes_sent,
            "bytes_received": self.bytes_received,
            "messages_sent": self.messages_sent,
            "messages_received": self.messages_received
        }
        metrics.update(self.impairment.get_metrics())
        return metrics

class LoopbackPeerTransport(BasePeerTransport):
    """
    In-Memory Shared Loopback Transport for fast multi-drone execution and simulation testing.
    Can be used by multiple drone nodes within the same process or test harness.
    """
    _shared_inboxes: Dict[str, deque] = {}  # receiver_id -> deque of (delivery_time, raw_bytes)
    _all_subscribers: List[str] = []

    def __init__(self, drone_id: str, impairment: Optional[NetworkImpairmentSimulator] = None):
        super().__init__(drone_id, impairment)
        if drone_id not in self._shared_inboxes:
            self._shared_inboxes[drone_id] = deque()
        if drone_id not in self._all_subscribers:
            self._all_subscribers.append(drone_id)

    def broadcast(self, msg: PeerMessage) -> bool:
        raw_bytes = msg.to_bytes()
        self.wire_bytes_sent += len(raw_bytes)
        self.payload_bytes_sent += msg.get_payload_size()
        self.messages_sent += 1
        now = time.monotonic()

        success = True
        subscribers = [p for p in self._all_subscribers if p != self.drone_id]
        if not subscribers:
            return True

        for peer_id in subscribers:
            delivery_time = self.impairment.process_transmission(
                self.drone_id,
                peer_id,
                raw_bytes,
                now,
                msg_type=int(msg.header.message_type),
                sequence_number=int(msg.header.sequence_number)
            )
            delivered = (delivery_time is not None)
            if self.packet_callback:
                self.packet_callback(self.drone_id, peer_id, msg, delivered)
            if delivered:
                self._shared_inboxes[peer_id].append((delivery_time, raw_bytes))
        return success

    def send_to(self, receiver_drone_id: str, msg: PeerMessage) -> bool:
        if receiver_drone_id not in self._shared_inboxes:
            return False
        raw_bytes = msg.to_bytes()
        self.bytes_sent += len(raw_bytes)
        self.messages_sent += 1
        now = time.monotonic()

        delivery_time = self.impairment.process_transmission(
            self.drone_id,
            receiver_drone_id,
            raw_bytes,
            now,
            msg_type=int(msg.header.message_type),
            sequence_number=int(msg.header.sequence_number)
        )
        delivered = (delivery_time is not None)
        if self.packet_callback:
            self.packet_callback(self.drone_id, receiver_drone_id, msg, delivered)
        if delivered:
            self._shared_inboxes[receiver_drone_id].append((delivery_time, raw_bytes))
            return True
        return False

    def poll_incoming(self) -> List[PeerMessage]:
        now = time.monotonic()
        arrived_messages = []
        inbox = self._shared_inboxes.get(self.drone_id, deque())

        # Collect messages whose simulated delivery time has arrived
        remaining = deque()
        while inbox:
            delivery_time, raw_bytes = inbox.popleft()
            if delivery_time <= now:
                msg = PeerMessage.from_bytes(raw_bytes)
                if msg is not None:
                    self.bytes_received += len(raw_bytes)
                    self.messages_received += 1
                    arrived_messages.append(msg)
            else:
                remaining.append((delivery_time, raw_bytes))
        self._shared_inboxes[self.drone_id] = remaining
        return arrived_messages

    @classmethod
    def reset_shared_bus(cls):
        cls._shared_inboxes.clear()
        cls._all_subscribers.clear()

class ROSTopicPeerTransport(BasePeerTransport):
    """
    ROS 2 Direct Inter-Node Topic Transport.
    Publishes to `/swarm/p2p/broadcast` and filters out self-messages by sender_drone_id.
    """
    def __init__(self, node, drone_id: str, impairment: Optional[NetworkImpairmentSimulator] = None):
        super().__init__(drone_id, impairment)
        self.node = node

        # Lazy import to ensure compatibility in environments without active ROS 2 rclpy context
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.broadcast_pub = self.node.create_publisher(String, '/swarm/p2p/broadcast', qos_best_effort)
        self.sub = self.node.create_subscription(String, '/swarm/p2p/broadcast', self._ros_callback, qos_best_effort)
        self._incoming_buffer = deque()

    def _ros_callback(self, msg):
        try:
            raw_bytes = msg.data.encode('utf-8')
            parsed = PeerMessage.from_bytes(raw_bytes)
            if parsed is not None and parsed.header.sender_drone_id != self.drone_id:
                # Apply simulated impairment (e.g. drop/delay)
                delivery_time = self.impairment.process_transmission(
                    parsed.header.sender_drone_id,
                    self.drone_id,
                    raw_bytes,
                    time.monotonic(),
                    msg_type=int(parsed.header.message_type),
                    sequence_number=int(parsed.header.sequence_number)
                )
                if delivery_time is not None:
                    # Non-blocking delay queue: append tuple with scheduled delivery timestamp
                    self._incoming_buffer.append((delivery_time, raw_bytes, parsed))
        except Exception:
            pass

    def broadcast(self, msg: PeerMessage) -> bool:
        from std_msgs.msg import String
        raw_json = msg.to_json()
        self.bytes_sent += len(raw_json.encode('utf-8'))
        self.messages_sent += 1
        ros_msg = String()
        ros_msg.data = raw_json
        self.broadcast_pub.publish(ros_msg)
        return True

    def send_to(self, receiver_drone_id: str, msg: PeerMessage) -> bool:
        # Over broadcast topic, target is filtered at application layer
        return self.broadcast(msg)

    def poll_incoming(self) -> List[PeerMessage]:
        now = time.monotonic()
        arrived = []
        remaining = deque()
        while self._incoming_buffer:
            item = self._incoming_buffer.popleft()
            if isinstance(item, tuple):
                delivery_time, raw_bytes, parsed = item
                if delivery_time <= now:
                    self.bytes_received += len(raw_bytes)
                    self.messages_received += 1
                    arrived.append(parsed)
                else:
                    remaining.append(item)
            else:
                arrived.append(item)
        self._incoming_buffer = remaining
        return arrived
