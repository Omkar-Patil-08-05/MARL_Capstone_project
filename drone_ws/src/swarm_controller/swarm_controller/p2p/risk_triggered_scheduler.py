"""
Candidate B: Risk-Triggered Semantic Peer Communication Scheduler (Corrected Architecture)
Protocol Version: 2.1-P2P

Key Corrections:
- Strict 5-level priority scheme: CRITICAL(0), HIGH(1), MEDIUM(2), LOW(3), BACKGROUND(4).
- Separate tracking of application payload bytes and transport wire bytes.
- Continuous measurement of wire bytes per second.
- Explicit semantic triggers for ACK, Journal Sync, Task Claim, Victim Confirmation, and Trajectory Intent.
- Rate-limiting for periodic BACKGROUND telemetry (1.0 Hz baseline).
"""

import heapq
import time
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass, field

from .peer_message_schema import (
    PeerMessage,
    PeerHeader,
    MessageType,
    MessagePriority,
    EmergencyPayload,
    VictimConfirmedPayload,
    TaskClaimPayload,
    TaskStatePayload,
    IntentTrajectoryPayload,
    HeartbeatPayload,
    AckPayload,
    JournalSyncReqPayload,
    JournalSyncRespPayload
)
from .peer_transport import BasePeerTransport

@dataclass(order=True)
class PrioritizedItem:
    priority: int
    monotonic_time: float
    sequence_num: int
    message: PeerMessage = field(compare=False)

@dataclass
class BandwidthStats:
    total_payload_bytes_sent: int = 0
    total_wire_bytes_sent: int = 0
    total_payload_bytes_received: int = 0
    total_wire_bytes_received: int = 0
    packets_sent_by_priority: Dict[int, int] = field(default_factory=lambda: {
        MessagePriority.CRITICAL: 0,
        MessagePriority.HIGH: 0,
        MessagePriority.MEDIUM: 0,
        MessagePriority.LOW: 0,
        MessagePriority.BACKGROUND: 0
    })
    packets_received_by_priority: Dict[int, int] = field(default_factory=lambda: {
        MessagePriority.CRITICAL: 0,
        MessagePriority.HIGH: 0,
        MessagePriority.MEDIUM: 0,
        MessagePriority.LOW: 0,
        MessagePriority.BACKGROUND: 0
    })

    @property
    def total_bytes_sent(self) -> int:
        """Backward-compatible alias for total_wire_bytes_sent."""
        return self.total_wire_bytes_sent

    @property
    def total_bytes_received(self) -> int:
        """Backward-compatible alias for total_wire_bytes_received."""
        return self.total_wire_bytes_received

class RiskTriggeredScheduler:
    def __init__(
        self,
        local_drone_id: str,
        session_id: Optional[str] = None,
        mission_id: str = "sar_mission",
        map_id: str = "earthquake_world",
        heartbeat_hz: float = 1.0,
        session_epoch: int = 1
    ):
        self.local_drone_id = local_drone_id
        self.session_id = session_id or f"sess_{local_drone_id}_{int(time.time())}"
        self.mission_id = mission_id
        self.map_id = map_id
        self.heartbeat_interval_sec = 1.0 / max(0.1, heartbeat_hz)
        self.session_epoch = session_epoch

        self._sequence_counter: int = 0
        self._last_heartbeat_monotonic: float = 0.0
        self._outbox_heap: List[PrioritizedItem] = []
        self.stats = BandwidthStats()

        # Rolling window wire byte tracking
        self._window_start_monotonic = time.monotonic()
        self._window_wire_bytes_sent: int = 0

    def _next_sequence_num(self) -> int:
        self._sequence_counter = (self._sequence_counter + 1) & 0xFFFFFFFF
        return self._sequence_counter

    def _create_header(self, msg_type: int, priority: int) -> PeerHeader:
        return PeerHeader(
            protocol_version=2,
            message_type=msg_type,
            priority=priority,
            sender_drone_id=self.local_drone_id,
            session_id=self.session_id,
            mission_id=self.mission_id,
            map_id=self.map_id,
            frame_id="world_enu",
            session_epoch=self.session_epoch,
            sequence_number=self._next_sequence_num(),
            sender_timestamp_utc=time.time()
        )

    def enqueue_message(self, message: PeerMessage):
        """Pushes a message onto the priority heap."""
        item = PrioritizedItem(
            priority=int(message.header.priority),
            monotonic_time=time.monotonic(),
            sequence_num=message.header.sequence_number,
            message=message
        )
        heapq.heappush(self._outbox_heap, item)

    def trigger_emergency_safety(
        self,
        emergency_code: str,
        conflicting_peer_id: str,
        collision_time_sec: float,
        current_pos: List[float],
        emergency_action: str = "HOVER_IMMEDIATE",
        evasion_vector: Optional[List[float]] = None
    ) -> PeerMessage:
        """CRITICAL priority: Immediate collision abort / evasive manoeuvre."""
        header = self._create_header(MessageType.EMERGENCY_SAFETY, MessagePriority.CRITICAL)
        payload = EmergencyPayload(
            emergency_code=emergency_code,
            conflicting_peer_id=conflicting_peer_id,
            predicted_collision_time_sec=collision_time_sec,
            current_position=list(current_pos),
            emergency_action=emergency_action,
            evasion_vector=list(evasion_vector or [0.0, 0.0, 0.0])
        )
        msg = PeerMessage(header=header, payload=payload.__dict__)
        self.enqueue_message(msg)
        return msg

    def trigger_victim_confirmed(
        self,
        victim_event_id: str,
        world_coords: List[float],
        grid_cell: List[int],
        uncertainty_m: float = 0.5,
        covariance_diag: Optional[List[float]] = None,
        confidence: float = 0.95,
        confirmation_frames: int = 5,
        observation_id: str = ""
    ) -> PeerMessage:
        """HIGH priority: Confirmed victim evidence broadcast."""
        header = self._create_header(MessageType.VICTIM_CONFIRMED, MessagePriority.HIGH)
        payload = VictimConfirmedPayload(
            victim_event_id=victim_event_id,
            reporting_drone_id=self.local_drone_id,
            world_coordinates=list(world_coords),
            grid_cell=list(grid_cell),
            position_uncertainty_m=uncertainty_m,
            covariance_diag=list(covariance_diag or [0.25, 0.25, 0.25]),
            detection_confidence=confidence,
            confirmation_frames=confirmation_frames,
            detection_timestamp_utc=time.time(),
            observation_id=observation_id
        )
        msg = PeerMessage(header=header, payload=payload.__dict__)
        self.enqueue_message(msg)
        return msg

    def trigger_ack(self, acked_seq: int, acked_msg_type: int, acked_session_id: str) -> PeerMessage:
        """HIGH priority: Immediate receipt acknowledgment for reliable P2P delivery."""
        header = self._create_header(MessageType.ACK, MessagePriority.HIGH)
        payload = AckPayload(
            acked_sequence=acked_seq,
            acked_message_type=acked_msg_type,
            acked_session_id=acked_session_id,
            ack_sender_id=self.local_drone_id,
            ack_timestamp_utc=time.time()
        )
        msg = PeerMessage(header=header, payload=payload.__dict__)
        self.enqueue_message(msg)
        return msg

    def trigger_journal_sync_req(self, known_victim_ids: List[str], known_task_epochs: Dict[str, int]) -> PeerMessage:
        """HIGH priority: Post-partition reconciliation request."""
        header = self._create_header(MessageType.JOURNAL_SYNC_REQ, MessagePriority.HIGH)
        payload = JournalSyncReqPayload(
            requesting_drone_id=self.local_drone_id,
            known_victim_ids=list(known_victim_ids),
            known_task_epochs=dict(known_task_epochs)
        )
        msg = PeerMessage(header=header, payload=payload.__dict__)
        self.enqueue_message(msg)
        return msg

    def trigger_journal_sync_resp(self, victims_delta: List[Dict[str, Any]], tasks_delta: List[Dict[str, Any]]) -> PeerMessage:
        """HIGH priority: Post-partition reconciliation delta response."""
        header = self._create_header(MessageType.JOURNAL_SYNC_RESP, MessagePriority.HIGH)
        payload = JournalSyncRespPayload(
            responding_drone_id=self.local_drone_id,
            victims_delta=list(victims_delta),
            tasks_delta=list(tasks_delta)
        )
        msg = PeerMessage(header=header, payload=payload.__dict__)
        self.enqueue_message(msg)
        return msg

    def trigger_task_claim(self, task_id: str, victim_event_id: str, epoch: int, lease_sec: float = 30.0) -> PeerMessage:
        """MEDIUM priority: Explicit epoch-fenced task lease claim."""
        header = self._create_header(MessageType.TASK_CLAIM, MessagePriority.MEDIUM)
        payload = TaskClaimPayload(
            task_id=task_id,
            victim_event_id=victim_event_id,
            claiming_drone_id=self.local_drone_id,
            ownership_epoch=epoch,
            lease_duration_sec=lease_sec
        )
        msg = PeerMessage(header=header, payload=payload.__dict__)
        self.enqueue_message(msg)
        return msg

    def trigger_task_state(
        self,
        task_id: str,
        victim_event_id: str,
        task_state: str,
        lease_duration_sec: float = 30.0,
        progress_pct: float = 0.0,
        ownership_epoch: int = 1
    ) -> PeerMessage:
        """MEDIUM priority: Task state progression or release."""
        header = self._create_header(MessageType.TASK_STATE, MessagePriority.MEDIUM)
        payload = TaskStatePayload(
            task_id=task_id,
            victim_event_id=victim_event_id,
            owner_drone_id=self.local_drone_id,
            task_state=task_state,
            ownership_epoch=ownership_epoch,
            lease_duration_sec=lease_duration_sec,
            progress_percent=progress_pct,
            state_update_timestamp_utc=time.time()
        )
        msg = PeerMessage(header=header, payload=payload.__dict__)
        self.enqueue_message(msg)
        return msg

    def trigger_trajectory_intent(
        self,
        current_grid: List[int],
        target_grid: List[int],
        start_world_enu: List[float],
        target_world_enu: List[float],
        velocity_vector: List[float],
        macro_action: str = "Hover",
        estimated_duration_sec: float = 2.0
    ) -> PeerMessage:
        """LOW priority: QMIX macro-action trajectory intention commitment."""
        header = self._create_header(MessageType.INTENT_TRAJECTORY, MessagePriority.LOW)
        payload = IntentTrajectoryPayload(
            current_grid=list(current_grid),
            target_grid=list(target_grid),
            start_world_enu=list(start_world_enu),
            target_world_enu=list(target_world_enu),
            velocity_vector=list(velocity_vector),
            estimated_duration_sec=estimated_duration_sec,
            macro_action=macro_action
        )
        msg = PeerMessage(header=header, payload=payload.__dict__)
        self.enqueue_message(msg)
        return msg

    def schedule_heartbeat(
        self,
        flight_state: str,
        position_enu: List[float],
        velocity_enu: List[float],
        battery_pct: float = 100.0,
        cells_explored: int = 0,
        comms_health: str = "CONNECTED",
        force: bool = False,
        is_armed: bool = True,
        is_offboard: bool = True,
        stability_verified: bool = False,
        last_seen_peer_sequences: Optional[Dict[str, int]] = None
    ) -> Optional[PeerMessage]:
        """BACKGROUND priority: Rate-limited routine telemetry beacon."""
        now = time.monotonic()
        if not force and (now - self._last_heartbeat_monotonic) < self.heartbeat_interval_sec:
            return None

        self._last_heartbeat_monotonic = now
        header = self._create_header(MessageType.HEARTBEAT_STATUS, MessagePriority.BACKGROUND)
        payload = HeartbeatPayload(
            flight_state=flight_state,
            position_enu=list(position_enu),
            velocity_enu=list(velocity_enu),
            battery_percentage=battery_pct,
            cells_explored_count=cells_explored,
            comms_health_state=comms_health,
            is_armed=is_armed,
            is_offboard=is_offboard,
            stability_verified=stability_verified,
            last_seen_peer_sequences=dict(last_seen_peer_sequences or {})
        )
        msg = PeerMessage(header=header, payload=payload.__dict__)
        self.enqueue_message(msg)
        return msg

    def flush_outbox(
        self,
        transport: BasePeerTransport,
        max_messages: Optional[int] = None
    ) -> int:
        """Pops and transmits messages from the priority heap via transport."""
        sent_count = 0
        limit = max_messages if max_messages is not None else len(self._outbox_heap)

        while self._outbox_heap and sent_count < limit:
            item = heapq.heappop(self._outbox_heap)
            msg = item.message
            wire_len = msg.get_wire_size()
            payload_len = msg.get_payload_size()

            success = transport.send_message(msg)
            if success:
                sent_count += 1
                self.stats.total_payload_bytes_sent += payload_len
                self.stats.total_wire_bytes_sent += wire_len
                self._window_wire_bytes_sent += wire_len

                p = int(msg.header.priority)
                self.stats.packets_sent_by_priority[p] = self.stats.packets_sent_by_priority.get(p, 0) + 1

        return sent_count

    def record_received_message(self, message: PeerMessage):
        """Records payload and wire metrics for inbound traffic."""
        self.stats.total_payload_bytes_received += message.get_payload_size()
        self.stats.total_wire_bytes_received += message.get_wire_size()
        p = int(message.header.priority)
        self.stats.packets_received_by_priority[p] = self.stats.packets_received_by_priority.get(p, 0) + 1

    def get_wire_bytes_per_second(self) -> float:
        """Calculates current transmission rate in wire bytes per second."""
        elapsed = time.monotonic() - self._window_start_monotonic
        if elapsed <= 0.0:
            return 0.0
        return self._window_wire_bytes_sent / elapsed

    @property
    def queue_size(self) -> int:
        return len(self._outbox_heap)
