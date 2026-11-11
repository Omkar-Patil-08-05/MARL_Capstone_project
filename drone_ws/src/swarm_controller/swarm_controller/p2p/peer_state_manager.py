"""
Peer State Manager & Communication-Health State Machine (Corrected Architecture)
Protocol Version: 2.1-P2P

Key Corrections:
- Strictly distinguishes Receiver-Local Silence Age (monotonic elapsed since last packet arrival)
  from True Age-of-Information (UTC elapsed since sender data sampling).
- Persistent Session UUID detection & Session Reboot recovery.
- RFC 1982 32-bit sequence wraparound arithmetic with 64-bit anti-replay bitmask window.
- 5-State FSM: CONNECTED, DEGRADED, ISOLATED, UNKNOWN, RECONNECTING.
- Transition to UNKNOWN state when silence exceeds operational threshold (default 20.0s).
"""

import time
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Set
from .peer_message_schema import PeerMessage, PeerHeader, MessageType

class CommsHealthState(Enum):
    CONNECTED = auto()     # Fresh updates: silence_age < 1.5s
    DEGRADED = auto()      # Delayed updates: 1.5s <= silence_age < 5.0s
    ISOLATED = auto()      # Signal lost: 5.0s <= silence_age < 20.0s
    UNKNOWN = auto()       # Uncontactable: silence_age >= 20.0s (Reachable set exceeds operational bounds)
    RECONNECTING = auto()  # Signal restored; accumulating consecutive packets before full CONNECTED transition
    ONE_WAY_FAILURE = auto() # Asymmetric partition: peer reports stale/zero sequence for local drone while local drone is sending

@dataclass
class PeerRecord:
    peer_id: str
    session_id: str = "init_session"
    last_arrival_monotonic: float = 0.0   # Receiver's local monotonic clock at packet arrival
    last_sample_utc: float = 0.0          # Sender's GPS/UTC sampling timestamp
    last_seen_sequence: int = 0
    last_peer_acknowledged_local_seq: int = 0
    session_epoch: int = 1
    last_position_enu: List[float] = field(default_factory=lambda: [0.0, 0.0, 9.0])
    last_velocity_enu: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    last_grid_cell: List[int] = field(default_factory=lambda: [0, 0])
    target_grid_cell: Optional[List[int]] = None
    flight_state: str = "UNKNOWN"
    battery_percentage: float = 100.0
    comms_health: CommsHealthState = CommsHealthState.ISOLATED
    is_armed: bool = False
    is_offboard: bool = False
    stability_verified: bool = False
    consecutive_reconnect_packets: int = 0
    sequence_history_mask: int = 0        # 64-bit sliding window for anti-replay
    total_packets_received: int = 0
    total_payload_bytes_received: int = 0
    total_wire_bytes_received: int = 0
    rtt_sec_ema: Optional[float] = None   # Validated round-trip time in local monotonic domain
    last_rtt_sample_sec: Optional[float] = None
    loss_rate_ema: float = 0.0            # Directional inbound packet loss ratio EMA

    @property
    def last_seen_monotonic(self) -> float:
        """Backward-compatible property for last_arrival_monotonic."""
        return self.last_arrival_monotonic

    @last_seen_monotonic.setter
    def last_seen_monotonic(self, val: float):
        self.last_arrival_monotonic = val

class PeerStateManager:
    def __init__(
        self,
        local_drone_id: str,
        degraded_timeout_sec: float = 1.5,
        isolated_timeout_sec: float = 5.0,
        unknown_timeout_sec: float = 20.0,
        reconnect_consecutive_needed: int = 3
    ):
        self.local_drone_id = local_drone_id
        self.degraded_timeout = degraded_timeout_sec
        self.isolated_timeout = isolated_timeout_sec
        self.unknown_timeout = unknown_timeout_sec
        self.reconnect_consecutive_needed = reconnect_consecutive_needed

        self.peers: Dict[str, PeerRecord] = {}
        self.overall_comms_state: CommsHealthState = CommsHealthState.ISOLATED
        self.local_outbound_sequence: int = 0
        self.outbound_packet_timestamps: Dict[int, float] = {}

    def set_local_sequence(self, seq: int, timestamp_mono: Optional[float] = None):
        """
        Updates the local drone's outbound sequence number for asymmetric partition detection
        and records local monotonic send timestamp for clock-skew-immune RTT calculation.
        """
        self.local_outbound_sequence = seq
        now_mono = timestamp_mono if timestamp_mono is not None else time.monotonic()
        self.outbound_packet_timestamps[seq] = now_mono
        if len(self.outbound_packet_timestamps) > 200:
            oldest_keys = sorted(self.outbound_packet_timestamps.keys())[:-100]
            for k in oldest_keys:
                self.outbound_packet_timestamps.pop(k, None)

    def register_peer(self, peer_id: str, initial_pos: Optional[List[float]] = None):
        """Registers a peer drone to track."""
        if peer_id == self.local_drone_id:
            return
        if peer_id not in self.peers:
            pos = initial_pos or [0.0, 0.0, 9.0]
            self.peers[peer_id] = PeerRecord(peer_id=peer_id, last_position_enu=list(pos))

    def get_silence_age_sec(self, peer_id: str, current_monotonic: Optional[float] = None) -> float:
        """
        Calculates receiver-local silence age in seconds strictly using receiver's monotonic clock.
        Measures the elapsed duration since the last packet successfully arrived from this peer.
        """
        now = current_monotonic if current_monotonic is not None else time.monotonic()
        if peer_id not in self.peers:
            return float('inf')
        record = self.peers[peer_id]
        if record.last_arrival_monotonic <= 0.0:
            return float('inf')
        return max(0.0, now - record.last_arrival_monotonic)

    def get_true_aoi_sec(self, peer_id: str, current_utc: Optional[float] = None) -> float:
        """
        Calculates True Age-of-Information (AoI) in seconds using synchronized UTC timestamps.
        Measures elapsed time since data was originally sampled on the sender.
        """
        now = current_utc if current_utc is not None else time.time()
        if peer_id not in self.peers:
            return float('inf')
        record = self.peers[peer_id]
        if record.last_sample_utc <= 0.0:
            return float('inf')
        return max(0.0, now - record.last_sample_utc)

    def get_age_of_information(self, peer_id: str, current_time: Optional[float] = None) -> float:
        """
        Backward-compatible AoI lookup returning receiver-local silence age.
        """
        return self.get_silence_age_sec(peer_id, current_monotonic=current_time)

    def record_incoming_message(self, msg: PeerMessage, arrival_time: Optional[float] = None) -> bool:
        """Alias for process_incoming_message for unified API."""
        return self.process_incoming_message(msg, arrival_time)

    def process_incoming_message(self, msg: PeerMessage, current_time: Optional[float] = None) -> bool:
        """
        Validates session identity and sequence progression, updates telemetry, and updates FSM.
        Returns True if packet was fresh and accepted, False if duplicate or rejected.
        """
        now_monotonic = current_time if current_time is not None else time.monotonic()
        sender_id = msg.header.sender_drone_id
        if sender_id == self.local_drone_id:
            return False

        if sender_id not in self.peers:
            self.register_peer(sender_id)

        peer = self.peers[sender_id]
        seq = msg.header.sequence_number & 0xFFFFFFFF
        epoch = msg.header.session_epoch & 0xFFFFFFFF
        session_id = msg.header.session_id

        # 1. Session Identity & Reboot Detection
        if peer.session_id != session_id:
            # Peer rebooted with a fresh session UUID
            peer.session_id = session_id
            peer.session_epoch = epoch
            peer.last_seen_sequence = seq
            peer.sequence_history_mask = 1
            peer.comms_health = CommsHealthState.RECONNECTING
            peer.consecutive_reconnect_packets = 0
        elif epoch > peer.session_epoch:
            # Explicit epoch increment
            peer.session_epoch = epoch
            peer.last_seen_sequence = seq
            peer.sequence_history_mask = 1
        elif epoch < peer.session_epoch:
            return False  # Discard packet from obsolete epoch
        else:
            # Same session and epoch: apply RFC 1982 32-bit arithmetic
            if PeerHeader.is_sequence_newer(seq, peer.last_seen_sequence):
                diff = (seq - peer.last_seen_sequence) & 0xFFFFFFFF
                dropped = max(0, diff - 1)
                instant_loss = float(dropped) / float(diff) if diff > 0 else 0.0
                peer.loss_rate_ema = 0.85 * peer.loss_rate_ema + 0.15 * instant_loss
                shift = min(diff, 64)
                peer.sequence_history_mask = ((peer.sequence_history_mask << shift) | 1) & 0xFFFFFFFFFFFFFFFF
                peer.last_seen_sequence = seq
            else:
                # Out-of-order or duplicate packet check
                diff = (peer.last_seen_sequence - seq) & 0xFFFFFFFF
                if diff >= 64:
                    return False  # Beyond 64-bit sliding window -> reject as stale
                mask_bit = 1 << diff
                if (peer.sequence_history_mask & mask_bit) != 0:
                    return False  # Duplicate packet rejected
                peer.sequence_history_mask |= mask_bit  # Out of order accepted within window

        # 2. Update Peer Telemetry & Byte Metrics
        peer.last_arrival_monotonic = now_monotonic
        peer.last_sample_utc = msg.header.sender_timestamp_utc
        peer.total_packets_received += 1
        peer.total_payload_bytes_received += msg.get_payload_size()
        peer.total_wire_bytes_received += msg.get_wire_size()

        p = msg.payload
        is_asymmetric_failure = False
        if msg.header.message_type == MessageType.HEARTBEAT_STATUS:
            peer.flight_state = p.get("flight_state", peer.flight_state)
            peer.last_position_enu = list(p.get("position_enu", peer.last_position_enu))
            peer.last_velocity_enu = list(p.get("velocity_enu", peer.last_velocity_enu))
            peer.battery_percentage = p.get("battery_percentage", peer.battery_percentage)
            peer.is_armed = p.get("is_armed", getattr(peer, "is_armed", False))
            peer.is_offboard = p.get("is_offboard", getattr(peer, "is_offboard", False))
            peer.stability_verified = p.get("stability_verified", getattr(peer, "stability_verified", False))

            # 2b. Evaluate Asymmetric Partition / One-Way Failure & Local RTT
            peer_seen_seqs = p.get("last_seen_peer_sequences", {})
            if peer_seen_seqs and self.local_outbound_sequence >= 3:
                peer_heard_my_seq = peer_seen_seqs.get(self.local_drone_id, 0)
                peer.last_peer_acknowledged_local_seq = peer_heard_my_seq
                if peer_heard_my_seq in self.outbound_packet_timestamps:
                    sent_mono = self.outbound_packet_timestamps[peer_heard_my_seq]
                    rtt_sample = max(0.0, now_monotonic - sent_mono)
                    peer.last_rtt_sample_sec = rtt_sample
                    if peer.rtt_sec_ema is None:
                        peer.rtt_sec_ema = rtt_sample
                    else:
                        peer.rtt_sec_ema = 0.80 * peer.rtt_sec_ema + 0.20 * rtt_sample

                seq_gap = (self.local_outbound_sequence - peer_heard_my_seq) & 0xFFFFFFFF
                if peer_heard_my_seq == 0:
                    is_asymmetric_failure = True
                elif 0 < seq_gap < 0x80000000 and seq_gap >= 3:
                    is_asymmetric_failure = True

        elif msg.header.message_type == MessageType.ACK:
            acked_seq = p.get("acked_sequence", 0)
            if acked_seq in self.outbound_packet_timestamps:
                sent_mono = self.outbound_packet_timestamps[acked_seq]
                rtt_sample = max(0.0, now_monotonic - sent_mono)
                peer.last_rtt_sample_sec = rtt_sample
                if peer.rtt_sec_ema is None:
                    peer.rtt_sec_ema = rtt_sample
                else:
                    peer.rtt_sec_ema = 0.80 * peer.rtt_sec_ema + 0.20 * rtt_sample

        elif msg.header.message_type == MessageType.INTENT_TRAJECTORY:
            peer.last_grid_cell = list(p.get("current_grid", peer.last_grid_cell))
            peer.target_grid_cell = list(p.get("target_grid", peer.target_grid_cell or peer.last_grid_cell))
            peer.last_position_enu = list(p.get("start_world_enu", p.get("current_world_enu", peer.last_position_enu)))
            peer.last_velocity_enu = list(p.get("velocity_vector", peer.last_velocity_enu))

        # 3. Handle Reconnection State Hysteresis & Asymmetric Failure
        if is_asymmetric_failure:
            peer.comms_health = CommsHealthState.ONE_WAY_FAILURE
            peer.consecutive_reconnect_packets = 0
        elif peer.comms_health in (CommsHealthState.ISOLATED, CommsHealthState.UNKNOWN, CommsHealthState.ONE_WAY_FAILURE):
            peer.comms_health = CommsHealthState.RECONNECTING
            peer.consecutive_reconnect_packets = 1
        elif peer.comms_health == CommsHealthState.RECONNECTING:
            peer.consecutive_reconnect_packets += 1
            if peer.consecutive_reconnect_packets >= self.reconnect_consecutive_needed:
                peer.comms_health = CommsHealthState.CONNECTED
        else:
            peer.comms_health = CommsHealthState.CONNECTED

        self._update_overall_comms_state()
        return True

    def tick_health_fsm(self, current_time: Optional[float] = None):
        """Ticks communication health state machines for all peers based on silence timeouts."""
        now = current_time if current_time is not None else time.monotonic()
        for peer_id, peer in self.peers.items():
            silence_age = self.get_silence_age_sec(peer_id, now)
            if silence_age >= self.unknown_timeout:
                peer.comms_health = CommsHealthState.UNKNOWN
                peer.consecutive_reconnect_packets = 0
            elif silence_age >= self.isolated_timeout:
                peer.comms_health = CommsHealthState.ISOLATED
                peer.consecutive_reconnect_packets = 0
            elif silence_age >= self.degraded_timeout:
                if peer.comms_health not in (CommsHealthState.RECONNECTING, CommsHealthState.ONE_WAY_FAILURE):
                    peer.comms_health = CommsHealthState.DEGRADED
            else:
                if peer.comms_health not in (CommsHealthState.RECONNECTING, CommsHealthState.ISOLATED, CommsHealthState.UNKNOWN, CommsHealthState.ONE_WAY_FAILURE):
                    peer.comms_health = CommsHealthState.CONNECTED

        self._update_overall_comms_state()

    def update_comms_health(self, current_time: Optional[float] = None):
        """Alias for tick_health_fsm."""
        self.tick_health_fsm(current_time)

    def _update_overall_comms_state(self):
        """Aggregate health state across all tracked peers."""
        if not self.peers:
            self.overall_comms_state = CommsHealthState.CONNECTED
            return
        states = [p.comms_health for p in self.peers.values()]
        if any(s == CommsHealthState.ONE_WAY_FAILURE for s in states):
            self.overall_comms_state = CommsHealthState.ONE_WAY_FAILURE
        elif any(s == CommsHealthState.UNKNOWN for s in states):
            self.overall_comms_state = CommsHealthState.UNKNOWN
        elif any(s == CommsHealthState.ISOLATED for s in states):
            self.overall_comms_state = CommsHealthState.DEGRADED if any(s == CommsHealthState.CONNECTED for s in states) else CommsHealthState.ISOLATED
        elif any(s == CommsHealthState.DEGRADED for s in states):
            self.overall_comms_state = CommsHealthState.DEGRADED
        elif any(s == CommsHealthState.RECONNECTING for s in states):
            self.overall_comms_state = CommsHealthState.RECONNECTING
        else:
            self.overall_comms_state = CommsHealthState.CONNECTED
