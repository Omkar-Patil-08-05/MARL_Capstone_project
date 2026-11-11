"""
Peer-to-Peer Decentralized Coordination Package (Phase 3 Additive Implementation)
Exports schema, network impairment simulation, transport, state management,
safety envelope, risk-triggered scheduler, and distributed event journal.
"""

from .peer_message_schema import (
    MessageType,
    MessagePriority,
    PeerHeader,
    PeerMessage,
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
from .network_impairment import (
    NetworkImpairmentConfig,
    NetworkImpairmentSimulator
)
from .peer_transport import (
    BasePeerTransport,
    LoopbackPeerTransport,
    ROSTopicPeerTransport
)
from .peer_state_manager import (
    PeerStateManager,
    CommsHealthState,
    PeerRecord
)
from .peer_safety_envelope import (
    PeerSafetyEnvelope,
    SafetyVerdict,
    SafetyCheckResult
)
from .risk_triggered_scheduler import (
    RiskTriggeredScheduler,
    BandwidthStats
)
from .distributed_event_journal import (
    DistributedEventJournal,
    ReconciledVictim,
    ReconciledTask,
    VictimSighting,
    CompetingAssignmentLog,
    JournalEntry
)
from .victim_perception import (
    SimulatedVictimSensor,
    PerceivedVictimDetection,
    PerceptionTrackletTracker
)
from .qmix_adapter import QMIXAdapter
try:
    from .decentralized_drone_node import DecentralizedDroneNode
except ImportError:
    DecentralizedDroneNode = None

try:
    from .p2p_ground_station import P2PGroundStation
except ImportError:
    P2PGroundStation = None

__all__ = [
    "MessageType",
    "MessagePriority",
    "PeerHeader",
    "PeerMessage",
    "EmergencyPayload",
    "VictimConfirmedPayload",
    "TaskStatePayload",
    "IntentTrajectoryPayload",
    "HeartbeatPayload",
    "NetworkImpairmentConfig",
    "NetworkImpairmentSimulator",
    "BasePeerTransport",
    "LoopbackPeerTransport",
    "ROSTopicPeerTransport",
    "PeerStateManager",
    "CommsHealthState",
    "PeerRecord",
    "PeerSafetyEnvelope",
    "SafetyVerdict",
    "SafetyCheckResult",
    "RiskTriggeredScheduler",
    "BandwidthStats",
    "DistributedEventJournal",
    "ReconciledVictim",
    "ReconciledTask",
    "JournalEntry",
    "QMIXAdapter",
    "SimulatedVictimSensor",
    "PerceivedVictimDetection",
    "PerceptionTrackletTracker",
    "DecentralizedDroneNode",
    "P2PGroundStation"
]
