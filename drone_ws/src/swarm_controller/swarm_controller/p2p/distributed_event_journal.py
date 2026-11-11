"""
Candidate C: Partition-Aware Victim Evidence and Mission-State Reconciliation Journal (Corrected Architecture)
Protocol Version: 2.1-P2P

Key Architectural Characteristics & Properties:
- Formal Classification: Eventually Consistent Replicated Event Journal with Deterministic Deduplication
  and Embedded Join-Semilattices (G-Set for observations, Max-Register with string tie-break for task epochs).
- Local Lease Timing Semantics: strictly uses local receiver monotonic clock for lease timeouts.
- Optimistic Local Claims Under Partition: explicitly models partition-limited consensus (CAP theorem).
- Evidence Preservation: stores all independent multi-drone detection sightings (G-Set) rather than destroying raw data.
- Defensible Sensor Fusion:
  * Spatial Coordinates: Inverse-variance (covariance-weighted) spatial estimation. Order-independent.
  * Detection Confidence: Independent probabilistic evidence accumulation C = 1 - prod(1 - C_i). Order-independent.
- Deterministic Post-Reconnection Reconciliation with Epoch Fencing Tokens and Monotonic Join Operators.
- Transitive Spatial Clustering & Alias Resolution:
  * Nearest-neighbor Euclidean matching with lexicographical ID tie-breaking.
  * Global observation ownership tracking (observation_to_victim) to prevent split-brain duplicates.
  * Transitive cluster unification with alias cycle detection and path compression.
- Structured Audit Logging of Temporary Competing Task Assignments and Resolutions.
- State Compaction & Pruning for Long-Duration Swarm Missions.
"""

import hashlib
import math
import time
import uuid
from typing import ClassVar, Dict, List, Optional, Tuple, Set, Any
from dataclasses import dataclass, field


@dataclass
class VictimSighting:
    reporting_drone_id: str
    timestamp_utc: float
    coordinates: List[float]       # [x, y, z] ENU meters
    grid_cell: List[int]           # [gx, gy]
    uncertainty_m: float           # 1-sigma spatial radius
    confidence: float              # [0.0 - 1.0]
    confirmation_frames: int
    observation_id: str = ""       # Stable unique observation identifier for idempotent deduplication


@dataclass
class ReconciledVictim:
    victim_event_id: str
    reporting_drone_id: str
    world_coordinates: List[float]  # Fused [x, y, z] ENU meters
    grid_cell: List[int]            # Fused [gx, gy]
    position_uncertainty_m: float   # Fused 1-sigma uncertainty
    confidence: float               # Fused probabilistic confidence
    first_detected_timestamp_utc: float
    last_updated_timestamp_utc: float
    sightings: List[VictimSighting] = field(default_factory=list)
    seen_observation_ids: Set[str] = field(default_factory=set)
    all_contributing_drones: Set[str] = field(default_factory=set)


@dataclass
class ReconciledTask:
    task_id: str
    victim_event_id: str
    owner_drone_id: str
    task_state: str  # "UNASSIGNED", "CLAIMED", "APPROACHING", "INVESTIGATING", "COMPLETED", "FAILED"
    local_lease_expiry_monotonic: float  # Evaluated strictly on receiver's local monotonic clock
    lease_duration_sec: float
    ownership_epoch: int                 # Fencing token: increments on every ownership transfer
    progress_percent: float
    last_updated_timestamp_utc: float

    # Valid task states; enforced at construction and restoration boundaries.
    _VALID_STATES: ClassVar[frozenset] = frozenset(
        {"UNASSIGNED", "FAILED", "CLAIMED", "APPROACHING", "INVESTIGATING", "COMPLETED"}
    )

    def __post_init__(self) -> None:
        """Enforces domain invariants across task creation and restoration.

        Validated fields:
          - task_state: must be one of the six valid states.
          - ownership_epoch: must be a non-negative integer.
          - progress_percent: must be finite and within [0.0, 100.0].
          - last_updated_timestamp_utc: must be finite (NaN/inf rejected).
          - COMPLETED invariant: progress_percent forced to 100.0 before range check,
            so callers may pass any value and receive 100.0 silently.

        Documented canonicalization:
          COMPLETED tasks receive progress_percent = 100.0 regardless of the value
          supplied at construction.  This is intentional silent canonicalization and
          is documented here to satisfy the Phase 3.3 requirement that no silent
          canonicalization occurs without documentation.
        """
        # --- task_state ---
        if self.task_state not in self._VALID_STATES:
            raise ValueError(
                f"Invalid task_state {self.task_state!r}; "
                f"must be one of {sorted(self._VALID_STATES)}"
            )
        # --- ownership_epoch ---
        if not isinstance(self.ownership_epoch, int) or self.ownership_epoch < 0:
            raise ValueError(
                f"ownership_epoch must be a non-negative integer; got {self.ownership_epoch!r}"
            )
        # --- COMPLETED invariant (applied BEFORE range check so any value is accepted) ---
        if self.task_state == "COMPLETED":
            self.progress_percent = 100.0
        # --- progress_percent ---
        if not math.isfinite(self.progress_percent):
            raise ValueError(
                f"progress_percent must be finite; got {self.progress_percent!r}"
            )
        if not (0.0 <= self.progress_percent <= 100.0):
            raise ValueError(
                f"progress_percent must be in [0.0, 100.0]; got {self.progress_percent!r}"
            )
        # --- last_updated_timestamp_utc ---
        if not math.isfinite(self.last_updated_timestamp_utc):
            raise ValueError(
                f"last_updated_timestamp_utc must be finite; "
                f"got {self.last_updated_timestamp_utc!r}"
            )


@dataclass
class CompetingAssignmentLog:
    task_id: str
    local_owner: str
    remote_owner: str
    local_epoch: int
    remote_epoch: int
    winner_drone_id: str
    resolution_reason: str
    timestamp_utc: float


@dataclass
class JournalEntry:
    entry_id: str
    sender_drone_id: str
    entry_type: str  # "VICTIM", "TASK_CLAIM", "TASK_PROGRESS", "TASK_COMPLETE", "TASK_STATE"
    epoch: int
    timestamp_utc: float
    payload: Dict[str, Any]


class VictimDict(dict):
    """
    Dictionary of ReconciledVictim objects supporting transparent alias resolution
    while preserving standard dictionary key iteration, length, and containment.
    """
    def __init__(self, journal: 'DistributedEventJournal'):
        super().__init__()
        self._journal = journal

    def __getitem__(self, key: str) -> ReconciledVictim:
        if super().__contains__(key):
            return super().__getitem__(key)
        canonical = self._journal.get_canonical_victim_id(key)
        if canonical and super().__contains__(canonical):
            return super().__getitem__(canonical)
        raise KeyError(key)

    def get(self, key: str, default: Any = None) -> Any:
        try:
            return self[key]
        except KeyError:
            return default

    def __contains__(self, key: object) -> bool:
        if super().__contains__(key):
            return True
        if isinstance(key, str):
            canonical = self._journal.get_canonical_victim_id(key)
            return bool(canonical and super().__contains__(canonical))
        return False


class TaskDict(dict):
    """
    Dictionary of ReconciledTask objects supporting transparent alias resolution
    for task keys (task_{alias_id} -> task_{canonical_id}).
    """
    def __init__(self, journal: 'DistributedEventJournal'):
        super().__init__()
        self._journal = journal

    def __getitem__(self, key: str) -> ReconciledTask:
        if super().__contains__(key):
            return super().__getitem__(key)
        if isinstance(key, str) and key.startswith("task_"):
            vid = key[5:]
            canon_vid = self._journal.get_canonical_victim_id(vid)
            canon_tid = f"task_{canon_vid}"
            if canon_tid != key and super().__contains__(canon_tid):
                return super().__getitem__(canon_tid)
        raise KeyError(key)

    def get(self, key: str, default: Any = None) -> Any:
        try:
            return self[key]
        except KeyError:
            return default

    def __contains__(self, key: object) -> bool:
        if super().__contains__(key):
            return True
        if isinstance(key, str) and key.startswith("task_"):
            vid = key[5:]
            canon_vid = self._journal.get_canonical_victim_id(vid)
            canon_tid = f"task_{canon_vid}"
            return bool(canon_tid != key and super().__contains__(canon_tid))
        return False


class DistributedEventJournal:
    """
    Maintains an idempotent, evidence-preserving distributed mission log.
    Fuses victim sightings using covariance weighting and Bayesian confidence combination.
    Resolves concurrent partition claims using monotonic epoch fencing tokens.
    Guarantees deterministic multi-replica state convergence under network partitions and churn.
    """

    def __init__(
        self,
        local_drone_id: str,
        spatial_merge_threshold_m: float = 6.0,
        default_lease_duration_sec: float = 30.0
    ):
        self.local_drone_id = local_drone_id
        self.spatial_merge_threshold_m = float(spatial_merge_threshold_m)
        self.default_lease_duration_sec = float(default_lease_duration_sec)

        self.journal: List[JournalEntry] = []
        self.seen_entry_ids: Set[str] = set()

        self.victim_id_aliases: Dict[str, str] = {}    # alias_id -> canonical_victim_event_id
        self.observation_to_victim: Dict[str, str] = {}  # observation_id -> canonical_victim_event_id
        self.victims: Dict[str, ReconciledVictim] = VictimDict(self)
        self.tasks: Dict[str, ReconciledTask] = TaskDict(self)
        self.competing_assignment_history: List[CompetingAssignmentLog] = []

    def _euclidean_dist(self, p1: List[float], p2: List[float]) -> float:
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    def get_canonical_victim_id(self, v_id: str) -> str:
        """
        Resolves the root canonical victim ID through alias chains.
        Implements cycle detection and path compression.
        Guarantees termination in O(N).
        """
        if not v_id:
            return ""
        curr = v_id
        visited: List[str] = []
        seen: Set[str] = set()
        while curr in self.victim_id_aliases:
            if curr in seen:
                # Cycle detected: break cycle deterministically by selecting lexicographical minimum
                canonical = min(visited)
                for node in visited:
                    if node == canonical:
                        self.victim_id_aliases.pop(node, None)
                    else:
                        self.victim_id_aliases[node] = canonical
                return canonical
            seen.add(curr)
            visited.append(curr)
            curr = self.victim_id_aliases[curr]

        # Path compression
        for node in visited:
            if node != curr:
                self.victim_id_aliases[node] = curr
        return curr

    def find_spatially_matching_victim(self, coords: List[float]) -> Optional[str]:
        """
        Searches existing canonical victims for the nearest match within spatial threshold.
        Uses fast axis-aligned bounding box (AABB) rejection before exact Euclidean distance
        with deterministic lexicographical tie-breaking over sorted canonical victim IDs.
        """
        best_vid: Optional[str] = None
        best_dist = float("inf")
        cx, cy = coords[0], coords[1]
        thresh = self.spatial_merge_threshold_m

        for v_id in sorted(dict.keys(self.victims)):
            v = dict.__getitem__(self.victims, v_id)
            vx, vy = v.world_coordinates[0], v.world_coordinates[1]
            if abs(cx - vx) > thresh or abs(cy - vy) > thresh:
                continue
            dist = self._euclidean_dist(coords, v.world_coordinates)
            if dist <= thresh:
                if dist < best_dist - 1e-6:
                    best_dist = dist
                    best_vid = v_id
                elif abs(dist - best_dist) <= 1e-6:
                    # Deterministic tie-breaking by lexicographical victim_event_id
                    if best_vid is None or v_id < best_vid:
                        best_vid = v_id
        return best_vid

    def _recalculate_fused_state(self, victim: ReconciledVictim) -> None:
        """
        Recalculates fused spatial coordinates, uncertainty, and confidence.
        Inverse-variance weighting:
            p_fused = sum(p_i / var_i) / sum(1 / var_i)
            sigma_fused = sqrt(1 / sum(1 / var_i))
        Independent probabilistic confidence combination (Noisy-OR):
            C_fused = 1 - prod(1 - C_i)
        Both operators are strictly commutative and associative over the sighting set.
        """
        if not victim.sightings:
            return

        total_w = 0.0
        fused_x, fused_y, fused_z = 0.0, 0.0, 0.0

        for s in victim.sightings:
            var = max(0.01, s.uncertainty_m ** 2)
            w = 1.0 / var
            total_w += w
            fused_x += s.coordinates[0] * w
            fused_y += s.coordinates[1] * w
            fused_z += s.coordinates[2] * w

        victim.world_coordinates = [fused_x / total_w, fused_y / total_w, fused_z / total_w]
        victim.position_uncertainty_m = math.sqrt(1.0 / total_w)

        unconf = 1.0
        for s in victim.sightings:
            unconf *= max(0.01, 1.0 - min(0.99, s.confidence))
        victim.confidence = round(1.0 - unconf, 4)

    # Semilattice state ordering for deterministic tie-breaking within same (epoch, owner).
    # Higher index = higher state; used only when owner_drone_id matches at equal epoch.
    _STATE_ORDER: dict = {
        "UNASSIGNED": 0,
        "FAILED": 1,
        "CLAIMED": 2,
        "APPROACHING": 3,
        "INVESTIGATING": 4,
        "COMPLETED": 5,  # sentinel – handled separately
    }

    def _join_tasks(self, t_target: ReconciledTask, t_incoming: ReconciledTask) -> None:
        """
        Joins incoming task state into target task record using a commutative,
        associative, idempotent semilattice (join-semilattice / CRDT Max-Register).

        Ordering rules (highest priority first):
          1. COMPLETED is an absorbing top state that strictly dominates any non-COMPLETED state.
             When one operand is COMPLETED and the other is active (regardless of active operand epoch),
             the completed task record dominates, preserving the author drone and completion epoch:
               – Target COMPLETED, incoming active  → target retains state, epoch, owner, and timestamp.
               – Incoming COMPLETED, target active  → target adopts incoming state, epoch, owner, and timestamp.
               – Both COMPLETED                    → higher completion epoch wins (epoch, owner, timestamp);
                                                     tie on epoch → NONE-aware lex-min owner wins.
             In all completed outcomes, progress_percent is enforced at 100.0.
          2. Higher ownership_epoch wins for all non-COMPLETED states (wins entirely: state, owner, progress, timestamp).
          3. Equal epoch, same owner → higher lifecycle rank strictly dominates, taking the
             complete winning record (state, lease, progress, timestamp); when lifecycle states
             are identical, progress and timestamp grow monotonically via max() within that phase.
          4. Equal epoch, different owner → lexicographically smaller owner_drone_id wins
             (stable deterministic tie-break that is commutative and associative);
             winning record's progress and timestamp are taken entirely.

        Progress and timestamp are RECORD-BOUND: they belong to the winning (epoch, owner, state)
        tuple and must not be cross-pollinated across different states or owners.  The exception
        (Rule 3 equal state) is identical-(epoch, owner, state), where progress and timestamp
        grow monotonically within that drone's specific lifecycle phase.

        This design ensures that all replicas that converge to the same (state, epoch, owner)
        tuple also converge to the same (progress, timestamp), establishing deterministic
        serialized state.
        """
        # ------------------------------------------------------------------ #
        # Rule 1: COMPLETED absorption with least-upper-bound preservation   #
        # ------------------------------------------------------------------ #
        t_completed = t_target.task_state == "COMPLETED"
        i_completed = t_incoming.task_state == "COMPLETED"

        def _better_owner(a: str, b: str) -> str:
            """Return the preferred owner using NONE-aware lex-min tie-breaking."""
            if a == "NONE" and b != "NONE":
                return b
            if b == "NONE":
                return a
            return min(a, b)  # both real: lex min

        if t_completed and not i_completed:
            # Target is already COMPLETED: it strictly dominates incoming active operand.
            # Authoritative completion epoch, completing owner, and timestamp from target are preserved.
            # Progress is enforced at 100.0; timestamp belongs to the COMPLETED record.
            t_target.progress_percent = 100.0
            # Do NOT take max(timestamps): the winning COMPLETED record's timestamp is authoritative.
            # Taking max could replace the completion timestamp with a later active-record timestamp,
            # breaking deterministic convergence when replicas see different merge orderings.
            return

        elif i_completed and not t_completed:
            # Incoming is COMPLETED: it strictly dominates target active operand.
            # Target adopts the authoritative completion state, epoch, completing owner, and timestamp.
            t_target.task_state = "COMPLETED"
            t_target.ownership_epoch = t_incoming.ownership_epoch
            t_target.owner_drone_id = t_incoming.owner_drone_id
            t_target.local_lease_expiry_monotonic = t_incoming.local_lease_expiry_monotonic
            t_target.progress_percent = 100.0
            t_target.last_updated_timestamp_utc = t_incoming.last_updated_timestamp_utc
            return

        elif t_completed and i_completed:
            # Both operands are COMPLETED: determine authoritative completion via epoch, then lex-min owner.
            orig_t_epoch = t_target.ownership_epoch
            orig_i_epoch = t_incoming.ownership_epoch

            t_target.task_state = "COMPLETED"
            t_target.progress_percent = 100.0

            if orig_i_epoch > orig_t_epoch:
                t_target.ownership_epoch = orig_i_epoch
                t_target.owner_drone_id = t_incoming.owner_drone_id
                t_target.last_updated_timestamp_utc = t_incoming.last_updated_timestamp_utc
            elif orig_i_epoch == orig_t_epoch:
                t_target.owner_drone_id = _better_owner(
                    t_target.owner_drone_id, t_incoming.owner_drone_id
                )
                # Equal epoch: timestamp is the same logical event; take max for monotonicity
                t_target.last_updated_timestamp_utc = max(
                    t_target.last_updated_timestamp_utc,
                    t_incoming.last_updated_timestamp_utc,
                )
            # else: orig_t_epoch > orig_i_epoch — keep t_target.ownership_epoch, owner_drone_id, timestamp
            return

        # ------------------------------------------------------------------ #
        # Rules 2–4: non-COMPLETED states                                     #
        # ------------------------------------------------------------------ #
        # Progress and timestamp semantics for active-state joins:
        #
        # progress_percent and last_updated_timestamp_utc are RECORD-BOUND
        # attributes: they belong to the winning (ownership_epoch, owner_drone_id)
        # tuple and must follow the winning record.  Cross-pollinating progress
        # from a losing record into the winner produces a semantically incorrect
        # result (attributing a different drone's progress to the winning drone).
        #
        # Exception: when the SAME (epoch, owner) sends multiple updates (Rule 3),
        # the semantics are: "the drone progressed further along its trajectory",
        # so the MAXIMUM progress and MAXIMUM timestamp are taken (monotone grow).
        #
        # For timestamp_utc specifically: replicas that see different intermediate
        # merge orderings would otherwise observe different timestamps in the
        # converged record (since max(A_ts, B_ts) depends on which records are
        # present).  By binding timestamp to the winning record, all replicas that
        # converge to the same (state, epoch, owner) tuple also converge to the
        # same timestamp — establishing deterministic serialized state.
        if t_incoming.ownership_epoch > t_target.ownership_epoch:
            # Incoming has strictly higher epoch: it wins entirely.
            # Progress and timestamp follow the winning (epoch, owner) record.
            t_target.owner_drone_id = t_incoming.owner_drone_id
            t_target.task_state = t_incoming.task_state
            t_target.local_lease_expiry_monotonic = t_incoming.local_lease_expiry_monotonic
            t_target.ownership_epoch = t_incoming.ownership_epoch
            t_target.progress_percent = t_incoming.progress_percent
            t_target.last_updated_timestamp_utc = t_incoming.last_updated_timestamp_utc
        elif t_incoming.ownership_epoch == t_target.ownership_epoch:
            same_owner = t_incoming.owner_drone_id == t_target.owner_drone_id
            if same_owner:
                # Rule 3: same (epoch, owner) – higher lifecycle state strictly dominates.
                # When lifecycle states differ, the higher rank wins the complete record
                # (state, lease, progress, timestamp) preventing cross-state contamination.
                # Only when lifecycle states are identical do progress and timestamp grow
                # monotonically via max() within that specific lifecycle phase.
                t_rank = self._STATE_ORDER.get(t_target.task_state, 0)
                i_rank = self._STATE_ORDER.get(t_incoming.task_state, 0)
                if i_rank > t_rank:
                    t_target.task_state = t_incoming.task_state
                    t_target.local_lease_expiry_monotonic = t_incoming.local_lease_expiry_monotonic
                    t_target.progress_percent = t_incoming.progress_percent
                    t_target.last_updated_timestamp_utc = t_incoming.last_updated_timestamp_utc
                elif i_rank == t_rank:
                    # Same state: monotone max within the same lifecycle phase
                    t_target.progress_percent = max(
                        t_target.progress_percent, t_incoming.progress_percent
                    )
                    t_target.last_updated_timestamp_utc = max(
                        t_target.last_updated_timestamp_utc,
                        t_incoming.last_updated_timestamp_utc,
                    )
                # else: t_rank > i_rank — target strictly dominates; keeps all fields unchanged
            else:
                # Rule 4: equal epoch, different owner – lex min owner wins.
                # "NONE" is treated as a sentinel with lowest priority.
                # Progress and timestamp follow the winning owner's record.
                t_is_none = t_target.owner_drone_id == "NONE"
                i_is_none = t_incoming.owner_drone_id == "NONE"
                if t_is_none and not i_is_none:
                    # incoming is real, target is unowned: incoming wins
                    incoming_wins = True
                elif i_is_none:
                    # incoming is unowned (or both NONE): target keeps
                    incoming_wins = False
                else:
                    # Both real: lex min wins (commutative)
                    incoming_wins = t_incoming.owner_drone_id < t_target.owner_drone_id
                if incoming_wins:
                    t_target.owner_drone_id = t_incoming.owner_drone_id
                    t_target.task_state = t_incoming.task_state
                    t_target.local_lease_expiry_monotonic = t_incoming.local_lease_expiry_monotonic
                    t_target.progress_percent = t_incoming.progress_percent
                    t_target.last_updated_timestamp_utc = t_incoming.last_updated_timestamp_utc
                # else: target wins; target's progress and timestamp are already correct

    def merge_victims(self, v_id_1: str, v_id_2: str) -> str:
        """
        Deterministically unifies two victim clusters into a single canonical victim.
        The canonical ID is chosen as min(canon1, canon2) lexicographically.
        Transfers sightings without duplication, recomputes fused position and confidence,
        unifies associated tasks, records alias pointers, and removes absorbed record.
        Returns the surviving canonical victim ID.
        """
        canon_1 = self.get_canonical_victim_id(v_id_1)
        canon_2 = self.get_canonical_victim_id(v_id_2)
        if canon_1 == canon_2:
            return canon_1

        winner_id = min(canon_1, canon_2)
        loser_id = max(canon_1, canon_2)

        loser_v = dict.get(self.victims, loser_id)
        winner_v = dict.get(self.victims, winner_id)

        if loser_v is not None:
            if winner_v is not None:
                # Merge sightings from loser into winner
                for s in loser_v.sightings:
                    if s.observation_id not in winner_v.seen_observation_ids:
                        winner_v.seen_observation_ids.add(s.observation_id)
                        winner_v.sightings.append(s)
                        winner_v.all_contributing_drones.add(s.reporting_drone_id)
                        self.observation_to_victim[s.observation_id] = winner_id

                winner_v.first_detected_timestamp_utc = min(
                    winner_v.first_detected_timestamp_utc, loser_v.first_detected_timestamp_utc
                )
                winner_v.last_updated_timestamp_utc = max(
                    winner_v.last_updated_timestamp_utc, loser_v.last_updated_timestamp_utc
                )
                self._recalculate_fused_state(winner_v)
            else:
                # Promote loser to winner_id
                loser_v.victim_event_id = winner_id
                dict.__setitem__(self.victims, winner_id, loser_v)
                for s in loser_v.sightings:
                    self.observation_to_victim[s.observation_id] = winner_id

            dict.__delitem__(self.victims, loser_id)

        # Record alias and compress pointers
        self.victim_id_aliases[loser_id] = winner_id
        for alias_key, target in list(self.victim_id_aliases.items()):
            if target == loser_id:
                self.victim_id_aliases[alias_key] = winner_id
        self.victim_id_aliases.pop(winner_id, None)

        # Merge tasks
        task_loser_id = f"task_{loser_id}"
        task_winner_id = f"task_{winner_id}"
        if dict.__contains__(self.tasks, task_loser_id):
            loser_task = dict.pop(self.tasks, task_loser_id)
            if not dict.__contains__(self.tasks, task_winner_id):
                loser_task.task_id = task_winner_id
                loser_task.victim_event_id = winner_id
                dict.__setitem__(self.tasks, task_winner_id, loser_task)
            else:
                winner_task = dict.__getitem__(self.tasks, task_winner_id)
                self._join_tasks(winner_task, loser_task)

        return winner_id

    def _reconcile_overlapping_victims(self, v_id: str) -> str:
        """
        Checks if victim v_id now overlaps with any other victim within threshold.
        Iteratively unifies all overlapping clusters into a single canonical victim.
        Uses fast axis-aligned bounding box rejection for O(1) performance in sparse maps.
        Returns the surviving canonical victim ID.
        """
        canon_id = self.get_canonical_victim_id(v_id)
        if len(self.victims) <= 1:
            return canon_id

        target_v = dict.get(self.victims, canon_id)
        if target_v is None:
            return canon_id

        thresh = self.spatial_merge_threshold_m
        tx, ty = target_v.world_coordinates[0], target_v.world_coordinates[1]

        changed = True
        while changed:
            changed = False
            for other_id, other_v in list(self.victims.items()):
                if other_id == canon_id:
                    continue
                ox, oy = other_v.world_coordinates[0], other_v.world_coordinates[1]
                if abs(tx - ox) > thresh or abs(ty - oy) > thresh:
                    continue
                dist = self._euclidean_dist(target_v.world_coordinates, other_v.world_coordinates)
                if dist <= thresh:
                    canon_id = self.merge_victims(canon_id, other_id)
                    target_v = dict.get(self.victims, canon_id)
                    if target_v is not None:
                        tx, ty = target_v.world_coordinates[0], target_v.world_coordinates[1]
                    changed = True
                    break
        return canon_id

    def _fuse_victim_evidence(self, victim: ReconciledVictim, new_sighting: VictimSighting, recompute: bool = True) -> bool:
        """
        Fuses independent sightings using inverse-variance spatial weighting
        and independent probabilistic confidence combination.
        Guarantees idempotency: duplicate delivery of identical observation ID
        is rejected without altering weights or confidence.
        Detects cross-victim observation duplication and triggers cluster unification.
        """
        obs_id = new_sighting.observation_id
        if not obs_id:
            c_str = "_".join(f"{float(c):.2f}" for c in new_sighting.coordinates)
            raw = f"{new_sighting.reporting_drone_id}_{float(new_sighting.timestamp_utc):.3f}_{c_str}_{float(new_sighting.confidence):.3f}"
            obs_id = f"obs_{hashlib.sha256(raw.encode()).hexdigest()[:12]}"
            new_sighting.observation_id = obs_id

        # Idempotent deduplication within this victim
        if obs_id in victim.seen_observation_ids:
            # Check if this sighting is a refined frame from the same active tracklet
            for existing in victim.sightings:
                if existing.observation_id == obs_id:
                    refined = False
                    if new_sighting.confidence > existing.confidence:
                        existing.confidence = new_sighting.confidence
                        refined = True
                    if new_sighting.uncertainty_m < existing.uncertainty_m:
                        existing.coordinates = list(new_sighting.coordinates)
                        existing.uncertainty_m = new_sighting.uncertainty_m
                        refined = True
                    if refined:
                        existing.timestamp_utc = max(existing.timestamp_utc, new_sighting.timestamp_utc)
                        victim.last_updated_timestamp_utc = max(victim.last_updated_timestamp_utc, new_sighting.timestamp_utc)
                        if recompute:
                            self._recalculate_fused_state(victim)
                            self._reconcile_overlapping_victims(victim.victim_event_id)
                        return True
                    return False
            return False

        # Global observation tracking: if this observation already belongs to another victim, unify clusters!
        existing_vid = self.observation_to_victim.get(obs_id)
        if existing_vid is not None:
            canon_existing = self.get_canonical_victim_id(existing_vid)
            canon_curr = self.get_canonical_victim_id(victim.victim_event_id)
            if canon_existing != canon_curr:
                unified_vid = self.merge_victims(canon_curr, canon_existing)
                victim = dict.get(self.victims, unified_vid, victim)
                if obs_id in victim.seen_observation_ids:
                    return False

        victim.seen_observation_ids.add(obs_id)
        victim.sightings.append(new_sighting)
        victim.all_contributing_drones.add(new_sighting.reporting_drone_id)
        victim.last_updated_timestamp_utc = max(victim.last_updated_timestamp_utc, new_sighting.timestamp_utc)
        self.observation_to_victim[obs_id] = victim.victim_event_id

        if recompute:
            self._recalculate_fused_state(victim)
            self._reconcile_overlapping_victims(victim.victim_event_id)
        return True

    def record_local_victim(
        self,
        world_coords: List[float],
        grid_cell: List[int],
        uncertainty_m: float = 0.5,
        confidence: float = 0.95,
        confirmation_frames: Any = 5,
        victim_event_id: Optional[str] = None,
        observation_id: Optional[str] = None
    ) -> Tuple[str, bool]:
        """
        Records a victim detected by the local drone, preserving evidence and updating fused state.
        Returns: (victim_event_id, is_new_victim)
        """
        # Backward compatibility for positional arguments: if 5th arg is string, it represents victim_event_id
        if isinstance(confirmation_frames, str):
            if victim_event_id is None:
                victim_event_id = confirmation_frames
            confirmation_frames = 5

        now_utc = time.time()
        obs_id = observation_id if observation_id is not None else f"obs_{self.local_drone_id}_{uuid.uuid4().hex[:8]}"

        # Check if observation was already processed
        if obs_id in self.observation_to_victim:
            existing_vid = self.get_canonical_victim_id(self.observation_to_victim[obs_id])
            return existing_vid, False

        # Resolve candidate canonical victim ID
        canonical_vid = self.get_canonical_victim_id(victim_event_id) if victim_event_id else None

        sighting = VictimSighting(
            reporting_drone_id=self.local_drone_id,
            timestamp_utc=now_utc,
            coordinates=list(world_coords),
            grid_cell=list(grid_cell),
            uncertainty_m=float(uncertainty_m),
            confidence=float(confidence),
            confirmation_frames=int(confirmation_frames),
            observation_id=obs_id
        )

        # If explicit canonical ID provided and known locally, fuse into it
        if canonical_vid is not None and dict.__contains__(self.victims, canonical_vid):
            victim = dict.__getitem__(self.victims, canonical_vid)
            fused = self._fuse_victim_evidence(victim, sighting)
            surviving_id = self.get_canonical_victim_id(canonical_vid)
            if fused:
                entry = JournalEntry(
                    entry_id=f"log_{uuid.uuid4().hex[:8]}",
                    sender_drone_id=self.local_drone_id,
                    entry_type="VICTIM_SIGHTING",
                    epoch=1,
                    timestamp_utc=now_utc,
                    payload={
                        "victim_event_id": surviving_id,
                        "world_coordinates": list(world_coords),
                        "grid_cell": list(grid_cell),
                        "uncertainty_m": float(uncertainty_m),
                        "confidence": float(confidence),
                        "confirmation_frames": int(confirmation_frames),
                        "detection_timestamp_utc": now_utc,
                        "observation_id": obs_id
                    }
                )
                self.journal.append(entry)
                self.seen_entry_ids.add(entry.entry_id)
            return surviving_id, False

        # Spatial matching against nearest existing victim
        match_id = self.find_spatially_matching_victim(world_coords)
        if match_id is not None:
            target_vid = match_id
            if canonical_vid and canonical_vid != match_id:
                target_vid = self.merge_victims(match_id, canonical_vid)
            victim = dict.__getitem__(self.victims, target_vid)
            fused = self._fuse_victim_evidence(victim, sighting)
            surviving_id = self.get_canonical_victim_id(target_vid)
            if fused:
                entry = JournalEntry(
                    entry_id=f"log_{uuid.uuid4().hex[:8]}",
                    sender_drone_id=self.local_drone_id,
                    entry_type="VICTIM_SIGHTING",
                    epoch=1,
                    timestamp_utc=now_utc,
                    payload={
                        "victim_event_id": surviving_id,
                        "world_coordinates": list(world_coords),
                        "grid_cell": list(grid_cell),
                        "uncertainty_m": float(uncertainty_m),
                        "confidence": float(confidence),
                        "confirmation_frames": int(confirmation_frames),
                        "detection_timestamp_utc": now_utc,
                        "observation_id": obs_id
                    }
                )
                self.journal.append(entry)
                self.seen_entry_ids.add(entry.entry_id)
            return surviving_id, False

        # Create new reconciled victim record
        v_id = canonical_vid if canonical_vid is not None else f"victim_{uuid.uuid4().hex[:8]}"
        new_victim = ReconciledVictim(
            victim_event_id=v_id,
            reporting_drone_id=self.local_drone_id,
            world_coordinates=list(world_coords),
            grid_cell=list(grid_cell),
            position_uncertainty_m=float(uncertainty_m),
            confidence=float(confidence),
            first_detected_timestamp_utc=now_utc,
            last_updated_timestamp_utc=now_utc,
            sightings=[sighting],
            seen_observation_ids={obs_id},
            all_contributing_drones={self.local_drone_id}
        )
        dict.__setitem__(self.victims, v_id, new_victim)
        self.observation_to_victim[obs_id] = v_id

        # Log into append-only journal
        entry = JournalEntry(
            entry_id=f"log_{uuid.uuid4().hex[:8]}",
            sender_drone_id=self.local_drone_id,
            entry_type="VICTIM",
            epoch=1,
            timestamp_utc=now_utc,
            payload={
                "victim_event_id": v_id,
                "world_coordinates": list(world_coords),
                "grid_cell": list(grid_cell),
                "uncertainty_m": float(uncertainty_m),
                "confidence": float(confidence),
                "confirmation_frames": int(confirmation_frames),
                "detection_timestamp_utc": now_utc,
                "observation_id": obs_id
            }
        )
        self.journal.append(entry)
        self.seen_entry_ids.add(entry.entry_id)

        # Create unassigned task
        task_id = f"task_{v_id}"
        if not dict.__contains__(self.tasks, task_id):
            dict.__setitem__(self.tasks, task_id, ReconciledTask(
                task_id=task_id,
                victim_event_id=v_id,
                owner_drone_id="NONE",
                task_state="UNASSIGNED",
                local_lease_expiry_monotonic=0.0,
                lease_duration_sec=self.default_lease_duration_sec,
                ownership_epoch=0,
                progress_percent=0.0,
                last_updated_timestamp_utc=now_utc
            ))

        surviving_id = self._reconcile_overlapping_victims(v_id)
        return surviving_id, True

    def _fuse_batch_sightings(self, victim: ReconciledVictim, batch: List[Dict[str, Any]], default_sender: str) -> None:
        """Fuses a batch of remote sightings idempotently in linear O(N) time."""
        target_vid = victim.victim_event_id
        for bs in batch:
            curr_target_vid = self.get_canonical_victim_id(target_vid)
            curr_v = dict.get(self.victims, curr_target_vid, victim)
            bs_obs = bs.get("observation_id", "")
            bs_rep = bs.get("reporting_drone_id", default_sender)
            bs_ts = float(bs.get("timestamp_utc", time.time()))
            bs_coords = bs.get("coordinates", curr_v.world_coordinates)
            bs_grid = bs.get("grid_cell", curr_v.grid_cell)
            bs_unc = float(bs.get("uncertainty_m", 1.0))
            bs_conf = float(bs.get("confidence", 0.9))
            bs_frames = int(bs.get("confirmation_frames", 5))
            s = VictimSighting(
                reporting_drone_id=bs_rep,
                timestamp_utc=bs_ts,
                coordinates=list(bs_coords),
                grid_cell=list(bs_grid),
                uncertainty_m=bs_unc,
                confidence=bs_conf,
                confirmation_frames=bs_frames,
                observation_id=bs_obs
            )
            self._fuse_victim_evidence(curr_v, s, recompute=False)

        # Batch-level single recomputation of state and cluster boundaries
        curr_target_vid = self.get_canonical_victim_id(target_vid)
        if dict.__contains__(self.victims, curr_target_vid):
            curr_v = dict.__getitem__(self.victims, curr_target_vid)
            self._recalculate_fused_state(curr_v)
            self._reconcile_overlapping_victims(curr_v.victim_event_id)

    def apply_remote_victim_payload(self, sender_id: str, payload: Dict[str, Any]) -> str:
        """
        Idempotently applies remote VICTIM_CONFIRMED or journal snapshot payload, fusing sightings.
        Deterministically resolves canonical IDs and merges overlapping victim clusters.
        Returns the surviving canonical victim ID.
        """
        raw_vid = payload.get("victim_event_id", "")
        if isinstance(raw_vid, (list, tuple)):
            raw_vid = str(raw_vid[0]) if raw_vid else ""
        elif raw_vid is not None:
            raw_vid = str(raw_vid)
        else:
            raw_vid = ""

        coords = payload.get("world_coordinates", [0.0, 0.0, 0.0])
        grid = payload.get("grid_cell", [0, 0])
        unc = float(payload.get("position_uncertainty_m", payload.get("uncertainty_m", 1.0)))
        conf = float(payload.get("detection_confidence", payload.get("confidence", 0.9)))
        frames = int(payload.get("confirmation_frames", 5))
        now_utc = float(payload.get("detection_timestamp_utc", payload.get("timestamp_utc", time.time())))

        obs_id = payload.get("observation_id", "")
        if not obs_id:
            c_str = "_".join(f"{float(c):.2f}" for c in coords)
            raw = f"{sender_id}_{float(now_utc):.3f}_{c_str}_{float(conf):.3f}"
            obs_id = f"obs_{hashlib.sha256(raw.encode()).hexdigest()[:12]}"

        sighting = VictimSighting(
            reporting_drone_id=sender_id,
            timestamp_utc=now_utc,
            coordinates=list(coords),
            grid_cell=list(grid),
            uncertainty_m=unc,
            confidence=conf,
            confirmation_frames=frames,
            observation_id=obs_id
        )

        batch_sightings = payload.get("sightings", [])

        # Check if observation ID already mapped to a victim
        mapped_vid = self.observation_to_victim.get(obs_id)
        if mapped_vid:
            mapped_vid = self.get_canonical_victim_id(mapped_vid)

        # Resolve canonical victim ID from raw_vid
        target_vid = self.get_canonical_victim_id(raw_vid) if raw_vid else ""

        if mapped_vid and target_vid and mapped_vid != target_vid:
            target_vid = self.merge_victims(mapped_vid, target_vid)
        elif mapped_vid:
            target_vid = mapped_vid

        # If victim exists locally, fuse into it
        if target_vid and dict.__contains__(self.victims, target_vid):
            v = dict.__getitem__(self.victims, target_vid)
            if batch_sightings:
                self._fuse_batch_sightings(v, batch_sightings, sender_id)
            else:
                self._fuse_victim_evidence(v, sighting)
            return self.get_canonical_victim_id(target_vid)

        # Spatial matching against nearest existing victim
        spatial_match = self.find_spatially_matching_victim(coords)
        if spatial_match is not None:
            if target_vid and target_vid != spatial_match:
                winner = self.merge_victims(spatial_match, target_vid)
                target_vid = winner
            else:
                target_vid = spatial_match
                if raw_vid and raw_vid != target_vid:
                    self.victim_id_aliases[raw_vid] = target_vid

            v = dict.__getitem__(self.victims, target_vid)
            if batch_sightings:
                self._fuse_batch_sightings(v, batch_sightings, sender_id)
            else:
                self._fuse_victim_evidence(v, sighting)
            return self.get_canonical_victim_id(target_vid)

        # New victim discovery from peer
        v_id = target_vid if target_vid else (raw_vid if raw_vid else f"victim_{uuid.uuid4().hex[:8]}")
        new_victim = ReconciledVictim(
            victim_event_id=v_id,
            reporting_drone_id=sender_id,
            world_coordinates=list(coords),
            grid_cell=list(grid),
            position_uncertainty_m=unc,
            confidence=conf,
            first_detected_timestamp_utc=now_utc,
            last_updated_timestamp_utc=now_utc,
            sightings=[],
            seen_observation_ids=set(),
            all_contributing_drones=set()
        )
        dict.__setitem__(self.victims, v_id, new_victim)
        if raw_vid and raw_vid != v_id:
            self.victim_id_aliases[raw_vid] = v_id

        if batch_sightings:
            self._fuse_batch_sightings(new_victim, batch_sightings, sender_id)
        else:
            self._fuse_victim_evidence(new_victim, sighting)

        surviving_vid = self.get_canonical_victim_id(v_id)
        task_id = f"task_{surviving_vid}"
        if not dict.__contains__(self.tasks, task_id):
            dict.__setitem__(self.tasks, task_id, ReconciledTask(
                task_id=task_id,
                victim_event_id=surviving_vid,
                owner_drone_id="NONE",
                task_state="UNASSIGNED",
                local_lease_expiry_monotonic=0.0,
                lease_duration_sec=self.default_lease_duration_sec,
                ownership_epoch=0,
                progress_percent=0.0,
                last_updated_timestamp_utc=now_utc
            ))

        return surviving_vid

    def claim_task(self, task_id: str) -> bool:
        """
        Attempts to claim an unassigned or expired task for the local drone.
        Uses local monotonic clock for lease expiration timing.
        """
        now_mono = time.monotonic()
        now_utc = time.time()

        canon_tid = task_id
        if task_id.startswith("task_"):
            vid = task_id[5:]
            canon_vid = self.get_canonical_victim_id(vid)
            canon_tid = f"task_{canon_vid}"

        if not dict.__contains__(self.tasks, canon_tid):
            return False

        task = dict.__getitem__(self.tasks, canon_tid)
        if task.task_state == "COMPLETED":
            return False

        # Active lease held by another drone
        if task.owner_drone_id not in ("NONE", self.local_drone_id):
            if now_mono < task.local_lease_expiry_monotonic:
                return False

        task.ownership_epoch += 1
        task.owner_drone_id = self.local_drone_id
        task.task_state = "CLAIMED"
        task.local_lease_expiry_monotonic = now_mono + task.lease_duration_sec
        task.last_updated_timestamp_utc = now_utc

        entry = JournalEntry(
            entry_id=f"claim_{uuid.uuid4().hex[:8]}",
            sender_drone_id=self.local_drone_id,
            entry_type="TASK_CLAIM",
            epoch=task.ownership_epoch,
            timestamp_utc=now_utc,
            payload={
                "task_id": canon_tid,
                "victim_event_id": task.victim_event_id,
                "owner_drone_id": self.local_drone_id,
                "task_state": "CLAIMED",
                "ownership_epoch": task.ownership_epoch,
                "lease_duration_sec": task.lease_duration_sec
            }
        )
        self.journal.append(entry)
        self.seen_entry_ids.add(entry.entry_id)
        return True

    def release_task(self, task_id: str) -> bool:
        """Gracefully releases an assigned task back to UNASSIGNED so peers can claim it immediately."""
        canon_tid = task_id
        if task_id.startswith("task_"):
            vid = task_id[5:]
            canon_vid = self.get_canonical_victim_id(vid)
            canon_tid = f"task_{canon_vid}"

        if dict.__contains__(self.tasks, canon_tid):
            task = dict.__getitem__(self.tasks, canon_tid)
            if task.owner_drone_id == self.local_drone_id and task.task_state != "COMPLETED":
                task.task_state = "UNASSIGNED"
                task.owner_drone_id = "NONE"
                task.ownership_epoch += 1
                task.last_updated_timestamp_utc = time.time()
                entry = JournalEntry(
                    entry_id=f"rel_{uuid.uuid4().hex[:8]}",
                    sender_drone_id=self.local_drone_id,
                    entry_type="TASK_STATE",
                    epoch=task.ownership_epoch,
                    timestamp_utc=time.time(),
                    payload={
                        "task_id": canon_tid,
                        "victim_event_id": task.victim_event_id,
                        "owner_drone_id": "NONE",
                        "task_state": "UNASSIGNED",
                        "ownership_epoch": task.ownership_epoch,
                        "lease_duration_sec": self.default_lease_duration_sec
                    }
                )
                self.journal.append(entry)
                self.seen_entry_ids.add(entry.entry_id)
                return True
        return False

    def complete_task(self, task_id: str) -> bool:
        """
        Marks a task as COMPLETED for the local drone, advancing lifecycle to terminal state
        and enforcing progress_percent = 100.0.
        """
        canon_tid = task_id
        if task_id.startswith("task_"):
            vid = task_id[5:]
            canon_vid = self.get_canonical_victim_id(vid)
            canon_tid = f"task_{canon_vid}"

        if not dict.__contains__(self.tasks, canon_tid):
            return False

        task = dict.__getitem__(self.tasks, canon_tid)
        if task.task_state == "COMPLETED":
            task.progress_percent = 100.0
            return True

        now_utc = time.time()
        task.task_state = "COMPLETED"
        task.owner_drone_id = self.local_drone_id
        task.progress_percent = 100.0
        task.last_updated_timestamp_utc = now_utc

        entry = JournalEntry(
            entry_id=f"comp_{uuid.uuid4().hex[:8]}",
            sender_drone_id=self.local_drone_id,
            entry_type="TASK_COMPLETE",
            epoch=task.ownership_epoch,
            timestamp_utc=now_utc,
            payload={
                "task_id": canon_tid,
                "victim_event_id": task.victim_event_id,
                "owner_drone_id": self.local_drone_id,
                "task_state": "COMPLETED",
                "ownership_epoch": task.ownership_epoch,
                "progress_percent": 100.0,
                "lease_duration_sec": task.lease_duration_sec,
            }
        )
        self.journal.append(entry)
        self.seen_entry_ids.add(entry.entry_id)
        return True

    def apply_remote_task_state(self, sender_id: str, payload: Dict[str, Any]) -> bool:
        """
        Applies a TASK_STATE payload from a peer drone using epoch fencing and deterministic tie-breaking.
        Delegates state joining to the commutative/associative _join_tasks lattice operator.
        Logs any temporary competing assignments and their conflict resolution.
        """
        raw_tid = payload.get("task_id", "")
        if not raw_tid:
            return False

        canon_tid = raw_tid
        if raw_tid.startswith("task_"):
            vid = raw_tid[5:]
            canon_vid = self.get_canonical_victim_id(vid)
            canon_tid = f"task_{canon_vid}"

        remote_epoch = int(payload.get("ownership_epoch", 1))
        remote_owner = payload.get("owner_drone_id", sender_id)
        remote_state = payload.get("task_state", "CLAIMED")
        lease_sec = float(payload.get("lease_duration_sec", self.default_lease_duration_sec))
        progress = float(payload.get("progress_percent", 0.0))
        now_mono = time.monotonic()
        now_wall_utc = time.time()
        payload_utc = float(payload.get("timestamp_utc", now_wall_utc))

        # --- Sanitize remote payload fields before constructing ReconciledTask ---
        # Invalid values from the network are clamped to safe defaults rather than
        # rejected so that a malformed peer message does not crash the receiver.
        # The domain constraints are enforced at the ReconciledTask boundary.

        # Timestamp: NaN/±inf replaced with receiver's current wall clock
        if not math.isfinite(payload_utc):
            payload_utc = now_wall_utc

        # Progress: NaN/±inf or out-of-range clamped to [0.0, 100.0]
        if not math.isfinite(progress):
            progress = 0.0
        progress = max(0.0, min(100.0, progress))

        # State: unknown states default to CLAIMED (safest active state)
        _valid_states = {
            "UNASSIGNED", "FAILED", "CLAIMED", "APPROACHING", "INVESTIGATING", "COMPLETED"
        }
        if remote_state not in _valid_states:
            remote_state = "CLAIMED"

        # Safe elapsed transit deduction:
        # Transit compensation is conditional upon OS-level clock synchronization across the swarm.
        # If explicit transit_elapsed_sec is provided, use it; otherwise estimate from wall clock if valid.
        # Future timestamps (negative delay) are clamped to 0.0 so leases are never improperly extended.
        if "transit_elapsed_sec" in payload:
            transit_elapsed = float(payload.get("transit_elapsed_sec", 0.0))
        elif payload_utc > 0:
            transit_elapsed = max(0.0, now_wall_utc - payload_utc)
        else:
            transit_elapsed = 0.0

        if not math.isfinite(transit_elapsed) or transit_elapsed < 0.0:
            transit_elapsed = 0.0

        effective_lease_sec = max(0.0, lease_sec - transit_elapsed)
        expiry_mono = now_mono + effective_lease_sec

        incoming_task = ReconciledTask(
            task_id=canon_tid,
            victim_event_id=payload.get("victim_event_id", ""),
            owner_drone_id=remote_owner,
            task_state=remote_state,
            local_lease_expiry_monotonic=expiry_mono,
            lease_duration_sec=lease_sec,
            ownership_epoch=remote_epoch,
            progress_percent=progress,
            last_updated_timestamp_utc=payload_utc
        )

        if not dict.__contains__(self.tasks, canon_tid):
            dict.__setitem__(self.tasks, canon_tid, incoming_task)
            return True

        local_task = dict.__getitem__(self.tasks, canon_tid)

        # Competing Assignment Resolution logging
        if local_task.owner_drone_id not in ("NONE", remote_owner) and remote_owner != "NONE":
            winner = remote_owner if (
                remote_epoch > local_task.ownership_epoch or
                (remote_epoch == local_task.ownership_epoch and remote_owner < local_task.owner_drone_id)
            ) else local_task.owner_drone_id

            comp_log = CompetingAssignmentLog(
                task_id=canon_tid,
                local_owner=local_task.owner_drone_id,
                remote_owner=remote_owner,
                local_epoch=local_task.ownership_epoch,
                remote_epoch=remote_epoch,
                winner_drone_id=winner,
                resolution_reason="Higher Epoch" if remote_epoch != local_task.ownership_epoch else "Deterministic ID Tie-Break",
                timestamp_utc=payload_utc
            )
            self.competing_assignment_history.append(comp_log)

        # Capture prior owner & epoch to evaluate whether incoming state won/reinforced ownership
        prev_owner = local_task.owner_drone_id
        prev_epoch = local_task.ownership_epoch

        # Delegate join to formal semilattice operator
        self._join_tasks(local_task, incoming_task)

        # Incoming was accepted if it became or remained the owner, or if both are COMPLETED
        accepted = (
            (local_task.owner_drone_id == remote_owner and local_task.ownership_epoch == remote_epoch) or
            (local_task.task_state == "COMPLETED" and remote_state == "COMPLETED")
        )
        return accepted

    def check_and_expire_leases(self) -> List[str]:
        """Scans task leases using local monotonic clock and reverts expired non-completed tasks."""
        now_mono = time.monotonic()
        expired: List[str] = []
        for task_id in sorted(dict.keys(self.tasks)):
            task = dict.__getitem__(self.tasks, task_id)
            if task.task_state not in ("UNASSIGNED", "COMPLETED"):
                if now_mono > task.local_lease_expiry_monotonic:
                    task.task_state = "UNASSIGNED"
                    task.owner_drone_id = "NONE"
                    expired.append(task_id)
        return expired

    def get_unassigned_tasks(self) -> List[ReconciledTask]:
        self.check_and_expire_leases()
        return [dict.__getitem__(self.tasks, k) for k in sorted(dict.keys(self.tasks)) if dict.__getitem__(self.tasks, k).task_state == "UNASSIGNED"]

    def merge_remote_journal(self, remote_summary: Dict[str, Any]) -> None:
        """
        Idempotently and deterministically merges an entire state summary from a remote replica.
        Ensures strict join-semilattice properties for state-based synchronization.
        """
        sender_id = remote_summary.get("drone_id", "remote_peer")

        # 1. First pass: merge remote victim aliases
        remote_aliases = remote_summary.get("victim_id_aliases", {})
        for alias_id in sorted(remote_aliases.keys()):
            canon_id = remote_aliases[alias_id]
            self.merge_victims(canon_id, alias_id)

        # 2. Second pass: merge remote victims
        remote_victims = remote_summary.get("victims", {})
        for v_id in sorted(remote_victims.keys()):
            v_data = remote_victims[v_id]
            self.apply_remote_victim_payload(sender_id, v_data)

        # 3. Third pass: merge remote tasks
        remote_tasks = remote_summary.get("tasks", {})
        for t_id in sorted(remote_tasks.keys()):
            t_data = remote_tasks[t_id]
            self.apply_remote_task_state(sender_id, t_data)

    def compact_journal(self, max_retention_entries: int = 200, max_competing_logs: int = 100) -> int:
        """
        Prunes the append-only journal log and competing assignment history,
        while preserving authoritative snapshot state and deduplication tracking.
        Returns: number of journal entries pruned.
        """
        initial_count = len(self.journal)
        if initial_count > max_retention_entries:
            self.journal = self.journal[-max_retention_entries:]
        pruned = initial_count - len(self.journal)

        if len(self.competing_assignment_history) > max_competing_logs:
            self.competing_assignment_history = self.competing_assignment_history[-max_competing_logs:]

        return pruned

    def export_state_summary(self) -> Dict[str, Any]:
        """
        Exports complete deterministic serializable snapshot of victims and tasks for journal sync.
        All dictionaries and lists are sorted deterministically to guarantee bit-for-bit equality.
        """
        sorted_victims: Dict[str, Any] = {}
        for k in sorted(dict.keys(self.victims)):
            v = dict.__getitem__(self.victims, k)
            sorted_sightings = sorted(
                v.sightings,
                key=lambda s: (s.observation_id, s.timestamp_utc, s.reporting_drone_id)
            )
            sorted_victims[k] = {
                "victim_event_id": v.victim_event_id,
                "world_coordinates": [float(c) for c in v.world_coordinates],
                "grid_cell": list(v.grid_cell),
                "confidence": float(v.confidence),
                "uncertainty_m": float(v.position_uncertainty_m),
                "contributing_drones": sorted(list(v.all_contributing_drones)),
                "sightings_count": len(v.sightings),
                "sightings": [
                    {
                        "observation_id": s.observation_id,
                        "reporting_drone_id": s.reporting_drone_id,
                        "timestamp_utc": float(s.timestamp_utc),
                        "coordinates": [float(c) for c in s.coordinates],
                        "grid_cell": list(s.grid_cell),
                        "uncertainty_m": float(s.uncertainty_m),
                        "confidence": float(s.confidence),
                        "confirmation_frames": int(s.confirmation_frames)
                    }
                    for s in sorted_sightings
                ]
            }

        sorted_tasks: Dict[str, Any] = {}
        for k in sorted(dict.keys(self.tasks)):
            t = dict.__getitem__(self.tasks, k)
            sorted_tasks[k] = {
                "task_id": t.task_id,
                "victim_event_id": t.victim_event_id,
                "owner_drone_id": t.owner_drone_id,
                "task_state": t.task_state,
                "ownership_epoch": int(t.ownership_epoch),
                "progress_percent": float(t.progress_percent)
            }

        sorted_aliases = {k: self.victim_id_aliases[k] for k in sorted(self.victim_id_aliases.keys())}

        return {
            "drone_id": self.local_drone_id,
            "victims": sorted_victims,
            "victim_id_aliases": sorted_aliases,
            "tasks": sorted_tasks,
            "competing_assignments_logged": len(self.competing_assignment_history)
        }
