"""
SAR Task Allocator Module
=========================
Modular task-allocation system for multi-drone Search and Rescue (SAR).
Connects autonomously discovered victims (from VictimManager.tracked_victims)
to multi-drone mission coordination.

Key Architectural Guarantees:
1. Strict Separation: Operates exclusively on autonomous TrackedVictim data;
   never accesses or leaks ground-truth simulation evaluation victims.
2. Duplicate Victim Suppression: Re-observations of the same victim update
   coordinates and metadata on the existing task rather than creating duplicate tasks.
3. One-Task / One-Drone: Enforces 1-to-1 assignment mapping. No victim task is
   assigned to multiple drones, and no drone is assigned multiple simultaneous tasks.
4. Robust State Machine: Tracks complete task lifecycle:
   DISCOVERED -> ASSIGNED -> APPROACHING -> INVESTIGATING -> COMPLETED (or FAILED).
5. Safe Reassignment: Automatically detects drone unavailability/failsafe and
   safely returns tasks to the unassigned pool or reassigns them to healthy drones.
6. Simulation Time Consistency: Uses simulation clock timestamps for all lifecycle
   events, timers, and dwells.
7. Telemetry Ready: Serializes clean, structured state dictionaries for dashboard
   and ROS 2 telemetry publishing.
"""

from enum import Enum
import math
import time
from typing import Optional, Dict, List, Tuple, Any

try:
    from .grid_world_transform import GridWorldTransform
except ImportError:
    try:
        from swarm_controller.grid_world_transform import GridWorldTransform
    except ImportError:
        from grid_world_transform import GridWorldTransform


class TaskState(Enum):
    DISCOVERED = "DISCOVERED"
    ASSIGNED = "ASSIGNED"
    APPROACHING = "APPROACHING"
    INVESTIGATING = "INVESTIGATING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"


class SARTask:
    """
    Represents an inspection or rescue task linked to an autonomously discovered victim.
    """
    def __init__(
        self,
        task_id: str,
        victim_id: str,
        world_x: float,
        world_y: float,
        sim_time: float,
        confidence: float = 1.0,
        source: Optional[str] = None,
        verification_mode: Optional[str] = None,
        evidence_state: str = "TEMPORALLY_CONFIRMED",
        approach_radius: float = 3.0,
        investigation_duration: float = 5.0
    ):
        self.task_id = str(task_id)
        self.victim_id = str(victim_id)
        self.world_x = float(world_x)
        self.world_y = float(world_y)
        self.grid_x, self.grid_y = GridWorldTransform.world_to_grid(self.world_x, self.world_y)
        
        self.state: TaskState = TaskState.DISCOVERED
        self.assigned_drone_id: Optional[str] = None
        
        # Timing (Strictly simulation time seconds)
        self.created_sim_time: float = float(sim_time)
        self.updated_sim_time: float = float(sim_time)
        self.assigned_sim_time: Optional[float] = None
        self.approach_start_sim_time: Optional[float] = None
        self.investigation_start_sim_time: Optional[float] = None
        self.completed_sim_time: Optional[float] = None
        self.failed_sim_time: Optional[float] = None
        
        # Parameters
        self.approach_radius: float = float(approach_radius)
        self.investigation_duration: float = float(investigation_duration)
        
        # Metadata
        self.confidence: float = float(confidence)
        self.source: Optional[str] = source
        self.verification_mode: Optional[str] = verification_mode
        self.evidence_state: str = evidence_state
        
        # Reassignment & error tracking
        self.reassignment_count: int = 0
        self.failure_reason: Optional[str] = None
        self.investigation_progress_sec: float = 0.0

    def update_coordinates(
        self,
        world_x: float,
        world_y: float,
        sim_time: float,
        confidence: Optional[float] = None,
        verification_mode: Optional[str] = None,
        evidence_state: Optional[str] = None
    ) -> None:
        """
        Updates task target coordinates when new observations refine the victim location.
        Preserves existing lifecycle state and assigned drone.
        """
        self.world_x = float(world_x)
        self.world_y = float(world_y)
        self.grid_x, self.grid_y = GridWorldTransform.world_to_grid(self.world_x, self.world_y)
        self.updated_sim_time = max(self.updated_sim_time, float(sim_time))
        if confidence is not None:
            self.confidence = max(self.confidence, float(confidence))
        if verification_mode is not None:
            self.verification_mode = str(verification_mode)
        if evidence_state is not None:
            self.evidence_state = str(evidence_state)

    def transition_to(self, new_state: TaskState, sim_time: float, reason: Optional[str] = None) -> bool:
        """
        Executes validated task state transitions and updates associated timestamps.
        """
        if self.state == new_state:
            return True

        st = float(sim_time)

        # Disallow transitions out of terminal COMPLETED state
        if self.state == TaskState.COMPLETED:
            return False

        # Validate allowed transitions
        valid = False
        if self.state == TaskState.DISCOVERED:
            if new_state in (TaskState.ASSIGNED, TaskState.FAILED):
                valid = True
        elif self.state == TaskState.ASSIGNED:
            if new_state in (TaskState.APPROACHING, TaskState.DISCOVERED, TaskState.FAILED):
                valid = True
        elif self.state == TaskState.APPROACHING:
            if new_state in (TaskState.INVESTIGATING, TaskState.DISCOVERED, TaskState.FAILED):
                valid = True
        elif self.state == TaskState.INVESTIGATING:
            if new_state in (TaskState.COMPLETED, TaskState.DISCOVERED, TaskState.FAILED):
                valid = True
        elif self.state == TaskState.FAILED:
            if new_state in (TaskState.DISCOVERED, TaskState.ASSIGNED):
                valid = True

        if not valid:
            return False

        self.state = new_state
        self.updated_sim_time = max(self.updated_sim_time, st)

        if new_state == TaskState.ASSIGNED:
            self.assigned_sim_time = st
            self.failure_reason = None
        elif new_state == TaskState.APPROACHING:
            self.approach_start_sim_time = st
        elif new_state == TaskState.INVESTIGATING:
            self.investigation_start_sim_time = st
        elif new_state == TaskState.COMPLETED:
            self.completed_sim_time = st
            self.investigation_progress_sec = self.investigation_duration
        elif new_state == TaskState.FAILED:
            self.failed_sim_time = st
            self.failure_reason = reason or "TASK_FAILED"
        elif new_state == TaskState.DISCOVERED:
            # Reassignment reset
            self.assigned_drone_id = None
            self.assigned_sim_time = None
            self.approach_start_sim_time = None
            self.investigation_start_sim_time = None
            self.investigation_progress_sec = 0.0

        return True

    def get_dict(self) -> Dict[str, Any]:
        """Serializes task data into a JSON-compatible dictionary."""
        return {
            "task_id": self.task_id,
            "victim_id": self.victim_id,
            "world_x": round(self.world_x, 2),
            "world_y": round(self.world_y, 2),
            "grid_x": int(self.grid_x),
            "grid_y": int(self.grid_y),
            "state": self.state.value,
            "assigned_drone_id": str(self.assigned_drone_id) if self.assigned_drone_id is not None else None,
            "created_sim_time": round(self.created_sim_time, 2),
            "updated_sim_time": round(self.updated_sim_time, 2),
            "assigned_sim_time": round(self.assigned_sim_time, 2) if self.assigned_sim_time is not None else None,
            "investigation_start_sim_time": round(self.investigation_start_sim_time, 2) if self.investigation_start_sim_time is not None else None,
            "completed_sim_time": round(self.completed_sim_time, 2) if self.completed_sim_time is not None else None,
            "failed_sim_time": round(self.failed_sim_time, 2) if self.failed_sim_time is not None else None,
            "failure_reason": self.failure_reason,
            "reassignment_count": self.reassignment_count,
            "confidence": round(self.confidence, 3),
            "source": self.source,
            "verification_mode": self.verification_mode,
            "evidence_state": self.evidence_state,
            "investigation_progress_sec": round(self.investigation_progress_sec, 2),
            "investigation_duration": round(self.investigation_duration, 2)
        }


class SARTaskAllocator:
    """
    Modular task allocator that manages SAR inspection and rescue tasks
    derived from autonomously discovered victims.
    """
    def __init__(
        self,
        node=None,
        approach_radius: float = 3.0,
        investigation_duration: float = 5.0,
        max_reassignments: int = 3
    ):
        self.node = node
        self.approach_radius = float(approach_radius)
        self.investigation_duration = float(investigation_duration)
        self.max_reassignments = int(max_reassignments)

        # Task registry: task_id -> SARTask
        self.tasks: Dict[str, SARTask] = {}
        # Victim ID to task ID index: victim_id -> task_id
        self.victim_to_task: Dict[str, str] = {}
        # Active drone assignments: drone_id -> task_id
        self.drone_assignments: Dict[str, str] = {}

    def _log_info(self, msg: str) -> None:
        if self.node is not None and hasattr(self.node, 'get_logger'):
            self.node.get_logger().info(f"[TaskAllocator] {msg}")

    def _log_warn(self, msg: str) -> None:
        if self.node is not None and hasattr(self.node, 'get_logger'):
            self.node.get_logger().warn(f"[TaskAllocator] {msg}")

    def sync_from_tracked_victims(self, tracked_victims: Dict[str, Any], sim_time: float) -> List[str]:
        """
        Synchronizes task collection with autonomous TrackedVictims.
        
        Guarantees:
        - Only autonomous TrackedVictim instances are ingested.
        - Existing tasks are updated atomically without duplicate creation (Duplicate Suppression).
        - New tasks are initialized in DISCOVERED state.
        
        Returns list of newly created task IDs.
        """
        new_task_ids = []
        st = float(sim_time)

        for v_id, track in tracked_victims.items():
            v_id_str = str(v_id)
            wx = getattr(track, 'world_x', None)
            wy = getattr(track, 'world_y', None)
            if wx is None or wy is None:
                continue

            conf = getattr(track, 'confidence', 1.0)
            src = getattr(track, 'source', None)
            v_mode = getattr(track, 'verification_mode', None)
            ev_state = getattr(track, 'evidence_state', "TEMPORALLY_CONFIRMED")

            if v_id_str in self.victim_to_task:
                # Task already exists -> Duplicate Suppression: update coordinates and metadata
                task_id = self.victim_to_task[v_id_str]
                task = self.tasks.get(task_id)
                if task is not None:
                    task.update_coordinates(
                        world_x=wx,
                        world_y=wy,
                        sim_time=st,
                        confidence=conf,
                        verification_mode=v_mode,
                        evidence_state=ev_state
                    )
            else:
                # Create a new unique task for this newly discovered victim
                task_id = f"task_{v_id_str}"
                task = SARTask(
                    task_id=task_id,
                    victim_id=v_id_str,
                    world_x=wx,
                    world_y=wy,
                    sim_time=st,
                    confidence=conf,
                    source=src,
                    verification_mode=v_mode,
                    evidence_state=ev_state,
                    approach_radius=self.approach_radius,
                    investigation_duration=self.investigation_duration
                )
                self.tasks[task_id] = task
                self.victim_to_task[v_id_str] = task_id
                new_task_ids.append(task_id)
                self._log_info(f"Created task '{task_id}' for victim '{v_id_str}' at ({wx:.1f}, {wy:.1f})")

        return new_task_ids

    def get_assigned_task_for_drone(self, drone_id: str) -> Optional[SARTask]:
        """Returns the task currently assigned to the given drone, if any."""
        drone_id_str = str(drone_id)
        task_id = self.drone_assignments.get(drone_id_str)
        if task_id:
            return self.tasks.get(task_id)
        return None

    def allocate_tasks(
        self,
        available_drones: Dict[str, Tuple[float, float]],
        sim_time: float
    ) -> Dict[str, str]:
        """
        Assigns unassigned tasks to available drones using greedy minimum Euclidean distance.
        
        Parameters:
            available_drones: Dict mapping drone_id -> (world_x, world_y) for drones that
                              are healthy, connected, and have no current assignment.
            sim_time: Current simulation time.
            
        Enforces:
        - 1-task / 1-drone: each drone gets at most one task; each task gets at most one drone.
        - Only tasks in DISCOVERED state (or FAILED with remaining retries) are eligible.
        
        Returns dict of newly assigned {drone_id: task_id}.
        """
        new_assignments: Dict[str, str] = {}
        st = float(sim_time)

        # Filter unassigned tasks eligible for assignment
        unassigned_tasks = [
            t for t in self.tasks.values()
            if t.state == TaskState.DISCOVERED or (
                t.state == TaskState.FAILED and t.reassignment_count < self.max_reassignments
            )
        ]

        if not unassigned_tasks or not available_drones:
            return new_assignments

        # Candidates pool of available drones (not already assigned)
        pool = {
            str(d_id): pos for d_id, pos in available_drones.items()
            if str(d_id) not in self.drone_assignments
        }

        # Prioritize tasks by creation time (FIFO) or confidence
        unassigned_tasks.sort(key=lambda t: (t.created_sim_time, -t.confidence))

        for task in unassigned_tasks:
            if not pool:
                break

            # Find closest available drone
            best_drone_id = None
            best_dist = float('inf')

            for d_id, (dwx, dwy) in pool.items():
                d = math.hypot(dwx - task.world_x, dwy - task.world_y)
                if d < best_dist:
                    best_dist = d
                    best_drone_id = d_id

            if best_drone_id is not None:
                task.assigned_drone_id = best_drone_id
                transition_ok = task.transition_to(TaskState.ASSIGNED, st)
                if transition_ok:
                    self.drone_assignments[best_drone_id] = task.task_id
                    new_assignments[best_drone_id] = task.task_id
                    del pool[best_drone_id]
                    self._log_info(
                        f"Assigned {task.task_id} to {best_drone_id} (distance={best_dist:.1f}m)"
                    )

        return new_assignments

    def update_task_lifecycle(
        self,
        drone_positions: Dict[str, Tuple[float, float]],
        drone_states: Dict[str, str],
        sim_time: float
    ) -> None:
        """
        Steps the task lifecycle state machine for all active tasks.
        
        Handles:
        - Distance checks: approaching -> investigating.
        - Dwell timer: investigating -> completed.
        - Drone unavailability: safe task failure / reassignment back to pool.
        """
        st = float(sim_time)
        active_states = (TaskState.ASSIGNED, TaskState.APPROACHING, TaskState.INVESTIGATING)

        for task in list(self.tasks.values()):
            if task.state not in active_states:
                continue

            drone_id = task.assigned_drone_id
            if drone_id is None:
                # Corrupted state: assigned task has no assigned drone -> return to DISCOVERED
                task.transition_to(TaskState.DISCOVERED, st, reason="MISSING_DRONE_ID")
                continue

            drone_id_str = str(drone_id)

            # Check drone health / state
            d_state = drone_states.get(drone_id_str, "UNKNOWN")
            is_unavailable = d_state in ("FAILSAFE", "DISARM", "COMPLETE", "LAND", "UNKNOWN")

            if is_unavailable:
                # Trigger safe task reassignment
                self._log_warn(
                    f"Drone '{drone_id_str}' unavailable (state='{d_state}') during task '{task.task_id}'. Initiating safe reassignment."
                )
                self.drone_assignments.pop(drone_id_str, None)

                task.reassignment_count += 1
                if task.reassignment_count <= self.max_reassignments:
                    task.transition_to(TaskState.DISCOVERED, st, reason=f"DRONE_UNAVAILABLE_{d_state}")
                    self._log_info(
                        f"Task '{task.task_id}' reset to DISCOVERED (attempt {task.reassignment_count}/{self.max_reassignments})"
                    )
                else:
                    task.transition_to(TaskState.FAILED, st, reason="EXCEEDED_MAX_REASSIGNMENTS")
                    self._log_warn(f"Task '{task.task_id}' FAILED permanently after {task.reassignment_count} attempts.")
                continue

            # Drone is available: retrieve position
            drone_pos = drone_positions.get(drone_id_str)
            if drone_pos is None:
                continue

            dwx, dwy = drone_pos
            dist_to_task = math.hypot(dwx - task.world_x, dwy - task.world_y)

            # State Progression:
            if task.state == TaskState.ASSIGNED:
                # Drone is airborne and moving -> transition to APPROACHING
                task.transition_to(TaskState.APPROACHING, st)

            if task.state == TaskState.APPROACHING:
                if dist_to_task <= task.approach_radius:
                    # Drone arrived within approach radius -> transition to INVESTIGATING
                    task.transition_to(TaskState.INVESTIGATING, st)
                    self._log_info(
                        f"Drone '{drone_id_str}' reached '{task.task_id}' (dist={dist_to_task:.1f}m <= {task.approach_radius}m). Commencing investigation."
                    )

            if task.state == TaskState.INVESTIGATING:
                if task.investigation_start_sim_time is not None:
                    elapsed_investigation = max(0.0, st - task.investigation_start_sim_time)
                    task.investigation_progress_sec = min(task.investigation_duration, elapsed_investigation)

                    if elapsed_investigation >= task.investigation_duration:
                        # Investigation complete -> transition to COMPLETED
                        task.transition_to(TaskState.COMPLETED, st)
                        self.drone_assignments.pop(drone_id_str, None)
                        self._log_info(
                            f"Task '{task.task_id}' COMPLETED by '{drone_id_str}' after {elapsed_investigation:.1f}s investigation."
                        )

    def cancel_or_reassign_drone_task(self, drone_id: str, sim_time: float, reason: str = "MANUAL_CANCEL") -> Optional[str]:
        """
        Safely cancels or unassigns a drone's task, returning the task to the unassigned pool.
        """
        drone_id_str = str(drone_id)
        task_id = self.drone_assignments.pop(drone_id_str, None)
        if not task_id:
            return None

        task = self.tasks.get(task_id)
        if task is not None and task.state in (TaskState.ASSIGNED, TaskState.APPROACHING, TaskState.INVESTIGATING):
            task.reassignment_count += 1
            if task.reassignment_count <= self.max_reassignments:
                task.transition_to(TaskState.DISCOVERED, float(sim_time), reason=reason)
            else:
                task.transition_to(TaskState.FAILED, float(sim_time), reason="MAX_REASSIGNMENTS_EXCEEDED")
            return task_id
        return None

    def get_telemetry(self) -> Dict[str, Any]:
        """Returns structured dictionary for telemetry publishing and dashboard display."""
        task_list = [t.get_dict() for t in self.tasks.values()]
        
        counts = {
            "total": len(self.tasks),
            "discovered": sum(1 for t in self.tasks.values() if t.state == TaskState.DISCOVERED),
            "assigned": sum(1 for t in self.tasks.values() if t.state == TaskState.ASSIGNED),
            "approaching": sum(1 for t in self.tasks.values() if t.state == TaskState.APPROACHING),
            "investigating": sum(1 for t in self.tasks.values() if t.state == TaskState.INVESTIGATING),
            "completed": sum(1 for t in self.tasks.values() if t.state == TaskState.COMPLETED),
            "failed": sum(1 for t in self.tasks.values() if t.state == TaskState.FAILED),
        }

        return {
            "tasks": task_list,
            "assignments": dict(self.drone_assignments),
            "counts": counts
        }
