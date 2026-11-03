export interface DroneTelemetry {
    id: string;
    state: string;
    x: number;
    y: number;
    z: number;
    grid_x: number;
    grid_y: number;
    action: string;
    safety_override: boolean;
    vx?: number;
    vy?: number;
    vz?: number;
    speed?: number;
}

export interface MissionTelemetry {
    status: string;
    decision_count: number;
    max_decisions: number;
    global_step_state?: string;
    sim_time?: number;
    rtf?: number;
    coverage: number;
    victims_detected: number;
    total_victims: number;
    explored_count: number;
    valid_count: number;
    safety_overrides: number;
    camera_enabled?: boolean;
}

export interface SARTaskTelemetry {
    task_id: string;
    victim_id: string;
    world_x: number;
    world_y: number;
    grid_x?: number;
    grid_y?: number;
    state: string; // 'DISCOVERED' | 'ASSIGNED' | 'APPROACHING' | 'INVESTIGATING' | 'COMPLETED' | 'FAILED'
    assigned_drone_id?: string | null;
    confidence?: number;
    verification_mode?: string;
    evidence_state?: string;
    created_sim_time?: number;
    updated_sim_time?: number;
    assigned_sim_time?: number | null;
    investigation_start_sim_time?: number | null;
    completed_sim_time?: number | null;
    failed_sim_time?: number | null;
    failure_reason?: string | null;
    reassignment_count?: number;
    investigation_progress_sec?: number;
    investigation_duration?: number;
}

export interface TaskAllocationSummary {
    total_tasks?: number;
    active_tasks?: number;
    completed_tasks?: number;
    failed_tasks?: number;
    state_counts?: Record<string, number>;
    active_assignments?: Record<string, string>;
    assignments?: Record<string, string>;
    counts?: {
        total?: number;
        discovered?: number;
        assigned?: number;
        approaching?: number;
        investigating?: number;
        completed?: number;
        failed?: number;
    };
    tasks?: SARTaskTelemetry[];
}

export interface VictimState {
    id: string;
    x: number;
    y: number;
    world_x?: number;
    world_y?: number;
    detected: boolean;
    state?: string;
    source?: string;
    confidence?: number;
    detected_by?: string;
    last_seen_sec_ago?: number;
    gt_error?: number;
    observations?: number;
    detection_distance?: number;
    detection_time?: number;
    verification_mode?: string;
    evidence_state?: string;
    confirming_drone?: string;
    confirmation_count?: number;
    sim_timestamp?: number;
    sim_time?: number;
}

export interface ExploredCell {
    x: number;
    y: number;
}

export interface TelemetryPayload {
    type: string;
    timestamp: number;
    mission: MissionTelemetry;
    drones: DroneTelemetry[];
    explored_cells: ExploredCell[];
    victims: VictimState[];
    tracked_victims?: VictimState[];
    tasks?: SARTaskTelemetry[];
    task_allocation?: TaskAllocationSummary;
    evaluation?: EvaluationMetrics;
    active_map_id?: string;
    backend_status?: string;
    camera_enabled?: boolean;
    coordination?: {
        active_frontiers: Record<string, [number, number]>;
        safety_holds: Record<string, string>;
        qmix_drones: number;
        coord_drones: number;
        task_assignments?: Record<string, string>;
    };
}

export interface EvaluationMetrics {
    total_ground_truth_victims?: number;
    total_autonomous_tracks?: number;
    true_positives?: number;
    false_positives?: number;
    false_negatives?: number;
    precision?: number;
    recall?: number;
    f1_score?: number;
    mean_localization_error_m?: number;
    median_localization_error_m?: number;
    max_localization_error_m?: number;
    rmse_localization_error_m?: number;
    matching_threshold_m?: number;
    per_drone_detections?: Record<string, number>;
}

export interface MissionResult {
    drone_count: number;
    mission_duration: number;
    final_coverage: number;
    searched_cells: number;
    total_valid_cells: number;
    victims_detected: number;
    total_victim_observations: number;
    safety_interventions: number;
    timestamp: string;
    status: string;
}

export interface WorldObstacle {
    id: string;
    type: string;
    aabb: {
        min_x: number;
        max_x: number;
        min_y: number;
        max_y: number;
    };
}

export interface WorldVictim {
    id: string;
    grid: {
        x: number;
        y: number;
    };
}

export interface WorldData {
    id?: string;
    name?: string;
    grid: {
        width: number;
        height: number;
        meters_per_cell: number;
    };
    world: {
        width_m: number;
        height_m: number;
        origin_x: number;
        origin_y: number;
        terrain_width_m?: number;
        terrain_height_m?: number;
        terrain_origin_x?: number;
        terrain_origin_y?: number;
    };
    obstacles: WorldObstacle[];
    victims: WorldVictim[];
    drone_spawns: { id: string; x: number; y: number }[];
    sectors?: { id: string; name: string; center: { x: number; y: number }; radius: number }[];
    tree_visuals?: { x: number; y: number; scale: number }[];
}

export interface AlertEvent {
    id: string;
    timestamp: number;
    message: string;
    type: 'info' | 'warning' | 'success';
}

export interface MapRegistryEntry {
    id: string;
    name: string;
    world_file: string;
    metadata_file: string;
    grid_width: number;
    grid_height: number;
    meters_per_cell: number;
    victim_count: number;
    policy_compatible: boolean;
    origin_x?: number;
    origin_y?: number;
    terrain_width_m?: number;
    terrain_height_m?: number;
    terrain_origin_x?: number;
    terrain_origin_y?: number;
}

export type MapRegistry = Record<string, MapRegistryEntry>;

export interface BackendMissionStatus {
    state: 'IDLE' | 'STARTING' | 'SIMULATOR_READY' | 'QMIX_STARTING' | 'RUNNING' | 'COMPLETE' | 'STOPPING' | 'STOPPED' | 'ERROR';
    active_map_id: string | null;
    error: string | null;
}
